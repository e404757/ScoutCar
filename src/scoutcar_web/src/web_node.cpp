// web_node —— 网页推流 + 录像回放/导出节点
//
// 对应原工程：main/web_server.cc + recorder/record_control/video_writer，
// 按 ROS2 架构重接：
//   数据面（订阅，全部由其他节点发布，本节点零改动它们）：
//     /camera/*/image_raw        原始帧（rgb8，best_effort）—— 非感知相机推流帧源
//     /perception/debug_image    感知同帧合成的叠加画面 —— 感知相机推流帧源
//     /perception/source_image   感知输入原图 —— 叠加录像的原始帧源
//     /perception/road_boundary  道路边界数值 —— 网页状态面板
//     /mission/status            任务状态 —— 网页状态面板
//   功能面：
//     ① 实时网页推流：网页选中哪路相机就推哪路。感知相机显示的是感知节点
//        同帧合成的叠加画面（掩膜/边界与图像零时间差），其它相机显示原始画面。
//        任务状态、偏差数值等不再绘制在画面上，由 /api/status 供网页面板展示。
//     ② 干净录像（用户需求：采集 YOLO 训练数据）：
//        网页一个按钮起停，每段同步录双文件（原始 .avi + 叠加 _mask.avi，帧号对齐）。
//     ③ 录像回放：网页完整播放器（播放/暂停/进度条/逐帧）。
//     ④ 导出补标：暂停在某帧 → 从原始文件抽该帧存 JPEG 到 label_dir，供 labelImg 补标。
//
// 线程模型（与原工程一致的分层）：
//   - ROS executor 线程：图像回调 = "主循环"（双路写帧 + 推流帧编码）；
//   - httplib 独立线程：HTTP API / 回放解码 / 导出（OpenCV VideoCapture 每请求使用）；
//   - 每个录像文件一个写线程（AsyncAviWriter，环形缓冲，编码跟不上丢帧不阻塞）。
//   - 网页按钮请求经原子槽 → 10Hz 定时器消费 → 开始/封存片段。
//
// 实时性约定：图像类订阅全部 best_effort + depth=1（只留最新帧）。
// 推流编码慢于相机帧率时丢旧帧而不是排队，保证网页看到的永远是"现在"。

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <scoutcar_msgs/msg/mission_status.hpp>
#include <scoutcar_msgs/msg/road_boundary.hpp>

#include <opencv2/opencv.hpp>

#include "httplib.h"
#include "bag_recorder.h"
#include "dual_recorder.h"

#include "page_html.h"

namespace fs = std::filesystem;

using sensor_msgs::msg::Image;
using scoutcar_msgs::msg::RoadBoundary;
using scoutcar_msgs::msg::MissionStatus;

class WebNode : public rclcpp::Node
{
public:
  WebNode() : Node("web_node")
  {
    // ── 参数 ──
    port_ = declare_parameter<int>("port", 8080);
    camera_topics_ = declare_parameter<std::vector<std::string>>(
      "camera_topics", {"camera/usb/image_raw", "camera/mipi/image_raw"});
    // 叠加画面（掩膜/边界）只来自"感知实际处理的那路相机"的 debug_image，
    // 避免把 USB 的掩膜错套到 MIPI。网页切到其它相机时该路只显示原始画面。
    overlay_topic_ = declare_parameter<std::string>(
      "overlay_topic", "/camera/usb/image_raw");
    source_topic_ = declare_parameter<std::string>(
      "source_topic", "/perception/source_image");
    debug_topic_ = declare_parameter<std::string>(
      "debug_topic", "/perception/debug_image");
    boundary_topic_ = declare_parameter<std::string>(
      "boundary_topic", "/perception/road_boundary");
    status_topic_ = declare_parameter<std::string>("status_topic", "/mission/status");
    deviation_enable_topic_ = declare_parameter<std::string>(
      "deviation_enable_topic", "/mission/deviation_enable");
    record_dir_ = declare_parameter<std::string>(
      "record_dir", "/home/orangepi/CityScout/data/record");
    // 0 表示开始录像时采用最近测得的实际感知帧率。
    record_fps_ = declare_parameter<int>("record_fps", 0);
    record_enable_ = declare_parameter<bool>("record_enable", true);
    bag_enable_ = declare_parameter<bool>("bag_enable", true);
    bag_dir_ = declare_parameter<std::string>(
      "bag_dir", "/home/orangepi/CityScout/data/bags");
    bag_topics_ = declare_parameter<std::vector<std::string>>(
      "bag_topics", {
        "/perception/seg_mask",
        "/perception/road_boundary",
        "/mission/deviation_enable",
        "/mission/status",
        "/mission/path_cmd",
        "/mcu/rx_event"
      });
    jpg_quality_ = declare_parameter<int>("jpg_quality", 85);

    if (camera_topics_.empty()) {
      camera_topics_ = {overlay_topic_};
    }
    // 选中相机默认指向叠加相机（感知画面）
    selected_idx_ = 0;
    for (size_t i = 0; i < camera_topics_.size(); ++i) {
      if (normalizeTopic(camera_topics_[i]) == overlay_topic_) {
        selected_idx_.store(static_cast<int>(i));
        break;
      }
    }

    // ── 订阅：每一路相机话题独立订阅；只有"选中"那路参与推流编码 ──
    for (const auto & topic : camera_topics_) {
      const std::string full = normalizeTopic(topic);
      camera_subs_.push_back(create_subscription<Image>(
        // best_effort + depth=1：编码期间来新帧直接替换旧帧，推流永远用最新画面。
        full, rclcpp::QoS(1).best_effort(),
        [this, full](const Image::SharedPtr msg) { onCamera(full, msg); }));
      RCLCPP_INFO(get_logger(), "订阅相机话题 %s", full.c_str());
    }
    // 感知同帧合成的叠加画面：感知相机的推流源（掩膜/边界与图像零时间差）。
    // best_effort + depth=1：编码慢于发布时只留最新帧，网页永远显示"现在"。
    sub_debug_ = create_subscription<Image>(
      debug_topic_, rclcpp::QoS(1).best_effort(),
      [this](const Image::SharedPtr msg) { onDebug(msg); });
    // 感知输入原图：叠加录像的"原始帧"来源（与 debug_image 同帧配对）。
    sub_source_ = create_subscription<Image>(
      source_topic_, rclcpp::QoS(5).best_effort(),
      [this](const Image::SharedPtr msg) { onSource(msg); });
    // 数值面板数据
    sub_boundary_ = create_subscription<RoadBoundary>(
      boundary_topic_, 10,
      [this](const RoadBoundary::SharedPtr msg) { onBoundary(msg); });
    sub_status_ = create_subscription<MissionStatus>(
      status_topic_, 10,
      [this](const MissionStatus::SharedPtr msg) { onStatus(msg); });
    sub_deviation_enable_ = create_subscription<std_msgs::msg::Bool>(
      deviation_enable_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        deviation_enabled_.store(msg->data);
      });

    // 录像按钮请求消费（与原工程 record_control 的主循环消费对应）
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { pollRecordRequests(); });

    recorder_.configure(record_dir_, record_fps_);
    bag_recorder_.configure(bag_dir_, bag_topics_);

    startHttpServer(port_);
    RCLCPP_INFO(get_logger(), "web_node 就绪：http://<本机IP>:%d（录像目录 %s）",
                port_, record_dir_.c_str());
    RCLCPP_INFO(get_logger(), "叠加画面来源: %s", debug_topic_.c_str());
  }

  ~WebNode() override
  {
    stopHttpServer();
    bag_recorder_.stop();
    recorder_.stop();
  }

private:
  // ═══════════════ 话题回调 ═══════════════

  static std::string normalizeTopic(const std::string & topic)
  {
    return (topic.empty() || topic[0] == '/') ? topic : ("/" + topic);
  }

  std::string currentTopic() const
  {
    if (camera_topics_.empty()) {
      return std::string();
    }
    int idx = selected_idx_.load();
    if (idx < 0 || idx >= static_cast<int>(camera_topics_.size())) { idx = 0; }
    return normalizeTopic(camera_topics_[idx]);
  }

  // 非感知相机：直接编码原始帧推流（无叠加内容）
  void onCamera(const std::string & topic, const Image::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(cam_mutex_);
      latest_cam_[topic] = msg;          // 记录最近一帧（共享指针，只加引用计数）
      on_frame_time_[topic] = std::chrono::steady_clock::now();
    }
    if (topic == currentTopic() && topic != overlay_topic_) {
      updateFps();
      const int width = static_cast<int>(msg->width);
      const int height = static_cast<int>(msg->height);
      if (width <= 0 || height <= 0 || msg->encoding != "rgb8" ||
          msg->data.size() < static_cast<size_t>(width) * height * 3)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "图像格式异常: enc=%s %ux%u",
                             msg->encoding.c_str(), msg->width, msg->height);
        return;
      }
      // rgb8 → BGR 后 JPEG 编码（推流色彩正确）
      cv::Mat rgb_view(height, width, CV_8UC3,
                       const_cast<uint8_t *>(msg->data.data()));
      cv::Mat bgr;
      cv::cvtColor(rgb_view, bgr, cv::COLOR_RGB2BGR);
      encodeLive(bgr);
    }
  }

  // 感知相机：直接显示感知同帧合成好的叠加画面（掩膜/边界与图像零时间差）
  void onDebug(const Image::SharedPtr msg)
  {
    have_debug_stream_.store(true);
    updateRecordInputFps();
    const int width = static_cast<int>(msg->width), height = static_cast<int>(msg->height);
    if (width <= 0 || height <= 0 || msg->encoding != "rgb8" ||
        msg->data.size() < static_cast<size_t>(width) * height * 3)
    {
      return;
    }
    // 叠加录像：原始帧按时间戳从 source 队列取回（同帧配对），帧号天然对齐
    if (recorder_.active()) {
      Image::SharedPtr source = matchSource(msg->header.stamp);
      if (source != nullptr && source->data.size() >=
          static_cast<size_t>(width) * height * 3)
      {
        recorder_.write_frame(source->data.data(), msg->data.data(), width, height);
      }
    }
    if (currentTopic() != overlay_topic_) { return; }
    updateFps();
    cv::Mat rgb(height, width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()));
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    encodeLive(bgr);
  }

  // 推流帧编码（JPEG，主线程编码后加锁换入缓冲，帧序号供推流线程检测更新）
  void encodeLive(const cv::Mat & bgr)
  {
    std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, jpg_quality_};
    std::vector<uint8_t> encoded;
    if (cv::imencode(".jpg", bgr, encoded, params)) {
      std::lock_guard<std::mutex> lock(live_mutex_);
      live_jpeg_ = std::move(encoded);
      frame_seq_.fetch_add(1);
    }
  }

  void onSource(const Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(source_mutex_);
    source_queue_.emplace_back(msg->header.stamp, msg);
    while (source_queue_.size() > 20) { source_queue_.pop_front(); }
  }

  Image::SharedPtr matchSource(const builtin_interfaces::msg::Time & stamp)
  {
    std::lock_guard<std::mutex> lock(source_mutex_);
    for (auto it = source_queue_.rbegin(); it != source_queue_.rend(); ++it) {
      if (rclcpp::Time(it->first) == rclcpp::Time(stamp)) { return it->second; }
    }
    return nullptr;
  }

  void onBoundary(const RoadBoundary::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(boundary_mutex_);
    latest_boundary_ = *msg;
    boundary_time_ = std::chrono::steady_clock::now();
  }

  void onStatus(const MissionStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    latest_status_ = *msg;
    status_time_ = std::chrono::steady_clock::now();
  }

  // ═══════════════ 录像控制 ═══════════════

  void pollRecordRequests()
  {
    const int req = record_request_.exchange(0);
    if (req == 1) {
      if (!record_enable_) {
        RCLCPP_WARN(get_logger(), "收到录像请求但 record_enable=false");
        return;
      }
      const int measured_fps = static_cast<int>(std::lround(record_input_fps_.load()));
      const int output_fps = record_fps_ > 0 ? record_fps_ : std::clamp(measured_fps, 1, 60);
      const int safe_fps = output_fps > 0 ? output_fps : 20;
      if (recorder_.start(safe_fps) && bag_enable_) {
        if (!bag_recorder_.start(recorder_.current_base())) {
          RCLCPP_ERROR(get_logger(), "视频已开始，但 rosbag 启动失败");
        }
      }
    } else if (req == -1) {
      bag_recorder_.stop();
      recorder_.stop();
    }
  }

  // ═══════════════ FPS 统计 ═══════════════

  void updateFps()
  {
    fps_count_++;
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(now - fps_last_).count();
    if (elapsed >= 2.0) {
      current_fps_.store(fps_count_ / elapsed);
      fps_count_ = 0;
      fps_last_ = now;
    }
  }

  void updateRecordInputFps()
  {
    record_fps_count_++;
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - record_fps_last_).count();
    if (elapsed >= 2.0) {
      record_input_fps_.store(record_fps_count_ / elapsed);
      record_fps_count_ = 0;
      record_fps_last_ = now;
    }
  }

  // ═══════════════ HTTP 服务器 ═══════════════

  void startHttpServer(int port)
  {
    svr_ = new httplib::Server();
    setupRoutes();
    running_.store(true);
    server_thread_ = new std::thread([this, port]() {
      RCLCPP_INFO(get_logger(), "HTTP 服务器启动，端口 %d", port);
      svr_->listen("0.0.0.0", port);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void stopHttpServer()
  {
    running_.store(false);
    if (svr_ != nullptr) {
      svr_->stop();
      delete svr_;
      svr_ = nullptr;
    }
    if (server_thread_ != nullptr && server_thread_->joinable()) {
      server_thread_->join();
      delete server_thread_;
      server_thread_ = nullptr;
    }
  }

  void setupRoutes()
  {
    // 页面
    svr_->Get("/", [](const httplib::Request &, httplib::Response & res) {
      res.set_content(kPageHtml, "text/html; charset=utf-8");
    });

    // 实时 MJPEG 推流（?cam=话题 → 推那一路；缺省推感知叠加画面）
    svr_->Get("/video_feed", [this](const httplib::Request & req,
                                    httplib::Response & res) {
      const std::string topic = normalizeTopic(req.get_param_value("cam"));
      const std::string wanted =
        topic.empty() || topic == "/" ? overlay_topic_ : topic;
      bool known = false;
      for (const auto & t : camera_topics_) {
        if (normalizeTopic(t) == wanted) { known = true; break; }
      }
      if (!known) {
        res.status = 404;
        res.set_content("unknown camera", "text/plain");
        return;
      }
      res.set_content_provider(
        static_cast<size_t>(-1),
        "multipart/x-mixed-replace; boundary=frame",
        [this](size_t, size_t, httplib::DataSink & sink) -> bool {
          uint64_t last_seq = 0;
          bool have_sent = false;
          while (running_.load()) {
            std::vector<uint8_t> frame;
            uint64_t seq;
            {
              std::lock_guard<std::mutex> lock(live_mutex_);
              frame = live_jpeg_;
              seq = frame_seq_.load();
            }
            // 帧未更新时不重发同一张 JPEG：降低带宽占用，新帧一到立即推送
            if (!frame.empty() && (!have_sent || seq != last_seq)) {
              std::string part = "--frame\r\n";
              part += "Content-Type: image/jpeg\r\n";
              part += "Content-Length: " + std::to_string(frame.size()) + "\r\n\r\n";
              part += std::string(reinterpret_cast<char *>(frame.data()), frame.size());
              part += "\r\n";
              if (!sink.write(part.data(), part.size())) {
                return false;
              }
              have_sent = true;
            }
            last_seq = seq;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 新帧到达后 ≤5ms 推出
          }
          sink.done();
          return false;
        },
        [this](bool) {
          RCLCPP_DEBUG(get_logger(), "视频客户端断开");
        });
    });

    // 状态（页面轮询：录像状态 + 相机信号 + 任务状态 + 道路边界数值）
    svr_->Get("/api/status", [this](const httplib::Request &, httplib::Response & res) {
      res.set_content(statusJson(), "application/json");
    });

    // 相机列表 + 当前选中
    svr_->Get("/api/cameras", [this](const httplib::Request &, httplib::Response & res) {
      char buf[256];
      snprintf(buf, sizeof(buf), "{\"ok\":true,\"selected\":\"%s\",\"cameras\":%s}",
               currentTopic().c_str(), camerasJson().c_str());
      res.set_content(buf, "application/json");
    });

    // 切换推流相机（网页端下拉/按钮）：?topic=/camera/mipi/image_raw
    svr_->Post("/api/camera/set", [this](const httplib::Request & req, httplib::Response & res) {
      std::string t = normalizeTopic(req.get_param_value("topic"));
      bool found = false;
      for (size_t i = 0; i < camera_topics_.size(); ++i) {
        if (normalizeTopic(camera_topics_[i]) == t) {
          selected_idx_.store(static_cast<int>(i)); found = true; break;
        }
      }
      if (!found) {
        res.set_content("{\"ok\":false,\"reason\":\"unknown topic\"}", "application/json");
        return;
      }
      // 切换后清空推流缓冲，避免新相机第一帧前残留旧相机画面
      {
        std::lock_guard<std::mutex> lock(live_mutex_);
        live_jpeg_.clear();
      }
      RCLCPP_INFO(get_logger(), "切换推流相机 → %s", t.c_str());
      res.set_content(statusJson(), "application/json");
    });

    // 录像起停（网页按钮）
    svr_->Post("/api/record/start", [this](const httplib::Request &, httplib::Response & res) {
      if (!record_enable_) {
        res.set_content("{\"ok\":false,\"reason\":\"record_enable=false\"}",
                        "application/json");
        return;
      }
      record_request_.store(1);
      res.set_content(statusJson(), "application/json");
    });
    svr_->Post("/api/record/stop", [this](const httplib::Request &, httplib::Response & res) {
      record_request_.store(-1);
      res.set_content(statusJson(), "application/json");
    });

  }

  // ═══════════════ JSON ═══════════════

  std::string statusJson()
  {
    const auto now = std::chrono::steady_clock::now();
    auto ageMs = [&now](const std::chrono::steady_clock::time_point & t) {
      return t.time_since_epoch().count() == 0 ? -1 :
             static_cast<long long>(
               std::chrono::duration_cast<std::chrono::milliseconds>(now - t).count());
    };

    // 任务状态数值面板（对应原 FF 01 调试帧的信息）
    std::string mission_json;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      const long long ms = ageMs(status_time_);
      const bool online = ms >= 0 && ms <= 1500;
      const int state = latest_status_ ? latest_status_->state : -1;
      char buf[256];
      snprintf(buf, sizeof(buf),
               "{\"online\":%s,\"driving_state\":\"%s\",\"state\":%d,\"current_node\":%d,\"seg_index\":%d,"
               "\"fixed_remain\":%d,\"random_remain\":%d}",
               online ? "true" : "false",
               deviation_enabled_.load() ? "NORMAL" : "TURNING", state,
               latest_status_ ? latest_status_->current_node : -1,
               latest_status_ ? latest_status_->seg_index : -1,
               latest_status_ ? latest_status_->fixed_remain : -1,
               latest_status_ ? latest_status_->random_remain : -1);
      mission_json = buf;
    }

    // 道路边界数值面板
    std::string boundary_json;
    {
      std::lock_guard<std::mutex> lock(boundary_mutex_);
      const long long ms = ageMs(boundary_time_);
      const bool online = ms >= 0 && ms <= 1500;
      char buf[1024];
      snprintf(buf, sizeof(buf),
               "{\"online\":%s,\"valid\":%s,\"deviation\":%d,\"preview_mode\":\"%s\","
               "\"selected_pt\":%.3f,\"scan_y\":%u,\"road_center\":%u,"
               "\"left\":%d,\"right\":%d,\"width\":%u,"
               "\"min_width\":%u,\"max_width\":%u,"
               "\"width_status\":\"%s\","
               "\"boundary_source\":\"%s\",\"algorithm_valid\":%s,"
               "\"algorithm_deviation\":%d,\"control_ready\":%s}",
               online ? "true" : "false",
               latest_boundary_ ? (latest_boundary_->valid ? "true" : "false") : "false",
               latest_boundary_ ? latest_boundary_->deviation : 0,
               latest_boundary_ ? latest_boundary_->preview_mode.c_str() : "",
               latest_boundary_ ? latest_boundary_->selected_pt : 0.0f,
               latest_boundary_ ? latest_boundary_->scan_y : 0,
               latest_boundary_ ? latest_boundary_->road_center : 0,
               latest_boundary_ ? latest_boundary_->left : -1,
               latest_boundary_ ? latest_boundary_->right : -1,
               latest_boundary_ ? latest_boundary_->width : 0,
               latest_boundary_ ? latest_boundary_->min_width : 0,
               latest_boundary_ ? latest_boundary_->max_width : 0,
               latest_boundary_ ? latest_boundary_->width_status.c_str() : "",
               latest_boundary_ ? latest_boundary_->boundary_source.c_str() : "",
               latest_boundary_ ? (latest_boundary_->algorithm_valid ? "true" : "false") : "false",
               latest_boundary_ ? latest_boundary_->algorithm_deviation : -999,
               latest_boundary_ ? (latest_boundary_->control_ready ? "true" : "false") : "false");
      boundary_json = buf;
    }

    char buf[1024];
    snprintf(buf, sizeof(buf),
             "{\"ok\":true,\"record_enable\":%s,\"recording\":%s,"
             "\"bag_enable\":%s,\"bag_recording\":%s,"
             "\"record_dir\":\"%s\",\"fps\":%.1f,\"cameras\":%s,"
             "\"mission\":%s,\"boundary\":%s}",
             record_enable_ ? "true" : "false",
             recorder_.active() ? "true" : "false",
             bag_enable_ ? "true" : "false",
             bag_recorder_.active() ? "true" : "false",
             record_dir_.c_str(),
             current_fps_.load(), camerasJson().c_str(),
             mission_json.c_str(), boundary_json.c_str());
    return std::string(buf);
  }

  // 相机列表 + 当前选中 + 是否有信号（最近 1.5s 内收过帧）
  std::string camerasJson()
  {
    std::string sel = currentTopic();
    std::string json = "[";
    auto now = std::chrono::steady_clock::now();
    for (size_t i = 0; i < camera_topics_.size(); ++i) {
      const std::string t = normalizeTopic(camera_topics_[i]);
      if (i > 0) { json += ","; }
      bool has = false;
      {
        std::lock_guard<std::mutex> lock(cam_mutex_);
        auto it = latest_cam_.find(t);
        if (it != latest_cam_.end() && it->second != nullptr) {
          const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - on_frame_time_[t]).count();
          has = (age >= 0 && age <= 1500) && it->second->header.stamp.sec > 0;
        }
      }
      char buf[256];
      snprintf(buf, sizeof(buf),
               "{\"topic\":\"%s\",\"selected\":%s,\"overlay\":%s,\"has_frame\":%s}",
               t.c_str(), (t == sel) ? "true" : "false",
               (t == overlay_topic_) ? "true" : "false",
               has ? "true" : "false");
      json += buf;
    }
    json += "]";
    return json;
  }

  // ═══════════════ 成员 ═══════════════

  // 参数
  int port_ = 8080;
  std::vector<std::string> camera_topics_;
  std::string overlay_topic_;
  std::string source_topic_;
  std::string debug_topic_;
  std::string boundary_topic_;
  std::string status_topic_;
  std::string deviation_enable_topic_;
  std::string record_dir_;
  int record_fps_ = 0;
  bool record_enable_ = true;
  bool bag_enable_ = true;
  std::string bag_dir_;
  std::vector<std::string> bag_topics_;
  int jpg_quality_ = 85;

  // 话题
  std::vector<rclcpp::Subscription<Image>::SharedPtr> camera_subs_;
  std::atomic<int> selected_idx_{0};
  std::mutex cam_mutex_;
  std::map<std::string, Image::SharedPtr> latest_cam_;
  std::map<std::string, std::chrono::steady_clock::time_point> on_frame_time_;
  rclcpp::Subscription<Image>::SharedPtr sub_source_;
  rclcpp::Subscription<Image>::SharedPtr sub_debug_;
  rclcpp::Subscription<RoadBoundary>::SharedPtr sub_boundary_;
  rclcpp::Subscription<MissionStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_deviation_enable_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 叠加录像的原始帧配对队列（source_image 按时间戳精确匹配 debug_image）
  std::mutex source_mutex_;
  std::deque<std::pair<builtin_interfaces::msg::Time, Image::SharedPtr>> source_queue_;

  // 数值面板数据（图像上不绘制文字，全部走这里）
  std::mutex boundary_mutex_;
  std::optional<RoadBoundary> latest_boundary_;
  std::chrono::steady_clock::time_point boundary_time_{};
  std::atomic<bool> have_debug_stream_{false};
  std::mutex status_mutex_;
  std::optional<MissionStatus> latest_status_;
  std::chrono::steady_clock::time_point status_time_{};
  // Web 未收到转向指令时默认显示正常；任务离线不再产生第三种状态。
  std::atomic<bool> deviation_enabled_{true};

  // 推流
  std::mutex live_mutex_;
  std::vector<uint8_t> live_jpeg_;
  std::atomic<uint64_t> frame_seq_{0};   // 每编码一帧 +1，推流线程据此跳过未更新帧
  std::atomic<int> fps_count_{0};
  std::chrono::steady_clock::time_point fps_last_{std::chrono::steady_clock::now()};
  std::atomic<double> current_fps_{0.0};
  int record_fps_count_ = 0;
  std::chrono::steady_clock::time_point record_fps_last_{std::chrono::steady_clock::now()};
  std::atomic<double> record_input_fps_{20.0};

  // 录像
  scoutcar_web::DualRecorder recorder_;
  scoutcar_web::BagRecorder bag_recorder_;
  std::atomic<int> record_request_{0};   // 0=无请求 1=开始 -1=停止

  // HTTP
  httplib::Server * svr_ = nullptr;
  std::thread * server_thread_ = nullptr;
  std::atomic<bool> running_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebNode>());
  rclcpp::shutdown();
  return 0;
}
