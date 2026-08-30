// web_node —— 网页推流 + 录像回放/导出节点
//
// 对应原工程：main/web_server.cc + recorder/record_control/video_writer，
// 按 ROS2 架构重接：
//   数据面（订阅，全部由其他节点发布，本节点零改动它们）：
//     /camera/*/image_raw          原始帧（rgb8，best_effort）—— 叠加与原始录像的帧源
//     /perception/seg_mask         分割掩膜（mono8：0背景/1路面/2挡板）—— 叠加内容
//     /perception/road_boundary    道路边界（扫描线/偏差）—— 叠加内容
//     /mission/status              任务状态 —— 叠加文字
//   功能面：
//     ① 实时网页推流：叠加画面 MJPEG（8080）；
//     ② 干净录像（用户需求：采集 YOLO 训练数据）：
//        网页一个按钮起停，每段同步录双文件（原始 .avi + 叠加 _mask.avi，帧号对齐）；
//     ③ 录像回放：网页完整播放器（播放/暂停/进度条/逐帧），看叠加画面找"模型没
//        分割好"的弱帧；
//     ④ 导出补标：暂停在某帧 → 从原始文件抽该帧存 JPEG 到 label_dir，供 labelImg 补标。
//
// 线程模型（与原工程一致的分层）：
//   - ROS executor 线程：图像回调 = "主循环"（叠加 + 双路写帧 + 推流帧编码）；
//   - httplib 独立线程：HTTP API / 回放解码 / 导出（OpenCV VideoCapture 每请求使用）；
//   - 每个录像文件一个写线程（AsyncAviWriter，环形缓冲，编码跟不上丢帧不阻塞）。
//   - 网页按钮请求经原子槽 → 10Hz 定时器消费 → 开始/封存片段。

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <scoutcar_msgs/msg/mission_status.hpp>
#include <scoutcar_msgs/msg/road_boundary.hpp>

#include <opencv2/opencv.hpp>

#include "httplib.h"
#include "dual_recorder.h"
#include "overlay.h"

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
    // 叠加画面（掩膜/边界）只叠加在"感知实际处理的那路相机"上，避免把 USB 的掩膜错套到 MIPI。
    // 默认 = 感知的 image_topic（USB）。网页切到其它相机时该路只显示原始画面。
    overlay_topic_ = declare_parameter<std::string>(
      "overlay_topic", "/camera/usb/image_raw");
    seg_topic_ = declare_parameter<std::string>("seg_topic", "/perception/seg_mask");
    boundary_topic_ = declare_parameter<std::string>(
      "boundary_topic", "/perception/road_boundary");
    source_topic_ = declare_parameter<std::string>(
      "source_topic", "/perception/source_image");
    debug_topic_ = declare_parameter<std::string>(
      "debug_topic", "/perception/debug_image");
    use_local_debug_ = declare_parameter<bool>("use_local_debug", true);
    status_topic_ = declare_parameter<std::string>("status_topic", "/mission/status");
    record_dir_ = declare_parameter<std::string>(
      "record_dir", "/home/orangepi/CityScout/data/record");
    label_dir_ = declare_parameter<std::string>(
      "label_dir", "/home/orangepi/CityScout/data/label_export");
    record_fps_ = declare_parameter<int>("record_fps", 30);
    record_enable_ = declare_parameter<bool>("record_enable", true);
    jpg_quality_ = declare_parameter<int>("jpg_quality", 85);

    if (camera_topics_.empty()) {
      camera_topics_ = {"camera/usb/image_raw"};
    }
    selected_idx_ = 0;

    // ── 订阅：每一路相机话题独立订阅（best_effort 只留最新），只有"选中"那路参与叠加/推流 ──
    for (const auto & topic : camera_topics_) {
      // 兼容不带前导斜杠的写法：统一补全 / 前缀，与话题名保持一致
      const std::string full = (topic.empty() || topic[0] == '/') ? topic : ("/" + topic);
      camera_subs_.push_back(create_subscription<Image>(
        // Web 还要做 JPEG 编码，depth=1 会在编码期间覆盖掉感知实际使用的那帧。
        // 回调本身只存 shared_ptr，适当排队不会阻塞相机。
        full, rclcpp::QoS(30).best_effort(),
        [this, full](const Image::SharedPtr msg) { onCamera(full, msg); }));
      RCLCPP_INFO(get_logger(), "订阅相机话题 %s", full.c_str());
    }
    // 默认直接使用感知节点在同一回调内合成的 debug_image。只有显式关闭时，
    // 才启用旧的 Mask/RGB 时间戳配对回退，避免启动阶段错帧和无意义警告。
    if (!use_local_debug_) {
      sub_seg_ = create_subscription<Image>(
        seg_topic_, 10,
        [this](const Image::SharedPtr msg) { onSeg(msg); });
      sub_boundary_ = create_subscription<RoadBoundary>(
        boundary_topic_, 10,
        [this](const RoadBoundary::SharedPtr msg) { onBoundary(msg); });
    }
    sub_source_ = create_subscription<Image>(
      source_topic_, 10, [this](const Image::SharedPtr msg) { onSource(msg); });
    sub_debug_ = create_subscription<Image>(
      debug_topic_, 10, [this](const Image::SharedPtr msg) { onDebug(msg); });
    sub_status_ = create_subscription<MissionStatus>(
      status_topic_, 10,
      [this](const MissionStatus::SharedPtr msg) { onStatus(msg); });

    // 录像按钮请求消费（与原工程 record_control 的主循环消费对应）
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { pollRecordRequests(); });

    recorder_.configure(record_dir_, record_fps_);

    startHttpServer(port_);
    RCLCPP_INFO(get_logger(),
                "web_node 就绪：http://<本机IP>:%d（录像目录 %s，补标导出 %s）",
                port_, record_dir_.c_str(), label_dir_.c_str());
    RCLCPP_INFO(get_logger(), "叠加画面来源: %s",
                use_local_debug_ ? debug_topic_.c_str() : "Web 端 Mask/RGB 配对");
  }

  ~WebNode() override
  {
    stopHttpServer();
    recorder_.stop();
  }

private:
  // ═══════════════ 话题回调 ═══════════════

  // 感知相机只缓存：等 Mask 到达后用原相机时间戳取回同一帧。
  // 非感知相机没有 Mask，仍然在相机回调中直接推流。
  void onCamera(const std::string & topic, const Image::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(cam_mutex_);
      latest_cam_[topic] = msg;          // 记录最近一帧（共享指针，只加引用计数）
      on_frame_time_[topic] = std::chrono::steady_clock::now();
      if (topic == overlay_topic_) {
        overlay_cam_queue_.emplace_back(msg->header.stamp, msg);
        while (overlay_cam_queue_.size() > kSyncQueueMax) {
          overlay_cam_queue_.pop_front();
        }
      }
    }
    const std::string selected = currentTopic();
    if (topic == selected && topic != overlay_topic_) {
      processFrame(msg, nullptr, std::nullopt, false);
    }
  }

  std::string currentTopic() const
  {
    if (camera_topics_.empty()) {
      return std::string();
    }
    int idx = selected_idx_.load();
    if (idx < 0 || idx >= static_cast<int>(camera_topics_.size())) { idx = 0; }
    std::string t = camera_topics_[idx];
    return (t.empty() || t[0] == '/') ? t : ("/" + t);
  }

  void processFrame(const Image::SharedPtr msg, const Image::SharedPtr seg,
                    const std::optional<RoadBoundary> & matched_boundary,
                    bool use_overlay)
  {
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

    RoadBoundary boundary = matched_boundary.value_or(RoadBoundary{});
    const bool have_boundary = matched_boundary.has_value();
    std::optional<MissionStatus> status;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status = latest_status_;
    }

    // 叠加画面（用副本绘制，原始帧留给原始录像；网页推流恒开，因此每帧都合成）
    const uint8_t * raw = msg->data.data();
    // 规范化：消息是 rgb8（R,G,B 字节序）。包一个视图、转成 OpenCV 的 BGR 之后再
    // 用 BGR 色板绘制（这样扫描线/文字颜色才正确），最后：
    //   实时推流 → 直接 imencode（BGR mat）；
    //   叠加录像 → 再转回 RGB 交给 AsyncAviWriter（它内部再 RGB2BGR 写 VideoWriter）。
    cv::Mat rgb_view(height, width, CV_8UC3, const_cast<uint8_t *>(raw));
    cv::Mat bgr;
    cv::cvtColor(rgb_view, bgr, cv::COLOR_RGB2BGR);
    {
      const uint8_t * seg_data = nullptr;
      if (use_overlay && seg != nullptr && seg->encoding == "mono8" &&
          static_cast<int>(seg->width) == width && static_cast<int>(seg->height) == height)
      {
        seg_data = seg->data.data();
      }
      scoutcar_web::draw_overlay(bgr, seg_data,
                                 use_overlay && have_boundary ? boundary : RoadBoundary{},
                                 status,
                                 static_cast<float>(current_fps_.load()));
    }

    // 干净录像：原始帧（RGB 原样）+ 叠加帧（转回 RGB）双路同步写（帧号对齐）
    if (recorder_.active()) {
      cv::Mat overlay_rgb;
      cv::cvtColor(bgr, overlay_rgb, cv::COLOR_BGR2RGB);
      recorder_.write_frame(raw, overlay_rgb.data, width, height);
    }

    // 实时推流帧编码（JPEG，主线程编码后加锁换入缓冲，与原版 BMP 思路一致）
    {
      std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, jpg_quality_};
      std::vector<uint8_t> encoded;
      if (cv::imencode(".jpg", bgr, encoded, params)) {
        std::lock_guard<std::mutex> lock(live_mutex_);
        live_jpeg_ = std::move(encoded);
      }
    }
  }

  void onSeg(const Image::SharedPtr msg)
  {
    // debug_image 由感知端同帧合成后，它是唯一的叠加推流源。
    if (have_debug_stream_.load()) { return; }
    if (currentTopic() != overlay_topic_) { return; }
    int64_t match_dt_ns = 0;
    Image::SharedPtr camera = matchCamera(msg->header.stamp, match_dt_ns);
    if (camera == nullptr) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Mask 找不到 40ms 内的原图，最近帧相差 %.1fms",
                           match_dt_ns < 0 ? -1.0 : match_dt_ns / 1e6);
      return;
    }
    if (match_dt_ns > kExactTimestampNs) {
      RCLCPP_DEBUG(get_logger(), "精确 RGB 帧已丢失，使用相邻帧：dt=%.1fms", match_dt_ns / 1e6);
    }
    processFrame(camera, msg, matchBoundary(msg->header.stamp), true);
  }

  void onSource(const Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(source_mutex_);
    source_queue_.emplace_back(msg->header.stamp,msg);
    while(source_queue_.size()>20) source_queue_.pop_front();
  }

  Image::SharedPtr matchSource(const builtin_interfaces::msg::Time & stamp)
  {
    std::lock_guard<std::mutex> lock(source_mutex_);
    for(auto it=source_queue_.rbegin();it!=source_queue_.rend();++it)
      if(rclcpp::Time(it->first)==rclcpp::Time(stamp)) return it->second;
    return nullptr;
  }

  void onDebug(const Image::SharedPtr msg)
  {
    have_debug_stream_.store(true);
    if(currentTopic()!=overlay_topic_||msg->encoding!="rgb8") return;
    updateFps();
    const int width=static_cast<int>(msg->width),height=static_cast<int>(msg->height);
    if(width<=0||height<=0||msg->data.size()<static_cast<size_t>(width)*height*3) return;
    cv::Mat rgb(height,width,CV_8UC3,const_cast<uint8_t *>(msg->data.data()));
    cv::Mat bgr;
    cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR);
    if(recorder_.active()) {
      Image::SharedPtr source=matchSource(msg->header.stamp);
      if(source!=nullptr&&source->data.size()>=static_cast<size_t>(width)*height*3)
        recorder_.write_frame(source->data.data(),msg->data.data(),width,height);
    }
    std::vector<int> params{cv::IMWRITE_JPEG_QUALITY,jpg_quality_};
    std::vector<uint8_t> encoded;
    if(cv::imencode(".jpg",bgr,encoded,params)) {
      std::lock_guard<std::mutex> lock(live_mutex_);
      live_jpeg_=std::move(encoded);
    }
  }

  Image::SharedPtr matchCamera(const builtin_interfaces::msg::Time & stamp, int64_t & match_dt_ns)
  {
    std::lock_guard<std::mutex> lock(cam_mutex_);
    const rclcpp::Time want(stamp);
    Image::SharedPtr best;
    int64_t best_dt = std::numeric_limits<int64_t>::max();
    for (const auto & item : overlay_cam_queue_) {
      const rclcpp::Time have(item.first);
      const int64_t dt = std::abs((want - have).nanoseconds());
      if (dt < best_dt) { best_dt = dt; best = item.second; }
    }
    match_dt_ns = best == nullptr ? -1 : best_dt;
    return best_dt <= kCameraFallbackToleranceNs ? best : nullptr;
  }

  void onBoundary(const RoadBoundary::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(boundary_mutex_);
    boundary_queue_.emplace_back(msg->header.stamp, *msg);
    while (boundary_queue_.size() > kSyncQueueMax) boundary_queue_.pop_front();
  }

  std::optional<RoadBoundary> matchBoundary(const builtin_interfaces::msg::Time & stamp)
  {
    std::lock_guard<std::mutex> lock(boundary_mutex_);
    const rclcpp::Time want(stamp);
    const RoadBoundary * best=nullptr;
    int64_t best_dt=kSyncToleranceNs+1;
    for(const auto & item:boundary_queue_) {
      const int64_t dt=std::abs((want-rclcpp::Time(item.first)).nanoseconds());
      if(dt<best_dt) {best_dt=dt;best=&item.second;}
    }
    if(best==nullptr||best_dt>kSyncToleranceNs) return std::nullopt;
    return *best;
  }

  void onStatus(const MissionStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    latest_status_ = *msg;
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
      recorder_.start();
    } else if (req == -1) {
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

    // 实时 MJPEG 推流（叠加画面）
    svr_->Get("/video_feed", [this](const httplib::Request &, httplib::Response & res) {
      res.set_content_provider(
        static_cast<size_t>(-1),
        "multipart/x-mixed-replace; boundary=frame",
        [this](size_t, size_t, httplib::DataSink & sink) -> bool {
          while (running_.load()) {
            std::vector<uint8_t> frame;
            {
              std::lock_guard<std::mutex> lock(live_mutex_);
              frame = live_jpeg_;
            }
            if (!frame.empty()) {
              std::string part = "--frame\r\n";
              part += "Content-Type: image/jpeg\r\n";
              part += "Content-Length: " + std::to_string(frame.size()) + "\r\n\r\n";
              part += std::string(reinterpret_cast<char *>(frame.data()), frame.size());
              part += "\r\n";
              if (!sink.write(part.data(), part.size())) {
                return false;
              }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(33));  // ~30 FPS
          }
          sink.done();
          return false;
        },
        [this](bool) {
          RCLCPP_DEBUG(get_logger(), "视频客户端断开");
        });
    });

    // 状态（页面轮询）
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
      std::string t = req.get_param_value("topic");
      if (!t.empty() && t[0] != '/') { t = "/" + t; }
      bool found = false;
      for (size_t i = 0; i < camera_topics_.size(); ++i) {
        std::string c = camera_topics_[i];
        c = (c.empty() || c[0] == '/') ? c : ("/" + c);
        if (c == t) { selected_idx_.store(static_cast<int>(i)); found = true; break; }
      }
      if (!found) {
        res.set_content("{\"ok\":false,\"reason\":\"unknown topic\"}", "application/json");
        return;
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

    // 录像片段列表
    svr_->Get("/api/recordings", [this](const httplib::Request &, httplib::Response & res) {
      res.set_content(recordingsJson(), "application/json");
    });

    // 单个片段信息（帧数/fps/尺寸）
    svr_->Get("/api/segment_info", [this](const httplib::Request & req, httplib::Response & res) {
      const std::string base = req.get_param_value("file");
      std::string raw_path, mask_path;
      if (!resolveSegment(base, raw_path, mask_path)) {
        res.set_content("{\"ok\":false,\"reason\":\"bad file\"}", "application/json");
        return;
      }
      cv::VideoCapture cap(raw_path);
      if (!cap.isOpened()) {
        res.set_content("{\"ok\":false,\"reason\":\"cannot open\"}", "application/json");
        return;
      }
      char buf[256];
      snprintf(buf, sizeof(buf),
               "{\"ok\":true,\"base\":\"%s\",\"raw\":\"%s\",\"mask\":\"%s\","
               "\"frames\":%d,\"fps\":%.2f,\"width\":%d,\"height\":%d}",
               base.c_str(), raw_path.c_str(), mask_path.c_str(),
               static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT)),
               cap.get(cv::CAP_PROP_FPS),
               static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
               static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)));
      res.set_content(buf, "application/json");
    });

    // 单帧 JPEG（播放器逐帧取图：?file=base&idx=N&kind=raw|mask）
    svr_->Get("/frame", [this](const httplib::Request & req, httplib::Response & res) {
      const std::string base = req.get_param_value("file");
      const std::string kind = req.get_param_value("kind");
      const int idx = std::atoi(req.get_param_value("idx").c_str());
      std::string raw_path, mask_path;
      if (!resolveSegment(base, raw_path, mask_path) || idx < 0) {
        res.status = 400;
        res.set_content("bad request", "text/plain");
        return;
      }
      const std::string path = (kind == "raw") ? raw_path : mask_path;
      cv::Mat bgr;
      if (!readFrameAt(path, idx, bgr)) {
        res.status = 404;
        res.set_content("frame not found", "text/plain");
        return;
      }
      std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, jpg_quality_};
      std::vector<uint8_t> encoded;
      cv::imencode(".jpg", bgr, encoded, params);
      res.set_content(reinterpret_cast<const char *>(encoded.data()), encoded.size(),
                      "image/jpeg");
      res.set_header("Cache-Control", "no-store");
    });

    // 导出本帧（补标）：?file=base&idx=N&kind=raw|mask → 存 JPEG 到 label_dir
    svr_->Post("/api/export", [this](const httplib::Request & req, httplib::Response & res) {
      const std::string base = req.get_param_value("file");
      const std::string kind = req.get_param_value("kind");
      const int idx = std::atoi(req.get_param_value("idx").c_str());
      std::string raw_path, mask_path;
      if (!resolveSegment(base, raw_path, mask_path) || idx < 0) {
        res.status = 400;
        res.set_content("{\"ok\":false,\"reason\":\"bad request\"}", "application/json");
        return;
      }
      const std::string path = (kind == "raw") ? raw_path : mask_path;
      cv::Mat bgr;
      if (!readFrameAt(path, idx, bgr)) {
        res.status = 404;
        res.set_content("{\"ok\":false,\"reason\":\"frame not found\"}", "application/json");
        return;
      }
      // 保存到补标目录：<base>_<idx>_<时间戳>.jpg
      fs::create_directories(label_dir_);
      char name[256];
      const time_t now = time(nullptr);
      struct tm * t = localtime(&now);
      strftime(name, sizeof(name), "%Y%m%d_%H%M%S", t);
      char filename[512];
      snprintf(filename, sizeof(filename), "%s_%d_%s_%s.jpg",
               base.c_str(), idx, name, kind.c_str());
      const std::string out_path = label_dir_ + "/" + filename;
      if (!cv::imwrite(out_path, bgr)) {
        res.set_content("{\"ok\":false,\"reason\":\"imwrite failed\"}", "application/json");
        return;
      }
      char buf[1024];
      snprintf(buf, sizeof(buf), "{\"ok\":true,\"path\":\"%s\",\"idx\":%d}",
               out_path.c_str(), idx);
      res.set_content(buf, "application/json");
    });
  }

  // ═══════════════ 回放辅助 ═══════════════

  // base 必须是 record_YYYYmmdd_HHMMSS 形态（防路径穿越）
  bool resolveSegment(const std::string & base, std::string & raw_path, std::string & mask_path)
  {
    // record_(7) + 8位日期 + _(1) + 6位时间 = 22
    if (base.size() != 22 || base.rfind("record_", 0) != 0) {
      return false;
    }
    for (int i = 7; i < 15; ++i) {
      if (!std::isdigit(static_cast<unsigned char>(base[i]))) {
        return false;
      }
    }
    if (base[15] != '_') {
      return false;
    }
    for (int i = 16; i < 22; ++i) {
      if (!std::isdigit(static_cast<unsigned char>(base[i]))) {
        return false;
      }
    }
    raw_path = record_dir_ + "/" + base + ".avi";
    mask_path = record_dir_ + "/" + base + "_mask.avi";
    return fs::exists(raw_path) && fs::exists(mask_path);
  }

  // 按帧号读帧。连续读（idx 递增）走缓存不 seek，跨跳才 seek（MJPEG avi seek 很快）。
  bool readFrameAt(const std::string & path, int idx, cv::Mat & bgr)
  {
    std::lock_guard<std::mutex> lock(playback_mutex_);
    if (playback_file_ != path) {
      playback_cap_.open(path);
      playback_file_ = path;
      playback_idx_ = -1;
    }
    if (!playback_cap_.isOpened()) {
      return false;
    }
    if (playback_idx_ != idx - 1) {
      playback_cap_.set(cv::CAP_PROP_POS_FRAMES, idx);
    }
    if (!playback_cap_.read(bgr)) {
      return false;
    }
    playback_idx_ = idx;
    return true;
  }

  // ═══════════════ JSON ═══════════════

  std::string statusJson()
  {
    char buf[1024];
    snprintf(buf, sizeof(buf),
             "{\"ok\":true,\"record_enable\":%s,\"recording\":%s,"
             "\"record_dir\":\"%s\",\"label_dir\":\"%s\",\"fps\":%.1f,\"cameras\":%s}",
             record_enable_ ? "true" : "false",
             recorder_.active() ? "true" : "false",
             record_dir_.c_str(), label_dir_.c_str(),
             current_fps_.load(), camerasJson().c_str());
    return std::string(buf);
  }

  // 相机列表 + 当前选中 + 是否有信号（最近 1s 内收过帧）
  std::string camerasJson()
  {
    std::string sel = currentTopic();
    std::string json = "[";
    auto now = std::chrono::steady_clock::now();
    for (size_t i = 0; i < camera_topics_.size(); ++i) {
      std::string t = camera_topics_[i];
      t = (t.empty() || t[0] == '/') ? t : ("/" + t);
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
               "{\"topic\":\"%s\",\"selected\":%s,\"has_frame\":%s}",
               t.c_str(), (t == sel) ? "true" : "false",
               has ? "true" : "false");
      json += buf;
    }
    json += "]";
    return json;
  }

  std::string recordingsJson()
  {
    // 扫描 record_dir 下 record_*.avi（排除 *_mask.avi），按 base 倒序（新的在前）
    std::vector<std::string> bases;
    if (fs::exists(record_dir_)) {
      for (const auto & entry : fs::directory_iterator(record_dir_)) {
        if (!entry.is_regular_file()) {
          continue;
        }
        const std::string name = entry.path().filename().string();
        if (name.size() > 4 && name.rfind("record_", 0) == 0 &&
            name.compare(name.size() - 4, 4, ".avi") == 0 &&
            name.find("_mask.avi") == std::string::npos)
        {
          bases.push_back(name.substr(0, name.size() - 4));
        }
      }
    }
    std::sort(bases.begin(), bases.end(), std::greater<std::string>());

    std::string json = "[";
    for (size_t i = 0; i < bases.size(); ++i) {
      const std::string & base = bases[i];
      const std::string raw_path = record_dir_ + "/" + base + ".avi";
      const std::string mask_path = record_dir_ + "/" + base + "_mask.avi";
      cv::VideoCapture cap(raw_path);
      int frames = 0;
      double fps = 0.0;
      int width = 0, height = 0;
      uintmax_t size = 0;
      if (cap.isOpened()) {
        frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
        fps = cap.get(cv::CAP_PROP_FPS);
        width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
      }
      std::error_code ec;
      size = fs::file_size(raw_path, ec);
      if (i > 0) { json += ","; }
      char buf[512];
      snprintf(buf, sizeof(buf),
               "{\"base\":\"%s\",\"raw\":\"%s\",\"mask\":\"%s\",\"frames\":%d,"
               "\"fps\":%.2f,\"width\":%d,\"height\":%d,\"size\":%llu}",
               base.c_str(), raw_path.c_str(), mask_path.c_str(), frames,
               fps, width, height, static_cast<unsigned long long>(size));
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
  std::string seg_topic_;
  std::string boundary_topic_;
  std::string source_topic_;
  std::string debug_topic_;
  bool use_local_debug_ = true;
  std::string status_topic_;
  std::string record_dir_;
  std::string label_dir_;
  int record_fps_ = 30;
  bool record_enable_ = true;
  int jpg_quality_ = 85;

  // 话题
  std::vector<rclcpp::Subscription<Image>::SharedPtr> camera_subs_;
  std::atomic<int> selected_idx_{0};
  std::mutex cam_mutex_;
  std::map<std::string, Image::SharedPtr> latest_cam_;
  std::map<std::string, std::chrono::steady_clock::time_point> on_frame_time_;
  static constexpr size_t kSyncQueueMax = 120;  // 30 FPS 约 4 s，覆盖推理和 JPEG 拥塞
  static constexpr int64_t kExactTimestampNs = 1000;  // header 复制时应为 0
  static constexpr int64_t kCameraFallbackToleranceNs = 40000000;  // 最多回退一帧
  static constexpr int64_t kSyncToleranceNs = 2000000;  // Boundary 必须与 Mask 同帧
  std::deque<std::pair<builtin_interfaces::msg::Time, Image::SharedPtr>> overlay_cam_queue_;
  rclcpp::Subscription<Image>::SharedPtr sub_seg_;
  rclcpp::Subscription<RoadBoundary>::SharedPtr sub_boundary_;
  rclcpp::Subscription<Image>::SharedPtr sub_source_;
  rclcpp::Subscription<Image>::SharedPtr sub_debug_;
  rclcpp::Subscription<MissionStatus>::SharedPtr sub_status_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 跨线程共享的最新数据
  std::mutex boundary_mutex_;
  std::deque<std::pair<builtin_interfaces::msg::Time, RoadBoundary>> boundary_queue_;
  std::mutex source_mutex_;
  std::deque<std::pair<builtin_interfaces::msg::Time, Image::SharedPtr>> source_queue_;
  std::atomic<bool> have_debug_stream_{false};
  std::mutex status_mutex_;
  std::optional<MissionStatus> latest_status_;

  // 推流
  std::mutex live_mutex_;
  std::vector<uint8_t> live_jpeg_;
  std::atomic<int> fps_count_{0};
  std::chrono::steady_clock::time_point fps_last_{std::chrono::steady_clock::now()};
  std::atomic<double> current_fps_{0.0};

  // 录像
  scoutcar_web::DualRecorder recorder_;
  std::atomic<int> record_request_{0};   // 0=无请求 1=开始 -1=停止

  // HTTP
  httplib::Server * svr_ = nullptr;
  std::thread * server_thread_ = nullptr;
  std::atomic<bool> running_{false};

  // 回放帧缓存（单槽：一次只服务一个播放器）
  std::mutex playback_mutex_;
  cv::VideoCapture playback_cap_;
  std::string playback_file_;
  int playback_idx_ = -1;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebNode>());
  rclcpp::shutdown();
  return 0;
}
