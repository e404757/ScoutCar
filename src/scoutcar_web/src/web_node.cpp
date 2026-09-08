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
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <scoutcar_msgs/msg/road_deviation.hpp>

#include <opencv2/opencv.hpp>

#include "httplib.h"
#include "bag_recorder.h"
#include "dual_recorder.h"

#include "page_html.h"

using sensor_msgs::msg::Image;
using scoutcar_msgs::msg::RoadDeviation;

enum class ViewMode
{
  PERCEPTION,
  IPM,
  USB_RAW,
  MIPI_RAW
};

class WebNode : public rclcpp::Node
{
public: 
  WebNode() : Node("web_node")
  {
    // ── 参数 ──
    port_ = declare_parameter<int>("port", 8080);
    record_dir_ = declare_parameter<std::string>(
      "record_dir", "/home/orangepi/CityScout/data/record");
    // 0 表示开始录像时采用最近测得的实际感知帧率。
    record_fps_ = declare_parameter<int>("record_fps", 0);
    bag_dir_ = declare_parameter<std::string>(
      "bag_dir", "/home/orangepi/CityScout/data/bags");
    bag_topics_ = declare_parameter<std::vector<std::string>>(
      "bag_topics", {
        "/perception/seg_mask",
        "/perception/road_boundary",
        "/mission/deviation_enable",
        "/mission/path_cmd",
        "/mcu/rx_event"
      });
    jpg_quality_ = declare_parameter<int>("jpg_quality", 85);


    sub_usb_image_ = create_subscription<Image>(
      "camera/usb/image_raw", rclcpp::QoS(1).best_effort(),
      [this](const Image::SharedPtr msg) { onUSBCamera(msg); });
    sub_mipi_image_ = create_subscription<Image>(
      "camera/mipi/image_raw", rclcpp::QoS(1).best_effort(),
      [this](const Image::SharedPtr msg) { onMIPICamera(msg); });
    sub_debug_ = create_subscription<Image>(
      "/perception/debug_image", rclcpp::QoS(1).best_effort(),
      [this](const Image::SharedPtr msg) { onDebug(msg); });
    sub_ipm_debug_ = create_subscription<Image>(
      "/perception/ipm_debug_image", rclcpp::QoS(1).best_effort(),
      [this](const Image::SharedPtr msg) { onIpmDebug(msg); });
    sub_source_ = create_subscription<Image>(
      "/perception/source_image", rclcpp::QoS(5).best_effort(),
      [this](const Image::SharedPtr msg) { onSource(msg); });
    // 数值面板数据
    sub_boundary_ = create_subscription<RoadDeviation>(
      "/perception/road_boundary", 10,
      [this](const RoadDeviation::SharedPtr msg) { onBoundary(msg); });
  
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { pollRecordRequests(); });

    recorder_.configure(record_dir_, record_fps_);
    bag_recorder_.configure(bag_dir_, bag_topics_);

    startHttpServer(port_);
    RCLCPP_INFO(get_logger(), "web_node 就绪：http://<本机IP>:%d（录像目录 %s）",
                port_, record_dir_.c_str());
   
  }

  ~WebNode() override
  {
    stopHttpServer();
    bag_recorder_.stop();
    recorder_.stop();
  }

private:
  // ═══════════════ 话题回调 ═══════════════

  static const char * viewModeName(ViewMode mode)
  {
    switch (mode) {
      case ViewMode::PERCEPTION: return "perception";
      case ViewMode::IPM: return "ipm";
      case ViewMode::USB_RAW: return "usb_raw";
      case ViewMode::MIPI_RAW: return "mipi_raw";
    }
    return "perception";
  }

  void onUSBCamera(const Image::SharedPtr msg)
  {
    if(view_mode_.load() != ViewMode::USB_RAW) { return; }
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
  void onMIPICamera(const Image::SharedPtr msg)
  {
    if(view_mode_.load() != ViewMode::MIPI_RAW) { return; }
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
  void onDebug(const Image::SharedPtr msg)
  {
    
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
    if(view_mode_.load() != ViewMode::PERCEPTION) { return;}
    cv::Mat rgb(height, width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()));
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    encodeLive(bgr);
    
  }
  void onIpmDebug(const Image::SharedPtr msg)
  {
    if(view_mode_.load() != ViewMode::IPM){return;}
    const int width = static_cast<int>(msg->width), height = static_cast<int>(msg->height);
    if (width <= 0 || height <= 0 || msg->encoding != "rgb8" ||
        msg->data.size() < static_cast<size_t>(width) * height * 3) return;
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

  void onBoundary(const RoadDeviation::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(boundary_mutex_);
    latest_boundary_ = *msg;
    boundary_time_ = std::chrono::steady_clock::now();
  }

  // ═══════════════ 录像控制 ═══════════════

  void pollRecordRequests()
  {
    const int req = record_request_.exchange(0);
    if (req == 1) {
      const int measured_fps = static_cast<int>(std::lround(record_input_fps_.load()));
      const int output_fps = record_fps_ > 0 ? record_fps_ : std::clamp(measured_fps, 1, 60);
      const int safe_fps = output_fps > 0 ? output_fps : 20;
      if (recorder_.start(safe_fps) ) {
        if (!bag_recorder_.start(recorder_.current_base())) {
          RCLCPP_ERROR(get_logger(), "视频已开始，但 rosbag 启动失败");
        }
      }
    } else if (req == -1) {
      bag_recorder_.stop();
      recorder_.stop();
    }
  }

  // ═══════════════ 录像输入帧率统计 ═══════════════
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

    // 实时 MJPEG 推流；具体画面由 view_mode_ 选择
    svr_->Get("/video_feed", [this](const httplib::Request &,
                                    httplib::Response & res) {
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

    // 状态（页面轮询：录像状态、当前视图、道路跟踪参数）
    svr_->Get("/api/status", [this](const httplib::Request &, httplib::Response & res) {
      res.set_content(statusJson(), "application/json");
    });

    svr_->Post("/api/view/set", [this](const httplib::Request & req, httplib::Response & res) {
      const std::string mode = req.get_param_value("mode");
      ViewMode next;
      if (mode == "perception") next = ViewMode::PERCEPTION;
      else if (mode == "ipm") next = ViewMode::IPM;
      else if (mode == "usb_raw") next = ViewMode::USB_RAW;
      else if (mode == "mipi_raw") next = ViewMode::MIPI_RAW;
      else {
        res.status = 400;
        res.set_content("{\"ok\":false,\"reason\":\"unknown mode\"}", "application/json");
        return;
      }
      view_mode_.store(next);
      {
        std::lock_guard<std::mutex> lock(live_mutex_);
        live_jpeg_.clear();
      }
      RCLCPP_INFO(get_logger(), "切换网页画面 → %s", viewModeName(next));
      res.set_content(statusJson(), "application/json");
    });

    // 录像起停（网页按钮）
    svr_->Post("/api/record/start", [this](const httplib::Request &, httplib::Response & res) {
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

    // 道路跟踪参数面板：仅暴露当前算法采用的阈值和扫描行。
    std::string boundary_json;
    {
      std::lock_guard<std::mutex> lock(boundary_mutex_);
      const long long ms = ageMs(boundary_time_);
      const bool online = ms >= 0 && ms <= 1500;
      char buf[1024];
      snprintf(buf, sizeof(buf),
               "{\"online\":%s,\"scan_y\":%u,"
               "\"min_width\":%u,\"max_width\":%u}",
               online ? "true" : "false",
               latest_boundary_ ? latest_boundary_->scan_y : 0,
               latest_boundary_ ? latest_boundary_->min_width : 0,
               latest_boundary_ ? latest_boundary_->max_width : 0);
      boundary_json = buf;
    }

    char buf[1024];
    snprintf(buf, sizeof(buf),
             "{\"ok\":true,\"recording\":%s,\"bag_recording\":%s,"
             "\"view_mode\":\"%s\","
             "\"boundary\":%s}",
             recorder_.active() ? "true" : "false",
             bag_recorder_.active() ? "true" : "false",
             viewModeName(view_mode_.load()),
             boundary_json.c_str());
    return std::string(buf);
  }

  // ═══════════════ 成员 ═══════════════

  // 参数
  int port_ = 8080;
  std::string record_dir_;
  int record_fps_ = 0;
  std::string bag_dir_;
  std::vector<std::string> bag_topics_;
  int jpg_quality_ = 85;

  // 话题
  std::atomic<ViewMode> view_mode_{ViewMode::PERCEPTION};
  rclcpp::Subscription<Image>::SharedPtr sub_usb_image_;
  rclcpp::Subscription<Image>::SharedPtr sub_mipi_image_;
  rclcpp::Subscription<Image>::SharedPtr sub_source_;
  rclcpp::Subscription<Image>::SharedPtr sub_debug_;
  rclcpp::Subscription<Image>::SharedPtr sub_ipm_debug_;
  rclcpp::Subscription<RoadDeviation>::SharedPtr sub_boundary_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 叠加录像的原始帧配对队列（source_image 按时间戳精确匹配 debug_image）
  std::mutex source_mutex_;
  std::deque<std::pair<builtin_interfaces::msg::Time, Image::SharedPtr>> source_queue_;

  // 参数面板数据
  std::mutex boundary_mutex_;
  std::optional<RoadDeviation> latest_boundary_;
  std::chrono::steady_clock::time_point boundary_time_{};

  // 推流
  std::mutex live_mutex_;
  std::vector<uint8_t> live_jpeg_;
  std::atomic<uint64_t> frame_seq_{0};   // 每编码一帧 +1，推流线程据此跳过未更新帧
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
