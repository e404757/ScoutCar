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

#include <scoutcar_msgs/msg/detect_task.hpp>
#include <scoutcar_msgs/msg/road_deviation.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>

#include <opencv2/opencv.hpp>

#include "bag_recorder.h"
#include "dual_recorder.h"
#include "httplib.h"

#include "page_html.h"

using scoutcar_msgs::msg::RoadDeviation;
using sensor_msgs::msg::Image;

enum class ViewMode {
  PERCEPTION,
  IPM,
  FRONT_RAW,
  TURN_RAW,
  FRONT_DETECTION,
  TURN_DETECTION
};

class WebNode : public rclcpp::Node {
public:
  WebNode() : Node("web_node") {
    // ── 参数 ──
    port_ = declare_parameter<int>("port", 8080);
    record_dir_ = declare_parameter<std::string>(
        "record_dir", "/home/orangepi/CityScout/data/record");
    // 0 表示开始录像时采用最近测得的实际感知帧率。
    record_fps_ = declare_parameter<int>("record_fps", 0);
    bag_dir_ = declare_parameter<std::string>(
        "bag_dir", "/home/orangepi/CityScout/data/bags");
    bag_topics_ = declare_parameter<std::vector<std::string>>(
        "bag_topics",
        {"/perception/seg_mask", "/perception/road_boundary",
         "/mission/deviation_enable", "/mission/path_cmd",
         "/mission/detect_task", "/mcu/rx_event"});
    jpg_quality_ = declare_parameter<int>("jpg_quality", 85);

    sub_front_image_ = create_subscription<Image>(
        "camera/front/image_raw", rclcpp::QoS(1).best_effort(),
        [this](const Image::SharedPtr msg) { onFrontCamera(msg); });
    sub_turn_image_ = create_subscription<Image>(
        "camera/turn/image_raw", rclcpp::QoS(1).best_effort(),
        [this](const Image::SharedPtr msg) { onTurnCamera(msg); });
    sub_debug_ = create_subscription<Image>(
        "/perception/debug_image", rclcpp::QoS(1).best_effort(),
        [this](const Image::SharedPtr msg) { onDebug(msg); });
    sub_ipm_debug_ = create_subscription<Image>(
        "/perception/ipm_debug_image", rclcpp::QoS(1).best_effort(),
        [this](const Image::SharedPtr msg) { onIpmDebug(msg); });
    sub_front_detection_ = create_subscription<Image>(
        "/perception/front_detection_image", rclcpp::QoS(1).best_effort(),
        [this](const Image::SharedPtr msg) {
          onDetectionImage(msg, ViewMode::FRONT_DETECTION);
        });
    sub_turn_detection_ = create_subscription<Image>(
        "/perception/turn_detection_image", rclcpp::QoS(1).best_effort(),
        [this](const Image::SharedPtr msg) {
          onDetectionImage(msg, ViewMode::TURN_DETECTION);
        });
    // 数值面板数据
    sub_boundary_ = create_subscription<RoadDeviation>(
        "/perception/road_boundary", 10,
        [this](const RoadDeviation::SharedPtr msg) { onBoundary(msg); });
    pub_obstacle_ =
        create_publisher<std_msgs::msg::Empty>("/obstacle/event", 10);
    const auto detect_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pub_detect_task_ = create_publisher<scoutcar_msgs::msg::DetectTask>(
        "/mission/detect_task", detect_qos);

    control_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                       [this]() { pollRecordRequests(); });

    recorder_.configure(record_dir_, record_fps_);
    bag_recorder_.configure(bag_dir_, bag_topics_);

    startHttpServer(port_);
    RCLCPP_INFO(get_logger(),
                "web_node 就绪：http://<本机IP>:%d（录像目录 %s）", port_,
                record_dir_.c_str());
  }

  ~WebNode() override {
    stopHttpServer();
    stopDetectionRecorders();
    bag_recorder_.stop();
    recorder_.stop();
  }

private:
  // ═══════════════ 话题回调 ═══════════════

  static const char *viewModeName(ViewMode mode) {
    switch (mode) {
    case ViewMode::PERCEPTION:
      return "perception";
    case ViewMode::IPM:
      return "ipm";
    case ViewMode::FRONT_RAW:
      return "front_raw";
    case ViewMode::TURN_RAW:
      return "turn_raw";
    case ViewMode::FRONT_DETECTION:
      return "front_detection";
    case ViewMode::TURN_DETECTION:
      return "turn_detection";
    }
    return "perception";
  }

  void onFrontCamera(const Image::SharedPtr msg) {
    onSource(msg);
    if (view_mode_.load() != ViewMode::FRONT_RAW) {
      return;
    }
    const int width = static_cast<int>(msg->width);
    const int height = static_cast<int>(msg->height);
    if (width <= 0 || height <= 0 || msg->encoding != "rgb8" ||
        msg->data.size() < static_cast<size_t>(width) * height * 3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "图像格式异常: enc=%s %ux%u", msg->encoding.c_str(),
                           msg->width, msg->height);
      return;
    }
    // rgb8 → BGR 后 JPEG 编码（推流色彩正确）
    cv::Mat rgb_view(height, width, CV_8UC3,
                     const_cast<uint8_t *>(msg->data.data()));
    cv::Mat bgr;
    cv::cvtColor(rgb_view, bgr, cv::COLOR_RGB2BGR);
    encodeLive(bgr);
  }
  void onTurnCamera(const Image::SharedPtr msg) {
    onSource(msg);
    if (view_mode_.load() != ViewMode::TURN_RAW) {
      return;
    }
    const int width = static_cast<int>(msg->width);
    const int height = static_cast<int>(msg->height);
    if (width <= 0 || height <= 0 || msg->encoding != "rgb8" ||
        msg->data.size() < static_cast<size_t>(width) * height * 3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "图像格式异常: enc=%s %ux%u", msg->encoding.c_str(),
                           msg->width, msg->height);
      return;
    }
    // rgb8 → BGR 后 JPEG 编码（推流色彩正确）
    cv::Mat rgb_view(height, width, CV_8UC3,
                     const_cast<uint8_t *>(msg->data.data()));
    cv::Mat bgr;
    cv::cvtColor(rgb_view, bgr, cv::COLOR_RGB2BGR);
    encodeLive(bgr);
  }
  void onDebug(const Image::SharedPtr msg) {

    updateRecordInputFps();
    const int width = static_cast<int>(msg->width),
              height = static_cast<int>(msg->height);
    if (width <= 0 || height <= 0 || msg->encoding != "rgb8" ||
        msg->data.size() < static_cast<size_t>(width) * height * 3) {
      return;
    }
    // 叠加录像：原始帧按时间戳从 source 队列取回（同帧配对），帧号天然对齐
    if (recorder_.active()) {
      Image::SharedPtr source = matchSource(msg->header.stamp);
      if (source != nullptr &&
          source->data.size() >= static_cast<size_t>(width) * height * 3) {
        recorder_.write_frame(source->data.data(), msg->data.data(), width,
                              height);
      }
    }
    if (view_mode_.load() != ViewMode::PERCEPTION) {
      return;
    }
    cv::Mat rgb(height, width, CV_8UC3,
                const_cast<uint8_t *>(msg->data.data()));
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    encodeLive(bgr);
  }
  void onIpmDebug(const Image::SharedPtr msg) {
    if (view_mode_.load() != ViewMode::IPM) {
      return;
    }
    const int width = static_cast<int>(msg->width),
              height = static_cast<int>(msg->height);
    if (width <= 0 || height <= 0 || msg->encoding != "rgb8" ||
        msg->data.size() < static_cast<size_t>(width) * height * 3)
      return;
    cv::Mat rgb(height, width, CV_8UC3,
                const_cast<uint8_t *>(msg->data.data()));
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    encodeLive(bgr);
  }

  void onDetectionImage(const Image::SharedPtr msg, ViewMode expected_mode) {
    const int width = static_cast<int>(msg->width);
    const int height = static_cast<int>(msg->height);
    if (width <= 0 || height <= 0 || msg->encoding != "rgb8" ||
        msg->step < msg->width * 3U ||
        msg->data.size() < static_cast<size_t>(msg->step) * height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "检测标注图格式异常: enc=%s %ux%u",
                           msg->encoding.c_str(), msg->width, msg->height);
      return;
    }
    cv::Mat rgb(height, width, CV_8UC3,
                const_cast<uint8_t *>(msg->data.data()), msg->step);
    cv::Mat contiguous_rgb;
    const uint8_t *record_data = msg->data.data();
    if (!rgb.isContinuous()) {
      contiguous_rgb = rgb.clone();
      record_data = contiguous_rgb.data;
    }

    if (recorder_.active()) {
      scoutcar_web::AsyncAviWriter *&writer =
          expected_mode == ViewMode::FRONT_DETECTION
              ? front_detection_writer_
              : turn_detection_writer_;
      if (writer == nullptr) {
        const char *suffix = expected_mode == ViewMode::FRONT_DETECTION
                                 ? "_front_detect.avi"
                                 : "_turn_detect.avi";
        writer = scoutcar_web::AsyncAviWriter::create(
            width, height, active_record_fps_, record_dir_,
            recorder_.current_base() + suffix);
      }
      if (writer != nullptr) writer->write(record_data);
    }

    if (view_mode_.load() != expected_mode) return;
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    encodeLive(bgr);
  }

  // 推流帧编码（JPEG，主线程编码后加锁换入缓冲，帧序号供推流线程检测更新）
  void encodeLive(const cv::Mat &bgr) {
    std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, jpg_quality_};
    std::vector<uint8_t> encoded;
    if (cv::imencode(".jpg", bgr, encoded, params)) {
      std::lock_guard<std::mutex> lock(live_mutex_);
      live_jpeg_ = std::move(encoded);
      frame_seq_.fetch_add(1);
    }
  }

  void onSource(const Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(source_mutex_);
    source_queue_.emplace_back(msg->header.stamp, msg);
    while (source_queue_.size() > 20) {
      source_queue_.pop_front();
    }
  }

  Image::SharedPtr matchSource(const builtin_interfaces::msg::Time &stamp) {
    std::lock_guard<std::mutex> lock(source_mutex_);
    for (auto it = source_queue_.rbegin(); it != source_queue_.rend(); ++it) {
      if (rclcpp::Time(it->first) == rclcpp::Time(stamp)) {
        return it->second;
      }
    }
    return nullptr;
  }

  void onBoundary(const RoadDeviation::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(boundary_mutex_);
    latest_boundary_ = *msg;
    boundary_time_ = std::chrono::steady_clock::now();
  }

  // ═══════════════ 录像控制 ═══════════════

  void pollRecordRequests() {
    const int req = record_request_.exchange(0);
    if (req == 1) {
      const int measured_fps =
          static_cast<int>(std::lround(record_input_fps_.load()));
      const int output_fps =
          record_fps_ > 0 ? record_fps_ : std::clamp(measured_fps, 1, 60);
      const int safe_fps = output_fps > 0 ? output_fps : 20;
      active_record_fps_ = safe_fps;
      if (recorder_.start(safe_fps)) {
        if (!bag_recorder_.start(recorder_.current_base())) {
          RCLCPP_ERROR(get_logger(), "视频已开始，但 rosbag 启动失败");
        }
      }
    } else if (req == -1) {
      bag_recorder_.stop();
      stopDetectionRecorders();
      recorder_.stop();
    }
  }

  void stopDetectionRecorders() {
    if (front_detection_writer_ != nullptr) {
      front_detection_writer_->release();
      delete front_detection_writer_;
      front_detection_writer_ = nullptr;
    }
    if (turn_detection_writer_ != nullptr) {
      turn_detection_writer_->release();
      delete turn_detection_writer_;
      turn_detection_writer_ = nullptr;
    }
  }

  // ═══════════════ 录像输入帧率统计 ═══════════════
  void updateRecordInputFps() {
    record_fps_count_++;
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
        std::chrono::duration<double>(now - record_fps_last_).count();
    if (elapsed >= 2.0) {
      record_input_fps_.store(record_fps_count_ / elapsed);
      record_fps_count_ = 0;
      record_fps_last_ = now;
    }
  }

  // ═══════════════ HTTP 服务器 ═══════════════

  void startHttpServer(int port) {
    svr_ = new httplib::Server();
    setupRoutes();
    running_.store(true);
    server_thread_ = new std::thread([this, port]() {
      RCLCPP_INFO(get_logger(), "HTTP 服务器启动，端口 %d", port);
      svr_->listen("0.0.0.0", port);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void stopHttpServer() {
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

  void setupRoutes() {
    // 页面
    svr_->Get("/", [](const httplib::Request &, httplib::Response &res) {
      res.set_content(kPageHtml, "text/html; charset=utf-8");
    });

    // 实时 MJPEG 推流；具体画面由 view_mode_ 选择
    svr_->Get("/video_feed", [this](const httplib::Request &,
                                    httplib::Response &res) {
      res.set_content_provider(
          static_cast<size_t>(-1), "multipart/x-mixed-replace; boundary=frame",
          [this](size_t, size_t, httplib::DataSink &sink) -> bool {
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
                part += "Content-Length: " + std::to_string(frame.size()) +
                        "\r\n\r\n";
                part += std::string(reinterpret_cast<char *>(frame.data()),
                                    frame.size());
                part += "\r\n";
                if (!sink.write(part.data(), part.size())) {
                  return false;
                }
                have_sent = true;
              }
              last_seq = seq;
              std::this_thread::sleep_for(
                  std::chrono::milliseconds(5)); // 新帧到达后 ≤5ms 推出
            }
            sink.done();
            return false;
          },
          [this](bool) { RCLCPP_DEBUG(get_logger(), "视频客户端断开"); });
    });

    // 状态（页面轮询：录像状态、当前视图、道路跟踪参数）
    svr_->Get("/api/status",
              [this](const httplib::Request &, httplib::Response &res) {
                res.set_content(statusJson(), "application/json");
              });

    svr_->Post("/api/view/set", [this](const httplib::Request &req,
                                       httplib::Response &res) {
      const std::string mode = req.get_param_value("mode");
      ViewMode next;
      if (mode == "perception")
        next = ViewMode::PERCEPTION;
      else if (mode == "ipm")
        next = ViewMode::IPM;
      else if (mode == "front_raw")
        next = ViewMode::FRONT_RAW;
      else if (mode == "turn_raw")
        next = ViewMode::TURN_RAW;
      else if (mode == "front_detection")
        next = ViewMode::FRONT_DETECTION;
      else if (mode == "turn_detection")
        next = ViewMode::TURN_DETECTION;
      else {
        res.status = 400;
        res.set_content("{\"ok\":false,\"reason\":\"unknown mode\"}",
                        "application/json");
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
    svr_->Post("/api/record/start",
               [this](const httplib::Request &, httplib::Response &res) {
                 record_request_.store(1);
                 res.set_content(statusJson(), "application/json");
               });
    svr_->Post("/api/record/stop",
               [this](const httplib::Request &, httplib::Response &res) {
                 record_request_.store(-1);
                 res.set_content(statusJson(), "application/json");
               });

    // 人工障碍只模拟“检测事件”。封边、重规划和掉头仍由 mission_node 决定。
    svr_->Post("/api/obstacle",
               [this](const httplib::Request &, httplib::Response &res) {
                 pub_obstacle_->publish(std_msgs::msg::Empty{});
                 RCLCPP_INFO(get_logger(), "Web 人工触发障碍事件");
                 res.set_content("{\"ok\":true}", "application/json");
               });

    // 网页发布统一的侦察任务状态；开始和结束都不等待下位机回传。
    svr_->Post("/api/detect/toggle",
               [this](const httplib::Request &, httplib::Response &res) {
                 bool active = false;
                 {
                   std::lock_guard<std::mutex> lock(detect_command_mutex_);
                   active = !detect_active_.load();

                   scoutcar_msgs::msg::DetectTask command;
                   command.status = active
                                        ? scoutcar_msgs::msg::DetectTask::START
                                        : scoutcar_msgs::msg::DetectTask::END;
                   command.left_result = 0;
                   command.right_result = 0;
                   pub_detect_task_->publish(command);
                   detect_active_.store(active);
                 }

                 RCLCPP_INFO(get_logger(), "Web %s侦察任务（结果 00 00）",
                             active ? "开始" : "结束");
                 res.set_content(statusJson(), "application/json");
               });
  }

  // ═══════════════ JSON ═══════════════

  std::string statusJson() {
    const auto now = std::chrono::steady_clock::now();
    auto ageMs = [&now](const std::chrono::steady_clock::time_point &t) {
      return t.time_since_epoch().count() == 0
                 ? -1
                 : static_cast<long long>(
                       std::chrono::duration_cast<std::chrono::milliseconds>(
                           now - t)
                           .count());
    };

    // 道路跟踪参数面板：仅暴露当前算法采用的阈值和扫描行。
    std::string boundary_json;
    {
      std::lock_guard<std::mutex> lock(boundary_mutex_);
      const long long ms = ageMs(boundary_time_);
      const bool online = ms >= 0 && ms <= 1500;
      const RoadDeviation *boundary = latest_boundary_ ? &*latest_boundary_ : nullptr;
      char buf[1024];
      snprintf(buf, sizeof(buf),
               "{\"online\":%s,\"status\":%u,\"deviation\":%d,"
               "\"road_center\":%u,\"scan_y\":%u,"
               "\"left\":%d,\"right\":%d,"
               "\"min_width\":%u,\"max_width\":%u,"
               "\"boundary_source\":\"%s\","
               "\"front_reference_x\":%d,\"turn_reference_x\":%d}",
               online ? "true" : "false",
               boundary ? static_cast<unsigned>(boundary->status) : 0U,
               boundary ? static_cast<int>(boundary->deviation) : 0,
               boundary ? static_cast<unsigned>(boundary->road_center) : 0U,
               boundary ? static_cast<unsigned>(boundary->scan_y) : 0U,
               boundary ? static_cast<int>(boundary->left) : 0,
               boundary ? static_cast<int>(boundary->right) : 0,
               boundary ? static_cast<unsigned>(boundary->min_width) : 0U,
               boundary ? static_cast<unsigned>(boundary->max_width) : 0U,
               boundary ? boundary->boundary_source.c_str() : "",
               boundary ? boundary->front_reference_x : 0,
               boundary ? boundary->turn_reference_x : 0);
      boundary_json = buf;
    }

    char buf[1024];
    snprintf(buf, sizeof(buf),
             "{\"ok\":true,\"recording\":%s,\"bag_recording\":%s,"
             "\"detecting\":%s,"
             "\"view_mode\":\"%s\","
             "\"boundary\":%s}",
             recorder_.active() ? "true" : "false",
             bag_recorder_.active() ? "true" : "false",
             detect_active_.load() ? "true" : "false",
             viewModeName(view_mode_.load()), boundary_json.c_str());
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
  std::atomic<bool> detect_active_{false};
  std::mutex detect_command_mutex_;
  rclcpp::Subscription<Image>::SharedPtr sub_front_image_;
  rclcpp::Subscription<Image>::SharedPtr sub_turn_image_;
  rclcpp::Subscription<Image>::SharedPtr sub_debug_;
  rclcpp::Subscription<Image>::SharedPtr sub_ipm_debug_;
  rclcpp::Subscription<RoadDeviation>::SharedPtr sub_boundary_;
  rclcpp::Subscription<Image>::SharedPtr sub_front_detection_;
  rclcpp::Subscription<Image>::SharedPtr sub_turn_detection_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_obstacle_;
  rclcpp::Publisher<scoutcar_msgs::msg::DetectTask>::SharedPtr pub_detect_task_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 两路相机原图按时间戳缓存，用于和 debug_image 精确配对录像。
  std::mutex source_mutex_;
  std::deque<std::pair<builtin_interfaces::msg::Time, Image::SharedPtr>>
      source_queue_;

  // 参数面板数据
  std::mutex boundary_mutex_;
  std::optional<RoadDeviation> latest_boundary_;
  std::chrono::steady_clock::time_point boundary_time_{};

  // 推流
  std::mutex live_mutex_;
  std::vector<uint8_t> live_jpeg_;
  std::atomic<uint64_t> frame_seq_{
      0}; // 每编码一帧 +1，推流线程据此跳过未更新帧
  int record_fps_count_ = 0;
  std::chrono::steady_clock::time_point record_fps_last_{
      std::chrono::steady_clock::now()};
  std::atomic<double> record_input_fps_{20.0};

  // 录像
  scoutcar_web::DualRecorder recorder_;
  scoutcar_web::BagRecorder bag_recorder_;
  std::atomic<int> record_request_{0}; // 0=无请求 1=开始 -1=停止
  int active_record_fps_ = 20;
  scoutcar_web::AsyncAviWriter *front_detection_writer_ = nullptr;
  scoutcar_web::AsyncAviWriter *turn_detection_writer_ = nullptr;

  // HTTP
  httplib::Server *svr_ = nullptr;
  std::thread *server_thread_ = nullptr;
  std::atomic<bool> running_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebNode>());
  rclcpp::shutdown();
  return 0;
}
