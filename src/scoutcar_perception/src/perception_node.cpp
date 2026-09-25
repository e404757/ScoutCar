#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <scoutcar_msgs/msg/road_deviation.hpp>
#include <scoutcar_msgs/msg/recon_result.hpp>
#include <scoutcar_msgs/msg/detect_task.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <scoutcar_msgs/msg/car_state.hpp>

#include "scoutcar_perception/perception_visualizer.hpp"
#include "scoutcar_perception/road_tracker.hpp"
#include "scoutcar_perception/segmentation_model.hpp"
#include "scoutcar_perception/yolov8_model.hpp"

namespace {

constexpr float kReconConfidenceThreshold = 0.9F;
constexpr int kReconRequiredFrames = 3;

struct ReconSideLock {
  int candidate_class = -1;
  int consecutive_frames = 0;
  bool locked = false;
  uint8_t result = 0;
  float confidence = 0.0F;
};

}  // namespace

class PerceptionNode : public rclcpp::Node {
public:
  PerceptionNode() : Node("perception_node") {

    //初始化Seg模型
    const auto package_share =
        ament_index_cpp::get_package_share_directory("scoutcar_perception");
    seg_model_path_ = declare_parameter<std::string>(
        "seg_model_path", package_share + "/models/yolov5_seg/V2.0/seg_V2.0_int8.rknn");
    seg_label_path_ = declare_parameter<std::string>(
        "seg_label_path", package_share + "/models/yolov5_seg/V1.0/seg_label.txt");
    scoutcar_perception::SegmentationConfig segmentation_config;
    segmentation_config.model_path = seg_model_path_;
    segmentation_config.label_path = seg_label_path_;
    segmentation_model_ =
        std::make_unique<scoutcar_perception::SegmentationModel>(
            segmentation_config);
    if (!segmentation_model_->is_initialized()) {
    throw std::runtime_error("Seg模型初始化失败: " +
                            segmentation_model_->initialization_error());
    }
    RCLCPP_INFO(get_logger(), "Seg模型初始化成功");

    // 初始化YOLOv8检测模型
    detect_model_path_ = declare_parameter<std::string>(
        "detect_model_path",
        package_share + "/models/yolov8_detect/V1.0/yolo8_v1.0_int8.rknn");
    detect_label_path_ = declare_parameter<std::string>(
        "detect_label_path",
        package_share + "/models/yolov8_detect/V1.0/labels.txt");
    detect_confidence_threshold_ =
        declare_parameter<double>("detect.confidence_threshold", 0.25);
    detect_nms_threshold_ =
        declare_parameter<double>("detect.nms_threshold", 0.45);
    enable_turning_inference_ =
        declare_parameter<bool>("enable_turning_inference", false);
    scoutcar_perception::YoloV8Config detection_config;
    detection_config.model_path = detect_model_path_;
    detection_config.label_path = detect_label_path_;
    detection_config.confidence_threshold =
        static_cast<float>(detect_confidence_threshold_);
    detection_config.nms_threshold = static_cast<float>(detect_nms_threshold_);
    detection_model_ =
        std::make_unique<scoutcar_perception::YoloV8Model>(detection_config);
    if (!detection_model_->is_initialized()) {
      throw std::runtime_error(
          "YOLOv8模型初始化失败: " +
          detection_model_->initialization_error());
    }
    RCLCPP_INFO(get_logger(), "YOLOv8模型初始化成功");

    //读取track参数
    road_tracking::Config tracking_config;
    tracking_config.scan_end_y =
        declare_parameter<int>("road_tracking.scan_end_y", 120);
    tracking_config.min_road_width_px =
        declare_parameter<int>("road_tracking.min_road_width_px", 225);
    tracking_config.max_road_width_px =
        declare_parameter<int>("road_tracking.max_road_width_px", 285);
    tracking_config.min_barrier_width_px =
        declare_parameter<int>("road_tracking.min_barrier_width_px", 12);
    tracking_config.search_expand_px =
        declare_parameter<int>("road_tracking.search_expand_px", 20);
    tracking_config.min_valid_rows =
        declare_parameter<int>("road_tracking.min_valid_rows", 60);
    front_reference_x_ =
        declare_parameter<int>("road_tracking.front_reference_x", 320);
    turn_left_reference_x_ =
        declare_parameter<int>("road_tracking.turn_left_reference_x", 320);
    turn_right_reference_x_ =
        declare_parameter<int>("road_tracking.turn_right_reference_x", 320);
    road_tracker_ =
        std::make_unique<road_tracking::RoadTracker>(tracking_config);

    //读取「挡板拼满整行」参数（接近侦察点的自动触发判据）
    blocked_window_bottom_ratio_ = declare_parameter<double>(
        "blocked_detect.window_bottom_ratio", 0.6);
    blocked_config_.required_rows =
        declare_parameter<int>("blocked_detect.required_rows", 30);
    blocked_config_.min_barrier_width_px =
        tracking_config.min_barrier_width_px;//复用 road_tracking 的挡板宽度过滤
    blocked_config_.edge_tolerance_px =
        declare_parameter<int>("blocked_detect.edge_tolerance_px", 20);
    blocked_rearm_absent_frames_ =
        declare_parameter<int>("blocked_detect.rearm_absent_frames", 30);
    blocked_auto_exit_s_ =
        declare_parameter<double>("blocked_detect.auto_exit_s", 5.0);

    build_ipm_matrices();

    pub_deviation_ = create_publisher<scoutcar_msgs::msg::RoadDeviation>(
        "perception/road_deviation", 10);
    const auto debug_qos = rclcpp::QoS(1).best_effort();
    pub_mask_debug_ = create_publisher<sensor_msgs::msg::Image>(
        "perception/debug_image", debug_qos);
    pub_ipm_debug_ = create_publisher<sensor_msgs::msg::Image>(
        "perception/ipm_debug_image", debug_qos);
    pub_front_detection_ = create_publisher<sensor_msgs::msg::Image>(
        "perception/front_detection_image", debug_qos);
    pub_turn_detection_ = create_publisher<sensor_msgs::msg::Image>(
        "perception/turn_detection_image", debug_qos);
    pub_recon_result_ = create_publisher<scoutcar_msgs::msg::ReconResult>(
      "perception/recon_result", 10);
    pub_detect_task_ = create_publisher<scoutcar_msgs::msg::DetectTask>(
      "mission/detect_task", 10);
    const auto status_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sub_btp_debug_cmd_ = create_subscription<std_msgs::msg::UInt8>(
        "debug/btp_camera_pose", status_qos,
        [this](const std_msgs::msg::UInt8::SharedPtr msg) {
          const bool active =
              msg->data == scoutcar_msgs::msg::CarState::DIRECTION_LEFT ||
              msg->data == scoutcar_msgs::msg::CarState::DIRECTION_RIGHT;
          btp_debug_pose_ = active ? msg->data : 0;
          RCLCPP_INFO(get_logger(), "BTP 调试%s，转向相机姿态=%u",
                      active ? "开启" : "关闭",
                      static_cast<unsigned>(btp_debug_pose_));
        });
    sub_mission_state_ = create_subscription<scoutcar_msgs::msg::CarState>(
        "mission/mission_state", status_qos,
        [this](const scoutcar_msgs::msg::CarState::SharedPtr msg) {
          const bool cameras_look_opposite =
              (msg->front_camera_pose ==
                 scoutcar_msgs::msg::CarState::DIRECTION_LEFT &&
               msg->turn_camera_pose ==
                 scoutcar_msgs::msg::CarState::DIRECTION_RIGHT) ||
              (msg->front_camera_pose ==
                 scoutcar_msgs::msg::CarState::DIRECTION_RIGHT &&
               msg->turn_camera_pose ==
                 scoutcar_msgs::msg::CarState::DIRECTION_LEFT);
          const bool next_detect_active =
              msg->mission_state == scoutcar_msgs::msg::CarState::DETECTING &&
              cameras_look_opposite;
          if (!detect_active_ && next_detect_active) {
            reset_recon_result();
          } else if (detect_active_ && !next_detect_active) {
            publish_recon_result();
          }
          detect_active_ = next_detect_active;
          mission_state_ = msg->mission_state;
          turn_camera_pose_ = msg->turn_camera_pose;
          in_tunnel_ = msg->in_tunnel != 0;
        });
    sub_turn_image_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/turn/image_raw",
        rclcpp::QoS(1)
            .best_effort(), // 与 camera_node 的 QoS 匹配；depth=1 只留最新帧
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          if (detect_active_) {
            process_Detectframe(msg, pub_turn_detection_, "转向相机", false);
          } else if (mission_state_ ==
                     scoutcar_msgs::msg::CarState::FINDING_BTP) {
            process_Segframe(msg, true);
          } else if (btp_debug_pose_ != 0) {
            process_Segframe(msg, true, false);
          }
        });
    sub_front_image_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/front/image_raw",
        rclcpp::QoS(1)
            .best_effort(), // 与 camera_node 的 QoS 匹配；depth=1 只留最新帧
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          if (detect_active_) {
            process_Detectframe(msg, pub_front_detection_, "前视相机", true);
          } else if (btp_debug_pose_ != 0) {
            
          } else if (
              mission_state_ != scoutcar_msgs::msg::CarState::FINDING_BTP &&
              (mission_state_ != scoutcar_msgs::msg::CarState::TURNING ||
               enable_turning_inference_)) {
            const bool publish_control_deviation =
                mission_state_ != scoutcar_msgs::msg::CarState::TURNING;
            process_Segframe(
                msg, false, publish_control_deviation);
          }
        });
  }

  ~PerceptionNode() override = default;

private:
  void build_ipm_matrices() {
    const auto front_source = declare_parameter<std::vector<double>>(
        "ipm.front_source_points",
        {232.0, 190.0, 431.0, 190.0, 95.0, 420.0, 588.0, 420.0});
    const auto turn_left_source = declare_parameter<std::vector<double>>(
        "ipm.turn_left_source_points",
        {260.0, 190.0, 400.0, 190.0, 145.0, 420.0, 520.0, 420.0});
    const auto turn_right_source = declare_parameter<std::vector<double>>(
        "ipm.turn_right_source_points",
        {260.0, 190.0, 400.0, 190.0, 145.0, 420.0, 520.0, 420.0});
    const auto front_destination = declare_parameter<std::vector<double>>(
        "ipm.front_destination_points",
        {180.0, 120.0, 460.0, 120.0, 180.0, 470.0, 460.0, 470.0});
    const auto turn_left_destination = declare_parameter<std::vector<double>>(
        "ipm.turn_left_destination_points",
        {170.0, 120.0, 470.0, 120.0, 170.0, 470.0, 470.0, 470.0});
    const auto turn_right_destination = declare_parameter<std::vector<double>>(
        "ipm.turn_right_destination_points",
        {170.0, 120.0, 470.0, 120.0, 170.0, 470.0, 470.0, 470.0});

    if (front_source.size() != 8 || turn_left_source.size() != 8 ||
        turn_right_source.size() != 8 || front_destination.size() != 8 ||
        turn_left_destination.size() != 8 ||
        turn_right_destination.size() != 8) {
      throw std::runtime_error(
          "front/turn_left/turn_right 的 IPM 点必须各有 8 个数");
    }

    const auto make_ipm = [](const std::vector<double> &source,
                             const std::vector<double> &destination) {
      std::vector<cv::Point2f> source_points;
      std::vector<cv::Point2f> destination_points;
      for (size_t i = 0; i < source.size(); i += 2) {
        source_points.emplace_back(source[i], source[i + 1]);
        destination_points.emplace_back(destination[i], destination[i + 1]);
      }
      return cv::getPerspectiveTransform(source_points, destination_points);
    };

    front_ipm_matrix_ = make_ipm(front_source, front_destination);
    turn_left_ipm_matrix_ =
        make_ipm(turn_left_source, turn_left_destination);
    turn_right_ipm_matrix_ =
        make_ipm(turn_right_source, turn_right_destination);
    front_ipm_inverse_ = front_ipm_matrix_.inv();
    turn_left_ipm_inverse_ = turn_left_ipm_matrix_.inv();
    turn_right_ipm_inverse_ = turn_right_ipm_matrix_.inv();
  }

  bool should_make_debug() const {
    return pub_mask_debug_->get_subscription_count() > 0 ||
           pub_ipm_debug_->get_subscription_count() > 0;
  }


  void publish_debug_images(const sensor_msgs::msg::Image &source,
                            const uint8_t *mask, const uint8_t *ipm_mask,
                            const uint8_t *selected_mask,
                            const road_tracking::Result &result,
                            int reference_x, const cv::Mat &ipm_inverse) {
    try {
      if (pub_mask_debug_->get_subscription_count() > 0) {
        auto debug_image = visualizer_.make_mask_debug(
            source, mask, selected_mask, result, reference_x, ipm_inverse);
        pub_mask_debug_->publish(std::move(debug_image));
      }
      if (pub_ipm_debug_->get_subscription_count() > 0) {
        auto ipm_debug_image = visualizer_.make_ipm_debug(
            source, ipm_mask, selected_mask, result, reference_x);
        pub_ipm_debug_->publish(std::move(ipm_debug_image));
      }
    } catch (const cv::Exception &error) {
      RCLCPP_ERROR(get_logger(), "绘制调试图失败: %s", error.what());
    }
  }

  void process_Segframe(const sensor_msgs::msg::Image::SharedPtr msg,
                        bool use_turn_camera,
                        bool publish_control_deviation = true) {
    const bool make_debug = should_make_debug();
    const uint8_t active_turn_pose =
        btp_debug_pose_ != 0 ? btp_debug_pose_ : turn_camera_pose_;
    const bool use_right_turn_ipm =
        active_turn_pose == scoutcar_msgs::msg::CarState::DIRECTION_RIGHT;
    const int configured_reference_x =
        !use_turn_camera
            ? front_reference_x_
            : (use_right_turn_ipm ? turn_right_reference_x_
                                  : turn_left_reference_x_);
    const cv::Mat &ipm_inverse =
        !use_turn_camera
            ? front_ipm_inverse_
            : (use_right_turn_ipm ? turn_right_ipm_inverse_
                                  : turn_left_ipm_inverse_);

    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();

    auto seg_result = segmentation_model_->infer(
        msg->data.data(), static_cast<int>(msg->width),
        static_cast<int>(msg->height));
    const auto t1 = Clock::now();

    if (!seg_result.error.empty()) {
      RCLCPP_ERROR(get_logger(), "Seg推理失败: %s", seg_result.error.c_str());
      if (publish_control_deviation) {
        publish_deviation(
            msg->header,
            scoutcar_msgs::msg::RoadDeviation::STATUS_INFERENCE_ERROR);
      }
      if (make_debug) {
        publish_debug_images(*msg, nullptr, nullptr, nullptr,
                             road_tracking::Result{}, configured_reference_x,
                             ipm_inverse);
      }
      return;
    }
    if (!seg_result.valid) {
      if (publish_control_deviation) {
        publish_deviation(
            msg->header,
            scoutcar_msgs::msg::RoadDeviation::STATUS_NO_SEGMENTATION);
      }
      if (make_debug) {
        publish_debug_images(*msg, nullptr, nullptr, nullptr,
                             road_tracking::Result{}, configured_reference_x,
                             ipm_inverse);
      }
      return;
    }
    cv::Mat source_mask(seg_result.height, seg_result.width, CV_8UC1,
                        seg_result.mask.data());
    cv::Mat ipm_mask;
    const cv::Mat &ipm_matrix =
        !use_turn_camera
            ? front_ipm_matrix_
            : (use_right_turn_ipm ? turn_right_ipm_matrix_
                                  : turn_left_ipm_matrix_);
    cv::warpPerspective(source_mask, ipm_mask, ipm_matrix, source_mask.size(),
                        cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
    const auto t2 = Clock::now(); 

    const int reference_x =
        std::clamp(configured_reference_x, 0, seg_result.width - 1);
    std::vector<uint8_t> selected_mask;
    if (make_debug) {
      selected_mask.resize(static_cast<size_t>(seg_result.width) *
                           seg_result.height);
    }
    const auto result = road_tracker_->process(
        ipm_mask.data, seg_result.width, seg_result.height, reference_x,
        make_debug ? selected_mask.data() : nullptr);
    const auto t3 = Clock::now();  

    if (!result.valid) {
      if (publish_control_deviation) {
        publish_deviation(
            msg->header,
            scoutcar_msgs::msg::RoadDeviation::STATUS_TRACKING_FAILED);
      }
    } else if (publish_control_deviation) {
      publish_deviation(msg->header,
                        scoutcar_msgs::msg::RoadDeviation::STATUS_OK,
                        &result);
    }

    // 挡板拼满整行 → 自动进入侦察（仅前视相机、仅巡航、非侦察中、非隧道内）
    if (!use_turn_camera && !detect_active_ && !in_tunnel_ &&
        mission_state_ == scoutcar_msgs::msg::CarState::DRIVING) {
      check_blocked_detect(ipm_mask.data, seg_result.width, seg_result.height);
    }

    if (make_debug) {
      publish_debug_images(*msg, seg_result.mask.data(), ipm_mask.data,
                           selected_mask.data(), result, reference_x,
                           ipm_inverse);
    }
    const auto t4 = Clock::now();
    RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Seg %s: infer=%.2f ipm=%.2f track=%.2f publish=%.2f total=%.2f ms",
    use_turn_camera ? "turn" : "front",
    std::chrono::duration<double, std::milli>(t1 - t0).count(),
    std::chrono::duration<double, std::milli>(t2 - t1).count(),
    std::chrono::duration<double, std::milli>(t3 - t2).count(),
    std::chrono::duration<double, std::milli>(t4 - t3).count(),
    std::chrono::duration<double, std::milli>(t4 - t0).count());  
  }

  int activeTurnReferenceX() const {
    const uint8_t pose =
        btp_debug_pose_ != 0 ? btp_debug_pose_ : turn_camera_pose_;
    return pose == scoutcar_msgs::msg::CarState::DIRECTION_RIGHT
               ? turn_right_reference_x_
               : turn_left_reference_x_;
  }

  void process_Detectframe(
      const sensor_msgs::msg::Image::SharedPtr msg,
      const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher,
      const char *camera_name,
      bool is_left) {
    const int width = static_cast<int>(msg->width);
    const int height = static_cast<int>(msg->height);
    const size_t row_bytes = static_cast<size_t>(width) * 3U;
    if (width <= 0 || height <= 0 || msg->encoding != "rgb8" ||
        msg->step < row_bytes ||
        msg->data.size() < static_cast<size_t>(msg->step) * height) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "检测图像格式异常: enc=%s %ux%u step=%u size=%zu",
          msg->encoding.c_str(), msg->width, msg->height, msg->step,
          msg->data.size());
      return;
    }

    const uint8_t *rgb_data = msg->data.data();
    cv::Mat contiguous_rgb;
    if (msg->step != row_bytes) {
      const cv::Mat rgb(height, width, CV_8UC3,
                        const_cast<uint8_t *>(msg->data.data()), msg->step);
      contiguous_rgb = rgb.clone();
      rgb_data = contiguous_rgb.data;
    }

    const auto detection_result =
        detection_model_->infer(rgb_data, width, height);
    if (!detection_result.success) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "%s YOLOv8推理失败: %s", camera_name,
                            detection_result.error.c_str());
    } else {
      update_recon_lock(
          is_left ? left_recon_lock_ : right_recon_lock_,
          detection_result, camera_name);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "%s YOLOv8检测到 %zu 个目标", camera_name,
                           detection_result.detections.size());
    }

    if (publisher->get_subscription_count() > 0) {
      try {
        auto debug_image =
            visualizer_.make_detection_debug(*msg, detection_result);
        publisher->publish(std::move(debug_image));
      } catch (const cv::Exception &error) {
        RCLCPP_ERROR(get_logger(), "%s绘制检测结果失败: %s", camera_name,
                     error.what());
      }
    }
  }

  void reset_recon_result() {
    left_recon_lock_ = ReconSideLock{};
    right_recon_lock_ = ReconSideLock{};
    RCLCPP_INFO(get_logger(), "侦察结果累计已清空");
  }

  void update_recon_lock(
      ReconSideLock &side,
      const scoutcar_perception::DetectionResult &detection_result,
      const char *camera_name) {
    if (side.locked) {
      return;
    }

    if (detection_result.detections.empty() ||
        detection_result.detections.front().confidence <
          kReconConfidenceThreshold) {
      side.candidate_class = -1;
      side.consecutive_frames = 0;
      return;
    }

    const auto &detection = detection_result.detections.front();
    if (side.candidate_class == detection.class_id) {
      ++side.consecutive_frames;
    } else {
      side.candidate_class = detection.class_id;
      side.consecutive_frames = 1;
    }

    if (side.consecutive_frames < kReconRequiredFrames) {
      return;
    }

    side.locked = true;
    side.result = static_cast<uint8_t>(detection.class_id + 1);
    side.confidence = detection.confidence;
    RCLCPP_INFO(
      get_logger(), "%s侦察结果锁定：类别=%u 置信度=%.3f",
      camera_name, static_cast<unsigned>(side.result), side.confidence);
  }

  void publish_recon_result() {
    scoutcar_msgs::msg::ReconResult result;
    result.header.stamp = now();
    result.left_valid = left_recon_lock_.locked;
    result.left_result = left_recon_lock_.result;
    result.left_confidence = left_recon_lock_.confidence;
    result.right_valid = right_recon_lock_.locked;
    result.right_result = right_recon_lock_.result;
    result.right_confidence = right_recon_lock_.confidence;
    pub_recon_result_->publish(result);
    RCLCPP_INFO(
      get_logger(), "侦察结果发布：左(valid=%d result=%u) 右(valid=%d result=%u)",
      result.left_valid, static_cast<unsigned>(result.left_result),
      result.right_valid, static_cast<unsigned>(result.right_result));
  }

  // 挡板从左到右把整行拼满，且从窗口下沿向上连续足够多行 → 判定接近侦察点，
  // 发布一次 START 进入 DETECTING，并arm 超时退出。只在门控通过时被调用。
  void check_blocked_detect(const uint8_t * ipm_mask, int width, int height) {
    road_tracking::BlockedRowConfig config = blocked_config_;
    config.window_bottom_y =
        static_cast<int>(blocked_window_bottom_ratio_ * height);

    const auto blocked =
        road_tracking::scan_blocked_rows(ipm_mask, width, height, config);
    const bool fired = blocked.consecutive_rows >= config.required_rows;

    if (blocked.consecutive_rows > 0) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "挡板拼满候选：连续=%d 行 y=%d 左=%d 右=%d 夹缝路面宽=%d",
          blocked.consecutive_rows, blocked.row_y, blocked.left_x,
          blocked.right_x, blocked.max_gap);
    }

    if (fired && !blocked_latch_) {
      blocked_latch_ = true;
      blocked_clear_frames_ = 0;
      scoutcar_msgs::msg::DetectTask cmd;
      cmd.status = scoutcar_msgs::msg::DetectTask::START;
      pub_detect_task_->publish(cmd);
      arm_auto_detect_exit_timer();
      RCLCPP_INFO(
          get_logger(),
          "挡板拼满整行 %d 行（y=%d 左=%d 右=%d 夹缝路面宽=%d），自动进入侦察",
          blocked.consecutive_rows, blocked.row_y, blocked.left_x,
          blocked.right_x, blocked.max_gap);
      return;
    }

    // 重臂：只能靠判据持续消失，不能靠"离开 DETECTING"——否则回到巡航时
    // 挡板还在视野里会立刻再次触发。
    if (blocked_latch_ && !fired &&
        ++blocked_clear_frames_ >= blocked_rearm_absent_frames_) {
      blocked_latch_ = false;
      blocked_clear_frames_ = 0;
      RCLCPP_INFO(get_logger(), "挡板拼满已连续 %d 帧消失，重新允许自动触发",
                  blocked_rearm_absent_frames_);
    }
  }

  void arm_auto_detect_exit_timer() {
    if (auto_detect_exit_timer_) {
      auto_detect_exit_timer_->cancel();
      auto_detect_exit_timer_.reset();
    }
    auto_detect_exit_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(blocked_auto_exit_s_)),
        [this]() {
          auto_detect_exit_timer_->cancel();
          if (mission_state_ != scoutcar_msgs::msg::CarState::DETECTING) {
            return;//mission 已自行退出（到达直行段终点 / 进入 BTP），不补发
          }
          scoutcar_msgs::msg::DetectTask cmd;
          cmd.status = scoutcar_msgs::msg::DetectTask::AUTO_END;
          pub_detect_task_->publish(cmd);
          RCLCPP_INFO(get_logger(), "自动侦察 %.1f 秒到期，发布 AUTO_END",
                      blocked_auto_exit_s_);
        });
  }

  void publish_deviation(
      const std_msgs::msg::Header &header, uint8_t status,
      const road_tracking::Result *result = nullptr) {
    scoutcar_msgs::msg::RoadDeviation deviation{};
    deviation.header = header;
    deviation.status = status;
    deviation.front_reference_x = front_reference_x_;
    deviation.turn_reference_x = activeTurnReferenceX();

    if (status == scoutcar_msgs::msg::RoadDeviation::STATUS_OK) {
      if (result == nullptr) {
        RCLCPP_ERROR(get_logger(), "STATUS_OK 必须提供 RoadTracker 结果");
        return;
      }
      deviation.deviation = static_cast<int16_t>(result->deviation);
      deviation.road_center =
          static_cast<uint16_t>(std::max(0, result->center_x));
      deviation.scan_y = static_cast<uint16_t>(std::max(0, result->y));
      deviation.left = static_cast<int16_t>(result->left);
      deviation.right = static_cast<int16_t>(result->right);
      deviation.min_width =
          static_cast<uint16_t>(std::max(0, result->min_width));
      deviation.max_width =
          static_cast<uint16_t>(std::max(0, result->max_width));
      deviation.boundary_source =
          result->used_barrier_gap ? "BARRIER_GAP" : "ROAD_MASK";
    }

    pub_deviation_->publish(deviation);
  }

  std::string seg_model_path_;
  std::string seg_label_path_;
  std::string detect_model_path_;
  std::string detect_label_path_;
  double detect_confidence_threshold_ = 0.25;
  double detect_nms_threshold_ = 0.45;
  bool enable_turning_inference_ = false;
  std::unique_ptr<road_tracking::RoadTracker> road_tracker_;
  std::unique_ptr<scoutcar_perception::SegmentationModel> segmentation_model_;
  scoutcar_perception::PerceptionVisualizer visualizer_;
  cv::Mat front_ipm_matrix_;
  cv::Mat turn_left_ipm_matrix_;
  cv::Mat turn_right_ipm_matrix_;
  cv::Mat front_ipm_inverse_;
  cv::Mat turn_left_ipm_inverse_;
  cv::Mat turn_right_ipm_inverse_;
  std::unique_ptr<scoutcar_perception::YoloV8Model> detection_model_;
  int front_reference_x_ = 320;
  int turn_left_reference_x_ = 320;
  int turn_right_reference_x_ = 320;
  uint8_t btp_debug_pose_ = 0;
  uint8_t mission_state_ = scoutcar_msgs::msg::CarState::FINISHED;
  uint8_t turn_camera_pose_ = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
  bool detect_active_ = false;
  bool in_tunnel_ = false;
  ReconSideLock left_recon_lock_;
  ReconSideLock right_recon_lock_;

  road_tracking::BlockedRowConfig blocked_config_;
  double blocked_window_bottom_ratio_ = 0.6;
  int blocked_rearm_absent_frames_ = 30;
  double blocked_auto_exit_s_ = 5.0;
  bool blocked_latch_ = false;      // 已触发，禁止重复发布 START
  int blocked_clear_frames_ = 0;    // 闭锁期间判据连续不成立的帧数
  rclcpp::TimerBase::SharedPtr auto_detect_exit_timer_;
  rclcpp::Publisher<scoutcar_msgs::msg::DetectTask>::SharedPtr pub_detect_task_;

  rclcpp::Subscription<scoutcar_msgs::msg::CarState>::SharedPtr sub_mission_state_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_btp_debug_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_front_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_turn_image_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_mask_debug_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ipm_debug_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_front_detection_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_turn_detection_;
  rclcpp::Publisher<scoutcar_msgs::msg::ReconResult>::SharedPtr pub_recon_result_;
  rclcpp::Publisher<scoutcar_msgs::msg::RoadDeviation>::SharedPtr pub_deviation_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
