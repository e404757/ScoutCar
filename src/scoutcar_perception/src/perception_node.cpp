#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <scoutcar_msgs/msg/road_deviation.hpp>

#include "image_utils.h"
#include "scoutcar_perception/road_tracker.hpp"
#include "yolov5_seg.h"

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode() : Node("perception_node")
  {
    const auto package_share =
      ament_index_cpp::get_package_share_directory("scoutcar_perception");
    model_path_ = declare_parameter<std::string>(
      "model_path", package_share + "/models/yolov5seg_V1.0/seg_V1.0.rknn");
    label_path_ = declare_parameter<std::string>(
      "label_path", package_share + "/resources/detect_label.txt");
    usb_image_topic_ = declare_parameter<std::string>(
      "usb_image_topic", "/camera/usb/image_raw");
    mipi_image_topic_ = declare_parameter<std::string>(
      "mipi_image_topic", "/camera/mipi/image_raw");

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
    road_tracker_ = std::make_unique<road_tracking::RoadTracker>(tracking_config);

    const auto mipi_source = declare_parameter<std::vector<double>>(
      "ipm.mipi_source_points", {276.0, 190.0, 404.0, 190.0, 163.0, 440.0, 599.0, 440.0});
    const auto usb_source = declare_parameter<std::vector<double>>(
      "ipm.usb_source_points", {239.0, 120.0, 411.0, 120.0, 50.0, 420.0, 511.0, 420.0});
    const auto destination = declare_parameter<std::vector<double>>(
      "ipm.destination_points", {180.0, 120.0, 460.0, 120.0, 180.0, 470.0, 460.0, 470.0});
    if (mipi_source.size() != 8 || usb_source.size() != 8 || destination.size() != 8) {
      throw std::runtime_error("两路 ipm source_points 和 destination_points 必须各有 8 个数");
    }
    std::vector<cv::Point2f> destination_points;
    for (size_t i = 0; i < 8; i += 2) {
      destination_points.emplace_back(destination[i], destination[i + 1]);
    }
    const auto make_ipm = [&destination_points](const std::vector<double> & source) {
      std::vector<cv::Point2f> source_points;
      for (size_t i = 0; i < 8; i += 2) {
        source_points.emplace_back(source[i], source[i + 1]);
      }
      return cv::getPerspectiveTransform(source_points, destination_points);
    };
    mipi_ipm_matrix_ = make_ipm(mipi_source);
    usb_ipm_matrix_ = make_ipm(usb_source);
    pub_seg_ = create_publisher<sensor_msgs::msg::Image>("perception/seg_mask", 10);
    pub_source_ = create_publisher<sensor_msgs::msg::Image>("perception/source_image", 5);
    pub_ipm_mask_ = create_publisher<sensor_msgs::msg::Image>("perception/ipm_mask", 5);
    pub_processed_mask_ = create_publisher<sensor_msgs::msg::Image>(
      "perception/processed_mask", 5);
    pub_deviation_ = create_publisher<scoutcar_msgs::msg::RoadDeviation>(
      "perception/road_boundary", 10);
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sub_cam_status_ = create_subscription<std_msgs::msg::Bool>("mission/cam_status", status_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        cam_turned_ = msg->data;
      });
    sub_usb_image_ = create_subscription<sensor_msgs::msg::Image>(
      usb_image_topic_, rclcpp::QoS(1).best_effort(),   // 与 camera_node 的 QoS 匹配；depth=1 只留最新帧
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {  
        if(cam_turned_){
          process_frame(msg); 
        }});
    sub_mipi_image_ = create_subscription<sensor_msgs::msg::Image>(
      mipi_image_topic_, rclcpp::QoS(1).best_effort(),   // 与 camera_node 的 QoS 匹配；depth=1 只留最新帧
    [this](const sensor_msgs::msg::Image::SharedPtr msg) { if(!cam_turned_){
          process_frame(msg); 
        }});
    if (init_post_process(label_path_.c_str()) != 0) {
      RCLCPP_ERROR(get_logger(), "类别标签加载失败: %s", label_path_.c_str());
      return;
    }
    post_process_ok_ = true;
    if (init_yolov5_seg_model(model_path_.c_str(), &rknn_ctx_) != 0) {
      RCLCPP_ERROR(get_logger(), "模型加载失败: %s", model_path_.c_str());
      model_ok_ = false;
    } else {
      model_ok_ = true;
      RCLCPP_INFO(get_logger(), "模型加载成功");
    }
  }

  ~PerceptionNode() override
  {
    if (model_ok_) {
      release_yolov5_seg_model(&rknn_ctx_);
    }
    if (post_process_ok_) {
      deinit_post_process();
    }
  }

private:
  void process_frame(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!model_ok_) {
      return;
    }
    
    const size_t bytes = static_cast<size_t>(msg->width) * msg->height * 3;
    if (msg->encoding != "rgb8" || msg->data.size() < bytes) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "图像格式异常: enc=%s %ux%u",
                           msg->encoding.c_str(), msg->width, msg->height);
      return;
    }
    uint8_t * frame = static_cast<uint8_t *>(std::malloc(bytes));
    if (frame == nullptr) {
      return;
    }
    std::memcpy(frame, msg->data.data(), bytes);

    image_buffer_t img{};
    img.width = static_cast<int>(msg->width);
    img.height = static_cast<int>(msg->height);
    img.format = IMAGE_FORMAT_RGB888;
    img.virt_addr = frame;

    // 2) 推理（CPU 预处理）
    object_detect_result_list od{};
    const int ret = inference_yolov5_seg_model_cpu(&rknn_ctx_, &img, &od);
    std::free(frame);
    if (ret != 0) {
      return;
    }
    if (od.count < 1) {
      publish_invalid_boundary(msg->header);
      sensor_msgs::msg::Image empty_mask;
      empty_mask.header = msg->header;
      empty_mask.width = msg->width;
      empty_mask.height = msg->height;
      empty_mask.encoding = "mono8";
      empty_mask.step = msg->width;
      empty_mask.data.assign(
          static_cast<size_t>(msg->width) * msg->height, 0);
      pub_source_->publish(*msg);
      pub_seg_->publish(empty_mask);
      if (visualizer_connected()) {
        pub_ipm_mask_->publish(empty_mask);
        pub_processed_mask_->publish(empty_mask);
      }
      return;
    }

    // 3) 掩膜 → sensor_msgs/Image（mono8：0背景/1路面/2挡板）
    uint8_t * seg_mask = od.results_seg[0].seg_mask;
    auto out = std::make_shared<sensor_msgs::msg::Image>();
    out->header = msg->header;              // 继承相机帧时间戳
    out->header.frame_id = "camera";
    out->width = img.width;
    out->height = img.height;
    out->encoding = "mono8";
    out->step = static_cast<uint32_t>(img.width);
    out->data.assign(seg_mask, seg_mask + static_cast<size_t>(img.width) * img.height);

    // 4) 原始分割掩膜先变换到俯视图，再从车前方向上筛选主通道。
    cv::Mat source_mask(img.height, img.width, CV_8UC1, seg_mask);
    cv::Mat ipm_mask;
    const cv::Mat & ipm_matrix = cam_turned_ ? usb_ipm_matrix_ : mipi_ipm_matrix_;
    cv::warpPerspective(source_mask, ipm_mask, ipm_matrix, source_mask.size(),
                        cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
    const bool make_visualization_data = visualizer_connected();
    std::vector<uint8_t> processed_mask;
    if (make_visualization_data) {
      processed_mask.assign(
          static_cast<size_t>(img.width) * img.height, 0);
    }
    const auto tracked = road_tracker_->process(
      ipm_mask.data, img.width, img.height, img.width / 2,
      make_visualization_data ? processed_mask.data() : nullptr);
    const auto & result = tracked;
    
    scoutcar_msgs::msg::RoadDeviation deviation;
    deviation.header = msg->header;
    deviation.valid = result.valid;
    deviation.deviation = deviation.valid ? static_cast<int16_t>(result.deviation) : -999;
    deviation.road_center = static_cast<uint16_t>(result.center_x);
    deviation.scan_y = static_cast<uint16_t>(result.y);
    deviation.left = static_cast<int16_t>(result.left);
    deviation.right = static_cast<int16_t>(result.right);
    deviation.width = static_cast<uint16_t>(std::max(0, result.width));
    deviation.boundary_source = result.used_barrier_gap ? "BARRIER_GAP" : "ROAD_MASK";
     
    pub_deviation_->publish(deviation);

    
    pub_source_->publish(*msg);
    if (make_visualization_data) {
      sensor_msgs::msg::Image ipm_message;
      ipm_message.header = msg->header;
      ipm_message.width = static_cast<uint32_t>(img.width);
      ipm_message.height = static_cast<uint32_t>(img.height);
      ipm_message.encoding = "mono8";
      ipm_message.step = static_cast<uint32_t>(img.width);
      ipm_message.data.assign(
          ipm_mask.data,
          ipm_mask.data + static_cast<size_t>(img.width) * img.height);

      sensor_msgs::msg::Image processed_message = ipm_message;
      processed_message.data = processed_mask;
      pub_ipm_mask_->publish(ipm_message);
      pub_processed_mask_->publish(processed_message);
    }

    std::free(seg_mask);                     // inference 内部分配，用完释放

    // 5) 发布掩膜（移动，避免整帧拷贝）
    pub_seg_->publish(std::move(*out));
  }
  void publish_invalid_boundary(const std_msgs::msg::Header & header)
  {
    scoutcar_msgs::msg::RoadDeviation boundary;
    boundary.header = header;
    boundary.valid = false;
    pub_deviation_->publish(boundary);
  }

  bool visualizer_connected() const
  {
    return pub_ipm_mask_->get_subscription_count() > 0 ||
           pub_processed_mask_->get_subscription_count() > 0;
  }

  std::string model_path_;
  std::string label_path_;
  std::string usb_image_topic_;
  std::string mipi_image_topic_;
  bool model_ok_ = false;
  bool post_process_ok_ = false;
  rknn_app_context_t rknn_ctx_{};   // 模型上下文（yolov5_seg.h）
  std::unique_ptr<road_tracking::RoadTracker> road_tracker_;
  cv::Mat mipi_ipm_matrix_;
  cv::Mat usb_ipm_matrix_;
  bool cam_turned_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_seg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_source_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ipm_mask_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_processed_mask_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cam_status_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_usb_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_mipi_image_;
  rclcpp::Publisher<scoutcar_msgs::msg::RoadDeviation>::SharedPtr pub_deviation_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
