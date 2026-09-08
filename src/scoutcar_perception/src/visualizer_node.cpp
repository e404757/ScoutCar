#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>

#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scoutcar_msgs/msg/road_deviation.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#include "scoutcar_perception/perception_visualizer.hpp"

class VisualizerNode : public rclcpp::Node {
public:
  VisualizerNode() : Node("visualizer_node") {
    const auto mipi_source = declare_parameter<std::vector<double>>(
        "ipm.mipi_source_points",
        {276.0, 190.0, 404.0, 190.0, 163.0, 440.0, 599.0, 440.0});
    const auto usb_source = declare_parameter<std::vector<double>>(
        "ipm.usb_source_points",
        {239.0, 120.0, 411.0, 120.0, 50.0, 420.0, 511.0, 420.0});
    const auto destination = declare_parameter<std::vector<double>>(
        "ipm.destination_points",
        {180.0, 120.0, 460.0, 120.0, 180.0, 470.0, 460.0, 470.0});

    if (mipi_source.size() != 8 || usb_source.size() != 8 ||
        destination.size() != 8) {
      throw std::runtime_error(
          "两路 ipm source_points 和 destination_points 必须各有 8 个数");
    }

    std::vector<cv::Point2f> destination_points;
    for (size_t i = 0; i < 8; i += 2) {
      destination_points.emplace_back(destination[i], destination[i + 1]);
    }

    const auto make_inverse = [&destination_points](
        const std::vector<double> & source) {
      std::vector<cv::Point2f> source_points;
      for (size_t i = 0; i < 8; i += 2) {
        source_points.emplace_back(source[i], source[i + 1]);
      }
      return cv::getPerspectiveTransform(
          source_points, destination_points).inv();
    };

    mipi_ipm_inverse_ = make_inverse(mipi_source);
    usb_ipm_inverse_ = make_inverse(usb_source);

    pub_debug_ = create_publisher<sensor_msgs::msg::Image>(
        "perception/debug_image", 5);
    pub_ipm_debug_ = create_publisher<sensor_msgs::msg::Image>(
        "perception/ipm_debug_image", 5);

    const auto status_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sub_cam_status_ = create_subscription<std_msgs::msg::Bool>(
        "mission/cam_status", status_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          cam_turned_ = msg->data;
        });
    sub_deviation_enable_ = create_subscription<std_msgs::msg::Bool>(
        "mission/deviation_enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          deviation_enabled_ = msg->data;
        });

    source_sub_.subscribe(this, "perception/source_image",
                          rmw_qos_profile_default);
    seg_sub_.subscribe(this, "perception/seg_mask",
                       rmw_qos_profile_default);
    ipm_sub_.subscribe(this, "perception/ipm_mask",
                       rmw_qos_profile_default);
    processed_sub_.subscribe(this, "perception/processed_mask",
                             rmw_qos_profile_default);
    deviation_sub_.subscribe(this, "perception/road_boundary",
                             rmw_qos_profile_default);

    sync_ = std::make_shared<Synchronizer>(
        SyncPolicy(5), source_sub_, seg_sub_, ipm_sub_, processed_sub_,
        deviation_sub_);
    sync_->registerCallback(std::bind(
        &VisualizerNode::on_frame, this,
        std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4,
        std::placeholders::_5));
  }

private:
  using Image = sensor_msgs::msg::Image;
  using Deviation = scoutcar_msgs::msg::RoadDeviation;
  using SyncPolicy = message_filters::sync_policies::ExactTime<
      Image, Image, Image, Image, Deviation>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void on_frame(const Image::ConstSharedPtr & source,
                const Image::ConstSharedPtr & seg_mask,
                const Image::ConstSharedPtr & ipm_mask,
                const Image::ConstSharedPtr & processed_mask,
                const Deviation::ConstSharedPtr & deviation) {
    const size_t pixel_count =
        static_cast<size_t>(source->width) * source->height;
    const bool dimensions_match =
        seg_mask->width == source->width &&
        seg_mask->height == source->height &&
        ipm_mask->width == source->width &&
        ipm_mask->height == source->height &&
        processed_mask->width == source->width &&
        processed_mask->height == source->height;
    if (!dimensions_match || source->encoding != "rgb8" ||
        seg_mask->data.size() < pixel_count ||
        ipm_mask->data.size() < pixel_count ||
        processed_mask->data.size() < pixel_count) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "可视化输入图像格式或尺寸不一致");
      return;
    }

    update_fps();

    road_tracking::Result result;
    result.valid = deviation->valid;
    result.deviation = deviation->deviation;
    result.center_x = static_cast<int>(deviation->road_center);
    result.y = static_cast<int>(deviation->scan_y);
    result.left = deviation->left;
    result.right = deviation->right;
    result.width = static_cast<int>(deviation->width);
    result.used_barrier_gap =
        deviation->boundary_source == "BARRIER_GAP";

    const cv::Mat & ipm_inverse =
        cam_turned_ ? usb_ipm_inverse_ : mipi_ipm_inverse_;
    pub_debug_->publish(visualizer_.make_source_debug(
        *source, seg_mask->data.data(), processed_mask->data.data(),
        result, ipm_inverse, fps_, deviation_enabled_));
    pub_ipm_debug_->publish(visualizer_.make_ipm_debug(
        *source, ipm_mask->data.data(), processed_mask->data.data(), result));
  }

  void update_fps() {
    ++frame_count_;
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
        std::chrono::duration<double>(now - fps_last_).count();
    if (elapsed >= 1.0) {
      fps_ = frame_count_ / elapsed;
      frame_count_ = 0;
      fps_last_ = now;
    }
  }

  scoutcar_perception::PerceptionVisualizer visualizer_;
  cv::Mat mipi_ipm_inverse_;
  cv::Mat usb_ipm_inverse_;
  bool cam_turned_ = false;
  bool deviation_enabled_ = true;
  int frame_count_ = 0;
  double fps_ = 0.0;
  std::chrono::steady_clock::time_point fps_last_ =
      std::chrono::steady_clock::now();

  message_filters::Subscriber<Image> source_sub_;
  message_filters::Subscriber<Image> seg_sub_;
  message_filters::Subscriber<Image> ipm_sub_;
  message_filters::Subscriber<Image> processed_sub_;
  message_filters::Subscriber<Deviation> deviation_sub_;
  std::shared_ptr<Synchronizer> sync_;

  rclcpp::Publisher<Image>::SharedPtr pub_debug_;
  rclcpp::Publisher<Image>::SharedPtr pub_ipm_debug_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cam_status_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_deviation_enable_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
