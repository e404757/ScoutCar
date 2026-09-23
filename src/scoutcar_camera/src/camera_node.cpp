#include <chrono>
#include <cstring>
#include <string>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#include "camera.h"
#include "image_utils.h"

class CameraNode : public rclcpp::Node
{
public:
  CameraNode() : Node("camera_node")
  {
    camera_path_ = declare_parameter<std::string>("camera_path", "");  
    fps_ = declare_parameter<int>("fps", 30);
    image_width_ = declare_parameter<int>("image_width", 640);
    image_height_ = declare_parameter<int>("image_height", 480);
    image_topic_ = declare_parameter<std::string>("image_topic", "camera/image_raw");
    health_topic_ = declare_parameter<std::string>("health_topic", "camera/healthy");
    prefer_mipi_ = declare_parameter<bool>("prefer_mipi", true);
    rotate_180_ = declare_parameter<bool>("rotate_180", false);

    pub_ = create_publisher<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::QoS(1).best_effort());
    health_pub_ = create_publisher<std_msgs::msg::Bool>(
      health_topic_, rclcpp::QoS(1).reliable().transient_local());

    try_open_camera();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / fps_),
      [this]() { on_timer(); });
    retry_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        if (cam_ == nullptr) {
          try_open_camera();
        }
      });
  }

  ~CameraNode() override
  {
    if (cam_ != nullptr) {
      close_camera(cam_);
    }
  }

private:
  void try_open_camera()
  {
    camera_context_t * opened = nullptr;
    const int ret = camera_path_.empty()
      ? open_camera_auto(&opened, image_width_, image_height_, fps_, prefer_mipi_)
      : open_camera_path(camera_path_.c_str(), &opened,
                         image_width_, image_height_, fps_);
    if (ret != 0) {
      RCLCPP_ERROR(get_logger(),
                   "相机打开失败，1 秒后重试: %s",
                   camera_path_.empty() ? "自动搜索" : camera_path_.c_str());
      publish_health(false);
      return;
    }

    cam_ = opened;
    camera_open_time_ = std::chrono::steady_clock::now();
    first_frame_received_ = false;
    RCLCPP_INFO(get_logger(), "相机已打开 %s %dx%d，等待首帧",
                cam_->device_path, cam_->width, cam_->height);
  }

  void on_timer()
  {
    if (cam_ == nullptr) {
      return;
    }

    image_buffer_t img;
    std::memset(&img, 0, sizeof(img));
    if (read_camera_frame(cam_, &img) != 0) {
      const auto now = std::chrono::steady_clock::now();
      const auto no_frame_time = now -
        (first_frame_received_ ? last_frame_time_ : camera_open_time_);

      if (no_frame_time >= std::chrono::seconds(5)) {
        publish_health(false);
      }

      const auto reopen_timeout = first_frame_received_
        ? std::chrono::seconds(5)
        : std::chrono::seconds(10);
      if (no_frame_time >= reopen_timeout) {
        RCLCPP_ERROR(get_logger(), "相机连续无画面，关闭设备并重新打开: %s",
                     camera_path_.empty() ? "自动搜索" : camera_path_.c_str());
        close_camera(cam_);
        cam_ = nullptr;
      }
      return;
    }

    last_frame_time_ = std::chrono::steady_clock::now();
    first_frame_received_ = true;
    publish_health(true);

    if (rotate_180_) {
      cv::Mat frame(img.height, img.width, CV_8UC3, img.virt_addr);
      cv::flip(frame, frame, -1);
    }

    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->header.stamp = now();
    msg->header.frame_id = "camera";
    msg->width = img.width;
    msg->height = img.height;
    msg->encoding = "rgb8";
    msg->step = static_cast<uint32_t>(img.width) * 3;
    const size_t bytes = static_cast<size_t>(img.width) * img.height * 3;
    msg->data.assign(img.virt_addr, img.virt_addr + bytes);
    free(img.virt_addr);  // read_camera_frame 内部分配

    pub_->publish(std::move(*msg));   // rclcpp 移动发布，避免整帧拷贝
    if (!first_frame_published_) {
      RCLCPP_INFO(get_logger(), "已开始发布 %s（%dx%d）",
                  image_topic_.c_str(), img.width, img.height);
      first_frame_published_ = true;
    }
  }

  void publish_health(bool healthy)
  {
    if (health_published_ && healthy == healthy_) {
      return;
    }
    healthy_ = healthy;
    health_published_ = true;
    std_msgs::msg::Bool msg;
    msg.data = healthy;
    health_pub_->publish(msg);
    if (healthy) {
      RCLCPP_INFO(get_logger(), "相机画面恢复: %s", image_topic_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "相机无有效画面: %s", image_topic_.c_str());
    }
  }

  std::string camera_path_;
  int fps_;
  int image_width_;
  int image_height_;
  std::string image_topic_;
  std::string health_topic_;
  bool prefer_mipi_;
  bool rotate_180_;
  bool first_frame_published_ = false;
  bool healthy_ = false;
  bool health_published_ = false;
  bool first_frame_received_ = false;
  std::chrono::steady_clock::time_point camera_open_time_{};
  std::chrono::steady_clock::time_point last_frame_time_{};
  camera_context_t * cam_ = nullptr;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  //make_shared<CameraNode>() 会实例化一个 CameraNode 对象，并返回指向该对象的共享智能指针
  //随后将该指针传给 spin()，使 ROS 2 持续运行这个节点并处理它的回调和事件。
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
