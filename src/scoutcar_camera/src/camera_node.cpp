#include <chrono>
#include <cstring>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

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
    prefer_mipi_ = declare_parameter<bool>("prefer_mipi", true);

    pub_ = create_publisher<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::QoS(1).best_effort());

    int ret = camera_path_.empty()
      ? open_camera_auto(&cam_, image_width_, image_height_, fps_, prefer_mipi_)
      : open_camera_path(camera_path_.c_str(), &cam_, image_width_, image_height_, fps_);
    if (ret != 0) {
      RCLCPP_ERROR(get_logger(),
                   "相机打开失败（可用 --ros-args -p camera_path:=/dev/videoN 指定）");
      return;  // 节点存活但不发布
    }
    RCLCPP_INFO(get_logger(), "相机已打开 %s %dx%d",
                cam_->device_path, cam_->width, cam_->height);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / fps_),
      [this]() { on_timer(); });
  }

  ~CameraNode() override
  {
    if (cam_ != nullptr) {
      close_camera(cam_);
    }
  }

private:
  void on_timer()
  {
    image_buffer_t img;
    std::memset(&img, 0, sizeof(img));
    if (read_camera_frame(cam_, &img) != 0) {
      return;  // 丢帧，跳过
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

  std::string camera_path_;
  int fps_;
  int image_width_;
  int image_height_;
  std::string image_topic_;
  bool prefer_mipi_;
  bool first_frame_published_ = false;
  camera_context_t * cam_ = nullptr;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
