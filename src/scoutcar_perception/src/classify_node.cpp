// mobilnetv2分类模型
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "image_utils.h"
#include "mobilenet.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

class ClassifierNode : public rclcpp::Node {
public:
  ClassifierNode() : Node("mobilenet_node") {
    const auto package_share =
        ament_index_cpp::get_package_share_directory("scoutcar_perception");
    model_path_ = declare_parameter<std::string>(
        "model_path", package_share + "/models/mobilenet/V1.0/mobilenet_V1.0.rknn");
    label_path_ = declare_parameter<std::string>(
        "label_path", package_share + "/models/mobilenet/V1.0/labels.txt");
    front_image_topic_ = declare_parameter<std::string>(
        "front_image_topic", "/camera/front/image_raw");

    if (!load_labels()) {
      RCLCPP_ERROR(get_logger(), "标签加载失败，节点不会执行推理");
      return;
    }

    pub_classification_ =
        create_publisher<std_msgs::msg::String>("mobilenet/classification", 10);
    sub_front_image_ = create_subscription<sensor_msgs::msg::Image>(
        front_image_topic_, rclcpp::QoS(1).best_effort(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          process_frame(msg);
        });

    if (init_mobilenet_model(model_path_.c_str(), &rknn_app_ctx_) != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize mobilenet model: %s",
                   model_path_.c_str());
      model_ok_ = false;
    } else {
      model_ok_ = true;
      RCLCPP_INFO(get_logger(), "Mobilenet model initialized successfully");
    }

    if (model_ok_ && labels_.size() != rknn_app_ctx_.output_attrs[0].n_elems) {
      RCLCPP_ERROR(get_logger(), "标签数(%zu)与模型输出类别数(%u)不一致",
                   labels_.size(), rknn_app_ctx_.output_attrs[0].n_elems);
      release_mobilenet_model(&rknn_app_ctx_);
      model_ok_ = false;
    }
  }
  ~ClassifierNode() override {
    if (model_ok_) {
      release_mobilenet_model(&rknn_app_ctx_);
    }
  }

private:
  bool load_labels() {
    std::ifstream file(label_path_);

    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "无法打开标签文件: %s", label_path_.c_str());
      return false;
    }

    labels_.clear();

    std::string line;
    while (std::getline(file, line)) {
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (!line.empty()) {
        labels_.push_back(line);
      }
    }

    if (labels_.empty()) {
      RCLCPP_ERROR(get_logger(), "标签文件为空: %s", label_path_.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "已加载 %zu 个类别标签", labels_.size());
    return true;
  }
  void process_frame(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!model_ok_) {
      return;
    }
    const size_t bytes = static_cast<size_t>(msg->width) * msg->height * 3;
    if (msg->encoding != "rgb8" || msg->data.size() < bytes) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "图像格式异常: enc=%s %ux%u", msg->encoding.c_str(),
                           msg->width, msg->height);
      return;
    }
    uint8_t *frame = static_cast<uint8_t *>(std::malloc(bytes));
    if (frame == nullptr) {
      return;
    }
    std::memcpy(frame, msg->data.data(), bytes);

    image_buffer_t img{};
    img.width = static_cast<int>(msg->width);
    img.height = static_cast<int>(msg->height);
    img.format = IMAGE_FORMAT_RGB888;
    img.virt_addr = frame;
    img.size = static_cast<int>(bytes);

    mobilenet_result result{};
    const int ret = inference_mobilenet_model(&rknn_app_ctx_, &img, &result, 1);
    std::free(frame);
    frame = nullptr;
    if (ret != 0) {
      RCLCPP_ERROR(get_logger(), "Mobilenet inference failed: ret=%d", ret);
      return;
    }

    const std::string &label = labels_[result.cls];
    if (result.score > 0.98) {
      RCLCPP_INFO(get_logger(), "class=%s score=%.3f", label.c_str(),
                  result.score);
    }

    std_msgs::msg::String output;
    output.data = label;
    pub_classification_->publish(output);
  }
  rknn_app_context_t rknn_app_ctx_{};
  bool model_ok_ = false;
  std::string model_path_;
  std::string label_path_;
  std::string front_image_topic_;
  std::vector<std::string> labels_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_classification_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_front_image_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClassifierNode>());
  rclcpp::shutdown();
  return 0;
}
