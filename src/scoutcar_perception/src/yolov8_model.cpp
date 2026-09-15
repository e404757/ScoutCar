#include "scoutcar_perception/yolov8_model.hpp"

#include "yolov8.h"

#include <algorithm>
#include <limits>
#include <utility>

namespace scoutcar_perception {

struct YoloV8Model::Impl {
  detect_rknn_app_context_t context{};
};

YoloV8Model::YoloV8Model(const YoloV8Config &config)
    : config_(config), impl_(std::make_unique<Impl>()) {
  initialized_ = initialize();
}

YoloV8Model::~YoloV8Model() { shutdown(); }

bool YoloV8Model::initialize() {
  if (config_.model_path.empty()) {
    initialization_error_ = "model_path 不能为空";
    return false;
  }
  if (config_.label_path.empty()) {
    initialization_error_ = "label_path 不能为空";
    return false;
  }

  if (detect_init_post_process(config_.label_path.c_str()) != 0) {
    initialization_error_ = "YOLOv8 标签或后处理初始化失败（需要20行标签）";
    return false;
  }
  post_process_initialized_ = true;

  if (init_yolov8_model(config_.model_path.c_str(), &impl_->context) != 0) {
    initialization_error_ = "YOLOv8 RKNN 模型加载失败";
    shutdown();
    return false;
  }
  model_initialized_ = true;
  initialization_error_.clear();
  return true;
}

void YoloV8Model::shutdown() {
  if (model_initialized_) {
    release_yolov8_model(&impl_->context);
    model_initialized_ = false;
  }
  if (post_process_initialized_) {
    detect_deinit_post_process();
    post_process_initialized_ = false;
  }
  initialized_ = false;
}

DetectionResult YoloV8Model::infer(const uint8_t *rgb_data, int width,
                                   int height) {
  std::lock_guard<std::mutex> lock(inference_mutex_);

  DetectionResult result;
  result.image_width = width;
  result.image_height = height;

  if (!initialized_) {
    result.error = "YOLOv8 模型尚未成功初始化";
    return result;
  }
  if (rgb_data == nullptr) {
    result.error = "RGB 图像数据为空";
    return result;
  }
  if (width <= 0 || height <= 0) {
    result.error = "RGB 图像尺寸无效";
    return result;
  }

  const size_t pixel_count =
      static_cast<size_t>(width) * static_cast<size_t>(height);
  if (pixel_count > static_cast<size_t>(std::numeric_limits<int>::max()) / 3U) {
    result.error = "RGB 图像尺寸过大";
    return result;
  }

  image_buffer_t image{};
  image.width = width;
  image.height = height;
  image.format = IMAGE_FORMAT_RGB888;
  image.size = static_cast<int>(pixel_count * 3U);
  image.virt_addr = const_cast<uint8_t *>(rgb_data);

  detect_object_detect_result_list vendor_result{};
  const int ret =
      inference_yolov8_model(&impl_->context, &image,
                             config_.confidence_threshold,
                             config_.nms_threshold, &vendor_result);
  if (ret != 0) {
    result.error = "YOLOv8 RKNN 推理失败，错误码=" + std::to_string(ret);
    return result;
  }

  const int detection_count =
      std::clamp(vendor_result.count, 0, static_cast<int>(OBJ_NUMB_MAX_SIZE));
  result.detections.reserve(static_cast<size_t>(detection_count));
  for (int i = 0; i < detection_count; ++i) {
    const detect_object_detect_result &vendor_detection = vendor_result.results[i];
    Detection detection;
    detection.class_id = vendor_detection.cls_id;
    const char *name = detect_cls_to_name(vendor_detection.cls_id);
    detection.class_name = name != nullptr ? name : "unknown";
    detection.confidence = vendor_detection.prop;
    detection.left = vendor_detection.box.left;
    detection.top = vendor_detection.box.top;
    detection.right = vendor_detection.box.right;
    detection.bottom = vendor_detection.box.bottom;
    result.detections.push_back(std::move(detection));
  }

  result.success = true;
  return result;
}

bool YoloV8Model::is_initialized() const { return initialized_; }

const std::string &YoloV8Model::initialization_error() const {
  return initialization_error_;
}

} // namespace scoutcar_perception
