#include "scoutcar_perception/segmentation_model.hpp"

#include "yolov5_seg.h"

#include <cstdlib>
#include <limits>
#include <memory>

namespace scoutcar_perception {

struct SegmentationModel::Impl {
  seg_rknn_app_context_t context{};
};

SegmentationModel::SegmentationModel(const SegmentationConfig &config)
    : config_(config), impl_(std::make_unique<Impl>()) {
  initialized_ = initialize();
}

SegmentationModel::~SegmentationModel() { shutdown(); }

bool SegmentationModel::initialize() {
  if (config_.model_path.empty()) {
    initialization_error_ = "model_path 不能为空";
    return false;
  }
  if (config_.label_path.empty()) {
    initialization_error_ = "label_path 不能为空";
    return false;
  }

  if (seg_init_post_process(config_.label_path.c_str()) != 0) {
    initialization_error_ = "分割标签和后处理初始化失败";
    return false;
  }
  post_process_initialized_ = true;

  if (init_yolov5_seg_model(config_.model_path.c_str(), &impl_->context) !=
      0) {
    initialization_error_ = "RKNN 分割模型加载失败";
    shutdown();
    return false;
  }
  model_initialized_ = true;
  initialization_error_.clear();
  return true;
}

void SegmentationModel::shutdown() {
  if (model_initialized_) {
    release_yolov5_seg_model(&impl_->context);
    model_initialized_ = false;
  }
  if (post_process_initialized_) {
    seg_deinit_post_process();
    post_process_initialized_ = false;
  }
  initialized_ = false;
}

SegmentationResult SegmentationModel::infer(const uint8_t *rgb_data, int width,
                                            int height) {
  SegmentationResult result;
  result.width = width;
  result.height = height;

  if (!initialized_) {
    result.error = "分割模型尚未成功初始化";
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

  seg_object_detect_result_list vendor_result{};
  const int ret =
      inference_yolov5_seg_model(&impl_->context, &image, &vendor_result);

  std::unique_ptr<uint8_t, decltype(&std::free)> mask_guard(
      vendor_result.results_seg[0].seg_mask, &std::free);

  if (ret != 0) {
    result.error = "RKNN 分割推理失败，错误码=" + std::to_string(ret);
    return result;
  }

  if (vendor_result.count == 0) {
    return result;
  }

  if (!mask_guard) {
    result.error = "分割后处理没有返回mask";
    return result;
  }

  result.mask.assign(mask_guard.get(), mask_guard.get() + pixel_count);
  result.valid = true;
  return result;
}

bool SegmentationModel::is_initialized() const { return initialized_; }

const std::string &SegmentationModel::initialization_error() const {
  return initialization_error_;
}

} // namespace scoutcar_perception
