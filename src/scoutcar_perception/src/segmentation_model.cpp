#include "scoutcar_perception/segmentation_model.hpp"

namespace scoutcar_perception {

SegmentationModel::SegmentationModel(const SegmentationConfig & config)
    : config_(config) {
  // TODO(第2步): 调用 initialize()，完成模型和后处理资源的初始化。
}

SegmentationModel::~SegmentationModel() {
  // TODO(第3步): 调用 shutdown()，释放本对象持有的模型资源。
}

bool SegmentationModel::initialize() {
  
  return false;
}

void SegmentationModel::shutdown() {
  // TODO(第3步): 只释放已经成功初始化的资源。
}

SegmentationResult SegmentationModel::infer(
    const uint8_t * rgb_data, int width, int height) {
  SegmentationResult result;

  // TODO(第4步): 检查输入和 initialized_，执行推理并填充 result。
  (void)rgb_data;
  (void)width;
  (void)height;
  return result;
}

bool SegmentationModel::is_initialized() const {
  return initialized_;
}

}  // namespace scoutcar_perception
