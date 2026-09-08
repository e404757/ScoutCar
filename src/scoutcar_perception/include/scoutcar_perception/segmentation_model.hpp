#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace scoutcar_perception {

// 创建分割模型时需要的配置。配置由 ROS 节点读取后传进来，
// SegmentationModel 自身不读取 ROS 参数。
struct SegmentationConfig {
  std::string model_path;
  std::string label_path;
};

// 一次分割推理的输出。mask 由 vector 持有，Result 销毁时会自动释放。
struct SegmentationResult {
  bool valid = false;
  int width = 0;
  int height = 0;
  std::vector<uint8_t> mask;
};

class SegmentationModel {
public:
  explicit SegmentationModel(const SegmentationConfig & config);
  ~SegmentationModel();

  // RKNN context 不能被无意复制，因此暂时禁止复制模型对象。
  SegmentationModel(const SegmentationModel &) = delete;
  SegmentationModel & operator=(const SegmentationModel &) = delete;

  // rgb_data 只在本次调用期间借用；调用结束后不保存这个指针。
  SegmentationResult infer(
      const uint8_t * rgb_data, int width, int height);

  bool is_initialized() const;

private:
  // 后续步骤依次把模型加载和释放放进这两个函数。
  bool initialize();
  void shutdown();

  SegmentationConfig config_;
  bool initialized_ = false;

  // TODO(下一步): 在这里加入由本对象独占的 RKNN context。
};

}  // namespace scoutcar_perception
