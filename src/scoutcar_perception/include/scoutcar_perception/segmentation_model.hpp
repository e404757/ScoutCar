#pragma once

#include <cstdint>
#include <memory>
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
  // true 表示本帧识别到了有效分割目标，可以继续执行IPM和RoadTracker。
  bool valid = false;
  int width = 0;
  int height = 0;
  std::vector<uint8_t> mask;
  std::string error;
};

class SegmentationModel {
public:
  explicit SegmentationModel(const SegmentationConfig &config);
  ~SegmentationModel();

  // RKNN context 不能被无意复制，因此暂时禁止复制模型对象。
  SegmentationModel(const SegmentationModel &) = delete;
  SegmentationModel &operator=(const SegmentationModel &) = delete;
  SegmentationModel(SegmentationModel &&) = delete;
  SegmentationModel &operator=(SegmentationModel &&) = delete;

  // rgb_data 只在本次调用期间借用；调用结束后不保存这个指针。
  SegmentationResult infer(const uint8_t *rgb_data, int width, int height);

  bool is_initialized() const;
  const std::string &initialization_error() const;

private:
  struct Impl;

  // 模型资源只允许本类管理，节点不能直接调用加载和释放。
  bool initialize();
  void shutdown();

  SegmentationConfig config_;
  bool initialized_ = false;
  bool post_process_initialized_ = false;
  bool model_initialized_ = false;
  std::string initialization_error_;
  std::unique_ptr<Impl> impl_;
};

} // namespace scoutcar_perception
