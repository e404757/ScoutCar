#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace scoutcar_perception {

// 当前模型和 vendor 后处理均为 20 类，因此标签文件必须有 20 行。
struct YoloV8Config {
  std::string model_path;
  std::string label_path;
  float confidence_threshold = 0.25F;
  float nms_threshold = 0.45F;
};

// 一个检测框。坐标已经由 vendor 后处理映射回输入图像坐标系。
struct Detection {
  int class_id = -1;
  std::string class_name;
  float confidence = 0.0F;
  int left = 0;
  int top = 0;
  int right = 0;
  int bottom = 0;
};

// success 表示本次推理是否正常完成；detections 为空只表示没有检测到目标。
struct DetectionResult {
  bool success = false;
  int image_width = 0;
  int image_height = 0;
  std::vector<Detection> detections;
  std::string error;
};

class YoloV8Model {
public:
  explicit YoloV8Model(const YoloV8Config &config);
  ~YoloV8Model();

  // RKNN context 和后处理标签具有唯一所有权，禁止复制和移动。
  YoloV8Model(const YoloV8Model &) = delete;
  YoloV8Model &operator=(const YoloV8Model &) = delete;
  YoloV8Model(YoloV8Model &&) = delete;
  YoloV8Model &operator=(YoloV8Model &&) = delete;

  // rgb_data 必须指向连续的 width * height * 3 字节 RGB888 数据。
  // 指针只在本次同步调用期间借用，函数返回后不会保存。
  DetectionResult infer(const uint8_t *rgb_data, int width, int height);

  bool is_initialized() const;
  const std::string &initialization_error() const;

private:
  struct Impl;

  bool initialize();
  void shutdown();

  YoloV8Config config_;
  bool initialized_ = false;
  bool post_process_initialized_ = false;
  bool model_initialized_ = false;
  std::string initialization_error_;
  std::unique_ptr<Impl> impl_;
  std::mutex inference_mutex_;
};

} // namespace scoutcar_perception
