#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "video_writer.h"

namespace scoutcar_web {

// 双文件同步片段录制器：一段录像 = 原始帧 .avi + 掩膜叠加帧 _mask.avi。
//
// 需求背景（用户确认 2026-08-28）：录像用于采集 YOLO 训练数据，回放时要看
// 带掩膜的叠加画面（肉眼找"模型没分割好"的弱帧），导出补标时要干净的原始帧。
// 所以两路共用一个时间戳 base 命名、在同一回调里逐帧写入 → 帧号天然对齐：
//   回放 → xxx_mask.avi；导出补标 → xxx.avi 的同一帧号抽帧。
//
// 线程模型：start/stop 在控制线程调用（10Hz 定时器消费网页按钮请求），
// write_frame 在图像订阅回调（30fps）调用；active_ 用原子变量保证跨线程可见。
class DualRecorder
{
public:
  DualRecorder() = default;
  ~DualRecorder() { stop(); }

  void configure(const std::string & record_dir, int fps);

  // 开始新片段：生成 base 名（record_YYYYmmdd_HHMMSS）并置 active。
  // 两个 writer 延迟到第一帧写入时按帧尺寸惰性创建（尺寸来自相机实际帧）。
  bool start();
  // 封存当前片段（写空剩余帧 + 关闭文件）。
  void stop();
  bool active() const { return active_.load(); }

  // 每帧调用：raw = 原始 RGB888；overlay = 叠加后 RGB888（可为 nullptr → 只写原始）。
  void write_frame(const uint8_t * raw, const uint8_t * overlay,
                   int width, int height);

  const std::string & current_base() const { return base_; }
  const std::string & raw_path() const { return raw_path_; }
  const std::string & overlay_path() const { return overlay_path_; }

private:
  void ensureWriters(int width, int height);

  std::string record_dir_;
  int fps_ = 30;
  std::atomic<bool> active_{false};
  std::string base_;
  std::string raw_path_;
  std::string overlay_path_;
  AsyncAviWriter * raw_writer_ = nullptr;
  AsyncAviWriter * overlay_writer_ = nullptr;
};

}  // namespace scoutcar_web
