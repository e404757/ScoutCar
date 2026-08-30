#include "dual_recorder.h"

#include <cstdio>
#include <ctime>

namespace scoutcar_web {

void DualRecorder::configure(const std::string & record_dir, int fps)
{
  record_dir_ = record_dir;
  fps_ = fps;
}

bool DualRecorder::start()
{
  if (active_.load()) {
    return false;   // 已在录制
  }
  if (record_dir_.empty()) {
    printf("[recorder] record_dir 未配置，无法开始录像\n");
    return false;
  }

  // 时间戳 base：record_YYYYmmdd_HHMMSS
  time_t now = time(nullptr);
  struct tm * tm_info = localtime(&now);
  char base[128];
  strftime(base, sizeof(base), "record_%Y%m%d_%H%M%S", tm_info);
  base_ = base;
  raw_path_ = record_dir_ + "/" + base_ + ".avi";
  overlay_path_ = record_dir_ + "/" + base_ + "_mask.avi";
  active_.store(true);
  printf("[recorder] 开始录制片段 %s （原始 + 叠加双文件）\n", base_.c_str());
  return true;
}

void DualRecorder::stop()
{
  if (!active_.load()) {
    return;
  }
  active_.store(false);
  if (raw_writer_ != nullptr) {
    raw_writer_->release();
    delete raw_writer_;
    raw_writer_ = nullptr;
  }
  if (overlay_writer_ != nullptr) {
    overlay_writer_->release();
    delete overlay_writer_;
    overlay_writer_ = nullptr;
  }
  printf("[recorder] 片段 %s 已封存\n", base_.c_str());
}

void DualRecorder::ensureWriters(int width, int height)
{
  if (raw_writer_ != nullptr) {
    return;   // 已按某帧尺寸创建
  }
  raw_writer_ = AsyncAviWriter::create(
    width, height, fps_, record_dir_, base_ + ".avi");
  overlay_writer_ = AsyncAviWriter::create(
    width, height, fps_, record_dir_, base_ + "_mask.avi");
  if (raw_writer_ == nullptr || overlay_writer_ == nullptr) {
    printf("[recorder] 创建双路写入器失败，停止片段\n");
    stop();
  }
}

void DualRecorder::write_frame(
  const uint8_t * raw, const uint8_t * overlay, int width, int height)
{
  if (!active_.load() || raw == nullptr) {
    return;
  }
  ensureWriters(width, height);
  if (raw_writer_ == nullptr) {
    return;   // 创建失败已 stop
  }
  raw_writer_->write(raw);
  if (overlay != nullptr && overlay_writer_ != nullptr) {
    overlay_writer_->write(overlay);
  }
}

}  // namespace scoutcar_web
