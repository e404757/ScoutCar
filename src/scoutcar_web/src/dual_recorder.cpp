#include "dual_recorder.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <system_error>

#include <opencv2/opencv.hpp>

namespace scoutcar_web {
namespace {

int64_t system_time_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::system_clock::now().time_since_epoch()).count();
}

const char * camera_name(CameraSource camera)
{
  return camera == CameraSource::TURN ? "turn" : "front";
}

}  // namespace

void DualRecorder::configure(const std::string & record_dir,
                             const std::string & bag_dir, int fps)
{
  record_dir_ = record_dir;
  bag_dir_ = bag_dir;
  fps_ = fps;
}

bool DualRecorder::start(int fps)
{
  if (active_.load() || writer_thread_.joinable()) return false;
  if (record_dir_.empty()) {
    std::printf("[recorder] record_dir 未配置，无法开始录像\n");
    return false;
  }
  if (fps > 0) fps_ = fps;

  std::error_code error;
  std::filesystem::create_directories(record_dir_, error);
  if (error) {
    std::fprintf(stderr, "[recorder] 无法创建目录 %s: %s\n",
                 record_dir_.c_str(), error.message().c_str());
    return false;
  }

  time_t now = time(nullptr);
  struct tm * tm_info = localtime(&now);
  char base[128];
  strftime(base, sizeof(base), "record_%Y%m%d_%H%M%S", tm_info);
  base_ = base;
  raw_path_ = record_dir_ + "/" + base_ + ".avi";
  overlay_path_ = record_dir_ + "/" + base_ + "_mask.avi";
  frames_path_ = record_dir_ + "/" + base_ + "_frames.jsonl";
  frames_part_path_ = frames_path_ + ".part";
  manifest_path_ = record_dir_ + "/" + base_ + "_manifest.json";
  manifest_part_path_ = manifest_path_ + ".part";

  reset_session();
  frames_stream_.open(frames_part_path_, std::ios::out | std::ios::trunc);
  if (!frames_stream_.is_open()) {
    std::fprintf(stderr, "[recorder] 无法创建逐帧索引 %s\n",
                 frames_part_path_.c_str());
    return false;
  }

  started_at_ns_ = system_time_ns();
  active_.store(true);
  writer_thread_ = std::thread([this]() { writer_loop(); });
  std::printf("[recorder] 开始录制片段 %s（同步双 AVI + 逐帧索引）\n",
              base_.c_str());
  return true;
}

void DualRecorder::reset_session()
{
  width_ = 0;
  height_ = 0;
  started_at_ns_ = 0;
  ended_at_ns_ = 0;
  stopping_.store(false);
  output_error_.store(false);
  dropped_frames_.store(0);
  written_frames_.store(0);
  std::lock_guard<std::mutex> lock(queue_mutex_);
  queue_.clear();
}

bool DualRecorder::open_outputs()
{
  if (raw_writer_.isOpened() && overlay_writer_.isOpened()) return true;
  if (width_ <= 0 || height_ <= 0) return false;

  const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  const cv::Size size(width_, height_);
  const bool raw_ok = raw_writer_.open(
    raw_path_, fourcc, static_cast<double>(fps_), size, true);
  const bool overlay_ok = overlay_writer_.open(
    overlay_path_, fourcc, static_cast<double>(fps_), size, true);
  if (!raw_ok || !overlay_ok) {
    std::fprintf(stderr, "[recorder] 无法创建同步双 AVI: %s / %s\n",
                 raw_path_.c_str(), overlay_path_.c_str());
    raw_writer_.release();
    overlay_writer_.release();
    return false;
  }
  std::printf("[recorder] 同步双 AVI 已打开 (%dx%d @ %d fps)\n",
              width_, height_, fps_);
  return true;
}

bool DualRecorder::write_frame(
  const uint8_t * raw, const uint8_t * overlay, int width, int height,
  const FrameMetadata & metadata)
{
  if (!active_.load() || output_error_.load() || raw == nullptr ||
      overlay == nullptr || width <= 0 || height <= 0) {
    return false;
  }

  const size_t frame_size =
    static_cast<size_t>(width) * static_cast<size_t>(height) * 3U;
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (width_ == 0 && height_ == 0) {
    width_ = width;
    height_ = height;
  } else if (width != width_ || height != height_) {
    std::fprintf(stderr,
                 "[recorder] 忽略尺寸变化帧: %dx%d，当前录像为 %dx%d\n",
                 width, height, width_, height_);
    dropped_frames_.fetch_add(1);
    return false;
  }
  if (queue_.size() >= kQueueSize) {
    const uint64_t dropped = dropped_frames_.fetch_add(1) + 1;
    if (dropped % 50 == 0) {
      std::printf("[recorder] 同步双帧队列已满，累计丢帧 %llu\n",
                  static_cast<unsigned long long>(dropped));
    }
    return false;
  }

  QueuedFrame frame;
  frame.raw.assign(raw, raw + frame_size);
  frame.overlay.assign(overlay, overlay + frame_size);
  frame.metadata = metadata;
  queue_.push_back(std::move(frame));
  queue_cv_.notify_one();
  return true;
}

void DualRecorder::writer_loop()
{
  while (true) {
    QueuedFrame frame;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() {
        return stopping_.load() || !queue_.empty();
      });
      if (queue_.empty()) {
        if (stopping_.load()) break;
        continue;
      }
      frame = std::move(queue_.front());
      queue_.pop_front();
    }

    if (!open_outputs()) {
      output_error_.store(true);
      continue;
    }
    cv::Mat raw_rgb(height_, width_, CV_8UC3, frame.raw.data());
    cv::Mat overlay_rgb(height_, width_, CV_8UC3, frame.overlay.data());
    cv::Mat raw_bgr;
    cv::Mat overlay_bgr;
    cv::cvtColor(raw_rgb, raw_bgr, cv::COLOR_RGB2BGR);
    cv::cvtColor(overlay_rgb, overlay_bgr, cv::COLOR_RGB2BGR);
    raw_writer_.write(raw_bgr);
    overlay_writer_.write(overlay_bgr);

    const uint64_t index = written_frames_.fetch_add(1);
    write_frame_metadata(index, frame.metadata);
    if (!frames_stream_) output_error_.store(true);
  }
}

void DualRecorder::write_frame_metadata(
  uint64_t frame_index, const FrameMetadata & metadata)
{
  frames_stream_
    << "{\"frame_index\":" << frame_index
    << ",\"image_stamp_ns\":" << metadata.image_stamp_ns
    << ",\"camera\":\"" << camera_name(metadata.camera) << "\""
    << ",\"has_mission_state\":"
    << (metadata.has_mission_state ? "true" : "false")
    << ",\"state_stamp_ns\":" << metadata.state_stamp_ns
    << ",\"mission_revision\":" << metadata.mission_revision
    << ",\"mission_state\":" << static_cast<unsigned>(metadata.mission_state)
    << ",\"segment_start\":" << metadata.segment_start
    << ",\"segment_goal\":" << metadata.segment_goal
    << ",\"segment_index\":" << metadata.segment_index
    << ",\"arrival_action\":" << static_cast<unsigned>(metadata.arrival_action)
    << ",\"front_camera_pose\":"
    << static_cast<unsigned>(metadata.front_camera_pose)
    << ",\"turn_camera_pose\":"
    << static_cast<unsigned>(metadata.turn_camera_pose)
    << "}\n";
}

bool DualRecorder::finalize_index()
{
  frames_stream_.flush();
  const bool stream_ok = frames_stream_.good();
  frames_stream_.close();
  if (!stream_ok) return false;

  std::error_code error;
  std::filesystem::rename(frames_part_path_, frames_path_, error);
  if (error) {
    std::fprintf(stderr, "[recorder] 逐帧索引封存失败: %s\n",
                 error.message().c_str());
    return false;
  }
  return true;
}

bool DualRecorder::write_manifest()
{
  const std::filesystem::path bag_path =
    std::filesystem::path(bag_dir_) / base_;
  const bool bag_complete =
    std::filesystem::is_regular_file(bag_path / "metadata.yaml");

  std::ofstream manifest(manifest_part_path_, std::ios::out | std::ios::trunc);
  if (!manifest.is_open()) return false;
  manifest
    << "{\n"
    << "  \"schema_version\": 1,\n"
    << "  \"name\": \"" << base_ << "\",\n"
    << "  \"complete\": true,\n"
    << "  \"width\": " << width_ << ",\n"
    << "  \"height\": " << height_ << ",\n"
    << "  \"fps\": " << fps_ << ",\n"
    << "  \"frame_count\": " << written_frames_.load() << ",\n"
    << "  \"dropped_frames\": " << dropped_frames_.load() << ",\n"
    << "  \"started_at_ns\": " << started_at_ns_ << ",\n"
    << "  \"ended_at_ns\": " << ended_at_ns_ << ",\n"
    << "  \"raw\": \"" << base_ << ".avi\",\n"
    << "  \"mask\": \"" << base_ << "_mask.avi\",\n"
    << "  \"frames\": \"" << base_ << "_frames.jsonl\",\n"
    << "  \"bag_name\": \"" << base_ << "\",\n"
    << "  \"bag_complete\": " << (bag_complete ? "true" : "false") << "\n"
    << "}\n";
  manifest.flush();
  const bool manifest_ok = manifest.good();
  manifest.close();
  if (!manifest_ok) return false;

  std::error_code error;
  std::filesystem::rename(manifest_part_path_, manifest_path_, error);
  if (error) {
    std::fprintf(stderr, "[recorder] manifest 封存失败: %s\n",
                 error.message().c_str());
    return false;
  }
  return true;
}

void DualRecorder::stop()
{
  if (!active_.exchange(false) && !writer_thread_.joinable()) return;
  stopping_.store(true);
  queue_cv_.notify_all();
  if (writer_thread_.joinable()) writer_thread_.join();

  raw_writer_.release();
  overlay_writer_.release();
  ended_at_ns_ = system_time_ns();

  const bool index_ok = finalize_index();
  const bool complete =
    !output_error_.load() && written_frames_.load() > 0 && index_ok;
  const bool manifest_ok = complete && write_manifest();
  if (manifest_ok) {
    std::printf(
      "[recorder] 片段 %s 已封存（帧:%llu，整组丢帧:%llu）\n",
      base_.c_str(),
      static_cast<unsigned long long>(written_frames_.load()),
      static_cast<unsigned long long>(dropped_frames_.load()));
  } else {
    std::fprintf(stderr,
      "[recorder] 片段 %s 未生成完成标记，请勿作为完整批次同步\n",
      base_.c_str());
  }
}

}  // namespace scoutcar_web
