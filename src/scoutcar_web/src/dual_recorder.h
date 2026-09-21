#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/videoio.hpp>

namespace scoutcar_web {

enum class CameraSource : uint8_t {
  FRONT = 1,
  TURN = 2,
};

// 与最终 AVI 帧一一对应的状态快照。字段保持原始 ROS 语义，网页端再按需要分组。
struct FrameMetadata {
  int64_t image_stamp_ns = 0;
  CameraSource camera = CameraSource::FRONT;
  bool has_mission_state = false;
  int64_t state_stamp_ns = 0;
  uint32_t mission_revision = 0;
  uint8_t mission_state = 0;
  int32_t segment_start = -1;
  int32_t segment_goal = -1;
  int32_t segment_index = -1;
  uint8_t arrival_action = 0;
  uint8_t front_camera_pose = 0;
  uint8_t turn_camera_pose = 0;
};

// 原始帧、掩膜叠加帧、逐帧元数据共用一个队列和一个写线程。
// 队列满时整组丢弃，保证两个 AVI 的第 N 帧与 JSONL 第 N 行始终对应。
class DualRecorder
{
public:
  DualRecorder() = default;
  ~DualRecorder() { stop(); }

  void configure(const std::string & record_dir,
                 const std::string & bag_dir, int fps);
  bool start(int fps);
  void stop();
  bool active() const { return active_.load(); }

  bool write_frame(const uint8_t * raw, const uint8_t * overlay,
                   int width, int height, const FrameMetadata & metadata);

  const std::string & current_base() const { return base_; }
  const std::string & raw_path() const { return raw_path_; }
  const std::string & overlay_path() const { return overlay_path_; }

private:
  struct QueuedFrame {
    std::vector<uint8_t> raw;
    std::vector<uint8_t> overlay;
    FrameMetadata metadata;
  };

  static constexpr size_t kQueueSize = 4;

  bool open_outputs();
  void writer_loop();
  void write_frame_metadata(uint64_t frame_index,
                            const FrameMetadata & metadata);
  bool finalize_index();
  bool write_manifest();
  void reset_session();

  std::string record_dir_;
  std::string bag_dir_;
  int fps_ = 20;
  int width_ = 0;
  int height_ = 0;
  std::atomic<bool> active_{false};
  std::atomic<bool> stopping_{false};
  std::atomic<bool> output_error_{false};
  std::atomic<uint64_t> dropped_frames_{0};
  std::atomic<uint64_t> written_frames_{0};
  int64_t started_at_ns_ = 0;
  int64_t ended_at_ns_ = 0;

  std::string base_;
  std::string raw_path_;
  std::string overlay_path_;
  std::string frames_path_;
  std::string frames_part_path_;
  std::string manifest_path_;
  std::string manifest_part_path_;

  cv::VideoWriter raw_writer_;
  cv::VideoWriter overlay_writer_;
  std::ofstream frames_stream_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<QueuedFrame> queue_;
  std::thread writer_thread_;
};

}  // namespace scoutcar_web
