#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/videoio.hpp>

namespace scoutcar_web {

// 异步 MJPEG .avi 写入器（搬自原工程 main/video_writer.cc，做两处适配）：
//   1. 帧类型从 image_buffer_t 改为裸 RGB888 指针——不再依赖感知包的 image_utils；
//   2. 文件名由调用方显式指定——双文件同步录（原始 + 叠加）需要共用一个时间戳 base。
// 线程/环形缓冲/丢帧计数逻辑与原版保持一致：
//   - 主线程写帧只做一次 memcpy 进环形队列（极快、不阻塞）；
//   - 写线程负责 RGB→BGR 转换 + MJPEG 编码落盘；
//   - 队列满（编码跟不上）时丢帧并计数，不拖慢主循环。
class AsyncAviWriter
{
public:
  // 创建写入器。save_dir 不存在会自动创建；filename 为完整文件名（含 .avi）。
  // 失败返回 nullptr（目录不可创建 / VideoWriter 打不开）。
  static AsyncAviWriter * create(int width, int height, int fps,
                                 const std::string & save_dir,
                                 const std::string & filename);

  ~AsyncAviWriter();

  // 写入一帧 RGB888（纯内存拷贝，立即返回）。未在录制时返回 false。
  bool write(const uint8_t * rgb);

  // 停止写线程（写空剩余帧）并释放资源。之后对象不可再用。
  void release();

  int dropped() const { return dropped_.load(); }
  const std::string & path() const { return path_; }

private:
  AsyncAviWriter() = default;
  void writerLoop();
  void drainRemaining();

  static constexpr int kQueueSize = 4;   // 缓冲帧数（原版 QUEUE_SIZE）

  int width_ = 0;
  int height_ = 0;
  int fps_ = 0;
  std::string path_;
  cv::VideoWriter * writer_ = nullptr;
  std::thread * thread_ = nullptr;
  std::atomic<bool> running_{false};
  std::atomic<int> write_idx_{0};
  std::atomic<int> read_idx_{0};
  std::atomic<int> dropped_{0};
  std::vector<uint8_t *> frames_;
};

}  // namespace scoutcar_web
