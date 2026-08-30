#include "video_writer.h"

#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

#include <opencv2/opencv.hpp>

namespace scoutcar_web {

AsyncAviWriter * AsyncAviWriter::create(
  int width, int height, int fps,
  const std::string & save_dir, const std::string & filename)
{
  // 1) 确保保存目录存在
  struct stat st;
  if (stat(save_dir.c_str(), &st) != 0) {
    if (mkdir(save_dir.c_str(), 0755) != 0) {
      printf("[video_writer] 无法创建目录 %s\n", save_dir.c_str());
      return nullptr;
    }
  }

  // 2) 分配上下文
  AsyncAviWriter * w = new AsyncAviWriter();
  w->width_ = width;
  w->height_ = height;
  w->fps_ = fps;
  w->path_ = save_dir + "/" + filename;

  // 3) 创建 OpenCV VideoWriter（MJPEG 编码 .avi，与原版一致）
  w->writer_ = new cv::VideoWriter();
  const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  const bool ok = w->writer_->open(
    w->path_, fourcc, static_cast<double>(fps), cv::Size(width, height), true);
  if (!ok) {
    printf("[video_writer] 无法创建视频文件 %s\n", w->path_.c_str());
    delete w->writer_;
    delete w;
    return nullptr;
  }

  // 4) 预分配帧缓冲（环形队列）
  const int frame_size = width * height * 3;
  w->frames_.resize(kQueueSize);
  for (int i = 0; i < kQueueSize; ++i) {
    w->frames_[i] = new uint8_t[frame_size];
  }

  // 5) 启动写线程
  w->running_.store(true);
  w->write_idx_.store(0);
  w->read_idx_.store(0);
  w->dropped_.store(0);
  w->thread_ = new std::thread([w]() { w->writerLoop(); });

  printf("[video_writer] 开始录制(异步) → %s (%dx%d @ %d fps)\n",
         w->path_.c_str(), width, height, fps);
  return w;
}

AsyncAviWriter::~AsyncAviWriter()
{
  release();
}

void AsyncAviWriter::writerLoop()
{
  const int frame_size = width_ * height_ * 3;
  (void)frame_size;

  while (running_.load()) {
    const int w_idx = write_idx_.load();
    const int r_idx = read_idx_.load();
    if (r_idx == w_idx) {
      usleep(2000);   // 队列空，等 2ms
      continue;
    }

    // 消费一帧
    cv::Mat rgb(height_, width_, CV_8UC3, frames_[r_idx]);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    writer_->write(bgr);
    read_idx_.store((r_idx + 1) % kQueueSize);
  }

  drainRemaining();
}

void AsyncAviWriter::drainRemaining()
{
  while (true) {
    const int w_idx = write_idx_.load();
    const int r_idx = read_idx_.load();
    if (r_idx == w_idx) {
      break;
    }
    cv::Mat rgb(height_, width_, CV_8UC3, frames_[r_idx]);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    writer_->write(bgr);
    read_idx_.store((r_idx + 1) % kQueueSize);
  }
}

bool AsyncAviWriter::write(const uint8_t * rgb)
{
  if (rgb == nullptr || !running_.load()) {
    return false;
  }

  const int w_idx = write_idx_.load();
  const int r_idx = read_idx_.load();
  const int next_w = (w_idx + 1) % kQueueSize;

  // 队列满 → 丢此帧（编码跟不上）
  if (next_w == r_idx) {
    const int d = dropped_.fetch_add(1) + 1;
    if (d % 50 == 0) {
      printf("[video_writer] 丢帧 %d 次（编码跟不上）\n", d);
    }
    return true;
  }

  const int frame_size = width_ * height_ * 3;
  std::memcpy(frames_[w_idx], rgb, frame_size);
  write_idx_.store(next_w);
  return true;
}

void AsyncAviWriter::release()
{
  if (!running_.load() && thread_ == nullptr) {
    return;
  }
  running_.store(false);
  if (thread_ && thread_->joinable()) {
    thread_->join();
    delete thread_;
    thread_ = nullptr;
  }

  if (writer_ != nullptr) {
    writer_->release();
    delete writer_;
    writer_ = nullptr;
  }
  for (auto * buf : frames_) {
    delete[] buf;
  }
  frames_.clear();

  printf("[video_writer] 录制完成 → %s (丢帧:%d)\n",
         path_.c_str(), dropped_.load());
}

}  // namespace scoutcar_web
