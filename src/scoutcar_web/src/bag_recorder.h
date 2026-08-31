#pragma once

#include <sys/types.h>

#include <atomic>
#include <string>
#include <vector>

namespace scoutcar_web {

// 网页录像按钮的附属 rosbag 录制器。使用 ros2 bag 命令保持实现独立，
// stop() 发送 SIGINT，让 rosbag 正常写完 metadata.yaml。
class BagRecorder
{
public:
  BagRecorder() = default;
  ~BagRecorder() { stop(); }

  void configure(const std::string & bag_dir, const std::vector<std::string> & topics);
  bool start(const std::string & base_name);
  void stop();
  bool active() const { return pid_.load() > 0; }

private:
  std::string bag_dir_;
  std::vector<std::string> topics_;
  std::atomic<pid_t> pid_{-1};
};

}  // namespace scoutcar_web
