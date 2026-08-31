#include "bag_recorder.h"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <thread>

#include <sys/wait.h>
#include <unistd.h>

namespace scoutcar_web {

void BagRecorder::configure(
  const std::string & bag_dir, const std::vector<std::string> & topics)
{
  bag_dir_ = bag_dir;
  topics_ = topics;
}

bool BagRecorder::start(const std::string & base_name)
{
  if (active() || bag_dir_.empty() || topics_.empty() || base_name.empty()) {
    return false;
  }

  std::error_code error;
  std::filesystem::create_directories(bag_dir_, error);
  if (error) {
    std::fprintf(stderr, "[bag_recorder] 无法创建目录 %s: %s\n",
                 bag_dir_.c_str(), error.message().c_str());
    return false;
  }

  const std::string output = bag_dir_ + "/" + base_name;
  if (std::filesystem::exists(output)) {
    std::fprintf(stderr, "[bag_recorder] 输出目录已存在: %s\n", output.c_str());
    return false;
  }

  std::vector<std::string> args{
    "ros2", "bag", "record",
    "--compression-mode", "file",
    "--compression-format", "zstd",
    "-o", output
  };
  args.insert(args.end(), topics_.begin(), topics_.end());
  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (auto & arg : args) argv.push_back(arg.data());
  argv.push_back(nullptr);

  const pid_t child = ::fork();
  if (child < 0) {
    std::perror("[bag_recorder] fork");
    return false;
  }
  if (child == 0) {
    ::setpgid(0, 0);
    ::execvp(argv[0], argv.data());
    std::perror("[bag_recorder] execvp ros2");
    _exit(127);
  }

  // 父子两侧都设置，消除 start 后立即 stop 时进程组尚未建立的竞态。
  ::setpgid(child, child);
  pid_ = child;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  int status = 0;
  if (::waitpid(child, &status, WNOHANG) == child) {
    std::fprintf(stderr, "[bag_recorder] ros2 bag 启动失败，status=%d\n", status);
    pid_ = -1;
    return false;
  }
  std::printf("[bag_recorder] 开始录制 %s\n", output.c_str());
  return true;
}

void BagRecorder::stop()
{
  if (!active()) return;
  const pid_t child = pid_.exchange(-1);
  if (child <= 0) return;

  // 子进程是独立进程组；向整个组发 SIGINT，确保 ros2 CLI 和 recorder 一起退出。
  ::kill(-child, SIGINT);
  int status = 0;
  for (int attempt = 0; attempt < 40; ++attempt) {
    const pid_t result = ::waitpid(child, &status, WNOHANG);
    if (result == child || result < 0) {
      std::printf("[bag_recorder] rosbag 已封存\n");
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  std::fprintf(stderr, "[bag_recorder] SIGINT 后未退出，发送 SIGTERM\n");
  ::kill(-child, SIGTERM);
  ::waitpid(child, &status, 0);
}

}  // namespace scoutcar_web
