#pragma once

#include <cstdint>

namespace road_tracking {

struct Config {
  int scan_end_y = 120;
  int min_road_width_px = 225;
  int max_road_width_px = 285;
  int min_barrier_width_px = 12;
  int search_expand_px = 20;
  int min_valid_rows = 60;
};

struct Result {
  bool valid = false;
  int deviation = 0;
  int y = 0;
  int left = -1;
  int right = -1;
  int center_x = -1;
  bool used_barrier_gap = false;
  int min_width = 0;
  int max_width = 0;
};

// 「挡板把整行拼满」的扫描配置。与 Config 分开：process() 用来算偏差，
// 这套参数只服务于"接近侦察点"的判断，两者互不影响。
struct BlockedRowConfig {
  int window_bottom_y = 288;      // 扫描窗口下沿（= 0.6 × 图像高）；y 越小越远
  int required_rows = 30;         // 从下沿向上需要连续满足的行数
  int min_barrier_width_px = 12;  // 与 Config::min_barrier_width_px 取同一值
  int edge_tolerance_px = 20;     // 左右两侧的贴边容差
};

struct BlockedRowResult {
  int consecutive_rows = 0;  // 从 window_bottom_y 向上数出的连续满足行数
  int row_y = -1;            // 连续段最靠上（最远）的那一行
  int left_x = -1;           // 该行最左挡板段的起点
  int right_x = -1;          // 该行最右挡板段的终点
  int max_gap = -1;          // 该行挡板段之间夹带的路面条最大宽度（仅诊断用，不参与判定）
};

// 独立于 RoadTracker::process()：全域扫描，不读写任何状态。
// 用于判断"挡板从左到右把整行拼满"（前方被封死），process() 的配对逻辑在这种
// 图案下必然返回无路面，所以不能复用它的结果。
BlockedRowResult scan_blocked_rows(const uint8_t * mask, int width, int height,
                                   const BlockedRowConfig & config);

class RoadTracker {
public:
  explicit RoadTracker(const Config & config = Config{});
  Result process(const uint8_t * mask, int width, int height,
                 int reference_center,
                 uint8_t * processed_mask = nullptr) const;

private:
  Config config_;
};

}  // namespace road_tracking
