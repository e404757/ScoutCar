#include "scoutcar_perception/road_tracker.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace road_tracking {
namespace {

struct Snack {
  int start_x;
  int end_x;
  int length;
};

struct TrackedSnack {
  Snack snack;
  int y;
  bool used_barrier_gap;
};

Snack greedy_snack(int start_x, int width, const uint8_t * row) {
  Snack snack{start_x, start_x, 1};
  const uint8_t target_value = row[static_cast<size_t>(start_x)];

  for (int x = start_x + 1; x < width; ++x) {
    if (row[static_cast<size_t>(x)] != target_value) {
      break;
    }
    ++snack.length;
    snack.end_x = x;
  }

  return snack;
}

bool check_snack(const Snack & snack, int min_width, int max_width) {
  return snack.length >= min_width && snack.length <= max_width;
}

Snack find_best_snack(const uint8_t * row,
                      int search_start_x,
                      int search_end_x,
                      int reference_center,
                      const Config & config,
                      bool & used_barrier_gap) {
  std::vector<Snack> road_snacks;
  std::vector<Snack> barrier_snacks;

  used_barrier_gap = false;

  for (int x = search_start_x; x <= search_end_x; ++x) {
    const uint8_t value = row[static_cast<size_t>(x)];
    if (value == 0) {
      continue;
    }

    const Snack snack = greedy_snack(x, search_end_x + 1, row);
    x += snack.length - 1;

    if (value == 1) {
      road_snacks.push_back(snack);
    } else if (value == 2 &&//过滤掉过窄的挡板
               snack.length >= config.min_barrier_width_px) {
      barrier_snacks.push_back(snack);
    }
  }

  for (const Snack & road : road_snacks) {
    if (check_snack(
            road, config.min_road_width_px, config.max_road_width_px)) {
      return road;
    }
  }

  const Snack * left_barrier = nullptr;
  const Snack * right_barrier = nullptr;

  for (const Snack & barrier : barrier_snacks) {
    if (barrier.end_x < reference_center &&//找到离参考点最近且不超过参考点的左挡板
        (!left_barrier || barrier.end_x > left_barrier->end_x)) {
      left_barrier = &barrier;
    }

    if (barrier.start_x > reference_center &&//找到离参考点最近且超过参考点的右挡板
        (!right_barrier ||
         barrier.start_x < right_barrier->start_x)) {
      right_barrier = &barrier;
    }
  }

  if (left_barrier && right_barrier) {
    const int road_start_x = left_barrier->end_x + 1;
    const int road_end_x = right_barrier->start_x - 1;
    const Snack barrier_gap{
      road_start_x,
      road_end_x,
      road_end_x - road_start_x + 1
    };

    if (check_snack(barrier_gap,
                    config.min_road_width_px,
                    config.max_road_width_px)) {
      used_barrier_gap = true;
      return barrier_gap;
    }
  }

  return Snack{-1, -1, 0};
}

// 单行判据：长度达标的挡板段从左侧贴到右侧，段间夹带的是路面。
// 也就是整行被"挡板—路面—挡板"拼满；夹带的路面多宽不限，但不允许没有夹缝
// （整行一大块挡板）或夹缝里是背景。不在这里判断参考中心。
bool row_is_blocked(const uint8_t * row, int width,
                    const BlockedRowConfig & config, int & left_x,
                    int & right_x, int & max_gap) {
  left_x = -1;
  right_x = -1;
  max_gap = 0;
  int prev_end = -1;
  bool has_gap = false;      // 必须真的存在夹缝，整行一大块挡板不算
  bool gaps_are_road = true;

  for (int x = 0; x < width;) {
    if (row[static_cast<size_t>(x)] != 2) {
      ++x;
      continue;
    }

    const Snack snack = greedy_snack(x, width, row);
    x = snack.end_x + 1;

    if (snack.length < config.min_barrier_width_px) {//过滤掉过窄的挡板
      continue;
    }
    if (left_x < 0) {
      left_x = snack.start_x;
    }
    if (prev_end >= 0) {
      //挡板之间必须夹带路面：缝隙里每个像素都得是路面；夹缝为 0
      //（两段挡板贴死，掩膜里会并成一段）没有路面可夹，不算有效
      const int gap = snack.start_x - prev_end - 1;
      max_gap = std::max(max_gap, gap);
      has_gap = true;
      if (gap <= 0) {
        gaps_are_road = false;
      } else {
        for (int gap_x = prev_end + 1; gap_x < snack.start_x; ++gap_x) {
          if (row[static_cast<size_t>(gap_x)] != 1) {
            gaps_are_road = false;
            break;
          }
        }
      }
    }
    prev_end = snack.end_x;
    right_x = snack.end_x;
  }

  if (left_x < 0) {
    return false;
  }
  if (left_x > config.edge_tolerance_px) {
    return false;
  }
  if (right_x < width - 1 - config.edge_tolerance_px) {
    return false;
  }
  if (!has_gap || !gaps_are_road) {
    return false;
  }
  return true;
}

}  // namespace

RoadTracker::RoadTracker(const Config & config) : config_(config) {}

Result RoadTracker::process(const uint8_t * mask, int width, int height,
                            int reference_center,
                            uint8_t * processed_mask) const {
  Result result;
  if (!mask || width <= 0 || height <= 0) {
    return result;
  }

  const size_t size =
      static_cast<size_t>(width) * static_cast<size_t>(height);
  if (processed_mask) {
    std::fill(processed_mask, processed_mask + size, 0);
  }

  const int scan_end_y = std::clamp(config_.scan_end_y, 0, height - 1);
  const int search_expand_px = config_.search_expand_px;
  const int required_valid_rows = config_.min_valid_rows;

  int search_start_x = 0;
  int search_end_x = width - 1;
  std::vector<TrackedSnack> tracked_snacks;

  for (int y = height - 1; y >= scan_end_y; --y) {
    const uint8_t * row =
        mask + static_cast<size_t>(y) * static_cast<size_t>(width);

    bool used_barrier_gap = false;
    const Snack best_snack =
        find_best_snack(row, search_start_x, search_end_x, reference_center,
                        config_, used_barrier_gap);

    if (best_snack.length <= 0) {
      tracked_snacks.clear();
      search_start_x = 0;
      search_end_x = width - 1;
      continue;
    }

    tracked_snacks.push_back({best_snack, y, used_barrier_gap});

    if (tracked_snacks.size() == required_valid_rows) {
      long long deviation_sum = 0;
      int min_road_x=width-1;
      int max_road_x=0;
      int min_road_width = width;
      int max_road_width = 0;
      bool any_used_barrier_gap = false;
      for (const TrackedSnack & tracked : tracked_snacks) {
        const int snack_center =
            (tracked.snack.start_x + tracked.snack.end_x) / 2;
        deviation_sum += snack_center - reference_center;
        min_road_width = std::min(min_road_width, tracked.snack.length);
        max_road_width = std::max(max_road_width, tracked.snack.length);
        min_road_x = std::min(min_road_x, tracked.snack.start_x);
        max_road_x = std::max(max_road_x, tracked.snack.end_x);
        any_used_barrier_gap =
            any_used_barrier_gap || tracked.used_barrier_gap;

        if (processed_mask) {
          uint8_t * processed_row =
              processed_mask +
              static_cast<size_t>(tracked.y) * static_cast<size_t>(width);
          std::fill(processed_row + tracked.snack.start_x,
                    processed_row + tracked.snack.end_x + 1, 1);
        }
      }

  
      result.valid = true;
      result.deviation =
          static_cast<int>(deviation_sum / required_valid_rows);
      result.center_x = reference_center + result.deviation;
      result.y = tracked_snacks[tracked_snacks.size() / 2].y;
      result.left = min_road_x;
      result.right = max_road_x;
      result.min_width = min_road_width;
      result.max_width = max_road_width;
      result.used_barrier_gap = any_used_barrier_gap;
      break;
    }

    search_start_x =
        std::max(0, best_snack.start_x - search_expand_px);
    search_end_x =
        std::min(width - 1, best_snack.end_x + search_expand_px);
  }

  return result;
}

BlockedRowResult scan_blocked_rows(const uint8_t * mask, int width, int height,
                                   const BlockedRowConfig & config) {
  BlockedRowResult result;
  if (!mask || width <= 0 || height <= 0) {
    return result;
  }

  // 从窗口下沿向图像上方（y 减小）连续计数，遇到第一行不满足就停。
  const int bottom = std::clamp(config.window_bottom_y, 0, height - 1);

  for (int y = bottom; y >= 0; --y) {
    const uint8_t * row =
        mask + static_cast<size_t>(y) * static_cast<size_t>(width);
    int left_x = -1;
    int right_x = -1;
    int max_gap = 0;
    if (!row_is_blocked(row, width, config, left_x, right_x, max_gap)) {
      break;
    }

    ++result.consecutive_rows;
    result.row_y = y;
    result.left_x = left_x;
    result.right_x = right_x;
    result.max_gap = max_gap;
  }

  return result;
}

}  // namespace road_tracking
