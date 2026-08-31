#include "scoutcar_perception/road_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace road_tracking {
namespace {
struct Run { int left; int right; bool from_barrier = false; };

int center(const Run & run) { return (run.left + run.right) / 2; }

int overlap(const Run & a, const Run & b, int margin) {
  return std::max(0, std::min(a.right + margin, b.right) -
                     std::max(a.left - margin, b.left) + 1);
}

std::vector<Run> find_road_runs(const uint8_t * row, int width, const Config & config) {
  std::vector<Run> runs;
  int left = -1;
  int last = -1;
  for (int x = 0; x < width; ++x) {
    if (row[x] != 1) continue;
    if (left < 0) left = last = x;
    else if (x - last - 1 <= config.max_inline_gap_px) last = x;
    else {
      if (last - left + 1 >= config.min_run_width_px) runs.push_back({left, last});
      left = last = x;
    }
  }
  if (left >= 0 && last - left + 1 >= config.min_run_width_px) runs.push_back({left, last});
  return runs;
}

bool find_barrier_gap(const uint8_t * row, int width, const Config & config, Run & gap) {
  const int image_center = width / 2;
  int left_edge = -1, right_edge = -1, left = -1, last = -1;
  auto take = [&](int run_left, int run_right) {
    if (run_right - run_left + 1 < config.min_run_width_px) return;
    if ((run_left + run_right) / 2 <= image_center) left_edge = std::max(left_edge, run_right);
    else if (right_edge < 0 || run_left < right_edge) right_edge = run_left;
  };
  for (int x = 0; x < width; ++x) {
    if (row[x] != 2) continue;
    if (left < 0) left = last = x;
    else if (x - last - 1 <= config.max_inline_gap_px) last = x;
    else { take(left, last); left = last = x; }
  }
  if (left >= 0) take(left, last);
  if (left_edge < 0 || right_edge < 0 ||
      right_edge - left_edge - 1 < config.min_run_width_px) return false;
  gap = {left_edge + 1, right_edge - 1, true};
  return true;
}
}  // namespace

struct RoadTracker::Estimate {
  bool valid = false;
  int y = 0;
  int left = -1;
  int right = -1;
  bool used_barrier = false;
};

RoadTracker::RoadTracker(const Config & config) : config_(config) {}

RoadTracker::Estimate RoadTracker::estimate(
    const uint8_t * mask, int width, int height, const PreviewConfig & preview,
    uint8_t * processed_mask) const {
  Estimate nearest;
  const int target_y = std::clamp(static_cast<int>(height * preview.row_ratio), 0, height - 1);
  nearest.y = target_y;
  const int image_center = width / 2;
  const int seed_top = std::max(target_y, static_cast<int>(height * config_.bottom_seed_ratio));
  Run tracked{-1, -1, false};
  int seed_y = -1;
  for (int y = height - 1; y >= seed_top && seed_y < 0; --y) {
    auto runs = find_road_runs(mask + static_cast<size_t>(y) * width, width, config_);
    if (config_.use_barrier_gap) {
      Run gap;
      if (find_barrier_gap(mask + static_cast<size_t>(y) * width, width, config_, gap)) runs.push_back(gap);
    }
    int best_score = std::numeric_limits<int>::min();
    for (const auto & run : runs) {
      int score = run.right - run.left + 1 - 2 * std::abs(center(run) - image_center);
      if (run.left <= image_center && image_center <= run.right) score += width;
      if (score > best_score) { best_score = score; tracked = run; seed_y = y; }
    }
  }
  if (seed_y < 0) return nearest;

  auto mark = [&](int y, const Run & run) {
    if (!processed_mask) return;
    const uint8_t * row = mask + static_cast<size_t>(y) * width;
    uint8_t * output = processed_mask + static_cast<size_t>(y) * width;
    for (int x = run.left; x <= run.right; ++x) if (row[x] == 1) output[x] = 255;
  };
  auto remember = [&](int y, const Run & connected) {
    const uint8_t * row = mask + static_cast<size_t>(y) * width;
    const auto roads = find_road_runs(row, width, config_);
    int best = -1;
    int best_score = std::numeric_limits<int>::min();
    for (size_t i = 0; i < roads.size(); ++i) {
      const int road_width = roads[i].right - roads[i].left + 1;
      if (road_width < preview.min_width || road_width > preview.max_width) continue;
      const int score = overlap(connected, roads[i], config_.overlap_margin_px) * 1000 -
                        std::abs(center(roads[i]) - image_center);
      if (score > best_score) { best_score = score; best = static_cast<int>(i); }
    }
    Run selected;
    bool found = false;
    if (best >= 0) { selected = roads[best]; found = true; }
    else if (config_.use_barrier_gap && find_barrier_gap(row, width, config_, selected)) {
      const int gap_width = selected.right - selected.left + 1;
      found = gap_width >= preview.min_width && gap_width <= preview.max_width;
    }
    if (found) nearest = {true, y, selected.left, selected.right, selected.from_barrier};
  };

  mark(seed_y, tracked);
  remember(seed_y, tracked);
  int missing_rows = 0;
  for (int y = seed_y - 1; y >= target_y; --y) {
    auto runs = find_road_runs(mask + static_cast<size_t>(y) * width, width, config_);
    if (config_.use_barrier_gap) {
      Run gap;
      if (find_barrier_gap(mask + static_cast<size_t>(y) * width, width, config_, gap)) runs.push_back(gap);
    }
    int best = -1, best_overlap = 0;
    for (size_t i = 0; i < runs.size(); ++i) {
      const int score = overlap(tracked, runs[i], config_.overlap_margin_px);
      if (score > best_overlap) { best_overlap = score; best = static_cast<int>(i); }
    }
    if (best < 0) {
      if (++missing_rows > config_.max_missing_rows) break;
      continue;
    }
    missing_rows = 0;
    tracked = runs[best];
    mark(y, tracked);
    remember(y, tracked);
  }
  return nearest;
}

Result RoadTracker::process(const uint8_t * mask, int width, int height,
                            uint8_t * processed_mask) const {
  Result result;
  if (!mask || width <= 0 || height <= 0) return result;
  const size_t mask_size = static_cast<size_t>(width) * height;
  if (processed_mask) std::fill(processed_mask, processed_mask + mask_size, 0);

  Estimate chosen = estimate(mask, width, height, config_.near, processed_mask);
  const bool near_bad = !chosen.valid || chosen.left <= config_.near.edge_margin_px ||
                        chosen.right >= width - 1 - config_.near.edge_margin_px;
  const PreviewConfig * selected_config = &config_.near;
  result.preview = Preview::NEAR;
  if (near_bad) {
    std::vector<uint8_t> far_mask(processed_mask ? mask_size : 0, 0);
    const Estimate far = estimate(mask, width, height, config_.far,
                                  far_mask.empty() ? nullptr : far_mask.data());
    const bool far_ok = far.valid && far.left > config_.far.edge_margin_px &&
                        far.right < width - 1 - config_.far.edge_margin_px;
    chosen = far_ok ? far : Estimate{};
    selected_config = &config_.far;
    result.preview = far_ok ? Preview::FAR : Preview::NONE;
    if (far_ok && processed_mask) std::copy(far_mask.begin(), far_mask.end(), processed_mask);
  }

  result.y = chosen.y;
  result.selected_pt = chosen.valid ? static_cast<float>(chosen.y) / height
                                    : selected_config->row_ratio;
  result.min_width = selected_config->min_width;
  result.max_width = selected_config->max_width;
  if (!chosen.valid) return result;
  result.valid = true;
  result.left = chosen.left;
  result.right = chosen.right;
  result.width = chosen.right - chosen.left + 1;
  result.center_x = (chosen.left + chosen.right) / 2;
  result.deviation = result.center_x - width / 2;
  result.used_barrier_gap = chosen.used_barrier;
  return result;
}

const char * preview_name(Preview preview) {
  switch (preview) {
    case Preview::NEAR: return "NEAR";
    case Preview::FAR: return "FAR";
    case Preview::NONE: return "NONE";
  }
  return "NONE";
}
}  // namespace road_tracking
