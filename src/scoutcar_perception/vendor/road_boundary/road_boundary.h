#pragma once

#include <cstdint>
#include <vector>

namespace road_boundary {

enum class Status { NORMAL, RECONSTRUCTED, NO_INFO };

struct Options {
  int max_inline_gap = 8;
  int min_run_width = 12;
  int boundary_jump_px = 36;
  int reconnect_tolerance_px = 20;
  int reconnect_rows = 2;
  int local_fit_points = 10;
  float far_row_ratio = 0.40f;
  float center_fit_top_ratio = 0.72f;
  float center_fit_bottom_ratio = 0.90f;
  int center_edge_margin = 10;
  int center_min_width = 40;
  int center_min_points = 30;
  int center_inlier_px = 8;
  float center_min_inlier_ratio = 0.60f;
  int center_hold_frames = 4;
  float center_smoothing = 0.35f;
  int center_max_jump_px = 50;
  float center_max_slope_jump = 0.35f;
  float center_max_abs_slope = 0.35f;
};

struct Result {
  bool valid = false;
  int deviation = -999;
  int y = 0;
  int raw_left = -1;
  int raw_right = -1;
  int raw_width = 0;
  int left = -1;
  int right = -1;
  int width = 0;
  int fit_top_y = 0;
  int fit_bottom_y = 0;
  bool left_reconstructed = false;
  bool right_reconstructed = false;
  Status status = Status::NO_INFO;
  std::vector<int16_t> raw_left_boundary;
  std::vector<int16_t> raw_right_boundary;
  std::vector<int16_t> fitted_left_boundary;
  std::vector<int16_t> fitted_right_boundary;
  std::vector<uint8_t> left_repaired;
  std::vector<uint8_t> right_repaired;
  std::vector<int16_t> center_boundary;
  float center_slope = 0.0f;
  float center_intercept = 0.0f;
  float center_confidence = 0.0f;
  int center_candidate_count = 0;
  int center_inlier_count = 0;
  bool center_predicted = false;
  int center_x = -1;
};

// 仅使用 road=1。barrier=2 与其他类别全部视为非路面。
class Tracker {
public:
  explicit Tracker(const Options & options = Options{});
  Result estimate(const uint8_t * mask, int width, int height, float near_row_ratio,
                  uint8_t * processed_mask = nullptr);
  void reset();
private:
  Options options_;
  bool have_center_line_ = false;
  float previous_center_slope_ = 0.0f;
  float previous_center_intercept_ = 0.0f;
  int center_lost_frames_ = 0;
};

const char * status_name(Status status);
}  // namespace road_boundary
