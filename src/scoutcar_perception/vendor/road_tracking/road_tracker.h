#pragma once

#include "road_boundary.h"
#include "legacy_width_tracker.h"

#include <cstdint>
#include <memory>
#include <string>

namespace road_tracking {

struct PreviewConfig {
  float row_ratio = 0.8f;
  int edge_margin_px = 10;
  int expected_width = 290;
  int min_width = 250;
  int max_width = 330;
};

struct Config {
  PreviewConfig near;
  PreviewConfig far{0.4f, 10, 86, 70, 110};
  std::string tracking_mode = "robust_center";
  int max_inline_gap_px = 8;
  int min_run_width_px = 12;
  int boundary_jump_px = 36;
  int reconnect_tolerance_px = 20;
  int reconnect_rows = 2;
  int local_fit_points = 10;
  float center_fit_top_ratio = 0.72f;
  float center_fit_bottom_ratio = 0.90f;
  int center_edge_margin_px = 10;
  int center_min_width_px = 40;
  int center_min_points = 30;
  int center_inlier_px = 8;
  float center_min_inlier_ratio = 0.60f;
  int center_hold_frames = 4;
  float center_smoothing = 0.35f;
  int center_max_jump_px = 50;
  float center_max_slope_jump = 0.35f;
  float center_max_abs_slope = 0.35f;
  int legacy_overlap_margin_px = 14;
  int legacy_max_missing_rows = 6;
  int legacy_boundary_stable_px = 45;
  float legacy_bottom_seed_ratio = 0.85f;
  bool legacy_use_barrier_gap = true;
};

enum class Mode { AUTO_NEAR, AUTO_FAR, AUTO_NONE };

struct Result {
  road_boundary::Result boundary;
  float selected_pt = 0.8f;
  Mode mode = Mode::AUTO_NEAR;
};

class RoadTracker {
public:
  explicit RoadTracker(const Config & config = Config{});
  Result process(const uint8_t * mask, int width, int height,
                 uint8_t * processed_mask = nullptr);
  void reset();

private:
  Config config_;
  road_boundary::Tracker boundary_tracker_;
  std::unique_ptr<LegacyWidthTracker> legacy_tracker_;
};

const char * mode_name(Mode mode);

}  // namespace road_tracking
