#include "road_tracker.h"

namespace road_tracking {
namespace {
road_boundary::Options boundary_options(const Config & config) {
  road_boundary::Options options;
  options.max_inline_gap = config.max_inline_gap_px;
  options.min_run_width = config.min_run_width_px;
  options.far_row_ratio = config.far.row_ratio;
  options.boundary_jump_px = config.boundary_jump_px;
  options.reconnect_tolerance_px = config.reconnect_tolerance_px;
  options.reconnect_rows = config.reconnect_rows;
  options.local_fit_points = config.local_fit_points;
  options.center_fit_top_ratio = config.center_fit_top_ratio;
  options.center_fit_bottom_ratio = config.center_fit_bottom_ratio;
  options.center_edge_margin = config.center_edge_margin_px;
  options.center_min_width = config.center_min_width_px;
  options.center_min_points = config.center_min_points;
  options.center_inlier_px = config.center_inlier_px;
  options.center_min_inlier_ratio = config.center_min_inlier_ratio;
  options.center_hold_frames = config.center_hold_frames;
  options.center_smoothing = config.center_smoothing;
  options.center_max_jump_px = config.center_max_jump_px;
  options.center_max_slope_jump = config.center_max_slope_jump;
  options.center_max_abs_slope = config.center_max_abs_slope;
  return options;
}
}  // namespace

RoadTracker::RoadTracker(const Config & config)
: config_(config), boundary_tracker_(boundary_options(config)) {
  LegacyConfig legacy;
  legacy.near={config.near.row_ratio,config.near.expected_width,config.near.min_width,
               config.near.max_width,config.near.edge_margin_px};
  legacy.far={config.far.row_ratio,config.far.expected_width,config.far.min_width,
              config.far.max_width,config.far.edge_margin_px};
  legacy.max_inline_gap=config.max_inline_gap_px;
  legacy.min_run_width=config.min_run_width_px;
  legacy.overlap_margin=config.legacy_overlap_margin_px;
  legacy.max_missing_rows=config.legacy_max_missing_rows;
  legacy.boundary_stable=config.legacy_boundary_stable_px;
  legacy.bottom_seed_ratio=config.legacy_bottom_seed_ratio;
  legacy.use_barrier_gap=config.legacy_use_barrier_gap;
  legacy_tracker_=std::make_unique<LegacyWidthTracker>(legacy);
}

void RoadTracker::reset() { boundary_tracker_.reset();legacy_tracker_->reset(); }

Result RoadTracker::process(const uint8_t * mask, int width, int height,
                            uint8_t * processed_mask) {
  Result result;
  result.selected_pt = config_.near.row_ratio;
  if(config_.tracking_mode=="legacy_width_barrier") {
    result.boundary=legacy_tracker_->process(mask,width,height,processed_mask);
    result.selected_pt=result.boundary.y>0?static_cast<float>(result.boundary.y)/height
                                        :config_.near.row_ratio;
    result.mode=result.boundary.valid?Mode::AUTO_NEAR:Mode::AUTO_NONE;
    if(!result.boundary.valid) result.boundary.deviation=-999;
    return result;
  }
  result.boundary = boundary_tracker_.estimate(
    mask, width, height, config_.near.row_ratio, processed_mask);
  if (!result.boundary.valid) {
    result.mode = Mode::AUTO_NONE;
    result.boundary.deviation = -999;
  } else if (result.boundary.status == road_boundary::Status::RECONSTRUCTED) {
    result.mode = Mode::AUTO_FAR;
  } else {
    result.mode = Mode::AUTO_NEAR;
  }
  return result;
}

const char * mode_name(Mode mode) {
  switch (mode) {
    case Mode::AUTO_NEAR: return "AUTO_NEAR";
    case Mode::AUTO_FAR: return "RECONSTRUCTED";
    case Mode::AUTO_NONE: return "AUTO_NONE";
  }
  return "AUTO_UNKNOWN";
}
}  // namespace road_tracking
