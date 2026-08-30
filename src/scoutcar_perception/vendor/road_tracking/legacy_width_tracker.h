#pragma once

#include "road_boundary.h"

#include <cstdint>

namespace road_tracking {

struct LegacyPreview {
  float row_ratio;
  int expected_width;
  int min_width;
  int max_width;
  int edge_margin;
};

struct LegacyConfig {
  LegacyPreview near{0.8f,290,250,330,10};
  LegacyPreview far{0.4f,86,70,110,10};
  int max_inline_gap=8;
  int min_run_width=12;
  int overlap_margin=14;
  int max_missing_rows=6;
  int boundary_stable=45;
  float bottom_seed_ratio=0.85f;
  bool use_barrier_gap=true;
};

// 旧工程 f977e72 的"固定道路宽度 + 两侧挡板间隙"策略。
class LegacyWidthTracker {
public:
  explicit LegacyWidthTracker(const LegacyConfig & config);
  road_boundary::Result process(const uint8_t * mask,int width,int height,
                                uint8_t * processed_mask=nullptr);
  void reset();
private:
  struct State { bool valid=false; int left=0; int right=0; };
  struct Estimate;
  Estimate estimate(const uint8_t * mask,int width,int height,
                    const LegacyPreview & preview,State & state,
                    uint8_t * processed_mask);
  LegacyConfig config_;
  State near_state_;
  State far_state_;
};

}  // namespace road_tracking
