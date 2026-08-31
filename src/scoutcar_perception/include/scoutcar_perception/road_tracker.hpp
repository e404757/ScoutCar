#pragma once

#include <cstdint>

namespace road_tracking {

struct PreviewConfig {
  float row_ratio = 0.8f;
  int edge_margin_px = 10;
  int min_width = 250;
  int max_width = 330;
};

struct Config {
  PreviewConfig near;
  PreviewConfig far{0.4f, 10, 70, 110};
  int max_inline_gap_px = 8;
  int min_run_width_px = 12;
  int overlap_margin_px = 14;
  int max_missing_rows = 6;
  float bottom_seed_ratio = 0.85f;
  bool use_barrier_gap = true;
};

enum class Preview { NEAR, FAR, NONE };

struct Result {
  bool valid = false;
  int deviation = -999;
  int y = 0;
  int left = -1;
  int right = -1;
  int width = 0;
  int center_x = -1;
  bool used_barrier_gap = false;
  float selected_pt = 0.8f;
  Preview preview = Preview::NONE;
  int min_width = 0;
  int max_width = 0;
};

class RoadTracker {
public:
  explicit RoadTracker(const Config & config = Config{});
  Result process(const uint8_t * mask, int width, int height,
                 uint8_t * processed_mask = nullptr) const;

private:
  struct Estimate;
  Estimate estimate(const uint8_t * mask, int width, int height,
                    const PreviewConfig & preview,
                    uint8_t * processed_mask) const;
  Config config_;
};

const char * preview_name(Preview preview);

}  // namespace road_tracking
