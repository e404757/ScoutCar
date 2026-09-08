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
  int deviation = -999;
  int y = 0;
  int left = -1;
  int right = -1;
  int width = 0;
  int center_x = -1;
  bool used_barrier_gap = false;
  float selected_pt = 0.0f;
  int min_width = 0;
  int max_width = 0;
};

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
