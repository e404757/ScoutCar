#pragma once

#include <cstdint>

#include <opencv2/core/mat.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "scoutcar_perception/road_tracker.hpp"

namespace scoutcar_perception {

class PerceptionVisualizer {
public:
  sensor_msgs::msg::Image make_source_debug(
      const sensor_msgs::msg::Image & source,
      const uint8_t * mask,
      const uint8_t * processed_ipm_mask,
      const road_tracking::Result & result,
      const cv::Mat & ipm_inverse,
      double inference_fps,
      bool deviation_enabled) const;

  sensor_msgs::msg::Image make_ipm_debug(
      const sensor_msgs::msg::Image & source,
      const uint8_t * mask,
      const uint8_t * processed_mask,
      const road_tracking::Result & result) const;

private:
  static void draw_inference_fps(cv::Mat & image, double inference_fps);
  static void draw_driving_state(cv::Mat & image, bool deviation_enabled);
  static void draw_deviation(
      cv::Mat & image, const road_tracking::Result & result);
  static void draw_frame_info(
      cv::Mat & image, double inference_fps, bool deviation_enabled);
  static sensor_msgs::msg::Image make_rgb8_debug(
      const sensor_msgs::msg::Image & source, const cv::Mat & bgr);
};

}  // namespace scoutcar_perception
