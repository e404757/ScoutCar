#include "scoutcar_perception/perception_visualizer.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace scoutcar_perception {

void PerceptionVisualizer::draw_inference_fps(
    cv::Mat & image, double inference_fps) {
  char text[32];
  std::snprintf(text, sizeof(text), "FPS %.1f", inference_fps);
  cv::putText(image, text, {12, 30}, cv::FONT_HERSHEY_SIMPLEX,
              0.8, {255, 255, 255}, 2, cv::LINE_AA);
}

void PerceptionVisualizer::draw_driving_state(
    cv::Mat & image, bool deviation_enabled) {
  const std::string text = deviation_enabled ? "AHEADING" : "TURNING";
  const cv::Scalar color = deviation_enabled
                               ? cv::Scalar(0, 255, 0)
                               : cv::Scalar(0, 0, 255);
  const double font_scale = 0.8;
  const int thickness = 2;
  int baseline = 0;
  const cv::Size text_size = cv::getTextSize(
      text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
  const int x = (image.cols - text_size.width) / 2;

  cv::putText(image, text, {x, 30}, cv::FONT_HERSHEY_SIMPLEX,
              font_scale, color, thickness, cv::LINE_AA);
}

void PerceptionVisualizer::draw_deviation(
    cv::Mat & image, const road_tracking::Result & result) {
  const std::string text = result.valid
                               ? "DEV " + std::to_string(result.deviation) + " px"
                               : "DEV N/A";
  cv::putText(image, text, {12, 60}, cv::FONT_HERSHEY_SIMPLEX,
              0.7, {0, 255, 255}, 2, cv::LINE_AA);
}

void PerceptionVisualizer::draw_frame_info(
    cv::Mat & image, double inference_fps, bool deviation_enabled) {
  draw_inference_fps(image, inference_fps);
  draw_driving_state(image, deviation_enabled);
}

sensor_msgs::msg::Image PerceptionVisualizer::make_source_debug(
    const sensor_msgs::msg::Image & source,
    const uint8_t * mask,
    const uint8_t * processed_ipm_mask,
    const road_tracking::Result & result,
    const cv::Mat & ipm_inverse,
    double inference_fps,
    bool deviation_enabled) const {
  const int width = static_cast<int>(source.width);
  const int height = static_cast<int>(source.height);
  cv::Mat rgb(height, width, CV_8UC3,
              const_cast<uint8_t *>(source.data.data()));
  cv::Mat bgr;
  cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);

  if (mask == nullptr) {
    draw_frame_info(bgr, inference_fps, deviation_enabled);
    draw_deviation(bgr, result);
    return make_rgb8_debug(source, bgr);
  }

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const uint8_t value = mask[static_cast<size_t>(y) * width + x];
      cv::Vec3b & pixel = bgr.at<cv::Vec3b>(y, x);
      if (value == 1) {
        pixel = cv::Vec3b(0, 255, 0);
      } else if (value == 2) {
        pixel = cv::Vec3b(0, 0, 255);
      }
    }
  }

  cv::Mat processed_source;
  if (processed_ipm_mask) {
    cv::Mat processed_ipm(
        height, width, CV_8UC1,
        const_cast<uint8_t *>(processed_ipm_mask));
    cv::warpPerspective(processed_ipm, processed_source, ipm_inverse,
                        cv::Size(width, height), cv::INTER_NEAREST,
                        cv::BORDER_CONSTANT, cv::Scalar(0));

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (processed_source.at<uint8_t>(y, x)) {
          bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 0);
        }
      }
    }
  }

  const int y = std::clamp(result.y, 0, height - 1);
  const int cx = width / 2;
  std::vector<cv::Point2f> bird_points{
    {0.0f, static_cast<float>(y)},
    {static_cast<float>(width - 1), static_cast<float>(y)},
    {static_cast<float>(cx), 0.0f},
    {static_cast<float>(cx), static_cast<float>(height - 1)}};
  cv::perspectiveTransform(bird_points, bird_points, ipm_inverse);
  cv::line(bgr, bird_points[0], bird_points[1], {0, 255, 255}, 1);
  cv::line(bgr, bird_points[2], bird_points[3], {255, 255, 255}, 1);

  if (result.left >= 0 && result.right >= 0) {
    std::vector<cv::Point2f> selected{
      {static_cast<float>(result.left), static_cast<float>(y)},
      {static_cast<float>(result.right), static_cast<float>(y)},
      {static_cast<float>(result.center_x), static_cast<float>(y)},
      {static_cast<float>(cx), static_cast<float>(y)}};
    cv::perspectiveTransform(selected, selected, ipm_inverse);
    cv::line(bgr, selected[0], selected[1], {0, 255, 255}, 2);
    cv::drawMarker(
        bgr, selected[2], {0, 255, 0}, cv::MARKER_CROSS, 11, 2);
    cv::drawMarker(
        bgr, selected[3], {255, 255, 255}, cv::MARKER_CROSS, 11, 2);
  }

  draw_frame_info(bgr, inference_fps, deviation_enabled);
  draw_deviation(bgr, result);
  return make_rgb8_debug(source, bgr);
}

sensor_msgs::msg::Image PerceptionVisualizer::make_ipm_debug(
    const sensor_msgs::msg::Image & source,
    const uint8_t * mask,
    const uint8_t * processed_mask,
    const road_tracking::Result & result) const {
  const int width = static_cast<int>(source.width);
  const int height = static_cast<int>(source.height);
  cv::Mat bgr(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

  if (mask != nullptr) {
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const size_t index = static_cast<size_t>(y) * width + x;
        if (mask[index] == 1) {
          bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 100, 0);
        } else if (mask[index] == 2) {
          bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 130);
        }
        if (processed_mask && processed_mask[index]) {
          bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 0);
        }
      }
    }

    const int y = std::clamp(result.y, 0, height - 1);
    const int cx = width / 2;
    cv::line(bgr, {0, y}, {width - 1, y}, {0, 255, 255}, 1);
    cv::line(bgr, {cx, 0}, {cx, height - 1}, {255, 255, 255}, 1);
    if (result.left >= 0 && result.right >= 0) {
      cv::line(
          bgr, {result.left, y}, {result.right, y}, {0, 255, 255}, 3);
      cv::drawMarker(bgr, {result.center_x, y}, {0, 255, 0},
                     cv::MARKER_CROSS, 11, 2);
    }
  }

  draw_deviation(bgr, result);

  return make_rgb8_debug(source, bgr);
}

sensor_msgs::msg::Image PerceptionVisualizer::make_rgb8_debug(
    const sensor_msgs::msg::Image & source, const cv::Mat & bgr) {
  cv::Mat debug_rgb;
  cv::cvtColor(bgr, debug_rgb, cv::COLOR_BGR2RGB);
  sensor_msgs::msg::Image debug;
  debug.header = source.header;
  debug.width = source.width;
  debug.height = source.height;
  debug.encoding = "rgb8";
  debug.step = source.width * 3;
  debug.data.assign(
      debug_rgb.data,
      debug_rgb.data + debug_rgb.total() * debug_rgb.elemSize());
  return debug;
}

}  // namespace scoutcar_perception
