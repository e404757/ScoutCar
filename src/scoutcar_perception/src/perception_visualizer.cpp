#include "scoutcar_perception/perception_visualizer.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace scoutcar_perception {

void PerceptionVisualizer::draw_deviation(cv::Mat &image,
                                          const road_tracking::Result &result,
                                          int reference_x,
                                          const cv::Mat &coordinate_transform) {
  if (!result.valid || image.empty()) {
    return;
  }

  std::vector<cv::Point2f> points{
      {static_cast<float>(reference_x), static_cast<float>(result.y)},
      {static_cast<float>(result.center_x), static_cast<float>(result.y)}};
  if (!coordinate_transform.empty()) {
    cv::perspectiveTransform(points, points, coordinate_transform);
  }

  const cv::Point reference_point{
      std::clamp(cvRound(points[0].x), 0, image.cols - 1),
      std::clamp(cvRound(points[0].y), 0, image.rows - 1)};
  const cv::Point road_center_point{
      std::clamp(cvRound(points[1].x), 0, image.cols - 1),
      std::clamp(cvRound(points[1].y), 0, image.rows - 1)};

  cv::line(image, reference_point, road_center_point, {0, 255, 255}, 3,
           cv::LINE_AA);
  cv::drawMarker(image, reference_point, {255, 255, 255},
                 cv::MARKER_CROSS, 11, 2, cv::LINE_AA);
  cv::drawMarker(image, road_center_point, {0, 255, 0},
                 cv::MARKER_CROSS, 11, 2, cv::LINE_AA);

  const std::string text = "DEV " + std::to_string(result.deviation) + " px";
  int baseline = 0;
  const cv::Size text_size = cv::getTextSize(
      text, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline);
  const cv::Point line_middle = (reference_point + road_center_point) * 0.5;
  const int text_x =
      std::clamp(line_middle.x - text_size.width / 2, 0,
                 std::max(0, image.cols - text_size.width));
  const int text_y = std::clamp(line_middle.y - 10, text_size.height,
                                image.rows - 1);
  cv::putText(image, text, {text_x, text_y}, cv::FONT_HERSHEY_SIMPLEX,
              0.7, {0, 255, 255}, 2, cv::LINE_AA);
}

sensor_msgs::msg::Image PerceptionVisualizer::make_mask_debug(
    const sensor_msgs::msg::Image &source, const uint8_t *mask,
    const uint8_t *processed_ipm_mask, const road_tracking::Result &result,
    int reference_x, const cv::Mat &ipm_inverse) const {
  const int width = static_cast<int>(source.width);
  const int height = static_cast<int>(source.height);
  cv::Mat rgb(height, width, CV_8UC3,
              const_cast<uint8_t *>(source.data.data()));
  cv::Mat bgr;
  cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);

  if (mask == nullptr) {
    draw_deviation(bgr, result, reference_x, ipm_inverse);
    return make_rgb8_debug(source, bgr);
  }

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const uint8_t value = mask[static_cast<size_t>(y) * width + x];
      cv::Vec3b &pixel = bgr.at<cv::Vec3b>(y, x);
      if (value == 1) {
        pixel = cv::Vec3b(0, 255, 0);
      } else if (value == 2) {
        pixel = cv::Vec3b(0, 0, 255);
      }
    }
  }

  cv::Mat processed_source;
  if (processed_ipm_mask) {
    cv::Mat processed_ipm(height, width, CV_8UC1,
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

  draw_deviation(bgr, result, reference_x, ipm_inverse);
  return make_rgb8_debug(source, bgr);
}

sensor_msgs::msg::Image PerceptionVisualizer::make_ipm_debug(
    const sensor_msgs::msg::Image &source, const uint8_t *mask,
    const uint8_t *processed_mask, const road_tracking::Result &result,
    int reference_x) const {
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
  }

  draw_deviation(bgr, result, reference_x, cv::Mat{});

  return make_rgb8_debug(source, bgr);
}

sensor_msgs::msg::Image PerceptionVisualizer::make_detection_debug(
    const sensor_msgs::msg::Image &source,
    const DetectionResult &result) const {
  const int width = static_cast<int>(source.width);
  const int height = static_cast<int>(source.height);
  cv::Mat rgb(height, width, CV_8UC3,
              const_cast<uint8_t *>(source.data.data()),
              static_cast<size_t>(source.step));
  cv::Mat bgr;
  cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);

  for (const auto &detection : result.detections) {
    const int left = std::clamp(detection.left, 0, width - 1);
    const int top = std::clamp(detection.top, 0, height - 1);
    const int right = std::clamp(detection.right, 0, width - 1);
    const int bottom = std::clamp(detection.bottom, 0, height - 1);
    if (right <= left || bottom <= top) {
      continue;
    }

    const cv::Scalar color(0, 255, 0);
    cv::rectangle(bgr, {left, top}, {right, bottom}, color, 2, cv::LINE_AA);

    char confidence[16];
    std::snprintf(confidence, sizeof(confidence), "%.1f%%",
                  detection.confidence * 100.0F);
    const std::string label =
        (detection.class_name.empty() ? "unknown" : detection.class_name) +
        " " + confidence;

    int baseline = 0;
    const double font_scale = 0.55;
    const int thickness = 1;
    const cv::Size text_size = cv::getTextSize(
        label, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
    const int text_x = left;
    const int text_y =
        top > text_size.height + baseline + 4
            ? top - 4
            : std::min(height - baseline - 1, top + text_size.height + 4);
    const int background_top =
        std::max(0, text_y - text_size.height - baseline - 3);
    const int background_right =
        std::min(width - 1, text_x + text_size.width + 4);
    const int background_bottom = std::min(height - 1, text_y + baseline + 1);

    cv::rectangle(bgr, {text_x, background_top},
                  {background_right, background_bottom}, color, cv::FILLED);
    cv::putText(bgr, label, {text_x + 2, text_y},
                cv::FONT_HERSHEY_SIMPLEX, font_scale, {0, 0, 0}, thickness,
                cv::LINE_AA);
  }

  return make_rgb8_debug(source, bgr);
}

sensor_msgs::msg::Image
PerceptionVisualizer::make_rgb8_debug(const sensor_msgs::msg::Image &source,
                                      const cv::Mat &bgr) {
  cv::Mat debug_rgb;
  cv::cvtColor(bgr, debug_rgb, cv::COLOR_BGR2RGB);
  sensor_msgs::msg::Image debug;
  debug.header = source.header;
  debug.width = source.width;
  debug.height = source.height;
  debug.encoding = "rgb8";
  debug.step = source.width * 3;
  debug.data.assign(debug_rgb.data,
                    debug_rgb.data + debug_rgb.total() * debug_rgb.elemSize());
  return debug;
}

} // namespace scoutcar_perception
