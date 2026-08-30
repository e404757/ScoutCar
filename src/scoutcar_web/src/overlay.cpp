#include "overlay.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace scoutcar_web {

void draw_overlay(
  cv::Mat & rgb,
  const uint8_t * seg_mask,
  const scoutcar_msgs::msg::RoadBoundary & boundary,
  const std::optional<scoutcar_msgs::msg::MissionStatus> & status,
  float fps)
{
  const int width = rgb.cols;
  const int height = rgb.rows;

  // ── 1) 掩膜半透明叠加（与原工程 main.cc 完全一致：路面绿 / 挡板红，alpha 0.5）──
  if (seg_mask != nullptr) {
    constexpr float alpha = 0.5f;
    uint8_t * ori = rgb.data;
    for (int j = 0; j < height; ++j) {
      const uint8_t * row = seg_mask + static_cast<size_t>(j) * width;
      for (int k = 0; k < width; ++k) {
        const uint8_t v = row[k];
        unsigned char r = 0, g = 0, b = 0;
        if (v == 1) { g = 255; }               // road
        else if (v == 2) { r = 255; }          // barrier
        else { continue; }
        const size_t off = 3 * (static_cast<size_t>(j) * width + k);
        // bgr 顺序：off+0=B, off+1=G, off+2=R（早期版本把 r/b 写反，导致挡板呈蓝色）
        ori[off + 0] = static_cast<uint8_t>(b * alpha + ori[off + 0] * (1 - alpha));
        ori[off + 1] = static_cast<uint8_t>(g * alpha + ori[off + 1] * (1 - alpha));
        ori[off + 2] = static_cast<uint8_t>(r * alpha + ori[off + 2] * (1 - alpha));
      }
    }
  }

  const int cx = width / 2;
  const int y_target = boundary.scan_y;

  // ── 2) 逐行边界：橙=mask 原始边界，青/紫=最终边界，红=局部补线。──
  if (boundary.valid && boundary.fit_bottom_y >= boundary.fit_top_y) {
    const int top = std::clamp(static_cast<int>(boundary.fit_top_y), 0, height - 1);
    auto draw_polyline = [&](const auto & xs, const auto * repaired,
                             const cv::Scalar & normal_color, int thickness) {
      for (size_t i = 1; i < xs.size(); ++i) {
        if (xs[i - 1] < 0 || xs[i] < 0) { continue; }
        const int y0 = top + static_cast<int>(i) - 1;
        const int y1 = top + static_cast<int>(i);
        if (y1 >= height) { break; }
        cv::Scalar color = normal_color;
        if (repaired != nullptr && i < repaired->size() &&
            ((*repaired)[i - 1] != 0 || (*repaired)[i] != 0)) {
          color = cv::Scalar(0, 0, 255);
        }
        cv::line(rgb, cv::Point(xs[i - 1], y0), cv::Point(xs[i], y1),
                 color, thickness, cv::LINE_AA);
      }
    };
    draw_polyline(boundary.raw_left_boundary, static_cast<const std::vector<uint8_t> *>(nullptr), cv::Scalar(0, 165, 255), 1);
    draw_polyline(boundary.raw_right_boundary, static_cast<const std::vector<uint8_t> *>(nullptr), cv::Scalar(0, 165, 255), 1);
    draw_polyline(boundary.fitted_left_boundary, &boundary.left_repaired,
                  cv::Scalar(255, 255, 0), 2);
    draw_polyline(boundary.fitted_right_boundary, &boundary.right_repaired,
                  cv::Scalar(255, 0, 255), 2);
    draw_polyline(boundary.center_boundary,
                  static_cast<const std::vector<uint8_t> *>(nullptr),
                  boundary.center_predicted ? cv::Scalar(0, 0, 255)
                                            : cv::Scalar(0, 255, 0), 3);
    char fit_text[64];
    snprintf(fit_text, sizeof(fit_text), "ROAD:%s%s C:%u/%u%s",
             boundary.left_reconstructed ? " L-FIX" : "",
             boundary.right_reconstructed ? " R-FIX" : "",
             boundary.center_inlier_count, boundary.center_candidate_count,
             boundary.center_predicted ? " HOLD" : "");
    cv::putText(rgb, fit_text, cv::Point(12, 48), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                (boundary.left_reconstructed || boundary.right_reconstructed)
                  ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  }

  // ── 3) 道路边界诊断线 ──
  if (boundary.valid && y_target > 0 && y_target < height) {
    const int road_center_x = static_cast<int>(boundary.road_center);
    // 扫描线：画面中心 → 道路中心（黄）
    cv::line(rgb, cv::Point(cx, y_target), cv::Point(road_center_x, y_target),
             cv::Scalar(0, 255, 255), 2);
    // 画面中心十字（白）
    constexpr int cross = 5;
    cv::line(rgb, cv::Point(cx - cross, y_target), cv::Point(cx + cross, y_target),
             cv::Scalar(255, 255, 255), 2);
    cv::line(rgb, cv::Point(cx, y_target - cross), cv::Point(cx, y_target + cross),
             cv::Scalar(255, 255, 255), 2);
    // 道路中心十字（绿）
    cv::line(rgb, cv::Point(road_center_x - cross, y_target),
             cv::Point(road_center_x + cross, y_target), cv::Scalar(0, 255, 0), 2);
    cv::line(rgb, cv::Point(road_center_x, y_target - cross),
             cv::Point(road_center_x, y_target + cross), cv::Scalar(0, 255, 0), 2);
    // Dev 文字
    char dev_text[32];
    snprintf(dev_text, sizeof(dev_text), "Dev:%d", boundary.deviation);
    cv::putText(rgb, dev_text,
                cv::Point((cx + road_center_x) / 2 - 20, y_target - 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
  } else if (y_target > 0 && y_target < height) {
    // 无有效边界：仍画扫描线提示当前预瞄行
    cv::line(rgb, cv::Point(0, y_target), cv::Point(width - 1, y_target),
             cv::Scalar(0, 255, 255), 1);
    cv::line(rgb, cv::Point(cx, 0), cv::Point(cx, height - 1),
             cv::Scalar(255, 255, 255), 1);
  }

  // ── 3) 任务状态文字（左下）──
  if (status.has_value()) {
    // OpenCV Hershey 字体只支持 ASCII，中文会渲染成问号，这里只能用英文
    const char * state_str = "WAIT";
    if (status->state == 1) { state_str = "RUN"; }
    else if (status->state == 2) { state_str = "DONE"; }
    char text[128];
    snprintf(text, sizeof(text),
             "Task:%s fixed:%d rand:%d node:%d",
             state_str, status->fixed_remain, status->random_remain,
             status->current_node);
    cv::putText(rgb, text, cv::Point(12, height - 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  }

  // ── 4) FPS（左上）──
  char fps_text[32];
  snprintf(fps_text, sizeof(fps_text), "FPS: %.1f", fps);
  cv::putText(rgb, fps_text, cv::Point(12, 24),
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

}  // namespace scoutcar_web
