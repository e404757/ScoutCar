#pragma once

#include <cstdint>
#include <optional>

#include <opencv2/core.hpp>

#include <scoutcar_msgs/msg/mission_status.hpp>
#include <scoutcar_msgs/msg/road_boundary.hpp>

namespace scoutcar_web {

// 在 RGB888 帧上叠加绘制诊断信息（web 推流画面与 _mask 录像共用同一绘制逻辑）：
//   1. 分割掩膜半透明叠加：路面(1)=绿色、挡板(2)=红色，alpha 0.5（与原工程 main.cc 一致）；
//   2. 道路边界：扫描线(y=scan_y)黄色横线、画面中心白竖线、
//      中心十字(白) + 道路中心十字(绿) + "Dev:±N" 文字（boundary.valid 时）；
//   3. 任务状态文字（左下）：等待/运行中/完成 + 固定剩余/随机剩余/当前格点；
//   4. FPS（左上）。
//
// 注意：就地修改 rgb（调用方务必传入副本，原始帧要留给原始录像/导出）。
void draw_overlay(
  cv::Mat & rgb,
  const uint8_t * seg_mask,                 // mono8（0背景/1路面/2挡板），与帧同尺寸；可空
  const scoutcar_msgs::msg::RoadBoundary & boundary,
  const std::optional<scoutcar_msgs::msg::MissionStatus> & status,
  float fps);

}  // namespace scoutcar_web
