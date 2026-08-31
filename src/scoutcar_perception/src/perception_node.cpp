// perception_node —— 感知节点（yolov5_seg 分割）
//
// 对应原工程：
//   - perception/yolov5_seg（算法库，已原样搬入 vendor/）
//   - main.cc 的"读帧 → 推理 → 掩膜"段
//
// 数据流：
//   可配置相机话题 ──▶ [image_buffer_t → 推理] ──▶ /perception/seg_mask
//
// 说明：
//   - 不依赖 cv_bridge：rgb8 的字节布局就是 RGB888，直接 memcpy 进 image_buffer_t，
//     拷贝一份避免推理触碰共享消息；
//   - 推理走 inference_yolov5_seg_model_cpu：输入是普通内存，用 CPU 预处理版本，
//     避免 RGA importbuffer_virtualaddr 崩溃（原工程官方回避方案）。

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include <opencv2/opencv.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <scoutcar_msgs/msg/road_boundary.hpp>

#include "image_utils.h"
#include "scoutcar_perception/road_tracker.hpp"
#include "yolov5_seg.h"

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode() : Node("perception_node")
  {
    const auto package_share =
      ament_index_cpp::get_package_share_directory("scoutcar_perception");
    model_path_ = declare_parameter<std::string>(
      "model_path", package_share + "/models/V1.0.rknn");
    label_path_ = declare_parameter<std::string>(
      "label_path", package_share + "/resources/detect_label.txt");
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/camera/mipi/image_raw");

    road_tracking::Config tracking_config;
    tracking_config.near.row_ratio = declare_parameter<double>("preview.near.row_ratio", 0.8);
    tracking_config.near.edge_margin_px =
      declare_parameter<int>("preview.near.edge_margin_px", 10);
    tracking_config.near.min_width =
      declare_parameter<int>("preview.near.min_width", 250);
    tracking_config.near.max_width =
      declare_parameter<int>("preview.near.max_width", 330);
    tracking_config.far.row_ratio = declare_parameter<double>("preview.far.row_ratio", 0.4);
    tracking_config.far.edge_margin_px =
      declare_parameter<int>("preview.far.edge_margin_px", 10);
    tracking_config.far.min_width =
      declare_parameter<int>("preview.far.min_width", 70);
    tracking_config.far.max_width =
      declare_parameter<int>("preview.far.max_width", 110);
    tracking_config.max_inline_gap_px =
      declare_parameter<int>("road_tracking.max_inline_gap_px", 8);
    tracking_config.min_run_width_px =
      declare_parameter<int>("road_tracking.min_run_width_px", 12);
    tracking_config.overlap_margin_px =
      declare_parameter<int>("road_tracking.overlap_margin_px", 14);
    tracking_config.max_missing_rows =
      declare_parameter<int>("road_tracking.max_missing_rows", 6);
    tracking_config.bottom_seed_ratio =
      declare_parameter<double>("road_tracking.bottom_seed_ratio", 0.85);
    tracking_config.use_barrier_gap =
      declare_parameter<bool>("road_tracking.use_barrier_gap", true);
    road_tracker_ = std::make_unique<road_tracking::RoadTracker>(tracking_config);

    pub_seg_ = create_publisher<sensor_msgs::msg::Image>("perception/seg_mask", 10);
    pub_source_ = create_publisher<sensor_msgs::msg::Image>("perception/source_image", 5);
    pub_debug_ = create_publisher<sensor_msgs::msg::Image>("perception/debug_image", 5);
    pub_boundary_ = create_publisher<scoutcar_msgs::msg::RoadBoundary>(
      "perception/road_boundary", 10);
    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::QoS(1).best_effort(),   // 与 camera_node 的 QoS 匹配；depth=1 只留最新帧
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { process_frame(msg); });
    sub_deviation_enable_ = create_subscription<std_msgs::msg::Bool>(
      "mission/deviation_enable", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        const bool changed = tracking_enabled_ != msg->data;
        if (!changed) return;  // 直行路口会重复发 true，不应重置正常跟踪。
        tracking_enabled_ = msg->data;
        RCLCPP_INFO(get_logger(), "循迹%s",
                    tracking_enabled_ ? "准备恢复" : "已关闭");
      });

    if (init_post_process(label_path_.c_str()) != 0) {
      RCLCPP_ERROR(get_logger(), "类别标签加载失败: %s", label_path_.c_str());
      return;
    }
    post_process_ok_ = true;
    if (init_yolov5_seg_model(model_path_.c_str(), &rknn_ctx_) != 0) {
      RCLCPP_ERROR(get_logger(), "模型加载失败: %s", model_path_.c_str());
      model_ok_ = false;
    } else {
      model_ok_ = true;
      RCLCPP_INFO(get_logger(), "模型加载成功");
    }
  }

  ~PerceptionNode() override
  {
    if (model_ok_) {
      release_yolov5_seg_model(&rknn_ctx_);
    }
    if (post_process_ok_) {
      deinit_post_process();
    }
  }

private:
  void process_frame(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!model_ok_) {
      return;
    }
    // 1) sensor_msgs/Image(rgb8) → image_buffer_t（拷贝一份，推理不碰共享消息）
    const size_t bytes = static_cast<size_t>(msg->width) * msg->height * 3;
    if (msg->encoding != "rgb8" || msg->data.size() < bytes) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "图像格式异常: enc=%s %ux%u",
                           msg->encoding.c_str(), msg->width, msg->height);
      return;
    }
    uint8_t * frame = static_cast<uint8_t *>(std::malloc(bytes));
    if (frame == nullptr) {
      return;
    }
    std::memcpy(frame, msg->data.data(), bytes);

    image_buffer_t img{};
    img.width = static_cast<int>(msg->width);
    img.height = static_cast<int>(msg->height);
    img.format = IMAGE_FORMAT_RGB888;
    img.virt_addr = frame;

    // 2) 推理（CPU 预处理）
    object_detect_result_list od{};
    const int ret = inference_yolov5_seg_model_cpu(&rknn_ctx_, &img, &od);
    std::free(frame);
    if (ret != 0) {
      return;
    }
    if (od.count < 1) {
      publish_invalid_boundary();
      // 无检测帧也要发布干净画面（不带掩膜）：旧工程每帧都推流；
      // 若这里直接 return，模型没检测到道路时 web 实时流会冻结（卡顿根因）。
      road_tracking::Result empty_tracked{};
      publish_debug_images(*msg, nullptr, nullptr, empty_tracked);
      return;
    }

    // 3) 掩膜 → sensor_msgs/Image（mono8：0背景/1路面/2挡板）
    uint8_t * seg_mask = od.results_seg[0].seg_mask;
    auto out = std::make_shared<sensor_msgs::msg::Image>();
    out->header = msg->header;              // 继承相机帧时间戳
    out->header.frame_id = "camera";
    out->width = img.width;
    out->height = img.height;
    out->encoding = "mono8";
    out->step = static_cast<uint32_t>(img.width);
    out->data.assign(seg_mask, seg_mask + static_cast<size_t>(img.width) * img.height);

    // 4) 使用 RoadTracker 的近/远预瞄和连通道路判定。
    std::vector<uint8_t> processed_mask(static_cast<size_t>(img.width)*img.height,0);
    const auto tracked = road_tracker_->process(
      seg_mask, img.width, img.height, processed_mask.data());
    const auto & result = tracked;
    scoutcar_msgs::msg::RoadBoundary boundary;
    boundary.header = msg->header;
    boundary.valid = result.valid && tracking_enabled_;
    boundary.deviation = boundary.valid ? static_cast<int16_t>(result.deviation) : -999;
    // road_center 是诊断量：即使控制尚处于 ACQUIRE，也展示算法本帧算出的中心；
    // 是否允许下发仍严格由 valid/deviation 控制。
    boundary.road_center = result.center_x >= 0
      ? static_cast<uint16_t>(result.center_x) : 0;
    boundary.scan_y = static_cast<uint16_t>(result.y);
    boundary.left = static_cast<int16_t>(result.left);
    boundary.right = static_cast<int16_t>(result.right);
    boundary.preview_mode = road_tracking::preview_name(result.preview);
    boundary.selected_pt = tracked.selected_pt;
    boundary.width = static_cast<uint16_t>(std::max(0, result.width));
    boundary.min_width = static_cast<uint16_t>(std::max(0, tracked.min_width));
    boundary.max_width = static_cast<uint16_t>(std::max(0, tracked.max_width));
    if (result.width <= 0) boundary.width_status = "NO_WIDTH";
    else if (result.width < tracked.min_width) boundary.width_status = "TOO_NARROW";
    else if (result.width > tracked.max_width) boundary.width_status = "TOO_WIDE";
    else boundary.width_status = "NORMAL";
    boundary.boundary_source = result.used_barrier_gap ? "BARRIER_GAP" : "ROAD_MASK";
    boundary.algorithm_valid = result.valid;
    boundary.algorithm_deviation = result.valid
      ? static_cast<int16_t>(result.deviation) : -999;
    boundary.control_ready = tracking_enabled_;
    pub_boundary_->publish(boundary);

    // 感知端直接合成调试画面：原图、Mask、边界和控制线天然同帧，
    // Web 不再尝试从三条异步 DDS 流中猜测配对。
    publish_debug_images(*msg, seg_mask, processed_mask.data(), result);

    std::free(seg_mask);                     // inference 内部分配，用完释放

    // 5) 发布掩膜（移动，避免整帧拷贝）
    pub_seg_->publish(std::move(*out));
  }

  void publish_invalid_boundary()
  {
    scoutcar_msgs::msg::RoadBoundary boundary;
    boundary.valid = false;
    pub_boundary_->publish(boundary);
  }

  void publish_debug_images(const sensor_msgs::msg::Image & source,
                            const uint8_t * mask,
                            const uint8_t * processed_mask,
                            const road_tracking::Result & result)
  {
    const int width=static_cast<int>(source.width),height=static_cast<int>(source.height);
    cv::Mat rgb(height,width,CV_8UC3,const_cast<uint8_t *>(source.data.data()));
    cv::Mat bgr;
    cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR);
    pub_source_->publish(source);
    // mask==nullptr 表示本帧无检测：发布干净画面（不画叠加），保证 web 实时流不冻结
    if (mask == nullptr) {
      pub_debug_->publish(make_rgb8_debug(source, bgr));
      return;
    }
    for(int y=0;y<height;++y) for(int x=0;x<width;++x) {
      const uint8_t value=mask[static_cast<size_t>(y)*width+x];
      cv::Vec3b & pixel=bgr.at<cv::Vec3b>(y,x);
      if(value==1) pixel=cv::Vec3b(0,255,0);       // road：常规纯绿色
      else if(value==2) pixel=cv::Vec3b(0,0,255); // barrier：常规纯红色
    }
    {
      for(int y=0;y<height;++y) for(int x=0;x<width;++x) {
        if(!processed_mask||!processed_mask[static_cast<size_t>(y)*width+x]) continue;
        bgr.at<cv::Vec3b>(y,x)=cv::Vec3b(255,0,0); // 实际参与跟踪：纯蓝色
      }
      const int y=std::clamp(result.y,0,height-1),cx=width/2;
      cv::line(bgr,{0,y},{width-1,y},{0,255,255},1);
      cv::line(bgr,{cx,0},{cx,height-1},{255,255,255},1);
      if(result.left>=0&&result.right>=0) {
        cv::line(bgr,{result.left,y-5},{result.left,y+5},{0,255,255},2);
        cv::line(bgr,{result.right,y-5},{result.right,y+5},{0,255,255},2);
      }
      if(result.valid) {
        const int road_center=(result.left+result.right)/2;
        cv::line(bgr,{result.left,y-10},{result.left,y+10},{255,0,0},3);
        cv::line(bgr,{result.right,y-10},{result.right,y+10},{0,0,255},3);
        cv::line(bgr,{cx,y},{road_center,y},{0,255,255},2);
        cv::drawMarker(bgr,{cx,y},{255,255,255},cv::MARKER_CROSS,11,2);
        cv::drawMarker(bgr,{road_center,y},{0,255,0},cv::MARKER_CROSS,11,2);
      }
      // 调试文字（DEV/MODE 等）不画在图上，由 /perception/road_boundary
      // 与 /mission/status 提供给网页状态面板展示。
      pub_debug_->publish(make_rgb8_debug(source, bgr));
    }
  }

  // bgr 绘制结果 → rgb8 Image 消息（叠加录像与 web 推流共用）
  static sensor_msgs::msg::Image make_rgb8_debug(const sensor_msgs::msg::Image & source,
                                                 const cv::Mat & bgr)
  {
    cv::Mat debug_rgb;
    cv::cvtColor(bgr, debug_rgb, cv::COLOR_BGR2RGB);
    sensor_msgs::msg::Image debug;
    debug.header = source.header;
    debug.width = source.width;
    debug.height = source.height;
    debug.encoding = "rgb8";
    debug.step = source.width * 3;
    debug.data.assign(debug_rgb.data, debug_rgb.data + debug_rgb.total() * debug_rgb.elemSize());
    return debug;
  }

  std::string model_path_;
  std::string label_path_;
  std::string image_topic_;
  bool model_ok_ = false;
  bool post_process_ok_ = false;
  rknn_app_context_t rknn_ctx_{};   // 模型上下文（yolov5_seg.h）
  std::unique_ptr<road_tracking::RoadTracker> road_tracker_;
  bool tracking_enabled_ = false;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_seg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_source_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;
  rclcpp::Publisher<scoutcar_msgs::msg::RoadBoundary>::SharedPtr pub_boundary_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_deviation_enable_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
