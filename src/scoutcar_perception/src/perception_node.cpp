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
#include "road_tracker.h"
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
    tracking_config.tracking_mode = declare_parameter<std::string>(
      "road_tracking.mode", "robust_center");
    tracking_config.near.row_ratio = declare_parameter<double>("preview.near.row_ratio", 0.8);
    tracking_config.near.edge_margin_px =
      declare_parameter<int>("preview.near.edge_margin_px", 10);
    tracking_config.near.expected_width =
      declare_parameter<int>("preview.near.expected_width", 290);
    tracking_config.near.min_width =
      declare_parameter<int>("preview.near.min_width", 250);
    tracking_config.near.max_width =
      declare_parameter<int>("preview.near.max_width", 330);
    tracking_config.far.row_ratio = declare_parameter<double>("preview.far.row_ratio", 0.4);
    tracking_config.far.edge_margin_px =
      declare_parameter<int>("preview.far.edge_margin_px", 10);
    tracking_config.far.expected_width =
      declare_parameter<int>("preview.far.expected_width", 86);
    tracking_config.far.min_width =
      declare_parameter<int>("preview.far.min_width", 70);
    tracking_config.far.max_width =
      declare_parameter<int>("preview.far.max_width", 110);
    tracking_config.max_inline_gap_px =
      declare_parameter<int>("road_tracking.max_inline_gap_px", 8);
    tracking_config.min_run_width_px =
      declare_parameter<int>("road_tracking.min_run_width_px", 12);
    tracking_config.boundary_jump_px =
      declare_parameter<int>("road_tracking.boundary_jump_px", 36);
    tracking_config.reconnect_tolerance_px =
      declare_parameter<int>("road_tracking.reconnect_tolerance_px", 20);
    tracking_config.reconnect_rows =
      declare_parameter<int>("road_tracking.reconnect_rows", 2);
    tracking_config.local_fit_points =
      declare_parameter<int>("road_tracking.local_fit_points", 10);
    tracking_config.center_fit_top_ratio =
      declare_parameter<double>("road_tracking.center_fit_top_ratio", 0.72);
    tracking_config.center_fit_bottom_ratio =
      declare_parameter<double>("road_tracking.center_fit_bottom_ratio", 0.90);
    tracking_config.center_edge_margin_px =
      declare_parameter<int>("road_tracking.center_edge_margin_px", 10);
    tracking_config.center_min_width_px =
      declare_parameter<int>("road_tracking.center_min_width_px", 40);
    tracking_config.center_min_points =
      declare_parameter<int>("road_tracking.center_min_points", 30);
    tracking_config.center_inlier_px =
      declare_parameter<int>("road_tracking.center_inlier_px", 8);
    tracking_config.center_min_inlier_ratio =
      declare_parameter<double>("road_tracking.center_min_inlier_ratio", 0.60);
    tracking_config.center_hold_frames =
      declare_parameter<int>("road_tracking.center_hold_frames", 4);
    tracking_config.center_smoothing =
      declare_parameter<double>("road_tracking.center_smoothing", 0.35);
    tracking_config.center_max_jump_px =
      declare_parameter<int>("road_tracking.center_max_jump_px", 50);
    tracking_config.center_max_slope_jump =
      declare_parameter<double>("road_tracking.center_max_slope_jump", 0.35);
    tracking_config.center_max_abs_slope =
      declare_parameter<double>("road_tracking.center_max_abs_slope", 0.35);
    tracking_config.legacy_overlap_margin_px =
      declare_parameter<int>("road_tracking.legacy_overlap_margin_px", 14);
    tracking_config.legacy_max_missing_rows =
      declare_parameter<int>("road_tracking.legacy_max_missing_rows", 6);
    tracking_config.legacy_boundary_stable_px =
      declare_parameter<int>("road_tracking.legacy_boundary_stable_px", 45);
    tracking_config.legacy_bottom_seed_ratio =
      declare_parameter<double>("road_tracking.legacy_bottom_seed_ratio", 0.85);
    tracking_config.legacy_use_barrier_gap =
      declare_parameter<bool>("road_tracking.legacy_use_barrier_gap", true);
    road_tracker_ = std::make_unique<road_tracking::RoadTracker>(tracking_config);
    tracking_mode_ = tracking_config.tracking_mode;
    RCLCPP_INFO(get_logger(), "道路跟踪模式: %s", tracking_mode_.c_str());

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
        acquisition_count_ = 0;
        control_ready_ = false;
        if (road_tracker_ != nullptr) road_tracker_->reset();
        RCLCPP_INFO(get_logger(), "循迹%s，重置中心线并重新捕获道路",
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
      return;   // 本帧无掩膜，不发布
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

    // 4) 使用 RoadTracker 的近/远预瞄、车道锚定与无效值判定。
    // 转弯期间不允许任意轮廓成为下一段道路的历史线。
    if (!tracking_enabled_) road_tracker_->reset();
    std::vector<uint8_t> processed_mask(static_cast<size_t>(img.width)*img.height,0);
    const auto tracked = road_tracker_->process(
      seg_mask, img.width, img.height, processed_mask.data());
    const auto & result = tracked.boundary;
    const bool legacy_mode = tracking_mode_ == "legacy_width_barrier";
    const bool stable_observation = result.valid && !result.center_predicted &&
      (legacy_mode || (result.center_confidence >= 0.80f &&
                       result.center_candidate_count >= 60));
    if (tracking_enabled_) {
      acquisition_count_ = stable_observation ? acquisition_count_ + 1 : 0;
      control_ready_ = acquisition_count_ >= kAcquisitionFrames;
    } else {
      acquisition_count_ = 0;
      control_ready_ = false;
    }
    scoutcar_msgs::msg::RoadBoundary boundary;
    boundary.header = msg->header;
    boundary.valid = result.valid && control_ready_;
    boundary.deviation = boundary.valid ? static_cast<int16_t>(result.deviation) : -999;
    boundary.road_center = boundary.valid
      ? static_cast<uint16_t>(result.center_x) : 0;
    boundary.scan_y = static_cast<uint16_t>(result.y);
    boundary.raw_left = static_cast<int16_t>(result.raw_left);
    boundary.raw_right = static_cast<int16_t>(result.raw_right);
    boundary.fit_top_y = static_cast<uint16_t>(result.fit_top_y);
    boundary.fit_bottom_y = static_cast<uint16_t>(result.fit_bottom_y);
    boundary.left_reconstructed = result.left_reconstructed;
    boundary.right_reconstructed = result.right_reconstructed;
    boundary.raw_left_boundary = result.raw_left_boundary;
    boundary.raw_right_boundary = result.raw_right_boundary;
    boundary.fitted_left_boundary = result.fitted_left_boundary;
    boundary.fitted_right_boundary = result.fitted_right_boundary;
    boundary.left_repaired = result.left_repaired;
    boundary.right_repaired = result.right_repaired;
    boundary.center_boundary = result.center_boundary;
    boundary.center_slope = result.center_slope;
    boundary.center_intercept = result.center_intercept;
    boundary.center_confidence = result.center_confidence;
    boundary.center_candidate_count = static_cast<uint16_t>(result.center_candidate_count);
    boundary.center_inlier_count = static_cast<uint16_t>(result.center_inlier_count);
    boundary.center_predicted = result.center_predicted;
    pub_boundary_->publish(boundary);

    // 感知端直接合成调试画面：原图、Mask、边界和控制线天然同帧，
    // Web 不再尝试从三条异步 DDS 流中猜测配对。
    publish_debug_images(*msg, seg_mask, processed_mask.data(), result, tracked,
                         control_ready_);

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
                            const road_boundary::Result & result,
                            const road_tracking::Result & tracked,
                            bool control_ready)
  {
    pub_source_->publish(source);
    const int width=static_cast<int>(source.width),height=static_cast<int>(source.height);
    cv::Mat rgb(height,width,CV_8UC3,const_cast<uint8_t *>(source.data.data()));
    cv::Mat bgr;
    cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR);
    for(int y=0;y<height;++y) for(int x=0;x<width;++x) {
      const uint8_t value=mask[static_cast<size_t>(y)*width+x];
      cv::Vec3b & pixel=bgr.at<cv::Vec3b>(y,x);
      if(value==1) pixel[1]=static_cast<uint8_t>((255+pixel[1])/2);
      else if(value==2) pixel[2]=static_cast<uint8_t>((255+pixel[2])/2);
    }
    if(tracking_mode_=="legacy_width_barrier") {
      // f977e72 原工程诊断图：红色表示从底部连通、实际参与宽度判断的道路主体。
      for(int y=0;y<height;++y) for(int x=0;x<width;++x) {
        if(!processed_mask[static_cast<size_t>(y)*width+x]) continue;
        cv::Vec3b & pixel=bgr.at<cv::Vec3b>(y,x);
        pixel=cv::Vec3b(static_cast<uint8_t>(pixel[0]*0.45),
                        static_cast<uint8_t>(pixel[1]*0.45),
                        static_cast<uint8_t>(255*0.55+pixel[2]*0.45));
      }
      const int y=std::clamp(result.y,0,height-1),cx=width/2;
      cv::line(bgr,{0,y},{width-1,y},{0,255,255},1);
      cv::line(bgr,{cx,0},{cx,height-1},{255,255,255},1);
      if(result.raw_left>=0&&result.raw_right>=0) {
        cv::line(bgr,{result.raw_left,y-5},{result.raw_left,y+5},{0,255,255},2);
        cv::line(bgr,{result.raw_right,y-5},{result.raw_right,y+5},{0,255,255},2);
      }
      if(result.valid) {
        const int road_center=(result.left+result.right)/2;
        cv::line(bgr,{result.left,y-10},{result.left,y+10},{255,0,0},3);
        cv::line(bgr,{result.right,y-10},{result.right,y+10},{0,0,255},3);
        cv::line(bgr,{cx,y},{road_center,y},{0,255,255},2);
        cv::drawMarker(bgr,{cx,y},{255,255,255},cv::MARKER_CROSS,11,2);
        cv::drawMarker(bgr,{road_center,y},{0,255,0},cv::MARKER_CROSS,11,2);
      }
      char diagnostic[220];
      if(result.valid) {
        std::snprintf(diagnostic,sizeof(diagnostic),
          "LEGACY %s pt=%.2f RAW=%d,%d,%d FIX=%d,%d,%d DEV=%d%s",
          road_boundary::status_name(result.status),tracked.selected_pt,
          result.raw_left,result.raw_right,result.raw_width,
          result.left,result.right,result.width,result.deviation,
          control_ready?"":" ACQUIRE");
      } else {
        std::snprintf(diagnostic,sizeof(diagnostic),
          "LEGACY INVALID pt=%.2f RAW=%d,%d,%d DEV=-999",
          tracked.selected_pt,result.raw_left,result.raw_right,result.raw_width);
      }
      cv::putText(bgr,diagnostic,{12,height-18},cv::FONT_HERSHEY_SIMPLEX,0.45,
                  result.valid?cv::Scalar(0,255,0):cv::Scalar(0,0,255),1,cv::LINE_AA);
      cv::putText(bgr,"MODE: legacy_width_barrier",{12,28},
                  cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,255,255),2,cv::LINE_AA);
      cv::Mat debug_rgb;
      cv::cvtColor(bgr,debug_rgb,cv::COLOR_BGR2RGB);
      sensor_msgs::msg::Image debug;
      debug.header=source.header;debug.width=source.width;debug.height=source.height;
      debug.encoding="rgb8";debug.step=source.width*3;
      debug.data.assign(debug_rgb.data,debug_rgb.data+debug_rgb.total()*debug_rgb.elemSize());
      pub_debug_->publish(std::move(debug));
      return;
    }
    const int top=result.fit_top_y;
    auto draw=[&](const std::vector<int16_t> & xs,const cv::Scalar & color,int thickness) {
      for(size_t i=1;i<xs.size();++i) {
        if(xs[i-1]<0||xs[i]<0) continue;
        const int y0=top+static_cast<int>(i)-1,y1=y0+1;
        if(y1>=height) break;
        cv::line(bgr,{xs[i-1],y0},{xs[i],y1},color,thickness,cv::LINE_AA);
      }
    };
    draw(result.raw_left_boundary,{0,165,255},1);
    draw(result.raw_right_boundary,{0,165,255},1);
    draw(result.fitted_left_boundary,{255,255,0},1);
    draw(result.fitted_right_boundary,{255,0,255},1);
    draw(result.center_boundary,result.center_predicted?cv::Scalar(0,0,255)
                                                     :cv::Scalar(0,255,0),3);
    char text[96];
    std::snprintf(text,sizeof(text),"Dev:%d C:%d/%d%s%s",result.deviation,
                  result.center_inlier_count,result.center_candidate_count,
                  result.center_predicted?" HOLD":"",
                  control_ready?"":" ACQUIRE");
    cv::putText(bgr,text,{12,28},cv::FONT_HERSHEY_SIMPLEX,0.6,
                result.center_predicted?cv::Scalar(0,0,255):cv::Scalar(0,255,0),2,
                cv::LINE_AA);
    cv::Mat debug_rgb;
    cv::cvtColor(bgr,debug_rgb,cv::COLOR_BGR2RGB);
    sensor_msgs::msg::Image debug;
    debug.header=source.header;
    debug.width=source.width;debug.height=source.height;
    debug.encoding="rgb8";debug.step=source.width*3;
    debug.data.assign(debug_rgb.data,debug_rgb.data+debug_rgb.total()*debug_rgb.elemSize());
    pub_debug_->publish(std::move(debug));
  }

  std::string model_path_;
  std::string label_path_;
  std::string image_topic_;
  std::string tracking_mode_;
  bool model_ok_ = false;
  bool post_process_ok_ = false;
  rknn_app_context_t rknn_ctx_{};   // 模型上下文（yolov5_seg.h）
  std::unique_ptr<road_tracking::RoadTracker> road_tracker_;
  static constexpr int kAcquisitionFrames = 5;
  bool tracking_enabled_ = false;
  bool control_ready_ = false;
  int acquisition_count_ = 0;

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
