// serial_node —— 串口驱动节点（话题 ⇄ 串口字节的翻译官）
//
// 对应原工程：
//   - control/serial.cc + control/protocol.h：收发与协议（已原样搬入本包）
//   - main.cc 的 on_rx_flag 回调：此处变成"发布 RxEvent"，决策移去 mission_node
//
// 职责（只翻译，不做决策）：
//   收 /mission/path_cmd      → 打包 FF 02 写串口
//   收 /mission/base_cmd      → 打包 FF 05 写串口
//   串口接收线程解出事件       → 发布 /mcu/rx_event
//   收 /perception/road_boundary → 按任务门控打包 FF 03 偏差帧

#include <chrono>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <scoutcar_msgs/msg/car_state.hpp>
#include <scoutcar_msgs/msg/road_deviation.hpp>
#include <scoutcar_msgs/msg/rx_event.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "scoutcar_control/serial.h"

class SerialNode : public rclcpp::Node {
public:
  SerialNode() : Node("serial_node") {
    serial_device_ = declare_parameter<std::string>("serial_device", "/dev/ttyS6");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    deviation_rate_hz_ = declare_parameter<double>("deviation_rate_hz", 30.0);
    deviation_timeout_ms_ = declare_parameter<int>("deviation_timeout_ms", 150);
    if (deviation_rate_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "deviation_rate_hz 必须大于 0，回退到 30 Hz");
      deviation_rate_hz_ = 30.0;
    }

    // ── 接口 ──
    pub_event_ = create_publisher<scoutcar_msgs::msg::RxEvent>("mcu/rx_event", 10);

    sub_base_cmd_ = create_subscription<scoutcar_msgs::msg::CarState>(
      "mission/base_cmd", rclcpp::QoS(10).reliable(),
      [this](const scoutcar_msgs::msg::CarState::SharedPtr msg) {
          const bool need_turn =
            msg->mission_state == scoutcar_msgs::msg::CarState::TURNING;

          pathplan::BaseCmdFrame frame{
            need_turn,
            static_cast<pathplan::Pose>(msg->front_camera_pose),
            static_cast<pathplan::Pose>(msg->turn_camera_pose),
          };

          uart_send_base_control(fd_, frame);
          if (msg->mission_state != scoutcar_msgs::msg::CarState::DRIVING) {
            deviation_enabled_ = false;
            has_fresh_deviation_ = false;
          }

          RCLCPP_INFO(
            get_logger(), "[发送] 0x05 execute=%u front=%u turn=%u",
            static_cast<unsigned>(frame.need_turn),
            static_cast<unsigned>(frame.front),
            static_cast<unsigned>(frame.turn));
      });

    sub_mission_state_ = create_subscription<scoutcar_msgs::msg::CarState>(
      "mission/mission_state",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const scoutcar_msgs::msg::CarState::SharedPtr msg) {
        const bool should_enable =
          msg->mission_state == scoutcar_msgs::msg::CarState::DRIVING;
        if (should_enable != deviation_enabled_) {
          has_fresh_deviation_ = false;
        }
        deviation_enabled_ = should_enable;
      });

    sub_btp_debug_cmd_ = create_subscription<std_msgs::msg::UInt8>(
      "debug/btp_camera_pose",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        pathplan::Pose turn_pose = pathplan::Pose::AHEAD;
        if (msg->data == static_cast<uint8_t>(pathplan::Pose::LEFT)) {
          turn_pose = pathplan::Pose::LEFT;
        } else if (msg->data == static_cast<uint8_t>(pathplan::Pose::RIGHT)) {
          turn_pose = pathplan::Pose::RIGHT;
        }

        const pathplan::BaseCmdFrame frame{
          false, pathplan::Pose::AHEAD, turn_pose};
        uart_send_base_control(fd_, frame);
        RCLCPP_INFO(
          get_logger(), "[BTP调试] 0x05 execute=0 front=1 turn=%u",
          static_cast<unsigned>(turn_pose));
      });

    sub_path_cmd_ = create_subscription<scoutcar_msgs::msg::CarState>(
        "mission/path_cmd", rclcpp::QoS(10).reliable(),
        [this](const scoutcar_msgs::msg::CarState::SharedPtr msg) {
          pathplan::PathPlanFrame frame{
            static_cast<uint8_t>(msg->segment_start),
            static_cast<uint8_t>(msg->segment_goal),
            static_cast<pathplan::TurnAction>(msg->arrival_action),
          };

          uart_send_path_segment(fd_, frame);
          RCLCPP_INFO(
            get_logger(), "[发送] 路径段 %u→%u，到达后%s",
            frame.start, frame.goal, pathplan::actionName(frame.finalAction));
        });
    
    sub_debug_cmd_ =create_subscription<scoutcar_msgs::msg::CarState>(
      "mission/debug_cmd",
      rclcpp::QoS(rclcpp::KeepLast(1))
          .reliable()
          .transient_local(),
      [this](const scoutcar_msgs::msg::CarState::SharedPtr msg) {
        const auto status =
          msg->system_status ==
            scoutcar_msgs::msg::CarState::SYSTEM_READY
          ? pathplan::Status::READY
          : pathplan::Status::ERROR;

        uart_send_debug_ready(
          fd_,
          status,
          msg->fixed_remaining,
          msg->random_remaining);
      });

    sub_deviation_ = create_subscription<scoutcar_msgs::msg::RoadDeviation>(
        "perception/road_deviation", 10,
        [this](const scoutcar_msgs::msg::RoadDeviation::SharedPtr msg) {
          if (!deviation_enabled_) {
            return;
          }
          latest_deviation_ =
              msg->status == scoutcar_msgs::msg::RoadDeviation::STATUS_OK
                  ? msg->deviation
                  : kInvalidDeviation;
          last_boundary_time_ = std::chrono::steady_clock::now();
          has_fresh_deviation_ = true;
        });


    fd_ = uart_init(serial_device_.c_str(), baud_rate_);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "串口打开失败，接收线程未启动");
      return;
    }
    uart_start_rx_thread(fd_, on_rx_flag_static, this);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / deviation_rate_hz_));
    deviation_timer_ =
        create_wall_timer(period, [this]() { send_cached_deviation(); });
  }

  ~SerialNode() override {
    uart_stop_rx_thread();
    uart_close(fd_);
  }

private:
  void send_cached_deviation() {
    if (fd_ < 0) {
      return;
    }
    int16_t deviation_to_send = kInvalidDeviation;
    if (deviation_enabled_ && has_fresh_deviation_) {
      deviation_to_send = latest_deviation_;
      //超时检查
      const auto now = std::chrono::steady_clock::now();
      const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                           now - last_boundary_time_)
                           .count();
      if (age > deviation_timeout_ms_) {
        deviation_to_send = kInvalidDeviation;
      }
    }
    uart_send_deviation_frame(fd_, deviation_to_send);
  }

  static void on_rx_flag_static(pathplan::RxFlag flag, void *user) {
    static_cast<SerialNode *>(user)->on_rx_flag(flag);
  }

  void on_rx_flag(pathplan::RxFlag flag) {
    switch (flag) {
    case pathplan::RxFlag::START:
      RCLCPP_INFO(get_logger(), "[串口接收] 收到启动指令，开始执行任务");
      publish_event(scoutcar_msgs::msg::RxEvent::START); // 0xAA
      break;
    case pathplan::RxFlag::STOP:
      RCLCPP_INFO(get_logger(), "[串口接收] 收到停止指令，重置状态");
      publish_event(scoutcar_msgs::msg::RxEvent::STOP); // 0xBB
      break;
    case pathplan::RxFlag::ARRIVED:
      RCLCPP_INFO(get_logger(), "[串口接收] 小车接近目标点");
      publish_event(scoutcar_msgs::msg::RxEvent::ARRIVED); // 0xDD
      break;
    case pathplan::RxFlag::TURN_FINISHED:
      RCLCPP_INFO(get_logger(), "[串口接收] 转向完成");
      publish_event(scoutcar_msgs::msg::RxEvent::TURN_FINISHED); // 0xEE
      break;
    case pathplan::RxFlag::HEARTED:

    
      break;
    default:
      RCLCPP_WARN(get_logger(), "[串口接收] 未知 flag 0x%02X",
                  static_cast<uint8_t>(flag));
      break;
      //后续增加下位机掉线检查
      // case pathplan::RxFlag::HEARTED:
      //   // 心跳流的开始，透传给话题（mission_node 忽略），不打日志
      //   publish_event(scoutcar_msgs::msg::RxEvent::HEARTED);   // 0xFF
      //   break;
    }
  }

  void publish_event(uint8_t event) {
    scoutcar_msgs::msg::RxEvent ev;
    ev.event = event;
    pub_event_->publish(ev);
  }

  std::string serial_device_;
  int baud_rate_;
  double deviation_rate_hz_;
  int deviation_timeout_ms_;
  int fd_ = -1;
  bool deviation_enabled_ = false;
  bool has_fresh_deviation_ = false;
  static constexpr int16_t kInvalidDeviation = -999;
  int16_t latest_deviation_ = 0;
  std::chrono::steady_clock::time_point last_boundary_time_{};

  rclcpp::Publisher<scoutcar_msgs::msg::RxEvent>::SharedPtr pub_event_;
  rclcpp::TimerBase::SharedPtr deviation_timer_;
  rclcpp::Subscription<scoutcar_msgs::msg::CarState>::SharedPtr sub_base_cmd_;
  rclcpp::Subscription<scoutcar_msgs::msg::CarState>::SharedPtr sub_mission_state_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_btp_debug_cmd_;
  rclcpp::Subscription<scoutcar_msgs::msg::CarState>::SharedPtr sub_path_cmd_;
  rclcpp::Subscription<scoutcar_msgs::msg::RoadDeviation>::SharedPtr sub_deviation_;
  rclcpp::Subscription<scoutcar_msgs::msg::CarState>::SharedPtr sub_debug_cmd_;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}
