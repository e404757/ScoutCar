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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <scoutcar_msgs/msg/car_state.hpp>
#include <scoutcar_msgs/msg/detect_task.hpp>
#include <scoutcar_msgs/msg/recon_result.hpp>
#include <scoutcar_msgs/msg/road_deviation.hpp>
#include <scoutcar_msgs/msg/rx_event.hpp>
#include <std_msgs/msg/bool.hpp>
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
          const auto control =
            msg->mission_state == scoutcar_msgs::msg::CarState::ERROR
              ? pathplan::BaseCmdFrame::Control::STOP
              : pathplan::BaseCmdFrame::Control::NORMAL;

          pathplan::BaseCmdFrame frame{
            control,
            static_cast<pathplan::Pose>(msg->front_camera_pose),
            static_cast<pathplan::Pose>(msg->turn_camera_pose),
          };

          uart_send_base_control(fd_, frame);
          if (msg->mission_state ==
              scoutcar_msgs::msg::CarState::FINDING_BTP) {
            btp_deviation_ready_time_ = std::chrono::steady_clock::now() +
                                        std::chrono::milliseconds(500);
            has_fresh_deviation_ = false;
          }
          if (msg->mission_state != scoutcar_msgs::msg::CarState::DRIVING &&
              msg->mission_state != scoutcar_msgs::msg::CarState::FINDING_BTP) {
            deviation_enabled_ = false;
            has_fresh_deviation_ = false;
          }

          RCLCPP_INFO(
            get_logger(), "[发送] 0x05 control=0x%02X front=%u turn=%u",
            static_cast<unsigned>(frame.control),
            static_cast<unsigned>(frame.front),
            static_cast<unsigned>(frame.turn));
      });

    sub_mission_state_ = create_subscription<scoutcar_msgs::msg::CarState>(
      "mission/mission_state",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const scoutcar_msgs::msg::CarState::SharedPtr msg) {
        const bool entering_btp =
          msg->mission_state == scoutcar_msgs::msg::CarState::FINDING_BTP &&
          mission_state_ != scoutcar_msgs::msg::CarState::FINDING_BTP;
        mission_state_ = msg->mission_state;
        if (entering_btp) {
          btp_deviation_ready_time_ = std::chrono::steady_clock::now() +
                                      std::chrono::milliseconds(500);
          has_fresh_deviation_ = false;
        }
        const bool should_enable =
          msg->mission_state == scoutcar_msgs::msg::CarState::DRIVING ||
          msg->mission_state == scoutcar_msgs::msg::CarState::FINDING_BTP;
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
          pathplan::BaseCmdFrame::Control::NORMAL,
          pathplan::Pose::AHEAD, turn_pose};
        uart_send_base_control(fd_, frame);
        RCLCPP_INFO(
          get_logger(), "[BTP调试] 0x05 control=0x00 front=1 turn=%u",
          static_cast<unsigned>(turn_pose));
      });

    sub_path_cmd_ = create_subscription<scoutcar_msgs::msg::CarState>(
        "mission/path_cmd", rclcpp::QoS(10).reliable(),
        [this](const scoutcar_msgs::msg::CarState::SharedPtr msg) {
          debug_resend_enabled_ = false;
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

    sub_detect_task_ = create_subscription<scoutcar_msgs::msg::DetectTask>(
      "mission/detect_task", 10,
      [this](const scoutcar_msgs::msg::DetectTask::SharedPtr msg) {
        if (msg->status == scoutcar_msgs::msg::DetectTask::END) {
          manual_detect_end_pending_ = true;
          return;
        }
        if (msg->status != scoutcar_msgs::msg::DetectTask::START) {
          return;
        }
        manual_detect_end_pending_ = false;
        uart_send_detect_task(fd_, pathplan::DetectStatus::START, 0, 0);
        RCLCPP_INFO(get_logger(), "[发送] 0x04 START left=0 right=0");
      });

    sub_recon_result_ = create_subscription<scoutcar_msgs::msg::ReconResult>(
      "perception/recon_result", 10,
      [this](const scoutcar_msgs::msg::ReconResult::SharedPtr msg) {
        pending_recon_left_ = msg->left_valid ? msg->left_result : 0;
        pending_recon_right_ = msg->right_valid ? msg->right_result : 0;
        if (!manual_detect_end_pending_) {
          uart_send_detect_task(
            fd_, pathplan::DetectStatus::END,
            pending_recon_left_, pending_recon_right_);
          RCLCPP_INFO(
            get_logger(), "[发送] 0x04 END left=%u right=%u",
            static_cast<unsigned>(pending_recon_left_),
            static_cast<unsigned>(pending_recon_right_));
          return;
        }
        recon_result_send_time_ =
          std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        recon_result_pending_ = true;
        RCLCPP_INFO(
          get_logger(), "侦察结果已缓存，0.5 秒后发送 left=%u right=%u",
          static_cast<unsigned>(pending_recon_left_),
          static_cast<unsigned>(pending_recon_right_));
      });
    
    sub_debug_cmd_ =create_subscription<scoutcar_msgs::msg::CarState>(
      "mission/debug_cmd",
      rclcpp::QoS(rclcpp::KeepLast(1))
          .reliable()
          .transient_local(),
      [this](const scoutcar_msgs::msg::CarState::SharedPtr msg) {
        const auto status =
          msg->mission_state == scoutcar_msgs::msg::CarState::WAIT_START
            ? pathplan::Status::READY
            : pathplan::Status::ERROR;

        cached_debug_status_ = status;
        cached_fixed_remaining_ = msg->fixed_remaining;
        cached_random_remaining_ = msg->random_remaining;
        has_cached_debug_ = true;
        debug_resend_enabled_ = true;
        uart_send_debug_ready(
          fd_,
          cached_debug_status_,
          cached_fixed_remaining_,
          cached_random_remaining_);
        if (debug_timer_) {
          debug_timer_->reset();
        }
      });

    sub_deviation_ = create_subscription<scoutcar_msgs::msg::RoadDeviation>(
        "perception/road_deviation", 10,
        [this](const scoutcar_msgs::msg::RoadDeviation::SharedPtr msg) {
          if (!deviation_enabled_) {
            return;
          }
          if (mission_state_ == scoutcar_msgs::msg::CarState::FINDING_BTP &&
              std::chrono::steady_clock::now() < btp_deviation_ready_time_) {
            return;
          }
          latest_deviation_ =
              msg->status == scoutcar_msgs::msg::RoadDeviation::STATUS_OK
                  ? msg->deviation
                  : kInvalidDeviation;
          last_boundary_time_ = std::chrono::steady_clock::now();
          has_fresh_deviation_ = true;
        });

    const auto health_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sub_front_camera_health_ = create_subscription<std_msgs::msg::Bool>(
        "/camera/front/healthy", health_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          front_camera_healthy_ = msg->data;
        });
    sub_turn_camera_health_ = create_subscription<std_msgs::msg::Bool>(
        "/camera/turn/healthy", health_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          turn_camera_healthy_ = msg->data;
        });
    health_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [this]() { update_health_indicator(); });


    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / deviation_rate_hz_));
    deviation_timer_ =
        create_wall_timer(period, [this]() { send_cached_deviation(); });
    debug_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]() { resend_cached_debug(); });
    reconnect_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]() { connect_serial(); });
    recon_result_timer_ = create_wall_timer(
        std::chrono::milliseconds(20), [this]() { send_pending_recon_result(); });
    connect_serial();
  }

  ~SerialNode() override {
    disconnect_serial();
    set_led_fault_mode("/sys/class/leds/blue_led", false);
    set_led_fault_mode("/sys/class/leds/green_led", false);
    uart_close(fd_);
  }

private:
  static int64_t steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  bool write_led(const char *path, const char *value) {
    std::ofstream output(path);
    if (!output.is_open()) {
      RCLCPP_ERROR(get_logger(), "无法打开板载 LED 接口: %s", path);
      return false;
    }
    output << value;
    return output.good();
  }

  bool set_led_fault_mode(const char *led_path, bool fault) {
    const std::string trigger = std::string(led_path) + "/trigger";
    const std::string brightness = std::string(led_path) + "/brightness";
    if (!fault) {
      return write_led(trigger.c_str(), "none") &&
             write_led(brightness.c_str(), "0");
    }
    const std::string delay_on = std::string(led_path) + "/delay_on";
    const std::string delay_off = std::string(led_path) + "/delay_off";
    return write_led(trigger.c_str(), "timer") &&
           write_led(delay_on.c_str(), "500") &&
           write_led(delay_off.c_str(), "500");
  }

  void update_health_indicator() {
    const int64_t now_ns = steady_now_ns();
    if (fd_ >= 0 &&
        now_ns - last_heartbeat_ns_.load() >=
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::seconds(3)).count()) {
      const bool was_healthy = serial_healthy_.exchange(false);
      RCLCPP_ERROR(
        get_logger(),
        was_healthy ? "连续 3 秒未收到下位机心跳，串口通信异常"
                    : "串口打开后连续 3 秒未收到下位机心跳，重新连接");
      disconnect_serial();
    }

    const bool fault = !front_camera_healthy_ || !turn_camera_healthy_ ||
                       !serial_healthy_.load();
    if (led_state_initialized_ && fault == led_fault_active_) {
      return;
    }
    led_state_initialized_ = true;
    led_fault_active_ = fault;
    const bool blue_ok = set_led_fault_mode("/sys/class/leds/blue_led", fault);
    const bool green_ok = set_led_fault_mode("/sys/class/leds/green_led", fault);
    if (!blue_ok || !green_ok) {
      RCLCPP_ERROR(get_logger(), "板载 LED 状态更新失败");
    } else if (fault) {
      RCLCPP_WARN(get_logger(), "设备健康异常，蓝绿灯开始闪烁");
    } else {
      RCLCPP_INFO(get_logger(), "摄像头与串口均正常，蓝绿灯已熄灭");
    }
  }

  void resend_cached_debug() {
    if (!debug_resend_enabled_ || !has_cached_debug_) {
      return;
    }
    uart_send_debug_ready(
        fd_, cached_debug_status_, cached_fixed_remaining_,
        cached_random_remaining_);
  }

  void send_pending_recon_result() {
    if (!recon_result_pending_ || fd_ < 0 ||
        std::chrono::steady_clock::now() < recon_result_send_time_) {
      return;
    }
    uart_send_detect_task(
      fd_, pathplan::DetectStatus::END,
      pending_recon_left_, pending_recon_right_);
    recon_result_pending_ = false;
    manual_detect_end_pending_ = false;
    RCLCPP_INFO(
      get_logger(), "[发送] 0x04 END left=%u right=%u",
      static_cast<unsigned>(pending_recon_left_),
      static_cast<unsigned>(pending_recon_right_));
  }

  void connect_serial() {
    if (fd_ >= 0) {
      return;
    }

    const int fd = uart_init(serial_device_.c_str(), baud_rate_);
    if (fd < 0) {
      return;
    }

    fd_ = fd;
    last_heartbeat_ns_.store(steady_now_ns());
    uart_start_rx_thread(fd_, on_rx_flag_static, this);
    RCLCPP_INFO(get_logger(), "串口已打开，等待下位机心跳");
    if (debug_resend_enabled_ && has_cached_debug_) {
      uart_send_debug_ready(
          fd_, cached_debug_status_, cached_fixed_remaining_,
          cached_random_remaining_);
    }
  }

  void disconnect_serial() {
    if (fd_ < 0) {
      return;
    }
    uart_stop_rx_thread();
    uart_close(fd_);
    fd_ = -1;
    serial_healthy_.store(false);
  }

  void send_cached_deviation() {
    if (fd_ < 0) {
      return;
    }
    int16_t deviation_to_send = kInvalidDeviation;
    const bool btp_settling =
        mission_state_ == scoutcar_msgs::msg::CarState::FINDING_BTP &&
        std::chrono::steady_clock::now() < btp_deviation_ready_time_;
    if (deviation_enabled_ && !btp_settling && has_fresh_deviation_) {
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
    case pathplan::RxFlag::BTP:
      RCLCPP_INFO(get_logger(), "[串口接收] 进入 BTP 阶段");
      publish_event(scoutcar_msgs::msg::RxEvent::BTP); // 0xCC
      break;
    case pathplan::RxFlag::ARRIVED:
      RCLCPP_INFO(get_logger(),
                  "[串口接收] 到达最佳转向点，下位机开始转向");
      publish_event(scoutcar_msgs::msg::RxEvent::ARRIVED); // 0xDD
      break;
    case pathplan::RxFlag::TURN_FINISHED:
      RCLCPP_INFO(get_logger(), "[串口接收] 转向完成");
      publish_event(scoutcar_msgs::msg::RxEvent::TURN_FINISHED); // 0xEE
      break;
    case pathplan::RxFlag::HEARTED:
      last_heartbeat_ns_.store(steady_now_ns());
      if (!serial_healthy_.exchange(true)) {
        RCLCPP_INFO(get_logger(), "收到下位机心跳，串口通信恢复");
      }
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
  uint8_t mission_state_ = scoutcar_msgs::msg::CarState::FINISHED;
  static constexpr int16_t kInvalidDeviation = -999;
  int16_t latest_deviation_ = 0;
  std::chrono::steady_clock::time_point last_boundary_time_{};
  std::chrono::steady_clock::time_point btp_deviation_ready_time_{};
  pathplan::Status cached_debug_status_ = pathplan::Status::ERROR;
  uint8_t cached_fixed_remaining_ = 0;
  uint8_t cached_random_remaining_ = 0;
  bool has_cached_debug_ = false;
  bool debug_resend_enabled_ = false;
  bool recon_result_pending_ = false;
  bool manual_detect_end_pending_ = false;
  uint8_t pending_recon_left_ = 0;
  uint8_t pending_recon_right_ = 0;
  std::chrono::steady_clock::time_point recon_result_send_time_{};
  bool front_camera_healthy_ = false;
  bool turn_camera_healthy_ = false;
  bool led_state_initialized_ = false;
  bool led_fault_active_ = true;
  std::atomic<bool> serial_healthy_{false};
  std::atomic<int64_t> last_heartbeat_ns_{0};

  rclcpp::Publisher<scoutcar_msgs::msg::RxEvent>::SharedPtr pub_event_;
  rclcpp::TimerBase::SharedPtr deviation_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::TimerBase::SharedPtr recon_result_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::Subscription<scoutcar_msgs::msg::CarState>::SharedPtr sub_base_cmd_;
  rclcpp::Subscription<scoutcar_msgs::msg::CarState>::SharedPtr sub_mission_state_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_btp_debug_cmd_;
  rclcpp::Subscription<scoutcar_msgs::msg::CarState>::SharedPtr sub_path_cmd_;
  rclcpp::Subscription<scoutcar_msgs::msg::DetectTask>::SharedPtr sub_detect_task_;
  rclcpp::Subscription<scoutcar_msgs::msg::ReconResult>::SharedPtr sub_recon_result_;
  rclcpp::Subscription<scoutcar_msgs::msg::RoadDeviation>::SharedPtr sub_deviation_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_front_camera_health_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_turn_camera_health_;
  rclcpp::Subscription<scoutcar_msgs::msg::CarState>::SharedPtr sub_debug_cmd_;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}
