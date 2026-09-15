// serial_node —— 串口驱动节点（话题 ⇄ 串口字节的翻译官）
//
// 对应原工程：
//   - control/serial.cc + control/protocol.h：收发与协议（已原样搬入本包）
//   - main.cc 的 on_rx_flag 回调：此处变成"发布 RxEvent"，决策移去 mission_node
//
// 职责（只翻译，不做决策）：
//   收 /mission/path_cmd      → 打包 FF 02 写串口
//   收 /mission/status        → 打包调试帧 FF 01 写串口
//   收 /mission/deviation_enable → 开关偏差帧发送
//   串口接收线程解出事件       → 发布 /mcu/rx_event
//   收 /perception/road_boundary → 按任务门控打包 FF 03 偏差帧

#include <chrono>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <scoutcar_msgs/msg/mission_status.hpp>
#include <scoutcar_msgs/msg/detect_task.hpp>
#include <scoutcar_msgs/msg/path_cmd.hpp>
#include <scoutcar_msgs/msg/road_deviation.hpp>
#include <scoutcar_msgs/msg/rx_event.hpp>
#include <std_msgs/msg/bool.hpp>

#include "scoutcar_control/serial.h"

class SerialNode : public rclcpp::Node {
public:
  SerialNode() : Node("serial_node") {
    serial_device_ =
        declare_parameter<std::string>("serial_device", "/dev/ttyS6");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    deviation_rate_hz_ = declare_parameter<double>("deviation_rate_hz", 30.0);
    deviation_timeout_ms_ = declare_parameter<int>("deviation_timeout_ms", 150);
    if (deviation_rate_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "deviation_rate_hz 必须大于 0，回退到 30 Hz");
      deviation_rate_hz_ = 30.0;
    }

    // ── 接口 ──
    pub_event_ =
        create_publisher<scoutcar_msgs::msg::RxEvent>("mcu/rx_event", 10);

    sub_path_cmd_ = create_subscription<scoutcar_msgs::msg::PathCmd>(
        "mission/path_cmd", 10,
        [this](const scoutcar_msgs::msg::PathCmd::SharedPtr msg) {
          pathplan::PathPlanFrame f{
              msg->start,
              msg->goal,
              static_cast<pathplan::TurnAction>(msg->action),
          };
          uart_send_path_segment(fd_, f);
          if (f.start == f.goal &&
              f.finalAction == pathplan::TurnAction::UTURN) {
            RCLCPP_INFO(get_logger(), "[发送] 立即掉头（参考节点 %u）", f.start);
          } else {
            RCLCPP_INFO(get_logger(), "[发送] 路径段 %u→%u，到达后%s",
                        f.start, f.goal,
                        pathplan::actionName(f.finalAction));
          }
        });
    const auto status_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sub_status_ = create_subscription<scoutcar_msgs::msg::MissionStatus>(
        "mission/car_status", status_qos,
        [this](const scoutcar_msgs::msg::MissionStatus::SharedPtr msg) {
          const pathplan::Status st =
              (msg->status == scoutcar_msgs::msg::MissionStatus::READY)
                  ? pathplan::Status::READY
                  : pathplan::Status::ERROR;
          uart_send_debug_ready(fd_, st, msg->fixed_remain, msg->random_remain);
        });

    sub_dev_ = create_subscription<std_msgs::msg::Bool>(
        "mission/deviation_enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          deviation_enabled_ = msg->data;
          RCLCPP_INFO(get_logger(), "偏差帧%s",
                      deviation_enabled_ ? "启用" : "停发");
        });

    sub_boundary_ = create_subscription<scoutcar_msgs::msg::RoadDeviation>(
        "perception/road_boundary", 10,
        [this](const scoutcar_msgs::msg::RoadDeviation::SharedPtr msg) {
          latest_deviation_ =
              msg->status == scoutcar_msgs::msg::RoadDeviation::STATUS_OK
                  ? msg->deviation
                  : kInvalidDeviation;
          last_boundary_time_ = std::chrono::steady_clock::now();
        });

    sub_detect_task_ = create_subscription<scoutcar_msgs::msg::DetectTask>(
        "mission/detect_task", status_qos,
        [this](const scoutcar_msgs::msg::DetectTask::SharedPtr msg) {
          pathplan::DetectStatus status;
          if (msg->status == scoutcar_msgs::msg::DetectTask::START) {
            status = pathplan::DetectStatus::START;
          } else if (msg->status == scoutcar_msgs::msg::DetectTask::END) {
            status = pathplan::DetectStatus::END;
          } else {
            RCLCPP_WARN(get_logger(), "忽略未知侦察状态: 0x%02X",
                        msg->status);
            return;
          }
          if (fd_ < 0) {
            RCLCPP_WARN(get_logger(), "串口未打开，侦察指令未发送");
            return;
          }
          uart_send_detect_task(fd_, status, 0, 0);
          RCLCPP_INFO(get_logger(), "[发送] 侦察%s，左右结果暂为00",
                      status == pathplan::DetectStatus::START ? "开始"
                                                              : "结束");
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
    //转向中偏差设置为0
    latest_deviation_ = deviation_enabled_ ? latest_deviation_ : 0;
    if (deviation_enabled_) {
      //超时检查
      const auto now = std::chrono::steady_clock::now();
      const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                           now - last_boundary_time_)
                           .count();
      if (age > deviation_timeout_ms_) {
        latest_deviation_ = kInvalidDeviation;
      }
    }
    uart_send_deviation_frame(fd_, latest_deviation_);
  }

  static void on_rx_flag_static(pathplan::RxFlag flag, void *user) {
    static_cast<SerialNode *>(user)->on_rx_flag(flag);
  }

  void on_rx_flag(pathplan::RxFlag flag) {
    if (flag != pathplan::RxFlag::START) {
      if (last_flag_.has_value() && *last_flag_ == flag) {
        return;
      }
    }
    last_flag_ = flag;

    switch (flag) {
    case pathplan::RxFlag::START:
      RCLCPP_INFO(get_logger(), "[接收] 收到启动指令，开始执行任务");
      publish_event(scoutcar_msgs::msg::RxEvent::START); // 0xAA
      break;
    case pathplan::RxFlag::CAM_AHEAD:
      RCLCPP_INFO(get_logger(),
                  "[接收] 摄像头转回正前方，使用front摄像头感知路面");
      publish_event(scoutcar_msgs::msg::RxEvent::CAM_AHEAD); // 0xBB
      break;
    case pathplan::RxFlag::CAM_TURNED:
      RCLCPP_INFO(get_logger(),
                  "[接收] 摄像头转向两侧，使用turn摄像头感知路面");
      publish_event(scoutcar_msgs::msg::RxEvent::CAM_TURNED); // 0xCC
      break;
    case pathplan::RxFlag::ARRIVED:
      RCLCPP_INFO(get_logger(), "[接收] 到达路口，开始执行转向动作");
      publish_event(scoutcar_msgs::msg::RxEvent::ARRIVED); // 0xDD
      break;
    case pathplan::RxFlag::TURN_FINISHED:
      RCLCPP_INFO(get_logger(), "[接收] 转向完成");
      publish_event(scoutcar_msgs::msg::RxEvent::TURN_FINISHED); // 0xEE
      break;
    default:
      RCLCPP_WARN(get_logger(), "[接收] 未知 flag 0x%02X",
                  static_cast<uint8_t>(flag));
      break;
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
  std::optional<pathplan::RxFlag>
      last_flag_; // 上一个收到的 flag（边沿触发去重）
  static constexpr int16_t kInvalidDeviation = -999;
  int16_t latest_deviation_ = kInvalidDeviation;
  std::chrono::steady_clock::time_point last_boundary_time_{};

  rclcpp::Publisher<scoutcar_msgs::msg::RxEvent>::SharedPtr pub_event_;
  rclcpp::TimerBase::SharedPtr deviation_timer_;
  rclcpp::Subscription<scoutcar_msgs::msg::PathCmd>::SharedPtr sub_path_cmd_;
  rclcpp::Subscription<scoutcar_msgs::msg::MissionStatus>::SharedPtr
      sub_status_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_dev_;
  rclcpp::Subscription<scoutcar_msgs::msg::RoadDeviation>::SharedPtr
      sub_boundary_;
  rclcpp::Subscription<scoutcar_msgs::msg::DetectTask>::SharedPtr
      sub_detect_task_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}
