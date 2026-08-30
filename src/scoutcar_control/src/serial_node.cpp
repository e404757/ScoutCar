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

#include <std_msgs/msg/bool.hpp>
#include <scoutcar_msgs/msg/mission_status.hpp>
#include <scoutcar_msgs/msg/path_cmd.hpp>
#include <scoutcar_msgs/msg/road_boundary.hpp>
#include <scoutcar_msgs/msg/rx_event.hpp>

#include "scoutcar_control/serial.h"

class SerialNode : public rclcpp::Node
{
public:
  SerialNode() : Node("serial_node")
  {
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

    sub_path_cmd_ = create_subscription<scoutcar_msgs::msg::PathCmd>(
      "mission/path_cmd", 10,
      [this](const scoutcar_msgs::msg::PathCmd::SharedPtr msg) {
        pathplan::PathPlanFrame f{
          msg->start,
          msg->goal,
          static_cast<pathplan::TurnAction>(msg->action),
        };
        uart_send_path_segment(fd_, f);
        RCLCPP_INFO(get_logger(), "[发送] 路径段 %u→%u，到达后%s",
                    f.start, f.goal, pathplan::actionName(f.finalAction));
      });

    // 收 /mission/status → 发调试帧 FF 01 [状态] [固定剩余] [随机剩余] DD（原 send_count_update）
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sub_status_ = create_subscription<scoutcar_msgs::msg::MissionStatus>(
      "mission/status", status_qos,
      [this](const scoutcar_msgs::msg::MissionStatus::SharedPtr msg) {
        const pathplan::Status st = (msg->status == scoutcar_msgs::msg::MissionStatus::READY)
          ? pathplan::Status::READY : pathplan::Status::ERROR;
        uart_send_debug_ready(fd_, st, msg->fixed_remain, msg->random_remain);
      });

    // 收 /mission/deviation_enable → 偏差帧模式：true=发真实偏差，false=发 0
    //（转向 DD→EE / 重规划期间帧流不断，值置 0，与下位机预期一致）
    sub_dev_ = create_subscription<std_msgs::msg::Bool>(
      "mission/deviation_enable", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        const bool rising = !deviation_enabled_ && msg->data;
        deviation_enabled_ = msg->data;
        if (rising) {
          // TURN_FINISHED 后禁止把转弯期间缓存的最后一个偏差立即发出。
          // 等感知完成连续稳定帧重新捕获后，才解锁真实偏差。
          has_boundary_ = false;
          latest_deviation_ = kInvalidDeviation;
          waiting_for_fresh_boundary_ = true;
        } else if (!deviation_enabled_) {
          waiting_for_fresh_boundary_ = false;
          has_boundary_ = false;
          latest_deviation_ = kInvalidDeviation;
        }
        RCLCPP_INFO(get_logger(), "偏差帧%s", deviation_enabled_ ? "启用" : "停发");
      });

    sub_boundary_ = create_subscription<scoutcar_msgs::msg::RoadBoundary>(
      "perception/road_boundary", 10,
      [this](const scoutcar_msgs::msg::RoadBoundary::SharedPtr msg) {
        if (deviation_enabled_ && msg->valid) waiting_for_fresh_boundary_ = false;
        latest_deviation_ = msg->valid ? msg->deviation : kInvalidDeviation;
        last_boundary_time_ = std::chrono::steady_clock::now();
        has_boundary_ = true;
      });

    // ── 串口（顺序：先建接口，再开设备、起接收线程）──
    fd_ = uart_init(serial_device_.c_str(), baud_rate_);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "串口打开失败，接收线程未启动");
      return;
    }
    uart_start_rx_thread(fd_, on_rx_flag_static, this);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / deviation_rate_hz_));
    deviation_timer_ = create_wall_timer(period, [this]() { send_cached_deviation(); });
    RCLCPP_INFO(get_logger(), "偏差发送 %.1f Hz，数据超时 %d ms",
                deviation_rate_hz_, deviation_timeout_ms_);
  }

  ~SerialNode() override
  {
    // 停止顺序不能反：先 join 接收线程，再关 fd
    uart_stop_rx_thread();
    uart_close(fd_);
  }

private:
  void send_cached_deviation()
  {
    if (fd_ < 0) {
      return;
    }
    // 偏差帧流不断：转向（DD→EE）/ 重规划期间下位机仍期待持续收到偏差帧，
    // 只是值恒为 0。所以"停发"语义 = 发 0，而不是不发。
    int16_t deviation = 0;
    if (deviation_enabled_) {
      // 转弯后重捕获期间属于"循迹已启用但感知无效"，按协议发 -999。
      // 只有 deviation_enabled_=false 的明确转弯阶段才发 0。
      deviation = kInvalidDeviation;
      if (has_boundary_) {
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - last_boundary_time_).count();
        if (age <= deviation_timeout_ms_ && !waiting_for_fresh_boundary_) {
          deviation = latest_deviation_;
        }
      }
    }
    uart_send_deviation_frame(fd_, deviation);
  }

  // 接收线程回调适配：C 风格函数指针 → 成员方法
  static void on_rx_flag_static(pathplan::RxFlag flag, void * user)
  {
    static_cast<SerialNode *>(user)->on_rx_flag(flag);
  }

  void on_rx_flag(pathplan::RxFlag flag)
  {
    // 下位机的 DD（转向中）/FF（直行中）是状态流，转向/直行期间持续连发，
    // 不是单次事件。这里做边沿触发：只在 flag 变化的瞬间发布一次事件，
    // 后续重复帧直接丢弃（mission_node 另有待定转向防重入，双保险）。
    if (last_flag_.has_value() && *last_flag_ == flag) {
      return;
    }
    last_flag_ = flag;

    switch (flag) {
      case pathplan::RxFlag::START:
        RCLCPP_INFO(get_logger(), "[接收] 收到启动指令，开始执行任务");
        publish_event(scoutcar_msgs::msg::RxEvent::START);      // 0xAA
        break;
      case pathplan::RxFlag::ARRIVED:
        RCLCPP_INFO(get_logger(), "[接收] 到达路口，开始执行转向动作");
        publish_event(scoutcar_msgs::msg::RxEvent::ARRIVED);    // 0xDD
        break;
      case pathplan::RxFlag::TURN_FINISHED:
        RCLCPP_INFO(get_logger(), "[接收] 转向完成，恢复循迹");
        publish_event(scoutcar_msgs::msg::RxEvent::TURN_FINISHED);  // 0xEE
        break;
      case pathplan::RxFlag::OBSTACLE:
        RCLCPP_INFO(get_logger(), "[接收] 检测到障碍，准备重新规划");
        publish_event(scoutcar_msgs::msg::RxEvent::OBSTACLE);   // 0xCC
        break;
      case pathplan::RxFlag::STRAIGHT:
        // 直行心跳流的开始，透传给话题（mission_node 忽略），不打日志
        publish_event(scoutcar_msgs::msg::RxEvent::STRAIGHT);   // 0xFF
        break;
      case pathplan::RxFlag::PT_80:
      case pathplan::RxFlag::PT_40:
        RCLCPP_DEBUG(get_logger(), "[接收] 下位机请求切换预瞄行，已忽略（上位机自动选择）");
        break;
    }
  }

  void publish_event(uint8_t event)
  {
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
  std::optional<pathplan::RxFlag> last_flag_;   // 上一个收到的 flag（边沿触发去重）
  static constexpr int16_t kInvalidDeviation = -999;
  int16_t latest_deviation_ = kInvalidDeviation;
  bool has_boundary_ = false;
  bool waiting_for_fresh_boundary_ = false;
  std::chrono::steady_clock::time_point last_boundary_time_{};

  rclcpp::Publisher<scoutcar_msgs::msg::RxEvent>::SharedPtr pub_event_;
  rclcpp::TimerBase::SharedPtr deviation_timer_;
  rclcpp::Subscription<scoutcar_msgs::msg::PathCmd>::SharedPtr sub_path_cmd_;
  rclcpp::Subscription<scoutcar_msgs::msg::MissionStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_dev_;
  rclcpp::Subscription<scoutcar_msgs::msg::RoadBoundary>::SharedPtr sub_boundary_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}
