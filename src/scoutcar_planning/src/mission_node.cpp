#include <algorithm>
#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <scoutcar_msgs/msg/rx_event.hpp>
#include <scoutcar_msgs/msg/car_state.hpp>


#include "graph.h"
#include "map.h"
#include "navigation.h"
#include "mission.h"

using namespace pathplan;

namespace {


const std::vector<int> kFixedPoints = {2, 4, 5, 8, 9, 12, 13, 16, 17, 18, 19, 20};
const std::set<int> kFixedSet(kFixedPoints.begin(), kFixedPoints.end());
const std::vector<mission::Edge> kTunnels = {{6, 7}, {10, 11}, {14, 15}, {18, 19}};
const int recon_target = 8;  
}  // namespace

class MissionNode : public rclcpp::Node
{
public:
  MissionNode()
  : Node("mission_node"),
    graph_(buildDefaultMap()),
    planner_(graph_, cfg_)
  {
    //规划器参数
    cfg_.turn_penalty = declare_parameter<double>("turn_penalty", 0.5);
    cfg_.u_turn_penalty = declare_parameter<double>("u_turn_penalty", 1.0);
    cfg_.tunnel_risk = declare_parameter<double>("tunnel_risk", 0.0);
    cfg_.home = 1;
    const std::string obstacle_mode = declare_parameter<std::string>(
      "obstacle_mode", "bidirectional");
    if (obstacle_mode != "bidirectional") {
      RCLCPP_WARN(get_logger(), "obstacle_mode=%s 未实现，当前使用双向边(单向阻断)模式",
                  obstacle_mode.c_str());
    }
    planner_.setConfig(cfg_);
  
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pub_path_ = create_publisher<scoutcar_msgs::msg::CarState>("mission/path_cmd", 10);
    pub_base_cmd_ = create_publisher<scoutcar_msgs::msg::CarState>(
      "mission/base_cmd", rclcpp::QoS(10).reliable());
    pub_debug_ = create_publisher<scoutcar_msgs::msg::CarState>(
      "mission/debug_cmd",state_qos);
    pub_mission_state_ = create_publisher<scoutcar_msgs::msg::CarState>(
      "mission/mission_state", state_qos);

    sub_event_ = create_subscription<scoutcar_msgs::msg::RxEvent>(
      "mcu/rx_event", 10,
      [this](const scoutcar_msgs::msg::RxEvent::SharedPtr msg) { on_mcu_event(msg->event); });

    sub_recon_ = create_subscription<std_msgs::msg::Bool>(
      "mission/recon_found", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          on_recon_found();
        }
      });
    sub_is_btp_ = create_subscription<std_msgs::msg::Empty>(//感知节点在FINDING_BTP状态找到合适时机发送该消息
      "perception/is_btp",10,
      [this](const std_msgs::msg::Empty::SharedPtr) {  do_turning(); });
    sub_obstacle_ = create_subscription<std_msgs::msg::Empty>(
      "obstacle/event", 10,
      [this](const std_msgs::msg::Empty::SharedPtr) {  });

    prepare_mission();
  }

private:
  
  void on_mcu_event(uint8_t event)
  {
    switch (event) {
      case scoutcar_msgs::msg::RxEvent::START:
        if (car_state_.mission_state == scoutcar_msgs::msg::CarState::WAIT_START &&
            car_state_.system_status == scoutcar_msgs::msg::CarState::SYSTEM_READY &&
            segments_.size() >= 2) {
          do_start();
        } else {
          RCLCPP_WARN(get_logger(), "当前状态不允许启动任务");
        }
        break;
      case scoutcar_msgs::msg::RxEvent::STOP:
        prepare_mission();
        break;
      case scoutcar_msgs::msg::RxEvent::ARRIVED:
        on_arrived();
        break;
      case scoutcar_msgs::msg::RxEvent::TURN_FINISHED:
        on_turn_finished();
        break;
      default:
        break;
    }
  }
  
  void prepare_mission()
  {
    car_state_.mission_state = scoutcar_msgs::msg::CarState::FINISHED;
    car_state_.system_status = scoutcar_msgs::msg::CarState::SYSTEM_UNKNOWN;
    car_state_.fixed_remaining = static_cast<uint8_t>(kFixedPoints.size());
    car_state_.random_remaining = static_cast<uint8_t>(recon_target);

    car_state_.segment_start = -1;
    car_state_.segment_goal = -1;
    car_state_.segment_index = -1;
    
    //约定下位机发送bb和初始情况下摄像头已复位且停止
    car_state_.front_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
    car_state_.turn_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
    car_state_.arrival_action = scoutcar_msgs::msg::CarState::DIRECTION_NONE;
    car_state_.recon_result_valid = false;
    car_state_.recon_left_result = 0;
    car_state_.recon_right_result = 0;

    publish_car_state();

    if (replan_mission() != 0 || segments_.size() < 2) {
      RCLCPP_WARN(get_logger(), "重新规划失败，保持等待");
      car_state_.system_status = scoutcar_msgs::msg::CarState::SYSTEM_ERROR;
      pub_debug_->publish(car_state_);
      publish_car_state();
      return;
    }

    car_state_.system_status = scoutcar_msgs::msg::CarState::SYSTEM_READY;
    car_state_.mission_state = scoutcar_msgs::msg::CarState::WAIT_START;
    pub_debug_->publish(car_state_);
    publish_car_state();
    RCLCPP_INFO(get_logger(), "准备就绪，等待启动命令，路径共 %zu 段）", segments_.size()-1);

  }
  void do_start()
  {
    car_state_.mission_state = scoutcar_msgs::msg::CarState::DRIVING;
    car_state_.segment_start = segments_[0].node;
    car_state_.segment_goal = segments_[0].next;
    car_state_.arrival_action = static_cast<uint8_t>(segments_[1].action);
    car_state_.segment_index = 0;
    RCLCPP_INFO(get_logger(), "任务开始");
    pub_path_->publish(car_state_);
    publish_car_state();
  }

  void on_arrived()
  {
    if(car_state_.mission_state != scoutcar_msgs::msg::CarState::DRIVING){
      RCLCPP_WARN(get_logger(),"小车在非巡航期间接受到到达命令(来自串口)");
      return;
    }

    switch (car_state_.arrival_action) {
    case scoutcar_msgs::msg::CarState::DIRECTION_AHEAD: {//小车直行，复位转向摄像头
      record_arrival_at_node();
      car_state_.turn_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
      advance_to_next_segment();
      return;
    }
    case scoutcar_msgs::msg::CarState::DIRECTION_LEFT:
    case scoutcar_msgs::msg::CarState::DIRECTION_RIGHT://小车接近路口转向，转动转向摄像头，等待感知节点发送转向命令
      car_state_.mission_state = scoutcar_msgs::msg::CarState::FINDING_BTP;
      car_state_.turn_camera_pose = car_state_.arrival_action;
      break;
    case scoutcar_msgs::msg::CarState::DIRECTION_NONE://小车停车，已经到达终点
      record_arrival_at_node();
      car_state_.mission_state = scoutcar_msgs::msg::CarState::FINISHED;
      car_state_.segment_start = -1;
      car_state_.segment_goal = -1;
      car_state_.segment_index = -1;
      car_state_.arrival_action = scoutcar_msgs::msg::CarState::DIRECTION_NONE;
      publish_car_state();
      pub_base_cmd_->publish(car_state_);
      RCLCPP_INFO(get_logger(), "任务完成，小车停在节点 %d", current_node_);
      return;
    case scoutcar_msgs::msg::CarState::DIRECTION_UTURN:
      record_arrival_at_node();
      car_state_.mission_state = scoutcar_msgs::msg::CarState::TURNING;
      car_state_.front_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
      car_state_.turn_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
      publish_car_state();
      pub_base_cmd_->publish(car_state_);
      return;
    default:
      RCLCPP_ERROR(get_logger(), "未知到达动作 %u", car_state_.arrival_action);
      return;
    }
    car_state_.front_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
    publish_car_state();
    pub_base_cmd_->publish(car_state_);
  }

  void do_turning()//向小车发送车身转向和摄像头转回命令
  {
    if(car_state_.mission_state != scoutcar_msgs::msg::CarState::FINDING_BTP){
      RCLCPP_WARN(get_logger(),"小车在非路口接受到左右转向命令（来自感知节点)");
      return;
    }
    car_state_.mission_state = scoutcar_msgs::msg::CarState::TURNING;
    car_state_.front_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
    car_state_.turn_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
    record_arrival_at_node();
    publish_car_state();
    pub_base_cmd_->publish(car_state_);
  }

  void on_turn_finished()
  {//默认没走完
    if(car_state_.mission_state != scoutcar_msgs::msg::CarState::TURNING){
      RCLCPP_WARN(get_logger(),"在非转向时接收到转向完成的消息(来自串口)");
      return;
    }
    advance_to_next_segment();
  }

  // ═══════════════ 规划 ═══════════════

  // 复位式重规划（上电/每次 0xAA）：清掉上次任务的障碍阻断与进度
  int replan_mission()
  {
    for (const auto& e : blocked_edges_) {
      graph_.setBlocked(e.first, e.second, false);
    }
    blocked_edges_.clear();
    fixed_done_.clear();
    tunnel_done_.clear();
    searched_edges_.clear();
    recon_found_ = 0;
    recon_finished_ = false;
    current_node_ = cfg_.home;
    car_state_.fixed_remaining = static_cast<uint8_t>(kFixedPoints.size());
    car_state_.random_remaining = static_cast<uint8_t>(recon_target);
    return replan_from_current();
  }

  // 从当前位置重规划（障碍时也用），结果存入 segments_
  int replan_from_current()
  {
    mission::MissionState st;
    st.current = current_node_;
    for (int n : kFixedPoints) {
      if (!fixed_done_.count(n)) {
        st.fixed_left.push_back(n);
      }
    }
    for (const auto& t : kTunnels) {
      if (!tunnel_done_.count(t)) {
        st.tunnel_left.push_back(t);
      }
    }
    st.searched = searched_edges_;
    st.recon_found = recon_found_;

    const std::vector<int> route = planner_.plan(st);
    if (route.size() < 2) {
      if (route.size() == 1 && route[0] == cfg_.home) {
        NavigationPlan nav = planNavigation(route);
        segments_ = nav.steps;
        RCLCPP_INFO(get_logger(), "剩余需求不可达或已清空，原地停车结束任务");
        return 0;
      }
      RCLCPP_WARN(get_logger(), "任务规划失败");
      return -1;
    }
    NavigationPlan nav = planNavigation(route);
    segments_ = nav.steps;
    RCLCPP_INFO(get_logger(), "任务规划完成，共 %zu 段", segments_.size());
    return 0;
  }

  void publish_car_state()
  {
    car_state_.header.stamp = now();
    ++car_state_.revision;
    pub_mission_state_->publish(car_state_);
  }

  void record_arrival_at_node()
  {
    const int reached_node = car_state_.segment_goal;
    if (reached_node < 0) {
      return;
    }

    current_node_ = reached_node;

    if (car_state_.segment_start >= 0) {
      const mission::Edge edge =
        mission::normEdge(car_state_.segment_start, reached_node);
      searched_edges_.insert(edge);
      if (std::find(kTunnels.begin(), kTunnels.end(), edge) != kTunnels.end()) {
        tunnel_done_.insert(edge);
      }
    }

    if (kFixedSet.count(reached_node) && fixed_done_.insert(reached_node).second) {
      if (car_state_.fixed_remaining > 0) {
        --car_state_.fixed_remaining;
      }
      RCLCPP_INFO(
        get_logger(), "到达固定点 %d，剩余固定点 %u",
        reached_node, static_cast<unsigned>(car_state_.fixed_remaining));
    }
  }

  void advance_to_next_segment()
  {
    const int next_index = car_state_.segment_index + 1;
    if (next_index < 0 ||
        static_cast<size_t>(next_index + 1) >= segments_.size()) {
      RCLCPP_ERROR(
        get_logger(), "无法推进路径：当前段=%d，路径记录数=%zu",
        car_state_.segment_index, segments_.size());
      car_state_.mission_state = scoutcar_msgs::msg::CarState::FINISHED;
      car_state_.segment_start = -1;
      car_state_.segment_goal = -1;
      car_state_.segment_index = -1;
      car_state_.arrival_action = scoutcar_msgs::msg::CarState::DIRECTION_NONE;
      pub_base_cmd_->publish(car_state_);
      publish_car_state();
      return;
    }

    car_state_.segment_index = next_index;
    car_state_.segment_start = segments_[next_index].node;
    car_state_.segment_goal = segments_[next_index].next;
    car_state_.arrival_action =
      static_cast<uint8_t>(segments_[next_index + 1].action);
    car_state_.mission_state = scoutcar_msgs::msg::CarState::DRIVING;

    pub_path_->publish(car_state_);
    pub_base_cmd_->publish(car_state_);
    publish_car_state();
  }

  void on_recon_found()
  {
    if (recon_found_ >= recon_target) {
      return;
    }

    ++recon_found_;
    if (car_state_.random_remaining > 0) {
      --car_state_.random_remaining;
    }
    publish_car_state();
    RCLCPP_INFO(
      get_logger(), "找到侦察目标，剩余 %u（已找到 %d/%d）",
      static_cast<unsigned>(car_state_.random_remaining),
      recon_found_, recon_target);

  }

  std::vector<StepCommand> segments_;


  int recon_found_ = 0;//已发现侦察点数量
  bool recon_finished_ = false;//侦察任务是否完成
  int current_node_ = 1;// Mission 内部记录最后确认到达的节点

  std::set<int> fixed_done_;
  std::set<mission::Edge> searched_edges_;
  std::set<mission::Edge> tunnel_done_;
  std::set<std::pair<int, int>> blocked_edges_;

  // 规划器（声明顺序 = 初始化顺序：cfg_ → graph_ → planner_）
  mission::CostConfig cfg_;
  Graph graph_;
  mission::MissionPlanner planner_;

  scoutcar_msgs::msg::CarState car_state_;

  rclcpp::Publisher<scoutcar_msgs::msg::CarState>::SharedPtr pub_base_cmd_;
  rclcpp::Publisher<scoutcar_msgs::msg::CarState>::SharedPtr pub_path_;
  rclcpp::Publisher<scoutcar_msgs::msg::CarState>::SharedPtr pub_mission_state_;
  rclcpp::Publisher<scoutcar_msgs::msg::CarState>::SharedPtr pub_debug_;

  rclcpp::Subscription<scoutcar_msgs::msg::RxEvent>::SharedPtr sub_event_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_recon_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_obstacle_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_is_btp_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}
