#include <algorithm>
#include <chrono>
#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/empty.hpp>
#include <scoutcar_msgs/msg/rx_event.hpp>
#include <scoutcar_msgs/msg/car_state.hpp>
#include <scoutcar_msgs/msg/detect_task.hpp>
#include <scoutcar_msgs/msg/recon_result.hpp>


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

struct RouteSegment {
  int start;
  int goal;
  TurnAction arrival_action;
};
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
    planner_.setConfig(cfg_);

    route_mode_ = declare_parameter<std::string>("route_mode", "auto");
    //隧道屏蔽往前看几段：0=只看当前段；隧道两侧壁在进洞前就会出现在画面里时调大
    tunnel_lookahead_segments_ =
      declare_parameter<int>("tunnel_lookahead_segments", 0);
    const auto route_nodes =
      declare_parameter<std::vector<int64_t>>(
        "route_nodes", std::vector<int64_t>{});
    fixed_route_.reserve(route_nodes.size());
    for (const int64_t node : route_nodes) {
      fixed_route_.push_back(static_cast<int>(node));
    }

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

    sub_recon_ = create_subscription<scoutcar_msgs::msg::ReconResult>(
      "perception/recon_result", 10,
      [this](const scoutcar_msgs::msg::ReconResult::SharedPtr) {
        on_recon_completed();
      });
    sub_obstacle_ = create_subscription<std_msgs::msg::Empty>(
      "obstacle/event", 10,
      [this](const std_msgs::msg::Empty::SharedPtr) { on_obstacle(); });
    sub_detect_task_ = create_subscription<scoutcar_msgs::msg::DetectTask>(
      "mission/detect_task", 10,
      [this](const scoutcar_msgs::msg::DetectTask::SharedPtr msg) {
        on_detect_task(msg->status);
      });

    prepare_mission();
  }

private:
  
  void on_mcu_event(uint8_t event)
  {
    switch (event) {
      case scoutcar_msgs::msg::RxEvent::START:
        if (car_state_.mission_state == scoutcar_msgs::msg::CarState::WAIT_START &&
            !segments_.empty()) {
          do_start();
        } else {
          RCLCPP_WARN(get_logger(), "当前状态不允许启动任务");
        }
        break;
      case scoutcar_msgs::msg::RxEvent::STOP:
        prepare_mission();
        break;
      case scoutcar_msgs::msg::RxEvent::BTP:
        on_enter_btp();
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
    segments_.clear();
    route_nodes_.clear();
    route_segment_offset_ = 0;
    car_state_.fixed_remaining = static_cast<uint8_t>(kFixedPoints.size());
    car_state_.random_remaining = static_cast<uint8_t>(recon_target);

    car_state_.segment_start = -1;
    car_state_.segment_goal = -1;
    car_state_.segment_index = -1;
    
    //约定下位机发送bb和初始情况下摄像头已复位且停止
    car_state_.front_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
    car_state_.turn_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
    car_state_.arrival_action = scoutcar_msgs::msg::CarState::DIRECTION_NONE;
    publish_car_state();

    if (replan_mission() != 0 || segments_.empty()) {
      enter_error("重新规划失败，无法执行任务");
      return;
    }

    car_state_.mission_state = scoutcar_msgs::msg::CarState::WAIT_START;
    pub_debug_->publish(car_state_);
    publish_car_state();
    RCLCPP_INFO(get_logger(), "准备就绪，等待启动命令，路径共 %zu 段", segments_.size());

  }

  void do_start()
  {
    car_state_.mission_state = scoutcar_msgs::msg::CarState::DRIVING;
    car_state_.segment_start = segments_[0].start;
    car_state_.segment_goal = segments_[0].goal;
    car_state_.arrival_action = static_cast<uint8_t>(segments_[0].arrival_action);
    car_state_.segment_index = 0;
    RCLCPP_INFO(get_logger(), "任务开始");
    pub_path_->publish(car_state_);
    publish_car_state();
  }

  void on_enter_btp()
  {
    const bool ending_detect =
      car_state_.mission_state == scoutcar_msgs::msg::CarState::DETECTING;
    if(car_state_.mission_state != scoutcar_msgs::msg::CarState::DRIVING &&
       !ending_detect){
      RCLCPP_WARN(get_logger(),"小车在非巡航期间（包括侦察期间）接收到 BTP 进入命令");
      return;
    }
    if(car_state_.arrival_action != scoutcar_msgs::msg::CarState::DIRECTION_LEFT &&
       car_state_.arrival_action != scoutcar_msgs::msg::CarState::DIRECTION_RIGHT){
      RCLCPP_WARN(get_logger(),"当前路段不是左右转，忽略 BTP 进入命令");
      return;
    }
    car_state_.mission_state = scoutcar_msgs::msg::CarState::FINDING_BTP;
    if (ending_detect) {
      publish_car_state();
      RCLCPP_INFO(
        get_logger(),
        "侦察中进入 BTP：立即结束侦察，沿用已转好的转向相机位置");
      return;
    }
    car_state_.front_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
    car_state_.turn_camera_pose = car_state_.arrival_action;
    publish_car_state();
    pub_base_cmd_->publish(car_state_);
  }

  void on_arrived()
  {
    switch (car_state_.arrival_action) {
    case scoutcar_msgs::msg::CarState::DIRECTION_AHEAD:
      if(car_state_.mission_state != scoutcar_msgs::msg::CarState::DRIVING &&
         car_state_.mission_state != scoutcar_msgs::msg::CarState::DETECTING){
        RCLCPP_WARN(get_logger(),"小车在非巡航期间接收到直行到达命令");
        return;
      }
      if (car_state_.mission_state == scoutcar_msgs::msg::CarState::DETECTING) {
        car_state_.front_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
        car_state_.turn_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
        RCLCPP_INFO(get_logger(), "侦察中到达直线路段终点，提前结束侦察并回正相机");
      }
      record_arrival_at_node();
      car_state_.turn_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
      advance_to_next_segment();
      return;
    case scoutcar_msgs::msg::CarState::DIRECTION_LEFT:
    case scoutcar_msgs::msg::CarState::DIRECTION_RIGHT:
      if(car_state_.mission_state != scoutcar_msgs::msg::CarState::FINDING_BTP){
        RCLCPP_WARN(get_logger(),"小车在非 BTP 阶段接收到最佳转向点命令");
        return;
      }
      car_state_.mission_state = scoutcar_msgs::msg::CarState::TURNING;
      car_state_.front_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
      car_state_.turn_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
      record_arrival_at_node();
      publish_car_state();
      pub_base_cmd_->publish(car_state_);
      return;
    case scoutcar_msgs::msg::CarState::DIRECTION_NONE://小车停车，已经到达终点
      if(car_state_.mission_state != scoutcar_msgs::msg::CarState::DRIVING){
        RCLCPP_WARN(get_logger(),"小车在非巡航期间接收到终点到达命令");
        return;
      }
      record_arrival_at_node();
      car_state_.mission_state = scoutcar_msgs::msg::CarState::FINISHED;
      car_state_.segment_start = -1;
      car_state_.segment_goal = -1;
      car_state_.segment_index = -1;
      car_state_.arrival_action = scoutcar_msgs::msg::CarState::DIRECTION_NONE;
      publish_car_state();
      RCLCPP_INFO(get_logger(), "任务完成，小车停在节点 %d", current_node_);
      return;
    case scoutcar_msgs::msg::CarState::DIRECTION_UTURN:
      if(car_state_.mission_state != scoutcar_msgs::msg::CarState::DRIVING){
        RCLCPP_WARN(get_logger(),"小车在非巡航期间接收到掉头命令");
        return;
      }
      record_arrival_at_node();
      car_state_.mission_state = scoutcar_msgs::msg::CarState::TURNING;
      car_state_.front_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
      car_state_.turn_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
      car_state_.segment_start = current_node_;
      car_state_.segment_goal = current_node_;
      publish_car_state();
      pub_path_->publish(car_state_);
      return;
    default:
      RCLCPP_ERROR(get_logger(), "未知到达动作 %u", car_state_.arrival_action);
      return;
    }
  }

  void on_detect_task(uint8_t status)
  {
    if (status == scoutcar_msgs::msg::DetectTask::START) {
      if (car_state_.mission_state != scoutcar_msgs::msg::CarState::DRIVING) {
        RCLCPP_WARN(get_logger(), "当前状态不允许开始侦察");
        return;
      }
      car_state_.mission_state = scoutcar_msgs::msg::CarState::DETECTING;
      if (car_state_.arrival_action ==
          scoutcar_msgs::msg::CarState::DIRECTION_LEFT) {
        car_state_.front_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_RIGHT;
        car_state_.turn_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_LEFT;
      } else {
        car_state_.front_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_LEFT;
        car_state_.turn_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_RIGHT;
      }
      publish_car_state();
      pub_base_cmd_->publish(car_state_);
      RCLCPP_INFO(
        get_logger(), "开始侦察：前视相机姿态=%u，转向相机姿态=%u",
        static_cast<unsigned>(car_state_.front_camera_pose),
        static_cast<unsigned>(car_state_.turn_camera_pose));
      return;
    }

    if (status == scoutcar_msgs::msg::DetectTask::END) {
      if (car_state_.mission_state != scoutcar_msgs::msg::CarState::DETECTING) {
        RCLCPP_WARN(get_logger(), "当前未在侦察，忽略结束命令");
        return;
      }
      if (car_state_.arrival_action ==
          scoutcar_msgs::msg::CarState::DIRECTION_LEFT) {
        car_state_.front_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
        car_state_.turn_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_LEFT;
      } else {
        car_state_.front_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
        car_state_.turn_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_RIGHT;
      }
      publish_car_state();
      pub_base_cmd_->publish(car_state_);
      detect_reset_timer_ = create_wall_timer(
        std::chrono::milliseconds(500), [this]() {
          detect_reset_timer_->cancel();
          if (car_state_.mission_state != scoutcar_msgs::msg::CarState::DETECTING ||
              car_state_.front_camera_pose !=
                scoutcar_msgs::msg::CarState::DIRECTION_AHEAD ||
              car_state_.turn_camera_pose !=
                scoutcar_msgs::msg::CarState::DIRECTION_AHEAD) {
            return;
          }
          car_state_.mission_state = scoutcar_msgs::msg::CarState::DRIVING;
          publish_car_state();
          RCLCPP_INFO(get_logger(), "侦察相机回正完成，恢复巡航");
        });
      RCLCPP_INFO(get_logger(), "结束侦察：两路相机复位，等待 0.5 秒");
      return;
    }

    if (status == scoutcar_msgs::msg::DetectTask::AUTO_END) {
      if (car_state_.mission_state != scoutcar_msgs::msg::CarState::DETECTING) {
        RCLCPP_WARN(get_logger(), "自动结束侦察：当前未在侦察，忽略");
        return;
      }
      //两路相机都回正（巡航状态的不变量），并直接恢复巡航
      car_state_.mission_state = scoutcar_msgs::msg::CarState::DRIVING;
      car_state_.front_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
      car_state_.turn_camera_pose = scoutcar_msgs::msg::CarState::DIRECTION_AHEAD;
      publish_car_state();
      pub_base_cmd_->publish(car_state_);
      RCLCPP_INFO(get_logger(), "自动侦察结束，相机回正，恢复巡航");
    }
  }

  void on_turn_finished()
  {
    if (car_state_.mission_state == scoutcar_msgs::msg::CarState::TURNING) {
      advance_to_next_segment();
      return;
    }

    RCLCPP_WARN(get_logger(), "当前状态不接受 0xEE");
  }

  // ═══════════════ 规划 ═══════════════

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
    current_node_ = cfg_.home;
    car_state_.fixed_remaining = static_cast<uint8_t>(kFixedPoints.size());
    car_state_.random_remaining = static_cast<uint8_t>(recon_target);
    std::vector<int> route;

    if (route_mode_ == "fixed") {
      if (validate_fixed_route() != 0) {
        return -1;
      }
      route = fixed_route_;
      current_node_ = route.front();
      RCLCPP_INFO(
        get_logger(), "使用固定路线：%zu 个节点，起点 %d，终点 %d",
        route.size(), route.front(), route.back());
    } else if (route_mode_ == "auto") {
      if (plan_from(current_node_, route) != 0) {
        return -1;
      }
      RCLCPP_INFO(get_logger(), "使用自动规划路线");
    } else {
      RCLCPP_ERROR(
        get_logger(), "未知 route_mode='%s'，仅支持 auto 或 fixed",
        route_mode_.c_str());
      return -1;
    }

    if (make_route_segments(route, segments_) != 0) {
      return -1;
    }
    route_nodes_ = route;
    route_segment_offset_ = 0;
    RCLCPP_INFO(get_logger(), "任务规划完成，共 %zu 段", segments_.size());
    return 0;
  }

  int validate_fixed_route() const
  {
    if (fixed_route_.size() < 2) {
      RCLCPP_ERROR(get_logger(), "固定路线至少需要两个节点");
      return -1;
    }

    for (size_t i = 0; i < fixed_route_.size(); ++i) {
      const int node = fixed_route_[i];
      if (node < 1 || node > 20) {
        RCLCPP_ERROR(
          get_logger(), "固定路线第 %zu 个节点 %d 不在地图节点 1~20 内",
          i + 1, node);
        return -1;
      }
      if (i > 0 && !graph_.canGo(fixed_route_[i - 1], node)) {
        RCLCPP_ERROR(
          get_logger(), "固定路线相邻节点不可直达：%d→%d",
          fixed_route_[i - 1], node);
        return -1;
      }
    }
    return 0;
  }

  mission::MissionState make_mission_state(int planning_start) const
  {
    mission::MissionState st;
    st.current = planning_start;
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
    return st;
  }

  int plan_from(int planning_start, std::vector<int>& route)
  {
    route = planner_.plan(make_mission_state(planning_start));
    if (route.empty()) {
      RCLCPP_WARN(get_logger(), "任务规划失败");
      return -1;
    }
    return 0;
  }

  int make_route_segments(
    const std::vector<int>& route, std::vector<RouteSegment>& result) const
  {
    result.clear();
    if (route.empty()) {
      return -1;
    }
    if (route.size() == 1) {
      result.push_back({route[0], route[0], TurnAction::STOP});
      return 0;
    }

    const NavigationPlan nav = planNavigation(route);
    for (size_t i = 0; i + 1 < nav.steps.size(); ++i) {
      result.push_back({
        nav.steps[i].node,
        nav.steps[i].next,
        nav.steps[i + 1].action});
    }
    return result.empty() ? -1 : 0;
  }

  void enter_error(const char* reason)
  {
    RCLCPP_ERROR(get_logger(), "%s", reason);
    car_state_.mission_state = scoutcar_msgs::msg::CarState::ERROR;
    car_state_.segment_start = -1;
    car_state_.segment_goal = -1;
    car_state_.segment_index = -1;
    car_state_.arrival_action = scoutcar_msgs::msg::CarState::DIRECTION_NONE;
    pub_base_cmd_->publish(car_state_);
    pub_debug_->publish(car_state_);
    publish_car_state();
  }

  bool current_segment_is_tunnel() const
  {
    //当前段，以及其后 tunnel_lookahead_segments_ 段内是否出现隧道边
    for (int k = 0; k <= tunnel_lookahead_segments_; ++k) {
      const int index = car_state_.segment_index + k;
      if (index < 0 || static_cast<size_t>(index) >= segments_.size()) {
        break;
      }
      const mission::Edge edge =
        mission::normEdge(segments_[index].start, segments_[index].goal);
      if (std::find(kTunnels.begin(), kTunnels.end(), edge) != kTunnels.end()) {
        return true;
      }
    }
    return false;
  }

  void publish_car_state()
  {
    //隧道内两侧壁会把掩膜整行铺满，感知侧据此屏蔽自动侦察触发
    car_state_.in_tunnel = current_segment_is_tunnel() ? 1 : 0;
    car_state_.route_nodes = route_nodes_;
    car_state_.route_segment_offset = route_segment_offset_;
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
    if (next_index < 0 || static_cast<size_t>(next_index) >= segments_.size()) {
      enter_error("无法推进路径：没有下一条路径段");
      return;
    }

    car_state_.segment_index = next_index;
    car_state_.segment_start = segments_[next_index].start;
    car_state_.segment_goal = segments_[next_index].goal;
    car_state_.arrival_action =
      static_cast<uint8_t>(segments_[next_index].arrival_action);
    car_state_.mission_state = scoutcar_msgs::msg::CarState::DRIVING;

    pub_path_->publish(car_state_);
    pub_base_cmd_->publish(car_state_);
    publish_car_state();
  }

  void on_recon_completed()
  {
    const bool accepts_recon_result =
      car_state_.mission_state == scoutcar_msgs::msg::CarState::DETECTING ||
      car_state_.mission_state == scoutcar_msgs::msg::CarState::FINDING_BTP ||
      car_state_.mission_state == scoutcar_msgs::msg::CarState::DRIVING;
    if (recon_found_ >= recon_target || !accepts_recon_result) {
      return;
    }

    ++recon_found_;
    if (car_state_.random_remaining > 0) {
      --car_state_.random_remaining;
    }
    const int start = car_state_.segment_start;
    const int goal = car_state_.segment_goal;
    searched_edges_.insert(mission::normEdge(start, goal));

    std::vector<int> replanned_route;
    if (plan_from(goal, replanned_route) != 0) {
      enter_error("侦察目标更新后重新规划失败");
      return;
    }

    std::vector<int> joined_route;
    joined_route.reserve(replanned_route.size() + 1);
    joined_route.push_back(start);
    joined_route.insert(
      joined_route.end(), replanned_route.begin(), replanned_route.end());

    std::vector<RouteSegment> replanned_segments;
    if (make_route_segments(joined_route, replanned_segments) != 0) {
      enter_error("侦察目标更新后无法生成路径帧");
      return;
    }

    segments_ = std::move(replanned_segments);
    route_nodes_ = joined_route;
    route_segment_offset_ = 0;
    car_state_.segment_index = 0;
    car_state_.segment_start = segments_[0].start;
    car_state_.segment_goal = segments_[0].goal;
    car_state_.arrival_action =
      static_cast<uint8_t>(segments_[0].arrival_action);
    pub_path_->publish(car_state_);
    publish_car_state();
    RCLCPP_INFO(
      get_logger(), "完成侦察任务并从节点 %d 重规划，剩余 %u（已完成 %d/%d）",
      goal,
      static_cast<unsigned>(car_state_.random_remaining),
      recon_found_, recon_target);
  }

  void on_obstacle()
  {
    if (car_state_.mission_state != scoutcar_msgs::msg::CarState::DRIVING) {
      return;
    }

    const int start = car_state_.segment_start;
    const int goal = car_state_.segment_goal;
    graph_.setBlocked(start, goal, true);
    blocked_edges_.insert({start, goal});

    std::vector<int> replanned_route;
    if (plan_from(start, replanned_route) != 0) {
      enter_error("遇到障碍后重新规划失败");
      return;
    }

    std::vector<RouteSegment> replanned_segments;
    if (make_route_segments(replanned_route, replanned_segments) != 0) {
      enter_error("遇到障碍后无法生成路径帧");
      return;
    }

    TurnAction action_at_start = TurnAction::STOP;
    if (replanned_route.size() > 1) {
      const Heading return_heading = edgeHeading(goal, start);
      const NavigationPlan nav = planNavigation(replanned_route, return_heading);
      action_at_start = nav.steps[0].action;
    }

    segments_.clear();
    segments_.reserve(replanned_segments.size() + 2);
    segments_.push_back({start, start, TurnAction::UTURN});
    segments_.push_back({goal, start, action_at_start});
    if (replanned_route.size() > 1) {
      segments_.insert(
        segments_.end(), replanned_segments.begin(), replanned_segments.end());
    }
    route_nodes_ = replanned_route;
    route_segment_offset_ = 2;

    car_state_.mission_state = scoutcar_msgs::msg::CarState::TURNING;
    car_state_.segment_index = 0;
    car_state_.segment_start = start;
    car_state_.segment_goal = start;
    car_state_.arrival_action = scoutcar_msgs::msg::CarState::DIRECTION_UTURN;
    pub_path_->publish(car_state_);
    publish_car_state();
    RCLCPP_INFO(
      get_logger(), "障碍阻断 %d→%d，立即掉头",
      start, goal);

  }

  std::vector<RouteSegment> segments_;
  std::vector<int32_t> route_nodes_;
  int32_t route_segment_offset_ = 0;
  std::string route_mode_;
  int tunnel_lookahead_segments_ = 0;
  std::vector<int> fixed_route_;


  int recon_found_ = 0;//已发现侦察点数量
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
  rclcpp::TimerBase::SharedPtr detect_reset_timer_;

  rclcpp::Subscription<scoutcar_msgs::msg::RxEvent>::SharedPtr sub_event_;
  rclcpp::Subscription<scoutcar_msgs::msg::ReconResult>::SharedPtr sub_recon_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_obstacle_;
  rclcpp::Subscription<scoutcar_msgs::msg::DetectTask>::SharedPtr sub_detect_task_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}
