// mission_node —— 任务节点（巡逻任务状态机）
//
// 对应原工程 main.cc 的任务状态机部分：
//   全局原子变量 + while(1) 轮询  →  节点私有成员 + 事件回调（事件驱动，无轮询）
//
// 输入：
//   /mcu/rx_event        下位机事件（START=复位开跑 / ARRIVED=到达并开始转向 /
//                                      TURN_FINISHED=转向结束 / OBSTACLE=障碍重规划）
//   /mission/recon_found 侦查点发现（v1 人工/调试发布；v3 detection 接入）
// 输出：
//   /mission/path_cmd           路径段指令 → serial_node（打包 FF 02 发下位机）
//   /mission/status             任务状态 → serial_node（发调试帧 FF 01）/ web / telemetry
//   /mission/deviation_enable   偏差帧开关 → serial_node（掉头/重规划期间关闭）

#include <algorithm>
#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <scoutcar_msgs/msg/mission_status.hpp>
#include <scoutcar_msgs/msg/path_cmd.hpp>
#include <scoutcar_msgs/msg/rx_event.hpp>

#include "graph.h"
#include "map.h"
#include "navigation.h"
#include "mission.h"

using namespace pathplan;

namespace {


const std::vector<int> kFixedPoints = {2, 4, 5, 8, 9, 12, 13, 16, 17, 18, 19, 20};
const std::set<int> kFixedSet(kFixedPoints.begin(), kFixedPoints.end());
const std::vector<mission::Edge> kTunnels = {{6, 7}, {10, 11}, {14, 15}, {18, 19}};

}  // namespace

class MissionNode : public rclcpp::Node
{
public:
  MissionNode()
  : Node("mission_node"),
    graph_(buildDefaultMap()),
    planner_(graph_, cfg_)
  {
    // ── 参数（对应 config/cityscout.yaml 的 planning 节）──
    cfg_.turn_penalty = declare_parameter<double>("turn_penalty", 0.5);
    cfg_.u_turn_penalty = declare_parameter<double>("u_turn_penalty", 1.0);
    cfg_.tunnel_risk = declare_parameter<double>("tunnel_risk", 0.0);
    cfg_.recon_target = declare_parameter<int>("recon_target", 8);
    cfg_.home = 1;
    const std::string obstacle_mode = declare_parameter<std::string>(
      "obstacle_mode", "bidirectional");
    if (obstacle_mode != "bidirectional") {
      RCLCPP_WARN(get_logger(), "obstacle_mode=%s 未实现，当前使用双向边(单向阻断)模式",
                  obstacle_mode.c_str());
    }
    planner_.setConfig(cfg_);
    random_remain_ = cfg_.recon_target;

    // ── 接口 ──
    pub_path_ = create_publisher<scoutcar_msgs::msg::PathCmd>("mission/path_cmd", 10);
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pub_status_ = create_publisher<scoutcar_msgs::msg::MissionStatus>(
      "mission/status", status_qos);
    pub_dev_ = create_publisher<std_msgs::msg::Bool>("mission/deviation_enable", 10);

    sub_event_ = create_subscription<scoutcar_msgs::msg::RxEvent>(
      "mcu/rx_event", 10,
      [this](const scoutcar_msgs::msg::RxEvent::SharedPtr msg) { on_event(msg->event); });

    sub_recon_ = create_subscription<std_msgs::msg::Bool>(
      "mission/recon_found", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          on_recon_found();
        }
      });

    // 上电：发一次就绪状态（serial_node 收到后发调试帧，同原 send_count_update）
    publish_status();
  }

private:
  // ═══════════════ 事件入口 ═══════════════

  void on_event(uint8_t event)
  {
    switch (event) {
      case scoutcar_msgs::msg::RxEvent::START:
        do_restart();
        break;
      case scoutcar_msgs::msg::RxEvent::ARRIVED:
        on_arrived();
        break;
      case scoutcar_msgs::msg::RxEvent::TURN_FINISHED:
        on_turn_finished();
        break;
      case scoutcar_msgs::msg::RxEvent::OBSTACLE:
        do_obstacle();
        break;
      default:
        break;
    }
  }

  // 0xAA：复位 + 重规划 + 开跑（原 restart_requested 分支 + replan_mission）
  void do_restart()
  {
    set_deviation(false);
    if (replan_mission() != 0) {
      RCLCPP_WARN(get_logger(), "重新规划失败，保持等待");
      return;
    }
    fixed_remain_ = static_cast<int>(kFixedPoints.size());
    random_remain_ = cfg_.recon_target;
    uturn_pending_ = false;
    awaiting_turn_finished_ = false;
    mission_started_ = true;
    mission_done_ = false;
    publish_status();
    // 原主循环"下一轮"即发第一段：事件驱动下直接发
    send_segment(0);
    pending_from_ = segments_[0].node;
    pending_arrival_ = segments_[0].next;
    first_segment_sent_ = true;
    seg_index_ = 1;
    set_deviation(true);
    RCLCPP_INFO(get_logger(), "任务已复位，开始新一轮（共 %zu 段）", segments_.size());
  }

  // 0xDD：到达路口并开始执行动作。上位机知道自己发的下一段帧里带的 action，
  // 它就是小车此刻正在执行的动作（= segments_[seg_index_].action）：
  //   左转/右转/掉头 → 真转向，偏差置 0（帧流不断），直到 EE 恢复；
  //   直行/停车     → 路口直行通过（MCU 会 DD 后紧跟 EE），偏差保持真实值不中断。
  void on_arrived()
  {
    if (!mission_started_ || mission_done_) {
      return;
    }
    if (awaiting_turn_finished_) {
      RCLCPP_DEBUG(get_logger(), "转向尚未完成，忽略重复的到达通知");
      return;
    }
    awaiting_turn_finished_ = true;
    const TurnAction acting = seg_index_ < segments_.size()
      ? segments_[seg_index_].action : TurnAction::STOP;
    if (acting == TurnAction::LEFT || acting == TurnAction::RIGHT ||
        acting == TurnAction::UTURN)
    {
      set_deviation(false);
    }
    if (uturn_pending_) {
      RCLCPP_DEBUG(get_logger(), "掉头尚未完成，忽略到达通知");
      return;
    }
    if (!first_segment_sent_) {
      send_segment(0);
      pending_from_ = segments_[0].node;
      pending_arrival_ = segments_[0].next;
      first_segment_sent_ = true;
      return;
    }
    // 正常推进：更新位置与已搜索边
    if (pending_arrival_ > 0) {
      current_node_ = pending_arrival_;
      on_fixed_reached(current_node_);
      if (pending_from_ > 0) {
        mission::Edge e = mission::normEdge(pending_from_, pending_arrival_);
        searched_edges_.insert(e);
        if (std::find(kTunnels.begin(), kTunnels.end(), e) != kTunnels.end()) {
          tunnel_done_.insert(e);
        }
      }
      // 侦查点钩子：v1 由 /mission/recon_found 订阅触发（原 on_edge_traversed 写死 false）
    }
    // 侦查点找齐：收尾重规划（原收尾分支）
    if (recon_found_ >= cfg_.recon_target && !recon_finished_) {
      recon_finished_ = true;
      if (replan_from_current() == 0) {
        RCLCPP_INFO(get_logger(), "侦查点找齐，收尾重规划");
        send_segment(0);
        pending_from_ = segments_[0].node;
        pending_arrival_ = segments_[0].next;
        first_segment_sent_ = true;
        seg_index_ = 1;
        mission_started_ = true;
        mission_done_ = false;
      }
    } else if (seg_index_ < segments_.size() - 1) {
      send_segment(seg_index_);
      pending_from_ = segments_[seg_index_].node;
      pending_arrival_ = segments_[seg_index_].next;
      seg_index_++;
      if (seg_index_ >= segments_.size() - 1) {
        mission_done_ = true;
        RCLCPP_INFO(get_logger(), "已发送最后一段（终点停车），任务结束");
      }
    } else {
      mission_done_ = true;
      RCLCPP_INFO(get_logger(), "任务结束");
    }
    publish_status();
  }

  // 0xEE：动作执行结束。真转向 → 恢复偏差；路口直行（DD 后紧跟 EE）→ 偏差
  // 从未关闭，重发一次启用是幂等确认。障碍掉头则先发重规划后的第一段。
  void on_turn_finished()
  {
    if (!mission_started_) {
      return;
    }
    awaiting_turn_finished_ = false;
    if (mission_done_) {
      return;
    }
    if (uturn_pending_) {
      if (segments_.empty()) {
        RCLCPP_WARN(get_logger(), "掉头结束，但重规划路径为空");
        return;
      }
      send_segment(0);
      pending_from_ = segments_[0].node;
      pending_arrival_ = segments_[0].next;
      first_segment_sent_ = true;
      uturn_pending_ = false;
      seg_index_ = 1;
    }
    set_deviation(true);
    RCLCPP_INFO(get_logger(), "转向完成，恢复循迹偏差发送");
  }

  // 0xCC：障碍 → 阻断当前边 + 重规划 + 发掉头（原 obstacle_requested 分支）
  void do_obstacle()
  {
    set_deviation(false);
    if (pending_from_ > 0 && pending_arrival_ > 0) {
      graph_.setBlocked(pending_from_, pending_arrival_, true);
      blocked_edges_.insert({pending_from_, pending_arrival_});
      RCLCPP_INFO(get_logger(), "阻断边 %d→%d（单向，反向仍可走）", pending_from_, pending_arrival_);
    }
    if (replan_from_current() != 0) {
      RCLCPP_WARN(get_logger(), "障碍重规划失败");
      return;
    }
    scoutcar_msgs::msg::PathCmd ut;
    ut.start = static_cast<uint8_t>(current_node_);
    ut.goal = static_cast<uint8_t>(current_node_);
    ut.action = scoutcar_msgs::msg::PathCmd::UTURN;
    pub_path_->publish(ut);
    uturn_pending_ = true;
    awaiting_turn_finished_ = true;
    mission_started_ = true;
    mission_done_ = false;
    first_segment_sent_ = false;
    seg_index_ = 1;
    RCLCPP_INFO(get_logger(), "障碍，已发送掉头");
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
    current_node_ = 1;
    pending_from_ = -1;
    pending_arrival_ = -1;
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

  // ═══════════════ 发布与状态 ═══════════════

  // 发布第 i 段路径（语义与原 sendSegmentLog 一致：action = 到达 goal 后执行的动作）
  void send_segment(size_t i)
  {
    if (segments_.empty() || i >= segments_.size()) {
      return;
    }
    const StepCommand& cur = segments_[i];
    const uint8_t goal = cur.next >= 0
      ? static_cast<uint8_t>(cur.next)
      : static_cast<uint8_t>(cur.node);
    TurnAction act = TurnAction::STOP;
    if (i + 1 < segments_.size()) {
      act = segments_[i + 1].action;
    }

    scoutcar_msgs::msg::PathCmd cmd;
    cmd.start = static_cast<uint8_t>(cur.node);
    cmd.goal = goal;
    cmd.action = static_cast<uint8_t>(act);
    pub_path_->publish(cmd);
    RCLCPP_INFO(get_logger(), "[任务] 路径段 %u→%u 到达后动作[%s]",
                cmd.start, cmd.goal, actionName(act));
  }

  void set_deviation(bool enabled)
  {
    std_msgs::msg::Bool b;
    b.data = enabled;
    pub_dev_->publish(b);
  }

  void publish_status()
  {
    scoutcar_msgs::msg::MissionStatus st;
    st.status = scoutcar_msgs::msg::MissionStatus::READY;
    st.fixed_remain = static_cast<uint8_t>(fixed_remain_);
    st.random_remain = static_cast<uint8_t>(random_remain_);
    st.current_node = current_node_;
    st.seg_index = static_cast<int32_t>(seg_index_);
    st.state = mission_done_ ? 2 : (mission_started_ ? 1 : 0);
    pub_status_->publish(st);
  }

  void on_fixed_reached(int node)
  {
    if (!kFixedSet.count(node)) {
      return;  // 非固定点不计数
    }
    if (fixed_done_.insert(node).second) {
      fixed_remain_ = fixed_remain_ > 0 ? fixed_remain_ - 1 : 0;
      publish_status();
      RCLCPP_INFO(get_logger(), "到达固定点 %d，剩余固定 %d / 随机 %d",
                  node, fixed_remain_, random_remain_);
    }
  }

  void on_recon_found()
  {
    random_remain_ = random_remain_ > 0 ? random_remain_ - 1 : 0;
    if (recon_found_ < cfg_.recon_target) {
      recon_found_++;
    }
    publish_status();
    RCLCPP_INFO(get_logger(), "找到侦查点，剩余固定 %d / 随机 %d（已找 %d/%d）",
                fixed_remain_, random_remain_, recon_found_, cfg_.recon_target);
  }

  // ═══════════════ 状态（原 main.cc 全局变量 → 节点私有成员） ═══════════════
  bool mission_started_ = false;
  bool mission_done_ = false;
  bool first_segment_sent_ = false;
  bool uturn_pending_ = false;
  bool awaiting_turn_finished_ = false;
  std::vector<StepCommand> segments_;
  size_t seg_index_ = 1;
  int current_node_ = 1;
  int pending_from_ = -1;
  int pending_arrival_ = -1;
  int fixed_remain_ = static_cast<int>(kFixedPoints.size());
  int random_remain_ = 0;   // 构造时按 cfg_.recon_target 初始化
  int recon_found_ = 0;
  bool recon_finished_ = false;
  std::set<int> fixed_done_;
  std::set<mission::Edge> searched_edges_;
  std::set<mission::Edge> tunnel_done_;
  std::set<std::pair<int, int>> blocked_edges_;

  // 规划器（声明顺序 = 初始化顺序：cfg_ → graph_ → planner_）
  mission::CostConfig cfg_;
  Graph graph_;
  mission::MissionPlanner planner_;

  rclcpp::Publisher<scoutcar_msgs::msg::PathCmd>::SharedPtr pub_path_;
  rclcpp::Publisher<scoutcar_msgs::msg::MissionStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_dev_;
  rclcpp::Subscription<scoutcar_msgs::msg::RxEvent>::SharedPtr sub_event_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_recon_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}
