#pragma once

// 巡逻任务规划器(独立模块,不接入主程序)
//
// 策略(2026-08-22 重构,见 planning/action_plan.md):
//   - 侦查点未找齐:把「未搜索非隧道边 + 剩余隧道」作为需求边,用
//     CoveringCircuit(奇度点最小匹配 + 欧拉回路)求出覆盖它们的最短闭合回路
//     (中国邮路精确解);固定点由回路顺路覆盖,漏掉的留给收尾 DP 兜底。
//   - 侦查点找齐:对「剩余固定点 + 剩余隧道」做状态压缩 DP 精确收尾,再回家。
//   - 代价模型为转弯感知:(节点 × 朝向) 扩展状态图(80 状态)上全源最短路,
//     直行 0 / 90° 转弯 turn_penalty / 180° 掉头 u_turn_penalty(允许掉头)。
//   - 事件驱动:信息一变(找到侦查点 / 完成固定点 / 穿越隧道 / 障碍变更)
//     就重新调用 plan()。

#include "graph.h"

#include <set>
#include <utility>
#include <vector>

namespace mission {

using Edge = std::pair<int, int>;

// 规范化无向边(a < b)
Edge normEdge(int a, int b);

// 可配置代价参数(数值后续按小车实测标定)
struct CostConfig {
    double base           = 1.0;   // 普通边基础代价
    double tunnel_risk    = 0.0;   // 隧道附加风险(暂不特殊处理,默认 0)
    double turn_penalty   = 0.5;   // 90° 转弯附加代价(直行优先)
    double u_turn_penalty = 1.0;   // 180° 掉头附加代价(允许掉头,默认 ≈2×转弯)
    bool   want_search    = true;  // (保留字段,新算法不再使用)
    double search_tax     = 0.1;   // (保留字段,新算法不再使用)
    int    recon_target   = 8;     // 需要找到的侦查点数
    int    home           = 1;     // 家(终点)
};

// 任务状态(由调用方维护,状态变化后重新 plan)
struct MissionState {
    int current = 1;                // 当前位置
    std::vector<int> fixed_left;    // 剩余固定点(节点)
    std::vector<Edge> tunnel_left;  // 剩余隧道(未穿越)
    std::set<Edge> searched;        // 已搜索(已走过)的边
    int recon_found = 0;            // 已找到侦查点数
};

// 是否还需要找侦查点
inline bool needSearch(const MissionState& s, const CostConfig& c) {
    return s.recon_found < c.recon_target;
}

// 巡逻任务规划器
class MissionPlanner {
public:
    MissionPlanner(const Graph& g, const CostConfig& cfg);

    // 返回从 state.current 出发、覆盖全部剩余需求并回到 home 的节点序列。
    // 允许掉头(按 u_turn_penalty 计费);避障掉头仍由调用方在 0xCC 处理里单独下发。
    std::vector<int> plan(const MissionState& state);

    // 运行时更新代价配置(如从配置文件读取后;代价改变影响后续 plan 结果)
    void setConfig(const CostConfig& cfg) { cfg_ = cfg; }

    const std::vector<Edge>& tunnels() const { return tunnels_; }
    const std::set<Edge>& edges() const { return edges_; }  // 图中所有无向边

private:
    const Graph& g_;     // 只读引用,障碍通过 setBlocked 反映在图上
    CostConfig cfg_;
    std::vector<Edge> tunnels_;  // 隧道边 {6-7, 10-11, 14-15, 18-19}
    std::vector<int> nodes_;
    std::set<Edge> edges_;
};

}  // namespace mission
