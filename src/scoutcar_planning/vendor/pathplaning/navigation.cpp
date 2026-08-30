#include "navigation.h"

#include <cmath>

namespace pathplan {

namespace {

// 将坐标差 (dcol, drow) 量化为四个方位之一（轴对齐边；对角边取主导轴）。
Heading quantize(double dcol, double drow) {
    if (std::abs(dcol) >= std::abs(drow)) {
        return dcol > 0 ? Heading::EAST : Heading::WEST;
    }
    return drow > 0 ? Heading::SOUTH : Heading::NORTH;
}

}  // namespace

Heading edgeHeading(int from, int to) {
    const auto& pos = nodePositions();
    const auto it_from = pos.find(from);
    const auto it_to = pos.find(to);
    if (it_from == pos.end() || it_to == pos.end()) return Heading::NORTH;  // 非法输入
    const double dcol = it_to->second.first - it_from->second.first;
    const double drow = it_to->second.second - it_from->second.second;
    return quantize(dcol, drow);
}

TurnAction turnAction(Heading current, Heading desired) {
    // 顺时针增序：+1=顺时针90°(右转)、+2=180°(掉头)、+3=逆时针90°(左转)
    const int d = (static_cast<int>(desired) - static_cast<int>(current)) & 3;
    switch (d) {
        case 0: return TurnAction::STRAIGHT;
        case 1: return TurnAction::RIGHT;   // 顺时针 = 右转
        case 2: return TurnAction::UTURN;
        case 3: return TurnAction::LEFT;    // 逆时针 = 左转
    }
    return TurnAction::STOP;  // 不可达
}

NavigationPlan planNavigation(const std::vector<int>& path, Heading initialHeading,
                              TurnAction finalAction) {
    NavigationPlan plan;
    plan.path = path;
    plan.finalAction = finalAction;

    if (path.empty()) return plan;

    const size_t n = path.size();
    plan.steps.reserve(n);

    if (n == 1) {
        // 起点即终点：原地停车
        plan.steps.push_back({path[0], -1, initialHeading, finalAction});
        return plan;
    }

    Heading h = initialHeading;
    for (size_t i = 0; i < n; ++i) {
        if (i + 1 < n) {
            const Heading desired = edgeHeading(path[i], path[i + 1]);
            StepCommand cmd;
            cmd.node = path[i];
            cmd.next = path[i + 1];
            cmd.heading = desired;             // 执行动作后的朝向
            cmd.action = turnAction(h, desired);
            plan.steps.push_back(cmd);
            h = desired;                       // 到达下一节点时的朝向
        } else {
            // 终点：不转向，action=finalAction，heading=到达时的朝向
            plan.steps.push_back({path[i], -1, h, finalAction});
        }
    }
    return plan;
}

NavigationPlan planNavigation(const std::vector<int>& path, TurnAction finalAction) {
    if (path.size() < 2) return planNavigation(path, Heading::NORTH, finalAction);
    return planNavigation(path, edgeHeading(path[0], path[1]), finalAction);
}

}  // namespace pathplan
