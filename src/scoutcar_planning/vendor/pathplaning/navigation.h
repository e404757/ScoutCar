#pragma once

#include "map.h"
#include "protocol.h"

#include <vector>

namespace pathplan {

// 小车方位（地图坐标：col 向右、row 向下）。顺时针依次增大。
// 默认约定：地图上 逆时针=左转、顺时针=右转（标准）。
enum class Heading : int {
    EAST  = 0,  // 朝右（列增大方向）
    SOUTH = 1,  // 朝下（行增大方向）
    WEST  = 2,  // 朝左（列减小方向）
    NORTH = 3,  // 朝上（行减小方向）
};

// 边的朝向：由 from→to 的坐标差量化（本图边均为轴对齐）。
// 若 from 或 to 不存在，返回 NORTH 并视为非法（调用前需保证节点有效）。
Heading edgeHeading(int from, int to);

// 由当前朝向到目标朝向需要的转向动作。
// 标准约定：顺时针 90°=右转、逆时针 90°=左转、180°=掉头、0°=直行。
// 若装车后发现左右相反，交换 RIGHT/LEFT 两个分支即可。
TurnAction turnAction(Heading current, Heading desired);

// 单个格点上的执行指令
struct StepCommand {
    int         node;     // 执行该动作的格点
    int         next;     // 执行后前往的目标格点；-1 表示终点停车
    Heading     heading;  // 执行完该动作后的朝向（终点停车时为到达时的朝向）
    TurnAction  action;   // 在该格点执行的动作
};

// 整条路径的导航结果
struct NavigationPlan {
    std::vector<int>        path;          // 节点序列（与输入一致）
    std::vector<StepCommand> steps;        // steps.size() == path.size()
    TurnAction              finalAction;   // 终点动作
};

// 对路径逐点计算朝向与动作。initialHeading 为起点处小车朝向；
// 后续节点到达朝向 = 上一条边的方向。终点节点动作 = finalAction（默认停车）。
NavigationPlan planNavigation(const std::vector<int>& path, Heading initialHeading,
                              TurnAction finalAction = TurnAction::STOP);

// 重载：初始朝向默认"面向首条边方向"，使起点动作为直行。
NavigationPlan planNavigation(const std::vector<int>& path,
                              TurnAction finalAction = TurnAction::STOP);

// 调试打印用：方位文本
inline const char* headingName(Heading h) {
    switch (h) {
        case Heading::EAST:  return "东";
        case Heading::SOUTH: return "南";
        case Heading::WEST:  return "西";
        case Heading::NORTH: return "北";
    }
    return "?";
}

}  // namespace pathplan
