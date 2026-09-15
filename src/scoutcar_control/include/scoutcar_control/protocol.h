#pragma once

#include <cstddef>
#include <cstdint>

namespace pathplan {

constexpr uint8_t kFrameHead = 0xFF;
constexpr uint8_t kFrameTail = 0xDD;


//程序状态
enum class Status : uint8_t {
    READY = 0xAA,  // 程序就绪，随时可开始
    ERROR = 0xBB,  // 程序出问题，无法通信
};
//侦察任务状态
enum class DetectStatus : uint8_t {
    START = 0xAA,  // 侦察任务开始
    END = 0xBB,    // 侦察任务结束
};
// 数据类型
enum class MsgType : uint8_t {
    DEBUG         = 0x01,  // 调试
    PATH_PLANNING = 0x02,  // 路径段
    DEVIATION     = 0x03,  // 中心偏差
    DETECT        = 0x04,  // 目标识别
};

// 下位机回传 flag（接收帧 `FF 01 00 [flag] 00 DD` 的 flag 字节）
enum class RxFlag : uint8_t {
    START    = 0xAA,  // 启动小车，可发第一段路径
    CAM_AHEAD = 0xBB,  // 摄像头朝前
    CAM_TURNED = 0xCC,  // 摄像头转向两侧
    ARRIVED  = 0xDD,  // 小车到达目标点，开始转向
    TURN_FINISHED = 0xEE,  // 小车转向结束，恢复循迹
};

// 转向动作（路径段帧的 action 字节）
enum class TurnAction : uint8_t {
    STOP     = 0x00,  // 停车
    STRAIGHT = 0x01,  // 直行
    LEFT     = 0x02,  // 左转
    RIGHT    = 0x03,  // 右转
    UTURN    = 0x04,  // 掉头
};

// 路径段消息：起点、终点、到达终点后执行的转向动作
struct PathPlanFrame {
    uint8_t     start;        // 起点格点
    uint8_t     goal;         // 终点格点
    TurnAction  finalAction;  // 到达终点后执行的转向动作
};

constexpr size_t kMaxFrameLen = 16;  // 预留足够空间

inline const char* actionName(TurnAction a) {
    switch (a) {
        case TurnAction::STOP:     return "停车";
        case TurnAction::STRAIGHT: return "直行";
        case TurnAction::LEFT:     return "左转";
        case TurnAction::RIGHT:    return "右转";
        case TurnAction::UTURN:    return "掉头";
    }
    return "未知";
}

// 通用帧打包：buf 需 ≥ len+3。布局 [0xFF][type][payload...][0xDD]。返回帧总长。
inline size_t packFrame(uint8_t* buf, MsgType type, const uint8_t* payload, size_t payloadLen) {
    buf[0] = kFrameHead;
    buf[1] = static_cast<uint8_t>(type);
    for (size_t i = 0; i < payloadLen; ++i) buf[2 + i] = payload[i];
    buf[2 + payloadLen] = kFrameTail;
    return 2 + payloadLen + 1;
}

// 通用解包：校验帧头/帧尾，取出类型与负载。返回是否合法。
inline bool unpackFrame(const uint8_t* buf, size_t len, MsgType& type,
                        uint8_t* payload, size_t& payloadLen) {
    if (len < 3) return false;
    if (buf[0] != kFrameHead) return false;
    if (buf[len - 1] != kFrameTail) return false;
    type = static_cast<MsgType>(buf[1]);
    payloadLen = len - 3;
    for (size_t i = 0; i < payloadLen; ++i) payload[i] = buf[2 + i];
    return true;
}

// 路径段帧打包（FF 02 [start][goal][action] DD，共 6 字节）
inline size_t packPathPlan(uint8_t* buf, const PathPlanFrame& msg) {
    const uint8_t payload[3] = {
        msg.start,
        msg.goal,
        static_cast<uint8_t>(msg.finalAction),
    };
    return packFrame(buf, MsgType::PATH_PLANNING, payload, sizeof(payload));
}

// 路径段帧解包
inline bool unpackPathPlan(const uint8_t* buf, size_t len, PathPlanFrame& msg) {
    MsgType type;
    uint8_t payload[3];
    size_t payloadLen = 0;
    if (!unpackFrame(buf, len, type, payload, payloadLen)) return false;
    if (type != MsgType::PATH_PLANNING || payloadLen != 3) return false;
    msg.start = payload[0];
    msg.goal = payload[1];
    msg.finalAction = static_cast<TurnAction>(payload[2]);
    return true;
}

// 就绪/调试帧打包（FF 01 [AA/BB] [固定剩余] [随机剩余] DD，共 6 字节）
// 每到一个任务点/侦查点，调用方更新剩余数量后重发此帧。
inline size_t packDebugReady(uint8_t* buf, Status st,
                             uint8_t fixed_remain, uint8_t random_remain) {
    const uint8_t payload[3] = {
        static_cast<uint8_t>(st),
        fixed_remain,
        random_remain,
    };
    return packFrame(buf, MsgType::DEBUG, payload, sizeof(payload));
}

// 中心偏差帧打包（FF 03 [AA负/BB正] [lo][hi] DD，共 6 字节）
inline size_t packDeviation(uint8_t* buf, int16_t deviation) {
    const uint8_t sign = deviation < 0 ? 0xAA : 0xBB;
    const uint16_t abs_val = static_cast<uint16_t>(deviation < 0 ? -deviation : deviation);
    const uint8_t payload[3] = {
        sign,
        static_cast<uint8_t>(abs_val & 0xFF),
        static_cast<uint8_t>((abs_val >> 8) & 0xFF),
    };
    return packFrame(buf, MsgType::DEVIATION, payload, sizeof(payload));
}

inline size_t packDetectTask(uint8_t* buf,DetectStatus st,uint8_t left_result,uint8_t right_result) {
    const uint8_t payload[3] = {
        static_cast<uint8_t>(st),
        left_result,
        right_result,
    };
    return packFrame(buf, MsgType::DETECT, payload, sizeof(payload));
}

}  // namespace pathplan
