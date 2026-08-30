#pragma once

// 串口通信模块：设备管理 + 协议帧收发 + 收发线程。
// 协议帧格式见 protocol.h（帧头 0xFF，第二位类型，帧尾 0xDD）。

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "scoutcar_control/protocol.h"

// ── 设备管理 ──
int uart_init(const char* device, int baud);
void uart_close(int fd);

// 自动查找可用的 USB 串口设备路径（CH340 等）。优先 /dev/serial/by-id（稳定标识）。
// 找到返回 0 并填 buf；未找到返回 -1。
int uart_find_device(char* buf, size_t buflen);

// ── 帧发送（底层，线程安全）──
void uart_send_frame(int fd, const uint8_t* buf, size_t len);

// ── 高层帧发送（协议打包 + 发送）──
void uart_send_debug_ready(int fd, pathplan::Status st, uint8_t fixed_remain, uint8_t random_remain); // FF 01 [AA/BB] [固定剩余] [随机剩余] DD
void uart_send_path_segment(int fd, const pathplan::PathPlanFrame& seg); // 路径段帧 FF 02 [起点][终点][动作] DD
void uart_send_deviation_frame(int fd, int16_t deviation);               // 中心偏差帧 FF 03 [符号][lo][hi] DD

// ── 接收线程：解析下位机帧 FF 01 00 [flag] 00 DD，回调上报事件 ──
typedef void (*uart_rx_callback_t)(pathplan::RxFlag flag, void* user);
void uart_start_rx_thread(int fd, uart_rx_callback_t cb, void* user);
void uart_stop_rx_thread(void);

// ── 偏差发送线程：周期发送中心偏差帧，启用后才发 ──
typedef int16_t (*uart_dev_source_t)(void* user);
void uart_start_deviation_sender(int fd, int interval_us, uart_dev_source_t get, void* user);
void uart_set_deviation_enabled(bool enabled);
void uart_stop_deviation_sender(void);

int uart_data_available(int fd);
int uart_read_byte(int fd);
