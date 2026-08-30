#include "scoutcar_control/serial.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdint.h>
#include <errno.h>
#include <termios.h>

#include <dirent.h>
#include <string.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

// ── 串口写互斥（发送线程/主循环并发写时保证帧不交错）──
static std::mutex g_uart_mutex;

// ── 接收线程状态 ──
static std::atomic<bool> g_rx_running{false};
static int g_rx_fd = -1;
static uart_rx_callback_t g_rx_callback = NULL;
static void* g_rx_user = NULL;
static std::thread g_rx_thread;

// ── 偏差发送线程状态 ──
static std::atomic<bool> g_dev_running{false};
static std::atomic<bool> g_dev_enabled{false};
static int g_dev_fd = -1;
static int g_dev_interval_us = 20000;
static uart_dev_source_t g_dev_get = NULL;
static void* g_dev_user = NULL;
static std::thread g_dev_thread;

// ═══════════════ 设备管理 ═══════════════

int uart_init(const char* device, int baud)
{
    // wiringPiSetup 只需调用一次，用静态变量保证
    static int wp_setup_done = 0;
    if (!wp_setup_done) {
        if (wiringPiSetup() == -1) {
            printf("uart: wiringPiSetup 失败\n");
            return -1;
        }
        wp_setup_done = 1;
    }

    int fd = -1;
    int max_retry = 30;  // 最多等 30 秒
    for (int retry = 0; retry < max_retry; retry++) {
        fd = serialOpen(device, baud);
        if (fd >= 0) break;
        printf("uart: 等待 %s ... (%d/%d)\n", device, retry + 1, max_retry);
        sleep(1);
    }

    if (fd < 0)
        printf("uart: 无法打开 %s（超时 %d 秒）\n", device, max_retry);
    else
        printf("uart: %s @ %d bps 就绪\n", device, baud);
    return fd;
}

void uart_close(int fd)
{
    if (fd >= 0) {
        serialClose(fd);
        printf("uart: 已关闭\n");
    }
}

// 自动查找可用 USB 串口设备路径（CH340 等）。USB 设备号可能变（ttyUSB0→1），
// 优先用 /dev/serial/by-id 的稳定软链接，回退扫描 /dev/ttyUSB*。
int uart_find_device(char* buf, size_t buflen)
{
    // 1) 优先稳定 by-id（USB 序列号，设备号变也不变）
    DIR* d = opendir("/dev/serial/by-id");
    if (d != NULL) {
        struct dirent* e;
        while ((e = readdir(d)) != NULL) {
            if (e->d_name[0] == '.') continue;
            snprintf(buf, buflen, "/dev/serial/by-id/%s", e->d_name);
            closedir(d);
            return 0;
        }
        closedir(d);
    }
    // 2) 回退扫描 /dev/ttyUSB*
    d = opendir("/dev");
    if (d != NULL) {
        struct dirent* e;
        while ((e = readdir(d)) != NULL) {
            if (strncmp(e->d_name, "ttyUSB", 6) == 0) {
                snprintf(buf, buflen, "/dev/%s", e->d_name);
                closedir(d);
                return 0;
            }
        }
        closedir(d);
    }
    return -1;
}

// ═══════════════ 帧发送 ═══════════════

void uart_send_frame(int fd, const uint8_t* buf, size_t len)
{
    if (fd < 0 || buf == NULL || len == 0) return;
    std::lock_guard<std::mutex> lk(g_uart_mutex);

    // wiringPi 的 serialFlush() 实际调用 tcflush(fd, TCIOFLUSH)，会把尚未
    // 处理的接收数据也直接丢掉。偏差帧以 50 Hz 发送时，这会频繁截断下位机
    // 的回传帧。这里改为完整 write，并用 tcdrain 等待数据真正发出。
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = write(fd, buf + sent, len - sent);
        if (n > 0) {
            sent += (size_t)n;
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        perror("uart: write");
        return;
    }
    if (tcdrain(fd) < 0)
        perror("uart: tcdrain");
}

void uart_send_debug_ready(int fd, pathplan::Status st, uint8_t fixed_remain, uint8_t random_remain)
{
    uint8_t buf[pathplan::kMaxFrameLen];
    size_t len = pathplan::packDebugReady(buf, st, fixed_remain, random_remain);
    uart_send_frame(fd, buf, len);
}

void uart_send_path_segment(int fd, const pathplan::PathPlanFrame& seg)
{
    uint8_t buf[pathplan::kMaxFrameLen];
    size_t len = pathplan::packPathPlan(buf, seg);
    uart_send_frame(fd, buf, len);
}

void uart_send_deviation_frame(int fd, int16_t deviation)
{
    uint8_t buf[pathplan::kMaxFrameLen];
    size_t len = pathplan::packDeviation(buf, deviation);
    uart_send_frame(fd, buf, len);
}

// ═══════════════ 接收线程 ═══════════════

// 解析下位机帧: FF 01 00 [flag] 00 DD
// flag=0xAA 启动 / 0xDD 到达并开始转向 / 0xEE 转向结束
static void rx_loop()
{
    enum State { WAIT_FF, WAIT_01, WAIT_00_1, READ_FLAG, WAIT_00_2, WAIT_DD };
    State state = WAIT_FF;
    pathplan::RxFlag pending_flag = pathplan::RxFlag::START;

    // 失配字节如果正好是 0xFF，就把它保留为下一帧帧头，避免连续帧中
    // 前一帧损坏时连下一帧也一起丢掉。
    auto reset_with_byte = [&state](int byte) {
        state = (byte == pathplan::kFrameHead) ? WAIT_01 : WAIT_FF;
    };

    while (g_rx_running.load()) {
        int avail = uart_data_available(g_rx_fd);
        if (avail > 0) {
            // 一次唤醒后尽量排空当前缓冲，避免逐字节 sleep 增加积压。
            while (avail-- > 0 && g_rx_running.load()) {
                int byte = uart_read_byte(g_rx_fd);
                if (byte < 0) break;

                switch (state) {
                    case WAIT_FF:
                        if (byte == 0xFF) state = WAIT_01;
                        break;
                    case WAIT_01:
                        if (byte == 0x01) state = WAIT_00_1;
                        else reset_with_byte(byte);
                        break;
                    case WAIT_00_1:
                        if (byte == 0x00) state = READ_FLAG;
                        else reset_with_byte(byte);
                        break;
                    case READ_FLAG:
                        if (byte == (int)pathplan::RxFlag::PT_80 ||
                            byte == (int)pathplan::RxFlag::START ||
                            byte == (int)pathplan::RxFlag::ARRIVED ||
                            byte == (int)pathplan::RxFlag::TURN_FINISHED ||
                            byte == (int)pathplan::RxFlag::OBSTACLE ||
                            byte == (int)pathplan::RxFlag::PT_40 ||
                            byte == (int)pathplan::RxFlag::STRAIGHT) {
                            pending_flag = static_cast<pathplan::RxFlag>(byte);
                            state = WAIT_00_2;
                        } else {
                            reset_with_byte(byte);
                        }
                        break;
                    case WAIT_00_2:
                        if (byte == 0x00) state = WAIT_DD;
                        else reset_with_byte(byte);
                        break;
                    case WAIT_DD:
                        if (byte == pathplan::kFrameTail) {
                            // 只有完整验证 6 字节帧后才通知业务层。
                            if (g_rx_callback)
                                g_rx_callback(pending_flag, g_rx_user);
                            state = WAIT_FF;
                        } else {
                            reset_with_byte(byte);
                        }
                        break;
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void uart_start_rx_thread(int fd, uart_rx_callback_t cb, void* user)
{
    uart_stop_rx_thread();  // 防止重复启动
    g_rx_fd = fd;
    g_rx_callback = cb;
    g_rx_user = user;
    g_rx_running = true;
    g_rx_thread = std::thread(rx_loop);
}

void uart_stop_rx_thread(void)
{
    g_rx_running = false;
    if (g_rx_thread.joinable()) g_rx_thread.join();
}

// ═══════════════ 偏差发送线程 ═══════════════

static void deviation_loop()
{
    while (g_dev_running.load()) {
        if (g_dev_enabled.load() && g_dev_get)
            uart_send_deviation_frame(g_dev_fd, g_dev_get(g_dev_user));
        std::this_thread::sleep_for(std::chrono::microseconds(g_dev_interval_us));
    }
}

void uart_start_deviation_sender(int fd, int interval_us, uart_dev_source_t get, void* user)
{
    uart_stop_deviation_sender();  // 防止重复启动
    g_dev_fd = fd;
    g_dev_interval_us = interval_us;
    g_dev_get = get;
    g_dev_user = user;
    g_dev_enabled = false;         // 任务启动后才开启
    g_dev_running = true;
    g_dev_thread = std::thread(deviation_loop);
}

void uart_set_deviation_enabled(bool enabled)
{
    g_dev_enabled.store(enabled);
}

void uart_stop_deviation_sender(void)
{
    g_dev_running = false;
    if (g_dev_thread.joinable()) g_dev_thread.join();
}

int uart_data_available(int fd)
{
    if (fd < 0) return 0;
    return serialDataAvail(fd);
}

int uart_read_byte(int fd)
{
    if (fd < 0) return -1;
    return serialGetchar(fd);
}
