#!/usr/bin/env bash
# 锁定 CPU/NPU/GPU/DDR 调频策略为 performance。
# 来自旧工程 run.sh 的实践（参考 f7b608f）：避免动态升降频造成推理耗时
# 与帧率抖动；排查 2026-08-30 感知推理卡死时确认新工程缺少该步骤。
# 需 root；小车每次上电后执行一次（可加入开机服务）。
set -u

if [[ "$(id -u)" -ne 0 ]]; then
    echo "[ERROR] 需要 root 运行：sudo $0" >&2
    exit 1
fi

set_performance_governor() {
    local governor_file="$1"
    if [[ -w "$governor_file" ]]; then
        printf '%s\n' performance > "$governor_file"
        printf '[OK] %s -> performance\n' "$governor_file"
    else
        printf '[SKIP] %s 不可写\n' "$governor_file"
    fi
}

for governor_file in /sys/devices/system/cpu/cpufreq/policy*/scaling_governor; do
    set_performance_governor "$governor_file"
done
for governor_file in \
    /sys/devices/platform/fdab0000.npu/devfreq/*/governor \
    /sys/devices/platform/fb000000.gpu/devfreq/*/governor \
    /sys/devices/platform/dmc/devfreq/*/governor; do
    set_performance_governor "$governor_file"
done
