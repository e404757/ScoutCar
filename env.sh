#!/usr/bin/env bash
# scoutcar 工作区环境加载脚本
#
# 背景：本机的 colcon 生成的 install/local_setup.bash 漏掉了 ament_prefix_path.sh
# 钩子（生成的命令里没有 AMENT_PREFIX_PATH 的导出），导致 ros2 CLI 找不到工作区包。
# 这里在 source 完 local_setup 之后手动补上包级前缀。
#
# 用法（在终端里）：
#   source /home/orangepi/CityScout/env.sh

# 同一终端重复 source 时不重复追加环境路径。
if [ "${SCOUTCAR_ENV_LOADED:-}" = "1" ]; then
  return 0 2>/dev/null || exit 0
fi
export SCOUTCAR_ENV_LOADED=1

source /home/orangepi/ros2_humble/install/setup.bash
source /home/orangepi/CityScout/install/local_setup.bash

# 图像传输用 CycloneDDS：默认的 FastDDS 2.6 在本机回环上传 921KB 相机大帧时
# 会出现"端点匹配但数据停滞"的间歇性断流（2026-08-30 排查 web 画面卡顿确认），
# 旧工程单进程无 DDS 不受影响。CycloneDDS 对大消息分片传输更稳。
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 大分片配置：921KB 帧按 ~65KB 分片，避免默认小分片产生海量报文拖垮传输
export CYCLONEDDS_URI=file:///home/orangepi/CityScout/config/cyclonedds.xml

for _p in /home/orangepi/CityScout/install/*/; do
  export AMENT_PREFIX_PATH="$_p:$AMENT_PREFIX_PATH"
done
unset _p
