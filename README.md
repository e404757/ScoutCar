# CityScout ROS 2

CityScout 的 ROS 2 Humble 版本，运行于 Orange Pi 5 Pro（RK3588）。工程将原单体程序拆分为相机、感知、规划、串口控制和 bringup 包。

## 当前组件

- `scoutcar_camera`：USB 与 MIPI OV13855 图像发布
- `scoutcar_perception`：路面分割与道路中心偏差计算
- `scoutcar_planning`：任务与转向状态管理
- `scoutcar_control`：串口协议与偏差帧发送
- `scoutcar_msgs`：自定义 ROS 2 消息
- `scoutcar_bringup`：双相机与完整小车链路启动

## 相机分工

| 设备 | 话题 | 当前用途 |
| --- | --- | --- |
| USB 摄像头 | `/camera/usb/image_raw` | 巡线与路面感知输入 |
| MIPI OV13855 | `/camera/mipi/image_raw` | 目标识别预留 |

感知节点当前订阅 USB 图像话题。

## 构建

```bash
# 进入工程目录
cd /home/orangepi/CityScout

# 加载 ROS 2、CityScout 工作空间和 CycloneDDS 环境
# 必须使用 source；直接执行 ./deploy/env.sh 不会修改当前终端环境
source deploy/env.sh

# 将 CPU、NPU、GPU 和 DDR 设置为 performance 调频策略
sudo /home/orangepi/CityScout/deploy/set_performance.sh

# 构建全部包
colcon build --symlink-install

colcon build --symlink-install --packages-select scoutcar_msgs
colcon build --symlink-install --packages-select scoutcar_camera
colcon build --symlink-install --packages-select scoutcar_control
colcon build --symlink-install --packages-select scoutcar_perception
colcon build --symlink-install --packages-select scoutcar_planning
colcon build --symlink-install --packages-select scoutcar_web
colcon build --symlink-install --packages-select scoutcar_bringup
```

## 启动
```bash
cd /home/orangepi/CityScout
source deploy/env.sh
systemctl stop cityscout-cameras.service
ros2 launch scoutcar_bringup cityscout.launch.py web:=true

```

## 固定路线模式

固定路线由 `src/scoutcar_bringup/config/cityscout.yaml` 中的两个参数控制：

```yaml
mission_node:
  ros__parameters:
    route_mode: "fixed"
    route_nodes: [1, 3, 2, 3, 1]
```

- `route_mode: "fixed"`：按照 `route_nodes` 执行固定路线。
- `route_mode: "auto"`：使用原有任务规划器自动生成路线，此时忽略 `route_nodes`。
- `route_nodes` 只填写依次经过的节点；相邻节点必须在地图上直接连通。
- 到达动作由相邻三点自动计算，最后一个节点自动停车。

使用配置文件中的固定路线启动整车：

```bash
cd /home/orangepi/CityScout
source deploy/env.sh
ros2 launch scoutcar_bringup cityscout.launch.py web:=true
```

只启动 mission 节点并临时传入一条固定路线：

```bash
cd /home/orangepi/CityScout
source deploy/env.sh
ros2 run scoutcar_planning mission_node --ros-args \
  -p route_mode:=fixed \
  -p 'route_nodes:=[1,3,2,3,1]'
```

命令行中的 `route_nodes` 只对本次启动有效，不会改写 YAML 配置文件。

sudo systemctl restart rkaiq_3A.service
pgrep -a -f 'codex|Codex|chatgpt|ChatGPT'
kill -9 12345


