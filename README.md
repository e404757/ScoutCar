# CityScout ROS 2

CityScout 的 ROS 2 Humble 版本，运行于 Orange Pi 5 Pro（RK3588）。工程将原单体程序拆分为相机、感知、规划、串口控制和 bringup 包。

## 当前组件

- `scoutcar_camera`：USB 与 MIPI OV13855 图像发布
- `scoutcar_perception`：路面分割与道路中心偏差计算
- `scoutcar_planning`：任务与转向状态管理
- `scoutcar_control`：串口协议与偏差帧发送
- `scoutcar_msgs`：自定义 ROS 2 消息
- `scoutcar_bringup`：相机和其余小车节点的独立启动入口

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

## 启动与停止

相机服务 `cityscout-cameras.service` 只运行两路相机；业务服务 `cityscout-app.service` 运行感知、规划和串口控制。业务服务可单独重启，相机进程继续运行。同一组节点只能选择 systemd 或手动 launch 一种启动方式，避免重复占用相机设备。

### 首次部署服务

先在原来的终端按 `Ctrl+C` 退出旧的整车 launch，确认相机进程已经退出。然后执行：

```bash
cd /home/orangepi/CityScout
colcon build --symlink-install --packages-select scoutcar_bringup
sudo install -m 644 deploy/cityscout-cameras.service /etc/systemd/system/cityscout-cameras.service
sudo install -m 644 deploy/cityscout-app.service /etc/systemd/system/cityscout-app.service
sudo systemctl daemon-reload
sudo systemctl enable cityscout-cameras.service cityscout-app.service
sudo systemctl start cityscout-cameras.service cityscout-app.service
```

`--symlink-install` 是本项目的构建要求；`enable` 设置开机启动，`start` 立即启动。

### 日常控制服务

```bash
# 只重启感知、规划和串口控制，相机保持运行
sudo systemctl restart cityscout-app.service

# 停止业务服务和相机服务
sudo systemctl stop cityscout-app.service
sudo systemctl stop cityscout-cameras.service

# 查看状态；两行均为 inactive 表示都已停止
systemctl is-active cityscout-app.service cityscout-cameras.service

# 以后不再开机自动启动时执行；不会停止当前正在运行的服务
sudo systemctl disable cityscout-app.service cityscout-cameras.service
```

服务日志分别查看；`-f` 会持续显示新日志，按 `Ctrl+C` 退出查看：

```bash
journalctl -u cityscout-cameras.service -f
journalctl -u cityscout-app.service -f
```

### 手动 launch 并在终端看日志

先停止两个服务。如果已有旧的手动整车 launch，也要在其终端按 `Ctrl+C` 退出。服务的 `stop` 命令不会停止手动 launch 进程。

```bash
sudo systemctl stop cityscout-app.service
sudo systemctl stop cityscout-cameras.service
systemctl is-active cityscout-app.service cityscout-cameras.service
```

终端 1 启动相机，直接在该终端看相机日志：

```bash
cd /home/orangepi/CityScout
source deploy/env.sh
ros2 launch scoutcar_bringup cameras.launch.py
```

终端 2 启动业务节点，直接在该终端看业务日志：

```bash
cd /home/orangepi/CityScout
source deploy/env.sh
ros2 launch scoutcar_bringup cityscout.launch.py web:=true
```

两个终端各按 `Ctrl+C` 停止对应的 launch。只重启业务节点时，仅在终端 2 按 `Ctrl+C` 后重新运行其 launch 命令；终端 1 的相机保持运行。`web:=true` 会同时启动 Web 节点；省略它时默认不启动 Web 节点。

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

服务已启动时，固定路线直接使用配置文件参数。手动启动业务节点（相机需已运行）：

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


