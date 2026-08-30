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
cd /home/orangepi/CityScout
source env.sh
colcon build --symlink-install
```

## 启动

仅启动双相机：

```bash
ros2 launch scoutcar_bringup cameras.launch.py
```

启动完整链路：

```bash
ros2 launch scoutcar_bringup cityscout.launch.py
```



