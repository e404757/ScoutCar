# CityScout 协作约定

## 工程边界

- 当前工程：`/home/orangepi/CityScout`
- 旧工程：`/home/orangepi/cityscout`，只作迁移参考，不直接修改
- 构建前使用：`source /home/orangepi/CityScout/env.sh`
- 不提交 `build/`、`install/`、`log/`、抓图、录像或任何凭据
- 生产模型 `src/scoutcar_perception/models/V1.0.rknn` 随仓库管理；其他大型模型需单独评估

## 当前运行约定

- USB 话题 `/camera/usb/image_raw`：巡线与路面感知输入
- MIPI 话题 `/camera/mipi/image_raw`：OV13855，目标识别预留
- 感知节点当前订阅 USB 话题
- 完整上电服务：`cityscout-cameras.service`
- `dsh-web.service` 是按需调试工具；小车运行时保持停止

## 验证原则

- 修改 C++ 或 launch/config 后，构建受影响的 ROS 2 包
- 不在未经请求时启动完整串口控制链路
- MIPI 图像由 `rkaiq_3A.service` 提供 3A / IQ 支持
