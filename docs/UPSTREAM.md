# 迁移来源

本仓库是独立的 ROS 2 工程，不与旧工程共享 Git 历史。

| 项目 | 信息 |
| --- | --- |
| 旧工程仓库 | `https://github.com/e404757/seg.git` |
| 迁移基线提交 | `a755772` (`The END`) |
| 旧工程目录 | `/home/orangepi/cityscout` |
| 本工程目录 | `/home/orangepi/CityScout` |

旧工程用于查阅原始业务逻辑、协议和算法实现；新的功能和 ROS 2 改动仅提交到本仓库。

## 已迁移方向

- 相机采集与图像话题发布
- 路面分割、固定扫描线道路中心与偏差发布
- 任务状态、0xDD / 0xEE 转向控制状态
- 串口接收事件及偏差帧发送
- ROS 2 bringup 与上电自启动

## 模型与运行资产

当前生产模型 `src/scoutcar_perception/models/V1.0.rknn` 纳入版本控制。其他实验模型、相机抓图、录像、colcon 构建产物和日志不纳入版本控制。
