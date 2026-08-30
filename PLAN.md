# scoutcar 工程：ROS2 移植计划与规范

> 源工程：旧版 CityScout（RK3588 侦查小车，单进程 CMake superbuild）
> 目标：`/home/orangepi/CityScout`（ROS2 colcon 工作区）
> 策略：**算法库原样复用，只把"进程内耦合"换成"话题通信"** —— 不是重写，是重新接线。

---

## 一、目标架构

```
CityScout/                       # colcon 工作区根
├── src/
│   ├── scoutcar_msgs/           # ✅ 消息包（v1：RxEvent / PathCmd）
│   ├── scoutcar_control/        # ✅ 驱动层：serial_node（真接线：PathCmd→FF02，串口帧→RxEvent）
│   ├── scoutcar_camera/         # ✅ 驱动层：camera_node（vendor 原样搬 + 发 /camera/image_raw）
│   ├── scoutcar_perception/     # ✅ 感知层：perception_node（推理已接入：image→seg_mask）
│   ├── scoutcar_planning/       # ✅ 应用层：mission_node（事件驱动状态机；vendor 已就位，待 ROS2 编译）
│   ├── scoutcar_web/            # ✅ 应用层：web_node（网页推流 + 录像回放/导出补标）
│   └── scoutcar_bringup/        # ⏳ launch + 参数
├── config/  build/ install/ log/
└── PLAN.md
```

分层与依赖方向：**应用层 → 感知层 → 驱动层**（高层通过话题依赖低层，驱动层只依赖消息定义）。

## 二、接口定稿（工程规范，改前先讨论）

### 消息（scoutcar_msgs，v1）

| 消息 | 内容 | 常量（= 协议字节） |
|---|---|---|
| `RxEvent` | 下位机回传事件 | START=0xAA, ARRIVED=0xDD, OBSTACLE=0xCC |
| `PathCmd` | 单跳路径指令（到达 goal 后执行 action） | STOP/STRAIGHT/LEFT/RIGHT/UTURN = 0..4 |
| `MissionStatus` | 任务状态（调试帧扩展） | READY=0xAA, ERROR=0xBB |

### 话题命名规范（定稿）

| 话题 | 消息类型 | 方向 | 语义 |
|---|---|---|---|
| `/mcu/rx_event` | RxEvent | serial_node → mission_node | 下位机接收方向的事件 |
| `/mission/path_cmd` | PathCmd | mission_node → serial_node | 路径段命令 |
| `/mission/status` | MissionStatus | mission_node → serial_node/web | 任务状态（serial 发调试帧 FF01） |
| `/mission/deviation_enable` | std_msgs/Bool | mission_node → serial_node | 偏差帧开关（掉头/重规划期间关闭） |
| `/mission/recon_found` | std_msgs/Bool | detection_node(人工) → mission_node | 侦查点发现（v3 接入） |
| `/camera/image_raw` | sensor_msgs/Image | camera_node → perception/web | 相机原始帧（RGB8） |
| `/perception/seg_mask` | sensor_msgs/Image | perception_node → web | 分割掩膜（mono8：0背景/1路面/2挡板） |
| `/perception/road_boundary` | RoadBoundary（v2 待定义） | perception_node → serial_node/web | 道路边界跟踪结果 |
| `/perception/detections` | vision_msgs/Detection2DArray（v3 预留） | detection_node → mission_node/web | 目标检测结果（独立 detect 模型） |

**话题名原则**：① 命名空间 = 子系统（`/mcu/` `/mission/` `/perception/`）；② 角色词表方向（`_cmd` 命令、`rx_` 接收、`_status` 状态）；③ 不带节点名；④ 与消息类型名解耦。
**话题名写法**：现阶段话题少，节点内直接写字符串字面量；跨节点拼写一致性靠运行时核对（`ros2 topic list` / `rqt_graph`）。若以后跨节点对名频繁出错，再升级为共享头 `topics.hpp`（两个节点 include 同一常量，编译期消灭拼写错误）。

## 三、分阶段进度

- [x] 阶段 0.5 消息包 `scoutcar_msgs`（RxEvent / PathCmd）
- [x] 阶段 4a `scoutcar_control`：serial_node 真接线（vendor 搬入 protocol/serial；PathCmd→FF02；串口帧→RxEvent；PT 请求忽略；配套订阅 /mission/status 调试帧 + /mission/deviation_enable 偏差开关）
- [x] 阶段 4b `scoutcar_camera`：camera_node（vendor 搬入 camera.cc；参数 camera_path/fps；发 /camera/image_raw，best_effort）
- [x] 阶段 4c `scoutcar_perception`：perception_node 推理接入（rgb8 → 手动 memcpy 进 image_buffer_t → inference_yolov5_seg_model_cpu → seg_mask；**不依赖 cv_bridge**，因构建里没有 vision_opencv）
- [x] 阶段 4c v2 定义 RoadBoundary.msg + road_tracker 接入（/perception/road_boundary 喂偏差帧）
- [ ] 阶段 4c v3 detection_node（独立 detect 模型 → /perception/detections，喂 mission 侦查点钩子）
- [x] 阶段 4d `scoutcar_planning`：mission_node 代码完成 + **编译通过 + 无硬件冒烟测试通过**（START→规划34段→发1→3右转；ARRIVED×3→推进+固定点计数11/8；OBSTACLE→阻断6→5+重规划31段+掉头）
- [x] 阶段 4d 前置：planning vendor 10 文件已复制（pathplaning 8 + mission 2）
- [x] 阶段 5a `scoutcar_web`：**web_node 网页推流 + 录像回放/导出补标**（编译通过；原工程 --web/--record 需求改：**录音只做"采集训练数据"的单路，用"原始+叠加双文件同步录 + 网页回放找弱帧 + 导出补标"替代原掩膜诊断录像/遥测**）
- [ ] 阶段 5b 参数化 + launch + foxglove（可选）
- [ ] 阶段 6（进阶）mission → Action；serial → lifecycle

## 四、serial_node 骨架要点（移植心法）

1. **executor 只管 ROS 事件，串口接收线程自管**（独立 std::thread，`publish()` 线程安全）；
2. 构造函数顺序：参数 → 接口 → 开串口 → 起线程（先建发布者再起线程）；
3. `on_rx_flag` 回调 → **发布 RxEvent**，决策移去 mission_node（serial 只翻译不做决策）；
4. 析构顺序不能反：**先 join 接收线程，再关 fd**；
5. 偏差发送简化：RoadBoundary 消息本身周期到达，回调即发，不再需要 20ms 独立线程。

## 四.5 感知层设计要点（v1）

1. **vendor 自持**：算法库原样搬入包内 `vendor/`（yolov5_seg + common：image_utils/file_utils）；utils 不进 3rdparty（3rdparty 约定放第三方编译产物，自己的代码会改）；
2. **3rdparty 引用原路径**：RKNN/RGA/JPEG/STB/allocator/timer 指向 `旧版 CityScout/3rdparty`（CMake 缓存变量 `SCOUTCAR_3RDPARTY_DIR` 可 `-D` 覆盖）；
3. **CPU 预处理**：推理走 `inference_yolov5_seg_model_cpu`（输入是 cv::Mat 普通内存，避免 RGA importbuffer 崩溃）；
4. **目标识别是独立 detect 模型**（v3，不是 seg 模型出框）：`detection_node` 占位，话题 `/perception/detections` 已定，输出喂 mission 侦查点钩子；
5. **掩膜用标准消息** `sensor_msgs/Image`（mono8），不自定义。

## 五、构建与验证（已完成第一轮）

```bash
cd /home/orangepi/CityScout
source /home/orangepi/ros2_humble/install/setup.bash
colcon build --packages-select scoutcar_msgs scoutcar_control scoutcar_camera scoutcar_perception scoutcar_planning
```

- ✅ 5 个包全部编译通过（2026 已验证）
- ⚠️ **本机环境坑**：colcon 生成的 `install/setup.bash` 漏了 ament_prefix_path 钩子，`ros2` 找不到工作区包。**加载环境必须用 `env.sh`**（已建，内部 source ros2_humble + local_setup + 手动补 AMENT_PREFIX_PATH）：
  ```bash
  source /home/orangepi/CityScout/env.sh
  ```

### mission_node 无硬件冒烟测试（已验证）

```bash
ros2 run scoutcar_planning mission_node &     # 终端1
ros2 topic pub /mcu/rx_event scoutcar_msgs/msg/RxEvent "{event: 170}" --once   # START=0xAA
ros2 topic pub /mcu/rx_event scoutcar_msgs/msg/RxEvent "{event: 221}" --once   # ARRIVED=0xDD
ros2 topic pub /mcu/rx_event scoutcar_msgs/msg/RxEvent "{event: 204}" --once   # OBSTACLE=0xCC
```

## 六、scoutcar_web 设计要点（web_node，2026-08-28 确认）

> 需求（用户确认）：网页推流是刚需；录像要采集 YOLO 训练数据 —— 用一个按钮起停，
> **不做**重新跑推理、不做掩膜诊断录像、不做遥测 CSV；而是在网页**回放带掩膜的叠加画面**，
> 肉眼找"模型没分割好"的弱帧，**导出该帧**补标。

### 数据面（web_node 订阅，其他节点零改动）

| 话题 | 用途 |
|---|---|
| `/camera/*/image_raw` | 原始帧（rgb8，best_effort）—— 叠加 + 原始录像帧源 |
| `/perception/seg_mask` | 掩膜（mono8 0/1/2）—— 叠加 |
| `/perception/road_boundary` | 道路边界（扫描线/偏差）—— 叠加 |
| `/mission/status` | 任务状态 —— 叠加文字 |

### 功能

1. **实时推流**：叠加画面 MJPEG（默认 8080）。叠加 = 掩膜半透明(路面绿/挡板红 alpha0.5)
   + 边界扫描线/十字 + "Dev:+N" + 任务状态文字 + FPS。用 OpenCV `cv::line/putText` 绘制，
   **不再依赖 image_drawing/font.h**（避免引入感知包 vendor）。
2. **录像（双文件同步录）**：一个按钮起停；每段 `record_<ts>.avi`（原始帧）+ `record_<ts>_mask.avi`（叠加帧），
   同一回调里逐帧写入 → 帧号对齐。异步写线程 + 环形缓冲（编码跟不上丢帧不阻塞，见 video_writer）。
3. **回放**：列出片段 → 完整播放器（播放/暂停/进度条拖动/逐帧）。服务端按帧号出 JPEG
   （`/frame?file=&idx=&kind=mask`），端上无进度条精度损失。逐帧读走缓存，跨跳才 seek（MJPEG avi seek 快）。
4. **导出补标**：暂停在某帧 → `POST /api/export?file=&idx=&kind=raw` → 从**原始文件**抽该帧存 JPEG
   到 `label_dir`（默认 `~/cityscout/label_export`），文件名 `record_<ts>_<idx>_<时刻>_<kind>.jpg`，
   直接丢进 labelImg 补标。kind 可切 mask（存叠加帧当"待修正掩膜"参考）。

### 关键实现点

- **颜色通道**：消息是 rgb8（R,G,B 字节序）。web_node 里**先把原始帧转成 BGR** 再叠加
  （用 BGR 色板，否则红/蓝会写反——这是踩过的坑），推流直接 `imencode`；写叠加录像时
  再转回 RGB 交给 `AsyncAviWriter`（它内部再做 RGB2BGR 写 VideoWriter）。
- **掩膜对帧**：web_node 在图像回调里取"最新掩膜快照"（互斥保护 shared_ptr），按帧时间戳近似对齐，
  两者都 ~30fps 时误差≤1 帧，肉眼无感。
- **线程模型**：ROS executor（图像回调=主循环）+ httplib 独立线程（API/回放解码/导出）+
  每录像文件一个写线程。网页按钮经原子槽 → 10Hz 定时器消费 → 创建/封存片段。
- **路径安全**：`resolveSegment` 只接受 `record_YYYYmmdd_HHMMSS` 形态的 base，防路径穿越。
- **回放帧缓存**：单槽 `VideoCapture`（一次只服务一个播放器），连续读不 seek、跨跳才 seek。

### 运行

```bash
# 单独跑 web（不依赖整车主 launch）
ros2 launch scoutcar_web web.launch.py
# 浏览器打开 http://<本机IP>:8080
```

### 参数（config/web.yaml）

| 参数 | 默认 | 说明 |
|---|---|---|
| port | 8080 | HTTP 端口 |
| image_topic | /camera/usb/image_raw | 原始帧话题（默认与感知一致：巡线/路面感知走 USB） |
| seg_topic / boundary_topic / status_topic | /perception/... | 叠加数据话题 |
| record_dir | /home/orangepi/CityScout/data/record | 录像保存目录 |
| record_fps / record_enable | 30 / true | 录像帧率与总开关 |
| label_dir | /home/orangepi/CityScout/data/label_export | 补标导出目录 |
| jpg_quality | 85 | 推流/导出 JPEG 质量 |
