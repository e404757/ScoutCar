# ROS 2 CMakeLists

`CMakeLists.txt` 告诉构建工具：如何编译、依赖谁、安装什么。

---

## 一个节点包的典型构建顺序

```text
① 声明 CMake 版本和包名
② 查找需要的、已经安装好的包或库
③ 将源文件编译成可执行程序或库
④ 设置头文件路径
⑤ 声明 ROS 2 依赖
⑥ 链接普通库或自建库
⑦ 安装程序和资源
⑧ 结束当前 ROS 2 包
```

```text
ROS 2 包：一个工程单位，例如 scoutcar_camera。
程序：可以直接启动，例如 camera_node。
库：可被程序复用，不能直接启动，例如 yolov5_seg。
```

一个 ROS 2 包可以同时包含程序、库、launch 文件和配置文件。

## 1. 声明 CMake 版本和包名

```cmake
cmake_minimum_required(VERSION 3.8)
project(工程名)
```

- `cmake_minimum_required`：要求最低 CMake 版本。
- `project`：声明当前工程名称；在 ROS 2 中通常应与 `package.xml` 中的包名一致。

## 2. 查找需要的、已经安装好的包或库

```cmake
find_package(依赖名 REQUIRED)
```
- `REQUIRED`：找不到这个依赖就停止构建并报错。
- `find_package` 只是“找到依赖”；后面还要让具体程序使用它。

本工程中常见的依赖：

```text
构建工具：
- ament_cmake：ROS 2 的 CMake 构建支持；基本每个 ROS 2 CMake 包需要。

ROS 2 C++ / 功能包：
- rclcpp：ROS 2 C++ 节点库；创建 Node、发布、订阅时使用。
- ament_index_cpp：运行时查找某个 ROS 2 包的 share 目录。
- sensor_msgs：传感器消息，例如 sensor_msgs/msg/Image。
- std_msgs：基础消息，例如 std_msgs/msg/Bool。
- scoutcar_msgs：本工程的自定义消息，例如 RoadBoundary。

消息生成：
- rosidl_default_generators：把 .msg 生成 C++ 等语言可用的消息代码。

普通 C/C++ 库：
- OpenCV：图像、相机与视觉处理。
- Threads：CMake 的线程库接口；工程中以 Threads::Threads 链接。
```

感知包还手动设置并链接了 RKNN、RGA、JPEG 等路径；它们当前不是由 `find_package(...)` 找到的。

`find_package` 和 `ament_target_dependencies` 的关系：

```text
find_package：先找到依赖。
ament_target_dependencies：让指定程序实际使用 ROS 2 依赖。
```

## 3. 将源文件编译成可执行程序或库

### 可执行程序

```cmake
add_executable(程序名
  源文件1.cpp
  源文件2.cc
)
```

- `add_executable`：生成可运行程序。
- 第一个参数是最终程序名；后面列出所有一起编译的 `.cpp/.cc`。
- 只给一个节点使用的小 vendor 源码，可以直接列在这里。

例子：

```cmake
add_executable(camera_node
  src/camera_node.cpp
  vendor/camera.cc
)
```

### 库

```cmake
add_library(库名 STATIC
  源文件1.cc
  源文件2.cc
)
```

- `add_library`：生成可复用库，不能直接 `ros2 run`。
- `STATIC`：静态库，最终会被编进使用它的程序。

例子：

```cmake
add_library(yolov5_seg STATIC
  vendor/yolov5_seg/yolov5_seg.cc
  vendor/yolov5_seg/postprocess.cc
)
```

然后让节点使用它：

```cmake
target_link_libraries(perception_node yolov5_seg)
```

## 4. 设置头文件路径

```cmake
target_include_directories(程序名 PRIVATE 目录)
```

当源码写：

```cpp
#include "camera.h"
```

编译器需要知道去哪里找 `camera.h`。若它在 `vendor/`，可写：

```cmake
target_include_directories(camera_node PRIVATE vendor)
```

```text
.h：声明。告诉调用方“有哪些函数/类、参数是什么”。
.cc/.cpp：实现。函数/类内部真正如何工作。
```

`PRIVATE` 与 `PUBLIC`：

```text
PRIVATE：只给当前目标自己使用。
PUBLIC ：给当前目标使用，也传给链接当前目标的下游目标。
```

`PUBLIC` 不等于“整个工程下所有程序都能找到这个目录”。

例如：

```cmake
target_include_directories(yolov5_seg PUBLIC vendor/yolov5_seg)
target_link_libraries(perception_node yolov5_seg)
```

因此 `perception_node` 可以 `#include "yolov5_seg.h"`；没有链接 `yolov5_seg` 的其他程序不会自动得到这个路径。

## 5. 声明 ROS 2 依赖

```cmake
ament_target_dependencies(程序名
  ROS包名1
  ROS包名2
)
```

例子：

```cmake
ament_target_dependencies(camera_node rclcpp sensor_msgs)
```

- 前提：每个 ROS 包前面都已写 `find_package`。
- 这里写 ROS 2 包名，例如 `rclcpp`、`sensor_msgs`、`scoutcar_msgs`。
- ament 会处理常见的头文件路径、ROS 库、编译参数和下游依赖。

从源码反推：

```cpp
#include <rclcpp/rclcpp.hpp>         // 需要 rclcpp
#include <sensor_msgs/msg/image.hpp> // 需要 sensor_msgs
```

## 6. 链接普通库或自建库

```cmake
target_link_libraries(程序名
  库名1
  库名2
)
```

例子：

```cmake
target_link_libraries(camera_node ${OpenCV_LIBS})
```

```text
find_package(OpenCV REQUIRED)：找到 OpenCV。
target_link_libraries(... ${OpenCV_LIBS})：让最终程序接上 OpenCV 函数的实现。
```

还可以链接：

```cmake
target_link_libraries(serial_node wiringPi Threads::Threads)
target_link_libraries(perception_node yolov5_seg)
```

当前阶段的区分：

```text
ament_target_dependencies：用于 ROS 2 / ament 管理的包。
target_link_libraries：常用于普通 C/C++ 库或自己用 add_library 创建的库。
```

## 7. 安装程序和资源

安装程序，使 `ros2 run` 或 launch 能找到它：

```cmake
install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME}
)
```

安装 launch、配置、模型等资源：

```cmake
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
```

关系：

```text
add_executable：构建阶段生成程序。
install：安装阶段把程序/资源放到 ROS 2 约定位置。
launch：通过 package + executable 找到并启动已安装程序。
```

## 8. 结束当前 ROS 2 包

```cmake
ament_package()
```

- 通常写在文件最后。
- 表示当前 ROS 2 包的 CMake 配置结束。
