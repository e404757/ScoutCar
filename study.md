1、
src/....../
  ├── CMakeLists.txt  ← 告诉构建工具：如何编译、依赖谁、安装什么
  ├── package.xml     ← 告诉 ROS 2：这是一个什么包、依赖谁
  ├── src/            ← 你自己写的节点源码
  ├── include/        ← 头文件（有些包会有）
  └── vendor/         ← 外部/第三方源码

2、
cmake_minimum_required(VERSION 3.8)
project("工程名")

3、
#查找已经安装好的工具或库
find_package(...... REQUIRED)

rclcpp 
OpenCV
rosidl_default_generators  ros2消息代码生成工具

4、
add_executable(生成的程序名 参与编译的源文件...)
把指定的源文件一起编译，生成一个程序
这适合很小、只被一个节点使用的 文件。

5、
target_include_directories(程序名 PRIVATE 目录)
#include "..."时，编译器默认只会在当前目录等少数位置找头文件。
这句是在告诉他额外去某个目录找文件，PRIVATE表示只给这个程序用

6、
ament_target_dependencies(camera_node rclcpp sensor_msgs) #用于ros2/ament管理的包，ament 会根据这些 ROS 包公开的信息，自动帮你处理常见的：
- 头文件路径；
- 需要链接的 ROS 库；
- 编译参数；
- 下游依赖关系。
camera_node 这个程序使用 ROS 2 的 rclcpp 和 sensor_msgs

7、这是标准 CMake 命令，常用来连接普通 C/C++ 库：
target_link_libraries(camera_node ${OpenCV_LIBS})

8、
ament_export_dependencies(rosidl_default_runtime)