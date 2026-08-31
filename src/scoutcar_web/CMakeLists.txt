cmake_minimum_required(VERSION 3.8)
project(scoutcar_web)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(scoutcar_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# web_node：网页推流 + 双文件录像（回放/补标在 Windows 端）
#   vendor/httplib.h —— 原工程单文件 HTTP 库（头文件库，无需链接）
#   src/video_writer.cpp —— 异步 MJPEG .avi 写入器（原工程 main/video_writer.cc 适配）
#   src/dual_recorder.cpp —— 原始 + 叠加双文件同步片段录制
#   （叠加画面由感知节点同帧合成发布 debug_image，Web 端不再自行绘制）
add_executable(web_node
  src/web_node.cpp
  src/video_writer.cpp
  src/dual_recorder.cpp
  src/bag_recorder.cpp
)
target_include_directories(web_node PRIVATE vendor)
target_compile_features(web_node PUBLIC cxx_std_17)
ament_target_dependencies(web_node rclcpp sensor_msgs std_msgs scoutcar_msgs)
target_link_libraries(web_node ${OpenCV_LIBS} pthread)

install(TARGETS web_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
