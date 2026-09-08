cmake_minimum_required(VERSION 3.8)
project(scoutcar_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#查找已经安装好的库
find_package(ament_cmake REQUIRED)#ros2的cmake包
find_package(rclcpp REQUIRED)#ros2的c++节点库
find_package(sensor_msgs REQUIRED)#ros2的传感器消息库
find_package(OpenCV REQUIRED)#opencv库

# 摄像头驱动节点
add_executable(camera_node
  src/camera_node.cpp
  vendor/camera.cc
)
target_include_directories(camera_node PRIVATE vendor)
ament_target_dependencies(camera_node rclcpp sensor_msgs)
target_link_libraries(camera_node ${OpenCV_LIBS})

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
