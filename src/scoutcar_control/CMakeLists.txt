cmake_minimum_required(VERSION 3.8)
project(scoutcar_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(scoutcar_msgs REQUIRED)
find_package(Threads REQUIRED)

# 串口驱动节点：节点壳 + 原样搬的 serial.cc（依赖 wiringPi）
add_executable(serial_node
  src/serial_node.cpp
  src/serial.cc
)
target_include_directories(serial_node PRIVATE include)
ament_target_dependencies(serial_node rclcpp std_msgs scoutcar_msgs)
target_link_libraries(serial_node wiringPi Threads::Threads)

install(TARGETS serial_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
