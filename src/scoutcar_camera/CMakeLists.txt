cmake_minimum_required(VERSION 3.8)
project(scoutcar_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 摄像头驱动节点：vendor/camera.cc 原样搬（OpenCV VideoCapture 封装）
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
