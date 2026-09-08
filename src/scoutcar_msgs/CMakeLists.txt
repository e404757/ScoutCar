cmake_minimum_required(VERSION 3.8)
project(scoutcar_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)#ros2的idl生成器
find_package(std_msgs REQUIRED)#ros2的标准消息包

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RxEvent.msg"
  "msg/PathCmd.msg"
  "msg/MissionStatus.msg"
  "msg/RoadDeviation.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
