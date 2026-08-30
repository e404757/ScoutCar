cmake_minimum_required(VERSION 3.8)
project(scoutcar_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(scoutcar_msgs REQUIRED)

# ── 原样搬的算法库（vendor/，只调 include 路径，不改逻辑）──
add_library(pathplaning STATIC
    vendor/pathplaning/graph.cpp
    vendor/pathplaning/astar.cpp
    vendor/pathplaning/map.cpp
    vendor/pathplaning/navigation.cpp
)
target_include_directories(pathplaning PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/pathplaning
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/scoutcar_control/include/scoutcar_control  # protocol.h（TurnAction）
)

add_library(mission STATIC vendor/mission/mission.cpp)
target_include_directories(mission PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/vendor/mission)
target_compile_options(mission PRIVATE -O3)   # DP 规划热路径，保留原工程优化
target_link_libraries(mission PUBLIC pathplaning)

# ── 任务节点 ──
add_executable(mission_node src/mission_node.cpp)
target_include_directories(mission_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/pathplaning
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/mission
)
ament_target_dependencies(mission_node rclcpp std_msgs scoutcar_msgs)
target_link_libraries(mission_node mission pathplaning)

install(TARGETS mission_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
