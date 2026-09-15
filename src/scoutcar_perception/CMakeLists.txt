cmake_minimum_required(VERSION 3.8)
project(scoutcar_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(TARGET_SOC "rk3588")
set(DISABLE_RGA "OFF")
set(DISABLE_LIBJPEG "OFF")


set(SCOUTCAR_3RDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../3rd_party"
    CACHE PATH "CityScout 的 3rdparty 目录")

set(RKNN_INCLUDES    ${SCOUTCAR_3RDPARTY_DIR}/rknpu2/include)
set(LIBRKNNRT        ${SCOUTCAR_3RDPARTY_DIR}/rknpu2/Linux/aarch64/librknnrt.so)
set(RGA_INCLUDES     ${SCOUTCAR_3RDPARTY_DIR}/librga/include)
set(LIBRGA           ${SCOUTCAR_3RDPARTY_DIR}/librga/Linux/aarch64/librga.a)
set(STB_INCLUDES     ${SCOUTCAR_3RDPARTY_DIR}/stb_image)
set(JPEG_INCLUDES    ${SCOUTCAR_3RDPARTY_DIR}/jpeg_turbo/include)
set(LIBJPEG          ${SCOUTCAR_3RDPARTY_DIR}/jpeg_turbo/Linux/aarch64/libturbojpeg.a)
set(ALLOC_INCLUDES   ${SCOUTCAR_3RDPARTY_DIR}/allocator/dma ${SCOUTCAR_3RDPARTY_DIR}/allocator/drm)
set(TIMER_INCLUDES   ${SCOUTCAR_3RDPARTY_DIR}/timer)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(scoutcar_msgs REQUIRED)


add_library(fileutils STATIC vendor/common/file_utils.c)
target_include_directories(fileutils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/vendor/common)

add_library(imageutils STATIC vendor/common/image_utils.c)
target_include_directories(imageutils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/common
    ${STB_INCLUDES}
    ${RGA_INCLUDES}
    ${JPEG_INCLUDES}
)
target_compile_definitions(imageutils PRIVATE LIBRGA_IM2D_HANDLE)
target_link_libraries(imageutils ${LIBRGA} ${LIBJPEG})

add_library(yolov5_seg STATIC
    vendor/yolov5_seg/yolov5_seg.cc
    vendor/yolov5_seg/postprocess.cc
)
target_include_directories(yolov5_seg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/vendor/yolov5_seg)
target_include_directories(yolov5_seg PRIVATE
    ${RKNN_INCLUDES}
    ${ALLOC_INCLUDES}
    ${TIMER_INCLUDES}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(yolov5_seg PUBLIC imageutils fileutils)

add_library(segmentation_model STATIC
    src/segmentation_model.cpp
)
target_include_directories(segmentation_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/yolov5_seg
    ${RKNN_INCLUDES}
)
target_link_libraries(segmentation_model PUBLIC
    yolov5_seg
    ${LIBRKNNRT}
)

# ── YOLOv8 检测算法：vendor RKNN实现 + 项目级C++封装 ──
add_library(yolov8_detect_vendor STATIC
    vendor/yolov8_detect/yolov8.cc
    vendor/yolov8_detect/postprocess.cc
)
target_include_directories(yolov8_detect_vendor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/yolov8_detect
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/common
    ${RKNN_INCLUDES}
)
target_link_libraries(yolov8_detect_vendor PUBLIC
    imageutils
    fileutils
    ${LIBRKNNRT}
)

add_library(yolov8_model STATIC
    src/yolov8_model.cpp
)
target_include_directories(yolov8_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/yolov8_detect
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/common
    ${RKNN_INCLUDES}
)
target_link_libraries(yolov8_model PUBLIC yolov8_detect_vendor)



add_library(mobilenet_classifier STATIC
    vendor/mobilenet/mobilenet.cc
)
target_include_directories(mobilenet_classifier PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/mobilenet
    ${RKNN_INCLUDES}
)
target_link_libraries(mobilenet_classifier PUBLIC imageutils fileutils)

add_library(road_tracking STATIC
    src/road_tracker.cpp
)
target_include_directories(road_tracking PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_library(perception_visualization STATIC
    src/perception_visualizer.cpp
)
target_include_directories(perception_visualization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(perception_visualization sensor_msgs)
target_link_libraries(perception_visualization
    road_tracking
    ${OpenCV_LIBS}
)

# ── 感知节点：图像 → Seg → IPM → RoadTracker → 发布道路偏差 ──
add_executable(perception_node src/perception_node.cpp)
target_include_directories(perception_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(perception_node ament_index_cpp rclcpp sensor_msgs std_msgs scoutcar_msgs)
target_link_libraries(perception_node
    segmentation_model
    yolov8_model
    road_tracking
    ${OpenCV_LIBS}
    perception_visualization
    pthread
    dl
)
# imageutils 静态链接了 vendored libturbojpeg.a，其 jpeg_* 符号默认会被导出到
# 可执行文件的动态符号表，导致 OpenCV imgcodecs 运行时错误绑定到这套 ABI
# 不兼容的实现（cv::imencode 内 jpeg_CreateCompress 段错误，2026-08-30）。
# --exclude-libs 阻止导出，符号仅供静态链接内部使用。
target_link_options(perception_node PRIVATE
    "LINKER:--exclude-libs,libturbojpeg.a"
)

# ── MobileNet 分类节点：收正前方图像 → 分类 → 发布最佳类别 ──
add_executable(classify_node src/classify_node.cpp)
ament_target_dependencies(classify_node ament_index_cpp rclcpp sensor_msgs std_msgs)
target_link_libraries(classify_node
    mobilenet_classifier imageutils fileutils
    ${LIBRKNNRT}
    pthread
    dl
)

install(TARGETS perception_node classify_node
  DESTINATION lib/${PROJECT_NAME})

# 模型和标签由 ament 资源索引定位；RKNN 运行库随节点安装，避免依赖旧工程路径。
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})
install(FILES ${LIBRKNNRT}
  DESTINATION lib/${PROJECT_NAME})
set_target_properties(perception_node PROPERTIES INSTALL_RPATH "$ORIGIN")
set_target_properties(classify_node PROPERTIES INSTALL_RPATH "$ORIGIN")

ament_package()
