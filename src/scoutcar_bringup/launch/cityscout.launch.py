from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    log_level = LaunchConfiguration("log_level")
    tracking_mode = LaunchConfiguration("tracking_mode")

    default_config = PathJoinSubstitution(
        [FindPackageShare("scoutcar_bringup"), "config", "cityscout.yaml"]
    )

    common = {
        "parameters": [config_file],
        "output": "screen",
        "arguments": ["--ros-args", "--log-level", log_level],
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="全车 ROS 2 参数文件",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS 日志级别",
            ),
            DeclareLaunchArgument(
                "tracking_mode",
                default_value="robust_center",
                description="road tracker: robust_center or legacy_width_barrier",
            ),
            Node(
                package="scoutcar_camera",
                executable="camera_node",
                name="mipi_camera_node",
                **common,
            ),
            Node(
                package="scoutcar_camera",
                executable="camera_node",
                name="usb_camera_node",
                **common,
            ),
            Node(
                package="scoutcar_perception",
                executable="perception_node",
                name="perception_node",
                parameters=[config_file, {"road_tracking.mode": tracking_mode}],
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="scoutcar_planning",
                executable="mission_node",
                name="mission_node",
                **common,
            ),
            Node(package="scoutcar_control", executable="serial_node", name="serial_node", **common),
        ]
    )
