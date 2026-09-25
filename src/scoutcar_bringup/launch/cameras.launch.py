from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    log_level = LaunchConfiguration("log_level")
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
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("log_level", default_value="info"),
            Node(
                package="scoutcar_camera",
                executable="camera_node",
                name="front_camera_node",
                **common,
            ),
            Node(
                package="scoutcar_camera",
                executable="camera_node",
                name="turn_camera_node",
                **common,
            ),
        ]
    )
