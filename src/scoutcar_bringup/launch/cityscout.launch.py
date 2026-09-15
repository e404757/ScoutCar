from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    log_level = LaunchConfiguration("log_level")
    web = LaunchConfiguration("web")
    default_config = PathJoinSubstitution(
        [FindPackageShare("scoutcar_bringup"), "config", "cityscout.yaml"]
    )
    web_config = PathJoinSubstitution(
        [FindPackageShare("scoutcar_web"), "config", "web.yaml"]
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
                "web",
                default_value="false",
                description="true 启动绘制与 Web 转发；false 使用纯净感知",
            ),
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
            Node(
                package="scoutcar_perception",
                executable="perception_node",
                name="perception_node",
                parameters=[config_file],
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="scoutcar_planning",
                executable="mission_node",
                name="mission_node",
                **common,
            ),
            # Node(
            #     package="scoutcar_perception",
            #     executable="classify_node",
            #     name="classify_node",
            #     **common,
            # ),
            Node(package="scoutcar_control", executable="serial_node", name="serial_node", **common),
            Node(
                package="scoutcar_web",
                executable="web_node",
                name="web_node",
                parameters=[web_config],
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                condition=IfCondition(web),
            ),
        ]
    )
