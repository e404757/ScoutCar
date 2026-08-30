"""仅启动 CityScout 的 MIPI 与 USB 图像发布节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    default_config = PathJoinSubstitution(
        [FindPackageShare("scoutcar_bringup"), "config", "cityscout.yaml"]
    )
    common = {"parameters": [config_file], "output": "screen"}

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="双相机参数文件",
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
        ]
    )
