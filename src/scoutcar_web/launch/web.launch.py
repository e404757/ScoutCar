# web_node 单独启动（不依赖整车主 launch，方便只调试网页/录像）：
#   ros2 launch scoutcar_web web.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scoutcar_web',
            executable='web_node',
            name='web_node',
            output='screen',
            parameters=['/home/orangepi/CityScout/src/scoutcar_web/config/web.yaml'],
        ),
    ])
