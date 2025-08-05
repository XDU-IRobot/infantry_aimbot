import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="infantry_aimbot",
                executable="infantry_aimbot_node",
                parameters=[os.path.join(get_package_share_directory(
                    'infantry_aimbot'), "config", "settings.yaml")],
                output="screen"),
        ],
    )