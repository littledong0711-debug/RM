from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("armor_aim"))
    return LaunchDescription(
        [
            Node(
                package="armor_aim",
                executable="armor_aim_node",
                name="armor_aim",
                output="screen",
                parameters=[str(package_share / "config" / "params.yaml")],
            )
        ]
    )
