import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'pure_pursuit_planner'

    pure_pursuit_planner_node = Node(
        package=package_name,
        executable='pure_pursuit_planner',
        output="screen",
    )

    nodes = [
        pure_pursuit_planner_node,
    ]

    return LaunchDescription(nodes)
