from launch import LaunchDescription
from launch_ros.actions import Node
import os
from glob import glob
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    a_star_planner = get_package_share_directory("a_star_planner")
    pure_pursuit_planner = get_package_share_directory("pure_pursuit_planner")

    a_star_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([a_star_planner + "/launch/a_star_planner.launch.py"]),
        launch_arguments={
        }.items()
    )
    pure_pursuit_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pure_pursuit_planner + "/launch/pure_pursuit_planner.py"]),
        launch_arguments={
        }.items()
    )
    
    return launch.LaunchDescription([
        a_star_launch,
        pure_pursuit_launch,
    ])