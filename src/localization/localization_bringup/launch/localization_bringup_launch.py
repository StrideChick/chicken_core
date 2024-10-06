from launch import LaunchDescription
from launch_ros.actions import Node
import os
from glob import glob
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    ekf_localization = get_package_share_directory("ekf_localization")

    ekf_localization_launch = launch.actions.IncludeLaunchDescription(
        XMLLaunchDescriptionSource([ekf_localization + "/launch/ekf_localization_launch.xml"]),
    )
    
    return launch.LaunchDescription([
        ekf_localization_launch,
    ])