from launch import LaunchDescription
from launch_ros.actions import Node
import os
from glob import glob
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

pkg_name = 'sensor_bringup'

def generate_launch_description():
    ld = LaunchDescription(
    )
    # IMU Filter Madgwick Node
    imu_filter_madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False
        }],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu')
        ]
    )
    realsense2_camera = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("realsense2_camera") + "/launch/rs_launch.py"]),
        launch_arguments={
            'enable_gyro': "True",
            'enable_accel': "True",
            'unite_imu_method': "1"
        }.items()
    )

    linefit_ground_segmentation_ros2 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("linefit_ground_segmentation_ros2") + "/launch/segmentation.launch.py"]),
    )


    ld.add_action(imu_filter_madgwick)
    ld.add_action(realsense2_camera)
    ld.add_action(linefit_ground_segmentation_ros2)
    return ld