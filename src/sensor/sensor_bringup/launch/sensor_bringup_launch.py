from launch import LaunchDescription
from launch_ros.actions import Node
import os
from glob import glob
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

pkg_name = 'sensor_bringup'

def generate_launch_description():
    hesai_ros_driver = get_package_share_directory("hesai_ros_driver")
    realsense2_camera = get_package_share_directory("realsense2_camera")

    hesai_ros_driver_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([hesai_ros_driver + "/launch/start.py"]),
        launch_arguments={
        }.items()
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

    realsense2_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense2_camera + "/launch/rs_launch.py"]),
        launch_arguments={
            'enable_gyro': "True",
            'enable_accel': "True",
            'unite_imu_method': "1",
        }.items()
    )

    # linefit_ground_segmentation_ros2 = launch.actions.IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory("linefit_ground_segmentation_ros2") + "/launch/segmentation.launch.py"]),
    # )

    return launch.LaunchDescription([
        imu_filter_madgwick,
        realsense2_launch,
        #linefit_ground_segmentation_ros2,
        hesai_ros_driver_launch,
    ])