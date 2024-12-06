from launch import LaunchDescription
from launch_ros.actions import Node
import os
from glob import glob
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

pkg_name = 'sensor_bringup'

def generate_launch_description():
    hesai_ros_driver = get_package_share_directory("hesai_ros_driver")
    realsense2_camera = get_package_share_directory("realsense2_camera")    
    linefit_ground_segmentation_ros2 = get_package_share_directory("linefit_ground_segmentation_ros") 
    grid_map_generator = get_package_share_directory("grid_map_generator")
    

    #-------------------------------------------------------------------------------------
    # camere
    realsense2_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense2_camera + "/launch/rs_launch.py"]),
        launch_arguments={
            'enable_gyro': "True",
            'enable_accel': "True",
            'unite_imu_method': "1",
        }.items()
    )

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

    #-------------------------------------------------------------------------------------
    # Lidar

    hesai_ros_driver_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([hesai_ros_driver + "/launch/start.py"]),
        launch_arguments={
        }.items()
    )
    
    linefit_ground_segmentation_ros2 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("linefit_ground_segmentation_ros") + "/launch/segmentation.launch.py"]),
    )

    grid_map_generator_launch = launch.actions.IncludeLaunchDescription(
        XMLLaunchDescriptionSource([grid_map_generator + "/launch/grid_map_generator.xml"]),
    )

    return launch.LaunchDescription([
        realsense2_launch,
        imu_filter_madgwick,
        hesai_ros_driver_launch,
        linefit_ground_segmentation_ros2,
        grid_map_generator_launch,
    ])