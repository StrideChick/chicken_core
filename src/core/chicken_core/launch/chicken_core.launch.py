import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    a_star_planner = get_package_share_directory("a_star_planner")
    pure_pursuit_planner = get_package_share_directory("pure_pursuit_planner")
    chicken_simulator = get_package_share_directory("chicken_simulator")
    map_server = get_package_share_directory("map_server")
    wheelbot_control = get_package_share_directory("wheelbot_control")
    realsense2_camera = get_package_share_directory("realsense2_camera")
    hesai_ros_driver = get_package_share_directory("hesai_ros_driver")
    
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
    chicken_simulator_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([chicken_simulator + "/launch/chicken_simulator_bringup.launch.py"]),
        launch_arguments={
        }.items()
    )
    wheelbot_control_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([wheelbot_control + "/wheelbot_control_launch.py"]),
        launch_arguments={
        }.items()
    )
    map_server_launch = launch.actions.IncludeLaunchDescription(
        XMLLaunchDescriptionSource([map_server + "/launch/map_server_launch.xml"]),
    )
    realsense2_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense2_camera + "/launch/rs_launch.py"]),
        launch_arguments={
            'enable_gyro': "True",
            'enable_accel': "True",
            'unite_imu_method': "1",
        }.items()
    )
    hesai_ros_driver_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([hesai_ros_driver + "/launch/start.py"]),
        launch_arguments={
        }.items()
    )
    return launch.LaunchDescription([
        # a_star_launch,
        # pure_pursuit_launch,
        # chicken_simulator_launch,
        wheelbot_control_launch,
        # map_server_launch,
        realsense2_launch,
        # hesai_ros_driver_launch,
    ])