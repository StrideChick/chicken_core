import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    chicken_simulator = get_package_share_directory("chicken_simulator")
    map_server = get_package_share_directory("map_server")
    wheelbot_control = get_package_share_directory("wheelbot_control")

    sensor_bringup = get_package_share_directory("sensor_bringup")
    planner_bringup = get_package_share_directory("planner_bringup")
    localization_bringup = get_package_share_directory("localization_bringup")
    
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
    
    sensor_bringup_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sensor_bringup + "/sensor_bringup_launch.py"]),
        launch_arguments={
        }.items()
    )
    
    planner_bringup_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([planner_bringup + "/planner_bringup_launch.py"]),
        launch_arguments={
        }.items()
    )
    
    localization_bringup_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_bringup + "/localization_bringup_launch.py"]),
        launch_arguments={
        }.items()
    )

    return launch.LaunchDescription([
        chicken_simulator_launch,
        wheelbot_control_launch,
        map_server_launch,
        sensor_bringup_launch,
        planner_bringup_launch,
        localization_bringup_launch,
    ])