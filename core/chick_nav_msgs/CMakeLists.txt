cmake_minimum_required(VERSION 3.8)
project(chick_nav_msgs)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()
rosidl_generate_interfaces(${PROJECT_NAME}
  # "action/NavigateToGoal.action"
  # "msg/Waypoints.msg"
  "srv/NavigateToGoal.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
