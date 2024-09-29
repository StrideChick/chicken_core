cmake_minimum_required(VERSION 3.8)
project(chicken_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
  urdf
)

ament_auto_add_library(chicken_simulator SHARED
  src/chicken_simulator.cpp
)

rclcpp_components_register_node(chicken_simulator
  PLUGIN "ChickenSimulator"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_auto_package(
  INSTALL_TO_SHARE
  config
  rviz
  launch
  urdf
  world
)
