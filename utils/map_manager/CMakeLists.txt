cmake_minimum_required(VERSION 3.8)
project(map_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#
ament_auto_add_library(map_manager_component SHARED
  src/map_manager.cpp
)
ament_auto_add_library(pcd_loader SHARED
  src/pcd_loader.cpp

)
ament_auto_add_library(pcd_to_pointcloud SHARED
  src/pcd_to_pointcloud.cpp
  
)
#
target_link_libraries(map_manager_component ${YAML_CPP_LIBRARIES})

#
rclcpp_components_register_nodes(map_manager_component
  "map_manager::MapManager"
)
rclcpp_components_register_nodes(pcd_loader
  "map_manager::PCDMapLoader"
)
rclcpp_components_register_nodes(pcd_to_pointcloud
  "pcl_to_pointcloud::PCDPublisher"
)

#
ament_auto_add_executable(map_manager_node
    NO_TARGET_LINK_LIBRARIES
    src/map_manager_node.cpp
)
ament_auto_add_executable(pcd_loader_node
    NO_TARGET_LINK_LIBRARIES
    src/pcd_loader_node.cpp
)
ament_auto_add_executable(pcd_to_pointcloud_node
    NO_TARGET_LINK_LIBRARIES
    src/pcd_to_pointcloud_node.cpp
)
#
target_link_libraries(map_manager_node
  map_manager_component
)
target_link_libraries(pcd_loader_node
  pcd_loader
)
target_link_libraries(pcd_to_pointcloud_node
  pcd_to_pointcloud
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



install(FILES DESTINATION share/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
