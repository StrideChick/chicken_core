#include "map_manager/pcd_to_pointcloud.hpp"
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<pcd_to_pointcloud::PCDPublisher>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
