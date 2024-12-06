#include "map_manager/pcd_loader.hpp"
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<map_manager::PCDMapLoader>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
