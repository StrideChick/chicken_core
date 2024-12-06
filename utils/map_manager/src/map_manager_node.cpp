#include "map_manager/map_manager.hpp"
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<map_manager::MapManager>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
