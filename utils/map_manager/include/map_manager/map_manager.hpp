#ifndef MAP_MANAGER_HPP_
#define MAP_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace map_manager
{
struct Pgm
{
  std::string header, image;
  int rows, cols, max_val, negate;
  double resolution, occupied_thresh, free_thresh;
  std::vector<unsigned char> pixels;
  std::vector<double> origin;
};

class MapManager: public rclcpp::Node
{
public:
  explicit MapManager(const rclcpp::NodeOptions & options);

private:    
  std::string map_yaml_path_;
  std::string mode_;

  void initMAP();
  bool loadParamsFromYaml(Pgm & pgm);
  bool readPgm(Pgm & pgm);
  bool readPCD(Pgm & pgm);
  void createMAP(Pgm & pgm, nav_msgs::msg::OccupancyGrid & occupancy_grid);

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;  
};
}  // namespace map_manager
#endif  