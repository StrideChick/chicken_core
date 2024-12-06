#include "map_manager/map_manager.hpp"

namespace map_manager
{
MapManager::MapManager(const rclcpp::NodeOptions & options) 
: Node("map_manager", options)
{
  std::string map_topic_name;
  declare_parameter<std::string>("map_yaml_path","/home/suke/chicken_core/src/utils/map_manager/config/image.yaml");
  declare_parameter<std::string>("map_topic_name","/map"); 
  get_parameter("map_yaml_path",map_yaml_path_);
  get_parameter("map_topic_name",map_topic_name);
  
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic_name, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  initMAP();
}

void MapManager::initMAP(){
  Pgm pgm;
  if(loadParamsFromYaml(pgm)){
    readPgm(pgm);
    nav_msgs::msg::OccupancyGrid map;
    createMAP(pgm, map);
    map_pub_->publish(map);
  }
}

bool MapManager::loadParamsFromYaml(Pgm & pgm)
{
  YAML::Node config = YAML::LoadFile(map_yaml_path_);
  try {
    pgm.resolution = config["resolution"].as<double>();
  } catch (YAML::InvalidScalar &) {
    RCLCPP_ERROR(this->get_logger(),"The map does not contain a resolution tag or it is invalid.");
    return false;
  }
  try {
    pgm.image = config["image"].as<std::string>();
  } catch (YAML::InvalidScalar &) {
    RCLCPP_ERROR(this->get_logger(),"The map does not contain a image tag or it is invalid.");
    return false;
  }
  try {
    pgm.negate = config["negate"].as<int>();
  } catch (YAML::InvalidScalar &) {
    RCLCPP_ERROR(this->get_logger(),"The map does not contain a negate tag or it is invalid.");
    return false;
  }
  try {
    pgm.occupied_thresh = config["occupied_thresh"].as<double>();
  } catch (YAML::InvalidScalar &) {
    RCLCPP_ERROR(this->get_logger(),"The map does not contain a occupied_thresh tag or it is invalid.");
    return false;
  }
  try {
    pgm.free_thresh = config["free_thresh"].as<double>();
  } catch (YAML::InvalidScalar &) {
    RCLCPP_ERROR(this->get_logger(),"The map does not contain a free_thresh tag or it is invalid.");
    return false;
  }
  try {
     pgm.origin = config["origin"].as<std::vector<double>>();
  } catch (YAML::InvalidScalar &) {
    RCLCPP_ERROR(this->get_logger(),"The map does not contain a origin tag or it is invalid.");
    return false;
  }
  return true;
}

bool MapManager::readPgm(Pgm & pgm)
{
  std::ifstream file(pgm.image,std::ios::binary);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open PGM file: %s", pgm.image.c_str());
    return false;
  }
  std::string magic_number;
  file >> magic_number;
  if (magic_number != "P5") {
    RCLCPP_ERROR(this->get_logger(), "Unsupported PGM format: %s", magic_number.c_str());
    return false;
  }

  std::string check_cols;
  file >> check_cols;
  if (check_cols == "#") {
    file >> check_cols >> check_cols >> check_cols >> check_cols;
    file >> pgm.cols >> pgm.rows >> pgm.max_val;
  } else {
    pgm.cols = std::stoi(check_cols);
    file >> pgm.rows >> pgm.max_val;
  }

  if (pgm.max_val > 255) {
    std::cerr << "Can only handle 8-bit pixels!\n";
    return false;
  }
  file.ignore(1);  // skip newline character
  pgm.pixels.resize(pgm.rows * pgm.cols);
  file.read(reinterpret_cast<char *>(pgm.pixels.data()), pgm.pixels.size());
  return true;
}

void MapManager::createMAP(Pgm & pgm, nav_msgs::msg::OccupancyGrid & occupancy_grid)
{
  occupancy_grid.header.stamp = now();
  occupancy_grid.header.frame_id = "map";
  occupancy_grid.info.map_load_time = now();
  occupancy_grid.info.resolution = pgm.resolution;
  occupancy_grid.info.width = pgm.cols;
  occupancy_grid.info.height = pgm.rows;
  occupancy_grid.info.origin.position.x = pgm.origin[0];
  occupancy_grid.info.origin.position.y = pgm.origin[1];
  occupancy_grid.info.origin.orientation.w = cos(pgm.origin[2] / 2);
  occupancy_grid.info.origin.orientation.z = sin(pgm.origin[2] / 2);
  
  int map_size = occupancy_grid.info.width * occupancy_grid.info.height;
  occupancy_grid.data.resize(map_size);

  for (auto & pixel : pgm.pixels) {
    if (pixel >= 254)
      pixel = 0;
    else if (pixel == 205)
      pixel = -1;
    else if (pixel == 0)
      pixel = 100;
  }

  std::vector<int8_t> pixels;
  for (auto & pixel : pgm.pixels) {
    pixels.push_back(static_cast<int8_t>(pixel));
  }

  unsigned int index = 0;
  for (unsigned int y = 0; y < occupancy_grid.info.height; y++) {
    for (unsigned int x = 0; x < occupancy_grid.info.width; x++) {
      unsigned int i = x + (occupancy_grid.info.height - y - 1) * occupancy_grid.info.width;
      occupancy_grid.data[i] = pixels[index];
      ++index;
    }
  }
}

}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(map_manager::MapManager)
