#include "map_manager/pcd_loader.hpp"

namespace map_manager
{
  PCDMapLoader::PCDMapLoader(const rclcpp::NodeOptions & options) 
  : Node("map_manager", options)
  {
    declare_parameter<std::string>("pcd_file_path","/home/suke/tukuba_ws/src/utils/map_manager/tsukuba_pointcloud.pcd");
    declare_parameter<float>("resolution", 0.1);  
    declare_parameter<float>("width", 300.0);      
    declare_parameter<float>("height", 300.0);     
    declare_parameter<std::vector<double>>("origin", {150.0, 150.0, 0.0});        
    get_parameter("pcd_file_path", pcd_file_path_);
    get_parameter("resolution", resolution_);
    get_parameter("width", width_);
    get_parameter("height", height_);
    get_parameter("origin", origin_double);
    origin.assign(origin_double.begin(), origin_double.end());
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);
    initMap();
  }

  void PCDMapLoader::initMap()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!loadPCDFile(cloud)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", pcd_file_path_.c_str());
      return;
    }
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    createOccupancyGrid(cloud, occupancy_grid);
    map_pub_->publish(occupancy_grid);
  }

  bool PCDMapLoader::loadPCDFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", pcd_file_path_.c_str());
      return false;
    } 
    return true;
  }

  void PCDMapLoader::createOccupancyGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & occupancy_grid)
  {
    occupancy_grid.header.stamp = now();
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.map_load_time = now();
    occupancy_grid.info.resolution = resolution_;
    occupancy_grid.info.width = static_cast<uint32_t>(width_ / resolution_);
    occupancy_grid.info.height = static_cast<uint32_t>(height_ / resolution_);
    occupancy_grid.info.origin.position.x = origin[0];
    occupancy_grid.info.origin.position.y = origin[1];
    occupancy_grid.info.origin.orientation.w = origin[2];
    occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height, -1);  
    for (const auto & point : cloud->points) {
      int grid_x = static_cast<int>((point.x - origin[0]) / (width_ * resolution_));
      int grid_y = static_cast<int>((point.y - origin[1]) / (height_* resolution_));
      if (grid_x >= 0 && grid_x < occupancy_grid.info.width && grid_y >= 0 && grid_y < occupancy_grid.info.height) {
        int index = grid_x + grid_y * occupancy_grid.info.width;
        occupancy_grid.data[index] = 100;  
      }
    }
  }
}  // namespace map_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(map_manager::PCDMapLoader)
