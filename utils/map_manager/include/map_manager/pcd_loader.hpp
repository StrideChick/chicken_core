#ifndef PCD_MAP_LOADER_HPP_
#define PCD_MAP_LOADER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace map_manager
{
  class PCDMapLoader : public rclcpp::Node
  {
  public:
    explicit PCDMapLoader(const rclcpp::NodeOptions & options);

  private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    std::string pcd_file_path_;
    float resolution_;
    float width_;
    float height_;
    std::vector<double> origin_double;
    std::vector<float> origin;
    void initMap();
    bool loadPCDFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void createOccupancyGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & occupancy_grid);
  };
}

#endif  // PCD_MAP_LOADER_HPP_
