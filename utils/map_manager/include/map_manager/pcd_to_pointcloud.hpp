#ifndef PCD_PUBLISHER_HPP_
#define PCD_PUBLISHER_HPP_

#include <string>
#include <chrono>
#include <memory>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace pcd_to_pointcloud
{
class PCDPublisher : public rclcpp::Node
{
public:
  explicit PCDPublisher(const rclcpp::NodeOptions & options);

private:
  void loadPCDFile();
  void publish();
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void filterPointCloud(sensor_msgs::msg::PointCloud2 & cloud);

  std::string tf_frame_;
  size_t period_ms_;
  double distance_threshold_; 

  geometry_msgs::msg::PoseWithCovarianceStamped robot_pose_;
  sensor_msgs::msg::PointCloud2 cloud_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> cloud_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace pcl_ros

#endif  // PCD_PUBLISHER_HPP_
