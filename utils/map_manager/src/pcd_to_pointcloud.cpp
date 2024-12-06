#include "map_manager/pcd_to_pointcloud.hpp"
#include <pcl/common/io.h>

namespace pcd_to_pointcloud
{
PCDPublisher::PCDPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("pcd_publisher", options)
{
  std::string file_name_;
  std::string cloud_topic_;

  declare_parameter("cloud_topic","/map_cloud");
  declare_parameter("tf_frame", "map");
  declare_parameter("publishing_period_ms", 100);
  declare_parameter<std::string>("pcd_file_path", "/home/suke/tukuba_ws/src/utils/map_manager/tsukuba_pointcloud.pcd");
  declare_parameter("distance_threshold", 50.0);
  
  get_parameter("cloud_topic", cloud_topic_);
  get_parameter("tf_frame", tf_frame_);
  get_parameter("period_ms", period_ms_);
  get_parameter("pcd_file_path", file_name_);
  get_parameter("distance_threshold", distance_threshold_);

  if (file_name_ == "" || pcl::io::loadPCDFile(file_name_, cloud_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "failed to open PCD file");
    throw std::runtime_error{"could not open pcd file"};
  }

  cloud_.header.frame_id = tf_frame_;
  int nr_points = cloud_.width * cloud_.height;
  auto fields_list = pcl::getFieldsList(cloud_);
  auto resolved_cloud_topic = this->get_node_topics_interface()->resolve_topic_name(cloud_topic_);

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 10);
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/estimated_pose", 10, std::bind(&PCDPublisher::pose_callback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period_ms_),
    std::bind(&PCDPublisher::publish, this)
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Publishing data with %d points (%s) on topic %s in frame %s, within distance %.2f m.",
    nr_points,
    fields_list.c_str(),
    resolved_cloud_topic.c_str(),
    cloud_.header.frame_id.c_str(),
    distance_threshold_);
}

void PCDPublisher::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  robot_pose_ = msg;
}

void PCDPublisher::publish()
{
  sensor_msgs::msg::PointCloud2 tmp_cloud = cloud_;
  filterPointCloud(tmp_cloud); 
  tmp_cloud.header.stamp = this->get_clock()->now();
  tmp_cloud.header.frame_id = tf_frame_;
  cloud_pub_->publish(tmp_cloud);
}

void PCDPublisher::filterPointCloud(sensor_msgs::msg::PointCloud2 & cloud)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(cloud, pcl_cloud);
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  for (const auto& point : pcl_cloud.points) {
      float distance = std::sqrt(std::pow(point.x - robot_pose_.pose.pose.position.x, 2) + std::pow(point.y - robot_pose_.pose.pose.position.y, 2));   
      if (distance <= distance_threshold_) {
          filtered_cloud.push_back(point);
      }
  }
  pcl::toROSMsg(filtered_cloud, cloud);
  cloud_.header.stamp = this->get_clock()->now();
  cloud_.header.frame_id = tf_frame_;

}  
}// namespace pcl_to_pointcloud

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd_to_pointcloud::PCDPublisher)
