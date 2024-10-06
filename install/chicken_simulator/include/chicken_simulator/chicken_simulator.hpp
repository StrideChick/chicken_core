#ifndef CHICKEN_SIMULATOR__Chicken_SIMPLE_SIMULATOR_HPP_
#define CHICKEN_SIMULATOR__Chicken_SIMPLE_SIMULATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class ChickenSimulator : public rclcpp::Node
{
public:
  explicit ChickenSimulator(const rclcpp::NodeOptions & node_options);
  
  void initial_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & msg);
  void cmd_vel_cb(const geometry_msgs::msg::Twist & msg);

  void update();
  void publish_tf(
    const geometry_msgs::msg::Pose pose, const std::string base_frame,
    const std::string child_frame, const rclcpp::Time stamp);

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pubilsher_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::Twist cmd_vel_msg_;

  rclcpp::Time current_time_;
  rclcpp::Time previous_time_;

  double period_ = 0.01;
  std::string base_frame_id_ = "base_footprint" ;

  std::array<double, 36> covariance_;
  std::vector<std::string> joint_names_{"right_wheel_joint", "left_wheel_joint"};
};

#endif  // Chicken_SIMULATOR__Chicken_SIMPLE_SIMULATOR_HPP_
