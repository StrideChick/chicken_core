#ifndef NAVYU_PLANNER__NAVYU_GLOBAL_PLANNER_HPP_
#define NAVYU_PLANNER__NAVYU_GLOBAL_PLANNER_HPP_

#include "a_star_planner/astar_planner.hpp"
#include "a_star_planner/smoother.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "chick_nav_msgs/srv/navigate_to_goal.hpp"


bool get_robot_pose(
  const std::string global_frame, const std::string robot_frame, tf2_ros::Buffer & tf_buffer,
  geometry_msgs::msg::Pose & robot_pose)
{
  static rclcpp::Logger logger = rclcpp::get_logger("get_robot_pose");

  geometry_msgs::msg::TransformStamped robot_pose_frame;

  try {
    robot_pose_frame = tf_buffer.lookupTransform(
      global_frame, robot_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(logger, "Can not get Transform " << global_frame << " to " << robot_frame);
    return false;
  }

  robot_pose.position.x = robot_pose_frame.transform.translation.x;
  robot_pose.position.y = robot_pose_frame.transform.translation.y;
  robot_pose.position.z = robot_pose_frame.transform.translation.z;
  robot_pose.orientation = robot_pose_frame.transform.rotation;

  return true;
}

class AStarGlobalPlanner : public rclcpp::Node
{
public:
  explicit AStarGlobalPlanner(const rclcpp::NodeOptions & node_options);
  ~AStarGlobalPlanner();

  bool plan(
    const geometry_msgs::msg::Pose start, const geometry_msgs::msg::Pose goal,
    std::vector<Node2D *> & path);

  void callback_costmap(const nav_msgs::msg::OccupancyGrid & msg);
  void callback_goal_pose(const geometry_msgs::msg::PoseStamped & msg);
  void publish_path(std::vector<Node2D *> path);
  void goal_server(
    const std::shared_ptr<chick_nav_msgs::srv::NavigateToGoal::Request> request,
    const std::shared_ptr<chick_nav_msgs::srv::NavigateToGoal::Response> response
  );
  
private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_publisher_;
  rclcpp::Service<chick_nav_msgs::srv::NavigateToGoal>::SharedPtr goal_pose_server_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  std::shared_ptr<AstarPlanner> planner_;
  std::shared_ptr<Smoother> smoother_;

  std::string map_frame_;
  std::string base_frame_;
};

#endif  // NAVYU_PLANNER__NAVYU_GLOBAL_PLANNER_HPP_
