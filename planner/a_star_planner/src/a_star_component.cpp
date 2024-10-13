#include "a_star_planner/a_star_component.hpp"
#include "a_star_planner/smoother.hpp"

AStarGlobalPlanner::AStarGlobalPlanner(const rclcpp::NodeOptions & node_options)
: Node("navyu_global_planner", node_options)
{
  declare_parameter<std::string>("map_frame","map");
  declare_parameter<std::string>("base_frame","base_link");
  declare_parameter<double>("lethal_cost_threshold",30.0);
  get_parameter("map_frame",map_frame_);
  get_parameter("base_frame",base_frame_);
  declare_parameter<double>("displacement_threshold",5.0);
  const double displacement_threshold = get_parameter("displacement_threshold").as_double();
  smoother_ = std::make_shared<Smoother>(displacement_threshold);

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  goal_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 5,
    std::bind(&AStarGlobalPlanner::callback_goal_pose, this, std::placeholders::_1));
  costmap_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(10).transient_local().reliable().keep_last(1),
    std::bind(&AStarGlobalPlanner::callback_costmap, this, std::placeholders::_1));

  raw_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
    "raw_path", rclcpp::QoS(10).transient_local().reliable().keep_last(1));
  path_publisher_ = create_publisher<nav_msgs::msg::Path>(
    "path", rclcpp::QoS(10).transient_local().reliable().keep_last(1));

  goal_pose_server_ = this->create_service<chick_nav_msgs::srv::NavigateToGoal>(
    "waypoint",
    std::bind(&AStarGlobalPlanner::goal_server, this, std::placeholders::_1, std::placeholders::_2)
  );
}

AStarGlobalPlanner::~AStarGlobalPlanner()
{
}

bool AStarGlobalPlanner::plan(
  const geometry_msgs::msg::Pose start, const geometry_msgs::msg::Pose goal,
  std::vector<Node2D *> & path)
{
  int start_x, start_y;
  int goal_x, goal_y;
  planner_->convert_map_to_grid(start.position.x, start.position.y, start_x, start_y);
  planner_->convert_map_to_grid(goal.position.x, goal.position.y, goal_x, goal_y);

  Node2D * start_node = new Node2D(start_x, start_y, 0.0, NULL);
  start_node->set_grid_index(planner_->get_grid_index(start_x, start_y));

  Node2D * goal_node = new Node2D(goal_x, goal_y, 0.0, NULL);
  goal_node->set_grid_index(planner_->get_grid_index(goal_x, goal_y));

  return planner_->plan(start_node, goal_node, path);
}

void AStarGlobalPlanner::publish_path(std::vector<Node2D *> path)
{
  nav_msgs::msg::Path path_msgs;
  path_msgs.header.frame_id = map_frame_;
  path_msgs.header.stamp = now();

  for (auto & node : path) {
    geometry_msgs::msg::PoseStamped path_pose;
    path_pose.header = path_msgs.header;
    planner_->convert_grid_to_map(
      node->x_, node->y_, path_pose.pose.position.x, path_pose.pose.position.y);
    path_msgs.poses.emplace_back(path_pose);
  }
  // publish raw path
  raw_path_publisher_->publish(path_msgs);

  // publish smoothing path
  nav_msgs::msg::Path optimized_path = smoother_->smooth(path_msgs);
  optimized_path.header.frame_id = map_frame_;
  optimized_path.header.stamp = now();
  path_publisher_->publish(optimized_path);
}

void AStarGlobalPlanner::callback_goal_pose(const geometry_msgs::msg::PoseStamped & msg)
{
  if (planner_ == nullptr) {
    RCLCPP_ERROR_STREAM(get_logger(), "Planner is not Initialized.");
    return;
  }

  geometry_msgs::msg::Pose start;
  if (!get_robot_pose(map_frame_, base_frame_, tf_buffer_, start)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Can not get Robot Pose.");
    return;
  }

  geometry_msgs::msg::Pose goal = msg.pose;

  std::vector<Node2D *> path;
  if (plan(start, goal, path)) {
    publish_path(path);
  }
}

void AStarGlobalPlanner::callback_costmap(const nav_msgs::msg::OccupancyGrid & msg)
{
  if (planner_ == nullptr) {
    int size_x = msg.info.width;
    int size_y = msg.info.height;
    double resolution = msg.info.resolution;
    double origin_x = msg.info.origin.position.x;
    double origin_y = msg.info.origin.position.y;

    // initialize global planner method
    planner_ = std::make_shared<AstarPlanner>(size_x, size_y, resolution, origin_x, origin_y);
    planner_->set_costmap(msg.data);

    double lethal_cost_threshold;
    get_parameter("lethal_cost_threshold", lethal_cost_threshold);
    planner_->set_lethal_cost_threshold(lethal_cost_threshold);

    return;
  }

  // set costmap info
  planner_->set_origin(msg.info.origin.position.x, msg.info.origin.position.y);
  planner_->set_size(msg.info.width, msg.info.height);
  planner_->set_resolution(msg.info.resolution);
  planner_->set_costmap(msg.data);
}


void AStarGlobalPlanner::goal_server(
  const std::shared_ptr<chick_nav_msgs::srv::NavigateToGoal::Request> request,
  const std::shared_ptr<chick_nav_msgs::srv::NavigateToGoal::Response> response
)
{    
  if (planner_ == nullptr) {
    RCLCPP_ERROR_STREAM(get_logger(), "Planner is not Initialized.");
    return;
  }
  geometry_msgs::msg::Pose start ;
  if (!get_robot_pose(map_frame_, base_frame_, tf_buffer_, start)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Can not get Robot Pose.");
    response->goal_reached = false;
    return;
  }
  geometry_msgs::msg::Pose goal = request->pose.pose;
  std::vector<Node2D *> path;
  if (plan(start, goal, path)) {
    publish_path(path);  
    response->goal_reached = true;
  }else{
  response->goal_reached = true;
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(AStarGlobalPlanner)
