#ifndef CHICKEN_UTILS__QUATERNION_UTILS_HPP_
#define CHICKEN_UTILS__QUATERNION_UTILS_HPP_

#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

namespace quaternion_utils
{

geometry_msgs::msg::Quaternion convert_euler_to_quaternion(const geometry_msgs::msg::Vector3 euler);

geometry_msgs::msg::Vector3 convert_quaternion_to_euler(
  const geometry_msgs::msg::Quaternion quaternion);

geometry_msgs::msg::Quaternion rotation(
  const geometry_msgs::msg::Quaternion quat1, const geometry_msgs::msg::Quaternion quat2);

Eigen::Matrix3f get_rotation_matrix_from_euler(const geometry_msgs::msg::Vector3 euler);

}  // namespace quaternion_utils

#endif