#include "g1_navigation/translate_or_rotate_critic.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace g1_navigation
{

void TranslateOrRotateCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TranslateOrRotateCritic failed to lock lifecycle node");
  }

  const std::string param_prefix = dwb_plugin_name_ + "." + name_;
  if (!node->has_parameter(param_prefix + ".deadband")) {
    node->declare_parameter(param_prefix + ".deadband", rclcpp::ParameterValue(1e-3));
  }
  node->get_parameter(param_prefix + ".deadband", deadband_);
}

double TranslateOrRotateCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  const double linear_speed = std::hypot(traj.velocity.x, traj.velocity.y);
  const bool has_translation = linear_speed > deadband_;
  const bool has_rotation = std::fabs(traj.velocity.theta) > deadband_;

  // DWB treats negative scores as invalid trajectories.
  if (has_translation && has_rotation) {
    return -1.0;
  }

  return 0.0;
}

}  // namespace g1_navigation

PLUGINLIB_EXPORT_CLASS(g1_navigation::TranslateOrRotateCritic, dwb_core::TrajectoryCritic)
