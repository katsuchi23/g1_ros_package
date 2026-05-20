#ifndef G1_NAVIGATION__TRANSLATE_OR_ROTATE_CRITIC_HPP_
#define G1_NAVIGATION__TRANSLATE_OR_ROTATE_CRITIC_HPP_

#include "dwb_core/trajectory_critic.hpp"

namespace g1_navigation
{

// Reject trajectories that command both translation (x/y) and rotation (theta) together.
class TranslateOrRotateCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

private:
  double deadband_{1e-3};
};

}  // namespace g1_navigation

#endif  // G1_NAVIGATION__TRANSLATE_OR_ROTATE_CRITIC_HPP_
