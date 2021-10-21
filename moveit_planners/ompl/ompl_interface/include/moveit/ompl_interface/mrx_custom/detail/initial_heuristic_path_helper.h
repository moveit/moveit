#pragma once

#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/robot_state/cartesian_interpolator.h>

namespace ompl_interface
{
class InitialHeuristicPathHelper : public StateValidityChecker
{
public:
  InitialHeuristicPathHelper(const std::shared_ptr<StateValidityChecker>& checker, const std::string& axis_name,
                             const double eef_step, const double max_length);

  // MRX custom
  bool isLeafGroup() const;

  const std::string& getGroupName() const
  {
    return group_name_;
  }

  std::vector<robot_state::RobotStatePtr> getCartesianPath(const ompl::base::State* start_state) const;

private:
  std::vector<robot_state::RobotStatePtr> getCartesianPathImpl(ompl::base::State* tmp_state,
                                                               const ompl::base::State* start_state) const;

  moveit::core::MaxEEFStep eef_step_;
  Eigen::Isometry3d diff_transform_;
};
}  // namespace ompl_interface
