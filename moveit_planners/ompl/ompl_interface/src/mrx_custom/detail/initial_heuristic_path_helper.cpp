#include <moveit/ompl_interface/mrx_custom/detail/initial_heuristic_path_helper.h>
#include <moveit/ompl_interface/model_based_planning_context.h>

namespace ompl_interface
{
InitialHeuristicPathHelper::InitialHeuristicPathHelper(const std::shared_ptr<StateValidityChecker>& checker,
                                                       const std::string& axis_name, const double eef_step,
                                                       const double max_length)
  : StateValidityChecker(checker->getPlanningContext()), eef_step_(eef_step)
{
  if (axis_name == "Z")
  {
    diff_transform_ = Eigen::Isometry3d::Identity();
    diff_transform_.translation() = Eigen::Vector3d{ 0, 0, -max_length };
  }
  else
  {
    throw std::runtime_error("Unknown axis_name = " + axis_name + ".");
  }
}

bool InitialHeuristicPathHelper::isLeafGroup() const
{
  const auto* rs = tss_.getStateStorage();
  return rs->getJointModelGroup(group_name_)->getSubgroupNames().empty();
}

std::vector<robot_state::RobotStatePtr>
InitialHeuristicPathHelper::getCartesianPath(const ompl::base::State* start_state) const
{
  ompl::base::State* tmp_state = planning_context_->getOMPLStateSpace()->allocState();
  auto traj = getCartesianPathImpl(tmp_state, start_state);
  planning_context_->getOMPLStateSpace()->freeState(tmp_state);
  return traj;
}

std::vector<robot_state::RobotStatePtr>
InitialHeuristicPathHelper::getCartesianPathImpl(ompl::base::State* tmp_state,
                                                 const ompl::base::State* start_state) const
{
  std::vector<robot_state::RobotStatePtr> traj;
  auto* start_rs = tss_.getStateStorage();
  const auto* jmg = start_rs->getJointModelGroup(group_name_);

  auto constraint_fn = [&](moveit::core::RobotState* tmp_rs, const moveit::core::JointModelGroup* group,
                           const double* ik_solution) {
    tmp_rs->setJointGroupPositions(group, ik_solution);
    tmp_rs->update();

    planning_context_->getOMPLStateSpace()->copyToOMPLState(tmp_state, *tmp_rs);

    return isValid(tmp_state);
  };

  const double jump_threshold = 0.0;

  planning_context_->getOMPLStateSpace()->copyToRobotState(*start_rs, start_state);
  const auto& link_name = jmg->getEndEffectorName();
  const auto* link_model = start_rs->getLinkModel(link_name);

  if (link_model == nullptr)
  {
    ROS_ERROR("group_name = %s has no end effector link.", group_name_.c_str());
    return traj;
  }

  const Eigen::Isometry3d pose = start_rs->getGlobalLinkTransform(link_name) * diff_transform_;

  const double result =
      moveit::core::CartesianInterpolator::computeCartesianPath(start_rs, jmg, traj, link_model, pose, true, eef_step_,
                                                                moveit::core::JumpThreshold(jump_threshold),
                                                                constraint_fn);

  return traj;
}
}  // namespace ompl_interface
