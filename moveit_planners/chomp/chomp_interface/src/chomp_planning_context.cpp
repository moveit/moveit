/*
 * chomp_planning_context.cpp
 *
 *  Created on: 27-Jul-2016
 *      Author: ace
 */

#include <chomp_interface/chomp_planning_context.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>

namespace chomp_interface
{
CHOMPPlanningContext::CHOMPPlanningContext(const std::string& name, const std::string& group,
                                           const robot_model::RobotModelConstPtr& model)
  : planning_interface::PlanningContext(name, group), robot_model_(model)
{
  chomp_interface_ = CHOMPInterfacePtr(new CHOMPInterface());
}

CHOMPPlanningContext::~CHOMPPlanningContext()
{
}

bool CHOMPPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  moveit_msgs::MotionPlanDetailedResponse res2;
  if (chomp_interface_->solve(planning_scene_, request_, chomp_interface_->getParams(), res2))
  {
    res.trajectory_.resize(1);
    res.trajectory_[0] =
        robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model_, getGroupName()));

    moveit::core::RobotState start_state(robot_model_);
    robot_state::robotStateMsgToRobotState(res2.trajectory_start, start_state);
    res.trajectory_[0]->setRobotTrajectoryMsg(start_state, res2.trajectory[0]);

    trajectory_processing::IterativeParabolicTimeParameterization itp;
    itp.computeTimeStamps(*res.trajectory_[0], request_.max_velocity_scaling_factor,
                          request_.max_acceleration_scaling_factor);

    res.description_.push_back("plan");
    res.processing_time_ = res2.processing_time;
    res.error_code_ = res2.error_code;
    return true;
  }
  else
  {
    res.error_code_ = res2.error_code;
    return false;
  }
}

bool CHOMPPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  planning_interface::MotionPlanDetailedResponse res_detailed;
  bool planning_success = solve(res_detailed);

  res.error_code_ = res_detailed.error_code_;

  if (planning_success)
  {
    res.trajectory_ = res_detailed.trajectory_[0];
    res.planning_time_ = res_detailed.processing_time_[0];
  }

  return planning_success;
}

bool CHOMPPlanningContext::terminate()
{
  // TODO - make interruptible
  return true;
}

void CHOMPPlanningContext::clear()
{
}

} /* namespace chomp_interface */
