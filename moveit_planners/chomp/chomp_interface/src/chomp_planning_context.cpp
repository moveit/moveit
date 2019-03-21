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

bool CHOMPPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  return chomp_interface_->solve(planning_scene_, request_, chomp_interface_->getParams(), res);
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
