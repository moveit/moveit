/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PLANNING_CONTEXT_BASE_H
#define PLANNING_CONTEXT_BASE_H

#include "pilz_trajectory_generation/joint_limits_container.h"
#include "pilz_trajectory_generation/trajectory_generator.h"

#include <ros/ros.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <atomic>
#include <thread>

namespace pilz {

/**
 * @brief PlanningContext for obtaining trajectories
 */
template <typename GeneratorT>
class PlanningContextBase : public planning_interface::PlanningContext
{
public:

  PlanningContextBase<GeneratorT>(const std::string& name,
                     const std::string& group,
                     const moveit::core::RobotModelConstPtr& model,
                     const pilz::LimitsContainer& limits):
  planning_interface::PlanningContext(name, group),
  terminated_(false),
  model_(model),
  limits_(limits),
  generator_(model, limits_){}

  virtual ~PlanningContextBase() {}

  /**
   * @brief Calculates a trajectory for the request this context is currently set for
   * @param res The result containing the respective trajectory, or error_code on failure
   * @return true on success, false otherwise
   */
  virtual bool solve(planning_interface::MotionPlanResponse& res) override;

  /**
   * @brief Will return the same trajectory as solve(planning_interface::MotionPlanResponse& res)
   * This function just delegates to the common response however here the same trajectory is stored with the
   * descriptions "plan", "simplify", "interpolate"
   * @param res The detailed response
   * @return true on success, false otherwise
   */
  virtual bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  /**
   * @brief Will terminate solve()
   * @return
   * @note Currently will not stop a runnning solve but
   */
  virtual bool terminate() override;

  /**
   * @copydoc planning_interface::PlanningContext::clear()
   */
  virtual void clear() override;

  /// Flag if terminated
  std::atomic_bool terminated_;

  /// The robot model
  robot_model::RobotModelConstPtr model_;

  /// Joint limits to be used during planning
  pilz::LimitsContainer limits_;

protected:
  GeneratorT generator_;

};


template <typename GeneratorT>
bool pilz::PlanningContextBase<GeneratorT>::solve(planning_interface::MotionPlanResponse &res)
{
  if(!terminated_)
  {
    // Use current state as start state if not set
    if(request_.start_state.joint_state.name.size() == 0)
    {
      moveit_msgs::RobotState currentState;
      moveit::core::robotStateToRobotStateMsg(getPlanningScene()->getCurrentState(), currentState);
      request_.start_state = currentState;
    }
    bool result = generator_.generate(request_, res);
    return result;
    //res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    //return false; // TODO
  }

  ROS_ERROR("Using solve on a terminated planning context!");
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
  return false;
}


template <typename GeneratorT>
bool pilz::PlanningContextBase<GeneratorT>::solve(planning_interface::MotionPlanDetailedResponse &res)
{
   // delegate to regular response
   planning_interface::MotionPlanResponse undetailed_response;
   bool result = solve(undetailed_response);

   res.description_.push_back("plan");
   res.trajectory_.push_back(undetailed_response.trajectory_);
   res.processing_time_.push_back(undetailed_response.planning_time_);

   res.description_.push_back("simplify");
   res.trajectory_.push_back(undetailed_response.trajectory_);
   res.processing_time_.push_back(0);

   res.description_.push_back("interpolate");
   res.trajectory_.push_back(undetailed_response.trajectory_);
   res.processing_time_.push_back(0);

   res.error_code_ = undetailed_response.error_code_;
   return result;
}


template <typename GeneratorT>
bool pilz::PlanningContextBase<GeneratorT>::terminate()
{
  ROS_DEBUG_STREAM("Terminate called");
  terminated_ = true;
  return true;
}


template <typename GeneratorT>
void pilz::PlanningContextBase<GeneratorT>::clear()
{
  // No structures that need cleaning
  return;
}


} // namespace


#endif // PLANNING_CONTEXT_BASE_H
