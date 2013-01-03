/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/pick_place/approach_stage.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <moveit/kinematic_constraints/utils.h>
#include <ros/console.h>

namespace pick_place
{

ApproachStage::ApproachStage(const planning_scene::PlanningSceneConstPtr &scene,
                             const planning_pipeline::PlanningPipelinePtr &planning_pipeline,
                             const constraint_samplers::ConstraintSamplerManagerPtr &constraints_sampler_manager,
                             unsigned int nthreads) :
  ManipulationStage(nthreads),
  planning_scene_(scene),
  planning_pipeline_(planning_pipeline),
  constraints_sampler_manager_(constraints_sampler_manager)
{
  name_ = "approach";
}

bool ApproachStage::isStateCollisionFree(kinematic_state::JointStateGroup *joint_state_group,
                                         const std::vector<double> &joint_group_variable_values) const
{
  joint_state_group->setVariableValues(joint_group_variable_values);
  return !planning_scene_->isStateColliding(*joint_state_group->getKinematicState(), joint_state_group->getName());
}

bool ApproachStage::evaluate(unsigned int thread_id, const ManipulationPlanPtr &plan) const
{ 
  moveit_msgs::GetMotionPlan::Request req;
  moveit_msgs::GetMotionPlan::Response res;
  req.motion_plan_request.group_name = plan->planning_group_;
  req.motion_plan_request.num_planning_attempts = 1;
  req.motion_plan_request.allowed_planning_time = ros::Duration((plan->timeout_ - ros::WallTime::now()).toSec() * 0.5);

  // construct the approach motion requests for desired approach distance
  unsigned int steps = (unsigned int)floor(plan->grasp_.desired_approach_distance / 0.01 + 0.5);
  req.motion_plan_request.trajectory_constraints.constraints.resize(steps);
  for (unsigned int i = 1 ; i < steps ; ++i)
  {
    moveit_msgs::Constraints &c = req.motion_plan_request.trajectory_constraints.constraints[i - 1];
    c = plan->goal_constraints_;
    double factor = plan->grasp_.desired_approach_distance * ((double)i / (double)steps);
    // apply the approach distance
    c.position_constraints[0].target_point_offset.x += factor * plan->grasp_.approach_direction.x;
    c.position_constraints[0].target_point_offset.y += factor * plan->grasp_.approach_direction.y;
    c.position_constraints[0].target_point_offset.z += factor * plan->grasp_.approach_direction.z;
  }
  req.motion_plan_request.trajectory_constraints.constraints.back() = plan->goal_constraints_;
  
  if (planning_pipeline_->generatePlan(planning_scene_, req, res) && res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    plan->trajectories_.push_back(res.trajectory);
    plan->trajectory_descriptions_.push_back(name_);
    return true;
  }
  
  return false;
}

}
