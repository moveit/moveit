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

#include <moveit/pick_place/translation_stage.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_state/conversions.h>
#include <ros/console.h>

namespace pick_place
{

TranslationStage::TranslationStage(const planning_scene::PlanningSceneConstPtr &scene,
                                   const planning_pipeline::PlanningPipelinePtr &planning_pipeline,
                                   const constraint_samplers::ConstraintSamplerManagerPtr &constraints_sampler_manager,
                                   unsigned int nthreads) :
  ManipulationStage(nthreads),
  planning_scene_(scene),
  planning_pipeline_(planning_pipeline),
  constraints_sampler_manager_(constraints_sampler_manager),
  max_translation_segment_length_(0.01),
  max_translation_segments_(10)
{
  name_ = "translation";
}

bool TranslationStage::isStateCollisionFree(const sensor_msgs::JointState *post_grasp_posture, 
                                            kinematic_state::JointStateGroup *joint_state_group,
                                            const std::vector<double> &joint_group_variable_values) const
{
  joint_state_group->setVariableValues(joint_group_variable_values);
  // apply post-grasp pose for the end effector (we always apply it here since it could be the case the sampler changes this posture)
  joint_state_group->getKinematicState()->setStateValues(*post_grasp_posture);
  return !planning_scene_->isStateColliding(*joint_state_group->getKinematicState(), joint_state_group->getName());
}

bool TranslationStage::tryDistance(const ManipulationPlanPtr &plan, double dist) const
{
  plan->translated_goal_constraints_ = plan->goal_constraints_;
  
  // apply the desired translation distance
  plan->translated_goal_constraints_.position_constraints[0].target_point_offset.x -= dist * plan->grasp_.translation_direction.x;
  plan->translated_goal_constraints_.position_constraints[0].target_point_offset.y -= dist * plan->grasp_.translation_direction.y;
  plan->translated_goal_constraints_.position_constraints[0].target_point_offset.z -= dist * plan->grasp_.translation_direction.z;
  
  // construct a sampler for the specified constraints; this can end up calling just IK, but it is more general
  // and allows for robot-specific samplers, producing samples that also change the base position if needed, etc
  constraint_samplers::ConstraintSamplerPtr desired_translated_goal_sampler =
    constraints_sampler_manager_->selectSampler(planning_scene_, plan->planning_group_, plan->translated_goal_constraints_);
  
  if (desired_translated_goal_sampler)
  {
    desired_translated_goal_sampler->setStateValidityCallback(boost::bind(&TranslationStage::isStateCollisionFree, this, &plan->grasp_.grasp_posture, _1, _2));
    
    // initialize with scene state 
    if (!plan->token_translated_state_)
      plan->token_translated_state_.reset(new kinematic_state::KinematicState(planning_scene_->getCurrentState()));
    
    if (desired_translated_goal_sampler->sample(plan->token_translated_state_->getJointStateGroup(plan->planning_group_),
                                                *plan->token_translated_state_,
                                                planning_scene_->getKinematicModel()->getJointModelGroup(plan->planning_group_)->getDefaultIKAttempts()))
    {
      plan->translated_goal_sampler_.swap(desired_translated_goal_sampler);
      return true;
    }
  }
  else
    ROS_ERROR_THROTTLE(1, "No sampler was constructed");
  return false;
}

bool TranslationStage::tryTranslation(const ManipulationPlanPtr &plan, double dist) const
{
  if (tryDistance(plan, dist))
  {
    moveit_msgs::GetMotionPlan::Request req;
    moveit_msgs::GetMotionPlan::Response res;
    req.motion_plan_request.group_name = plan->planning_group_;
    req.motion_plan_request.num_planning_attempts = 1;
    req.motion_plan_request.allowed_planning_time = ros::Duration((plan->timeout_ - ros::WallTime::now()).toSec() * 0.5);
    
    // set start state
    trajectory_processing::robotTrajectoryPointToRobotState(plan->trajectories_.back(), trajectory_processing::trajectoryPointCount(plan->trajectories_.back()) - 1,
                                                            req.motion_plan_request.start_state);
    
    // construct the translation motion requests for desired translation distance
    unsigned int steps = std::min(max_translation_segments_, (unsigned int)floor(dist / max_translation_segment_length_ + 0.5));
    ROS_DEBUG("Using %u steps in translation", steps);
    
    req.motion_plan_request.trajectory_constraints.constraints.resize(steps);
    for (unsigned int i = 1 ; i <= steps ; ++i)
    {
      moveit_msgs::Constraints &c = req.motion_plan_request.trajectory_constraints.constraints[i - 1];
      c = plan->goal_constraints_;
      double factor = dist * ((double)i / (double)steps);
      // apply the translation distance
      c.position_constraints[0].target_point_offset.x -= factor * plan->grasp_.translation_direction.x;
      c.position_constraints[0].target_point_offset.y -= factor * plan->grasp_.translation_direction.y;
      c.position_constraints[0].target_point_offset.z -= factor * plan->grasp_.translation_direction.z;
    }
    
    if (planning_pipeline_->generatePlan(planning_scene_, req, res) && res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      plan->trajectories_.push_back(res.trajectory);
      plan->trajectory_descriptions_.push_back(name_);  
      return true;
    }
  }
  return false;
}

bool TranslationStage::evaluate(unsigned int thread_id, const ManipulationPlanPtr &plan) const
{ 
  bool zero1 = false;
  if (fabs(plan->grasp_.desired_translation_distance) > 0.0)
  {
    if (tryTranslation(plan, plan->grasp_.desired_translation_distance))
      return true;
  }
  else
    zero1 = true;
  bool zero2 = false;
  if (fabs(plan->grasp_.min_translation_distance) > 0.0)
  {
    if (tryTranslation(plan, plan->grasp_.min_translation_distance))
      return true;
  }
  else
    zero2 = true;
  return zero1 && zero2;
}

}
