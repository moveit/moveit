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

#include <moveit/pick_place/reachable_valid_grasp_filter.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/bind.hpp>
#include <ros/console.h>

namespace pick_place
{

ReachableAndValidGraspFilter::ReachableAndValidGraspFilter(const planning_scene::PlanningSceneConstPtr &scene,
                                                           const collision_detection::AllowedCollisionMatrixConstPtr &collision_matrix,
                                                           const constraint_samplers::ConstraintSamplerManagerPtr &constraints_sampler_manager) :
  ManipulationStage("reachable & valid grasp filter"),
  planning_scene_(scene),
  collision_matrix_(collision_matrix),
  constraints_sampler_manager_(constraints_sampler_manager)
{
}

bool ReachableAndValidGraspFilter::isStateCollisionFree(const ManipulationPlan *manipulation_plan,
                                                        robot_state::JointStateGroup *joint_state_group,
                                                        const std::vector<double> &joint_group_variable_values) const
{
  joint_state_group->setVariableValues(joint_group_variable_values);  
  // apply pre-grasp pose for the end effector (we always apply it here since it could be the case the sampler changes this posture)
  joint_state_group->getRobotState()->setStateValues(manipulation_plan->grasp_.pre_grasp_posture);
  
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = manipulation_plan->planning_group_;
  planning_scene_->checkCollision(req, res, *joint_state_group->getRobotState(), *collision_matrix_);
  if (res.collision == false)
    return planning_scene_->isStateFeasible(*joint_state_group->getRobotState());
  else
    return false;  
}

bool ReachableAndValidGraspFilter::isEndEffectorFree(const ManipulationPlanPtr &plan, robot_state::RobotState &token_state) const
{
  Eigen::Affine3d eigen_pose;
  tf::poseMsgToEigen(plan->grasp_.grasp_pose, eigen_pose);
  token_state.updateStateWithLinkAt(plan->ik_link_name_, eigen_pose);
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = plan->end_effector_group_;
  planning_scene_->checkCollision(req, res, token_state, *collision_matrix_);
  return res.collision == false;
}

bool ReachableAndValidGraspFilter::evaluate(const ManipulationPlanPtr &plan) const
{   
  // initialize with scene state 
  robot_state::RobotStatePtr token_state(new robot_state::RobotState(planning_scene_->getCurrentState()));
  if (isEndEffectorFree(plan, *token_state))
  {
    geometry_msgs::PoseStamped pose;
    pose.header = plan->grasp_.header;
    pose.pose = plan->grasp_.grasp_pose;
    
    // convert the pose we want to reach to a set of constraints
    plan->goal_constraints_ = kinematic_constraints::constructGoalConstraints(plan->ik_link_name_, pose);
    
    // construct a sampler for the specified constraints; this can end up calling just IK, but it is more general
    // and allows for robot-specific samplers, producing samples that also change the base position if needed, etc
    plan->goal_sampler_ = constraints_sampler_manager_->selectSampler(planning_scene_, plan->planning_group_, plan->goal_constraints_);
    if (plan->goal_sampler_)
    {
      plan->goal_sampler_->setStateValidityCallback(boost::bind(&ReachableAndValidGraspFilter::isStateCollisionFree, this, plan.get(), _1, _2));
      plan->sampling_attempts_ = std::max(1u, planning_scene_->getRobotModel()->getJointModelGroup(plan->planning_group_)->getDefaultIKAttempts());
      
      if (plan->goal_sampler_->sample(token_state->getJointStateGroup(plan->planning_group_), *token_state, plan->sampling_attempts_))
      {
        plan->possible_goal_states_.push_back(token_state);
        return true;
      }
    }
    else
      ROS_ERROR_THROTTLE(1, "No sampler was constructed");
  }  
  plan->error_code_.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
  return false;
}

}
