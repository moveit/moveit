/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Sachin Chitta
*********************************************************************/

// // ROS msgs
#include <moveit/kinematics_constraint_aware/kinematics_constraint_aware.h>
#include <moveit/kinematic_model/kinematic_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <eigen_conversions/eigen_msg.h>

namespace kinematics_constraint_aware
{

KinematicsConstraintAware::KinematicsConstraintAware(const kinematic_model::KinematicModelConstPtr &kinematic_model,
                                                     const std::string &group_name)
{
  if(!kinematic_model->hasJointModelGroup(group_name))
  {
    logError("The group %s does not exist",group_name.c_str());
    joint_model_group_ = NULL;
    return;    
  }  

  kinematic_model_ = kinematic_model;  
  group_name_ = group_name;
  joint_model_group_ = kinematic_model_->getJointModelGroup(group_name);
  if(joint_model_group_->getSolverInstance())
  {
    has_sub_groups_ = false;    
    sub_groups_names_.push_back(group_name_);
    kinematics_solvers_.push_back(joint_model_group_->getSolverInstance());
    kinematics_base_frames_.push_back(joint_model_group_->getSolverInstance()->getBaseFrame());    
  }   
  else
  {
    logDebug("No kinematics solver instance defined for group %s", group_name.c_str());
    bool is_solvable_group = true;
    if(!joint_model_group_->getSubgroupNames().empty())
    {
      const std::vector<std::string> sub_groups_names = joint_model_group_->getSubgroupNames();
      for(std::size_t i=0; i < sub_groups_names.size(); ++i)
      {
        if(!kinematic_model_->getJointModelGroup(sub_groups_names[i])->getSolverInstance())
        {
          is_solvable_group = false;
          break;
        }
        kinematics_solvers_.push_back(kinematic_model_->getJointModelGroup(sub_groups_names[i])->getSolverInstance());        
        kinematics_base_frames_.push_back(kinematic_model_->getJointModelGroup(sub_groups_names[i])->getSolverInstance()->getBaseFrame());
      }
      if(is_solvable_group)
      {
        logDebug("Group %s is a group for which we can solve IK", joint_model_group_->getName().c_str());
        sub_groups_names_ = sub_groups_names;        
      }
      else
      {
        joint_model_group_ = NULL;
        return;        
      }      
    }
    else
    {
      joint_model_group_ = NULL;
      logError("No solver allocated for group %s", group_name.c_str());
    }
    has_sub_groups_ = true;    
  }
}

bool KinematicsConstraintAware::getIK(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                      const kinematics_constraint_aware::KinematicsRequest &request,
                                      kinematics_constraint_aware::KinematicsResponse &response) const
{
  if(!joint_model_group_)
  {
    logError("This solver has not been constructed properly");
    return false;
  }
  
  ros::WallTime start_time = ros::WallTime::now();
  if(request.group_name_ != group_name_)
  {
    response.error_code_.val = response.error_code_.INVALID_GROUP_NAME;
    return false;
  }
  
  // Setup the seed and the values for all other joints in the robot
  kinematic_state::KinematicState kinematic_state = *request.robot_state_;

  // Get a joint state group for each sub-group that we are dealing with
  std::vector<kinematic_state::JointStateGroup*> joint_state_groups(sub_groups_names_.size());
  for(std::size_t i=0; i < sub_groups_names_.size(); ++i)
    joint_state_groups[i] = kinematic_state.getJointStateGroup(sub_groups_names_[i]);    

  // Transform request to tip frame if necessary
  if(!request.ik_link_names_.empty())
  {
    for(std::size_t i=0; i < request.pose_stamped_vector_.size(); ++i)
    {        
      geometry_msgs::PoseStamped tmp_pose = request.pose_stamped_vector_[i];        
      //The assumption is that this new link is rigidly attached to the tip link for the group
      if(!kinematic_model_->getJointModelGroup(sub_groups_names_[i])->hasLinkModel(request.ik_link_names_[i]) && 
         kinematic_model_->getJointModelGroup(sub_groups_names_[i])->isLinkUpdated(request.ik_link_names_[i]))
      {
        tmp_pose.pose = getTipFramePose(planning_scene,
                                        kinematic_state,
                                        request.pose_stamped_vector_[i].pose,
                                        request.ik_link_names_[i],
                                        i);       
      }      
      else if(!kinematic_model_->getJointModelGroup(sub_groups_names_[i])->canSetStateFromIK(request.ik_link_names_[i]))
      {
        logError("Could not find IK solver for link %s for group %s", request.ik_link_names_[i].c_str(), sub_groups_names_[i].c_str());
        return false;
      }
    }    
  }  
    
  // Transform the requests
  std::vector<geometry_msgs::PoseStamped> goals = transformPoses(planning_scene,
                                                                 kinematic_state,
                                                                 request.pose_stamped_vector_,
                                                                 kinematics_base_frames_);
  
  // Do the Inverse kinematics
  bool first_time = true;  
  while( (ros::WallTime::now()-start_time) <= ros::WallDuration(request.timeout_.toSec()))
  {
    bool success = true;    
    if(!first_time)
    {
      for(unsigned int i=0; i < sub_groups_names_.size(); ++i)
        joint_state_groups[i]->setToRandomValues();            
    }
    first_time = false;

    // Run through all sub-groups and try IK    
    for(unsigned int i=0; i < sub_groups_names_.size(); ++i)
    {
      std::vector<double> joint_state_values, solutions;
      joint_state_groups[i]->getVariableValues(joint_state_values);
      joint_state_groups[i]->getVariableValues(solutions);
      const kinematics::KinematicsBaseConstPtr kinematics_solver = kinematics_solvers_[i];
      geometry_msgs::Pose ik_pose = goals[i].pose;
      if(!kinematics_solver->getPositionIK(ik_pose,
                                           joint_state_values,
                                           solutions,
                                           response.error_code_))
      {
          logDebug("getPositionIK failed with error code %d", response.error_code_.val);
          success = false;
          break;
      }
      joint_state_groups[i]->setVariableValues(solutions);
    }      
    if(!success)
      continue;            

    // Now check for collisions
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;  
    collision_request.group_name = group_name_;
    planning_scene->checkCollision(collision_request, collision_result, kinematic_state);    
    if(collision_result.collision)
    {
      response.error_code_.val = response.error_code_.GOAL_IN_COLLISION;
      continue;      
    }    
 
    // Now check for constraints
    kinematic_constraints::ConstraintEvaluationResult constraint_result;
    constraint_result = request.constraints_->decide(kinematic_state, response.constraint_eval_results_);
    if(!constraint_result.satisfied)
    {
      response.error_code_.val = response.error_code_.GOAL_VIOLATES_PATH_CONSTRAINTS;
      continue;
    }
    // We are good
    std::vector<double> solution_values;
    kinematic_state.getJointStateGroup(group_name_)->getVariableValues(solution_values);
    response.solution_->getJointStateGroup(group_name_)->setVariableValues(solution_values);
    return true;
  }
  return false;  
}

bool KinematicsConstraintAware::getIK(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                      const moveit_msgs::GetConstraintAwarePositionIK::Request &request,
                                      moveit_msgs::GetConstraintAwarePositionIK::Response &response) const
{
  if(!joint_model_group_)
  {
    logError("This solver has not been constructed properly");
    return false;
  }

  ros::WallTime start_time = ros::WallTime::now();
  kinematics_constraint_aware::KinematicsRequest kinematics_request;
  kinematics_constraint_aware::KinematicsResponse kinematics_response;

  if(!convertServiceRequest(planning_scene, request, kinematics_request, kinematics_response))
  {
    response.error_code = kinematics_response.error_code_;
    return false;    
  }  

  bool result = getIK(planning_scene, kinematics_request, kinematics_response);
  response.error_code = kinematics_response.error_code_;
  kinematics_response.solution_->setStateValues(response.solution.joint_state);
  return result;
}

bool KinematicsConstraintAware::convertServiceRequest(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                                      const moveit_msgs::GetConstraintAwarePositionIK::Request &request,
                                                      kinematics_constraint_aware::KinematicsRequest &kinematics_request,
                                                      kinematics_constraint_aware::KinematicsResponse &kinematics_response) const
{
  if(request.ik_request.group_name != group_name_)
  {
    logError("This kinematics solver does not support requests for group: %s", request.ik_request.group_name.c_str());    
    kinematics_response.error_code_.val = kinematics_response.error_code_.INVALID_GROUP_NAME;    
    return false;
  }

  if(request.ik_request.pose_stamped_vector.size() != sub_groups_names_.size())
  {
    logError("Number of poses in request: %d must match number of sub groups %d in this group", 
             request.ik_request.pose_stamped_vector.size(),
             sub_groups_names_.size());
    kinematics_response.error_code_.val = kinematics_response.error_code_.INVALID_GROUP_NAME;    
    return false;      
  }
  if(!request.ik_request.ik_link_names.empty() && request.ik_request.ik_link_names.size() != sub_groups_names_.size())
  {
    logError("Number of ik_link_names in request: %d must match number of sub groups %d in this group or must be zero", 
             request.ik_request.ik_link_names.size(),
             sub_groups_names_.size());
    kinematics_response.error_code_.val = kinematics_response.error_code_.INVALID_GROUP_NAME;    
    return false;            
  }

  kinematics_request.ik_link_names_ = request.ik_request.ik_link_names;
  kinematics_request.pose_stamped_vector_ = request.ik_request.pose_stamped_vector;      
  kinematics_request.robot_state_.reset(new kinematic_state::KinematicState(planning_scene->getCurrentState()));
  kinematics_request.robot_state_->setStateValues(request.ik_request.robot_state.joint_state);    
  kinematics_request.constraints_.reset(new kinematic_constraints::KinematicConstraintSet(kinematic_model_, planning_scene->getTransforms()));
  kinematics_request.constraints_->add(request.constraints);
  kinematics_request.timeout_ = request.timeout;
  kinematics_request.group_name_ = request.ik_request.group_name;
  return true;  
}


std::vector<geometry_msgs::PoseStamped> KinematicsConstraintAware::transformPoses(const planning_scene::PlanningSceneConstPtr& planning_scene, 
                                                                                  const kinematic_state::KinematicState &kinematic_state,
                                                                                  const std::vector<geometry_msgs::PoseStamped> &poses,
                                                                                  const std::vector<std::string> &target_frames) const
{
  Eigen::Affine3d eigen_pose, eigen_pose_2;
  std::vector<geometry_msgs::PoseStamped> result(sub_groups_names_.size());  
  for(std::size_t i = 0; i <= poses.size(); ++i)
  {    
    bool target_frame_is_root_frame = (target_frames[i] == kinematic_state.getKinematicModel()->getModelFrame());  
    geometry_msgs::PoseStamped pose_stamped = poses[i];    
    tf::poseMsgToEigen(pose_stamped.pose, eigen_pose_2);
    planning_scene->getTransforms()->transformPose(kinematic_state, pose_stamped.header.frame_id, eigen_pose_2, eigen_pose);
    if(!target_frame_is_root_frame)
    {
      eigen_pose_2 = planning_scene->getTransforms()->getTransform(kinematic_state, target_frames[i]);
      eigen_pose = eigen_pose_2.inverse()*eigen_pose;
    }    
    pose_stamped.header.frame_id = target_frames[i];
    tf::poseEigenToMsg(eigen_pose, pose_stamped.pose);
    result[i] = pose_stamped;    
  }
  return result;  
}

geometry_msgs::Pose KinematicsConstraintAware::getTipFramePose(const planning_scene::PlanningSceneConstPtr& planning_scene, 
                                                               const kinematic_state::KinematicState &kinematic_state,
                                                               const geometry_msgs::Pose &pose,
                                                               const std::string &link_name,
                                                               unsigned int sub_group_index) const
{
  geometry_msgs::Pose result;  
  Eigen::Affine3d eigen_pose_in, eigen_pose_link, eigen_pose_tip;
  std::string tip_name = kinematics_solvers_[sub_group_index]->getTipFrame();  
  tf::poseMsgToEigen(pose, eigen_pose_in);
  eigen_pose_link = planning_scene->getTransforms()->getTransform(kinematic_state, link_name);
  eigen_pose_tip = planning_scene->getTransforms()->getTransform(kinematic_state, tip_name);
  eigen_pose_in = eigen_pose_in*(eigen_pose_link.inverse()*eigen_pose_tip);
  tf::poseEigenToMsg(eigen_pose_in, result);
  return result;  
}

}
