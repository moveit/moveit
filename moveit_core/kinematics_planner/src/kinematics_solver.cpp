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

// ROS msgs
#include <kinematics_planner/kinematics_solver.h>
#include <planning_models/kinematic_model.h>
#include <planning_scene/planning_scene.h>

namespace kinematics_planner
{

bool KinematicsSolver::initialize(const planning_models::KinematicModelConstPtr &kinematic_model,
                                  const std::map<std::string, kinematics::KinematicsBasePtr> &kinematics_solver_map,
                                  const std::string &group_name)
{
  group_name_ = group_name;
  if(!kinematic_model->hasJointModelGroup(group_name))
  {
    ROS_ERROR("Group name: %s invalid",group_name.c_str());
    return false;
  }  

  /*const planning_models::KinematicModel::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_name_);
    group_names_ = joint_model_group->getSubgroupNames();
    for(unsigned int i=0; i < group_names_.size(); ++i)
    {
    ROS_INFO("Sub group names: %d %s",i,group_names_[i].c_str());
    }*/
  
  if(group_names_.empty())
    group_names_.push_back(group_name_);
  num_groups_= group_names_.size();

  ROS_INFO("Num groups: %d",num_groups_);

  for(unsigned int i=0; i < group_names_.size(); ++i)
  {
    if(kinematics_solver_map.find(group_names_[i]) == kinematics_solver_map.end())
    {
      ROS_WARN("No kinematics solver found for group %s",group_names_[i].c_str());      
      return false;      
    }    
    kinematics_solvers_.push_back(kinematics_solver_map.find(group_names_[i])->second);    
    kinematics_base_frames_.push_back(kinematics_solvers_.back()->getBaseFrame());    
    ROS_INFO("Adding group %s with base frame %s",group_names_[i].c_str(),kinematics_solvers_.back()->getBaseFrame().c_str());
    

    const std::vector<std::string> joint_names = kinematic_model->getJointModelGroup(group_names_[i])->getJointModelNames();
    for(unsigned int j=0; j < joint_names.size(); ++j)
      joint_names_.push_back(joint_names[j]);      
  }
  return true;  
}


bool KinematicsSolver::solve(const geometry_msgs::PoseStamped &pose,
                             const planning_scene::PlanningSceneConstPtr& planning_scene,
                             double timeout,
                             moveit_msgs::RobotState &robot_state,
                             moveit_msgs::MoveItErrorCodes &error_code,
                             const kinematic_constraints::KinematicConstraintSet& kinematic_constraint_set) const
{
  std::map<std::string,geometry_msgs::PoseStamped> goal;
  goal[group_names_.front()] = pose;
  return solve(goal,planning_scene,timeout,robot_state,error_code,kinematic_constraint_set);
}


bool KinematicsSolver::solve(const std::map<std::string,geometry_msgs::PoseStamped> &poses,
                             const planning_scene::PlanningSceneConstPtr& planning_scene,
                             double timeout,
                             moveit_msgs::RobotState &robot_state,
                             moveit_msgs::MoveItErrorCodes &error_code,
                             const kinematic_constraints::KinematicConstraintSet& kinematic_constraint_set) const
{
  ros::WallTime start_time = ros::WallTime::now();  
  if(!checkRequest(poses))
  {
    error_code.val = error_code.INVALID_GROUP_NAME;
    return false;    
  }

  planning_models::KinematicState kinematic_state = planning_scene->getCurrentState();
  std::vector<planning_models::KinematicState::JointStateGroup*> joint_state_groups(num_groups_);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.group_name = group_name_;
  
  for(unsigned int i=0; i < num_groups_; ++i)
    joint_state_groups[i] = kinematic_state.getJointStateGroup(group_names_[i]);    
  
  std::map<std::string,geometry_msgs::PoseStamped> start = transformPoses(planning_scene,kinematic_state,poses,kinematics_base_frames_);

  kinematics_planner::SolutionStateMap solutions;
  for(unsigned int i=0; i < num_groups_; ++i)
  {
    solutions[group_names_[i]].resize(joint_state_groups[i]->getVariableCount());    
  }
    
  while( (ros::WallTime::now()-start_time) <= ros::WallDuration(timeout))
  {
    bool success = true;    
    for(unsigned int i=0; i < num_groups_; ++i)
      joint_state_groups[i]->setToRandomValues();            
    for(unsigned int i=0; i < num_groups_; ++i)
    {
      std::vector<double> joint_state_values;
      joint_state_groups[i]->getGroupStateValues(joint_state_values);        
      const kinematics::KinematicsBaseConstPtr kinematics_solver = kinematics_solvers_[i];        
      if(!kinematics_solver->getPositionIK((start.find(group_names_[i])->second).pose,
                                           joint_state_values,
                                           solutions.find(group_names_[i])->second,
                                           error_code))
      {
        success = false;
        break;        
      }
      joint_state_groups[i]->setStateValues(solutions.find(group_names_[i])->second);        
    }      
    if(!success)
      continue;            

    planning_scene->checkCollision(collision_request,collision_result,kinematic_state);    
    if(collision_result.collision || !planning_scene->isStateConstrained(kinematic_state,kinematic_constraint_set))
      continue;      

    robot_state = getRobotState(solutions);      
    return true;
  }
  return false;    
}

std::map<std::string,geometry_msgs::PoseStamped> KinematicsSolver::transformPoses(const planning_scene::PlanningSceneConstPtr& planning_scene, 
                                                                                  const planning_models::KinematicState &kinematic_state,
                                                                                  const std::map<std::string,geometry_msgs::PoseStamped> &poses,
                                                                                  const std::vector<std::string> &target_frames) const
{
  Eigen::Affine3d eigen_pose, eigen_pose_2;
  std::map<std::string,geometry_msgs::PoseStamped> result;  
  for(unsigned int i=0; i < group_names_.size(); ++i)
  {
    std::string target_frame = target_frames[i];   
    bool target_frame_is_root_frame = (target_frame == kinematic_state.getKinematicModel()->getModelFrame());  
    geometry_msgs::PoseStamped pose_stamped = poses.find(group_names_[i])->second;    
    planning_models::poseFromMsg(pose_stamped.pose,eigen_pose_2);
    planning_scene->getTransforms()->transformPose(kinematic_state,pose_stamped.header.frame_id,eigen_pose_2,eigen_pose);
    if(!target_frame_is_root_frame)
    {
      eigen_pose_2 = planning_scene->getTransforms()->getTransform(target_frame);
      eigen_pose = eigen_pose_2.inverse()*eigen_pose;
    }    
    pose_stamped.header.frame_id = target_frame;
    planning_models::msgFromPose(eigen_pose,pose_stamped.pose);    
    result[group_names_[i]] = pose_stamped;    
  }
  return result;  
}

std::map<std::string,geometry_msgs::PoseStamped> KinematicsSolver::transformPoses(const planning_scene::PlanningSceneConstPtr& planning_scene, 
                                                                                  const planning_models::KinematicState &kinematic_state,
                                                                                  const std::map<std::string,geometry_msgs::PoseStamped> &poses,
                                                                                  const std::string &target_frame) const
{
  std::vector<std::string> target_frames;
  for(unsigned int i=0; i < group_names_.size(); ++i)
    target_frames.push_back(target_frame);  
  return transformPoses(planning_scene,kinematic_state,poses,target_frames);  
}

moveit_msgs::RobotState KinematicsSolver::getRobotState(const kinematics_planner::SolutionStateMap &solutions) const
{
  moveit_msgs::RobotState robot_state;
  robot_state.joint_state.name = joint_names_;
  for(unsigned int i=0; i < num_groups_; ++i)
  {
    const std::vector<double>& group_solutions = (solutions.find(group_names_[i])->second);      
    robot_state.joint_state.position.insert(robot_state.joint_state.position.end(),group_solutions.begin(),group_solutions.end());          
  }   
  return robot_state;  
}

bool KinematicsSolver::checkRequest(const std::map<std::string,geometry_msgs::PoseStamped> &start) const
{
  for(unsigned int i=0; i < group_names_.size(); ++i)
    if(start.find(group_names_[i]) == start.end())
      return false;
  return true;  
}

std::vector<double> KinematicsSolver::getFloatingJointValues(const geometry_msgs::Pose &pose) const
{
  std::vector<double> values(7);
  values[0] = pose.position.x;
  values[1] = pose.position.y;
  values[2] = pose.position.z;
  
  values[3] = pose.orientation.x;
  values[4] = pose.orientation.y;
  values[5] = pose.orientation.z;
  values[6] = pose.orientation.w;
  return values;  
}

geometry_msgs::Pose KinematicsSolver::getPose(const std::vector<double> &values) const
{
  geometry_msgs::Pose pose;
  pose.position.x = values[0];
  pose.position.y = values[1];
  pose.position.z = values[2];

  pose.orientation.x = values[3];
  pose.orientation.y = values[4];
  pose.orientation.z = values[5];
  pose.orientation.w = values[6];
  
  return pose;  
}

}
