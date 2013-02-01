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
#include <moveit/kinematics_planner/kinematics_solver.h>
#include <moveit/kinematic_model/kinematic_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <eigen_conversions/eigen_msg.h>

namespace kinematics_planner
{

bool KinematicsSolver::initialize(const kinematic_model::KinematicModelConstPtr &kinematic_model)
{
  kinematic_model_ = kinematic_model;  
  const std::map<std::string, kinematic_model::JointModelGroup*> joint_model_group_map = kinematic_model_->getJointModelGroupMap();
  for(std::map<std::string, kinematic_model::JointModelGroup*>::const_iterator iter = joint_model_group_map.begin();
      iter != joint_model_group_map.end(); ++iter)
  {
    if(iter->second->getSolverInstance())
    {
      std::vector<std::string> sub_groups;
      sub_groups.push_back(iter->first);
      kinematics_solver_map_.insert(std::pair<std::string,kinematics::KinematicsBaseConstPtr> (iter->first,iter->second->getSolverInstance()));
      group_map_.insert(std::pair<std::string,std::vector<std::string> > (iter->first,sub_groups));
    }   
    else
    {
      // ROS_DEBUG("No kinematics solver instance defined for group %s",iter->first.c_str());
      bool is_solvable_group = true;
      if(!iter->second->getSubgroupNames().empty())
      {
        const std::vector<std::string> sub_group_names = iter->second->getSubgroupNames();
        for(unsigned int i=0; i < sub_group_names.size(); ++i)
        {
          if(!kinematic_model->getJointModelGroup(sub_group_names[i])->getSolverInstance())
          {
            is_solvable_group = false;
            break;
          }
        }
        if(is_solvable_group)
        {
          logDebug("Group %s is a group for which we can solve IK",iter->first.c_str());
          group_map_.insert(std::pair<std::string,std::vector<std::string> >(iter->first,sub_group_names));
        }
      }
    }
  }
  return true;
}

bool KinematicsSolver::getIK(const planning_scene::PlanningSceneConstPtr &planning_scene,
                             const moveit_msgs::GetConstraintAwarePositionIK::Request &request,
                             moveit_msgs::GetConstraintAwarePositionIK::Response &response) const
{
  ros::WallTime start_time = ros::WallTime::now();
  if(group_map_.find(request.ik_request.group_name) == group_map_.end())
  {
    // ROS_ERROR("Could not find group %s",request.ik_request.group_name.c_str());
    response.error_code.val = response.error_code.INVALID_GROUP_NAME;
    return false;
  }

  // Setup the seed and the values for all other joints in the robot
  robot_state::RobotState kinematic_state = planning_scene->getCurrentState();
  kinematic_state.setStateValues(request.ik_request.robot_state.joint_state);

  // Get the set of sub-groups
  std::vector<std::string> group_names = group_map_.find(request.ik_request.group_name)->second;
  std::vector<kinematics::KinematicsBaseConstPtr> kinematics_solvers = getKinematicsSolvers(group_names);
  std::map<std::string,std::string> kinematics_base_frames;
  for(unsigned int i=0; i < kinematics_solvers.size(); ++i)
  {
    kinematics_base_frames.insert(std::pair<std::string,std::string> (group_names[i],kinematics_solvers[i]->getBaseFrame()));
  }

  // Get goals (for multi-group as well)
  std::map<std::string,geometry_msgs::PoseStamped> goals;
  if(!getGoal(planning_scene,kinematic_state,request,goals))
  {
    // ROS_ERROR("Request is invalid");
    response.error_code.val = response.error_code.GOAL_IN_COLLISION;
    return false;
  }

  // Get a joint state group for each sub-group that we are dealing with
  std::vector<robot_state::JointStateGroup*> joint_state_groups(group_names.size());
  for(unsigned int i=0; i < group_names.size(); ++i)
    joint_state_groups[i] = kinematic_state.getJointStateGroup(group_names[i]);

  // Create a collision request
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.group_name = request.ik_request.group_name;
    
  // Pre-allocate the solution vector
  kinematics_planner::SolutionStateMap solutions;
  for(unsigned int i=0; i < group_names.size(); ++i)
  {
    solutions[group_names[i]].resize(joint_state_groups[i]->getVariableCount());    
  }
    
  // Transform the requests
  std::map<std::string,geometry_msgs::PoseStamped> start = transformPoses(planning_scene,kinematic_state,goals,kinematics_base_frames);
  
  // Do IK 
  bool first_time = true;  
  while( (ros::WallTime::now()-start_time) <= ros::WallDuration(request.timeout.toSec()))
  {
    bool success = true;    
    if(!first_time)
    {
      for(unsigned int i=0; i < group_names.size(); ++i)
        joint_state_groups[i]->setToRandomValues();            
    }
    first_time = false;

    // Run through all sub-groups and try IK    
    for(unsigned int i=0; i < group_names.size(); ++i)
    {
      std::vector<double> joint_state_values;
      joint_state_groups[i]->getVariableValues(joint_state_values);
      const kinematics::KinematicsBaseConstPtr kinematics_solver = kinematics_solvers[i];        
      if(!kinematics_solver->getPositionIK((start.find(group_names[i])->second).pose,
                                           joint_state_values,
                                           solutions.find(group_names[i])->second,
                                           response.error_code))
      {
          // ROS_INFO("getPositionIK fialed with error code %d", response.error_code.val);
          success = false;
          break;
      }
      joint_state_groups[i]->setVariableValues(solutions.find(group_names[i])->second);
    }      
    if(!success)
      continue;            

    // Now check for collisions and constraints
    planning_scene->checkCollision(collision_request,collision_result,kinematic_state);    
    if(collision_result.collision)
    {
      response.error_code.val = response.error_code.GOAL_IN_COLLISION;      
      continue;      
    }    
    if(!planning_scene->isStateConstrained(kinematic_state,request.constraints))
    {
      response.error_code.val = response.error_code.GOAL_VIOLATES_PATH_CONSTRAINTS;
      continue;
    }
    // We are good
    response.solution = getRobotState(solutions,group_names);      
    return true;
  }
  return false;
}


bool KinematicsSolver::getGoal(const planning_scene::PlanningSceneConstPtr &planning_scene,
                               const robot_state::RobotState &kinematic_state,
                               const moveit_msgs::GetConstraintAwarePositionIK::Request &request,
                               std::map<std::string,geometry_msgs::PoseStamped>& pose_stamped) const
{
  if(group_map_.find(request.ik_request.group_name)->second.size() == 1)
  {
    if(request.ik_request.ik_link_name.empty())
    {
      pose_stamped.insert(std::pair<std::string,geometry_msgs::PoseStamped> (request.ik_request.group_name,request.ik_request.pose_stamped));
    }
    else
    {
      //The assumption is that this new link is rigidly attached to the tip link for the group
      if(!kinematic_model_->getJointModelGroup(request.ik_request.group_name)->hasLinkModel(request.ik_request.ik_link_name) && 
         kinematic_model_->getJointModelGroup(request.ik_request.group_name)->isLinkUpdated(request.ik_request.ik_link_name))
      {
        geometry_msgs::PoseStamped tmp_pose = request.ik_request.pose_stamped;        
        tmp_pose.pose = getTipFramePose(planning_scene,kinematic_state,request.ik_request.pose_stamped.pose,request.ik_request.ik_link_name,request.ik_request.group_name);       
        pose_stamped.insert(std::pair<std::string,geometry_msgs::PoseStamped> (request.ik_request.group_name,tmp_pose));
      }      
      else if(kinematic_model_->getJointModelGroup(request.ik_request.group_name)->canSetStateFromIK(request.ik_request.ik_link_name))
      {
        pose_stamped.insert(std::pair<std::string,geometry_msgs::PoseStamped> (request.ik_request.group_name,request.ik_request.pose_stamped));
      }      
      else
      {
        // ROS_ERROR("Could not find IK solver for link %s for group %s",request.ik_request.ik_link_name.c_str(),request.ik_request.group_name.c_str());
        return false;
      }      
    }    
  }
  else //This is a request for a multi-group - e.g. arms = right_arm + left_arm
  {
    std::vector<std::string> group_vector = group_map_.find(request.ik_request.group_name)->second;
    if(request.ik_request.pose_stamped_vector.size() != group_vector.size())
    {
      // ROS_ERROR("Request must contain %d poses",(int) request.ik_request.pose_stamped_vector.size());
      return false;
    }
        
    if(request.ik_request.ik_link_names.empty())
    {
      for(unsigned int i = 0; i < request.ik_request.pose_stamped_vector.size(); ++i)
      {
        pose_stamped.insert(std::pair<std::string,geometry_msgs::PoseStamped> (group_vector[i],request.ik_request.pose_stamped_vector[i]));
      }      
    }
    else
    {
      if(request.ik_request.ik_link_names.size() != group_vector.size())
      {
        // ROS_ERROR("Request must contain either 0 or %d elements in ik_link_names vector",(int) group_vector.size());
        return false;
      }      
      for(unsigned int i=0; i < request.ik_request.pose_stamped_vector.size(); ++i)
      {        
        //The assumption is that this new link is rigidly attached to the tip link for the group
        if(!kinematic_model_->getJointModelGroup(group_vector[i])->hasLinkModel(request.ik_request.ik_link_names[i]) && 
           kinematic_model_->getJointModelGroup(group_vector[i])->isLinkUpdated(request.ik_request.ik_link_names[i]))
        {
          geometry_msgs::PoseStamped tmp_pose = request.ik_request.pose_stamped_vector[i];        
          tmp_pose.pose = getTipFramePose(planning_scene,kinematic_state,request.ik_request.pose_stamped_vector[i].pose,request.ik_request.ik_link_names[i],group_vector[i]);       
          pose_stamped.insert(std::pair<std::string,geometry_msgs::PoseStamped> (group_vector[i],tmp_pose));
        }      
        else if(kinematic_model_->getJointModelGroup(request.ik_request.group_name)->canSetStateFromIK(request.ik_request.ik_link_names[i]))
        {
          pose_stamped.insert(std::pair<std::string,geometry_msgs::PoseStamped> (group_vector[i],request.ik_request.pose_stamped_vector[i]));
        }      
        else
        {
          // ROS_ERROR("Could not find IK solver for link %s for group %s",request.ik_request.ik_link_names[i].c_str(),group_vector[i].c_str());
          return false;
        }      
      }      
    }        
  }  
  return true;  
}

std::vector<kinematics::KinematicsBaseConstPtr> KinematicsSolver::getKinematicsSolvers(const std::vector<std::string> &group_names) const
{
  std::vector<kinematics::KinematicsBaseConstPtr> kinematics_solvers;
  for(unsigned int i=0; i < group_names.size(); ++i)
  {
    kinematics_solvers.push_back(kinematics_solver_map_.find(group_names[i])->second);    
  }  
  return kinematics_solvers;  
}

std::map<std::string,geometry_msgs::PoseStamped> KinematicsSolver::transformPoses(const planning_scene::PlanningSceneConstPtr& planning_scene, 
                                                                                  const robot_state::RobotState &kinematic_state,
                                                                                  const std::map<std::string,geometry_msgs::PoseStamped> &poses,
                                                                                  const std::map<std::string,std::string> &target_frames) const
{
  Eigen::Affine3d eigen_pose, eigen_pose_2;
  std::map<std::string,geometry_msgs::PoseStamped> result;  
  for(std::map<std::string,geometry_msgs::PoseStamped>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter)
  {
    std::string target_frame = target_frames.find(iter->first)->second;   
    bool target_frame_is_root_frame = (target_frame == kinematic_state.getKinematicModel()->getModelFrame());  
    geometry_msgs::PoseStamped pose_stamped = iter->second;    
    tf::poseMsgToEigen(pose_stamped.pose, eigen_pose_2);
    planning_scene->getTransforms()->transformPose(kinematic_state,pose_stamped.header.frame_id,eigen_pose_2,eigen_pose);
    if(!target_frame_is_root_frame)
    {
      eigen_pose_2 = planning_scene->getTransforms()->getTransform(kinematic_state,target_frame);
      eigen_pose = eigen_pose_2.inverse()*eigen_pose;
    }    
    pose_stamped.header.frame_id = target_frame;
    tf::poseEigenToMsg(eigen_pose, pose_stamped.pose);
    result.insert(std::pair<std::string,geometry_msgs::PoseStamped> (iter->first,pose_stamped));    
  }
  return result;  
}

geometry_msgs::Pose KinematicsSolver::getTipFramePose(const planning_scene::PlanningSceneConstPtr& planning_scene, 
                                                      const robot_state::RobotState &kinematic_state,
                                                      const geometry_msgs::Pose &pose,
                                                      const std::string &link_name,
                                                      const std::string &group_name) const
{
  geometry_msgs::Pose result;  
  Eigen::Affine3d eigen_pose_in, eigen_pose_link, eigen_pose_tip;
  std::string tip_name = kinematics_solver_map_.find(group_name)->second->getBaseFrame();  
  tf::poseMsgToEigen(pose, eigen_pose_in);
  eigen_pose_link = planning_scene->getTransforms()->getTransform(kinematic_state,link_name);
  eigen_pose_tip = planning_scene->getTransforms()->getTransform(kinematic_state,tip_name);
  eigen_pose_in = eigen_pose_in*(eigen_pose_link.inverse()*eigen_pose_tip);
  tf::poseEigenToMsg(eigen_pose_in, result);
  return result;  
}


moveit_msgs::RobotState KinematicsSolver::getRobotState(const kinematics_planner::SolutionStateMap &solutions,
                                                        const std::vector<std::string> &group_names) const
{
  moveit_msgs::RobotState robot_state;
  for(unsigned int i=0; i < group_names.size(); ++i)
  {
    const std::vector<double>& group_solutions = (solutions.find(group_names[i])->second);      
    robot_state.joint_state.position.insert(robot_state.joint_state.position.end(),group_solutions.begin(),group_solutions.end());    
    const std::vector<std::string> joint_names = kinematic_model_->getJointModelGroup(group_names[i])->getJointModelNames();    
    robot_state.joint_state.name.insert(robot_state.joint_state.name.end(),joint_names.begin(),joint_names.end());
  }   
  return robot_state;  
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
