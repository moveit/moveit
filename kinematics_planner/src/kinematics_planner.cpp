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
#include <kinematics_planner/kinematics_planner.h>
#include <planning_models/kinematic_model.h>

namespace kinematics_planner
{

bool KinematicsPlanner::initialize(const planning_models::KinematicModelConstPtr &kinematic_model,
                                   const kinematics_planner::KinematicsSolverMapConstPtr &kinematics_solver_map,
                                   const std::string &group_name)
{
  group_name_ = group_name;
  if(!kinematic_model->hasJointModelGroup(group_name))
  {
    ROS_ERROR("Group name: %s invalid",group_name.c_str());
    return false;
  }  

  const planning_models::KinematicModel::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_name_);
  group_names_ = joint_model_group->getSubgroupNames();
  if(group_names_.empty())
    group_names_.push_back(group_name_);
  num_groups_= group_names_.size();

  for(unsigned int i=0; i < group_names_.size(); ++i)
  {
    if(kinematics_solver_map->find(group_names_[i]) == kinematics_solver_map->end())
    {
      ROS_ERROR("No kinematics solver found for group %s",group_names_[i].c_str());      
      return false;      
    }    
    kinematics_solvers_.push_back(kinematics_solver_map->find(group_names_[i])->second);    
    kinematics_base_frames_.push_back(kinematics_solvers_.back()->getBaseFrame());    
    const std::vector<std::string> joint_names = kinematic_model->getJointModelGroup(group_names_[i])->getJointModelNames();
    for(unsigned int j=0; j < joint_names.size(); ++j)
      joint_names_.push_back(joint_names[j]);      
  }
  return true;  
}

bool KinematicsPlanner::solve(const std::map<std::string,geometry_msgs::PoseStamped> &start_request,
                              const std::map<std::string,geometry_msgs::PoseStamped> &goal_request,
                              const planning_scene::PlanningSceneConstPtr& planning_scene,
                              const moveit_msgs::Constraints &path_constraints,
                              double timeout,
                              moveit_msgs::RobotTrajectory &robot_trajectory,
                              moveit_msgs::MoveItErrorCodes &error_code) const
{
  kinematic_constraints::KinematicConstraintSet kinematic_constraints_set(planning_scene->getKinematicModel(),planning_scene->getTransforms());  
  kinematic_constraints_set.add(path_constraints);  
  return solve(start_request,goal_request,planning_scene,kinematic_constraints_set,timeout,robot_trajectory,error_code);
}

bool KinematicsPlanner::solve(const std::map<std::string,geometry_msgs::PoseStamped> &start_request,
                              const std::map<std::string,geometry_msgs::PoseStamped> &goal_request,
                              const planning_scene::PlanningSceneConstPtr& planning_scene,
                              const kinematic_constraints::KinematicConstraintSet& kinematic_constraint_set,
                              double timeout,
                              moveit_msgs::RobotTrajectory &robot_trajectory,
                              moveit_msgs::MoveItErrorCodes &error_code) const
{
  ros::WallTime start_time = ros::WallTime::now();  
  if(!checkRequest(start_request) || !checkRequest(goal_request))
  {
    error_code.val = error_code.INVALID_GROUP_NAME;
    return false;    
  }

  planning_models::KinematicState kinematic_state = planning_scene->getCurrentState();
  std::vector<planning_models::KinematicState::JointStateGroup*> joint_state_groups(num_groups_);
  for(unsigned int i=0; i < num_groups_; ++i)
    joint_state_groups[i] = kinematic_state.getJointStateGroup(group_names_[i]);    
  
  std::map<std::string,geometry_msgs::PoseStamped> start = transformPoses(planning_scene,kinematic_state,start_request,kinematics_base_frames_);
  std::map<std::string,geometry_msgs::PoseStamped> goal = transformPoses(planning_scene,kinematic_state,goal_request,kinematics_base_frames_);
  
  std::map<std::string,std::vector<geometry_msgs::Pose> > interpolated_poses = getInterpolatedPosesMap(start,goal);  
  unsigned int num_poses = interpolated_poses[group_names_[0]].size();
    
  kinematics_planner::SolutionTrajectoryMap solutions;
  for(unsigned int i=0; i < num_groups_; ++i)
  {
    solutions[group_names_[i]].resize(num_poses);
    for(unsigned int j=0; j < num_poses; j++)
      solutions[group_names_[i]][j].resize(joint_state_groups[i]->getVariableCount());    
  }
  ros::WallDuration elapsed_time = ros::WallTime::now()-start_time;
  while(elapsed_time <= ros::WallDuration(timeout))
  {
    bool success = true;    
    for(unsigned int i=0; i < num_groups_; ++i)
      joint_state_groups[i]->setToRandomValues();
        
    for(unsigned int i=0; i < num_poses_; ++i)
    {      
      for(unsigned int j=0; j < num_groups_; ++j)
      {
        std::vector<double> joint_state_values;
        joint_state_groups[j]->getGroupStateValues(joint_state_values);
        
        const kinematics::KinematicsBaseConstPtr kinematics_solver = kinematics_solvers_[j];        
        if(!kinematics_solver->getPositionIK(interpolated_poses[group_names_[j]][i],
                                             joint_state_values,
                                             solutions[group_names_[j]][i],
                                             error_code))
        {
          success = false;        
          break;                                                                           
        }
        joint_state_groups[j]->setStateValues(solutions[group_names_[j]][i]);        
      }      
      if(!success)
        break;            
      if(planning_scene->isStateColliding(kinematic_state))
      {
        success = false;        
        break;
      }      
      if(!planning_scene->isStateConstrained(kinematic_state,kinematic_constraint_set))
      {
        success = false;        
        break;      
      } 
    }  
    if(success)
    {
      robot_trajectory = getRobotTrajectory(solutions,num_poses);      
      return true;
    }  
    elapsed_time = ros::WallTime::now()-start_time;
  }
  return false;    
}

std::map<std::string,geometry_msgs::PoseStamped> KinematicsPlanner::transformPoses(const planning_scene::PlanningSceneConstPtr& planning_scene, 
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

std::map<std::string,geometry_msgs::PoseStamped> KinematicsPlanner::transformPoses(const planning_scene::PlanningSceneConstPtr& planning_scene, 
                                                                                   const planning_models::KinematicState &kinematic_state,
                                                                                   const std::map<std::string,geometry_msgs::PoseStamped> &poses,
                                                                                   const std::string &target_frame) const
{
  std::vector<std::string> target_frames;
  for(unsigned int i=0; i < group_names_.size(); ++i)
    target_frames.push_back(target_frame);  
  return transformPoses(planning_scene,kinematic_state,poses,target_frames);  
}

moveit_msgs::RobotTrajectory KinematicsPlanner::getRobotTrajectory(const kinematics_planner::SolutionTrajectoryMap &solutions,
                                                                   unsigned int num_poses) const
{
  moveit_msgs::RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory.joint_names = joint_names_;
  robot_trajectory.joint_trajectory.points.resize(num_poses);
  for(unsigned int i=0; i < robot_trajectory.joint_trajectory.points.size(); ++i)
  {
    for(unsigned int j=0; j < num_groups_; ++j)
    {
      const std::vector<double>& group_solutions = (solutions.find(group_names_[j])->second)[i];      
      robot_trajectory.joint_trajectory.points[i].positions.insert(robot_trajectory.joint_trajectory.points[i].positions.end(),group_solutions.begin(),group_solutions.end());      
    }    
  }   
  return robot_trajectory;  
}

bool KinematicsPlanner::checkRequest(const std::map<std::string,geometry_msgs::PoseStamped> &start) const
{
  for(unsigned int i=0; i < group_names_.size(); ++i)
    if(start.find(group_names_[i]) == start.end())
      return false;
  return true;  
}

std::map<std::string,std::vector<geometry_msgs::Pose> > KinematicsPlanner::getInterpolatedPosesMap(const std::map<std::string,geometry_msgs::PoseStamped> &start,
                                                                                                   const std::map<std::string,geometry_msgs::PoseStamped> &goal) const
{
  unsigned int num_segments = 0;  
  for(unsigned int i=0; i < group_names_.size(); ++i)
  {
    unsigned int group_segments = getNumSegments(start.find(group_names_[i])->second.pose,goal.find(group_names_[i])->second.pose);
    if(group_segments > num_segments)
      num_segments = group_segments;    
  }

  std::map<std::string,std::vector<geometry_msgs::Pose> > result;  
  for(unsigned int i=0; i < group_names_.size(); ++i)
    result[group_names_[i]] = getInterpolatedPoses(start.find(group_names_[i])->second.pose,
                                                   goal.find(group_names_[i])->second.pose,
                                                   num_segments);
  
  return result;  
}

/*
bool solve::searchPositionIK(const geometry_msgs::Pose &start,
                             const geometry_msgs::Pose &goal,
                             double timeout,
                             double consistency_limit,                      
                             std::vector<std::vector<double> > &joint_solutions,
                             std::vector<geometry_msgs::Pose> &interpolated_poses,
                             moveit_msgs::MoveItErrorCodes &error_code,
                             const IKCallbackFn &desired_pose_callback,
                             const IKCallbackFn &solution_callback) const
{
  joint_solutions.resize(interpolated_poses.size());  
  while((ros::WallTime::now() - start_time) <= ros::WallDuration(timeout))
  {
    bool success = true;    
    getRandomConfiguration(jnt_pos_in_);
    for(unsigned int i = 0; i < interpolated_poses.size(); ++i)
    {
      KDL::Frame pose_desired;
      tf::PoseMsgToKDL(interpolated_poses[i], pose_desired);
      joint_solutions[i].resize(dimension_);      
      if(!desired_pose_callback.empty())
      {
        desired_pose_callback(interpolated_poses[i],joint_solutions[i],error_code);
        if(error_code.val != error_code.SUCCESS)
        {
          ROS_DEBUG("Could not find inverse kinematics for desired end-effector pose since the pose may be in collision");
          success = false;          
          break;          
        }
      }
      int ik_valid = ik_solver_pos_->CartToJnt(jnt_pos_in_,pose_desired,jnt_pos_out_);
      if(ik_valid < 0)
      {        
        success = false;        
        break;      
      }      
      if(i > 0)
      {
        if(!checkConsistency(jnt_pos_in_, consistency_limit, jnt_pos_out_))
        {
          success = false;
          break;          
        }        
      }
      for(unsigned int j=0; j < dimension_; ++j)
        joint_solutions[i][j] = jnt_pos_out_(j);      
      if(!solution_callback.empty())
        solution_callback(interpolated_poses[i],joint_solutions[i],error_code);
      if(error_code.val != error_code.SUCCESS)
      {
        ROS_DEBUG("Could not find inverse kinematics for joint solution");
        success = false;          
        break;          
      }
      jnt_pos_in_ = jnt_pos_out_;      
    }
    if(success)
      return true;    
  }  
  return false;  
}
*/

std::vector<double> KinematicsPlanner::getFloatingJointValues(const geometry_msgs::Pose &pose) const
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

geometry_msgs::Pose KinematicsPlanner::getPose(const std::vector<double> &values) const
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

unsigned int KinematicsPlanner::getNumSegments(const geometry_msgs::Pose &start,
                                               const geometry_msgs::Pose &goal) const
{
  planning_models::KinematicModel::FloatingJointModel floating_joint("end_effector");
  std::vector<double> start_values = getFloatingJointValues(start);
  std::vector<double> goal_values = getFloatingJointValues(goal);
     
  double translation_distance = floating_joint.distanceTranslation(start_values,goal_values);
  double rotation_angle = fabs(floating_joint.distanceRotation(start_values,goal_values));
  
  unsigned int num_segments = std::ceil(translation_distance/discretization_translation_);
  unsigned int num_rotation_segments = std::ceil(rotation_angle/discretization_rotation_);
  if(num_rotation_segments > num_segments)
    num_segments = num_rotation_segments;  
  return num_segments;
}

std::vector<geometry_msgs::Pose> KinematicsPlanner::getInterpolatedPoses(const geometry_msgs::Pose &start,
                                                                         const geometry_msgs::Pose &goal,
                                                                         const unsigned int num_segments) const
{
  planning_models::KinematicModel::FloatingJointModel floating_joint("end_effector");
  std::vector<double> start_values = getFloatingJointValues(start);
  std::vector<double> goal_values = getFloatingJointValues(goal);

  std::vector<geometry_msgs::Pose> interp_poses;  
  interp_poses.resize(num_segments+1);  
  for(unsigned int i = 0; i < num_segments; ++i)
  {
    double t = ((double) i)/num_segments;
    std::vector<double> state(7);
    floating_joint.interpolate(start_values,goal_values,t,state);
    geometry_msgs::Pose pose = getPose(state);
    interp_poses.push_back(pose);    
  }  
  interp_poses.push_back(goal);  
  return interp_poses;  
}

}
