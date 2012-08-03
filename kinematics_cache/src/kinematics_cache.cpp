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

#include <kinematics_cache/kinematics_cache.h>

namespace kinematics_cache
{

KinematicsCache::KinematicsCache()
{
  
}

bool KinematicsCache::initialize(kinematics::KinematicsBaseConstPtr &kinematics_solver,
                                 planning_models::KinematicModelConstPtr &kinematic_model,
                                 const KinematicsCache::Options &opt)
{
  options_ = opt;  
  kinematics_solver_ = kinematics_solver;
  kinematic_model_   = kinematic_model;  
  joint_model_group_ =  kinematic_model_->getJointModelGroup(kinematics_solver_->getGroupName());
  kinematic_state_.reset(new planning_models::KinematicState(kinematic_model));
  joint_state_group_.reset(new planning_models::KinematicState::JointStateGroup(kinematic_state_.get(),joint_model_group_));

  cache_origin_ = opt.origin;
  cache_resolution_ = opt.resolution;
  cache_size_x_ = (unsigned int) (opt.workspace_size.x/opt.resolution);
  cache_size_y_ = (unsigned int) (opt.workspace_size.y/opt.resolution);
  cache_size_z_ = (unsigned int) (opt.workspace_size.z/opt.resolution);
  max_solutions_per_grid_location_ = opt.max_solutions_per_grid_location;
  solution_dimension_ = joint_model_group_->getVariableCount();  
  size_grid_node_ = max_solutions_per_grid_location_ * solution_dimension_;  
  kinematics_cache_size_ = cache_size_x_*cache_size_y_*cache_size_z_;
  kinematics_cache_points_with_solution_ = 0;  
  solution_local_.resize(solution_dimension_);
  kinematics_cache_vector_.resize(kinematics_cache_size_*size_grid_node_,0.0);    
  num_solutions_vector_.resize(kinematics_cache_size_,0);  
  ROS_INFO("Origin: %f %f %f",cache_origin_.x,cache_origin_.y,cache_origin_.z);
  ROS_INFO("Cache size: %d %d %d",cache_size_x_,cache_size_y_,cache_size_z_);  
  ROS_INFO("Initialized");  
  return true;  
}

bool KinematicsCache::generateCacheMap(double timeout) 
{
  ros::WallTime start_time = ros::WallTime::now();  
  std::vector<std::string> fk_names;
  std::vector<double> fk_values;  
  std::vector<geometry_msgs::Pose> poses;    

  fk_names.push_back(kinematics_solver_->getTipFrame());
  fk_values.resize(kinematics_solver_->getJointNames().size(),0.0);
  poses.resize(1);    

  while((ros::WallTime::now()-start_time).toSec() <= timeout && kinematics_cache_points_with_solution_ <= kinematics_cache_size_)
  {
    joint_state_group_->setToRandomValues();
    joint_state_group_->getGroupStateValues(fk_values);    
    if(!kinematics_solver_->getPositionFK(fk_names,fk_values,poses))
    {
      ROS_ERROR("Fk failed");      
      return false;    
    }    
    if(!addToCache(poses[0],fk_values))
    {
      ROS_DEBUG("Adding to cache failed for: %f %f %f",poses[0].position.x,poses[0].position.y,poses[0].position.z);      
    }    
    ROS_DEBUG("Adding: %d",kinematics_cache_points_with_solution_);    
  }
  ROS_INFO("Cache map generated with %d valid points",kinematics_cache_points_with_solution_);  
  return true;  
}

bool KinematicsCache::addToCache(const geometry_msgs::Pose &pose, const std::vector<double> &joint_values)
{
  unsigned int grid_index;
  if(!getGridIndex(pose,grid_index))
  {    
    ROS_DEBUG("Failed to get grid index");    
    return false;  
  }  
  unsigned int num_solutions = num_solutions_vector_[grid_index];  
  if(num_solutions >= max_solutions_per_grid_location_)
  {
    ROS_DEBUG("Pose already has max number of solutions");
    return true;    
  }
  if(num_solutions == 0)
    kinematics_cache_points_with_solution_++;    
  unsigned int start_index = getSolutionLocation(grid_index,num_solutions);
  for(unsigned int i=0; i < joint_values.size(); ++i)
  {
    //    ROS_INFO("Joint value[%d]: %f, localtion: %d",i,joint_values[i],start_index+i);    
    kinematics_cache_vector_[start_index+i] = joint_values[i];    
  }
  num_solutions_vector_[grid_index]++;  
  return true;  
}

bool KinematicsCache::getGridIndex(const geometry_msgs::Pose &pose, unsigned int &grid_index) const
{
  int x_index = (int) ((pose.position.x - cache_origin_.x) /cache_resolution_);
  int y_index = (int) ((pose.position.y - cache_origin_.y) /cache_resolution_);
  int z_index = (int) ((pose.position.z - cache_origin_.z) /cache_resolution_);

  if(x_index >= (int) cache_size_x_ || x_index < 0)
  {    
    ROS_DEBUG("X position %f,%d lies outside grid: %d %d",pose.position.x,x_index,0,cache_size_x_);    
    return false;
  }  
  if(y_index >= (int) cache_size_y_ || y_index < 0)
  {
    ROS_DEBUG("Y position %f,%d lies outside grid: %d %d",pose.position.y,y_index,0,cache_size_y_);    
    return false;
  }  
  if(z_index >= (int) cache_size_z_ || z_index < 0)
  {
    ROS_DEBUG("Z position %f,%d lies outside grid: %d %d",pose.position.z,z_index,0,cache_size_z_);    
    return false;
  }  
  ROS_DEBUG("Grid indices: %d %d %d",x_index,y_index,z_index);  
  grid_index = (x_index + y_index * cache_size_x_ + z_index * cache_size_x_ * cache_size_y_);
  return true;  
}

bool KinematicsCache::getNumSolutions(const geometry_msgs::Pose &pose, unsigned int &num_solutions) const
{
  unsigned int grid_index;  
  if(!getGridIndex(pose,grid_index))
    return false;
  
  num_solutions = num_solutions_vector_[grid_index];  
  return true;  
}

unsigned int KinematicsCache::getSolutionLocation(unsigned int &grid_index, unsigned int &solution_index) const
{
  ROS_DEBUG("[Grid Index, Solution number location]: %d, %d",grid_index,grid_index * size_grid_node_ + solution_index * solution_dimension_);  
  return (grid_index * size_grid_node_ + solution_index * solution_dimension_);  
}

bool KinematicsCache::getSolution(const geometry_msgs::Pose &pose, unsigned int solution_index, std::vector<double> &solution) const
{
  unsigned int grid_index;
  if(!getGridIndex(pose,grid_index))
    return false;
  if(solution_index >= max_solutions_per_grid_location_)
    return false;
  if(solution.size() < solution_dimension_)
    return false;

  solution = getSolution(grid_index,solution_index);
  return true;
}

bool KinematicsCache::getSolutions(const geometry_msgs::Pose &pose, std::vector<std::vector<double> > &solution) const
{
  unsigned int grid_index;
  if(!getGridIndex(pose,grid_index))
    return false;
  if(solution.size() != num_solutions_vector_[grid_index])
    return false;
  for(unsigned int i=0; i < solution.size(); i++)
  {
    if (solution[i].size() != solution_dimension_)
      return false;
    solution[i] = getSolution(grid_index,i);
  }
  return true;
}

std::vector<double>& KinematicsCache::getSolution(unsigned int &grid_index, unsigned int solution_index) const
{  
  unsigned int solution_location = getSolutionLocation(grid_index,solution_index);
  for(unsigned int i=0; i < solution_dimension_; ++i)
    solution_local_[i] = kinematics_cache_vector_[solution_location+i];  
  return solution_local_;  
}

}
