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
*
* Author: Sachin Chitta
*********************************************************************/

#include <kinematics_metrics/kinematics_metrics.h>
#include <Eigen/Eigenvalues>

namespace kinematics_metrics
{

bool KinematicsMetrics::checkState(const planning_models::KinematicState &kinematic_state)
{
  if(!kinematic_state.hasJointStateGroup(group_name_))
    return false;  
  return true;
}

Eigen::MatrixXd KinematicsMetrics::getJacobian(const planning_models::KinematicState &kinematic_state)
{
  Eigen::MatrixXd jacobian;
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  std::string link_name = kinematic_state.getKinematicModel()->getJointModelGroup(group_name_)->getLinkModelNames().back();  
  kinematic_state.getJointStateGroup(group_name_)->getJacobian(link_name,reference_point_position,jacobian);
  return jacobian;
}

bool KinematicsMetrics::getManipulabilityIndex(const planning_models::KinematicState &kinematic_state, 
                                               double &manipulability_index)
{
  if(!checkState(kinematic_state))
    return false;  
  Eigen::MatrixXd jacobian = getJacobian(kinematic_state);  
  Eigen::MatrixXd matrix = jacobian*jacobian.transpose();  
  // Get manipulability index
  manipulability_index = matrix.determinant();
  return true;  
}

bool KinematicsMetrics::getManipulabilityEllipsoid(const planning_models::KinematicState &kinematic_state,
                                                   Eigen::MatrixXcd &eigen_values,
                                                   Eigen::MatrixXcd &eigen_vectors)
{
  if(!checkState(kinematic_state))
    return false;  
  Eigen::MatrixXd jacobian = getJacobian(kinematic_state);  
  Eigen::MatrixXd matrix = jacobian*jacobian.transpose();  
  Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(matrix);
  eigen_values = eigensolver.eigenvalues();
  eigen_vectors = eigensolver.eigenvectors();
  return true;  
}
    
bool KinematicsMetrics::initialize(const planning_models::KinematicModelConstPtr &kinematic_model,
                                   const std::string &group_name)
{
  if(!kinematic_model_->hasJointModelGroup(group_name))
  {
    ROS_ERROR("Kinematic model does not have group: %s",group_name.c_str());
    return false;
  }  
  kinematic_model_ = kinematic_model;
  group_name_ = group_name;
  return true;  
}


} // namespace
