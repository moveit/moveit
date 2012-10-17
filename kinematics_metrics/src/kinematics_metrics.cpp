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

bool KinematicsMetrics::checkState(const planning_models::KinematicState &kinematic_state,
                                   const std::string &group_name) const
{
  if(!kinematic_state.hasJointStateGroup(group_name))
    return false;  
  return true;
}

Eigen::MatrixXd KinematicsMetrics::getJacobian(const planning_models::KinematicState &kinematic_state,
                                               const std::string &group_name) const
{
  Eigen::MatrixXd jacobian;
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  std::string link_name = kinematic_state.getKinematicModel()->getJointModelGroup(group_name)->getLinkModelNames().back();  
  kinematic_state.getJointStateGroup(group_name)->getJacobian(link_name,reference_point_position,jacobian);
  return jacobian;
}

bool KinematicsMetrics::getManipulabilityIndex(const planning_models::KinematicState &kinematic_state, 
                                               const std::string &group_name,
                                               double &manipulability_index) const
{
  if(!checkState(kinematic_state,group_name))
    return false;  
  Eigen::MatrixXd jacobian = getJacobian(kinematic_state,group_name);  
  Eigen::MatrixXd matrix = jacobian*jacobian.transpose();  
  // Get manipulability index
  manipulability_index = sqrt(matrix.determinant());
  return true;  
}

bool KinematicsMetrics::getManipulabilityEllipsoid(const planning_models::KinematicState &kinematic_state,
                                                   const std::string &group_name,
                                                   Eigen::MatrixXcd &eigen_values,
                                                   Eigen::MatrixXcd &eigen_vectors) const
{
  if(!checkState(kinematic_state,group_name))
    return false;  
  Eigen::MatrixXd jacobian = getJacobian(kinematic_state,group_name);  
  Eigen::MatrixXd matrix = jacobian*jacobian.transpose();  
  Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(matrix.block(0,0,3,3));
  eigen_values = eigensolver.eigenvalues();
  eigen_vectors = eigensolver.eigenvectors();
  return true;  
}

bool KinematicsMetrics::getConditionNumber(const planning_models::KinematicState &kinematic_state,
                                           const std::string &group_name,
                                           double &condition_number)
{
  if(!checkState(kinematic_state,group_name))
    return false;  
  Eigen::MatrixXd jacobian = getJacobian(kinematic_state,group_name);  
  Eigen::MatrixXd matrix = jacobian*jacobian.transpose();  
  Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(matrix);
  Eigen::MatrixXcd eigen_values = eigensolver.eigenvalues();
  Eigen::MatrixXcd eigen_vectors = eigensolver.eigenvectors();
  condition_number = eigen_values.real().maxCoeff()/eigen_values.real().minCoeff();  
  return true;  
}

} // namespace
