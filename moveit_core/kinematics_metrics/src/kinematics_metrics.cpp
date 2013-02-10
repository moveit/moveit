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

#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <Eigen/Eigenvalues>

namespace kinematics_metrics
{

Eigen::MatrixXd KinematicsMetrics::getJacobian(const robot_state::RobotState &kinematic_state,
                                               const robot_model::JointModelGroup *joint_model_group) const
{
  Eigen::MatrixXd jacobian;
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  std::string link_name = joint_model_group->getLinkModelNames().back();  
  kinematic_state.getJointStateGroup(joint_model_group->getName())->getJacobian(link_name, reference_point_position, jacobian);
  return jacobian;
}

bool KinematicsMetrics::getManipulabilityIndex(const robot_state::RobotState &kinematic_state, 
                                               const std::string &group_name,
                                               double &manipulability_index) const
{
  const robot_model::JointModelGroup *joint_model_group = kinematic_model_->getJointModelGroup(group_name);
  return getManipulabilityIndex(kinematic_state, joint_model_group, manipulability_index);  
}

bool KinematicsMetrics::getManipulabilityIndex(const robot_state::RobotState &kinematic_state, 
                                               const robot_model::JointModelGroup *joint_model_group,
                                               double &manipulability_index) const
{
  if (!joint_model_group)
  {    
    logError("Joint model group does not exist");
    return false;
  }
  Eigen::MatrixXd jacobian = getJacobian(kinematic_state, joint_model_group);  
  Eigen::MatrixXd matrix = jacobian*jacobian.transpose();  
  // Get manipulability index
  manipulability_index = sqrt(matrix.determinant());
  return true;  
}

bool KinematicsMetrics::getManipulabilityEllipsoid(const robot_state::RobotState &kinematic_state,
                                                   const std::string &group_name,
                                                   Eigen::MatrixXcd &eigen_values,
                                                   Eigen::MatrixXcd &eigen_vectors) const
{
  const robot_model::JointModelGroup *joint_model_group = kinematic_model_->getJointModelGroup(group_name);
  return getManipulabilityEllipsoid(kinematic_state, joint_model_group, eigen_values, eigen_vectors);  
}

bool KinematicsMetrics::getManipulabilityEllipsoid(const robot_state::RobotState &kinematic_state,
                                                   const robot_model::JointModelGroup *joint_model_group,
                                                   Eigen::MatrixXcd &eigen_values,
                                                   Eigen::MatrixXcd &eigen_vectors) const
{
  if (!joint_model_group)
  {    
    logError("Joint model group does not exist");    
    return false;
  }  
  Eigen::MatrixXd jacobian = getJacobian(kinematic_state, joint_model_group);  
  Eigen::MatrixXd matrix = jacobian*jacobian.transpose();  
  Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(matrix.block(0, 0, 3, 3));
  eigen_values = eigensolver.eigenvalues();
  eigen_vectors = eigensolver.eigenvectors();
  return true;  
}

bool KinematicsMetrics::getManipulability(const robot_state::RobotState &kinematic_state,
                                          const std::string &group_name,
                                          double &manipulability) const
{
  const robot_model::JointModelGroup *joint_model_group = kinematic_model_->getJointModelGroup(group_name);
  return getManipulability(kinematic_state, joint_model_group, manipulability);
}

bool KinematicsMetrics::getManipulability(const robot_state::RobotState &kinematic_state,
                                          const robot_model::JointModelGroup *joint_model_group,
                                          double &manipulability) const
{
  if (!joint_model_group)
  {    
    logError("Joint model group does not exist");    
    return false;
  }  
  Eigen::MatrixXd jacobian = getJacobian(kinematic_state, joint_model_group);  
  Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jacobian);
  Eigen::MatrixXd singular_values = svdsolver.singularValues();
  for(unsigned int i=0; i < singular_values.rows(); ++i)
    logDebug("Singular value: %d %f",i,singular_values(i,0));  
  manipulability = singular_values.minCoeff()/singular_values.maxCoeff();  
  return true;  
}

} // namespace
