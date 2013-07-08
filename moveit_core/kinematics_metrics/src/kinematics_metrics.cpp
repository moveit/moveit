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
#include <boost/math/constants/constants.hpp>

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

double KinematicsMetrics::getJointLimitsPenalty(const robot_state::JointStateGroup* joint_state_group) const
{
  if(fabs(penalty_multiplier_) <= boost::math::tools::epsilon<double>())
     return 1.0;
  double joint_limits_multiplier(1.0);
  const std::vector<robot_state::JointState*> &joint_state_vector = joint_state_group->getJointStateVector();
  for(std::size_t i=0; i < joint_state_group->getJointStateVector().size(); ++i)
  {
     if(joint_state_vector[i]->getType() == robot_model::JointModel::REVOLUTE)
    {
      const robot_model::RevoluteJointModel* revolute_model = dynamic_cast<const robot_model::RevoluteJointModel*> (joint_state_vector[i]->getJointModel());
      if(revolute_model->isContinuous())
        continue;
    }
    if(joint_state_vector[i]->getType() == robot_model::JointModel::PLANAR)
    {
      const std::vector<std::pair<double, double> >& planar_bounds = joint_state_vector[i]->getVariableBounds();
      if(planar_bounds[0].first == -std::numeric_limits<double>::max() || planar_bounds[0].second == std::numeric_limits<double>::max() ||
         planar_bounds[1].first == -std::numeric_limits<double>::max() || planar_bounds[1].second == std::numeric_limits<double>::max() ||
         planar_bounds[2].first == -boost::math::constants::pi<double>() || planar_bounds[2].second == boost::math::constants::pi<double>())
        continue;
    }
    if(joint_state_vector[i]->getType() == robot_model::JointModel::FLOATING)
    {
      //Joint limits are not well-defined for floating joints
      continue;
    }
    const std::vector<double>& joint_values = joint_state_vector[i]->getVariableValues();
    const std::vector<std::pair<double, double> >& bounds = joint_state_vector[i]->getVariableBounds();
    std::vector<double> lower_bounds, upper_bounds;
    for(std::size_t j=0; j < bounds.size(); ++j)
    {
      lower_bounds.push_back(bounds[j].first);
      upper_bounds.push_back(bounds[j].second);
    }
    double lower_bound_distance = joint_state_vector[i]->getJointModel()->distance(joint_values, lower_bounds);
    double upper_bound_distance = joint_state_vector[i]->getJointModel()->distance(joint_values, upper_bounds);
    double range = lower_bound_distance + upper_bound_distance;
    if(range <= boost::math::tools::epsilon<double>())
      continue;
    joint_limits_multiplier *= (lower_bound_distance * upper_bound_distance/(range*range));
  }
  return(1.0 - exp(-penalty_multiplier_*joint_limits_multiplier));
}

bool KinematicsMetrics::getManipulabilityIndex(const robot_state::RobotState &kinematic_state,
                                               const std::string &group_name,
                                               double &manipulability_index,
                                               bool translation) const
{
  const robot_model::JointModelGroup *joint_model_group = kinematic_model_->getJointModelGroup(group_name);
  return getManipulabilityIndex(kinematic_state, joint_model_group, manipulability_index, translation);
}

bool KinematicsMetrics::getManipulabilityIndex(const robot_state::RobotState &kinematic_state,
                                               const robot_model::JointModelGroup *joint_model_group,
                                               double &manipulability_index,
                                               bool translation) const
{
  if (!joint_model_group)
  {
    logError("Joint model group does not exist");
    return false;
  }
  Eigen::MatrixXd jacobian = getJacobian(kinematic_state, joint_model_group);
  // Get joint limits penalty
  double penalty = getJointLimitsPenalty(kinematic_state.getJointStateGroup(joint_model_group->getName()));
  if(translation)
  {
    Eigen::MatrixXd jacobian_2 = jacobian.topLeftCorner(3,jacobian.cols());
    Eigen::MatrixXd matrix = jacobian_2*jacobian_2.transpose();
    // Get manipulability index
    manipulability_index = penalty * sqrt(matrix.determinant());
  }
  else
  {
    Eigen::MatrixXd matrix = jacobian*jacobian.transpose();
    // Get manipulability index
    manipulability_index = penalty * sqrt(matrix.determinant());
  }
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
                                          double &manipulability,
                                          bool translation) const
{
  const robot_model::JointModelGroup *joint_model_group = kinematic_model_->getJointModelGroup(group_name);
  return getManipulability(kinematic_state, joint_model_group, manipulability, translation);
}

bool KinematicsMetrics::getManipulability(const robot_state::RobotState &kinematic_state,
                                          const robot_model::JointModelGroup *joint_model_group,
                                          double &manipulability,
                                          bool translation) const
{
  if (!joint_model_group)
  {
    logError("Joint model group does not exist");
    return false;
  }

  // Get joint limits penalty
  double penalty = getJointLimitsPenalty(kinematic_state.getJointStateGroup(joint_model_group->getName()));
  if(translation)
  {
    Eigen::MatrixXd jacobian = getJacobian(kinematic_state, joint_model_group);
    Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jacobian.topLeftCorner(3,jacobian.cols()));
    Eigen::MatrixXd singular_values = svdsolver.singularValues();
    for(unsigned int i=0; i < singular_values.rows(); ++i)
      logDebug("Singular value: %d %f",i,singular_values(i,0));
    manipulability = penalty * singular_values.minCoeff()/singular_values.maxCoeff();
  }
  else
  {
    Eigen::MatrixXd jacobian = getJacobian(kinematic_state, joint_model_group);
    Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jacobian);
    Eigen::MatrixXd singular_values = svdsolver.singularValues();
    for(unsigned int i=0; i < singular_values.rows(); ++i)
      logDebug("Singular value: %d %f",i,singular_values(i,0));
    manipulability = penalty * singular_values.minCoeff()/singular_values.maxCoeff();
  }
  return true;
}

} // namespace
