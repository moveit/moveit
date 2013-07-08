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
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef MOVEIT_ROBOT_STATE_JOINT_STATE_
#define MOVEIT_ROBOT_STATE_JOINT_STATE_

#include <moveit/robot_model/joint_model.h>

namespace robot_state
{

class RobotState;

/** @brief Definition of a joint state - representation of state for a single joint */
class JointState
{
  friend class RobotState;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructs the joint state from the model */
  JointState(const robot_model::JointModel* jm);

  /** \brief Copy constructor */
  JointState(const JointState &other);

  /** \brief Destructor */
  ~JointState();

  /** \brief Copy the data from one joint state to the other */
  JointState& operator=(const JointState &other);

  /** \brief Set the value of a particular variable for this joint */
  bool setVariableValue(const std::string &variable, double value);

  /** \brief Sets the internal values from a map of joint variable names to actual values */
  void setVariableValues(const std::map<std::string, double>& value_map);

  /** \brief Sets the internal values from a map of joint
      variable names to actual values. The function also
      fills the missing vector with the variable names that
      were not set. */
  void setVariableValues(const std::map<std::string, double>& value_map, std::vector<std::string>& missing);

  /** \brief Sets the internal values from the supplied vector, which are assumed to be in the required order */
  bool setVariableValues(const std::vector<double>& value_vector);

  /** \brief Sets the internal values from the supplied
      array, which are assumed to be in the required
      order. This function is intended to be fast, does no
      input checking and should be used carefully. */
  void setVariableValues(const double *value_vector);

  /** \brief Sets the internal values from the transform */
  void setVariableValues(const Eigen::Affine3d& transform);

  /** \brief Update the joints that mimic this one. This function is called automatically by the setVariable* functions */
  void updateMimicJoints();

  /** \brief Specifies whether or not all values associated with a joint are defined in the
      supplied joint value map */
  bool allVariablesAreDefined(const std::map<std::string, double>& value_map) const;

  /** \brief Checks if the current joint state values are all within the bounds set in the model */
  bool satisfiesBounds(double margin = 0.0) const
  {
    return joint_model_->satisfiesBounds(joint_state_values_, margin);
  }

  /** \brief Force the joint to be inside bounds and normalized. Quaternions are normalized, continuous joints are made between -Pi and Pi. */
  void enforceBounds();

  double distance(const JointState *other) const
  {
    return joint_model_->distance(joint_state_values_, other->joint_state_values_);
  }

  void interpolate(const JointState *to, const double t, JointState *dest) const;

  /** \brief Get the name of the model associated with this state */
  const std::string& getName() const
  {
    return joint_model_->getName();
  }

  /** \brief Get the type of joint associated with this state */
  robot_model::JointModel::JointType getType() const
  {
    return joint_model_->getType();
  }

  /** \brief Get the number of variable DOFs for this joint*/
  unsigned int getVariableCount() const
  {
    return joint_model_->getVariableCount();
  }

  /** \brief Get the joint state values stored in the required order */
  const std::vector<double>& getVariableValues() const
  {
    return joint_state_values_;
  }

  /** \brief Get the joint state values stored in the required order */
  std::vector<double>& getVariableValues()
  {
    return joint_state_values_;
  }

  /** \brief Get the required name order for the joint state values */
  const std::vector<std::string>& getVariableNames() const
  {
    return joint_model_->getVariableNames();
  }

  /** \brief Get the variable bounds for this joint, in the same order as the names returned by getVariableNames() */
  const std::vector<std::pair<double, double> > &getVariableBounds() const
  {
    return joint_model_->getVariableBounds();
  }

  /** \brief Get the current variable transform */
  const Eigen::Affine3d& getVariableTransform() const
  {
    return variable_transform_;
  }

  /** \brief Get the current variable transform */
  Eigen::Affine3d& getVariableTransform()
  {
    return variable_transform_;
  }

  /** \brief Get the joint model corresponding to this state*/
  const robot_model::JointModel* getJointModel() const
  {
    return joint_model_;
  }

  /** \brief The set of variables that make up the state value of a joint are stored in some order. This map
      gives the position of each variable in that order, for each variable name */
  const std::map<std::string, unsigned int>& getVariableIndexMap() const
  {
    return joint_model_->getVariableIndexMap();
  }

  /// this will go away; don't use it
  std::vector<double>& getVelocities()
  {
    return horrible_velocity_placeholder_;
  }

  /// this will go away; don't use it
  std::vector<double>& getAccelerations()
  {
    return horrible_acceleration_placeholder_;
  }

  /// this will go away; don't use it
  const std::vector<double>& getVelocities() const
  {
    return horrible_velocity_placeholder_;
  }

  /// this will go away; don't use it
  const std::vector<double>& getAccelerations() const
  {
    return horrible_acceleration_placeholder_;
  }

private:

  /** \brief The joint model this state corresponds to */
  const robot_model::JointModel      *joint_model_;

  /** \brief Tthe local transform (computed by forward kinematics) */
  Eigen::Affine3d                     variable_transform_;

  /** \brief The joint values given in the order indicated by joint_variables_index_map_ */
  std::vector<double>                 joint_state_values_;

  /// ignore this
  std::vector<double>                 horrible_velocity_placeholder_;

  /// ignore this
  std::vector<double>                 horrible_acceleration_placeholder_;

  /** \brief The set of joints that need to be updated when this one is */
  std::vector<JointState*>            mimic_requests_;
};

}
#endif
