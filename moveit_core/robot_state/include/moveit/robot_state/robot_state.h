/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Ioan A. Sucan
*  Copyright (c) 2013, Willow Garage, Inc.
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

#ifndef MOVEIT_CORE_ROBOT_STATE_
#define MOVEIT_CORE_ROBOT_STATE_

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/attached_body.h>
#include <moveit/macros/deprecation.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Twist.h>
#include <cassert>

#include <boost/assert.hpp>

namespace moveit
{
namespace core
{
MOVEIT_CLASS_FORWARD(RobotState);

/** \brief Signature for functions that can verify that if the group \e joint_group in \e robot_state is set to \e
   joint_group_variable_values
    the state is valid or not. Returns true if the state is valid. This call is allowed to modify \e robot_state (e.g.,
   set \e joint_group_variable_values) */
typedef boost::function<bool(RobotState* robot_state, const JointModelGroup* joint_group,
                             const double* joint_group_variable_values)>
    GroupStateValidityCallbackFn;

/** \brief Struct for containing jump_threshold.

    For the purposes of maintaining API, we support both \e jump_threshold_factor which provides a scaling factor for
    detecting joint space jumps and \e revolute_jump_threshold and \e prismatic_jump_threshold which provide abolute
    thresholds for detecting joint space jumps. */
struct JumpThreshold
{
  double factor;
  double revolute;   // Radians
  double prismatic;  // Meters

  explicit JumpThreshold() : factor(0.0), revolute(0.0), prismatic(0.0)
  {
  }

  explicit JumpThreshold(double jt_factor) : JumpThreshold()
  {
    factor = jt_factor;
  }

  explicit JumpThreshold(double jt_revolute, double jt_prismatic) : JumpThreshold()
  {
    revolute = jt_revolute;    // Radians
    prismatic = jt_prismatic;  // Meters
  }
};

/** \brief Struct for containing max_step for computeCartesianPath

    Setting translation to zero will disable checking for translations and the same goes for rotation */
struct MaxEEFStep
{
  MaxEEFStep(double translation = 0.0, double rotation = 0.0) : translation(translation), rotation(rotation)
  {
  }

  double translation;  // Meters
  double rotation;     // Radians
};

/** \brief Representation of a robot's state. This includes position,
    velocity, acceleration and effort.

    At the lowest level, a state is a collection of variables. Each
    variable has a name and can have position, velocity, acceleration
    and effort associated to it. Effort and acceleration share the
    memory area for efficiency reasons (one should not set both
    acceleration and effort in the same state and expect things to
    work). Often variables correspond to joint names as well (joints
    with one degree of freedom have one variable), but joints with
    multiple degrees of freedom have more variables. Operations are
    allowed at variable level, joint level (see JointModel) and joint
    group level (see JointModelGroup).

    For efficiency reasons a state computes forward kinematics in a
    lazy fashion. This can sometimes lead to problems if the update()
    function was not called on the state. */
class RobotState
{
public:
  /** \brief A state can be constructed from a specified robot model. No values are initialized.
      Call setToDefaultValues() if a state needs to provide valid information. */
  RobotState(const RobotModelConstPtr& robot_model);
  ~RobotState();

  /** \brief Copy constructor. */
  RobotState(const RobotState& other);

  /** \brief Copy operator */
  RobotState& operator=(const RobotState& other);

  /** \brief Get the robot model this state is constructed for. */
  const RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  /** \brief Get the number of variables that make up this state. */
  std::size_t getVariableCount() const
  {
    return robot_model_->getVariableCount();
  }

  /** \brief Get the names of the variables that make up this state, in the order they are stored in memory. */
  const std::vector<std::string>& getVariableNames() const
  {
    return robot_model_->getVariableNames();
  }

  /** \brief Get the model of a particular link */
  const LinkModel* getLinkModel(const std::string& link) const
  {
    return robot_model_->getLinkModel(link);
  }

  /** \brief Get the model of a particular joint */
  const JointModel* getJointModel(const std::string& joint) const
  {
    return robot_model_->getJointModel(joint);
  }

  /** \brief Get the model of a particular joint group */
  const JointModelGroup* getJointModelGroup(const std::string& group) const
  {
    return robot_model_->getJointModelGroup(group);
  }

  /** \name Getting and setting variable position
   *  @{
   */

  /** \brief Get a raw pointer to the positions of the variables
      stored in this state. Use carefully. If you change these values
      externally you need to make sure you trigger a forced update for
      the state by calling update(true). */
  double* getVariablePositions()
  {
    return position_;
  }

  /** \brief Get a raw pointer to the positions of the variables
      stored in this state. */
  const double* getVariablePositions() const
  {
    return position_;
  }

  /** \brief It is assumed \e positions is an array containing the new
      positions for all variables in this state. Those values are
      copied into the state. */
  void setVariablePositions(const double* position);

  /** \brief It is assumed \e positions is an array containing the new
      positions for all variables in this state. Those values are
      copied into the state. */
  void setVariablePositions(const std::vector<double>& position)
  {
    assert(robot_model_->getVariableCount() <= position.size());  // checked only in debug mode
    setVariablePositions(&position[0]);
  }

  /** \brief Set the positions of a set of variables. If unknown variable names are specified, an exception is thrown.
   */
  void setVariablePositions(const std::map<std::string, double>& variable_map);

  /** \brief Set the positions of a set of variables. If unknown variable names are specified, an exception is thrown.
      Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void setVariablePositions(const std::map<std::string, double>& variable_map,
                            std::vector<std::string>& missing_variables);

  /** \brief Set the positions of a set of variables. If unknown variable names are specified, an exception is thrown.
      Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void setVariablePositions(const std::vector<std::string>& variable_names,
                            const std::vector<double>& variable_position);

  /** \brief Set the position of a single variable. An exception is thrown if the variable name is not known */
  void setVariablePosition(const std::string& variable, double value)
  {
    setVariablePosition(robot_model_->getVariableIndex(variable), value);
  }

  /** \brief Set the position of a single variable. The variable is specified by its index (a value associated by the
   * RobotModel to each variable) */
  void setVariablePosition(int index, double value)
  {
    position_[index] = value;
    const JointModel* jm = robot_model_->getJointOfVariable(index);
    if (jm)
    {
      markDirtyJointTransforms(jm);
      updateMimicJoint(jm);
    }
  }

  /** \brief Get the position of a particular variable. An exception is thrown if the variable is not known. */
  double getVariablePosition(const std::string& variable) const
  {
    return position_[robot_model_->getVariableIndex(variable)];
  }

  /** \brief Get the position of a particular variable. The variable is
      specified by its index. No checks are performed for the validity
      of the index passed  */
  double getVariablePosition(int index) const
  {
    return position_[index];
  }

  /** @} */

  /** \name Getting and setting variable velocity
   *  @{
   */

  /** \brief By default, if velocities are never set or initialized,
      the state remembers that there are no velocities set. This is
      useful to know when serializing or copying the state.*/
  bool hasVelocities() const
  {
    return has_velocity_;
  }

  /** \brief Get raw access to the velocities of the variables that make up this state. The values are in the same order
   * as reported by getVariableNames() */
  double* getVariableVelocities()
  {
    markVelocity();
    return velocity_;
  }

  /** \brief Get const access to the velocities of the variables that make up this state. The values are in the same
   * order as reported by getVariableNames() */
  const double* getVariableVelocities() const
  {
    return velocity_;
  }

  /** \brief Given an array with velocity values for all variables, set those values as the velocities in this state */
  void setVariableVelocities(const double* velocity)
  {
    has_velocity_ = true;
    // assume everything is in order in terms of array lengths (for efficiency reasons)
    memcpy(velocity_, velocity, robot_model_->getVariableCount() * sizeof(double));
  }

  /** \brief Given an array with velocity values for all variables, set those values as the velocities in this state */
  void setVariableVelocities(const std::vector<double>& velocity)
  {
    assert(robot_model_->getVariableCount() <= velocity.size());  // checked only in debug mode
    setVariableVelocities(&velocity[0]);
  }

  /** \brief Set the velocities of a set of variables. If unknown variable names are specified, an exception is thrown.
   */
  void setVariableVelocities(const std::map<std::string, double>& variable_map);

  /** \brief Set the velocities of a set of variables. If unknown variable names are specified, an exception is thrown.
      Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void setVariableVelocities(const std::map<std::string, double>& variable_map,
                             std::vector<std::string>& missing_variables);

  /** \brief Set the velocities of a set of variables. If unknown variable names are specified, an exception is thrown.
   */
  void setVariableVelocities(const std::vector<std::string>& variable_names,
                             const std::vector<double>& variable_velocity);

  /** \brief Set the velocity of a variable. If an unknown variable name is specified, an exception is thrown. */
  void setVariableVelocity(const std::string& variable, double value)
  {
    setVariableVelocity(robot_model_->getVariableIndex(variable), value);
  }

  /** \brief Set the velocity of a single variable. The variable is specified by its index (a value associated by the
   * RobotModel to each variable) */
  void setVariableVelocity(int index, double value)
  {
    markVelocity();
    velocity_[index] = value;
  }

  /** \brief Get the velocity of a particular variable. An exception is thrown if the variable is not known. */
  double getVariableVelocity(const std::string& variable) const
  {
    return velocity_[robot_model_->getVariableIndex(variable)];
  }

  /** \brief Get the velocity of a particular variable. The variable is
      specified by its index. No checks are performed for the validity
      of the index passed  */
  double getVariableVelocity(int index) const
  {
    return velocity_[index];
  }

  /** @} */

  /** \name Getting and setting variable acceleration
   *  @{
   */

  /** \brief By default, if accelerations are never set or initialized, the state remembers that there are no
     accelerations set. This is
      useful to know when serializing or copying the state. If hasAccelerations() reports true, hasEffort() will
     certainly report false. */
  bool hasAccelerations() const
  {
    return has_acceleration_;
  }

  /** \brief Get raw access to the accelerations of the variables that make up this state. The values are in the same
   * order as reported by getVariableNames(). The area of memory overlaps with effort (effort and acceleration should
   * not be set at the same time) */
  double* getVariableAccelerations()
  {
    markAcceleration();
    return acceleration_;
  }

  /** \brief Get const raw access to the accelerations of the variables that make up this state. The values are in the
   * same order as reported by getVariableNames() */
  const double* getVariableAccelerations() const
  {
    return acceleration_;
  }

  /** \brief Given an array with acceleration values for all variables, set those values as the accelerations in this
   * state */
  void setVariableAccelerations(const double* acceleration)
  {
    has_acceleration_ = true;
    has_effort_ = false;

    // assume everything is in order in terms of array lengths (for efficiency reasons)
    memcpy(acceleration_, acceleration, robot_model_->getVariableCount() * sizeof(double));
  }

  /** \brief Given an array with acceleration values for all variables, set those values as the accelerations in this
   * state */
  void setVariableAccelerations(const std::vector<double>& acceleration)
  {
    assert(robot_model_->getVariableCount() <= acceleration.size());  // checked only in debug mode
    setVariableAccelerations(&acceleration[0]);
  }

  /** \brief Set the accelerations of a set of variables. If unknown variable names are specified, an exception is
   * thrown. */
  void setVariableAccelerations(const std::map<std::string, double>& variable_map);

  /** \brief Set the accelerations of a set of variables. If unknown variable names are specified, an exception is
     thrown.
      Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void setVariableAccelerations(const std::map<std::string, double>& variable_map,
                                std::vector<std::string>& missing_variables);

  /** \brief Set the accelerations of a set of variables. If unknown variable names are specified, an exception is
   * thrown. */
  void setVariableAccelerations(const std::vector<std::string>& variable_names,
                                const std::vector<double>& variable_acceleration);

  /** \brief Set the acceleration of a variable. If an unknown variable name is specified, an exception is thrown. */
  void setVariableAcceleration(const std::string& variable, double value)
  {
    setVariableAcceleration(robot_model_->getVariableIndex(variable), value);
  }

  /** \brief Set the acceleration of a single variable. The variable is specified by its index (a value associated by
   * the RobotModel to each variable) */
  void setVariableAcceleration(int index, double value)
  {
    markAcceleration();
    acceleration_[index] = value;
  }

  /** \brief Get the acceleration of a particular variable. An exception is thrown if the variable is not known. */
  double getVariableAcceleration(const std::string& variable) const
  {
    return acceleration_[robot_model_->getVariableIndex(variable)];
  }

  /** \brief Get the acceleration of a particular variable. The variable is
      specified by its index. No checks are performed for the validity
      of the index passed  */
  double getVariableAcceleration(int index) const
  {
    return acceleration_[index];
  }

  /** @} */

  /** \name Getting and setting variable effort
   *  @{
   */

  /** \brief By default, if effort is never set or initialized, the state remembers that there is no effort set. This is
      useful to know when serializing or copying the state. If hasEffort() reports true, hasAccelerations() will
     certainly report false. */
  bool hasEffort() const
  {
    return has_effort_;
  }

  /** \brief Get raw access to the effort of the variables that make up this state. The values are in the same order as
   * reported by getVariableNames(). The area of memory overlaps with accelerations (effort and acceleration should not
   * be set at the same time) */
  double* getVariableEffort()
  {
    markEffort();
    return effort_;
  }

  /** \brief Get const raw access to the effort of the variables that make up this state. The values are in the same
   * order as reported by getVariableNames(). */
  const double* getVariableEffort() const
  {
    return effort_;
  }

  /** \brief Given an array with effort values for all variables, set those values as the effort in this state */
  void setVariableEffort(const double* effort)
  {
    has_effort_ = true;
    has_acceleration_ = false;
    // assume everything is in order in terms of array lengths (for efficiency reasons)
    memcpy(effort_, effort, robot_model_->getVariableCount() * sizeof(double));
  }

  /** \brief Given an array with effort values for all variables, set those values as the effort in this state */
  void setVariableEffort(const std::vector<double>& effort)
  {
    assert(robot_model_->getVariableCount() <= effort.size());  // checked only in debug mode
    setVariableEffort(&effort[0]);
  }

  /** \brief Set the effort of a set of variables. If unknown variable names are specified, an exception is thrown. */
  void setVariableEffort(const std::map<std::string, double>& variable_map);

  /** \brief Set the effort of a set of variables. If unknown variable names are specified, an exception is thrown.
      Additionally, \e missing_variables is filled with the names of the variables that are not set. */
  void setVariableEffort(const std::map<std::string, double>& variable_map,
                         std::vector<std::string>& missing_variables);

  /** \brief Set the effort of a set of variables. If unknown variable names are specified, an exception is thrown. */
  void setVariableEffort(const std::vector<std::string>& variable_names,
                         const std::vector<double>& variable_acceleration);

  /** \brief Set the effort of a variable. If an unknown variable name is specified, an exception is thrown. */
  void setVariableEffort(const std::string& variable, double value)
  {
    setVariableEffort(robot_model_->getVariableIndex(variable), value);
  }

  /** \brief Set the effort of a single variable. The variable is specified by its index (a value associated by the
   * RobotModel to each variable) */
  void setVariableEffort(int index, double value)
  {
    markEffort();
    effort_[index] = value;
  }

  /** \brief Get the effort of a particular variable. An exception is thrown if the variable is not known. */
  double getVariableEffort(const std::string& variable) const
  {
    return effort_[robot_model_->getVariableIndex(variable)];
  }

  /** \brief Get the effort of a particular variable. The variable is
      specified by its index. No checks are performed for the validity
      of the index passed  */
  double getVariableEffort(int index) const
  {
    return effort_[index];
  }

  /** \brief Invert velocity if present. */
  void invertVelocity();

  /** @} */

  /** \name Getting and setting joint positions, velocities, accelerations and effort
   *  @{
   */
  void setJointPositions(const std::string& joint_name, const double* position)
  {
    setJointPositions(robot_model_->getJointModel(joint_name), position);
  }

  void setJointPositions(const std::string& joint_name, const std::vector<double>& position)
  {
    setJointPositions(robot_model_->getJointModel(joint_name), &position[0]);
  }

  void setJointPositions(const JointModel* joint, const std::vector<double>& position)
  {
    setJointPositions(joint, &position[0]);
  }

  void setJointPositions(const JointModel* joint, const double* position)
  {
    memcpy(position_ + joint->getFirstVariableIndex(), position, joint->getVariableCount() * sizeof(double));
    markDirtyJointTransforms(joint);
    updateMimicJoint(joint);
  }

  void setJointPositions(const std::string& joint_name, const Eigen::Affine3d& transform)
  {
    setJointPositions(robot_model_->getJointModel(joint_name), transform);
  }

  void setJointPositions(const JointModel* joint, const Eigen::Affine3d& transform)
  {
    joint->computeVariablePositions(transform, position_ + joint->getFirstVariableIndex());
    markDirtyJointTransforms(joint);
    updateMimicJoint(joint);
  }

  void setJointVelocities(const JointModel* joint, const double* velocity)
  {
    has_velocity_ = true;
    memcpy(velocity_ + joint->getFirstVariableIndex(), velocity, joint->getVariableCount() * sizeof(double));
  }

  void setJointEfforts(const JointModel* joint, const double* effort);

  const double* getJointPositions(const std::string& joint_name) const
  {
    return getJointPositions(robot_model_->getJointModel(joint_name));
  }

  const double* getJointPositions(const JointModel* joint) const
  {
    return position_ + joint->getFirstVariableIndex();
  }

  const double* getJointVelocities(const std::string& joint_name) const
  {
    return getJointVelocities(robot_model_->getJointModel(joint_name));
  }

  const double* getJointVelocities(const JointModel* joint) const
  {
    return velocity_ + joint->getFirstVariableIndex();
  }

  const double* getJointAccelerations(const std::string& joint_name) const
  {
    return getJointAccelerations(robot_model_->getJointModel(joint_name));
  }

  const double* getJointAccelerations(const JointModel* joint) const
  {
    return acceleration_ + joint->getFirstVariableIndex();
  }

  const double* getJointEffort(const std::string& joint_name) const
  {
    return getJointEffort(robot_model_->getJointModel(joint_name));
  }

  const double* getJointEffort(const JointModel* joint) const
  {
    return effort_ + joint->getFirstVariableIndex();
  }

  /** @} */

  /** \name Getting and setting group positions
   *  @{
   */

  /** \brief Given positions for the variables that make up a group, in the order found in the group (including values
of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupPositions(const std::string& joint_group_name, const double* gstate)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupPositions(jmg, gstate);
  }

  /** \brief Given positions for the variables that make up a group, in the order found in the group (including values
of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupPositions(const std::string& joint_group_name, const std::vector<double>& gstate)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupPositions(jmg, &gstate[0]);
  }

  /** \brief Given positions for the variables that make up a group, in the order found in the group (including values
of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupPositions(const JointModelGroup* group, const std::vector<double>& gstate)
  {
    setJointGroupPositions(group, &gstate[0]);
  }

  /** \brief Given positions for the variables that make up a group, in the order found in the group (including values
of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupPositions(const JointModelGroup* group, const double* gstate);

  /** \brief Given positions for the variables that make up a group, in the order found in the group (including values
of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupPositions(const std::string& joint_group_name, const Eigen::VectorXd& values)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupPositions(jmg, values);
  }

  /** \brief Given positions for the variables that make up a group, in the order found in the group (including values
of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupPositions(const JointModelGroup* group, const Eigen::VectorXd& values);

  /** \brief For a given group, copy the position values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupPositions(const std::string& joint_group_name, std::vector<double>& gstate) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
    {
      gstate.resize(jmg->getVariableCount());
      copyJointGroupPositions(jmg, &gstate[0]);
    }
  }

  /** \brief For a given group, copy the position values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupPositions(const std::string& joint_group_name, double* gstate) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      copyJointGroupPositions(jmg, gstate);
  }

  /** \brief For a given group, copy the position values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupPositions(const JointModelGroup* group, std::vector<double>& gstate) const
  {
    gstate.resize(group->getVariableCount());
    copyJointGroupPositions(group, &gstate[0]);
  }

  /** \brief For a given group, copy the position values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupPositions(const JointModelGroup* group, double* gstate) const;

  /** \brief For a given group, copy the position values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupPositions(const std::string& joint_group_name, Eigen::VectorXd& values) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      copyJointGroupPositions(jmg, values);
  }

  /** \brief For a given group, copy the position values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupPositions(const JointModelGroup* group, Eigen::VectorXd& values) const;

  /** @} */

  /** \name Getting and setting group velocities
   *  @{
   */

  /** \brief Given velocities for the variables that make up a group, in the order found in the group (including values
of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupVelocities(const std::string& joint_group_name, const double* gstate)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupVelocities(jmg, gstate);
  }

  /** \brief Given velocities for the variables that make up a group, in the order found in the group (including values
of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupVelocities(const std::string& joint_group_name, const std::vector<double>& gstate)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupVelocities(jmg, &gstate[0]);
  }

  /** \brief Given velocities for the variables that make up a group, in the order found in the group (including values
of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupVelocities(const JointModelGroup* group, const std::vector<double>& gstate)
  {
    setJointGroupVelocities(group, &gstate[0]);
  }

  /** \brief Given velocities for the variables that make up a group, in the order found in the group (including values
of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupVelocities(const JointModelGroup* group, const double* gstate);

  /** \brief Given velocities for the variables that make up a group, in the order found in the group (including values
of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupVelocities(const std::string& joint_group_name, const Eigen::VectorXd& values)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupVelocities(jmg, values);
  }

  /** \brief Given velocities for the variables that make up a group, in the order found in the group (including values
of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupVelocities(const JointModelGroup* group, const Eigen::VectorXd& values);

  /** \brief For a given group, copy the velocity values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupVelocities(const std::string& joint_group_name, std::vector<double>& gstate) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
    {
      gstate.resize(jmg->getVariableCount());
      copyJointGroupVelocities(jmg, &gstate[0]);
    }
  }

  /** \brief For a given group, copy the velocity values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupVelocities(const std::string& joint_group_name, double* gstate) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      copyJointGroupVelocities(jmg, gstate);
  }

  /** \brief For a given group, copy the velocity values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupVelocities(const JointModelGroup* group, std::vector<double>& gstate) const
  {
    gstate.resize(group->getVariableCount());
    copyJointGroupVelocities(group, &gstate[0]);
  }

  /** \brief For a given group, copy the velocity values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupVelocities(const JointModelGroup* group, double* gstate) const;

  /** \brief For a given group, copy the velocity values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupVelocities(const std::string& joint_group_name, Eigen::VectorXd& values) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      copyJointGroupVelocities(jmg, values);
  }

  /** \brief For a given group, copy the velocity values of the variables that make up the group into another location,
   * in the order that the variables are found in the group. This is not necessarily a contiguous block of memory in the
   * RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupVelocities(const JointModelGroup* group, Eigen::VectorXd& values) const;

  /** @} */

  /** \name Getting and setting group accelerations
   *  @{
   */

  /** \brief Given accelerations for the variables that make up a group, in the order found in the group (including
values of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupAccelerations(const std::string& joint_group_name, const double* gstate)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupAccelerations(jmg, gstate);
  }

  /** \brief Given accelerations for the variables that make up a group, in the order found in the group (including
values of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupAccelerations(const std::string& joint_group_name, const std::vector<double>& gstate)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupAccelerations(jmg, &gstate[0]);
  }

  /** \brief Given accelerations for the variables that make up a group, in the order found in the group (including
values of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupAccelerations(const JointModelGroup* group, const std::vector<double>& gstate)
  {
    setJointGroupAccelerations(group, &gstate[0]);
  }

  /** \brief Given accelerations for the variables that make up a group, in the order found in the group (including
values of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupAccelerations(const JointModelGroup* group, const double* gstate);

  /** \brief Given accelerations for the variables that make up a group, in the order found in the group (including
values of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupAccelerations(const std::string& joint_group_name, const Eigen::VectorXd& values)
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      setJointGroupAccelerations(jmg, values);
  }

  /** \brief Given accelerations for the variables that make up a group, in the order found in the group (including
values of mimic joints), set those
as the new values that correspond to the group */
  void setJointGroupAccelerations(const JointModelGroup* group, const Eigen::VectorXd& values);

  /** \brief For a given group, copy the acceleration values of the variables that make up the group into another
   * location, in the order that the variables are found in the group. This is not necessarily a contiguous block of
   * memory in the RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupAccelerations(const std::string& joint_group_name, std::vector<double>& gstate) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
    {
      gstate.resize(jmg->getVariableCount());
      copyJointGroupAccelerations(jmg, &gstate[0]);
    }
  }

  /** \brief For a given group, copy the acceleration values of the variables that make up the group into another
   * location, in the order that the variables are found in the group. This is not necessarily a contiguous block of
   * memory in the RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupAccelerations(const std::string& joint_group_name, double* gstate) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      copyJointGroupAccelerations(jmg, gstate);
  }

  /** \brief For a given group, copy the acceleration values of the variables that make up the group into another
   * location, in the order that the variables are found in the group. This is not necessarily a contiguous block of
   * memory in the RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupAccelerations(const JointModelGroup* group, std::vector<double>& gstate) const
  {
    gstate.resize(group->getVariableCount());
    copyJointGroupAccelerations(group, &gstate[0]);
  }

  /** \brief For a given group, copy the acceleration values of the variables that make up the group into another
   * location, in the order that the variables are found in the group. This is not necessarily a contiguous block of
   * memory in the RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupAccelerations(const JointModelGroup* group, double* gstate) const;

  /** \brief For a given group, copy the acceleration values of the variables that make up the group into another
   * location, in the order that the variables are found in the group. This is not necessarily a contiguous block of
   * memory in the RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupAccelerations(const std::string& joint_group_name, Eigen::VectorXd& values) const
  {
    const JointModelGroup* jmg = robot_model_->getJointModelGroup(joint_group_name);
    if (jmg)
      copyJointGroupAccelerations(jmg, values);
  }

  /** \brief For a given group, copy the acceleration values of the variables that make up the group into another
   * location, in the order that the variables are found in the group. This is not necessarily a contiguous block of
   * memory in the RobotState itself, so we copy instead of returning a pointer.*/
  void copyJointGroupAccelerations(const JointModelGroup* group, Eigen::VectorXd& values) const;

  /** @} */

  /** \name Setting from Inverse Kinematics
   *  @{
   */

  /**
   * \brief Convert the frame of reference of the pose to that same frame as the IK solver expects
   * @param pose - the input to change
   * @param solver - a kin solver whose base frame is important to us
   * @return true if no error
   */
  bool setToIKSolverFrame(Eigen::Affine3d& pose, const kinematics::KinematicsBaseConstPtr& solver);

  /**
   * \brief Convert the frame of reference of the pose to that same frame as the IK solver expects
   * @param pose - the input to change
   * @param ik_frame - the name of frame of reference of base of ik solver
   * @return true if no error
   */
  bool setToIKSolverFrame(Eigen::Affine3d& pose, const std::string& ik_frame);

  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be
     set by computing inverse kinematics.
      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
      @param pose The pose the last link in the chain needs to achieve
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIK(const JointModelGroup* group, const geometry_msgs::Pose& pose, unsigned int attempts = 0,
                 double timeout = 0.0, const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
                 const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());

  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be
     set by computing inverse kinematics.
      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
      @param pose The pose the \e tip  link in the chain needs to achieve
      @param tip The name of the link the pose is specified for
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIK(const JointModelGroup* group, const geometry_msgs::Pose& pose, const std::string& tip,
                 unsigned int attempts = 0, double timeout = 0.0,
                 const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
                 const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());

  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be
     set by computing inverse kinematics.
      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
      @param pose The pose the last link in the chain needs to achieve
      @param tip The name of the link the pose is specified for
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt */
  bool setFromIK(const JointModelGroup* group, const Eigen::Affine3d& pose, unsigned int attempts = 0,
                 double timeout = 0.0, const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
                 const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());

  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be
     set by computing inverse kinematics.
      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
      @param pose The pose the last link in the chain needs to achieve
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIK(const JointModelGroup* group, const Eigen::Affine3d& pose, const std::string& tip,
                 unsigned int attempts = 0, double timeout = 0.0,
                 const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
                 const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());

  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be
     set by computing inverse kinematics.
      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
      @param pose The pose the last link in the chain needs to achieve
      @param tip The name of the frame for which IK is attempted.
      @param consistency_limits This specifies the desired distance between the solution and the seed state
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIK(const JointModelGroup* group, const Eigen::Affine3d& pose, const std::string& tip,
                 const std::vector<double>& consistency_limits, unsigned int attempts = 0, double timeout = 0.0,
                 const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
                 const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());

  /** \brief  Warning: This function inefficiently copies all transforms around.
      If the group consists of a set of sub-groups that are each a chain and a solver
      is available for each sub-group, then the joint values can be set by computing inverse kinematics.
      The poses are assumed to be in the reference frame of the kinematic model. The poses are assumed
      to be in the same order as the order of the sub-groups in this group. Returns true on success.
      @param poses The poses the last link in each chain needs to achieve
      @param tips The names of the frames for which IK is attempted.
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIK(const JointModelGroup* group, const EigenSTL::vector_Affine3d& poses,
                 const std::vector<std::string>& tips, unsigned int attempts = 0, double timeout = 0.0,
                 const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
                 const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());

  /** \brief Warning: This function inefficiently copies all transforms around.
      If the group consists of a set of sub-groups that are each a chain and a solver
      is available for each sub-group, then the joint values can be set by computing inverse kinematics.
      The poses are assumed to be in the reference frame of the kinematic model. The poses are assumed
      to be in the same order as the order of the sub-groups in this group. Returns true on success.
      @param poses The poses the last link in each chain needs to achieve
      @param tips The names of the frames for which IK is attempted.
      @param consistency_limits This specifies the desired distance between the solution and the seed state
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIK(const JointModelGroup* group, const EigenSTL::vector_Affine3d& poses,
                 const std::vector<std::string>& tips, const std::vector<std::vector<double> >& consistency_limits,
                 unsigned int attempts = 0, double timeout = 0.0,
                 const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
                 const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());

  /**
      \brief setFromIK for multiple poses and tips (end effectors) when no solver exists for the jmg that can solver for
      non-chain kinematics. In this case, we divide the group into subgroups and do IK solving individually
      @param poses The poses the last link in each chain needs to achieve
      @param tips The names of the frames for which IK is attempted.
      @param consistency_limits This specifies the desired distance between the solution and the seed state
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIKSubgroups(const JointModelGroup* group, const EigenSTL::vector_Affine3d& poses,
                          const std::vector<std::string>& tips,
                          const std::vector<std::vector<double> >& consistency_limits, unsigned int attempts = 0,
                          double timeout = 0.0,
                          const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn(),
                          const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());

  /** \brief Set the joint values from a Cartesian velocity applied during a time dt
   * @param group the group of joints this function operates on
   * @param twist a Cartesian velocity on the 'tip' frame
   * @param tip the frame for which the twist is given
   * @param dt a time interval (seconds)
   * @param st a secondary task computation function
   */
  bool setFromDiffIK(const JointModelGroup* group, const Eigen::VectorXd& twist, const std::string& tip, double dt,
                     const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn());

  /** \brief Set the joint values from a Cartesian velocity applied during a time dt
   * @param group the group of joints this function operates on
   * @param twist a Cartesian velocity on the 'tip' frame
   * @param tip the frame for which the twist is given
   * @param dt a time interval (seconds)
   * @param st a secondary task computation function
   */
  bool setFromDiffIK(const JointModelGroup* group, const geometry_msgs::Twist& twist, const std::string& tip, double dt,
                     const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn());

  /** \brief Compute the sequence of joint values that correspond to a straight Cartesian path for a particular group.

     The Cartesian path to be followed is specified as a direction of motion (\e direction, unit vector) for the origin
     The Cartesian path to be followed is specified as a direction of motion (\e direction, unit vector) for the origin
     of a robot link (\e link). The direction is assumed to be either in a global reference frame or in the local
     reference frame of the link. In the latter case (\e global_reference_frame is false) the \e direction is rotated
     accordingly. The link needs to move in a straight line, following the specified direction, for the desired \e
     distance. The resulting joint values are stored in the vector \e traj, one by one. The maximum distance in
     Cartesian space between consecutive points on the resulting path is specified in the \e MaxEEFStep struct which
     provides two fields: translation and rotation. If a \e validCallback is specified, this is passed to the internal
     call to setFromIK(). In case of IK failure, the computation of the path stops and the value returned corresponds to
     the distance that was computed and for which corresponding states were added to the path.  At the end of the
     function call, the state of the group corresponds to the last attempted Cartesian pose.

     During the computation of the trajectory, it is usually preferred if consecutive joint values do not 'jump' by a
     large amount in joint space, even if the Cartesian distance between the corresponding points is small as expected.
     To account for this, the \e jump_threshold struct is provided, which comprises three fields:
     \e jump_threshold_factor, \e revolute_jump_threshold and \e prismatic_jump_threshold.
     If either \e revolute_jump_threshold or \e prismatic_jump_threshold  are non-zero, we test for absolute jumps.
     If \e jump_threshold_factor is non-zero, we test for relative jumps. Otherwise (all params are zero), jump
     detection is disabled.

     For relative jump detection, the average joint-space distance between consecutive points in the trajectory is
     computed. If any individual joint-space motion delta is larger then this average distance by a factor of
     \e jump_threshold_factor, this step is considered a failure and the returned path is truncated up to just
     before the jump.

     For absolute jump thresholds, if any individual joint-space motion delta is larger then \e revolute_jump_threshold
     for revolute joints or \e prismatic_jump_threshold for prismatic joints then this step is considered a failure and
     the returned path is truncated up to just before the jump.*/
  double computeCartesianPath(const JointModelGroup* group, std::vector<RobotStatePtr>& traj, const LinkModel* link,
                              const Eigen::Vector3d& direction, bool global_reference_frame, double distance,
                              const MaxEEFStep& max_step, const JumpThreshold& jump_threshold,
                              const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
                              const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());

  double computeCartesianPath(const JointModelGroup* group, std::vector<RobotStatePtr>& traj, const LinkModel* link,
                              const Eigen::Vector3d& direction, bool global_reference_frame, double distance,
                              double max_step, double jump_threshold_factor,
                              const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
                              const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions())
  {
    return computeCartesianPath(group, traj, link, direction, global_reference_frame, distance,
                                MaxEEFStep(max_step, max_step), JumpThreshold(jump_threshold_factor), validCallback,
                                options);
  }

  /** \brief Compute the sequence of joint values that correspond to a straight Cartesian path, for a particular group.

     In contrast to the previous function, the Cartesian path is specified as a target frame to be reached (\e target)
     for the origin of a robot link (\e link). The target frame is assumed to be either in a global reference frame or
     in the local reference frame of the link. In the latter case (\e global_reference_frame is false) the \e target is
     rotated accordingly. All other comments from the previous function apply. */
  double computeCartesianPath(const JointModelGroup* group, std::vector<RobotStatePtr>& traj, const LinkModel* link,
                              const Eigen::Affine3d& target, bool global_reference_frame, const MaxEEFStep& max_step,
                              const JumpThreshold& jump_threshold,
                              const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
                              const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());

  double computeCartesianPath(const JointModelGroup* group, std::vector<RobotStatePtr>& traj, const LinkModel* link,
                              const Eigen::Affine3d& target, bool global_reference_frame, double max_step,
                              double jump_threshold_factor,
                              const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
                              const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions())
  {
    return computeCartesianPath(group, traj, link, target, global_reference_frame, MaxEEFStep(max_step),
                                JumpThreshold(jump_threshold_factor), validCallback, options);
  }

  /** \brief Compute the sequence of joint values that perform a general Cartesian path.

     In contrast to the previous functions, the Cartesian path is specified as a set of \e waypoints to be sequentially
     reached for the origin of a robot link (\e link). The waypoints are transforms given either in a global reference
     frame or in the local reference frame of the link at the immediately preceeding waypoint. The link needs to move
     in a straight line between two consecutive waypoints. All other comments apply. */
  double computeCartesianPath(const JointModelGroup* group, std::vector<RobotStatePtr>& traj, const LinkModel* link,
                              const EigenSTL::vector_Affine3d& waypoints, bool global_reference_frame,
                              const MaxEEFStep& max_step, const JumpThreshold& jump_threshold,
                              const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
                              const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());

  double computeCartesianPath(const JointModelGroup* group, std::vector<RobotStatePtr>& traj, const LinkModel* link,
                              const EigenSTL::vector_Affine3d& waypoints, bool global_reference_frame, double max_step,
                              double jump_threshold_factor,
                              const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
                              const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions())
  {
    return computeCartesianPath(group, traj, link, waypoints, global_reference_frame, MaxEEFStep(max_step),
                                JumpThreshold(jump_threshold_factor), validCallback, options);
  }

  /** \brief Tests joint space jumps of a trajectory.

     If \e jump_threshold_factor is non-zero, we test for relative jumps.
     If \e revolute_jump_threshold  or \e prismatic_jump_threshold are non-zero, we test for absolute jumps.
     Both tests can be combined. If all params are zero, jump detection is disabled.
     For relative jump detection, the average joint-space distance between consecutive points in the trajectory is
     computed. If any individual joint-space motion delta is larger then this average distance by a factor of
     \e jump_threshold_factor, this step is considered a failure and the returned path is truncated up to just
     before the jump.

     @param group The joint model group of the robot state.
     @param traj The trajectory that should be tested.
     @param jump_threshold The struct holding jump thresholds to determine if a joint space jump has occurred.
     @return The fraction of the trajectory that passed.
  */
  static double testJointSpaceJump(const JointModelGroup* group, std::vector<RobotStatePtr>& traj,
                                   const JumpThreshold& jump_threshold);

  /** \brief Tests for relative joint space jumps of the trajectory \e traj.

     First, the average distance between adjacent trajectory points is computed. If two adjacent trajectory points
     have distance > \e jump_threshold_factor * average, the trajectory is truncated at this point.

     @param group The joint model group of the robot state.
     @param traj The trajectory that should be tested.
     @param jump_threshold_factor The threshold to determine if a joint space jump has occurred .
     @return The fraction of the trajectory that passed.
  */
  static double testRelativeJointSpaceJump(const JointModelGroup* group, std::vector<RobotStatePtr>& traj,
                                           double jump_threshold_factor);

  /** \brief Tests for absolute joint space jumps of the trajectory \e traj.

     The joint-space difference between consecutive waypoints is computed for each active joint and compared to the
     absolute thresholds \e revolute_jump_threshold for revolute joints and \e prismatic_jump_threshold for prismatic
     joints. If these thresholds are exceeded, the trajectory is truncated.

     @param group The joint model group of the robot state.
     @param traj The trajectory that should be tested.
     @param revolute_jump_threshold Absolute joint-space threshold for revolute joints.
     @param prismatic_jump_threshold Absolute joint-space threshold for prismatic joints.
     @return The fraction of the trajectory that passed.
  */
  static double testAbsoluteJointSpaceJump(const JointModelGroup* group, std::vector<RobotStatePtr>& traj,
                                           double revolute_jump_threshold, double prismatic_jump_threshold);

  /** \brief Compute the Jacobian with reference to a particular point on a given link, for a specified group.
   * \param group The group to compute the Jacobian for
   * \param link_name The name of the link
   * \param reference_point_position The reference point position (with respect to the link specified in link_name)
   * \param jacobian The resultant jacobian
   * \param use_quaternion_representation Flag indicating if the Jacobian should use a quaternion representation
   * (default is false)
   * \return True if jacobian was successfully computed, false otherwise
   */
  bool getJacobian(const JointModelGroup* group, const LinkModel* link, const Eigen::Vector3d& reference_point_position,
                   Eigen::MatrixXd& jacobian, bool use_quaternion_representation = false) const;

  /** \brief Compute the Jacobian with reference to a particular point on a given link, for a specified group.
   * \param group The group to compute the Jacobian for
   * \param link_name The name of the link
   * \param reference_point_position The reference point position (with respect to the link specified in link_name)
   * \param jacobian The resultant jacobian
   * \param use_quaternion_representation Flag indicating if the Jacobian should use a quaternion representation
   * (default is false)
   * \return True if jacobian was successfully computed, false otherwise
   */
  bool getJacobian(const JointModelGroup* group, const LinkModel* link, const Eigen::Vector3d& reference_point_position,
                   Eigen::MatrixXd& jacobian, bool use_quaternion_representation = false)
  {
    updateLinkTransforms();
    return static_cast<const RobotState*>(this)->getJacobian(group, link, reference_point_position, jacobian,
                                                             use_quaternion_representation);
  }

  /** \brief Compute the Jacobian with reference to the last link of a specified group. If the group is not a chain, an
   * exception is thrown.
   * \param group The group to compute the Jacobian for
   * \param reference_point_position The reference point position (with respect to the link specified in link_name)
   * \return The computed Jacobian.
   */
  Eigen::MatrixXd getJacobian(const JointModelGroup* group,
                              const Eigen::Vector3d& reference_point_position = Eigen::Vector3d(0.0, 0.0, 0.0)) const;

  /** \brief Compute the Jacobian with reference to the last link of a specified group. If the group is not a chain, an
   * exception is thrown.
   * \param group The group to compute the Jacobian for
   * \param reference_point_position The reference point position (with respect to the link specified in link_name)
   * \return The computed Jacobian.
   */
  Eigen::MatrixXd getJacobian(const JointModelGroup* group,
                              const Eigen::Vector3d& reference_point_position = Eigen::Vector3d(0.0, 0.0, 0.0))
  {
    updateLinkTransforms();
    return static_cast<const RobotState*>(this)->getJacobian(group, reference_point_position);
  }

  /** \brief Given a twist for a particular link (\e tip), compute the corresponding velocity for every variable and
   * store it in \e qdot */
  void computeVariableVelocity(const JointModelGroup* jmg, Eigen::VectorXd& qdot, const Eigen::VectorXd& twist,
                               const LinkModel* tip) const;

  /** \brief Given a twist for a particular link (\e tip), compute the corresponding velocity for every variable and
   * store it in \e qdot */
  void computeVariableVelocity(const JointModelGroup* jmg, Eigen::VectorXd& qdot, const Eigen::VectorXd& twist,
                               const LinkModel* tip)
  {
    updateLinkTransforms();
    static_cast<const RobotState*>(this)->computeVariableVelocity(jmg, qdot, twist, tip);
  }

  /** \brief Given the velocities for the variables in this group (\e qdot) and an amount of time (\e dt),
      update the current state using the Euler forward method. If the constraint specified is satisfied, return true,
     otherwise return false. */
  bool integrateVariableVelocity(const JointModelGroup* jmg, const Eigen::VectorXd& qdot, double dt,
                                 const GroupStateValidityCallbackFn& constraint = GroupStateValidityCallbackFn());

  /** @} */

  /** \name Getting and setting whole states
   *  @{
   */

  void setVariableValues(const sensor_msgs::JointState& msg)
  {
    if (!msg.position.empty())
      setVariablePositions(msg.name, msg.position);
    if (!msg.velocity.empty())
      setVariableVelocities(msg.name, msg.velocity);
  }

  /** \brief Set all joints to their default positions.
       The default position is 0, or if that is not within bounds then half way
       between min and max bound.  */
  void setToDefaultValues();

  /** \brief Set the joints in \e group to the position \e name defined in the SRDF */
  bool setToDefaultValues(const JointModelGroup* group, const std::string& name);

  /** \brief Set all joints to random values.  Values will be within default bounds. */
  void setToRandomPositions();

  /** \brief Set all joints in \e group to random values.  Values will be within default bounds. */
  void setToRandomPositions(const JointModelGroup* group);

  /** \brief Set all joints in \e group to random values using a specified random number generator.
      Values will be within default bounds. */
  void setToRandomPositions(const JointModelGroup* group, random_numbers::RandomNumberGenerator& rng);

  /** \brief Set all joints in \e group to random values near the value in \near.
   *  \e distance is the maximum amount each joint value will vary from the
   *  corresponding value in \e near.  \distance represents meters for
   *  prismatic/postitional joints and radians for revolute/orientation joints.
   *  Resulting values are clamped within default bounds. */
  void setToRandomPositionsNearBy(const JointModelGroup* group, const RobotState& near, double distance);

  /** \brief Set all joints in \e group to random values near the value in \near.
   *  \e distances \b MUST have the same size as \c
   *  group.getActiveJointModels().  Each value in \e distances is the maximum
   *  amount the corresponding active joint in \e group will vary from the
   *  corresponding value in \e near.  \distance represents meters for
   *  prismatic/postitional joints and radians for revolute/orientation joints.
   *  Resulting values are clamped within default bounds. */
  void setToRandomPositionsNearBy(const JointModelGroup* group, const RobotState& near,
                                  const std::vector<double>& distances);

  /** @} */

  /** \name Updating and getting transforms
   *  @{
   */

  /** \brief Update the transforms for the collision bodies. This call is needed before calling collision checking.
      If updating link transforms or joint transorms is needed, the corresponding updates are also triggered. */
  void updateCollisionBodyTransforms();

  /** \brief Update the reference frame transforms for links. This call is needed before using the transforms of links
   * for coordinate transforms. */
  void updateLinkTransforms();

  /** \brief Update all transforms. */
  void update(bool force = false);

  /** \brief Update the state after setting a particular link to the input global transform pose.

      This "warps" the given link to the given pose, neglecting the joint values of its parent joint.
      The link transforms of link and all its descendants are updated, but not marked as dirty,
      although they do not match the joint values anymore!
      Collision body transforms are not yet updated, but marked dirty only.
      Use update(false) or updateCollisionBodyTransforms() to update them as well.
   */
  void updateStateWithLinkAt(const std::string& link_name, const Eigen::Affine3d& transform, bool backward = false)
  {
    updateStateWithLinkAt(robot_model_->getLinkModel(link_name), transform, backward);
  }

  /** \brief Update the state after setting a particular link to the input global transform pose.*/
  void updateStateWithLinkAt(const LinkModel* link, const Eigen::Affine3d& transform, bool backward = false);

  const Eigen::Affine3d& getGlobalLinkTransform(const std::string& link_name)
  {
    return getGlobalLinkTransform(robot_model_->getLinkModel(link_name));
  }

  const Eigen::Affine3d& getGlobalLinkTransform(const LinkModel* link)
  {
    updateLinkTransforms();
    return global_link_transforms_[link->getLinkIndex()];
  }

  const Eigen::Affine3d& getCollisionBodyTransforms(const std::string& link_name, std::size_t index)
  {
    return getCollisionBodyTransform(robot_model_->getLinkModel(link_name), index);
  }

  const Eigen::Affine3d& getCollisionBodyTransform(const LinkModel* link, std::size_t index)
  {
    updateCollisionBodyTransforms();
    return global_collision_body_transforms_[link->getFirstCollisionBodyTransformIndex() + index];
  }

  const Eigen::Affine3d& getJointTransform(const std::string& joint_name)
  {
    return getJointTransform(robot_model_->getJointModel(joint_name));
  }

  const Eigen::Affine3d& getJointTransform(const JointModel* joint)
  {
    const int idx = joint->getJointIndex();
    unsigned char& dirty = dirty_joint_transforms_[idx];
    if (dirty)
    {
      joint->computeTransform(position_ + joint->getFirstVariableIndex(), variable_joint_transforms_[idx]);
      dirty = 0;
    }
    return variable_joint_transforms_[idx];
  }

  const Eigen::Affine3d& getGlobalLinkTransform(const std::string& link_name) const
  {
    return getGlobalLinkTransform(robot_model_->getLinkModel(link_name));
  }

  const Eigen::Affine3d& getGlobalLinkTransform(const LinkModel* link) const
  {
    BOOST_VERIFY(checkLinkTransforms());
    return global_link_transforms_[link->getLinkIndex()];
  }

  const Eigen::Affine3d& getCollisionBodyTransform(const std::string& link_name, std::size_t index) const
  {
    return getCollisionBodyTransform(robot_model_->getLinkModel(link_name), index);
  }

  const Eigen::Affine3d& getCollisionBodyTransform(const LinkModel* link, std::size_t index) const
  {
    BOOST_VERIFY(checkCollisionTransforms());
    return global_collision_body_transforms_[link->getFirstCollisionBodyTransformIndex() + index];
  }

  const Eigen::Affine3d& getJointTransform(const std::string& joint_name) const
  {
    return getJointTransform(robot_model_->getJointModel(joint_name));
  }

  const Eigen::Affine3d& getJointTransform(const JointModel* joint) const
  {
    BOOST_VERIFY(checkJointTransforms(joint));
    return variable_joint_transforms_[joint->getJointIndex()];
  }

  bool dirtyJointTransform(const JointModel* joint) const
  {
    return dirty_joint_transforms_[joint->getJointIndex()];
  }

  bool dirtyLinkTransforms() const
  {
    return dirty_link_transforms_;
  }

  bool dirtyCollisionBodyTransforms() const
  {
    return dirty_link_transforms_ || dirty_collision_body_transforms_;
  }

  /** \brief Returns true if anything in this state is dirty */
  bool dirty() const
  {
    return dirtyCollisionBodyTransforms();
  }

  /** @} */

  /** \name Computing distances
   *  @{
   */

  double distance(const RobotState& other) const
  {
    return robot_model_->distance(position_, other.getVariablePositions());
  }

  double distance(const RobotState& other, const JointModelGroup* joint_group) const;

  double distance(const RobotState& other, const JointModel* joint) const
  {
    const int idx = joint->getFirstVariableIndex();
    return joint->distance(position_ + idx, other.position_ + idx);
  }

  /** \brief Interpolate from this state towards state \e to, at time \e t in [0,1].
      The result is stored in \e state, mimic joints are correctly updated and flags are set
      so that FK is recomputed when needed. */
  void interpolate(const RobotState& to, double t, RobotState& state) const;

  /** \brief Interpolate from this state towards \e to, at time \e t in [0,1], but only for the joints in the
      specified group. If mimic joints need to be updated, they are updated accordingly. Flags are set so that FK
      computation is triggered when needed. */
  void interpolate(const RobotState& to, double t, RobotState& state, const JointModelGroup* joint_group) const;

  /** \brief Update \e state by interpolating form this state towards \e to, at time \e t in [0,1] but only for
      the joint \e joint. If there are joints that mimic this joint, they are updated. Flags are set so that
      FK computation is triggered as needed. */
  void interpolate(const RobotState& to, double t, RobotState& state, const JointModel* joint) const
  {
    const int idx = joint->getFirstVariableIndex();
    joint->interpolate(position_ + idx, to.position_ + idx, t, state.position_ + idx);
    state.markDirtyJointTransforms(joint);
    state.updateMimicJoint(joint);
  }

  void enforceBounds();
  void enforceBounds(const JointModelGroup* joint_group);
  void enforceBounds(const JointModel* joint)
  {
    enforcePositionBounds(joint);
    if (has_velocity_)
      enforceVelocityBounds(joint);
  }
  void enforcePositionBounds(const JointModel* joint)
  {
    if (joint->enforcePositionBounds(position_ + joint->getFirstVariableIndex()))
    {
      markDirtyJointTransforms(joint);
      updateMimicJoint(joint);
    }
  }
  void enforceVelocityBounds(const JointModel* joint)
  {
    joint->enforceVelocityBounds(velocity_ + joint->getFirstVariableIndex());
  }

  bool satisfiesBounds(double margin = 0.0) const;
  bool satisfiesBounds(const JointModelGroup* joint_group, double margin = 0.0) const;
  bool satisfiesBounds(const JointModel* joint, double margin = 0.0) const
  {
    return satisfiesPositionBounds(joint, margin) && (!has_velocity_ || satisfiesVelocityBounds(joint, margin));
  }
  bool satisfiesPositionBounds(const JointModel* joint, double margin = 0.0) const
  {
    return joint->satisfiesPositionBounds(getJointPositions(joint), margin);
  }
  bool satisfiesVelocityBounds(const JointModel* joint, double margin = 0.0) const
  {
    return joint->satisfiesVelocityBounds(getJointVelocities(joint), margin);
  }

  /** \brief Get the minimm distance from this state to the bounds.
      The minimum distance and the joint for which this minimum is achieved are returned. */
  std::pair<double, const JointModel*> getMinDistanceToPositionBounds() const;

  /** \brief Get the minimm distance from a group in this state to the bounds.
      The minimum distance and the joint for which this minimum is achieved are returned. */
  std::pair<double, const JointModel*> getMinDistanceToPositionBounds(const JointModelGroup* group) const;

  /** \brief Get the minimm distance from a set of joints in the state to the bounds.
      The minimum distance and the joint for which this minimum is achieved are returned. */
  std::pair<double, const JointModel*>
  getMinDistanceToPositionBounds(const std::vector<const JointModel*>& joints) const;

  /**
   * \brief Check that the time to move between two waypoints is sufficient given velocity limits and time step
   * \param other - robot state to compare joint positions against
   * \param group - planning group to compare joint positions against
   * \param dt - time step between the two points
   */
  bool isValidVelocityMove(const RobotState& other, const JointModelGroup* group, double dt) const;

  /** @} */

  /** \name Managing attached bodies
   *  @{
   */

  /** \brief Add an attached body to this state. Ownership of the
   * memory for the attached body is assumed by the state.
   *
   * This only adds the given body to this RobotState
   * instance.  It does not change anything about other
   * representations of the object elsewhere in the system.  So if the
   * body represents an object in a collision_detection::World (like
   * from a planning_scene::PlanningScene), you will likely need to remove the
   * corresponding object from that world to avoid having collisions
   * detected against it.
   *
   * \note This version of the function (taking an AttachedBody
   * pointer) does not copy the AttachedBody object, it just uses it
   * directly.  The AttachedBody object stores its position data
   * internally.  This means you should <b>never attach a single
   * AttachedBody instance to multiple RobotState instances</b>, or
   * the body positions will get corrupted.  You need to make a fresh
   * copy of the AttachedBody object for each RobotState you attach it
   * to.*/
  void attachBody(AttachedBody* attached_body);

  /** @brief Add an attached body to a link
   * @param id The string id associated with the attached body
   * @param shapes The shapes that make up the attached body
   * @param attach_trans The desired transform between this link and the attached body
   * @param touch_links The set of links that the attached body is allowed to touch
   * @param link_name The link to attach to
   *
   * This only adds the given body to this RobotState
   * instance.  It does not change anything about other
   * representations of the object elsewhere in the system.  So if the
   * body represents an object in a collision_detection::World (like
   * from a planning_scene::PlanningScene), you will likely need to remove the
   * corresponding object from that world to avoid having collisions
   * detected against it. */
  void attachBody(const std::string& id, const std::vector<shapes::ShapeConstPtr>& shapes,
                  const EigenSTL::vector_Affine3d& attach_trans, const std::set<std::string>& touch_links,
                  const std::string& link_name,
                  const trajectory_msgs::JointTrajectory& detach_posture = trajectory_msgs::JointTrajectory());

  /** @brief Add an attached body to a link
   * @param id The string id associated with the attached body
   * @param shapes The shapes that make up the attached body
   * @param attach_trans The desired transform between this link and the attached body
   * @param touch_links The set of links that the attached body is allowed to touch
   * @param link_name The link to attach to
   *
   * This only adds the given body to this RobotState
   * instance.  It does not change anything about other
   * representations of the object elsewhere in the system.  So if the
   * body represents an object in a collision_detection::World (like
   * from a planning_scene::PlanningScene), you will likely need to remove the
   * corresponding object from that world to avoid having collisions
   * detected against it. */
  void attachBody(const std::string& id, const std::vector<shapes::ShapeConstPtr>& shapes,
                  const EigenSTL::vector_Affine3d& attach_trans, const std::vector<std::string>& touch_links,
                  const std::string& link_name,
                  const trajectory_msgs::JointTrajectory& detach_posture = trajectory_msgs::JointTrajectory())
  {
    std::set<std::string> touch_links_set(touch_links.begin(), touch_links.end());
    attachBody(id, shapes, attach_trans, touch_links_set, link_name, detach_posture);
  }

  /** \brief Get all bodies attached to the model corresponding to this state */
  void getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies) const;

  /** \brief Get all bodies attached to a particular group the model corresponding to this state */
  void getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies, const JointModelGroup* lm) const;

  /** \brief Get all bodies attached to a particular link in the model corresponding to this state */
  void getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies, const LinkModel* lm) const;

  /** \brief Remove the attached body named \e id. Return false if the object was not found (and thus not removed).
   * Return true on success. */
  bool clearAttachedBody(const std::string& id);

  /** \brief Clear the bodies attached to a specific link */
  void clearAttachedBodies(const LinkModel* link);

  /** \brief Clear the bodies attached to a specific group */
  void clearAttachedBodies(const JointModelGroup* group);

  /** \brief Clear all attached bodies. This calls delete on the AttachedBody instances, if needed. */
  void clearAttachedBodies();

  /** \brief Get the attached body named \e name. Return NULL if not found. */
  const AttachedBody* getAttachedBody(const std::string& name) const;

  /** \brief Check if an attached body named \e id exists in this state */
  bool hasAttachedBody(const std::string& id) const;

  void setAttachedBodyUpdateCallback(const AttachedBodyCallback& callback);
  /** @} */

  /** \brief Compute an axis-aligned bounding box that contains the current state.
      The format for \e aabb is (minx, maxx, miny, maxy, minz, maxz) */
  void computeAABB(std::vector<double>& aabb) const;

  /** \brief Compute an axis-aligned bounding box that contains the current state.
      The format for \e aabb is (minx, maxx, miny, maxy, minz, maxz) */
  void computeAABB(std::vector<double>& aabb)
  {
    updateLinkTransforms();
    static_cast<const RobotState*>(this)->computeAABB(aabb);
  }

  /** \brief Return the instance of a random number generator */
  random_numbers::RandomNumberGenerator& getRandomNumberGenerator()
  {
    if (!rng_)
      rng_ = new random_numbers::RandomNumberGenerator();
    return *rng_;
  }

  /** \brief Get the transformation matrix from the model frame to the frame identified by \e id */
  const Eigen::Affine3d& getFrameTransform(const std::string& id);

  /** \brief Get the transformation matrix from the model frame to the frame identified by \e id */
  const Eigen::Affine3d& getFrameTransform(const std::string& id) const;

  /** \brief Check if a transformation matrix from the model frame to frame \e id is known */
  bool knowsFrameTransform(const std::string& id) const;

  /** @brief Get a MarkerArray that fully describes the robot markers for a given robot.
   *  @param arr The returned marker array
   *  @param link_names The list of link names for which the markers should be created.
   *  @param color The color for the marker
   *  @param ns The namespace for the markers
   *  @param dur The ros::Duration for which the markers should stay visible
   */
  void getRobotMarkers(visualization_msgs::MarkerArray& arr, const std::vector<std::string>& link_names,
                       const std_msgs::ColorRGBA& color, const std::string& ns, const ros::Duration& dur,
                       bool include_attached = false) const;

  /** @brief Get a MarkerArray that fully describes the robot markers for a given robot. Update the state first.
   *  @param arr The returned marker array
   *  @param link_names The list of link names for which the markers should be created.
   *  @param color The color for the marker
   *  @param ns The namespace for the markers
   *  @param dur The ros::Duration for which the markers should stay visible
   */
  void getRobotMarkers(visualization_msgs::MarkerArray& arr, const std::vector<std::string>& link_names,
                       const std_msgs::ColorRGBA& color, const std::string& ns, const ros::Duration& dur,
                       bool include_attached = false)
  {
    updateCollisionBodyTransforms();
    static_cast<const RobotState*>(this)->getRobotMarkers(arr, link_names, color, ns, dur, include_attached);
  }

  /** @brief Get a MarkerArray that fully describes the robot markers for a given robot.
   *  @param arr The returned marker array
   *  @param link_names The list of link names for which the markers should be created.
   */
  void getRobotMarkers(visualization_msgs::MarkerArray& arr, const std::vector<std::string>& link_names,
                       bool include_attached = false) const;

  /** @brief Get a MarkerArray that fully describes the robot markers for a given robot. Update the state first.
   *  @param arr The returned marker array
   *  @param link_names The list of link names for which the markers should be created.
   */
  void getRobotMarkers(visualization_msgs::MarkerArray& arr, const std::vector<std::string>& link_names,
                       bool include_attached = false)
  {
    updateCollisionBodyTransforms();
    static_cast<const RobotState*>(this)->getRobotMarkers(arr, link_names, include_attached);
  }

  void printStatePositions(std::ostream& out = std::cout) const;

  void printStateInfo(std::ostream& out = std::cout) const;

  void printTransforms(std::ostream& out = std::cout) const;

  void printTransform(const Eigen::Affine3d& transform, std::ostream& out = std::cout) const;

  void printDirtyInfo(std::ostream& out = std::cout) const;

  std::string getStateTreeString(const std::string& prefix = "") const;

private:
  void allocMemory();

  void copyFrom(const RobotState& other);

  void markDirtyJointTransforms(const JointModel* joint)
  {
    dirty_joint_transforms_[joint->getJointIndex()] = 1;
    dirty_link_transforms_ =
        dirty_link_transforms_ == NULL ? joint : robot_model_->getCommonRoot(dirty_link_transforms_, joint);
  }

  void markDirtyJointTransforms(const JointModelGroup* group)
  {
    const std::vector<const JointModel*>& jm = group->getActiveJointModels();
    for (std::size_t i = 0; i < jm.size(); ++i)
      dirty_joint_transforms_[jm[i]->getJointIndex()] = 1;
    dirty_link_transforms_ = dirty_link_transforms_ == NULL ?
                                 group->getCommonRoot() :
                                 robot_model_->getCommonRoot(dirty_link_transforms_, group->getCommonRoot());
  }

  void markVelocity();
  void markAcceleration();
  void markEffort();

  void updateMimicJoint(const JointModel* joint)
  {
    const std::vector<const JointModel*>& mim = joint->getMimicRequests();
    double v = position_[joint->getFirstVariableIndex()];
    for (std::size_t i = 0; i < mim.size(); ++i)
    {
      position_[mim[i]->getFirstVariableIndex()] = mim[i]->getMimicFactor() * v + mim[i]->getMimicOffset();
      markDirtyJointTransforms(mim[i]);
    }
  }

  /** \brief Update a set of joints that are certain to be mimicking other joints */
  /* use updateMimicJoints() instead, which also marks joints dirty */
  MOVEIT_DEPRECATED void updateMimicJoint(const std::vector<const JointModel*>& mim)
  {
    for (std::size_t i = 0; i < mim.size(); ++i)
    {
      const int fvi = mim[i]->getFirstVariableIndex();
      position_[fvi] =
          mim[i]->getMimicFactor() * position_[mim[i]->getMimic()->getFirstVariableIndex()] + mim[i]->getMimicOffset();
      // Only mark joint transform dirty, but not the associated link transform
      // as this function is always used in combination of
      // updateMimicJoint(group->getMimicJointModels()) + markDirtyJointTransforms(group);
      dirty_joint_transforms_[mim[i]->getJointIndex()] = 1;
    }
  }

  /** \brief Update all mimic joints within group */
  void updateMimicJoints(const JointModelGroup* group)
  {
    for (const JointModel* jm : group->getMimicJointModels())
    {
      const int fvi = jm->getFirstVariableIndex();
      position_[fvi] = jm->getMimicFactor() * position_[jm->getMimic()->getFirstVariableIndex()] + jm->getMimicOffset();
      markDirtyJointTransforms(jm);
    }
    markDirtyJointTransforms(group);
  }

  void updateLinkTransformsInternal(const JointModel* start);

  void getMissingKeys(const std::map<std::string, double>& variable_map,
                      std::vector<std::string>& missing_variables) const;
  void getStateTreeJointString(std::ostream& ss, const JointModel* jm, const std::string& pfx0, bool last) const;

  /** \brief This function is only called in debug mode */
  bool checkJointTransforms(const JointModel* joint) const;

  /** \brief This function is only called in debug mode */
  bool checkLinkTransforms() const;

  /** \brief This function is only called in debug mode */
  bool checkCollisionTransforms() const;

  RobotModelConstPtr robot_model_;
  void* memory_;

  double* position_;
  double* velocity_;
  double* acceleration_;
  double* effort_;
  bool has_velocity_;
  bool has_acceleration_;
  bool has_effort_;

  const JointModel* dirty_link_transforms_;
  const JointModel* dirty_collision_body_transforms_;

  Eigen::Affine3d* variable_joint_transforms_;         // this points to an element in transforms_, so it is aligned
  Eigen::Affine3d* global_link_transforms_;            // this points to an element in transforms_, so it is aligned
  Eigen::Affine3d* global_collision_body_transforms_;  // this points to an element in transforms_, so it is aligned
  unsigned char* dirty_joint_transforms_;

  /** \brief All attached bodies that are part of this state, indexed by their name */
  std::map<std::string, AttachedBody*> attached_body_map_;

  /** \brief This event is called when there is a change in the attached bodies for this state;
      The event specifies the body that changed and whether it was just attached or about to be detached. */
  AttachedBodyCallback attached_body_update_callback_;

  /** \brief For certain operations a state needs a random number generator. However, it may be slightly expensive
      to allocate the random number generator if many state instances are generated. For this reason, the generator
      is allocated on a need basis, by the getRandomNumberGenerator() function. Never use the rng_ member directly, but
     call
      getRandomNumberGenerator() instead. */
  random_numbers::RandomNumberGenerator* rng_;
};

/** \brief Operator overload for printing variable bounds to a stream */
std::ostream& operator<<(std::ostream& out, const RobotState& s);
}
}

#endif
