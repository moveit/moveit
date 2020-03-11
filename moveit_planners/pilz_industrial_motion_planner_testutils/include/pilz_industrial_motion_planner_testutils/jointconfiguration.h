/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef JOINTCONFIGURATION_H
#define JOINTCONFIGURATION_H

#include <string>
#include <vector>
#include <functional>
#include <stdexcept>

#include <sensor_msgs/JointState.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include "robotconfiguration.h"

namespace pilz_industrial_motion_planner_testutils
{
class JointConfigurationException : public std::runtime_error
{
public:
  JointConfigurationException(const std::string error_desc) : std::runtime_error(error_desc)
  {
  }
};

using CreateJointNameFunc = std::function<std::string(const size_t&)>;

/**
 * @brief Class to define a robot configuration in space with the help
 * of joint values.
 */
class JointConfiguration : public RobotConfiguration
{
public:
  JointConfiguration();

  JointConfiguration(const std::string& group_name, const std::vector<double>& config,
                     CreateJointNameFunc&& create_joint_name_func);

  JointConfiguration(const std::string& group_name, const std::vector<double>& config,
                     const moveit::core::RobotModelConstPtr& robot_model);

public:
  void setJoint(const size_t index, const double value);
  double getJoint(const size_t index) const;
  const std::vector<double> getJoints() const;

  size_t size() const;

  moveit_msgs::Constraints toGoalConstraints() const override;
  moveit_msgs::RobotState toMoveitMsgsRobotState() const override;

  sensor_msgs::JointState toSensorMsg() const;

  robot_state::RobotState toRobotState() const;

  void setCreateJointNameFunc(CreateJointNameFunc create_joint_name_func);

private:
  moveit_msgs::RobotState toMoveitMsgsRobotStateWithoutModel() const;
  moveit_msgs::RobotState toMoveitMsgsRobotStateWithModel() const;

  moveit_msgs::Constraints toGoalConstraintsWithoutModel() const;
  moveit_msgs::Constraints toGoalConstraintsWithModel() const;

private:
  //! Joint positions
  std::vector<double> joints_;

  CreateJointNameFunc create_joint_name_func_;
};

std::ostream& operator<<(std::ostream&, const JointConfiguration&);

inline moveit_msgs::Constraints JointConfiguration::toGoalConstraints() const
{
  return robot_model_ ? toGoalConstraintsWithModel() : toGoalConstraintsWithoutModel();
}

inline moveit_msgs::RobotState JointConfiguration::toMoveitMsgsRobotState() const
{
  return robot_model_ ? toMoveitMsgsRobotStateWithModel() : toMoveitMsgsRobotStateWithoutModel();
}

inline void JointConfiguration::setJoint(const size_t index, const double value)
{
  joints_.at(index) = value;
}

inline double JointConfiguration::getJoint(const size_t index) const
{
  return joints_.at(index);
}

inline const std::vector<double> JointConfiguration::getJoints() const
{
  return joints_;
}

inline size_t JointConfiguration::size() const
{
  return joints_.size();
}

inline void JointConfiguration::setCreateJointNameFunc(CreateJointNameFunc create_joint_name_func)
{
  create_joint_name_func_ = create_joint_name_func;
}
}

#endif  // JOINTCONFIGURATION_H
