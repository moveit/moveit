/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#ifndef TESTDATA_LOADER_H
#define TESTDATA_LOADER_H

#include <string>

#include <moveit/robot_model/robot_model.h>

#include "jointconfiguration.h"
#include "cartesianconfiguration.h"
#include "command_types_typedef.h"
#include "sequence.h"
#include "gripper.h"

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Abstract base class describing the interface to access
 * test data like robot poses and robot commands.
 */
class TestdataLoader
{
public:
  TestdataLoader() = default;

  TestdataLoader(moveit::core::RobotModelConstPtr robot_model) : robot_model_(robot_model)
  {
  }

  virtual ~TestdataLoader() = default;

public:
  void setRobotModel(moveit::core::RobotModelConstPtr robot_model);

  virtual JointConfiguration getJoints(const std::string& pos_name, const std::string& group_name) const = 0;

  virtual CartesianConfiguration getPose(const std::string& pos_name, const std::string& group_name) const = 0;

  /**
   * @brief Returns the command with the specified name from the test data.
   */
  virtual PtpJoint getPtpJoint(const std::string& cmd_name) const = 0;
  virtual PtpCart getPtpCart(const std::string& cmd_name) const = 0;
  virtual PtpJointCart getPtpJointCart(const std::string& cmd_name) const = 0;

  /**
   * @brief Returns the command with the specified name from the test data.
   */
  virtual LinJoint getLinJoint(const std::string& cmd_name) const = 0;
  virtual LinCart getLinCart(const std::string& cmd_name) const = 0;
  virtual LinJointCart getLinJointCart(const std::string& cmd_name) const = 0;

  /**
   * @brief Returns the command with the specified name from the test data.
   */
  virtual CircCenterCart getCircCartCenterCart(const std::string& cmd_name) const = 0;
  virtual CircJointCenterCart getCircJointCenterCart(const std::string& cmd_name) const = 0;
  virtual CircInterimCart getCircCartInterimCart(const std::string& cmd_name) const = 0;
  virtual CircJointInterimCart getCircJointInterimCart(const std::string& cmd_name) const = 0;

  /**
   * @brief Returns the command with the specified name from the test data.
   */
  virtual Sequence getSequence(const std::string& cmd_name) const = 0;

  /**
   * @brief Returns the command with the specified name from the test data.
   */
  virtual Gripper getGripper(const std::string& cmd_name) const = 0;

protected:
  moveit::core::RobotModelConstPtr robot_model_;
};

inline void TestdataLoader::setRobotModel(moveit::core::RobotModelConstPtr robot_model)
{
  robot_model_ = robot_model;
}

using TestdataLoaderUPtr = std::unique_ptr<TestdataLoader>;
}

#endif  // TESTDATA_LOADER_H
