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

#include "pilz_industrial_motion_planner_testutils/cartesianconfiguration.h"

#include <stdexcept>

namespace pilz_industrial_motion_planner_testutils
{
CartesianConfiguration::CartesianConfiguration() : RobotConfiguration()
{
}

CartesianConfiguration::CartesianConfiguration(const std::string& group_name, const std::string& link_name,
                                               const std::vector<double>& config)
  : RobotConfiguration(group_name), link_name_(link_name), pose_(toPose(config))
{
}

CartesianConfiguration::CartesianConfiguration(const std::string& group_name, const std::string& link_name,
                                               const std::vector<double>& config,
                                               const moveit::core::RobotModelConstPtr& robot_model)
  : RobotConfiguration(group_name, robot_model), link_name_(link_name), pose_(toPose(config))
{
  if (robot_model && (!robot_model_->hasLinkModel(link_name_)))
  {
    std::string msg{ "Link \"" };
    msg.append(link_name).append("\" not known to robot model");
    throw std::invalid_argument(msg);
  }

  if (robot_model && (!robot_state::RobotState(robot_model_).knowsFrameTransform(link_name_)))
  {
    std::string msg{ "Tranform of \"" };
    msg.append(link_name).append("\" is unknown");
    throw std::invalid_argument(msg);
  }
}

geometry_msgs::Pose CartesianConfiguration::toPose(const std::vector<double>& pose)
{
  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = pose.at(0);
  pose_msg.position.y = pose.at(1);
  pose_msg.position.z = pose.at(2);
  pose_msg.orientation.x = pose.at(3);
  pose_msg.orientation.y = pose.at(4);
  pose_msg.orientation.z = pose.at(5);
  pose_msg.orientation.w = pose.at(6);

  return pose_msg;
}

geometry_msgs::PoseStamped CartesianConfiguration::toStampedPose(const geometry_msgs::Pose& pose)
{
  geometry_msgs::PoseStamped pose_stamped_msg;
  pose_stamped_msg.pose = pose;
  return pose_stamped_msg;
}

moveit_msgs::RobotState CartesianConfiguration::toMoveitMsgsRobotState() const
{
  if (!robot_model_)
  {
    throw std::runtime_error("No robot model set");
  }

  robot_state::RobotState rstate(robot_model_);
  rstate.setToDefaultValues();
  if (hasSeed())
  {
    rstate.setJointGroupPositions(group_name_, getSeed().getJoints());
  }

  rstate.update();

  // set to Cartesian pose
  Eigen::Isometry3d start_pose;
  tf::poseMsgToEigen(pose_, start_pose);
  if (!rstate.setFromIK(rstate.getRobotModel()->getJointModelGroup(group_name_), start_pose, link_name_))
  {
    std::ostringstream os;
    os << "No solution for ik \n" << start_pose.translation() << "\n" << start_pose.linear();
    throw std::runtime_error(os.str());
  }

  // Conversion to RobotState msg type
  moveit_msgs::RobotState robot_state_msg;
  moveit::core::robotStateToRobotStateMsg(rstate, robot_state_msg, true);
  return robot_state_msg;
}

std::ostream& operator<<(std::ostream& os, const CartesianConfiguration& obj)
{
  os << "Group name: \"" << obj.getGroupName() << "\"";
  os << " | link name: \"" << obj.getLinkName() << "\"";
  os << "\n" << obj.getPose();
  return os;
}

}  // namespace pilz_industrial_motion_planner_testutils
