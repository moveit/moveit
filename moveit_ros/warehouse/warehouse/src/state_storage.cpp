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
 *   * Neither the name of Willow Garage nor the names of its
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

#include <moveit/warehouse/state_storage.h>

const std::string moveit_warehouse::RobotStateStorage::DATABASE_NAME = "moveit_robot_states";

const std::string moveit_warehouse::RobotStateStorage::STATE_NAME = "state_id";
const std::string moveit_warehouse::RobotStateStorage::ROBOT_NAME = "robot_id";

moveit_warehouse::RobotStateStorage::RobotStateStorage(const std::string& host, const unsigned int port,
                                                       double wait_seconds)
  : MoveItMessageStorage(host, port, wait_seconds)
{
  createCollections();
  ROS_DEBUG("Connected to MongoDB '%s' on host '%s' port '%u'.", DATABASE_NAME.c_str(), db_host_.c_str(), db_port_);
}

void moveit_warehouse::RobotStateStorage::createCollections()
{
  state_collection_.reset(
      new RobotStateCollection::element_type(DATABASE_NAME, "robot_states", db_host_, db_port_, timeout_));
}

void moveit_warehouse::RobotStateStorage::reset()
{
  state_collection_.reset();
  MoveItMessageStorage::drop(DATABASE_NAME);
  createCollections();
}

void moveit_warehouse::RobotStateStorage::addRobotState(const moveit_msgs::RobotState& msg, const std::string& name,
                                                        const std::string& robot)
{
  bool replace = false;
  if (hasRobotState(name, robot))
  {
    removeRobotState(name, robot);
    replace = true;
  }
  mongo_ros::Metadata metadata(STATE_NAME, name, ROBOT_NAME, robot);
  state_collection_->insert(msg, metadata);
  ROS_DEBUG("%s robot state '%s'", replace ? "Replaced" : "Added", name.c_str());
}

bool moveit_warehouse::RobotStateStorage::hasRobotState(const std::string& name, const std::string& robot) const
{
  mongo_ros::Query q(STATE_NAME, name);
  if (!robot.empty())
    q.append(ROBOT_NAME, robot);
  std::vector<RobotStateWithMetadata> constr = state_collection_->pullAllResults(q, true);
  return !constr.empty();
}

void moveit_warehouse::RobotStateStorage::getKnownRobotStates(const std::string& regex, std::vector<std::string>& names,
                                                              const std::string& robot) const
{
  getKnownRobotStates(names, robot);
  filterNames(regex, names);
}

void moveit_warehouse::RobotStateStorage::getKnownRobotStates(std::vector<std::string>& names,
                                                              const std::string& robot) const
{
  names.clear();
  mongo_ros::Query q;
  if (!robot.empty())
    q.append(ROBOT_NAME, robot);
  std::vector<RobotStateWithMetadata> constr = state_collection_->pullAllResults(q, true, STATE_NAME, true);
  for (std::size_t i = 0; i < constr.size(); ++i)
    if (constr[i]->metadata.hasField(STATE_NAME.c_str()))
      names.push_back(constr[i]->lookupString(STATE_NAME));
}

bool moveit_warehouse::RobotStateStorage::getRobotState(RobotStateWithMetadata& msg_m, const std::string& name,
                                                        const std::string& robot) const
{
  mongo_ros::Query q(STATE_NAME, name);
  if (!robot.empty())
    q.append(ROBOT_NAME, robot);
  std::vector<RobotStateWithMetadata> constr = state_collection_->pullAllResults(q, false);
  if (constr.empty())
    return false;
  else
  {
    msg_m = constr.front();
    return true;
  }
}

void moveit_warehouse::RobotStateStorage::renameRobotState(const std::string& old_name, const std::string& new_name,
                                                           const std::string& robot)
{
  mongo_ros::Query q(STATE_NAME, old_name);
  if (!robot.empty())
    q.append(ROBOT_NAME, robot);
  mongo_ros::Metadata m(STATE_NAME, new_name);
  state_collection_->modifyMetadata(q, m);
  ROS_DEBUG("Renamed robot state from '%s' to '%s'", old_name.c_str(), new_name.c_str());
}

void moveit_warehouse::RobotStateStorage::removeRobotState(const std::string& name, const std::string& robot)
{
  mongo_ros::Query q(STATE_NAME, name);
  if (!robot.empty())
    q.append(ROBOT_NAME, robot);
  unsigned int rem = state_collection_->removeMessages(q);
  ROS_DEBUG("Removed %u RobotState messages (named '%s')", rem, name.c_str());
}
