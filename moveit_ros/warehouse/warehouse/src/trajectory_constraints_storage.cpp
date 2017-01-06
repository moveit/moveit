/*********************************************************************
 * Software License Agreement (BSD License)
 *
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

/* Author: Mario Prats, Ioan Sucan */

#include <moveit/warehouse/trajectory_constraints_storage.h>

const std::string moveit_warehouse::TrajectoryConstraintsStorage::DATABASE_NAME = "moveit_trajectory_constraints";

const std::string moveit_warehouse::TrajectoryConstraintsStorage::CONSTRAINTS_ID_NAME = "constraints_id";
const std::string moveit_warehouse::TrajectoryConstraintsStorage::CONSTRAINTS_GROUP_NAME = "group_id";
const std::string moveit_warehouse::TrajectoryConstraintsStorage::ROBOT_NAME = "robot_id";

moveit_warehouse::TrajectoryConstraintsStorage::TrajectoryConstraintsStorage(const std::string& host,
                                                                             const unsigned int port,
                                                                             double wait_seconds)
  : MoveItMessageStorage(host, port, wait_seconds)
{
  createCollections();
  ROS_DEBUG("Connected to MongoDB '%s' on host '%s' port '%u'.", DATABASE_NAME.c_str(), db_host_.c_str(), db_port_);
}

void moveit_warehouse::TrajectoryConstraintsStorage::createCollections(void)
{
  constraints_collection_.reset(new TrajectoryConstraintsCollection::element_type(
      DATABASE_NAME, "trajectory_constraints", db_host_, db_port_, timeout_));
}

void moveit_warehouse::TrajectoryConstraintsStorage::reset(void)
{
  constraints_collection_.reset();
  MoveItMessageStorage::drop(DATABASE_NAME);
  createCollections();
}

void moveit_warehouse::TrajectoryConstraintsStorage::addTrajectoryConstraints(
    const moveit_msgs::TrajectoryConstraints& msg, const std::string& name, const std::string& robot,
    const std::string& group)
{
  bool replace = false;
  if (hasTrajectoryConstraints(name, robot, group))
  {
    removeTrajectoryConstraints(name, robot, group);
    replace = true;
  }
  mongo_ros::Metadata metadata(CONSTRAINTS_ID_NAME, name, ROBOT_NAME, robot, CONSTRAINTS_GROUP_NAME, group);
  constraints_collection_->insert(msg, metadata);
  ROS_DEBUG("%s constraints '%s'", replace ? "Replaced" : "Added", name.c_str());
}

bool moveit_warehouse::TrajectoryConstraintsStorage::hasTrajectoryConstraints(const std::string& name,
                                                                              const std::string& robot,
                                                                              const std::string& group) const
{
  mongo_ros::Query q(CONSTRAINTS_ID_NAME, name);
  if (!robot.empty())
    q.append(ROBOT_NAME, robot);
  if (!group.empty())
    q.append(CONSTRAINTS_GROUP_NAME, group);
  std::vector<TrajectoryConstraintsWithMetadata> constr = constraints_collection_->pullAllResults(q, true);
  return !constr.empty();
}

void moveit_warehouse::TrajectoryConstraintsStorage::getKnownTrajectoryConstraints(const std::string& regex,
                                                                                   std::vector<std::string>& names,
                                                                                   const std::string& robot,
                                                                                   const std::string& group) const
{
  getKnownTrajectoryConstraints(names, robot, group);
  filterNames(regex, names);
}

void moveit_warehouse::TrajectoryConstraintsStorage::getKnownTrajectoryConstraints(std::vector<std::string>& names,
                                                                                   const std::string& robot,
                                                                                   const std::string& group) const
{
  names.clear();
  mongo_ros::Query q;
  if (!robot.empty())
    q.append(ROBOT_NAME, robot);
  if (!group.empty())
    q.append(CONSTRAINTS_GROUP_NAME, group);
  std::vector<TrajectoryConstraintsWithMetadata> constr =
      constraints_collection_->pullAllResults(q, true, CONSTRAINTS_ID_NAME, true);
  for (std::size_t i = 0; i < constr.size(); ++i)
    if (constr[i]->metadata.hasField(CONSTRAINTS_ID_NAME.c_str()))
      names.push_back(constr[i]->lookupString(CONSTRAINTS_ID_NAME));
}

bool moveit_warehouse::TrajectoryConstraintsStorage::getTrajectoryConstraints(TrajectoryConstraintsWithMetadata& msg_m,
                                                                              const std::string& name,
                                                                              const std::string& robot,
                                                                              const std::string& group) const
{
  mongo_ros::Query q(CONSTRAINTS_ID_NAME, name);
  if (!robot.empty())
    q.append(ROBOT_NAME, robot);
  if (!group.empty())
    q.append(CONSTRAINTS_GROUP_NAME, group);
  std::vector<TrajectoryConstraintsWithMetadata> constr = constraints_collection_->pullAllResults(q, false);
  if (constr.empty())
    return false;
  else
  {
    msg_m = constr.back();
    return true;
  }
}

void moveit_warehouse::TrajectoryConstraintsStorage::renameTrajectoryConstraints(const std::string& old_name,
                                                                                 const std::string& new_name,
                                                                                 const std::string& robot,
                                                                                 const std::string& group)
{
  mongo_ros::Query q(CONSTRAINTS_ID_NAME, old_name);
  if (!robot.empty())
    q.append(ROBOT_NAME, robot);
  if (!group.empty())
    q.append(CONSTRAINTS_GROUP_NAME, group);
  mongo_ros::Metadata m(CONSTRAINTS_ID_NAME, new_name);
  constraints_collection_->modifyMetadata(q, m);
  ROS_DEBUG("Renamed constraints from '%s' to '%s'", old_name.c_str(), new_name.c_str());
}

void moveit_warehouse::TrajectoryConstraintsStorage::removeTrajectoryConstraints(const std::string& name,
                                                                                 const std::string& robot,
                                                                                 const std::string& group)
{
  mongo_ros::Query q(CONSTRAINTS_ID_NAME, name);
  if (!robot.empty())
    q.append(ROBOT_NAME, robot);
  if (!group.empty())
    q.append(CONSTRAINTS_GROUP_NAME, group);
  unsigned int rem = constraints_collection_->removeMessages(q);
  ROS_DEBUG("Removed %u TrajectoryConstraints messages (named '%s')", rem, name.c_str());
}
