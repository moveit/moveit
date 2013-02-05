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
*********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/warehouse/planning_scene_world_storage.h>

const std::string moveit_warehouse::PlanningSceneWorldStorage::DATABASE_NAME = "moveit_planning_scene_worlds";
const std::string moveit_warehouse::PlanningSceneWorldStorage::PLANNING_SCENE_WORLD_ID_NAME = "world_id";

moveit_warehouse::PlanningSceneWorldStorage::PlanningSceneWorldStorage(const std::string &host, const unsigned int port, double wait_seconds) :
  MoveItMessageStorage(host, port, wait_seconds)
{
  createCollections();
  ROS_DEBUG("Connected to MongoDB '%s' on host '%s' port '%u'.", DATABASE_NAME.c_str(), db_host_.c_str(), db_port_);
}

void moveit_warehouse::PlanningSceneWorldStorage::createCollections()
{
  planning_scene_world_collection_.reset(new PlanningSceneWorldCollection::element_type(DATABASE_NAME, "planning_scene_worlds", db_host_, db_port_, timeout_));
}

void moveit_warehouse::PlanningSceneWorldStorage::reset()
{
  planning_scene_world_collection_.reset();
  MoveItMessageStorage::drop(DATABASE_NAME);
  createCollections();  
}

void moveit_warehouse::PlanningSceneWorldStorage::addPlanningSceneWorld(const moveit_msgs::PlanningSceneWorld &msg, const std::string &name)
{
  bool replace = false;
  if (hasPlanningSceneWorld(name))
  {
    removePlanningSceneWorld(name);
    replace = true;
  }  
  mongo_ros::Metadata metadata(PLANNING_SCENE_WORLD_ID_NAME, name);
  planning_scene_world_collection_->insert(msg, metadata);
  ROS_DEBUG("%s planning scene world '%s'", replace ? "Replaced" : "Added", name.c_str());
}

bool moveit_warehouse::PlanningSceneWorldStorage::hasPlanningSceneWorld(const std::string &name) const
{
  mongo_ros::Query q(PLANNING_SCENE_WORLD_ID_NAME, name);
  std::vector<PlanningSceneWorldWithMetadata> psw = planning_scene_world_collection_->pullAllResults(q, true);
  return !psw.empty();
}

void moveit_warehouse::PlanningSceneWorldStorage::getKnownPlanningSceneWorlds(const std::string &regex, std::vector<std::string> &names) const
{
  getKnownPlanningSceneWorlds(names);
  filterNames(regex, names);
}

void moveit_warehouse::PlanningSceneWorldStorage::getKnownPlanningSceneWorlds(std::vector<std::string> &names) const
{
  names.clear();
  mongo_ros::Query q;
  std::vector<PlanningSceneWorldWithMetadata> constr = planning_scene_world_collection_->pullAllResults(q, true, PLANNING_SCENE_WORLD_ID_NAME, true);
  for (std::size_t i = 0; i < constr.size() ; ++i)
    if (constr[i]->metadata.hasField(PLANNING_SCENE_WORLD_ID_NAME.c_str()))
      names.push_back(constr[i]->lookupString(PLANNING_SCENE_WORLD_ID_NAME));
}

bool moveit_warehouse::PlanningSceneWorldStorage::getPlanningSceneWorld(PlanningSceneWorldWithMetadata &msg_m, const std::string &name) const
{
  mongo_ros::Query q(PLANNING_SCENE_WORLD_ID_NAME, name);
  std::vector<PlanningSceneWorldWithMetadata> psw = planning_scene_world_collection_->pullAllResults(q, false);
  if (psw.empty())
    return false;
  else
  {
    msg_m = psw.front();
    return true;
  }
}

void moveit_warehouse::PlanningSceneWorldStorage::renamePlanningSceneWorld(const std::string &old_name, const std::string &new_name)
{
  mongo_ros::Query q(PLANNING_SCENE_WORLD_ID_NAME, old_name);
  mongo_ros::Metadata m(PLANNING_SCENE_WORLD_ID_NAME, new_name);
  planning_scene_world_collection_->modifyMetadata(q, m);  
  ROS_DEBUG("Renamed planning scene world from '%s' to '%s'", old_name.c_str(), new_name.c_str());
}

void moveit_warehouse::PlanningSceneWorldStorage::removePlanningSceneWorld(const std::string &name)
{
  mongo_ros::Query q(PLANNING_SCENE_WORLD_ID_NAME, name);
  unsigned int rem = planning_scene_world_collection_->removeMessages(q);
  ROS_DEBUG("Removed %u PlanningSceneWorld messages (named '%s')", rem, name.c_str());
}
