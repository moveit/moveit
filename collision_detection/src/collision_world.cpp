/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <moveit/collision_detection/collision_world.h>
#include <geometric_shapes/shape_operations.h>

collision_detection::CollisionWorld::CollisionWorld() : record_changes_(false)
{
}

collision_detection::CollisionWorld::CollisionWorld(const CollisionWorld &other) : record_changes_(false)
{
  objects_ = other.objects_;
}

void collision_detection::CollisionWorld::checkCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state) const
{
  robot.checkSelfCollision(req, res, state);
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    checkRobotCollision(req, res, robot, state);
}

void collision_detection::CollisionWorld::checkCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const
{
  robot.checkSelfCollision(req, res, state, acm);
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    checkRobotCollision(req, res, robot, state, acm);
}

void collision_detection::CollisionWorld::checkCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot,
                                                         const robot_state::RobotState &state1, const robot_state::RobotState &state2) const
{
  robot.checkSelfCollision(req, res, state1, state2);
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    checkRobotCollision(req, res, robot, state1, state2);
}

void collision_detection::CollisionWorld::checkCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot,
                                                         const robot_state::RobotState &state1, const robot_state::RobotState &state2, const AllowedCollisionMatrix &acm) const
{
  robot.checkSelfCollision(req, res, state1, state2, acm);
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    checkRobotCollision(req, res, robot, state1, state2, acm);
}

void collision_detection::CollisionWorld::addToObject(const std::string &id, const std::vector<shapes::ShapeConstPtr> &shapes, const EigenSTL::vector_Affine3d &poses)
{
  if (shapes.size() != poses.size())
    logError("Number of shapes and number of poses do not match. Not adding this object to collision world.");
  else
  {
    // make sure that if a new object is created, it knows its name
    std::map<std::string, ObjectPtr>::iterator it = objects_.find(id);
    if (it == objects_.end())
    {  
      objects_[id].reset(new Object(id));
      it = objects_.find(id);
    }
    else
      if (record_changes_)
        changeRemoveObject(id);
    
    ensureUnique(it->second);
    for (std::size_t i = 0 ; i < shapes.size() ; ++i)
      addToObjectInternal(it->second, shapes[i], poses[i]);
    
    if (record_changes_)
      changeAddObject(it->second->id_);
  }
}

std::vector<std::string> collision_detection::CollisionWorld::getObjectIds() const
{
  std::vector<std::string> id;
  for (std::map<std::string, ObjectPtr>::const_iterator it = objects_.begin() ; it != objects_.end() ; ++it)
    id.push_back(it->first);
  return id;
}

collision_detection::CollisionWorld::ObjectConstPtr collision_detection::CollisionWorld::getObject(const std::string &id) const
{
  static ObjectConstPtr empty;
  std::map<std::string, ObjectPtr>::const_iterator it = objects_.find(id);
  if (it == objects_.end())
    return empty;
  else
    return it->second;
}

bool collision_detection::CollisionWorld::hasObject(const std::string &id) const
{
  return objects_.find(id) != objects_.end();
}

void collision_detection::CollisionWorld::ensureUnique(ObjectPtr &id)
{
  if (id && !id.unique()) 
    id.reset(new Object(*id));
}

void collision_detection::CollisionWorld::addToObject(const std::string &id, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose)
{
  // make sure that if a new object is created, it knows its name
  std::map<std::string, ObjectPtr>::iterator it = objects_.find(id);
  if (it == objects_.end())
  {  
    objects_[id].reset(new Object(id));
    it = objects_.find(id);
  }
  else
    if (record_changes_)
      changeRemoveObject(id);

  ensureUnique(it->second);
  addToObjectInternal(it->second, shape, pose);
  
  if (record_changes_)
    changeAddObject(it->second->id_);
}

void collision_detection::CollisionWorld::addToObjectInternal(const ObjectPtr &obj, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose)
{
  obj->shapes_.push_back(shape);
  obj->shape_poses_.push_back(pose);
}

bool collision_detection::CollisionWorld::moveShapeInObject(const std::string &id, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose)
{
  std::map<std::string, ObjectPtr>::iterator it = objects_.find(id);
  if (it != objects_.end())
  {
    unsigned int n = it->second->shapes_.size();
    for (unsigned int i = 0 ; i < n ; ++i)
      if (it->second->shapes_[i] == shape)
      {
        ensureUnique(it->second);
        it->second->shape_poses_[i] = pose;
        
        if (record_changes_)
        {
          changeRemoveObject(id);
          changeAddObject(it->second->id_);
        }
        return true;
      }
  }
  return false;
}

bool collision_detection::CollisionWorld::removeShapeFromObject(const std::string &id, const shapes::ShapeConstPtr &shape)
{
  std::map<std::string, ObjectPtr>::iterator it = objects_.find(id);
  if (it != objects_.end())
  {
    unsigned int n = it->second->shapes_.size();
    for (unsigned int i = 0 ; i < n ; ++i)
      if (it->second->shapes_[i] == shape)
      {
        ensureUnique(it->second);
        it->second->shapes_.erase(it->second->shapes_.begin() + i);
        it->second->shape_poses_.erase(it->second->shape_poses_.begin() + i);
        if (it->second->shapes_.empty())
        {
          objects_.erase(it);
          if (record_changes_)
            changeRemoveObject(id);
        }
        else
          if (record_changes_)
          {
            changeRemoveObject(id);
            changeAddObject(it->second->id_);
          }
        return true;
      }
  }
  return false;
}

void collision_detection::CollisionWorld::removeObject(const std::string &id)
{
  if (objects_.erase(id) == 1)
    if (record_changes_)
      changeRemoveObject(id);
}

void collision_detection::CollisionWorld::clearObjects()
{
  if (record_changes_)
    for (std::map<std::string, ObjectPtr>::const_iterator it = objects_.begin() ; it != objects_.end() ; ++it)
      changeRemoveObject(it->first);
  objects_.clear();
}

collision_detection::CollisionWorld::Object::Object(const std::string &id) : id_(id)
{
}

void collision_detection::CollisionWorld::changeRemoveObject(const std::string &id)
{
  for (std::vector<Change>::iterator it = changes_.begin() ; it != changes_.end() ; )
    if (it->id_ == id)
      it = changes_.erase(it);
    else
      ++it;
  Change c;
  c.type_ = Change::REMOVE;
  c.id_ = id;
  changes_.push_back(c);
}

void collision_detection::CollisionWorld::changeAddObject(const std::string &id)
{
  Change c;
  c.type_ = Change::ADD;
  c.id_ = id;
  changes_.push_back(c);
}

void collision_detection::CollisionWorld::recordChanges(bool flag)
{
  record_changes_ = flag;
}

bool collision_detection::CollisionWorld::isRecordingChanges() const
{
  return record_changes_;
}

const std::vector<collision_detection::CollisionWorld::Change>& collision_detection::CollisionWorld::getChanges() const
{
  return changes_;
}

void collision_detection::CollisionWorld::clearChanges()
{
  changes_.clear();
}

collision_detection::CollisionWorld::Object::~Object()
{
}
