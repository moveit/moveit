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

/* Author Ioan Sucan */

#include <moveit/collision_detection_fcl/collision_world.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>
#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/collision_node.h>

collision_detection::CollisionWorldFCL::CollisionWorldFCL(void) : CollisionWorld()
{
  fcl::DynamicAABBTreeCollisionManager* m = new fcl::DynamicAABBTreeCollisionManager();
  // m->tree_init_level = 2;
  manager_.reset(m);
}

collision_detection::CollisionWorldFCL::CollisionWorldFCL(const CollisionWorldFCL &other) : CollisionWorld(other)
{
  fcl::DynamicAABBTreeCollisionManager* m = new fcl::DynamicAABBTreeCollisionManager();
  // m->tree_init_level = 2;
  manager_.reset(m);

  fcl_objs_ = other.fcl_objs_;
  for (std::map<std::string, FCLObject>::iterator it = fcl_objs_.begin() ; it != fcl_objs_.end() ; ++it)
    it->second.registerTo(manager_.get());
  // manager_->update();
}

collision_detection::CollisionWorldFCL::~CollisionWorldFCL(void)
{
}

void collision_detection::CollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const kinematic_state::KinematicState &state) const
{
  checkRobotCollisionHelper(req, res, robot, state, NULL);
}

void collision_detection::CollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const kinematic_state::KinematicState &state, const AllowedCollisionMatrix &acm) const
{
  checkRobotCollisionHelper(req, res, robot, state, &acm);
}

void collision_detection::CollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const kinematic_state::KinematicState &state1, const kinematic_state::KinematicState &state2) const
{
  logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::CollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const kinematic_state::KinematicState &state1, const kinematic_state::KinematicState &state2, const AllowedCollisionMatrix &acm) const
{
  logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::CollisionWorldFCL::checkRobotCollisionHelper(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const kinematic_state::KinematicState &state, const AllowedCollisionMatrix *acm) const
{
  const CollisionRobotFCL &robot_fcl = dynamic_cast<const CollisionRobotFCL&>(robot);
  FCLObject fcl_obj;
  robot_fcl.constructFCLObject(state, fcl_obj);
  
  CollisionData cd(&req, &res, acm);
  cd.enableGroup(robot.getKinematicModel());
  for (std::size_t i = 0 ; !cd.done_ && i < fcl_obj.collision_objects_.size() ; ++i)
    manager_->collide(fcl_obj.collision_objects_[i].get(), &cd, &collisionCallback);
  
  if (req.distance)
    res.distance = distanceRobotHelper(robot, state, acm);
}

void collision_detection::CollisionWorldFCL::checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world) const
{
  checkWorldCollisionHelper(req, res, other_world, NULL);
}

void collision_detection::CollisionWorldFCL::checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world, const AllowedCollisionMatrix &acm) const
{
  checkWorldCollisionHelper(req, res, other_world, &acm);
}

void collision_detection::CollisionWorldFCL::checkWorldCollisionHelper(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world, const AllowedCollisionMatrix *acm) const
{
  const CollisionWorldFCL &other_fcl_world = dynamic_cast<const CollisionWorldFCL&>(other_world);
  CollisionData cd(&req, &res, acm);
  manager_->collide(other_fcl_world.manager_.get(), &cd, &collisionCallback);
  
  if (req.distance)
    res.distance = distanceWorldHelper(other_world, acm);
}

void collision_detection::CollisionWorldFCL::constructFCLObject(const Object *obj, FCLObject &fcl_obj) const
{
  for (std::size_t i = 0 ; i < obj->shapes_.size() ; ++i)
  {
    FCLGeometryConstPtr g = createCollisionGeometry(obj->shapes_[i], obj);
    if (g)
    {
      fcl::CollisionObject *co = new fcl::CollisionObject(g->collision_geometry_,  transform2fcl(obj->shape_poses_[i]));
      fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(co));
      fcl_obj.collision_geometry_.push_back(g);
    }
  }
}

void collision_detection::CollisionWorldFCL::updateFCLObject(const std::string &id)
{
  // remove FCL objects that correspond to this object
  std::map<std::string, FCLObject>::iterator jt = fcl_objs_.find(id);
  if (jt != fcl_objs_.end())
  {
    jt->second.unregisterFrom(manager_.get());
    jt->second.clear();
  }

  // check to see if we have this object
  std::map<std::string, ObjectPtr>::iterator it = objects_.find(id);
  if (it != objects_.end())
  {
    // construct FCL objects that correspond to this object
    if (jt != fcl_objs_.end())
    {
      constructFCLObject(it->second.get(), jt->second);
      jt->second.registerTo(manager_.get());
    }
    else
    {
      constructFCLObject(it->second.get(), fcl_objs_[id]);
      fcl_objs_[id].registerTo(manager_.get());
    }
  }
  else
  {
    if (jt != fcl_objs_.end())
      fcl_objs_.erase(jt);
  }
  
  // manager_->update();
}

void collision_detection::CollisionWorldFCL::addToObject(const std::string &id, const std::vector<shapes::ShapeConstPtr> &shapes, const EigenSTL::vector_Affine3d &poses)
{
  CollisionWorld::addToObject(id, shapes, poses);
  updateFCLObject(id);
}

void collision_detection::CollisionWorldFCL::addToObject(const std::string &id, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose)
{
  CollisionWorld::addToObject(id, shape, pose);
  updateFCLObject(id);
}

bool collision_detection::CollisionWorldFCL::moveShapeInObject(const std::string &id, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose)
{
  if (CollisionWorld::moveShapeInObject(id, shape, pose))
  {
    updateFCLObject(id);
    return true;
  }
  else
    return false;
}

bool collision_detection::CollisionWorldFCL::removeShapeFromObject(const std::string &id, const shapes::ShapeConstPtr &shape)
{    
  if (CollisionWorld::removeShapeFromObject(id, shape))
  {
    updateFCLObject(id);
    cleanCollisionGeometryCache();
    return true;
  }
  else
    return false;
}

void collision_detection::CollisionWorldFCL::removeObject(const std::string &id)
{  
  CollisionWorld::removeObject(id);
  std::map<std::string, FCLObject>::iterator it = fcl_objs_.find(id);
  if (it != fcl_objs_.end())
  {
    it->second.unregisterFrom(manager_.get());
    it->second.clear();
    fcl_objs_.erase(it);
    // manager_->update();
  }
  cleanCollisionGeometryCache();
}

void collision_detection::CollisionWorldFCL::clearObjects(void)
{
  CollisionWorld::clearObjects();
  manager_->clear();
  fcl_objs_.clear();  
  cleanCollisionGeometryCache();
}

double collision_detection::CollisionWorldFCL::distanceRobotHelper(const CollisionRobot &robot, const kinematic_state::KinematicState &state, const AllowedCollisionMatrix *acm) const
{       
  const CollisionRobotFCL& robot_fcl = dynamic_cast<const CollisionRobotFCL&>(robot);
  FCLObject fcl_obj;
  robot_fcl.constructFCLObject(state, fcl_obj);

  CollisionRequest req;
  CollisionResult res;
  CollisionData cd(&req, &res, acm);
  cd.enableGroup(robot.getKinematicModel());
  
  for(std::size_t i = 0; !cd.done_ && i < fcl_obj.collision_objects_.size(); ++i)
    manager_->distance(fcl_obj.collision_objects_[i].get(), &cd, &distanceCallback);


  return res.distance;
}

double collision_detection::CollisionWorldFCL::distanceRobot(const CollisionRobot &robot, const kinematic_state::KinematicState &state) const
{
  return distanceRobotHelper(robot, state, NULL);
}

double collision_detection::CollisionWorldFCL::distanceRobot(const CollisionRobot &robot, const kinematic_state::KinematicState &state, const AllowedCollisionMatrix &acm) const
{
  return distanceRobotHelper(robot, state, &acm);
}

double collision_detection::CollisionWorldFCL::distanceWorld(const CollisionWorld &world) const
{
  return distanceWorldHelper(world, NULL);
}

double collision_detection::CollisionWorldFCL::distanceWorld(const CollisionWorld &world, const AllowedCollisionMatrix &acm) const
{
  return distanceWorldHelper(world, &acm);
}

double collision_detection::CollisionWorldFCL::distanceWorldHelper(const CollisionWorld &other_world, const AllowedCollisionMatrix *acm) const
{
  const CollisionWorldFCL& other_fcl_world = dynamic_cast<const CollisionWorldFCL&>(other_world);
  CollisionRequest req;
  CollisionResult res;
  CollisionData cd(&req, &res, acm);
  manager_->distance(other_fcl_world.manager_.get(), &cd, &distanceCallback);

  return res.distance;
}
