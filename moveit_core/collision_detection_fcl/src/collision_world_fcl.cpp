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

#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>
#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/collision_node.h>
#include <boost/bind.hpp>

namespace collision_detection
{
const std::string CollisionDetectorAllocatorFCL::NAME_("FCL");

CollisionWorldFCL::CollisionWorldFCL() : CollisionWorld()
{
  auto m = new fcl::DynamicAABBTreeCollisionManager();
  // m->tree_init_level = 2;
  manager_.reset(m);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldFCL::notifyObjectChange, this, _1, _2));
}

CollisionWorldFCL::CollisionWorldFCL(const WorldPtr& world) : CollisionWorld(world)
{
  auto m = new fcl::DynamicAABBTreeCollisionManager();
  // m->tree_init_level = 2;
  manager_.reset(m);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldFCL::notifyObjectChange, this, _1, _2));
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionWorldFCL::CollisionWorldFCL(const CollisionWorldFCL& other, const WorldPtr& world)
  : CollisionWorld(other, world)
{
  auto m = new fcl::DynamicAABBTreeCollisionManager();
  // m->tree_init_level = 2;
  manager_.reset(m);

  fcl_objs_ = other.fcl_objs_;
  for (auto& fcl_obj : fcl_objs_)
    fcl_obj.second.registerTo(manager_.get());
  // manager_->update();

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldFCL::notifyObjectChange, this, _1, _2));
}

CollisionWorldFCL::~CollisionWorldFCL()
{
  getWorld()->removeObserver(observer_handle_);
}

void CollisionWorldFCL::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionRobot& robot, const robot_state::RobotState& state) const
{
  checkRobotCollisionHelper(req, res, robot, state, nullptr);
}

void CollisionWorldFCL::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionRobot& robot, const robot_state::RobotState& state,
                                            const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelper(req, res, robot, state, &acm);
}

void CollisionWorldFCL::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionRobot& robot, const robot_state::RobotState& state1,
                                            const robot_state::RobotState& state2) const
{
  ROS_ERROR_NAMED("collision_detection.fcl", "FCL continuous collision checking not yet implemented");
}

void CollisionWorldFCL::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionRobot& robot, const robot_state::RobotState& state1,
                                            const robot_state::RobotState& state2,
                                            const AllowedCollisionMatrix& acm) const
{
  ROS_ERROR_NAMED("collision_detection.fcl", "FCL continuous collision checking not yet implemented");
}

void CollisionWorldFCL::checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                  const CollisionRobot& robot, const robot_state::RobotState& state,
                                                  const AllowedCollisionMatrix* acm) const
{
  const CollisionRobotFCL& robot_fcl = dynamic_cast<const CollisionRobotFCL&>(robot);
  FCLObject fcl_obj;
  robot_fcl.constructFCLObject(state, fcl_obj);

  CollisionData cd(&req, &res, acm);
  cd.enableGroup(robot.getRobotModel());
  for (std::size_t i = 0; !cd.done_ && i < fcl_obj.collision_objects_.size(); ++i)
    manager_->collide(fcl_obj.collision_objects_[i].get(), &cd, &collisionCallback);

  if (req.distance)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    dreq.enableGroup(robot.getRobotModel());
    distanceRobot(dreq, dres, robot, state);
    res.distance = dres.minimum_distance.distance;
  }
}

void CollisionWorldFCL::checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionWorld& other_world) const
{
  checkWorldCollisionHelper(req, res, other_world, nullptr);
}

void CollisionWorldFCL::checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionWorld& other_world, const AllowedCollisionMatrix& acm) const
{
  checkWorldCollisionHelper(req, res, other_world, &acm);
}

void CollisionWorldFCL::checkWorldCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                  const CollisionWorld& other_world,
                                                  const AllowedCollisionMatrix* acm) const
{
  const CollisionWorldFCL& other_fcl_world = dynamic_cast<const CollisionWorldFCL&>(other_world);
  CollisionData cd(&req, &res, acm);
  manager_->collide(other_fcl_world.manager_.get(), &cd, &collisionCallback);

  if (req.distance)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    distanceWorld(dreq, dres, other_world);
    res.distance = dres.minimum_distance.distance;
  }
}

void CollisionWorldFCL::constructFCLObject(const World::Object* obj, FCLObject& fcl_obj) const
{
  for (std::size_t i = 0; i < obj->shapes_.size(); ++i)
  {
    FCLGeometryConstPtr g = createCollisionGeometry(obj->shapes_[i], obj);
    if (g)
    {
      auto co = new fcl::CollisionObject(g->collision_geometry_, transform2fcl(obj->shape_poses_[i]));
      fcl_obj.collision_objects_.push_back(FCLCollisionObjectPtr(co));
      fcl_obj.collision_geometry_.push_back(g);
    }
  }
}

void CollisionWorldFCL::updateFCLObject(const std::string& id)
{
  // remove FCL objects that correspond to this object
  auto jt = fcl_objs_.find(id);
  if (jt != fcl_objs_.end())
  {
    jt->second.unregisterFrom(manager_.get());
    jt->second.clear();
  }

  // check to see if we have this object
  auto it = getWorld()->find(id);
  if (it != getWorld()->end())
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

void CollisionWorldFCL::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);

  // clear out objects from old world
  manager_->clear();
  fcl_objs_.clear();
  cleanCollisionGeometryCache();

  CollisionWorld::setWorld(world);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldFCL::notifyObjectChange, this, _1, _2));

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionWorldFCL::notifyObjectChange(const ObjectConstPtr& obj, World::Action action)
{
  if (action == World::DESTROY)
  {
    auto it = fcl_objs_.find(obj->id_);
    if (it != fcl_objs_.end())
    {
      it->second.unregisterFrom(manager_.get());
      it->second.clear();
      fcl_objs_.erase(it);
    }
    cleanCollisionGeometryCache();
  }
  else
  {
    updateFCLObject(obj->id_);
    if (action & (World::DESTROY | World::REMOVE_SHAPE))
      cleanCollisionGeometryCache();
  }
}

void CollisionWorldFCL::distanceRobot(const DistanceRequest& req, DistanceResult& res, const CollisionRobot& robot,
                                      const robot_state::RobotState& state) const
{
  const CollisionRobotFCL& robot_fcl = dynamic_cast<const CollisionRobotFCL&>(robot);
  FCLObject fcl_obj;
  robot_fcl.constructFCLObject(state, fcl_obj);

  DistanceData drd(&req, &res);
  for (std::size_t i = 0; !drd.done && i < fcl_obj.collision_objects_.size(); ++i)
    manager_->distance(fcl_obj.collision_objects_[i].get(), &drd, &distanceCallback);
}

void CollisionWorldFCL::distanceWorld(const DistanceRequest& req, DistanceResult& res,
                                      const CollisionWorld& world) const
{
  const CollisionWorldFCL& other_fcl_world = dynamic_cast<const CollisionWorldFCL&>(world);
  DistanceData drd(&req, &res);
  manager_->distance(other_fcl_world.manager_.get(), &drd, &distanceCallback);
}

}  // end of namespace collision_detection