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

#include <moveit/collision_detection_fcl/collision_robot_fcl.h>

namespace collision_detection
{
CollisionRobotFCL::CollisionRobotFCL(const robot_model::RobotModelConstPtr& model, double padding, double scale)
  : CollisionRobot(model, padding, scale)
{
  const std::vector<const robot_model::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  std::size_t index;
  geoms_.resize(robot_model_->getLinkGeometryCount());
  fcl_objs_.resize(robot_model_->getLinkGeometryCount());
  // we keep the same order of objects as what RobotState *::getLinkState() returns
  for (auto link : links)
    for (std::size_t j = 0; j < link->getShapes().size(); ++j)
    {
      FCLGeometryConstPtr g = createCollisionGeometry(link->getShapes()[j], getLinkScale(link->getName()),
                                                      getLinkPadding(link->getName()), link, j);
      if (g)
      {
        index = link->getFirstCollisionBodyTransformIndex() + j;
        geoms_[index] = g;

        // Need to store the FCL object so the AABB does not get recreated every time.
        // Every time this object is created, g->computeLocalAABB() is called  which is
        // very expensive and should only be calculated once. To update the AABB, use the
        // collObj->setTransform and then call collObj->computeAABB() to transform the AABB.
        fcl_objs_[index] = FCLCollisionObjectConstPtr(new fcl::CollisionObject(g->collision_geometry_));
      }
      else
        ROS_ERROR_NAMED("collision_detection.fcl", "Unable to construct collision geometry for link '%s'",
                        link->getName().c_str());
    }
}

CollisionRobotFCL::CollisionRobotFCL(const CollisionRobotFCL& other) : CollisionRobot(other)
{
  geoms_ = other.geoms_;
  fcl_objs_ = other.fcl_objs_;
}

void CollisionRobotFCL::getAttachedBodyObjects(const robot_state::AttachedBody* ab,
                                               std::vector<FCLGeometryConstPtr>& geoms) const
{
  const std::vector<shapes::ShapeConstPtr>& shapes = ab->getShapes();
  for (std::size_t i = 0; i < shapes.size(); ++i)
  {
    FCLGeometryConstPtr co = createCollisionGeometry(shapes[i], ab, i);
    if (co)
      geoms.push_back(co);
  }
}

void CollisionRobotFCL::constructFCLObject(const robot_state::RobotState& state, FCLObject& fcl_obj) const
{
  fcl_obj.collision_objects_.reserve(geoms_.size());
  fcl::Transform3f fcl_tf;

  for (std::size_t i = 0; i < geoms_.size(); ++i)
    if (geoms_[i] && geoms_[i]->collision_geometry_)
    {
      transform2fcl(state.getCollisionBodyTransform(geoms_[i]->collision_geometry_data_->ptr.link,
                                                    geoms_[i]->collision_geometry_data_->shape_index),
                    fcl_tf);
      auto collObj = new fcl::CollisionObject(*fcl_objs_[i]);
      collObj->setTransform(fcl_tf);
      collObj->computeAABB();
      fcl_obj.collision_objects_.push_back(FCLCollisionObjectPtr(collObj));
    }

  // TODO: Implement a method for caching fcl::CollisionObject's for robot_state::AttachedBody's
  std::vector<const robot_state::AttachedBody*> ab;
  state.getAttachedBodies(ab);
  for (auto& body : ab)
  {
    std::vector<FCLGeometryConstPtr> objs;
    getAttachedBodyObjects(body, objs);
    const EigenSTL::vector_Affine3d& ab_t = body->getGlobalCollisionBodyTransforms();
    for (std::size_t k = 0; k < objs.size(); ++k)
      if (objs[k]->collision_geometry_)
      {
        transform2fcl(ab_t[k], fcl_tf);
        fcl_obj.collision_objects_.push_back(
            FCLCollisionObjectPtr(new fcl::CollisionObject(objs[k]->collision_geometry_, fcl_tf)));
        // we copy the shared ptr to the CollisionGeometryData, as this is not stored by the class itself,
        // and would be destroyed when objs goes out of scope.
        fcl_obj.collision_geometry_.push_back(objs[k]);
      }
  }
}

void CollisionRobotFCL::allocSelfCollisionBroadPhase(const robot_state::RobotState& state, FCLManager& manager) const
{
  auto m = new fcl::DynamicAABBTreeCollisionManager();
  // m->tree_init_level = 2;
  manager.manager_.reset(m);
  constructFCLObject(state, manager.object_);
  manager.object_.registerTo(manager.manager_.get());
  // manager.manager_->update();
}

void CollisionRobotFCL::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state) const
{
  checkSelfCollisionHelper(req, res, state, nullptr);
}

void CollisionRobotFCL::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state,
                                           const AllowedCollisionMatrix& acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void CollisionRobotFCL::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state1,
                                           const robot_state::RobotState& state2) const
{
  ROS_ERROR_NAMED("collision_detection.fcl", "FCL continuous collision checking not yet implemented");
}

void CollisionRobotFCL::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                           const AllowedCollisionMatrix& acm) const
{
  ROS_ERROR_NAMED("collision_detection.fcl", "FCL continuous collision checking not yet implemented");
}

void CollisionRobotFCL::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                 const robot_state::RobotState& state,
                                                 const AllowedCollisionMatrix* acm) const
{
  FCLManager manager;
  allocSelfCollisionBroadPhase(state, manager);
  CollisionData cd(&req, &res, acm);
  cd.enableGroup(getRobotModel());
  manager.manager_->collide(&cd, &collisionCallback);
  if (req.distance)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    dreq.enableGroup(getRobotModel());
    distanceSelf(dreq, dres, state);
    res.distance = dres.minimum_distance.distance;
  }
}

void CollisionRobotFCL::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                            const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                            const robot_state::RobotState& other_state) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, nullptr);
}

void CollisionRobotFCL::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                            const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                            const robot_state::RobotState& other_state,
                                            const AllowedCollisionMatrix& acm) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, &acm);
}

void CollisionRobotFCL::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                            const robot_state::RobotState& state1,
                                            const robot_state::RobotState& state2, const CollisionRobot& other_robot,
                                            const robot_state::RobotState& other_state1,
                                            const robot_state::RobotState& other_state2) const
{
  ROS_ERROR_NAMED("collision_detection.fcl", "FCL continuous collision checking not yet implemented");
}

void CollisionRobotFCL::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                            const robot_state::RobotState& state1,
                                            const robot_state::RobotState& state2, const CollisionRobot& other_robot,
                                            const robot_state::RobotState& other_state1,
                                            const robot_state::RobotState& other_state2,
                                            const AllowedCollisionMatrix& acm) const
{
  ROS_ERROR_NAMED("collision_detection.fcl", "FCL continuous collision checking not yet implemented");
}

void CollisionRobotFCL::checkOtherCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                  const robot_state::RobotState& state,
                                                  const CollisionRobot& other_robot,
                                                  const robot_state::RobotState& other_state,
                                                  const AllowedCollisionMatrix* acm) const
{
  FCLManager manager;
  allocSelfCollisionBroadPhase(state, manager);

  const CollisionRobotFCL& fcl_rob = dynamic_cast<const CollisionRobotFCL&>(other_robot);
  FCLObject other_fcl_obj;
  fcl_rob.constructFCLObject(other_state, other_fcl_obj);

  CollisionData cd(&req, &res, acm);
  cd.enableGroup(getRobotModel());
  for (std::size_t i = 0; !cd.done_ && i < other_fcl_obj.collision_objects_.size(); ++i)
    manager.manager_->collide(other_fcl_obj.collision_objects_[i].get(), &cd, &collisionCallback);

  if (req.distance)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    dreq.enableGroup(getRobotModel());
    distanceOther(dreq, dres, state, other_robot, other_state);
    res.distance = dres.minimum_distance.distance;
  }
}

void CollisionRobotFCL::updatedPaddingOrScaling(const std::vector<std::string>& links)
{
  std::size_t index;
  for (const auto& link : links)
  {
    const robot_model::LinkModel* lmodel = robot_model_->getLinkModel(link);
    if (lmodel)
    {
      for (std::size_t j = 0; j < lmodel->getShapes().size(); ++j)
      {
        FCLGeometryConstPtr g = createCollisionGeometry(lmodel->getShapes()[j], getLinkScale(lmodel->getName()),
                                                        getLinkPadding(lmodel->getName()), lmodel, j);
        if (g)
        {
          index = lmodel->getFirstCollisionBodyTransformIndex() + j;
          geoms_[index] = g;
          fcl_objs_[index] = FCLCollisionObjectConstPtr(new fcl::CollisionObject(g->collision_geometry_));
        }
      }
    }
    else
      ROS_ERROR_NAMED("collision_detection.fcl", "Updating padding or scaling for unknown link: '%s'", link.c_str());
  }
}

void CollisionRobotFCL::distanceSelf(const DistanceRequest& req, DistanceResult& res,
                                     const robot_state::RobotState& state) const
{
  FCLManager manager;
  allocSelfCollisionBroadPhase(state, manager);
  DistanceData drd(&req, &res);

  manager.manager_->distance(&drd, &distanceCallback);
}

void CollisionRobotFCL::distanceOther(const DistanceRequest& req, DistanceResult& res,
                                      const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                      const robot_state::RobotState& other_state) const
{
  FCLManager manager;
  allocSelfCollisionBroadPhase(state, manager);

  const CollisionRobotFCL& fcl_rob = dynamic_cast<const CollisionRobotFCL&>(other_robot);
  FCLObject other_fcl_obj;
  fcl_rob.constructFCLObject(other_state, other_fcl_obj);

  DistanceData drd(&req, &res);
  for (std::size_t i = 0; !drd.done && i < other_fcl_obj.collision_objects_.size(); ++i)
    manager.manager_->distance(other_fcl_obj.collision_objects_[i].get(), &drd, &distanceCallback);
}

}  // end of namespace collision_detection