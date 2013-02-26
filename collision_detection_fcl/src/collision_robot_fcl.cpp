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

#include <moveit/collision_detection_fcl/collision_robot_fcl.h>

collision_detection::CollisionRobotFCL::CollisionRobotFCL(const robot_model::RobotModelConstPtr &kmodel, double padding, double scale) : CollisionRobot(kmodel, padding, scale)
{
  links_ = kmodel_->getLinkModels();

  // we keep the same order of objects as what RobotState *::getLinkState() returns
  for (std::size_t i = 0 ; i < links_.size() ; ++i)
    if (links_[i] && links_[i]->getShape())
    {
      FCLGeometryConstPtr g = createCollisionGeometry(links_[i]->getShape(), getLinkScale(links_[i]->getName()), getLinkPadding(links_[i]->getName()), links_[i]);
      if (g)
        index_map_[links_[i]->getName()] = geoms_.size();
      else
        links_[i] = NULL;
      geoms_.push_back(g);
    }
    else
    {
      links_[i] = NULL;
      geoms_.push_back(FCLGeometryConstPtr());
    }
}

collision_detection::CollisionRobotFCL::CollisionRobotFCL(const CollisionRobotFCL &other) : CollisionRobot(other)
{
  links_ = other.links_;
  geoms_ = other.geoms_;
  index_map_ = other.index_map_;
}

void collision_detection::CollisionRobotFCL::getAttachedBodyObjects(const robot_state::AttachedBody *ab,
                                                                    std::vector<FCLGeometryConstPtr> &geoms) const
{
  const std::vector<shapes::ShapeConstPtr> &shapes = ab->getShapes();
  for (std::size_t i = 0 ; i < shapes.size() ; ++i)
  {
    FCLGeometryConstPtr co = createCollisionGeometry(shapes[i], ab);
    if (co)
      geoms.push_back(co);
  }
}

void collision_detection::CollisionRobotFCL::constructFCLObject(const robot_state::RobotState &state, FCLObject &fcl_obj) const
{
  const std::vector<robot_state::LinkState*> &link_states = state.getLinkStateVector();
  fcl_obj.collision_objects_.reserve(geoms_.size());

  for (std::size_t i = 0 ; i < geoms_.size() ; ++i)
  {
    if (geoms_[i] && geoms_[i]->collision_geometry_)
    {
      fcl::CollisionObject *collObj = new fcl::CollisionObject(geoms_[i]->collision_geometry_, transform2fcl(link_states[i]->getGlobalCollisionBodyTransform()));
      fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(collObj));
      // the CollisionGeometryData is already stored in the class member geoms_, so we need not copy it
    }
    std::vector<const robot_state::AttachedBody*> ab;
    link_states[i]->getAttachedBodies(ab);
    for (std::size_t j = 0 ; j < ab.size() ; ++j)
    {
      std::vector<FCLGeometryConstPtr> objs;
      getAttachedBodyObjects(ab[j], objs);
      const EigenSTL::vector_Affine3d &ab_t = ab[j]->getGlobalCollisionBodyTransforms();
      for (std::size_t k = 0 ; k < objs.size() ; ++k)
        if (objs[k]->collision_geometry_)
        {
          fcl::CollisionObject *collObj = new fcl::CollisionObject(objs[k]->collision_geometry_, transform2fcl(ab_t[k]));
          fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(collObj));
          // we copy the shared ptr to the CollisionGeometryData, as this is not stored by the class itself,
          // and would be destroyed when objs goes out of scope.
          fcl_obj.collision_geometry_.push_back(objs[k]);
        }
    }
  }
}

void collision_detection::CollisionRobotFCL::allocSelfCollisionBroadPhase(const robot_state::RobotState &state, FCLManager &manager) const
{
  fcl::DynamicAABBTreeCollisionManager* m = new fcl::DynamicAABBTreeCollisionManager();
  // m->tree_init_level = 2;
  manager.manager_.reset(m);
  constructFCLObject(state, manager.object_);
  manager.object_.registerTo(manager.manager_.get());
  // manager.manager_->update();
}

void collision_detection::CollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state) const
{
  checkSelfCollisionHelper(req, res, state, NULL);
}

void collision_detection::CollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                const AllowedCollisionMatrix &acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void collision_detection::CollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2) const
{
  logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2, const AllowedCollisionMatrix &acm) const
{
  logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotFCL::checkSelfCollisionHelper(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                      const AllowedCollisionMatrix *acm) const
{
  FCLManager manager;
  allocSelfCollisionBroadPhase(state, manager);
  CollisionData cd(&req, &res, acm);
  cd.enableGroup(getRobotModel());
  manager.manager_->collide(&cd, &collisionCallback);
  if (req.distance)
    res.distance = distanceSelfHelper(state, acm);
}

void collision_detection::CollisionRobotFCL::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, NULL);
}

void collision_detection::CollisionRobotFCL::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state,
                                                                 const AllowedCollisionMatrix &acm) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, &acm);
}

void collision_detection::CollisionRobotFCL::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2) const
{
  logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotFCL::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2,
                                                                 const AllowedCollisionMatrix &acm) const
{
  logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotFCL::checkOtherCollisionHelper(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                       const CollisionRobot &other_robot, const robot_state::RobotState &other_state,
                                                                       const AllowedCollisionMatrix *acm) const
{
  FCLManager manager;
  allocSelfCollisionBroadPhase(state, manager);

  const CollisionRobotFCL &fcl_rob = dynamic_cast<const CollisionRobotFCL&>(other_robot);
  FCLObject other_fcl_obj;
  fcl_rob.constructFCLObject(other_state, other_fcl_obj);

  CollisionData cd(&req, &res, acm);
  cd.enableGroup(getRobotModel());
  for (std::size_t i = 0 ; !cd.done_ && i < other_fcl_obj.collision_objects_.size() ; ++i)
    manager.manager_->collide(other_fcl_obj.collision_objects_[i].get(), &cd, &collisionCallback);
  if (req.distance)
    res.distance = distanceOtherHelper(state, other_robot, other_state, acm);
}

void collision_detection::CollisionRobotFCL::updatedPaddingOrScaling(const std::vector<std::string> &links)
{
  for (std::size_t i = 0 ; i < links.size() ; ++i)
  {
    std::map<std::string, std::size_t>::const_iterator it = index_map_.find(links[i]);
    const robot_model::LinkModel *lmodel = kmodel_->getLinkModel(links[i]);
    if (it != index_map_.end() && lmodel)
    {
      FCLGeometryConstPtr g = createCollisionGeometry(lmodel->getShape(), getLinkScale(links[i]), getLinkPadding(links[i]), lmodel);
      geoms_[it->second] = g;
    }
    else
      logError("Updating padding or scaling for unknown link: '%s'", links[i].c_str());
  }
}

double collision_detection::CollisionRobotFCL::distanceSelf(const robot_state::RobotState &state) const
{
  return distanceSelfHelper(state, NULL);
}

double collision_detection::CollisionRobotFCL::distanceSelf(const robot_state::RobotState &state,
                                                            const AllowedCollisionMatrix &acm) const
{
  return distanceSelfHelper(state, &acm);
}

double collision_detection::CollisionRobotFCL::distanceSelfHelper(const robot_state::RobotState &state,
                                                                  const AllowedCollisionMatrix *acm) const
{
  FCLManager manager;
  allocSelfCollisionBroadPhase(state, manager);

  CollisionRequest req;
  CollisionResult res;
  CollisionData cd(&req, &res, acm);
  cd.enableGroup(getRobotModel());

  manager.manager_->distance(&cd, &distanceCallback);

  return res.distance;
}

double collision_detection::CollisionRobotFCL::distanceOther(const robot_state::RobotState &state,
                                                             const CollisionRobot &other_robot,
                                                             const robot_state::RobotState &other_state) const
{
  return distanceOtherHelper(state, other_robot, other_state, NULL);
}

double collision_detection::CollisionRobotFCL::distanceOther(const robot_state::RobotState &state,
                                                             const CollisionRobot &other_robot,
                                                             const robot_state::RobotState &other_state,
                                                             const AllowedCollisionMatrix &acm) const
{
  return distanceOtherHelper(state, other_robot, other_state, &acm);
}

double collision_detection::CollisionRobotFCL::distanceOtherHelper(const robot_state::RobotState &state,
                                                                   const CollisionRobot &other_robot,
                                                                   const robot_state::RobotState &other_state,
                                                                   const AllowedCollisionMatrix *acm) const
{
  FCLManager manager;
  allocSelfCollisionBroadPhase(state, manager);

  const CollisionRobotFCL& fcl_rob = dynamic_cast<const CollisionRobotFCL&>(other_robot);
  FCLObject other_fcl_obj;
  fcl_rob.constructFCLObject(other_state, other_fcl_obj);

  CollisionRequest req;
  CollisionResult res;
  CollisionData cd(&req, &res, acm);
  cd.enableGroup(getRobotModel());
  for(std::size_t i = 0; !cd.done_ && i < other_fcl_obj.collision_objects_.size(); ++i)
    manager.manager_->distance(other_fcl_obj.collision_objects_[i].get(), &cd, &distanceCallback);

  return res.distance;
}
