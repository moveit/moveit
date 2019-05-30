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

#include <moveit/collision_detection_bullet/collision_robot_bt.h>
#include <moveit/collision_detection_bullet/fcl_compat.h>
#include <tesseract_ros/ros_tesseract_utils.h>

#if (MOVEIT_FCL_VERSION >= FCL_VERSION_CHECK(0, 6, 0))
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#endif

namespace collision_detection
{
CollisionRobotBt::CollisionRobotBt(const robot_model::RobotModelConstPtr& model, double padding, double scale)
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
        fcl_objs_[index] = FCLCollisionObjectConstPtr(new fcl::CollisionObjectd(g->collision_geometry_));
      }
      else
        ROS_ERROR_NAMED("collision_detection.bullet", "Unable to construct collision geometry for link '%s'",
                        link->getName().c_str());
    }

  // this is from tesseract when creating a new KDLenv
  // ---------------
  for (const auto& link : robot_model_->getURDF()->links_)
  {
    if (link.second->collision_array.size() > 0)
    {
      const std::vector<urdf::CollisionSharedPtr>& col_array =
          link.second->collision_array.empty() ? std::vector<urdf::CollisionSharedPtr>(1, link.second->collision) :
                                                 link.second->collision_array;

      std::vector<shapes::ShapeConstPtr> shapes;
      tesseract::VectorIsometry3d shape_poses;
      tesseract::CollisionObjectTypeVector collision_object_types;

      for (std::size_t i = 0; i < col_array.size(); ++i)
      {
        if (col_array[i] && col_array[i]->geometry)
        {
          shapes::ShapeConstPtr s = tesseract::tesseract_ros::constructShape(col_array[i]->geometry.get());
          if (s)
          {
            shapes.push_back(s);
            shape_poses.push_back(tesseract::tesseract_ros::urdfPose2Eigen(col_array[i]->origin));

            if (s->type == shapes::MESH)
              collision_object_types.push_back(tesseract::CollisionObjectType::ConvexHull);
            else
              collision_object_types.push_back(tesseract::CollisionObjectType::UseShapeType);
          }
        }
      }
      bt_manager_.addCollisionObject(link.second->name, BodyType::ROBOT_LINK, shapes, shape_poses,
                                     collision_object_types, true);
    }
  }
  // ---------------
}

CollisionRobotBt::CollisionRobotBt(const CollisionRobotBt& other) : CollisionRobot(other)
{
  geoms_ = other.geoms_;
  fcl_objs_ = other.fcl_objs_;
  const CollisionRobotBt& other_bt = dynamic_cast<const CollisionRobotBt&>(other);
  tesseract::DiscreteContactManagerBasePtr test = other_bt.bt_manager_.clone();

  for (const auto& cow : other_bt.bt_manager_.getCollisionObjects())
  {
    tesseract::tesseract_bullet::COWPtr new_cow = cow.second->clone();
    new_cow->setWorldTransform(cow.second->getWorldTransform());

    new_cow->setContactProcessingThreshold(static_cast<btScalar>(other_bt.bt_manager_.getContactDistanceThreshold()));
    bt_manager_.addCollisionObject(new_cow);

    bt_manager_.setActiveCollisionObjects(other_bt.bt_manager_.getActiveCollisionObjects());
    bt_manager_.setContactDistanceThreshold(other_bt.bt_manager_.getContactDistanceThreshold());
    bt_manager_.setIsContactAllowedFn(other_bt.bt_manager_.getIsContactAllowedFn());
  }
}

void CollisionRobotBt::getAttachedBodyObjects(const robot_state::AttachedBody* ab,
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

void CollisionRobotBt::constructFCLObject(const robot_state::RobotState& state, FCLObject& fcl_obj) const
{
  fcl_obj.collision_objects_.reserve(geoms_.size());
  fcl::Transform3d fcl_tf;

  for (std::size_t i = 0; i < geoms_.size(); ++i)
    if (geoms_[i] && geoms_[i]->collision_geometry_)
    {
      transform2fcl(state.getCollisionBodyTransform(geoms_[i]->collision_geometry_data_->ptr.link,
                                                    geoms_[i]->collision_geometry_data_->shape_index),
                    fcl_tf);
      std::string test;
      test = geoms_[i]->collision_geometry_data_->ptr.link->getName();
      // ROS_ERROR_NAMED("collision_detection.fcl", "link '%s'", test.c_str());
      auto coll_obj = new fcl::CollisionObjectd(*fcl_objs_[i]);
      coll_obj->setTransform(fcl_tf);
      coll_obj->computeAABB();
      fcl_obj.collision_objects_.push_back(FCLCollisionObjectPtr(coll_obj));
    }

  // TODO: Implement a method for caching fcl::CollisionObject's for robot_state::AttachedBody's
  std::vector<const robot_state::AttachedBody*> ab;
  state.getAttachedBodies(ab);
  for (auto& body : ab)
  {
    std::vector<FCLGeometryConstPtr> objs;
    getAttachedBodyObjects(body, objs);
    const EigenSTL::vector_Isometry3d& ab_t = body->getGlobalCollisionBodyTransforms();
    for (std::size_t k = 0; k < objs.size(); ++k)
      if (objs[k]->collision_geometry_)
      {
        transform2fcl(ab_t[k], fcl_tf);
        fcl_obj.collision_objects_.push_back(
            FCLCollisionObjectPtr(new fcl::CollisionObjectd(objs[k]->collision_geometry_, fcl_tf)));
        // we copy the shared ptr to the CollisionGeometryData, as this is not stored by the class itself,
        // and would be destroyed when objs goes out of scope.
        fcl_obj.collision_geometry_.push_back(objs[k]);
      }
  }
}

void CollisionRobotBt::allocSelfCollisionBroadPhase(const robot_state::RobotState& state, FCLManager& manager) const
{
  auto m = new fcl::DynamicAABBTreeCollisionManagerd();
  // m->tree_init_level = 2;
  manager.manager_.reset(m);
  constructFCLObject(state, manager.object_);
  manager.object_.registerTo(manager.manager_.get());
  // manager.manager_->update();
}

void CollisionRobotBt::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                          const robot_state::RobotState& state) const
{
  checkSelfCollisionHelper(req, res, state, nullptr);
}

void CollisionRobotBt::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                          const robot_state::RobotState& state, const AllowedCollisionMatrix& acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void CollisionRobotBt::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                          const robot_state::RobotState& state1,
                                          const robot_state::RobotState& state2) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "FCL continuous collision checking not yet implemented");
}

void CollisionRobotBt::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                          const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                          const AllowedCollisionMatrix& acm) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "FCL continuous collision checking not yet implemented");
}

void CollisionRobotBt::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                const robot_state::RobotState& state,
                                                const AllowedCollisionMatrix* acm) const
{
  // updating the link position with the current robot state
  for (std::size_t i = 0; i < geoms_.size(); ++i)
    if (geoms_[i] && geoms_[i]->collision_geometry_)
    {
      bt_manager_.setCollisionObjectsTransform(
          geoms_[i]->collision_geometry_data_->ptr.link->getName(),
          state.getCollisionBodyTransform(geoms_[i]->collision_geometry_data_->ptr.link,
                                          geoms_[i]->collision_geometry_data_->shape_index));
    }

  bt_manager_.contactTest(res, tesseract::ContactTestType::FIRST, acm, req);

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

void CollisionRobotBt::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                           const robot_state::RobotState& other_state) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, nullptr);
}

void CollisionRobotBt::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                           const robot_state::RobotState& other_state,
                                           const AllowedCollisionMatrix& acm) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, &acm);
}

void CollisionRobotBt::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                           const CollisionRobot& other_robot,
                                           const robot_state::RobotState& other_state1,
                                           const robot_state::RobotState& other_state2) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "FCL continuous collision checking not yet implemented");
}

void CollisionRobotBt::checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                           const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                           const CollisionRobot& other_robot,
                                           const robot_state::RobotState& other_state1,
                                           const robot_state::RobotState& other_state2,
                                           const AllowedCollisionMatrix& acm) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "FCL continuous collision checking not yet implemented");
}

void CollisionRobotBt::checkOtherCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                 const robot_state::RobotState& state,
                                                 const CollisionRobot& other_robot,
                                                 const robot_state::RobotState& other_state,
                                                 const AllowedCollisionMatrix* acm) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Other collision not implemented yet.");
}

void CollisionRobotBt::updatedPaddingOrScaling(const std::vector<std::string>& links)
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
          fcl_objs_[index] = FCLCollisionObjectConstPtr(new fcl::CollisionObjectd(g->collision_geometry_));
        }
      }
    }
    else
      ROS_ERROR_NAMED("collision_detection.bullet", "Updating padding or scaling for unknown link: '%s'", link.c_str());
  }
}

void CollisionRobotBt::distanceSelf(const DistanceRequest& req, DistanceResult& res,
                                    const robot_state::RobotState& state) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Collision distance to self not implemented yet.");
}

void CollisionRobotBt::distanceOther(const DistanceRequest& req, DistanceResult& res,
                                     const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                     const robot_state::RobotState& other_state) const
{
  ROS_ERROR_NAMED("collision_detection.bullet", "Collision distance to other not implemented yet.");
}

}  // end of namespace collision_detection
