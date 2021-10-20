/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, MakinaRocks, Inc.
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
 *   * Neither the name of MakinaRocks nor the names of its
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

/* Author: Vinnam Kim */

#include <moveit/collision_detection_fcl/mrx_custom/collision_env_mrx.h>
#include <moveit/collision_detection_fcl/mrx_custom/collision_detector_allocator_mrx.h>
#include <moveit/collision_detection_fcl/collision_common.h>

#include <moveit/collision_detection_fcl/fcl_compat.h>

#if (MOVEIT_FCL_VERSION >= FCL_VERSION_CHECK(0, 6, 0))
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#endif

inline bool validateScale(double scale)
{
  if (scale < std::numeric_limits<double>::epsilon())
  {
    ROS_ERROR_NAMED("collision_detection", "Scale must be positive");
    return false;
  }
  if (scale > std::numeric_limits<double>::max())
  {
    ROS_ERROR_NAMED("collision_detection", "Scale must be finite");
    return false;
  }
  return true;
}

inline bool validatePadding(double padding)
{
  if (padding < 0.0)
  {
    ROS_ERROR_NAMED("collision_detection", "Padding cannot be negative");
    return false;
  }
  if (padding > std::numeric_limits<double>::max())
  {
    ROS_ERROR_NAMED("collision_detection", "Padding must be finite");
    return false;
  }
  return true;
}

namespace collision_detection
{
const std::string CollisionDetectorAllocatorMRX::NAME("FCLMRX");
constexpr char LOGNAME[] = "collision_detection.mrx";

CollisionEnvMRX::CollisionEnvMRX(const moveit::core::RobotModelConstPtr& model, double padding, double scale)
  : CollisionEnvFCL(model, 0.0, 1.0)
{
  initPaddedRobotObjects(padding, scale);
}

CollisionEnvMRX::CollisionEnvMRX(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world, double padding,
                                 double scale)
  : CollisionEnvFCL(model, world, 0.0, 1.0)
{
  initPaddedRobotObjects(padding, scale);
}

CollisionEnvMRX::CollisionEnvMRX(const CollisionEnvMRX& other, const WorldPtr& world) : CollisionEnvFCL(other, world)
{
  padded_robot_geoms_ = other.padded_robot_geoms_;
  padded_robot_fcl_objs_ = other.padded_robot_fcl_objs_;
}

CollisionEnvMRX::~CollisionEnvMRX()
{
}

void CollisionEnvMRX::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                         const moveit::core::RobotState& state) const
{
  checkSelfCollisionHelper(req, res, state, nullptr);
}

void CollisionEnvMRX::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                         const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void CollisionEnvMRX::distanceSelf(const DistanceRequest& req, DistanceResult& res,
                                   const moveit::core::RobotState& state) const
{
  FCLManager manager;
  const auto unpadded_geometry_flags = getUnpaddedGeometryFlags(req);
  allocSelfCollisionBroadPhase(state, manager, unpadded_geometry_flags);
  DistanceData drd(&req, &res);

  manager.manager_->distance(&drd, &distanceCallback);
}

void CollisionEnvMRX::setWorld(const WorldPtr& world)
{
  CollisionEnvFCL::setWorld(world);
}

void CollisionEnvMRX::updatedPaddingOrScaling(const std::vector<std::string>& links)
{
  std::size_t index;
  for (const auto& link : links)
  {
    const moveit::core::LinkModel* lmodel = robot_model_->getLinkModel(link);
    if (lmodel)
    {
      for (std::size_t j = 0; j < lmodel->getShapes().size(); ++j)
      {
        FCLGeometryConstPtr g = createCollisionGeometry(lmodel->getShapes()[j], getLinkScale(lmodel->getName()),
                                                        getLinkPadding(lmodel->getName()), lmodel, j);
        if (g)
        {
          index = lmodel->getFirstCollisionBodyTransformIndex() + j;
          padded_robot_geoms_[index] = g;
          padded_robot_fcl_objs_[index] = FCLCollisionObjectConstPtr(new fcl::CollisionObjectd(g->collision_geometry_));
        }
      }
    }
    else
      ROS_ERROR_NAMED(LOGNAME, "Updating padding or scaling for unknown link: '%s'", link.c_str());
  }
}

void CollisionEnvMRX::constructFCLObjectRobot(const moveit::core::RobotState& state, FCLObject& fcl_obj,
                                              const std::vector<bool>& unpadded_geometry_flags) const
{
  assert(unpadded_geometry_flags.size() == robot_geoms_.size());

  fcl_obj.collision_objects_.reserve(robot_geoms_.size());
  fcl::Transform3d fcl_tf;

  for (std::size_t i = 0; i < robot_geoms_.size(); ++i)
  {
    if (unpadded_geometry_flags[i] && robot_geoms_[i] && robot_geoms_[i]->collision_geometry_)
    {
      transform2fcl(state.getCollisionBodyTransform(robot_geoms_[i]->collision_geometry_data_->ptr.link,
                                                    robot_geoms_[i]->collision_geometry_data_->shape_index),
                    fcl_tf);
      auto coll_obj = new fcl::CollisionObjectd(*robot_fcl_objs_[i]);
      coll_obj->setTransform(fcl_tf);
      coll_obj->computeAABB();
      fcl_obj.collision_objects_.push_back(FCLCollisionObjectPtr(coll_obj));
    }
    else if (!unpadded_geometry_flags[i] && padded_robot_geoms_[i] && padded_robot_geoms_[i]->collision_geometry_)
    {
      transform2fcl(state.getCollisionBodyTransform(padded_robot_geoms_[i]->collision_geometry_data_->ptr.link,
                                                    padded_robot_geoms_[i]->collision_geometry_data_->shape_index),
                    fcl_tf);
      auto coll_obj = new fcl::CollisionObjectd(*padded_robot_fcl_objs_[i]);
      coll_obj->setTransform(fcl_tf);
      coll_obj->computeAABB();
      fcl_obj.collision_objects_.push_back(FCLCollisionObjectPtr(coll_obj));
    }
  }

  // TODO: Implement a method for caching fcl::CollisionObject's for moveit::core::AttachedBody's
  std::vector<const moveit::core::AttachedBody*> ab;
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

void CollisionEnvMRX::allocSelfCollisionBroadPhase(const moveit::core::RobotState& state, FCLManager& manager,
                                                   const std::vector<bool>& unpadded_geometry_flags) const
{
  auto m = new fcl::DynamicAABBTreeCollisionManagerd();
  // m->tree_init_level = 2;
  manager.manager_.reset(m);
  constructFCLObjectRobot(state, manager.object_, unpadded_geometry_flags);
  manager.object_.registerTo(manager.manager_.get());
  // manager.manager_->update();
}

void CollisionEnvMRX::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                               const moveit::core::RobotState& state,
                                               const AllowedCollisionMatrix* acm) const
{
  FCLManager manager;
  const auto unpadded_geometry_flags = getUnpaddedGeometryFlags(req);
  allocSelfCollisionBroadPhase(state, manager, unpadded_geometry_flags);
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

void CollisionEnvMRX::initPaddedRobotObjects(double padding, double scale)
{
  if (!validateScale(scale))
    scale = 1.0;
  if (!validatePadding(padding))
    padding = 0.0;

  const std::vector<const moveit::core::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  for (auto link : links)
  {
    link_padding_[link->getName()] = padding;
    link_scale_[link->getName()] = scale;
  }

  std::size_t index;
  padded_robot_geoms_.resize(robot_model_->getLinkGeometryCount());
  padded_robot_fcl_objs_.resize(robot_model_->getLinkGeometryCount());
  // we keep the same order of objects as what RobotState *::getLinkState() returns
  for (auto link : links)
    for (std::size_t j = 0; j < link->getShapes().size(); ++j)
    {
      FCLGeometryConstPtr link_geometry = createCollisionGeometry(link->getShapes()[j], getLinkScale(link->getName()),
                                                                  getLinkPadding(link->getName()), link, j);

      if (link_geometry)
      {
        index = link->getFirstCollisionBodyTransformIndex() + j;
        padded_robot_geoms_[index] = link_geometry;

        // Need to store the FCL object so the AABB does not get recreated every time.
        // Every time this object is created, g->computeLocalAABB() is called  which is
        // very expensive and should only be calculated once. To update the AABB, use the
        // collObj->setTransform and then call collObj->computeAABB() to transform the AABB.
        padded_robot_fcl_objs_[index] =
            FCLCollisionObjectConstPtr(new fcl::CollisionObjectd(link_geometry->collision_geometry_));
      }
      else
        ROS_ERROR_NAMED(LOGNAME, "Unable to construct collision geometry for link '%s'", link->getName().c_str());
    }
}

std::vector<bool> CollisionEnvMRX::getUnpaddedGeometryFlags(const std::string& group_name) const
{
  // Default behavior is unpadded self collision
  if (group_name == "")
    return std::vector<bool>(robot_model_->getLinkGeometryCount(), true);

  std::vector<bool> unpadded_geometry_flags(robot_model_->getLinkGeometryCount(), false);

  const auto jmg = robot_model_->getJointModelGroup(group_name);

  if (jmg != nullptr)
  {
    const auto& links = jmg->getLinkModels();
    for (auto link : links)
      for (std::size_t j = 0; j < link->getShapes().size(); ++j)
      {
        const std::size_t index = link->getFirstCollisionBodyTransformIndex() + j;
        unpadded_geometry_flags[index] = true;
      }
  }

  return unpadded_geometry_flags;
}
}  // namespace collision_detection
