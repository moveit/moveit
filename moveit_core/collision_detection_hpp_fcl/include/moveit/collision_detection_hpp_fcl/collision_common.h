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

#pragma once

#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_env.h>
#include <moveit/macros/class_forward.h>
#include <moveit/collision_detection_hpp_fcl/fcl_compat.h>
#include <geometric_shapes/check_isometry.h>

#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/data_types.h>

#include <memory>
#include <set>

namespace collision_detection
{
MOVEIT_STRUCT_FORWARD(CollisionGeometryData);

/** \brief Wrapper around world, link and attached objects' geometry data. */
struct CollisionGeometryData
{
  /** \brief Constructor for a robot link collision geometry object. */
  CollisionGeometryData(const moveit::core::LinkModel* link, int index)
    : type(BodyTypes::ROBOT_LINK), shape_index(index)
  {
    ptr.link = link;
  }

  /** \brief Constructor for a new collision geometry object which is attached to the robot. */
  CollisionGeometryData(const moveit::core::AttachedBody* ab, int index)
    : type(BodyTypes::ROBOT_ATTACHED), shape_index(index)
  {
    ptr.ab = ab;
  }

  /** \brief Constructor for a new world collision geometry. */
  CollisionGeometryData(const World::Object* obj, int index) : type(BodyTypes::WORLD_OBJECT), shape_index(index)
  {
    ptr.obj = obj;
  }

  /** \brief Returns the name which is saved in the member pointed to in \e ptr. */
  const std::string& getID() const
  {
    switch (type)
    {
      case BodyTypes::ROBOT_LINK:
        return ptr.link->getName();
      case BodyTypes::ROBOT_ATTACHED:
        return ptr.ab->getName();
      default:
        break;
    }
    return ptr.obj->id_;
  }

  /** \brief Returns a string of the corresponding \e type. */
  std::string getTypeString() const
  {
    switch (type)
    {
      case BodyTypes::ROBOT_LINK:
        return "Robot link";
      case BodyTypes::ROBOT_ATTACHED:
        return "Robot attached";
      default:
        break;
    }
    return "Object";
  }

  /** \brief Check if two CollisionGeometryData objects point to the same source object. */
  bool sameObject(const CollisionGeometryData& other) const
  {
    return type == other.type && ptr.raw == other.ptr.raw;
  }

  /** \brief Indicates the body type of the object. */
  BodyType type;

  /** \brief Multiple \e CollisionGeometryData objects construct a collision object. The collision object refers to an
   *  array of coordinate transformations at a certain start index. The index of the transformation of a child \e
   *  CollisionGeometryData object is then given by adding the parent collision object index and the \e shape_index of a
   *  geometry data object. */
  int shape_index;

  /** \brief Points to the type of body which contains the geometry. */
  union
  {
    const moveit::core::LinkModel* link;
    const moveit::core::AttachedBody* ab;
    const World::Object* obj;
    const void* raw;
  } ptr;
};

/** \brief Data structure which is passed to the collision callback function of the collision manager. */
struct CollisionData
{
  CollisionData() : req_(nullptr), active_components_only_(nullptr), res_(nullptr), acm_(nullptr), done_(false)
  {
  }

  CollisionData(const CollisionRequest* req, CollisionResult* res, const AllowedCollisionMatrix* acm)
    : req_(req), active_components_only_(nullptr), res_(res), acm_(acm), done_(false)
  {
  }

  ~CollisionData()
  {
  }

  /** \brief Compute \e active_components_only_ based on the joint group specified in \e req_ */
  void enableGroup(const moveit::core::RobotModelConstPtr& robot_model);

  /** \brief The collision request passed by the user */
  const CollisionRequest* req_;

  /** \brief  If the collision request includes a group name, this set contains the pointers to the link models that
   *  are considered for collision.
   *
   *  If the pointer is NULL, all collisions are considered. */
  const std::set<const moveit::core::LinkModel*>* active_components_only_;

  /** \brief The user-specified response location. */
  CollisionResult* res_;

  /** \brief The user-specified collision matrix (may be NULL). */
  const AllowedCollisionMatrix* acm_;

  /** \brief Flag indicating whether collision checking is complete. */
  bool done_;
};

/** \brief Data structure which is passed to the distance callback function of the collision manager. */
struct DistanceData
{
  DistanceData(const DistanceRequest* req, DistanceResult* res) : req(req), res(res), done(false)
  {
  }
  ~DistanceData()
  {
  }

  /** \brief Distance query request information. */
  const DistanceRequest* req;

  /** \brief Distance query results information. */
  DistanceResult* res;

  /** \brief Indicates if distance query is finished. */
  bool done;
};

MOVEIT_STRUCT_FORWARD(HPPFCLGeometry);

/** \brief Bundles the \e CollisionGeometryData and FCL collision geometry representation into a single class. */
struct HPPFCLGeometry
{
  HPPFCLGeometry()
  {
  }

  /** \brief Constructor for a robot link. */
  HPPFCLGeometry(hpp::fcl::CollisionGeometry* collision_geometry, const moveit::core::LinkModel* link, int shape_index)
    : collision_geometry_(collision_geometry), collision_geometry_data_(new CollisionGeometryData(link, shape_index))
  {
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  /** \brief Constructor for an attached body. */
  HPPFCLGeometry(hpp::fcl::CollisionGeometry* collision_geometry, const moveit::core::AttachedBody* ab, int shape_index)
    : collision_geometry_(collision_geometry), collision_geometry_data_(new CollisionGeometryData(ab, shape_index))
  {
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  /** \brief Constructor for a world object. */
  HPPFCLGeometry(hpp::fcl::CollisionGeometry* collision_geometry, const World::Object* obj, int shape_index)
    : collision_geometry_(collision_geometry), collision_geometry_data_(new CollisionGeometryData(obj, shape_index))
  {
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  /** \brief Updates the \e collision_geometry_data_ with new data while also setting the \e collision_geometry_ to the
   *   new data. */
  template <typename T>
  void updateCollisionGeometryData(const T* data, int shape_index, bool newType)
  {
    if (!newType && collision_geometry_data_)
      if (collision_geometry_data_->ptr.raw == reinterpret_cast<const void*>(data))
        return;
    collision_geometry_data_ = std::make_shared<CollisionGeometryData>(data, shape_index);
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  /** \brief Pointer to FCL collision geometry. */
  std::shared_ptr<hpp::fcl::CollisionGeometry> collision_geometry_;

  /** \brief Pointer to the user-defined geometry data. */
  CollisionGeometryDataPtr collision_geometry_data_;
};

typedef std::shared_ptr<hpp::fcl::CollisionObject> HPPFCLCollisionObjectPtr;
typedef std::shared_ptr<const hpp::fcl::CollisionObject> HPPFCLCollisionObjectConstPtr;

/** \brief A general high-level object which consists of multiple \e FCLCollisionObjects. It is the top level data
 *  structure which is used in the collision checking process. */
struct HPPFCLObject
{
  void registerTo(hpp::fcl::BroadPhaseCollisionManager* manager);
  void unregisterFrom(hpp::fcl::BroadPhaseCollisionManager* manager);
  void clear();

  std::vector<HPPFCLCollisionObjectPtr> collision_objects_;

  /** \brief Geometry data corresponding to \e collision_objects_. */
  std::vector<HPPFCLGeometryConstPtr> collision_geometry_;
};

/** \brief Bundles an \e FCLObject and a broadphase FCL collision manager. */
struct FCLManager
{
  HPPFCLObject object_;
  std::shared_ptr<hpp::fcl::BroadPhaseCollisionManager> manager_;
};

/** \brief Callback function used by the FCLManager used for each pair of collision objects to
 *   calculate object contact information.
 *
 *   \param o1 First FCL collision object
 *   \param o2 Second FCL collision object
 *   \data General pointer to arbitrary data which is used during the callback
 *   \return True terminates the collision check, false continues it to the next pair of objects */
struct CollisionCallback : hpp::fcl::CollisionCallBackBase
{
  CollisionData* data;
  virtual bool collide(hpp::fcl::CollisionObject* o1, hpp::fcl::CollisionObject* o2) override;
};

/** \brief Callback function used by the FCLManager used for each pair of collision objects to
 *   calculate collisions and distances.
 *
 *   \param o1 First FCL collision object
 *   \param o2 Second FCL collision object
 *   \data General pointer to arbitrary data which is used during the callback
 *   \return True terminates the distance check, false continues it to the next pair of objects */
// bool distanceCallback(hpp::fcl::CollisionObjectd* o1, hpp::fcl::CollisionObjectd* o2, void* data, double& min_dist);
struct DistanceCallback : public hpp::fcl::DistanceCallBackBase
{
  DistanceData* data;

  virtual bool distance(hpp::fcl::CollisionObject* o1, hpp::fcl::CollisionObject* o2, hpp::fcl::FCL_REAL& dist) override;
};

/** \brief Create new HPPFCLGeometry object out of robot link model. */
HPPFCLGeometryConstPtr createCollisionGeometryHppFcl(const shapes::ShapeConstPtr& shape,
                                                     const moveit::core::LinkModel* link, int shape_index);

/** \brief Create new HPPFCLGeometry object out of attached body. */
HPPFCLGeometryConstPtr createCollisionGeometryHppFcl(const shapes::ShapeConstPtr& shape,
                                                     const moveit::core::AttachedBody* ab, int shape_index);

/** \brief Create new HPPFCLGeometry object out of a world object.
 *
 *  A world object always consists only of a single shape, therefore we don't need the \e shape_index. */
HPPFCLGeometryConstPtr createCollisionGeometryHppFcl(const shapes::ShapeConstPtr& shape, const World::Object* obj);

/** \brief Create new scaled and / or padded HPPFCLGeometry object out of robot link model. */
HPPFCLGeometryConstPtr createCollisionGeometryHppFcl(const shapes::ShapeConstPtr& shape, double scale, double padding,
                                                     const moveit::core::LinkModel* link, int shape_index);

/** \brief Create new scaled and / or padded HPPFCLGeometry object out of an attached body. */
HPPFCLGeometryConstPtr createCollisionGeometryHppFcl(const shapes::ShapeConstPtr& shape, double scale, double padding,
                                                     const moveit::core::AttachedBody* ab, int shape_index);

/** \brief Create new scaled and / or padded HPPFCLGeometry object out of an world object. */
HPPFCLGeometryConstPtr createCollisionGeometryHppFcl(const shapes::ShapeConstPtr& shape, double scale, double padding,
                                                     const World::Object* obj);

/** \brief Increases the counter of the caches which can trigger the cleaning of expired entries from them. */
void cleanCollisionGeometryCache();

/** \brief Transforms an Eigen Isometry3d to FCL coordinate transformation */
inline void transform2fcl(const Eigen::Isometry3d& b, hpp::fcl::Transform3f& f)
{
  ASSERT_ISOMETRY(b);
  Eigen::Quaterniond q(b.linear());
  f.setTranslation(hpp::fcl::Vec3f(b.translation().x(), b.translation().y(), b.translation().z()));
  f.setQuatRotation(hpp::fcl::Quaternion3f(q.w(), q.x(), q.y(), q.z()));
}

/** \brief Transforms an Eigen Isometry3d to FCL coordinate transformation */
inline hpp::fcl::Transform3f transform2fcl(const Eigen::Isometry3d& b)
{
  hpp::fcl::Transform3f t;
  transform2fcl(b, t);
  return t;
}

/** \brief Transforms an FCL contact into a MoveIt contact point. */
inline void hppfcl2contact(const hpp::fcl::Contact& fc, Contact& c)
{
  c.pos = Eigen::Vector3d(fc.pos[0], fc.pos[1], fc.pos[2]);
  c.normal = Eigen::Vector3d(fc.normal[0], fc.normal[1], fc.normal[2]);
  c.depth = fc.penetration_depth;
  const CollisionGeometryData* cgd1 = static_cast<const CollisionGeometryData*>(fc.o1->getUserData());
  c.body_name_1 = cgd1->getID();
  c.body_type_1 = cgd1->type;
  const CollisionGeometryData* cgd2 = static_cast<const CollisionGeometryData*>(fc.o2->getUserData());
  c.body_name_2 = cgd2->getID();
  c.body_type_2 = cgd2->type;
}

/** \brief Transforms the FCL internal representation from a Collision Geometry to the MoveIt \e CostSource data structure. */
inline void hppfcl2costsource(const hpp::fcl::CollisionGeometry& fcs, CostSource& cs)
{
  cs.aabb_min[0] = fcs.aabb_local.min_[0];
  cs.aabb_min[1] = fcs.aabb_local.min_[1];
  cs.aabb_min[2] = fcs.aabb_local.min_[2];

  cs.aabb_max[0] = fcs.aabb_local.max_[0];
  cs.aabb_max[1] = fcs.aabb_local.max_[1];
  cs.aabb_max[2] = fcs.aabb_local.max_[2];

  cs.cost = fcs.cost_density;
}
}  // namespace collision_detection
