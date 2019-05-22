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

#ifndef MOVEIT_COLLISION_DETECTION_FCL_COLLISION_COMMON_
#define MOVEIT_COLLISION_DETECTION_FCL_COLLISION_COMMON_

#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/macros/class_forward.h>
#include <moveit/collision_detection_fcl/fcl_compat.h>

#if (MOVEIT_FCL_VERSION >= FCL_VERSION_CHECK(0, 6, 0))
#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#else
#include <fcl/broadphase/broadphase.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#endif

#include <memory>
#include <set>

namespace collision_detection
{
MOVEIT_STRUCT_FORWARD(CollisionGeometryData);

struct CollisionGeometryData
{
  CollisionGeometryData(const robot_model::LinkModel* link, int index) : type(BodyTypes::ROBOT_LINK), shape_index(index)
  {
    ptr.link = link;
  }

  CollisionGeometryData(const robot_state::AttachedBody* ab, int index)
    : type(BodyTypes::ROBOT_ATTACHED), shape_index(index)
  {
    ptr.ab = ab;
  }

  CollisionGeometryData(const World::Object* obj, int index) : type(BodyTypes::WORLD_OBJECT), shape_index(index)
  {
    ptr.obj = obj;
  }

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

  /** \brief Check if two CollisionGeometryData objects point to the same source object */
  bool sameObject(const CollisionGeometryData& other) const
  {
    return type == other.type && ptr.raw == other.ptr.raw;
  }

  BodyType type;
  int shape_index;
  union
  {
    const robot_model::LinkModel* link;
    const robot_state::AttachedBody* ab;
    const World::Object* obj;
    const void* raw;
  } ptr;
};

struct CollisionData
{
  CollisionData() : req_(NULL), active_components_only_(NULL), res_(NULL), acm_(NULL), done_(false)
  {
  }

  CollisionData(const CollisionRequest* req, CollisionResult* res, const AllowedCollisionMatrix* acm)
    : req_(req), active_components_only_(NULL), res_(res), acm_(acm), done_(false)
  {
  }

  ~CollisionData()
  {
  }

  /// Compute \e active_components_only_ based on \e req_
  void enableGroup(const robot_model::RobotModelConstPtr& robot_model);

  /// The collision request passed by the user
  const CollisionRequest* req_;

  /// If the collision request includes a group name, this set contains the pointers to the link models that are
  /// considered for collision;
  /// If the pointer is NULL, all collisions are considered.
  const std::set<const robot_model::LinkModel*>* active_components_only_;

  /// The user specified response location
  CollisionResult* res_;

  /// The user specified collision matrix (may be NULL)
  const AllowedCollisionMatrix* acm_;

  /// Flag indicating whether collision checking is complete
  bool done_;
};

struct DistanceData
{
  DistanceData(const DistanceRequest* req, DistanceResult* res) : req(req), res(res), done(false)
  {
  }
  ~DistanceData()
  {
  }

  /// Distance query request information
  const DistanceRequest* req;

  /// Distance query results information
  DistanceResult* res;

  /// Indicates if distance query is finished.
  bool done;
};

MOVEIT_STRUCT_FORWARD(FCLGeometry);

struct FCLGeometry
{
  FCLGeometry()
  {
  }

  FCLGeometry(fcl::CollisionGeometryd* collision_geometry, const robot_model::LinkModel* link, int shape_index)
    : collision_geometry_(collision_geometry), collision_geometry_data_(new CollisionGeometryData(link, shape_index))
  {
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  FCLGeometry(fcl::CollisionGeometryd* collision_geometry, const robot_state::AttachedBody* ab, int shape_index)
    : collision_geometry_(collision_geometry), collision_geometry_data_(new CollisionGeometryData(ab, shape_index))
  {
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  FCLGeometry(fcl::CollisionGeometryd* collision_geometry, const World::Object* obj, int shape_index)
    : collision_geometry_(collision_geometry), collision_geometry_data_(new CollisionGeometryData(obj, shape_index))
  {
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  template <typename T>
  void updateCollisionGeometryData(const T* data, int shape_index, bool newType)
  {
    if (!newType && collision_geometry_data_)
      if (collision_geometry_data_->ptr.raw == reinterpret_cast<const void*>(data))
        return;
    collision_geometry_data_.reset(new CollisionGeometryData(data, shape_index));
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  std::shared_ptr<fcl::CollisionGeometryd> collision_geometry_;
  CollisionGeometryDataPtr collision_geometry_data_;
};

typedef std::shared_ptr<fcl::CollisionObjectd> FCLCollisionObjectPtr;
typedef std::shared_ptr<const fcl::CollisionObjectd> FCLCollisionObjectConstPtr;

struct FCLObject
{
  void registerTo(fcl::BroadPhaseCollisionManagerd* manager);
  void unregisterFrom(fcl::BroadPhaseCollisionManagerd* manager);
  void clear();

  std::vector<FCLCollisionObjectPtr> collision_objects_;
  std::vector<FCLGeometryConstPtr> collision_geometry_;
};

struct FCLManager
{
  FCLObject object_;
  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> manager_;
};

bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data);

bool distanceCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& min_dist);

FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, const robot_model::LinkModel* link,
                                            int shape_index);
FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, const robot_state::AttachedBody* ab,
                                            int shape_index);
FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, const World::Object* obj);

FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, double scale, double padding,
                                            const robot_model::LinkModel* link, int shape_index);
FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, double scale, double padding,
                                            const robot_state::AttachedBody* ab, int shape_index);
FCLGeometryConstPtr createCollisionGeometry(const shapes::ShapeConstPtr& shape, double scale, double padding,
                                            const World::Object* obj);
void cleanCollisionGeometryCache();

inline void transform2fcl(const Eigen::Isometry3d& b, fcl::Transform3d& f)
{
#if (MOVEIT_FCL_VERSION >= FCL_VERSION_CHECK(0, 6, 0))
  f = b.matrix();
#else
  Eigen::Quaterniond q(b.rotation());
  f.setTranslation(fcl::Vector3d(b.translation().x(), b.translation().y(), b.translation().z()));
  f.setQuatRotation(fcl::Quaternion3f(q.w(), q.x(), q.y(), q.z()));
#endif
}

inline fcl::Transform3d transform2fcl(const Eigen::Isometry3d& b)
{
  fcl::Transform3d t;
  transform2fcl(b, t);
  return t;
}

inline void fcl2contact(const fcl::Contactd& fc, Contact& c)
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

inline void fcl2costsource(const fcl::CostSourced& fcs, CostSource& cs)
{
  cs.aabb_min[0] = fcs.aabb_min[0];
  cs.aabb_min[1] = fcs.aabb_min[1];
  cs.aabb_min[2] = fcs.aabb_min[2];

  cs.aabb_max[0] = fcs.aabb_max[0];
  cs.aabb_max[1] = fcs.aabb_max[1];
  cs.aabb_max[2] = fcs.aabb_max[2];

  cs.cost = fcs.cost_density;
}
}

#endif
