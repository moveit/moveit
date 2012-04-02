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

#ifndef COLLISION_DETECTION_FCL_COLLISION_COMMON_
#define COLLISION_DETECTION_FCL_COLLISION_COMMON_

#include "collision_detection/collision_world.h"
#include <fcl/broad_phase_collision.h>
#include <fcl/collision.h>

namespace collision_detection
{

struct CollisionGeometryData
{
  CollisionGeometryData(const planning_models::KinematicModel::LinkModel *link) : type(BodyTypes::ROBOT_LINK)
  {
    ptr.link = link;
  }
  
  CollisionGeometryData(const planning_models::KinematicState::AttachedBody *ab) : type(BodyTypes::ROBOT_ATTACHED)
  {
    ptr.ab = ab;
  }
  
  CollisionGeometryData(const CollisionWorld::Object *obj) : type(BodyTypes::WORLD_OBJECT)
  {
    ptr.obj = obj;
  }
  
  const std::string& getID(void) const
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
  
  BodyType type;
  union
  {
    const planning_models::KinematicModel::LinkModel    *link;
    const planning_models::KinematicState::AttachedBody *ab;
    const CollisionWorld::Object                        *obj;
  } ptr;
};

struct CollisionData
{
  CollisionData(void) : req_(NULL), res_(NULL), acm_(NULL), done_(false)
  {
  }
  
  CollisionData(const CollisionRequest *req, CollisionResult *res,
                const AllowedCollisionMatrix *acm) : req_(req), res_(res), acm_(acm), done_(false)
  {
  }
  
  const CollisionRequest       *req_;
  CollisionResult              *res_;
  const AllowedCollisionMatrix *acm_;
  bool                          done_;
};

struct FCLGeometry
{
  FCLGeometry(void)
  {
  }
  
  FCLGeometry(fcl::CollisionGeometry *collision_geometry, const planning_models::KinematicModel::LinkModel *link) :
    collision_geometry_(collision_geometry), collision_geometry_data_(new CollisionGeometryData(link))
  {
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  FCLGeometry(fcl::CollisionGeometry *collision_geometry, const planning_models::KinematicState::AttachedBody *ab) :
    collision_geometry_(collision_geometry), collision_geometry_data_(new CollisionGeometryData(ab))
  {
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  FCLGeometry(fcl::CollisionGeometry *collision_geometry, const CollisionWorld::Object *obj) :
    collision_geometry_(collision_geometry), collision_geometry_data_(new CollisionGeometryData(obj))
  {
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }

  template<typename T>
  void updateCollisionGeometryData(const T* data)
  {
    collision_geometry_data_.reset(new CollisionGeometryData(data));
    collision_geometry_->setUserData(collision_geometry_data_.get());
  }
  
  boost::shared_ptr<fcl::CollisionGeometry> collision_geometry_;
  boost::shared_ptr<CollisionGeometryData>  collision_geometry_data_;
};

typedef boost::shared_ptr<FCLGeometry> FCLGeometryPtr;
typedef boost::shared_ptr<const FCLGeometry> FCLGeometryConstPtr;

struct FCLObject
{
  void registerTo(fcl::BroadPhaseCollisionManager *manager);
  void unregisterFrom(fcl::BroadPhaseCollisionManager *manager);
  void clear(void);
  
  std::vector<boost::shared_ptr<fcl::CollisionObject> > collision_objects_;
  std::vector<FCLGeometryConstPtr> collision_geometry_;
};

struct FCLManager
{
  FCLObject                                          object_;
  boost::shared_ptr<fcl::BroadPhaseCollisionManager> manager_;
};

bool collisionCallback(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data);

FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::StaticShapeConstPtr &shape,
                                            const planning_models::KinematicModel::LinkModel *link);
FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::StaticShapeConstPtr &shape,
                                            const planning_models::KinematicState::AttachedBody *ab);
FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::StaticShapeConstPtr &shape,
                                            const CollisionWorld::Object *obj);

FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::ShapeConstPtr &shape,
                                            const planning_models::KinematicModel::LinkModel *link);
FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::ShapeConstPtr &shape,
                                            const planning_models::KinematicState::AttachedBody *ab);
FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::ShapeConstPtr &shape,
                                            const CollisionWorld::Object *obj);

FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::ShapeConstPtr &shape, double scale, double padding,
                                            const planning_models::KinematicModel::LinkModel *link);
FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::ShapeConstPtr &shape, double scale, double padding,
                                            const planning_models::KinematicState::AttachedBody *ab);
FCLGeometryConstPtr createCollisionGeometry(bool obb, const shapes::ShapeConstPtr &shape, double scale, double padding,
                                            const CollisionWorld::Object *obj);

inline void transform2fcl(const Eigen::Affine3d &b, fcl::SimpleTransform &f)
{
  Eigen::Quaterniond q(b.rotation());
  f.setTranslation(fcl::Vec3f(b.translation().x(), b.translation().y(), b.translation().z()));
  f.setQuatRotation(fcl::SimpleQuaternion(q.w(), q.x(), q.y(), q.z()));
}

inline fcl::SimpleTransform transform2fcl(const Eigen::Affine3d &b)
{
  fcl::SimpleTransform t;
  transform2fcl(b, t);
  return t;
}

inline void fcl2contact(const fcl::Contact &fc, Contact &c)
{
  c.pos = Eigen::Vector3d(fc.pos[0], fc.pos[1], fc.pos[2]);
  c.normal = Eigen::Vector3d(fc.normal[0], fc.normal[1], fc.normal[2]);
  c.depth = fc.penetration_depth;
  const CollisionGeometryData *cgd1 = static_cast<const CollisionGeometryData*>(fc.o1->getUserData());
  c.body_name_1 = cgd1->getID();
  c.body_type_1 = cgd1->type;
  const CollisionGeometryData *cgd2 = static_cast<const CollisionGeometryData*>(fc.o2->getUserData());
  c.body_name_2 = cgd2->getID();
  c.body_type_2 = cgd2->type;
}

}

#endif
