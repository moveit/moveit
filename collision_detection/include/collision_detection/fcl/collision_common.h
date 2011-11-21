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
#include <fcl/collision.h>

namespace collision_detection
{
    struct CollisionObjectData
    {
        CollisionObjectData(const planning_models::KinematicModel::LinkModel *link) : type(BodyTypes::ROBOT_LINK)
        {
            ptr.link = link;
        }

        CollisionObjectData(const planning_models::KinematicModel::AttachedBodyProperties *ab) : type(BodyTypes::ROBOT_ATTACHED)
        {
            ptr.ab = ab;
        }

        CollisionObjectData(const CollisionWorld::NamespaceObjects *obj) : type(BodyTypes::WORLD_OBJECT)
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
                return ptr.ab->id;
            }
            return ptr.obj->ns;
        }

        BodyType type;
        union
        {
            const planning_models::KinematicModel::LinkModel              *link;
            const planning_models::KinematicState::AttachedBodyProperties *ab;
            const CollisionWorld::NamespaceObjects                        *obj;
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

    bool collisionCallback(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data);

    fcl::CollisionObject* createCollisionObject(const shapes::StaticShape *shape);
    fcl::CollisionObject* createCollisionObject(const shapes::Shape *shape, double scale, double padding);
    fcl::CollisionObject* createCollisionObject(const shapes::Shape *shape);

    inline void bt2fcl(const btTransform &b, fcl::SimpleTransform &f)
    {
        const btVector3 &o = b.getOrigin();
        const btQuaternion &q = b.getRotation();
        f.setTranslation(fcl::Vec3f(o.x(), o.y(), o.z()));
        f.setQuatRotation(fcl::SimpleQuaternion(q.w(), q.x(), q.y(), q.z()));
    }

    inline void fcl2contact(const fcl::Contact &fc, Contact &c)
    {
        c.pos.setValue(fc.pos[0], fc.pos[1], fc.pos[2]);
        c.normal.setValue(fc.normal[0], fc.normal[1], fc.normal[2]);
        c.depth = fc.penetration_depth;
        const CollisionObjectData *cod1 = static_cast<const CollisionData*>(fc.o1->getUserData());
        c.body_name_1 = cod1->getID();
        c.body_type_1 = cod1->type;
        const CollisionObjectData *cod2 = static_cast<const CollisionData*>(fc.o2->getUserData());
        c.body_name_2 = cod2->getID();
        c.body_type_2 = cod2->type;
    }

}

#endif
