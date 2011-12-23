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

#include "collision_detection/fcl/collision_world.h"

collision_detection::CollisionWorldFCL::CollisionWorldFCL(void) : CollisionWorld()
{
    manager_.reset(new fcl::SSaPCollisionManager());
}


void collision_detection::CollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const planning_models::KinematicState &state) const
{
}

void collision_detection::CollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const planning_models::KinematicState &state, const AllowedCollisionMatrix &acm) const
{
}

void collision_detection::CollisionWorldFCL::checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world) const
{
    const CollisionWorldFCL *other_fcl_world = dynamic_cast<const CollisionWorldFCL*>(&other_world);
    if (other_fcl_world)
    {
    }
    else
    {
        static bool first_time = true;
        if (first_time)
        {
            first_time = false;
            ROS_WARN("Collision checking between worlds of different type. This will be slow.");
        }

        std::vector<std::string> other_ns = other_world.getNamespaces();
        for (std::size_t i = 0 ; i < other_ns.size() ; ++i)
        {
            const NamespaceObjects& obj = other_world.getObjects(other_ns[i]);
            // create collision objects for everything & call collide()
        }
    }
}

void collision_detection::CollisionWorldFCL::checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world, const AllowedCollisionMatrix &acm) const
{
}

void collision_detection::CollisionWorldFCL::addShapeToObject(const std::string &id, shapes::StaticShape *shape)
{
    CollisionWorld::addObject(ns, shape);
    fcl::CollisionObject *co = createCollisionObject(shape);
    CollisionObjectData *cod = new CollisionObjectData(&objects_[ns]);
    co->setUserData(cod);
    fcl_objs_[ns].push_back(co);
    manager_->registerObject(co);
}

void collision_detection::CollisionWorldFCL::addShapeToObject(const std::string &id, shapes::Shape *shape, const btTransform &pose)
{
    CollisionWorld::addShapeToObject(ns, shape, pose);

}

bool collision_detection::CollisionWorldFCL::moveShapeInObject(const std::string &id, const shapes::Shape *shape, const btTransform &pose)
{
    if (!CollisionWorld::moveShapeInObject(ns, shape, pose))
        return false;

}

bool collision_detection::CollisionWorldFCL::removeShapeFromObject(const std::string &id, const shapes::Shape *shape)
{
    if (!CollisionWorld::removeShapeFromObject(ns, shape))
        return false;

}

bool collision_detection::CollisionWorldFCL::removeShapeFromObject(const std::string &id, const shapes::StaticShape *shape)
{
    if (!CollisionWorld::removeShapeFromObject(ns, shape))
        return false;

}

/*bool collision_detection::CollisionWorldFCL::removeObject(const std::string &id)
{
    if (!CollisionWorld::removeObjects(id))
        return false;

        }*/

void collision_detection::CollisionWorldFCL::clearObject(const std::string &id)
{
    CollisionWorld::clearObjects(id);

}

void collision_detection::CollisionWorldFCL::clearObjects(void)
{
    CollisionWorld::clearObjects();
}
