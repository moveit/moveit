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

/** \author Ioan Sucan */

#ifndef COLLISION_DETECTION_COLLISION_WORLD_
#define COLLISION_DETECTION_COLLISION_WORLD_

#include "collision_detection/collision_matrix.h"
#include "collision_detection/collision_robot.h"
#include "collision_detection/collision_objects.h"

namespace collision_detection
{

    class CollisionWorld
    {
    public:

        CollisionWorld(void)
        {
        }

        virtual ~CollisionWorld(void)
        {
        }

        /**********************************************************************/
        /* Collision Checking Routines                                        */
        /**********************************************************************/

        /** \brief Check whether the robot model is in collision with itself or the world. Any collisions are considered. */
        void checkCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const planning_models::KinematicState &state) const;

        /** \brief Check whether the robot model is in collision with itself or the world. Allowed collisions are ignored. */
        void checkCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const planning_models::KinematicState &state, const AllowedCollisionMatrix &acm) const;

        /** \brief Check whether the robot model is in collision with the world. Any collisions between a robot link and the world are considered. Self collisions are not checked. */
        virtual void checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const planning_models::KinematicState &state) const = 0;

        /** \brief Check whether the robot model is in collision with the world. Allowed collisions are ignored. Self collisions are not checked. */
        virtual void checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const planning_models::KinematicState &state, const AllowedCollisionMatrix &acm) const = 0;

        /** \brief Check whether a given set of objects is in collision with objects from another world. Any contacts are considered. */
        virtual void checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world) const = 0;

        /** \brief Check whether a given set of objects is in collision with objects from another world. Allowed collisions are ignored. */
        virtual void checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world, const AllowedCollisionMatrix &acm) const = 0;

        /**********************************************************************/
        /* Collision Bodies                                                   */
        /**********************************************************************/

        /** \brief Get the objects currently contained in the model */
        const CollisionObjects& getObjects(void) const
        {
            return objects_;
        }

        /** \brief Tells whether or not there is an object with the given name in the collision model */
        bool haveNamespace(const std::string& ns) const
        {
            return objects_.haveNamespace(ns);
        }

        /** \brief Add a set of collision objects to the map. The user releases ownership of the passed objects. Memory allocated for the shapes is freed by the collision environment.*/
        void addObjects(const std::string &ns, const std::vector<shapes::Shape*> &shapes, const std::vector<btTransform> &poses);

        /** \brief Remove all objects from collision model */
        virtual void clearObjects(void)
        {
            objects_.clearObjects();
        }

        /** \brief Remove objects from a specific namespace in the collision model */
        virtual void clearObjects(const std::string &ns)
        {
            objects_.clearObjects(ns);
        }

        /** \brief Add a static collision object to the map. The user releases ownership of the passed object. Memory allocated for the shape is freed by the collision environment. */
        virtual void addObject(const std::string &ns, shapes::StaticShape *shape)
        {
            objects_.addObject(ns, shape);
        }

        /** \brief Add a collision object to the map. The user releases ownership of the passed object. Memory allocated for the shape is freed by the collision environment.*/
        virtual void addObject(const std::string &ns, shapes::Shape* shape, const btTransform &pose)
        {
            objects_.addObject(ns, shape, pose);
        }

        /** \brief Update the pose of an object. Object equality is verified by comparing pointers. Returns true on success. */
        virtual bool moveObject(const std::string &ns, const shapes::Shape *shape, const btTransform &pose)
        {
            return objects_.moveObject(ns, shape, pose);
        }

    protected:

        CollisionObjects objects_;

    };

    typedef boost::shared_ptr<CollisionWorld> CollisionWorldPtr;
}

#endif
