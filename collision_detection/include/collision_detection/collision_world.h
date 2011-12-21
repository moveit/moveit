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

#ifndef COLLISION_DETECTION_COLLISION_WORLD_
#define COLLISION_DETECTION_COLLISION_WORLD_

#include "collision_detection/collision_matrix.h"
#include "collision_detection/collision_robot.h"
#include <boost/thread/recursive_mutex.hpp>

namespace collision_detection
{

    class CollisionWorld
    {
    public:

        CollisionWorld(void);
        CollisionWorld(const CollisionWorld &other);

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

        /** \brief The objects in a particular namespace */
        struct NamespaceObjects
        {
            NamespaceObjects(const std::string &ns);
            virtual ~NamespaceObjects(void);

            virtual NamespaceObjects* clone(void) const;

            /** \brief The namespace for these objects */
            std::string                         ns_;

            /** \brief An array of static shapes */
            std::vector< shapes::StaticShape* > static_shapes_;

            /** \brief An array of shapes */
            std::vector< shapes::Shape* >       shapes_;

            /** \brief An array of shape poses */
            std::vector< btTransform >          shape_poses_;
        };

        typedef boost::shared_ptr<NamespaceObjects> NamespaceObjectsPtr;
        typedef boost::shared_ptr<NamespaceObjects> NamespaceObjectsConstPtr;

        struct Change
        {
            enum { ADD, REMOVE } type_;
            std::string          ns_;
        };

        /** \brief Get the list of namespaces */
        std::vector<std::string> getNamespaces(void) const;

        /** \brief Get the list of objects */
        NamespaceObjectsConstPtr getObjects(const std::string &ns) const;

        /** \brief Check if a particular namespace exists */
        bool haveNamespace(const std::string &ns) const;

        /** \brief Add a set of collision objects to the map. The user releases ownership of the passed objects. Memory allocated for the shapes is freed by the collision environment.*/
        void addObjects(const std::string &ns, const std::vector<shapes::Shape*> &shapes, const std::vector<btTransform> &poses);

	/** \brief Add a set of collision objects to the map. The user releases ownership of the passed objects. Memory allocated for the shapes is freed by the collision environment.*/
        void addObjects(const std::string &ns, const std::vector<shapes::StaticShape*> &shapes);

        /** \brief Add a static object to the namespace. The user releases ownership of the object. */
        virtual void addObject(const std::string &ns, shapes::StaticShape *shape);

        /** \brief Add an object to the namespace. The user releases ownership of the object. */
        virtual void addObject(const std::string &ns, shapes::Shape *shape, const btTransform &pose);

        /** \brief Update the pose of an object. Object equality is verified by comparing pointers. Returns true on success. */
        virtual bool moveObject(const std::string &ns, const shapes::Shape *shape, const btTransform &pose);

        /** \brief Remove object. Object equality is verified by comparing pointers. Ownership of the object is renounced upon (no memory freed). Returns true on success. */
        virtual bool removeObject(const std::string &ns, const shapes::Shape *shape);

        /** \brief Remove object. Object equality is verified by comparing pointers. Ownership of the object is renounced upon (no memory freed). Returns true on success. */
        virtual bool removeObject(const std::string &ns, const shapes::StaticShape *shape);

        /** \brief Clear the objects in a specific namespace. If there are no other pointers to the corresponding instance of NamespaceObjects, the memory is freed. */
        virtual void clearObjects(const std::string &ns);

        /** \brief Clear all objects. If there are no other pointers to corresponding instances of NamespaceObjects, the memory is freed. */
        virtual void clearObjects(void);

        /** \brief Set a flag that tells the world representation to record the changes made */
        virtual void recordChanges(bool flag);

        bool isRecordingChanges(void) const;

        const std::vector<Change>& getChanges(void) const;

        void clearChanges(void);

    protected:

        std::map<std::string, NamespaceObjectsPtr> objects_;
        mutable boost::recursive_mutex             objects_lock_;

        void ensureUnique(NamespaceObjectsPtr &ns);

    private:

        void changeRemoveObj(const std::string &ns);
        void changeAddObj(const NamespaceObjects *obj);

        bool                                       record_changes_;
        std::vector<Change>                        changes_;
    };

    typedef boost::shared_ptr<CollisionWorld> CollisionWorldPtr;
    typedef boost::shared_ptr<const CollisionWorld> CollisionWorldConstPtr;
}

#endif
