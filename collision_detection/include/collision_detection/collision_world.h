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

/* Author: Ioan Sucan, Sachin Chitta */

#ifndef COLLISION_DETECTION_COLLISION_WORLD_
#define COLLISION_DETECTION_COLLISION_WORLD_

#include "collision_detection/collision_matrix.h"
#include "collision_detection/collision_robot.h"
#include <boost/thread/mutex.hpp>

namespace collision_detection
{

    /** @brief Perform collision checking with the environment. The
     *  collision world maintains a representation of the environment
     *  that the robot is operating in. */
    class CollisionWorld
    {
    public:

       /** @brief Constructor */
        CollisionWorld(void);

       /** @brief A copy constructor*/
        CollisionWorld(const CollisionWorld &other);

        virtual ~CollisionWorld(void)
        {
        }

        /**********************************************************************/
        /* Collision Checking Routines                                        */
        /**********************************************************************/

        /** @brief Check whether the robot model is in collision with itself or the world.
         *  Any collision between any pair of links is checked for, NO collisions are ignored.
         *  @param req A CollisionRequest object that encapsulates the collision request
         *  @param res A CollisionResult object that encapsulates the collision result
         *  @param state The kinematic state for which checks are being made         */
        void checkCollision(const CollisionRequest &req,
                            CollisionResult &res,
                            const CollisionRobot &robot,
                            const planning_models::KinematicState &state) const;

        /** @brief Check whether the robot model is in collision with itself or the world.
         *  Allowed collisions specified by the allowed collision matrix are taken into account.
         *  @param req A CollisionRequest object that encapsulates the collision request
         *  @param res A CollisionResult object that encapsulates the collision result
         *  @param state The kinematic state for which checks are being made
         *  @param acm The allowed collision matrix. */
        void checkCollision(const CollisionRequest &req,
                            CollisionResult &res,
                            const CollisionRobot &robot,
                            const planning_models::KinematicState &state,
                            const AllowedCollisionMatrix &acm) const;

        /** \brief Check whether the robot model is in collision with the world. Any collisions between a robot link
         *  and the world are considered. Self collisions are not checked.
         *  @param req A CollisionRequest object that encapsulates the collision request
         *  @param res A CollisionResult object that encapsulates the collision result
         *  @robot robot The collision model for the robot
         *  @param state The kinematic state for which checks are being made
         */
        virtual void checkRobotCollision(const CollisionRequest &req,
                                         CollisionResult &res,
                                         const CollisionRobot &robot,
                                         const planning_models::KinematicState &state) const = 0;

        /** \brief Check whether the robot model is in collision with the world. Allowed collisions are ignored.
         *  Self collisions are not checked.
         *  @param req A CollisionRequest object that encapsulates the collision request
         *  @param res A CollisionResult object that encapsulates the collision result
         *  @robot robot The collision model for the robot
         *  @param state The kinematic state for which checks are being made
         *  @param acm The allowed collision matrix.*/
        virtual void checkRobotCollision(const CollisionRequest &req,
                                         CollisionResult &res,
                                         const CollisionRobot &robot,
                                         const planning_models::KinematicState &state,
                                         const AllowedCollisionMatrix &acm) const = 0;

        /** \brief Check whether a given set of objects is in collision with objects from another world.
         *  Any contacts are considered.
         *  @param req A CollisionRequest object that encapsulates the collision request
         *  @param res A CollisionResult object that encapsulates the collision result
         *  @param other_world The other collision world
         */
        virtual void checkWorldCollision(const CollisionRequest &req,
                                         CollisionResult &res,
                                         const CollisionWorld &other_world) const = 0;

        /** \brief Check whether a given set of objects is in collision with objects from another world.
         *  Allowed collisions are ignored. Any contacts are considered.
         *  @param req A CollisionRequest object that encapsulates the collision request
         *  @param res A CollisionResult object that encapsulates the collision result
         *  @param other_world The other collision world
         *  @param acm The allowed collision matrix.*/
        virtual void checkWorldCollision(const CollisionRequest &req,
                                         CollisionResult &res,
                                         const CollisionWorld &other_world,
                                         const AllowedCollisionMatrix &acm) const = 0;

        /**********************************************************************/
        /* Collision Bodies                                                   */
        /**********************************************************************/

        /** @class Object
            @brief A representation of an object */
        struct Object
        {
            Object(const std::string &id);
            virtual ~Object(void);

            /** @brief Clone this object */
            virtual Object* clone(void) const;

            /** \brief The id for this object */
            std::string                         id_;

            /** \brief An array of static shapes */
            std::vector< shapes::StaticShape* > static_shapes_;

            /** \brief An array of shapes */
            std::vector< shapes::Shape* >       shapes_;

            /** \brief An array of shape poses */
            std::vector< Eigen::Affine3f >          shape_poses_;
        };

        typedef boost::shared_ptr<Object> ObjectPtr;
        typedef boost::shared_ptr<Object> ObjectConstPtr;

        /** @class Change
            @brief Contains the change operation to apply (ADD or REMOVE) and the object to apply it on*/
        struct Change
        {
            enum { ADD, REMOVE } type_;
            std::string          id_;
        };

        /** \brief Get the list of Object ids */
        std::vector<std::string> getObjectIds(void) const;

	/** \brief Get the number of objects in this collision world */
	std::size_t getObjectsCount(void) const
	{
	    return objects_.size();
	}
	
        /** \brief Get a particular object */
        ObjectConstPtr getObject(const std::string &id) const;

        /** \brief Check if a particular object exists in the collision world*/
        bool hasObject(const std::string &id) const;

        /** \brief Add shapes to an object in the map. The user releases ownership of the passed shapes. Memory allocated for the shapes is freed by the collision environment.*/
        void addToObject(const std::string &id,
                         const std::vector<shapes::Shape*> &shapes,
                         const std::vector<Eigen::Affine3f> &poses);

        /** \brief Add static shapes to an object in the map. The user releases ownership of the passed shapes. Memory allocated for the shapes is freed by the collision environment.*/
        void addToObject(const std::string &id,
                         const std::vector<shapes::StaticShape*> &shapes);

        /** \brief Add an object. The user releases ownership of the shape. If object already exists, this will add the shape to the object at the specified pose.*/
        virtual void addToObject(const std::string &id, shapes::Shape *shape, const Eigen::Affine3f &pose);

        /** \brief Add an object. The user releases ownership of the shape. If object already exists, this will add the shape to the object at the specified pose.*/
        virtual void addToObject(const std::string &id, shapes::StaticShape *shape);

        /** \brief Update the pose of a shape in an object. Shape equality is verified by comparing pointers. Returns true on success. */
        virtual bool moveShapeInObject(const std::string &id, const shapes::Shape *shape, const Eigen::Affine3f &pose);

        /** \brief Remove shape from object. Shape equality is verified by comparing pointers. Ownership of the object is renounced upon (no memory freed). Returns true on success. */
        virtual bool removeShapeFromObject(const std::string &id, const shapes::Shape *shape);

        /** \brief Remove shape from object. Object equality is verified by comparing pointers. Ownership of the object is renounced upon (no memory freed). Returns true on success. */
        virtual bool removeStaticShapeFromObject(const std::string &id, const shapes::StaticShape *shape);

        /** \brief Remove a particular object. If there are no other pointers to the corresponding instance of Object, the memory is freed. */
        virtual void removeObject(const std::string &id);

        /** \brief Clear all objects. If there are no other pointers to corresponding instances of Objects, the memory is freed. */
        virtual void clearObjects(void);

        /** \brief Set a flag that tells the world representation to record the changes made */
        virtual void recordChanges(bool flag);

        /** \brief Returns true if changes are being recorded */
        bool isRecordingChanges(void) const;

        /** \brief Return all the changes that have been recorded */
        const std::vector<Change>& getChanges(void) const;

        /** \brief Clear the internally maintained vector of changes */
        void clearChanges(void);

    protected:

        /** \brief The objects maintained in the collision world */
        std::map<std::string, ObjectPtr> objects_;

        /** \brief A lock to prevent multiple access to the set of objects */
        mutable boost::mutex             objects_lock_;

        void ensureUnique(ObjectPtr &id);

    private:

        void changeRemoveObj(const std::string &id);
        void changeAddObj(const Object *obj);

        bool                             record_changes_;
        std::vector<Change>              changes_;
    };

    typedef boost::shared_ptr<CollisionWorld> CollisionWorldPtr;
    typedef boost::shared_ptr<const CollisionWorld> CollisionWorldConstPtr;
}

#endif
