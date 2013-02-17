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

#ifndef MOVEIT_COLLISION_DETECTION_COLLISION_WORLD_
#define MOVEIT_COLLISION_DETECTION_COLLISION_WORLD_

#define ACORN_USE_WORLD 1
#define ACORN_USE_WORLD_DEPRICATED 1

#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/collision_detection/collision_robot.h>

#if ACORN_USE_WORLD
#include <moveit/collision_detection/world.h>
#if ACORN_USE_WORLD_DEPRICATED
#include <moveit/collision_detection/world_diff.h>
#endif
#endif

/** \brief Generic interface to collision detection */
namespace collision_detection
{

  class CollisionWorldDeprecated
  {
#if ACORN_USE_WORLD && ACORN_USE_WORLD_DEPRICATED
  public:
    virtual const WorldPtr& getWorld() = 0;
    virtual const WorldConstPtr& getWorld() const = 0;

    CollisionWorldDeprecated() :
      diff_(0)
    {}

    ~CollisionWorldDeprecated()
    {
      if (diff_)
        delete diff_;
    }

    typedef World::ObjectPtr ObjectPtr;
    typedef World::ObjectConstPtr ObjectConstPtr;

    struct Change
    {
      enum ChangeType { ADD, REMOVE } type_;
      std::string          id_;
      Change(int t, const std::string& id) : type_(ChangeType(t)), id_(id) {}
    };

    /** \brief Get the list of Object ids */
    std::vector<std::string> getObjectIds() const
    {
      return getWorld()->getObjectIds();
    }

    /** \brief Get the number of objects in this collision world */
    std::size_t getObjectsCount() const
    {
      return getWorld()->getObjectsCount();
    }

    /** \brief Get a particular object */
    ObjectConstPtr getObject(const std::string &id) const
    {
      return getWorld()->getObject(id);
    }

    /** \brief Check if a particular object exists in the collision world*/
    bool hasObject(const std::string &id) const
    {
      return getWorld()->hasObject(id);
    }

    /** \brief Add shapes to an object in the map. This function makes repeated calls to addToObjectInternal() to add the shapes one by one.
        \note This function does NOT call the addToObject() variant that takes a single shape and a single pose as input. */
    virtual void addToObject(const std::string &id,
                             const std::vector<shapes::ShapeConstPtr> &shapes,
                             const EigenSTL::vector_Affine3d &poses)
    {
      getWorld()->addToObject(id, shapes, poses);
    }

    /** \brief Add a shape to an object. If the object already exists, this call will add the shape to the object at the specified pose. Otherwise, the object is created and the specified shape is added. This calls addToObjectInternal(). */
    virtual void addToObject(const std::string &id, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose)
    {
      getWorld()->addToObject(id, shape, pose);
    }

    /** \brief Update the pose of a shape in an object. Shape equality is verified by comparing pointers. Returns true on success. */
    virtual bool moveShapeInObject(const std::string &id, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose)
    {
      return getWorld()->moveShapeInObject(id, shape, pose);
    }

    /** \brief Remove shape from object. Shape equality is verified by comparing pointers. Ownership of the object is renounced upon (no memory freed). Returns true on success. */
    virtual bool removeShapeFromObject(const std::string &id, const shapes::ShapeConstPtr &shape)
    {
      return getWorld()->removeShapeFromObject(id, shape);
    }

    /** \brief Remove a particular object. If there are no other pointers to the corresponding instance of Object, the memory is freed. */
    virtual void removeObject(const std::string &id)
    {
      getWorld()->removeObject(id);
    }

    /** \brief Clear all objects. If there are no other pointers to corresponding instances of Objects, the memory is freed. */
    virtual void clearObjects()
    {
      getWorld()->clearObjects();
    }

    /** \brief Set a flag that tells the world representation to record the changes made */
    virtual void recordChanges(bool flag)
    {
      if (flag)
      {
        if (!diff_)
          diff_ = new WorldDiff(getWorld());
      }
      else if (diff_)
      {
        delete diff_;
        diff_ = 0;
      }
    }

    /** \brief Returns true if changes are being recorded */
    bool isRecordingChanges() const
    {
      return diff_ != 0;
    }

    /** \brief Return all the changes that have been recorded */
    const std::vector<Change>& getChanges() const
    {
      cp_changes_.clear();
      if (diff_)
      {
        for (WorldDiff::Iterator it=diff_->begin(); it!= diff_->end(); ++it)
        {
          if (it->second == World::DESTROY)
          {
            cp_changes_.push_back(Change(Change::REMOVE, it->first));
          }
          else if ((it->second & World::CREATE) && (it->second & (World::DESTROY|World::MOVE_SHAPE|World::REMOVE_SHAPE)) == 0)
          {
            cp_changes_.push_back(Change(Change::ADD, it->first));
          }
          else
          {
            cp_changes_.push_back(Change(Change::REMOVE, it->first));
            cp_changes_.push_back(Change(Change::ADD, it->first));
          }
        }
      }
      return cp_changes_; 
    }

    /** \brief Remember a change for removing the object named \e id */
    void changeRemoveObject(const std::string &id)
    {
      throw std::exception();
    }

    /** \brief Remember a change for adding the object named \e id */
    void changeAddObject(const std::string &id)
    {
      throw std::exception();
    }

    /** \brief Clear the internally maintained vector of changes */
    void clearChanges()
    {
      cp_changes_.clear();
      if (diff_) diff_->clearChanges();
    }


    virtual void setWorld(WorldPtr world)
    {
      if (diff_)
        diff_->setWorld(world);
    }

  private:
    WorldDiff *diff_;
    mutable std::vector<Change> cp_changes_;
#endif
  };

  /** @brief Perform collision checking with the environment. The
   *  collision world maintains a representation of the environment
   *  that the robot is operating in. */
  class CollisionWorld : public CollisionWorldDeprecated
  {
  public:

    /** @brief Constructor */
    CollisionWorld();

#if ACORN_USE_WORLD
    /** @brief Constructor with world */
    CollisionWorld(WorldPtr world);
#endif

    /** @brief A copy constructor. \e other should not be changed while the copy constructor is running */
    CollisionWorld(const CollisionWorld &other);

    virtual ~CollisionWorld()
    {
    }

    /**********************************************************************/
    /* Collision Checking Routines                                        */
    /**********************************************************************/

    /** @brief Check whether the robot model is in collision with itself or the world at a particular state.
     *  Any collision between any pair of links is checked for, NO collisions are ignored.
     *  @param req A CollisionRequest object that encapsulates the collision request
     *  @param res A CollisionResult object that encapsulates the collision result
     *  @param state The kinematic state for which checks are being made         */
    virtual void checkCollision(const CollisionRequest &req,
                                CollisionResult &res,
                                const CollisionRobot &robot,
                                const robot_state::RobotState &state) const;

    /** @brief Check whether the robot model is in collision with itself or the world at a particular state.
     *  Allowed collisions specified by the allowed collision matrix are taken into account.
     *  @param req A CollisionRequest object that encapsulates the collision request
     *  @param res A CollisionResult object that encapsulates the collision result
     *  @param state The kinematic state for which checks are being made
     *  @param acm The allowed collision matrix. */
    virtual void checkCollision(const CollisionRequest &req,
                                CollisionResult &res,
                                const CollisionRobot &robot,
                                const robot_state::RobotState &state,
                                const AllowedCollisionMatrix &acm) const;

    /** @brief Check whether the robot model is in collision with itself or the world in a continuous manner
     *  (between two robot states)
     *  Any collision between any pair of links is checked for, NO collisions are ignored.
     *  @param req A CollisionRequest object that encapsulates the collision request
     *  @param res A CollisionResult object that encapsulates the collision result
     *  @param state1 The kinematic state at the start of the segment for which checks are being made
     *  @param state2 The kinematic state at the end of the segment for which checks are being made */
    virtual void checkCollision(const CollisionRequest &req,
                                CollisionResult &res,
                                const CollisionRobot &robot,
                                const robot_state::RobotState &state1,
                                const robot_state::RobotState &state2) const;

    /** @brief Check whether the robot model is in collision with itself or the world in a continuous manner
     *  (between two robot states).
     *  Allowed collisions specified by the allowed collision matrix are taken into account.
     *  @param req A CollisionRequest object that encapsulates the collision request
     *  @param res A CollisionResult object that encapsulates the collision result
     *  @param state1 The kinematic state at the start of the segment for which checks are being made
     *  @param state2 The kinematic state at the end of the segment for which checks are being made
     *  @param acm The allowed collision matrix. */
    virtual void checkCollision(const CollisionRequest &req,
                                CollisionResult &res,
                                const CollisionRobot &robot,
                                const robot_state::RobotState &state1,
                                const robot_state::RobotState &state2,
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
                                     const robot_state::RobotState &state) const = 0;

    /** \brief Check whether the robot model is in collision with the world.
     *  Allowed collisions are ignored. Self collisions are not checked.
     *  @param req A CollisionRequest object that encapsulates the collision request
     *  @param res A CollisionResult object that encapsulates the collision result
     *  @robot robot The collision model for the robot
     *  @param state The kinematic state for which checks are being made
     *  @param acm The allowed collision matrix.*/
    virtual void checkRobotCollision(const CollisionRequest &req,
                                     CollisionResult &res,
                                     const CollisionRobot &robot,
                                     const robot_state::RobotState &state,
                                     const AllowedCollisionMatrix &acm) const = 0;

    /** \brief Check whether the robot model is in collision with the world in a continuous manner (between two robot states).
     *  Any collisions between a robot link and the world are considered. Self collisions are not checked.
     *  @param req A CollisionRequest object that encapsulates the collision request
     *  @param res A CollisionResult object that encapsulates the collision result
     *  @robot robot The collision model for the robot
     *  @param state1 The kinematic state at the start of the segment for which checks are being made
     *  @param state2 The kinematic state at the end of the segment for which checks are being made */
    virtual void checkRobotCollision(const CollisionRequest &req,
                                     CollisionResult &res,
                                     const CollisionRobot &robot,
                                     const robot_state::RobotState &state1,
                                     const robot_state::RobotState &state2) const = 0;

    /** \brief Check whether the robot model is in collision with the world in a continuous manner (between two robot states).
     *  Allowed collisions are ignored. Self collisions are not checked.
     *  @param req A CollisionRequest object that encapsulates the collision request
     *  @param res A CollisionResult object that encapsulates the collision result
     *  @robot robot The collision model for the robot
     *  @param state1 The kinematic state at the start of the segment for which checks are being made
     *  @param state2 The kinematic state at the end of the segment for which checks are being made
     *  @param acm The allowed collision matrix.*/
    virtual void checkRobotCollision(const CollisionRequest &req,
                                     CollisionResult &res,
                                     const CollisionRobot &robot,
                                     const robot_state::RobotState &state1,
                                     const robot_state::RobotState &state2,
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

    /** \brief Compute the shortest distance between a robot and the world
     *  @param robot The robot to check distance for
     *  @param state The state for the robot to check distances from */
    virtual double distanceRobot(const CollisionRobot &robot,
                                 const robot_state::RobotState &state) const = 0;

    /** \brief Compute the shortest distance between a robot and the world
     *  @param robot The robot to check distance for
     *  @param state The state for the robot to check distances from
     *  @param acm Using an allowed collision matrix has the effect of ignoring distances from links that are always allowed to be in collision. */
    virtual double distanceRobot(const CollisionRobot &robot,
                                 const robot_state::RobotState &state,
                                 const AllowedCollisionMatrix &acm) const = 0;

    /** \brief The shortest distance to another world instance (\e world) */
    virtual double distanceWorld(const CollisionWorld &world) const = 0;

    /** \brief The shortest distance to another world instance (\e world), ignoring the distances between world elements that are allowed to collide (as specified by \e acm) */
    virtual double distanceWorld(const CollisionWorld &world,
                                 const AllowedCollisionMatrix &acm) const = 0;
#if ACORN_USE_WORLD
    /** set the world to use.
     * This can be expensive unless the new and old world are empty. 
     * Passing NULL will result in a new empty world being created. */
    virtual void setWorld(WorldPtr world);

    /** access the world geometry */
    const WorldPtr& getWorld()
    {
      return world_;
    }

    /** access the world geometry */
    const WorldConstPtr& getWorld() const
    {
      return world_const_;
    }

  private:
    WorldPtr      world_;       // The world.  Always valid.  Never NULL.
    WorldConstPtr world_const_; // always same as world_
#else
    /**********************************************************************/
    /* Collision Bodies                                                   */
    /**********************************************************************/

    /** @class Object
        @brief A representation of an object */
    struct Object
    {
      Object(const std::string &id);

      virtual ~Object();

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** \brief The id for this object */
      std::string                         id_;

      /** \brief An array of shapes */
      std::vector< shapes::ShapeConstPtr> shapes_;

      /** \brief An array of shape poses */
      EigenSTL::vector_Affine3d           shape_poses_;
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
    std::vector<std::string> getObjectIds() const;

    /** \brief Get the number of objects in this collision world */
    std::size_t getObjectsCount() const
    {
      return objects_.size();
    }

    /** \brief Get a particular object */
    ObjectConstPtr getObject(const std::string &id) const;

    /** \brief Check if a particular object exists in the collision world*/
    bool hasObject(const std::string &id) const;

    /** \brief Add shapes to an object in the map. This function makes repeated calls to addToObjectInternal() to add the shapes one by one.
        \note This function does NOT call the addToObject() variant that takes a single shape and a single pose as input. */
    virtual void addToObject(const std::string &id,
                             const std::vector<shapes::ShapeConstPtr> &shapes,
                             const EigenSTL::vector_Affine3d &poses);

    /** \brief Add a shape to an object. If the object already exists, this call will add the shape to the object at the specified pose. Otherwise, the object is created and the specified shape is added. This calls addToObjectInternal(). */
    virtual void addToObject(const std::string &id, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose);

    /** \brief Update the pose of a shape in an object. Shape equality is verified by comparing pointers. Returns true on success. */
    virtual bool moveShapeInObject(const std::string &id, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose);

    /** \brief Remove shape from object. Shape equality is verified by comparing pointers. Ownership of the object is renounced upon (no memory freed). Returns true on success. */
    virtual bool removeShapeFromObject(const std::string &id, const shapes::ShapeConstPtr &shape);

    /** \brief Remove a particular object. If there are no other pointers to the corresponding instance of Object, the memory is freed. */
    virtual void removeObject(const std::string &id);

    /** \brief Clear all objects. If there are no other pointers to corresponding instances of Objects, the memory is freed. */
    virtual void clearObjects();

    /** \brief Set a flag that tells the world representation to record the changes made */
    virtual void recordChanges(bool flag);

    /** \brief Returns true if changes are being recorded */
    bool isRecordingChanges() const;

    /** \brief Return all the changes that have been recorded */
    const std::vector<Change>& getChanges() const;

    /** \brief Remember a change for removing the object named \e id */
    void changeRemoveObject(const std::string &id);

    /** \brief Remember a change for adding the object named \e id */
    void changeAddObject(const std::string &id);

    /** \brief Clear the internally maintained vector of changes */
    void clearChanges();

  protected:

    /** \brief The objects maintained in the collision world */
    std::map<std::string, ObjectPtr> objects_;

    /** \brief Make sure that the object named \e id is known only to this instance of the CollisionWorld. If the object is known outside of it, a clone is made so that it can be safely modified later on. */
    void ensureUnique(ObjectPtr &id);

    /** \brief Add a shape to a specified object. All the sanity checks are done at the call site; this function should be efficient and not perform work that can be done only once (e.g., in the addToObject() call) */
    virtual void addToObjectInternal(const ObjectPtr &obj, const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose);

    bool                             record_changes_;
    std::vector<Change>              changes_;
#endif
  };

  typedef boost::shared_ptr<CollisionWorld> CollisionWorldPtr;
  typedef boost::shared_ptr<const CollisionWorld> CollisionWorldConstPtr;
}

#endif
