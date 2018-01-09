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

/* Author: Ioan Sucan, Sachin Chitta */

#ifndef MOVEIT_COLLISION_DETECTION_COLLISION_WORLD_
#define MOVEIT_COLLISION_DETECTION_COLLISION_WORLD_

#include <boost/utility.hpp>

#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection/world.h>
#include <moveit/macros/class_forward.h>

/** \brief Generic interface to collision detection */
namespace collision_detection
{
MOVEIT_CLASS_FORWARD(CollisionWorld);

/** \brief Perform collision checking with the environment. The
 *  collision world maintains a representation of the environment
 *  that the robot is operating in. */
class CollisionWorld : private boost::noncopyable
{
public:
  CollisionWorld();

  explicit CollisionWorld(const WorldPtr& world);

  /** \brief A copy constructor. \e other should not be changed while the copy constructor is running.
   * world must be the same world as used by other or a (not-yet-modified) copy of the world used by other */
  CollisionWorld(const CollisionWorld& other, const WorldPtr& world);

  virtual ~CollisionWorld()
  {
  }

  /**********************************************************************/
  /* Collision Checking Routines                                        */
  /**********************************************************************/

  /** \brief Check whether the robot model is in collision with itself or the world at a particular state.
   *  Any collision between any pair of links is checked for, NO collisions are ignored.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state The kinematic state for which checks are being made         */
  virtual void checkCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                              const robot_state::RobotState& state) const;

  /** \brief Check whether the robot model is in collision with itself or the world at a particular state.
   *  Allowed collisions specified by the allowed collision matrix are taken into account.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state The kinematic state for which checks are being made
   *  @param acm The allowed collision matrix. */
  virtual void checkCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                              const robot_state::RobotState& state, const AllowedCollisionMatrix& acm) const;

  /** \brief Check whether the robot model is in collision with itself or the world in a continuous manner
   *  (between two robot states)
   *  Any collision between any pair of links is checked for, NO collisions are ignored.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state1 The kinematic state at the start of the segment for which checks are being made
   *  @param state2 The kinematic state at the end of the segment for which checks are being made */
  virtual void checkCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                              const robot_state::RobotState& state1, const robot_state::RobotState& state2) const;

  /** \brief Check whether the robot model is in collision with itself or the world in a continuous manner
   *  (between two robot states).
   *  Allowed collisions specified by the allowed collision matrix are taken into account.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state1 The kinematic state at the start of the segment for which checks are being made
   *  @param state2 The kinematic state at the end of the segment for which checks are being made
   *  @param acm The allowed collision matrix. */
  virtual void checkCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                              const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                              const AllowedCollisionMatrix& acm) const;

  /** \brief Check whether the robot model is in collision with the world. Any collisions between a robot link
   *  and the world are considered. Self collisions are not checked.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @robot robot The collision model for the robot
   *  @param state The kinematic state for which checks are being made
   */
  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                   const robot_state::RobotState& state) const = 0;

  /** \brief Check whether the robot model is in collision with the world.
   *  Allowed collisions are ignored. Self collisions are not checked.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @robot robot The collision model for the robot
   *  @param state The kinematic state for which checks are being made
   *  @param acm The allowed collision matrix.*/
  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                   const robot_state::RobotState& state, const AllowedCollisionMatrix& acm) const = 0;

  /** \brief Check whether the robot model is in collision with the world in a continuous manner (between two robot
   * states).
   *  Any collisions between a robot link and the world are considered. Self collisions are not checked.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @robot robot The collision model for the robot
   *  @param state1 The kinematic state at the start of the segment for which checks are being made
   *  @param state2 The kinematic state at the end of the segment for which checks are being made */
  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                   const robot_state::RobotState& state1,
                                   const robot_state::RobotState& state2) const = 0;

  /** \brief Check whether the robot model is in collision with the world in a continuous manner (between two robot
   * states).
   *  Allowed collisions are ignored. Self collisions are not checked.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @robot robot The collision model for the robot
   *  @param state1 The kinematic state at the start of the segment for which checks are being made
   *  @param state2 The kinematic state at the end of the segment for which checks are being made
   *  @param acm The allowed collision matrix.*/
  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                   const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                   const AllowedCollisionMatrix& acm) const = 0;

  /** \brief Check whether a given set of objects is in collision with objects from another world.
   *  Any contacts are considered.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param other_world The other collision world
   */
  virtual void checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                   const CollisionWorld& other_world) const = 0;

  /** \brief Check whether a given set of objects is in collision with objects from another world.
   *  Allowed collisions are ignored. Any contacts are considered.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param other_world The other collision world
   *  @param acm The allowed collision matrix.*/
  virtual void checkWorldCollision(const CollisionRequest& req, CollisionResult& res, const CollisionWorld& other_world,
                                   const AllowedCollisionMatrix& acm) const = 0;

  /** \brief Compute the shortest distance between a robot and the world
   *  @param robot The robot to check distance for
   *  @param state The state for the robot to check distances from
   *  @param verbose Output debug information about distance checks */
  inline double distanceRobot(const CollisionRobot& robot, const robot_state::RobotState& state,
                              bool verbose = false) const
  {
    DistanceRequest req;
    DistanceResult res;

    req.verbose = verbose;
    req.enableGroup(robot.getRobotModel());

    distanceRobot(req, res, robot, state);
    return res.minimum_distance.distance;
  }

  /** \brief Compute the shortest distance between a robot and the world
   *  @param robot The robot to check distance for
   *  @param state The state for the robot to check distances from
   *  @param acm Using an allowed collision matrix has the effect of ignoring distances from links that are always
   * allowed to be in collision.
   *  @param verbose Output debug information about distance checks */
  inline double distanceRobot(const CollisionRobot& robot, const robot_state::RobotState& state,
                              const AllowedCollisionMatrix& acm, bool verbose = false) const
  {
    DistanceRequest req;
    DistanceResult res;

    req.acm = &acm;
    req.verbose = verbose;
    req.enableGroup(robot.getRobotModel());

    distanceRobot(req, res, robot, state);
    return res.minimum_distance.distance;
  }

  /** \brief Compute the distance between a robot and the world
   *  @param req A DistanceRequest object that encapsulates the distance request
   *  @param res A DistanceResult object that encapsulates the distance result
   *  @param robot The robot to check distance for
   *  @param state The state for the robot to check distances from */
  virtual void distanceRobot(const DistanceRequest& req, DistanceResult& res, const CollisionRobot& robot,
                             const robot_state::RobotState& state) const = 0;

  /** \brief The shortest distance to another world instance (\e world)
   *  @param verbose Output debug information about distance checks */
  inline double distanceWorld(const CollisionWorld& world, bool verbose = false) const
  {
    DistanceRequest req;
    DistanceResult res;

    req.verbose = verbose;
    distanceWorld(req, res, world);

    return res.minimum_distance.distance;
  }

  /** \brief The shortest distance to another world instance (\e world), ignoring the distances between world elements
   * that are allowed to collide (as specified by \e acm)
   *  @param verbose Output debug information about distance checks */
  inline double distanceWorld(const CollisionWorld& world, const AllowedCollisionMatrix& acm,
                              bool verbose = false) const
  {
    DistanceRequest req;
    DistanceResult res;

    req.acm = &acm;
    req.verbose = verbose;
    distanceWorld(req, res, world);

    return res.minimum_distance.distance;
  }

  /** \brief Compute the distance between another world
   *  @param req A DistanceRequest object that encapsulates the distance request
   *  @param res A DistanceResult object that encapsulates the distance result
   *  @param world The world to check distance for */
  virtual void distanceWorld(const DistanceRequest& req, DistanceResult& res, const CollisionWorld& world) const = 0;

  /** set the world to use.
   * This can be expensive unless the new and old world are empty.
   * Passing NULL will result in a new empty world being created. */
  virtual void setWorld(const WorldPtr& world);

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

  typedef World::ObjectPtr ObjectPtr;
  typedef World::ObjectConstPtr ObjectConstPtr;

private:
  WorldPtr world_;             // The world.  Always valid.  Never NULL.
  WorldConstPtr world_const_;  // always same as world_
};
}

#endif
