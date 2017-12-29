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

#ifndef MOVEIT_COLLISION_DETECTION_COLLISION_ROBOT_
#define MOVEIT_COLLISION_DETECTION_COLLISION_ROBOT_

#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/macros/class_forward.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/LinkPadding.h>
#include <moveit_msgs/LinkScale.h>

namespace collision_detection
{
MOVEIT_CLASS_FORWARD(CollisionRobot);

/** @brief This class represents a collision model of the robot and can be used for self collision checks
    (to check if the robot is in collision with itself) or in collision checks with a different robot. Collision checks
   with
    the environment are performed using the CollisionWorld class. */
class CollisionRobot
{
public:
  /** @brief Constructor
   *  @param model A robot model to construct the collision robot from
   *  @param padding The padding to use for all objects/links on the robot
   *  @scale scale A common scaling to use for all objects/links on the robot
   */
  CollisionRobot(const robot_model::RobotModelConstPtr& model, double padding = 0.0, double scale = 1.0);  // NOLINT

  /**  @brief A copy constructor*/
  CollisionRobot(const CollisionRobot& other);

  virtual ~CollisionRobot()
  {
  }

  /** @brief Check for self collision. Any collision between any pair of links is checked for, NO collisions are
   *   ignored.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state The kinematic state for which checks are being made */
  virtual void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                  const robot_state::RobotState& state) const = 0;

  /** \brief Check for self collision. Allowed collisions specified by the allowed collision matrix are
   *   taken into account.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state The kinematic state for which checks are being made
   *  @param acm The allowed collision matrix. */
  virtual void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                  const robot_state::RobotState& state, const AllowedCollisionMatrix& acm) const = 0;

  /** @brief Check for self collision in a continuous manner. Any collision between any pair of links is checked for,
   *  NO collisions are ignored.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state1 The kinematic state at the start of the segment for which checks are being made
   *  @param state2 The kinematic state at the end of the segment for which checks are being made */
  virtual void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                  const robot_state::RobotState& state1,
                                  const robot_state::RobotState& state2) const = 0;

  /** \brief Check for self collision. Allowed collisions specified by the allowed collision matrix are
   *   taken into account.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state1 The kinematic state at the start of the segment for which checks are being made
   *  @param state2 The kinematic state at the end of the segment for which checks are being made
   *  @param acm The allowed collision matrix. */
  virtual void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                  const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                  const AllowedCollisionMatrix& acm) const = 0;

  /** \brief Check for collision with a different robot (possibly a different kinematic model as well).
   *  Any collision between any pair of links is checked for, NO collisions are ignored.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state The kinematic state for which checks are being made.
   *  @param other_robot The collision representation for the other robot
   *  @param other_state The kinematic state corresponding to the other robot */
  virtual void checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                   const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                   const robot_state::RobotState& other_state) const = 0;

  /** \brief Check for collision with a different robot (possibly a different kinematic model as well).
   *  Allowed collisions specified by the allowed collision matrix are taken into account.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state The kinematic state for which checks are being made.
   *  @param other_robot The collision representation for the other robot
   *  @param other_state The kinematic state corresponding to the other robot
   *  @param acm The allowed collision matrix. */
  virtual void checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                   const robot_state::RobotState& state, const CollisionRobot& other_robot,
                                   const robot_state::RobotState& other_state,
                                   const AllowedCollisionMatrix& acm) const = 0;

  /** \brief Check for collision with a different robot (possibly a different kinematic model as well), in a continuous
   * fashion.
   *  Any collision between any pair of links is checked for, NO collisions are ignored.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state1 The kinematic state at the start of the segment for which checks are being made (this robot)
   *  @param state2 The kinematic state at the end of the segment for which checks are being made (this robot)
   *  @param other_robot The collision representation for the other robot
   *  @param other_state1 The kinematic state at the start of the segment for which checks are being made (other robot)
   *  @param other_state2 The kinematic state at the end of the segment for which checks are being made (other robot) */
  virtual void checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                   const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                   const CollisionRobot& other_robot, const robot_state::RobotState& other_state1,
                                   const robot_state::RobotState& other_state2) const = 0;

  /** \brief Check for collision with a different robot (possibly a different kinematic model as well), in a continuous
   * fashion.
   *  Allowed collisions specified by the allowed collision matrix are taken into account.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state1 The kinematic state at the start of the segment for which checks are being made (this robot)
   *  @param state2 The kinematic state at the end of the segment for which checks are being made (this robot)
   *  @param other_robot The collision representation for the other robot
   *  @param other_state1 The kinematic state at the start of the segment for which checks are being made (other robot)
   *  @param other_state2 The kinematic state at the end of the segment for which checks are being made (other robot)
   *  @param acm The allowed collision matrix. */
  virtual void checkOtherCollision(const CollisionRequest& req, CollisionResult& res,
                                   const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                   const CollisionRobot& other_robot, const robot_state::RobotState& other_state1,
                                   const robot_state::RobotState& other_state2,
                                   const AllowedCollisionMatrix& acm) const = 0;

  /** \brief The distance to self-collision given the robot is at state \e state. */
  virtual double distanceSelf(const robot_state::RobotState& state) const = 0;

  /** \brief The distance to self-collision given the robot is at state \e state, ignoring
      the distances between links that are allowed to always collide (as specified by \e acm) */
  virtual double distanceSelf(const robot_state::RobotState& state, const AllowedCollisionMatrix& acm) const = 0;

  /** \brief The distance to another robot instance.
      @param state The state of this robot to consider
      @param other_robot The other robot instance to measure distance to
      @param other_state The state of the other robot */
  virtual double distanceOther(const robot_state::RobotState& state, const CollisionRobot& other_robot,
                               const robot_state::RobotState& other_state) const = 0;

  /** \brief The distance to another robot instance, ignoring distances between links that are allowed to always
     collide.
      @param state The state of this robot to consider
      @param other_robot The other robot instance to measure distance to
      @param other_state The state of the other robot
      @param acm The collision matrix specifying which links are allowed to always collide */
  virtual double distanceOther(const robot_state::RobotState& state, const CollisionRobot& other_robot,
                               const robot_state::RobotState& other_state, const AllowedCollisionMatrix& acm) const = 0;

  /** @brief The kinematic model corresponding to this collision model*/
  const robot_model::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  /** @brief Set the link padding for a particular link.
   *  @param link_name The link name to set padding for
   *  @param padding The padding to set (in meters)
   */
  void setLinkPadding(const std::string& link_name, double padding);

  /** @brief Get the link padding for a particular link*/
  double getLinkPadding(const std::string& link_name) const;

  /** @brief Set the link paddings using a map (from link names to padding value) */
  void setLinkPadding(const std::map<std::string, double>& padding);

  /** @brief Get the link paddings as a map (from link names to padding value) */
  const std::map<std::string, double>& getLinkPadding() const;

  /** @brief Set the scaling for a particular link*/
  void setLinkScale(const std::string& link_name, double scale);

  /** @brief Set the scaling for a particular link*/
  double getLinkScale(const std::string& link_name) const;

  /** @brief Set the link scaling using a map (from link names to scale value) */
  void setLinkScale(const std::map<std::string, double>& scale);

  /** @brief Get the link scaling as a map (from link names to scale value)*/
  const std::map<std::string, double>& getLinkScale() const;

  /** @brief Set the link padding (for every link)*/
  void setPadding(double padding);

  /** @brief Set the link scaling (for every link)*/
  void setScale(double scale);

  /** @brief Set the link padding from a vector of messages*/
  void setPadding(const std::vector<moveit_msgs::LinkPadding>& padding);

  /** @brief Get the link padding as a vector of messages*/
  void getPadding(std::vector<moveit_msgs::LinkPadding>& padding) const;

  /** @brief Set the link scaling from a vector of messages*/
  void setScale(const std::vector<moveit_msgs::LinkScale>& scale);

  /** @brief Get the link scaling as a vector of messages*/
  void getScale(std::vector<moveit_msgs::LinkScale>& scale) const;

protected:
  /** @brief When the scale or padding is changed for a set of links by any of the functions in this class,
     updatedPaddingOrScaling() function is called.
      This function has an empty default implementation. The intention is to override this function in a derived class
     to allow for updating
      additional structures that may need such updating when link scale or padding changes.
      @param links the names of the links whose padding or scaling were updated */
  virtual void updatedPaddingOrScaling(const std::vector<std::string>& links);

  /** @brief The kinematic model corresponding to this collision model*/
  robot_model::RobotModelConstPtr robot_model_;

  /** @brief The internally maintained map (from link names to padding)*/
  std::map<std::string, double> link_padding_;

  /** @brief The internally maintained map (from link names to scaling)*/
  std::map<std::string, double> link_scale_;
};
}

#endif
