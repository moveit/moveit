/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Adam Leeper */

#ifndef MOVEIT__ROBOT_INTERACTION__INTERACTION_HANDLER_H
#define MOVEIT__ROBOT_INTERACTION__INTERACTION_HANDLER_H

#include <moveit/robot_interaction/robot_interaction.h>

namespace robot_interaction
{

/// Manage interactive markers to control a RobotState.
///
/// Each instance maintains one or more interactive markers to control
/// various joints in one group of one RobotState.
/// The group being controlled is maintained by the RobotInteraction object
/// that contains this InteractionHandler object.
/// All InteractionHandler objects in the same RobotInteraction are controlling the same group.
class RobotInteraction::InteractionHandler
{
public:

  InteractionHandler(const std::string &name,
                     const robot_state::RobotState &kstate,
                     const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>());
  InteractionHandler(const std::string &name,
                     const robot_model::RobotModelConstPtr &kmodel,
                     const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>());

  virtual ~InteractionHandler()
  {
  }

  const std::string& getName() const
  {
    return name_;
  }

  robot_state::RobotStateConstPtr getState() const;
  void setState(const robot_state::RobotState& kstate);

  void setUpdateCallback(const InteractionHandlerCallbackFn &callback)
  {
    update_callback_ = callback;
  }

  const InteractionHandlerCallbackFn& getUpdateCallback() const
  {
    return update_callback_;
  }

  void setGroupStateValidityCallback(const robot_state::GroupStateValidityCallbackFn &callback)
  {
    state_validity_callback_fn_ = callback;
  }

  const robot_state::GroupStateValidityCallbackFn& getGroupStateValidityCallback() const
  {
    return state_validity_callback_fn_;
  }

  void setIKTimeout(double timeout)
  {
    ik_timeout_ = timeout;
  }

  double getIKTimeout() const
  {
    return ik_timeout_;
  }

  void setIKAttempts(unsigned int attempts)
  {
    ik_attempts_ = attempts;
  }

  unsigned int getIKAttempts() const
  {
    return ik_attempts_;
  }

  const kinematics::KinematicsQueryOptions& getKinematicsQueryOptions() const
  {
    return kinematics_query_options_;
  }

  void setKinematicsQueryOptions(const kinematics::KinematicsQueryOptions &opt)
  {
    kinematics_query_options_ = opt;
  }

  void setKinematicsQueryOptionsForGroup(const std::string& group_name, 
           kinematics::KinematicsQueryOptions &options)
  {
    kinematics_query_options_map_[group_name] = options;
  }

  const bool getKinematicsQueryOptionsForGroup(const std::string& group_name, 
           kinematics::KinematicsQueryOptions &opt)
  {
    std::map<std::string, kinematics::KinematicsQueryOptions>::const_iterator it = kinematics_query_options_map_.find(group_name);
    if (it == kinematics_query_options_map_.end())
return false;
    opt = it->second;
    return true;
  }

  void setMeshesVisible(bool visible)
  {
    display_meshes_ = visible;
  }

  bool getMeshesVisible() const
  {
    return display_meshes_;
  }

  void setControlsVisible(bool visible)
  {
    display_controls_ = visible;
  }

  bool getControlsVisible() const
  {
    return display_controls_;
  }

  /** \brief Set the offset for drawing the interactive marker controls for an end-effector,
   *         expressed in the frame of the end-effector parent.
   * @param  The target end-effector.
   * @param  The desired pose offset. */
  void setPoseOffset(const RobotInteraction::EndEffector& eef, const geometry_msgs::Pose& m);

  /** \brief Set the offset for drawing the interactive marker controls for a joint,
   *         expressed in the frame of the joint parent.
   * @param  The target joint.
   * @param  The desired pose offset. */
  void setPoseOffset(const RobotInteraction::Joint& eef, const geometry_msgs::Pose& m);

  /** \brief Get the offset for the interactive marker controls for an end-effector,
   *         expressed in the frame of the end-effector parent.
   * @param  The target end-effector.
   * @param  The pose offset (only valid if return value is true).
   * @return True if an offset was found for the given end-effector, false otherwise. */
  bool getPoseOffset(const RobotInteraction::EndEffector& eef, geometry_msgs::Pose& m);

  /** \brief Get the offset for the interactive marker controls for a joint,
   *         expressed in the frame of the joint parent.
   * @param  The target joint.
   * @param  The pose offset (only valid if return value is true).
   * @return True if an offset was found for the given joint, false otherwise. */
  bool getPoseOffset(const RobotInteraction::Joint& vj, geometry_msgs::Pose& m);

  /** \brief Clear the interactive marker pose offset for the given end-effector.
   * @param  The target end-effector. */
  void clearPoseOffset(const RobotInteraction::EndEffector& eef);

  /** \brief Clear the interactive marker pose offset for the given joint.
   * @param  The target joint. */
  void clearPoseOffset(const RobotInteraction::Joint& vj);

  /** \brief Clear the pose offset for all end-effectors and virtual joints. */
  void clearPoseOffsets();

  /** \brief Set the menu handler that defines menus and callbacks for all
   *         interactive markers drawn by this interaction handler.
   * @param  A menu handler. */
  void setMenuHandler(const boost::shared_ptr<interactive_markers::MenuHandler>& mh);


  /** \brief Get the menu handler that defines menus and callbacks for all
   *         interactive markers drawn by this interaction handler.
   * @return  The menu handler. */
  const boost::shared_ptr<interactive_markers::MenuHandler>& getMenuHandler();

  /** \brief Remove the menu handler for this interaction handler. */
  void clearMenuHandler();

  /** \brief Get the last interactive_marker command pose for an end-effector.
   * @param The end-effector in question.
   * @param A PoseStamped message containing the last (offset-removed) pose commanded for the end-effector.
   * @return True if a pose for that end-effector was found, false otherwise. */
  bool getLastEndEffectorMarkerPose(const RobotInteraction::EndEffector& eef, geometry_msgs::PoseStamped& pose);

  /** \brief Get the last interactive_marker command pose for a joint.
   * @param The joint in question.
   * @param A PoseStamped message containing the last (offset-removed) pose commanded for the joint.
   * @return True if a pose for that joint was found, false otherwise. */
  bool getLastJointMarkerPose(const RobotInteraction::Joint& vj, geometry_msgs::PoseStamped& pose);

  /** \brief Clear the last interactive_marker command pose for the given end-effector.
   * @param  The target end-effector. */
  void clearLastEndEffectorMarkerPose(const RobotInteraction::EndEffector& eef);

  /** \brief Clear the last interactive_marker command pose for the given joint.
   * @param  The target joint. */
  void clearLastJointMarkerPose(const RobotInteraction::Joint& vj);

  /** \brief Clear the last interactive_marker command poses for all end-effectors and joints. */
  void clearLastMarkerPoses();

  /** \brief Update the internal state maintained by the handler using
   * information from the received feedback message. */
  virtual void handleEndEffector(const RobotInteraction::EndEffector &eef,
                                 const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /** \brief Update the internal state maintained by the handler using
   * information from the received feedback message. */
  virtual void handleJoint(const RobotInteraction::Joint &vj,
                           const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /** \brief Update the internal state maintained by the handler using
   * information from the received feedback message. */
  virtual void handleGeneric(const RobotInteraction::Generic &g,
                             const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /** \brief Check if the marker corresponding to this end-effector leads to an invalid state */
  virtual bool inError(const RobotInteraction::EndEffector& eef) const;

  /** \brief Check if the marker corresponding to this joint leads to an invalid state */
  virtual bool inError(const RobotInteraction::Joint& vj) const;

  /** \brief Check if the generic marker to an invalid state */
  virtual bool inError(const RobotInteraction::Generic& g) const;

  /** \brief Clear any error settings. This makes the markers appear as if the state is no longer invalid. */
  void clearError(void);

protected:

  bool transformFeedbackPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
                             const geometry_msgs::Pose &offset,
                             geometry_msgs::PoseStamped &tpose);

  robot_state::RobotStatePtr getUniqueStateAccess();
  void setStateToAccess(robot_state::RobotStatePtr &state);

  std::string name_;
  std::string planning_frame_;
  boost::shared_ptr<tf::Transformer> tf_;
  std::set<std::string> error_state_;

  // For adding menus (and associated callbacks) to all the
  // end-effector and virtual-joint interactive markers
  boost::shared_ptr<interactive_markers::MenuHandler> menu_handler_;

  // Called when the RobotState maintained by the handler changes.  The caller may, for example, redraw the robot at the new state.
  // handler is the handler that changed.
  // error_state_changed is true if an end effector's error state may have changed.
  boost::function<void(InteractionHandler* handler, bool error_state_changed)> update_callback_;

  robot_state::GroupStateValidityCallbackFn state_validity_callback_fn_;
  double ik_timeout_;
  unsigned int ik_attempts_;

  // additional options for kinematics queries
  kinematics::KinematicsQueryOptions kinematics_query_options_;

  bool display_meshes_;
  bool display_controls_;

private:
// The state maintained by this handler.
// PROTECTED BY state_lock_
  robot_state::RobotStatePtr kstate_;

  // Contains the (user-programmable) pose offset between the end-effector
  // parent link (or a virtual joint) and the desired control frame for the
  // interactive marker. The offset is expressed in the frame of the parent
  // link or virtual joint. For example, on a PR2 an offset of +0.20 along
  // the x-axis will move the center of the 6-DOF interactive marker from
  // the wrist to the finger tips.
  // PROTECTED BY offset_map_lock_
  std::map<std::string, geometry_msgs::Pose> offset_map_;

  // Contains the most recent poses received from interactive marker feedback,
  // with the offset removed (e.g. in theory, coinciding with the end-effector
  // parent or virtual joint). This allows a user application to query for the
  // interactive marker pose (which could be useful for robot control using
  // gradient-based methods) even when the IK solver failed to find a valid
  // robot state that satisfies the feedback pose.
  // PROTECTED BY pose_map_lock_
  std::map<std::string, geometry_msgs::PoseStamped> pose_map_;


  std::map<std::string, kinematics::KinematicsQueryOptions> kinematics_query_options_map_;
  mutable boost::mutex state_lock_;
  mutable boost::condition_variable state_available_condition_;
  boost::mutex pose_map_lock_;
  boost::mutex offset_map_lock_;

  void setup();
};

}

#endif
