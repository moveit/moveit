/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2013, Willow Garage, Inc.
 *  Copyright (c) 2013, Ioan A. Sucan
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

#include <moveit/robot_interaction/interaction_handler.h>
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>
#include <moveit/robot_interaction/kinematic_options_map.h>
#include <moveit/transforms/transforms.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>
#include <algorithm>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace robot_interaction
{

InteractionHandler::InteractionHandler(
      const RobotInteractionPtr& robot_interaction,
      const std::string &name,
      const robot_state::RobotState &initial_robot_state,
      const boost::shared_ptr<tf::Transformer> &tf)
:LockedRobotState(initial_robot_state)
,name_(fixName(name))
,planning_frame_(initial_robot_state.getRobotModel()->getModelFrame())
,tf_(tf)
,robot_interaction_(NULL)
,kinematic_options_map_(robot_interaction->getKinematicOptionsMap())
,display_meshes_(true)
,display_controls_(true)
{
  setRobotInteraction(robot_interaction.get());
}

InteractionHandler::InteractionHandler(
      const RobotInteractionPtr& robot_interaction,
      const std::string &name,
      const boost::shared_ptr<tf::Transformer> &tf)
:LockedRobotState(robot_interaction->getRobotModel())
,name_(fixName(name))
,planning_frame_(robot_interaction->getRobotModel()->getModelFrame())
,tf_(tf)
,robot_interaction_(NULL)
,kinematic_options_map_(robot_interaction->getKinematicOptionsMap())
,display_meshes_(true)
,display_controls_(true)
{
  setRobotInteraction(robot_interaction.get());
}

// DEPRECATED
InteractionHandler::InteractionHandler(
      const std::string &name,
      const robot_state::RobotState &initial_robot_state,
      const boost::shared_ptr<tf::Transformer> &tf)
:LockedRobotState(initial_robot_state)
,name_(fixName(name))
,planning_frame_(initial_robot_state.getRobotModel()->getModelFrame())
,tf_(tf)
,robot_interaction_(NULL)
,kinematic_options_map_(new KinematicOptionsMap)
,display_meshes_(true)
,display_controls_(true)
{
}

// DEPRECATED
InteractionHandler::InteractionHandler(
      const std::string &name,
      const robot_model::RobotModelConstPtr &robot_model,
      const boost::shared_ptr<tf::Transformer> &tf)
:LockedRobotState(robot_model)
,name_(fixName(name))
,planning_frame_(robot_model->getModelFrame())
,tf_(tf)
,robot_interaction_(NULL)
,kinematic_options_map_(new KinematicOptionsMap)
,display_meshes_(true)
,display_controls_(true)
{
}

std::string InteractionHandler::fixName(std::string name)
{
  std::replace(name.begin(), name.end(), '_', '-'); // we use _ as a special char in marker name
  return name;
}

void InteractionHandler::setPoseOffset(const EndEffectorInteraction& eef, const geometry_msgs::Pose& m)
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  offset_map_[eef.eef_group] = m;
}

void InteractionHandler::setPoseOffset(const JointInteraction& vj, const geometry_msgs::Pose& m)
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  offset_map_[vj.joint_name] = m;
}

void InteractionHandler::clearPoseOffset(const EndEffectorInteraction& eef)
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  offset_map_.erase(eef.eef_group);
}

void InteractionHandler::clearPoseOffset(const JointInteraction& vj)
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  offset_map_.erase(vj.joint_name);
}

void InteractionHandler::clearPoseOffsets()
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  offset_map_.clear();
}

bool InteractionHandler::getPoseOffset(const EndEffectorInteraction& eef, geometry_msgs::Pose& m)
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  std::map<std::string, geometry_msgs::Pose>::iterator it = offset_map_.find(eef.eef_group);
  if (it != offset_map_.end())
  {
    m = it->second;
    return true;
  }
  return false;
}

bool InteractionHandler::getPoseOffset(const JointInteraction& vj, geometry_msgs::Pose& m)
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  std::map<std::string, geometry_msgs::Pose>::iterator it = offset_map_.find(vj.joint_name);
  if (it != offset_map_.end())
  {
    m = it->second;
    return true;
  }
  return false;
}

bool InteractionHandler::getLastEndEffectorMarkerPose(const EndEffectorInteraction& eef, geometry_msgs::PoseStamped& ps)
{
  boost::mutex::scoped_lock slock(pose_map_lock_);
  std::map<std::string, geometry_msgs::PoseStamped>::iterator it = pose_map_.find(eef.eef_group);
  if (it != pose_map_.end())
  {
    ps = it->second;
    return true;
  }
  return false;
}

bool InteractionHandler::getLastJointMarkerPose(const JointInteraction& vj, geometry_msgs::PoseStamped& ps)
{
  boost::mutex::scoped_lock slock(pose_map_lock_);
  std::map<std::string, geometry_msgs::PoseStamped>::iterator it = pose_map_.find(vj.joint_name);
  if (it != pose_map_.end())
  {
    ps = it->second;
    return true;
  }
  return false;
}

void InteractionHandler::clearLastEndEffectorMarkerPose(const EndEffectorInteraction& eef)
{
  boost::mutex::scoped_lock slock(pose_map_lock_);
  pose_map_.erase(eef.eef_group);
}

void InteractionHandler::clearLastJointMarkerPose(const JointInteraction& vj)
{
  boost::mutex::scoped_lock slock(pose_map_lock_);
  pose_map_.erase(vj.joint_name);
}

void InteractionHandler::clearLastMarkerPoses()
{
  boost::mutex::scoped_lock slock(pose_map_lock_);
  pose_map_.clear();
}

void InteractionHandler::setMenuHandler(const boost::shared_ptr<interactive_markers::MenuHandler>& mh)
{
  boost::mutex::scoped_lock lock(state_lock_);
  menu_handler_ = mh;
}

const boost::shared_ptr<interactive_markers::MenuHandler>& InteractionHandler::getMenuHandler()
{
  boost::mutex::scoped_lock lock(state_lock_);
  return menu_handler_;
}

void InteractionHandler::clearMenuHandler()
{
  boost::mutex::scoped_lock lock(state_lock_);
  menu_handler_.reset();
}


void InteractionHandler::handleGeneric(
      const GenericInteraction &g,
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (g.process_feedback)
  {
    StateChangeCallbackFn callback;
    // modify the RobotState in-place with the state_lock_ held.
    LockedRobotState::modifyState(boost::bind(&InteractionHandler::updateStateGeneric,
                                              this,
                                              _1,
                                              &g,
                                              &feedback,
                                              &callback));

    // This calls update_callback_ to notify client that state changed.
    if (callback)
      callback(this);
  }
}

void InteractionHandler::handleEndEffector(
      const EndEffectorInteraction &eef,
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    return;

  geometry_msgs::PoseStamped tpose;
  geometry_msgs::Pose offset;
  if (!getPoseOffset(eef, offset))
    offset.orientation.w = 1;
  if (transformFeedbackPose(feedback, offset, tpose))
  {
    pose_map_lock_.lock();
    pose_map_[eef.eef_group] = tpose;
    pose_map_lock_.unlock();
  }
  else
    return;

  StateChangeCallbackFn callback;

  // modify the RobotState in-place with state_lock_ held.
  // This locks state_lock_ before calling updateState()
  LockedRobotState::modifyState(boost::bind(&InteractionHandler::updateStateEndEffector,
                                            this,
                                            _1,
                                            &eef,
                                            &tpose.pose,
                                            &callback));

  // This calls update_callback_ to notify client that state changed.
  if (callback)
    callback(this);
}

void InteractionHandler::handleJoint(
      const JointInteraction &vj,
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    return;

  geometry_msgs::PoseStamped tpose;
  geometry_msgs::Pose offset;
  if (!getPoseOffset(vj, offset))
    offset.orientation.w = 1;
  if (transformFeedbackPose(feedback, offset, tpose))
  {
    pose_map_lock_.lock();
    pose_map_[vj.joint_name] = tpose;
    pose_map_lock_.unlock();
  }
  else
    return;

  StateChangeCallbackFn callback;

  // modify the RobotState in-place with state_lock_ held.
  // This locks state_lock_ before calling updateState()
  LockedRobotState::modifyState(boost::bind(&InteractionHandler::updateStateJoint,
                                            this,
                                            _1,
                                            &vj,
                                            &tpose.pose,
                                            &callback));

  // This calls update_callback_ to notify client that state changed.
  if (callback)
    callback(this);
}

// MUST hold state_lock_ when calling this!
void InteractionHandler::updateStateGeneric(
      robot_state::RobotState* state,
      const GenericInteraction *g,
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr *feedback,
      StateChangeCallbackFn *callback)
{
  bool ok = g->process_feedback(*state, *feedback);
  bool error_state_changed = setErrorState(g->marker_name_suffix, !ok);
  if (update_callback_)
    *callback = boost::bind(update_callback_,
                            _1,
                            error_state_changed);
}

// MUST hold state_lock_ when calling this!
void InteractionHandler::updateStateEndEffector(
      robot_state::RobotState* state,
      const EndEffectorInteraction* eef,
      const geometry_msgs::Pose* pose,
      StateChangeCallbackFn *callback)
{
  // This is called with state_lock_ held, so no additional locking needed to
  // access kinematic_options_map_.
  KinematicOptions kinematic_options = kinematic_options_map_->getOptions(eef->parent_group);

  bool ok = kinematic_options.setStateFromIK(*state,
                                             eef->parent_group,
                                             eef->parent_link,
                                             *pose);
  bool error_state_changed = setErrorState(eef->parent_group, !ok);
  if (update_callback_)
    *callback = boost::bind(update_callback_,
                            _1,
                            error_state_changed);
}

// MUST hold state_lock_ when calling this!
void InteractionHandler::updateStateJoint(
      robot_state::RobotState* state,
      const JointInteraction* vj,
      const geometry_msgs::Pose* feedback_pose,
      StateChangeCallbackFn *callback)
{
  geometry_msgs::Pose rel_pose = *feedback_pose;
  if (!vj->parent_frame.empty() && !robot_state::Transforms::sameFrame(vj->parent_frame, planning_frame_))
  {
    Eigen::Affine3d p;
    tf::poseMsgToEigen(rel_pose, p);
    tf::poseEigenToMsg(state->getGlobalLinkTransform(vj->parent_frame).inverse() * p, rel_pose);
  }

  Eigen::Quaterniond q;
  tf::quaternionMsgToEigen(rel_pose.orientation, q);
  std::map<std::string, double> vals;
  if (vj->dof == 3)
  {
    vals[vj->joint_name + "/x"] = rel_pose.position.x;
    vals[vj->joint_name + "/y"] = rel_pose.position.y;
    Eigen::Vector3d xyz = q.matrix().eulerAngles(0, 1, 2);
    vals[vj->joint_name + "/theta"] = xyz[2];
  }
  else
    if (vj->dof == 6)
    {
      vals[vj->joint_name + "/trans_x"] = rel_pose.position.x;
      vals[vj->joint_name + "/trans_y"] = rel_pose.position.y;
      vals[vj->joint_name + "/trans_z"] = rel_pose.position.z;
      vals[vj->joint_name + "/rot_x"] = q.x();
      vals[vj->joint_name + "/rot_y"] = q.y();
      vals[vj->joint_name + "/rot_z"] = q.z();
      vals[vj->joint_name + "/rot_w"] = q.w();
    }
  state->setVariablePositions(vals);
  state->update();

  if (update_callback_)
    *callback = boost::bind(update_callback_,
                            _1,
                            false);
}

bool InteractionHandler::inError(const EndEffectorInteraction& eef) const
{
  return getErrorState(eef.parent_group);
}

bool InteractionHandler::inError(const GenericInteraction& g) const
{
  return getErrorState(g.marker_name_suffix);
}

bool InteractionHandler::inError(const JointInteraction& vj) const
{
  return false;
}

void InteractionHandler::clearError(void)
{
  boost::mutex::scoped_lock lock(state_lock_);
  error_state_.clear();
}

// return true if the state changed.
// MUST hold state_lock_ when calling this!
bool InteractionHandler::setErrorState(const std::string& name, bool new_error_state)
{
  bool old_error_state = error_state_.find(name) != error_state_.end();

  if (new_error_state == old_error_state)
    return false;

  if (new_error_state)
    error_state_.insert(name);
  else
    error_state_.erase(name);

  return true;
}

bool InteractionHandler::getErrorState(const std::string& name) const
{
  boost::mutex::scoped_lock lock(state_lock_);
  return error_state_.find(name) != error_state_.end();

}

bool InteractionHandler::transformFeedbackPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
                                                                 const geometry_msgs::Pose &offset,
                                                                 geometry_msgs::PoseStamped &tpose)
{
  tpose.header = feedback->header;
  tpose.pose = feedback->pose;
  if (feedback->header.frame_id != planning_frame_)
  {
    if (tf_)
      try
      {
        tf::Stamped<tf::Pose> spose;
        tf::poseStampedMsgToTF(tpose, spose);
        // Express feedback (marker) pose in planning frame
        tf_->transformPose(planning_frame_, spose, spose);
        // Apply inverse of offset to bring feedback pose back into the end-effector support link frame
        tf::Transform tf_offset;
        tf::poseMsgToTF(offset, tf_offset);
        spose.setData(spose * tf_offset.inverse());
        tf::poseStampedTFToMsg(spose, tpose);
      }
      catch (tf::TransformException& e)
      {
        ROS_ERROR("Error transforming from frame '%s' to frame '%s'",
          tpose.header.frame_id.c_str(),
          planning_frame_.c_str());
        return false;
      }
    else
    {
      ROS_ERROR("Cannot transform from frame '%s' to frame '%s' (no TF instance provided)",
          tpose.header.frame_id.c_str(),
          planning_frame_.c_str());
      return false;
    }
  }
  return true;
}

// This syncs the InteractionHandler's KinematicOptionsMap with the
// RobotInteraction's.  After this both will share the same
// KinematicOptionsMap.
//
// With the constructors that take a RobotInteraction parameter this function
// is not needed (except as a sanity check that the RobotInteraction and
// InteractionHandler are matched).  This function is necessary because when
// the old constructors are used the InteractionHandler may not know what
// RobotInteraction it is associated with until after some options have been
// set on the InteractionHandler.
void InteractionHandler::setRobotInteraction(RobotInteraction* robot_interaction)
{
  boost::mutex::scoped_lock lock(state_lock_);

  // Verivy that this InteractionHandler is only used with one
  // RobotInteraction.
  // This is the only use for robot_interaction_.
  if (robot_interaction_)
  {
    if (robot_interaction_ != robot_interaction)
    {
      ROS_ERROR("setKinematicOptions() called from 2 different RobotInteraction instances.");
    }
    return;
  }

  robot_interaction_ = robot_interaction;

  KinematicOptionsMapPtr shared_kinematic_options_map = robot_interaction->getKinematicOptionsMap();

  // merge old options into shared options
  // This is necessary because some legacy code sets values using deprecated
  // InteractionHandler methods before a RobotInteraction is associated with
  // this InteractionHandler.
  //
  // This is a nop if a constructor with a robot_interaction parameter is used.
  shared_kinematic_options_map->merge(*kinematic_options_map_);

  // from now on the InteractionHandler shares the same KinematicOptionsMap
  // with RobotInteraction.
  // The old *kinematic_options_map_ is automatically deleted by boost::shared_ptr.
  //
  // This is a nop if a constructor with a robot_interaction parameter is used.
  kinematic_options_map_ = shared_kinematic_options_map;
}


// ============= DEPRECATED FUNCTIONS =====================

void InteractionHandler::setIKTimeout(double timeout)
{
  KinematicOptions delta;
  delta.timeout_seconds_ = timeout;

  boost::mutex::scoped_lock lock(state_lock_);
  kinematic_options_map_->setOptions(KinematicOptionsMap::ALL,
                                     delta,
                                     KinematicOptions::TIMEOUT);
}

void InteractionHandler::setIKAttempts(unsigned int attempts)
{
  KinematicOptions delta;
  delta.max_attempts_ = attempts;

  boost::mutex::scoped_lock lock(state_lock_);
  kinematic_options_map_->setOptions(KinematicOptionsMap::ALL,
                                     delta,
                                     KinematicOptions::MAX_ATTEMPTS);
}

void InteractionHandler::setKinematicsQueryOptions(
      const kinematics::KinematicsQueryOptions &opt)
{
  KinematicOptions delta;
  delta.options_ = opt;

  boost::mutex::scoped_lock lock(state_lock_);
  kinematic_options_map_->setOptions(KinematicOptionsMap::ALL,
                                     delta,
                                     KinematicOptions::ALL_QUERY_OPTIONS);
}

void InteractionHandler::setKinematicsQueryOptionsForGroup(
      const std::string& group_name,
      const kinematics::KinematicsQueryOptions &opt)
{
  KinematicOptions delta;
  delta.options_ = opt;

  boost::mutex::scoped_lock lock(state_lock_);
  kinematic_options_map_->setOptions(group_name,
                                     delta,
                                     KinematicOptions::ALL_QUERY_OPTIONS);
}

void InteractionHandler::setGroupStateValidityCallback(
      const robot_state::GroupStateValidityCallbackFn &callback)
{
  KinematicOptions delta;
  delta.state_validity_callback_ = callback;

  boost::mutex::scoped_lock lock(state_lock_);
  kinematic_options_map_->setOptions(KinematicOptionsMap::ALL,
                                     delta,
                                     KinematicOptions::STATE_VALIDITY_CALLBACK);
}

const kinematics::KinematicsQueryOptions& InteractionHandler::getKinematicsQueryOptions() const
{
  boost::mutex::scoped_lock lock(state_lock_);
  return kinematic_options_map_->getOptions(KinematicOptionsMap::DEFAULT).options_;
}


void InteractionHandler::setUpdateCallback(const InteractionHandlerCallbackFn &callback)
{
  boost::mutex::scoped_lock lock(state_lock_);
  update_callback_ = callback;
}

const InteractionHandlerCallbackFn& InteractionHandler::getUpdateCallback() const
{
  boost::mutex::scoped_lock lock(state_lock_);
  return update_callback_;
}

void InteractionHandler::setMeshesVisible(bool visible)
{
  boost::mutex::scoped_lock lock(state_lock_);
  display_meshes_ = visible;
}

bool InteractionHandler::getMeshesVisible() const
{
  boost::mutex::scoped_lock lock(state_lock_);
  return display_meshes_;
}

void InteractionHandler::setControlsVisible(bool visible)
{
  boost::mutex::scoped_lock lock(state_lock_);
  display_controls_ = visible;
}

bool InteractionHandler::getControlsVisible() const
{
  boost::mutex::scoped_lock lock(state_lock_);
  return display_controls_;
}


}
