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

RobotInteraction::InteractionHandler::InteractionHandler(const std::string &name,
                                                         const robot_state::RobotState &kstate,
                                                         const boost::shared_ptr<tf::Transformer> &tf) :
  name_(name),
  kstate_(new robot_state::RobotState(kstate)),
  tf_(tf),
  display_meshes_(true),
  display_controls_(true)
{
  setup();
}

RobotInteraction::InteractionHandler::InteractionHandler(const std::string &name,
                                                         const robot_model::RobotModelConstPtr &robot_model,
                                                         const boost::shared_ptr<tf::Transformer> &tf) :
  name_(name),
  kstate_(new robot_state::RobotState(robot_model)),
  tf_(tf),
  display_meshes_(true),
  display_controls_(true)
{
  setup();
}

void RobotInteraction::InteractionHandler::setup()
{
  std::replace(name_.begin(), name_.end(), '_', '-'); // we use _ as a special char in marker name
  ik_timeout_ = 0.0; // so that the default IK timeout is used in setFromIK()
  ik_attempts_ = 0; // so that the default IK attempts is used in setFromIK()
  planning_frame_ = kstate_->getRobotModel()->getModelFrame();
}

void RobotInteraction::InteractionHandler::setPoseOffset(const RobotInteraction::EndEffector& eef, const geometry_msgs::Pose& m)
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  offset_map_[eef.eef_group] = m;
}

void RobotInteraction::InteractionHandler::setPoseOffset(const RobotInteraction::Joint& vj, const geometry_msgs::Pose& m)
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  offset_map_[vj.joint_name] = m;
}

void RobotInteraction::InteractionHandler::clearPoseOffset(const RobotInteraction::EndEffector& eef)
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  offset_map_.erase(eef.eef_group);
}

void RobotInteraction::InteractionHandler::clearPoseOffset(const RobotInteraction::Joint& vj)
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  offset_map_.erase(vj.joint_name);
}

void RobotInteraction::InteractionHandler::clearPoseOffsets()
{
  boost::mutex::scoped_lock slock(offset_map_lock_);
  offset_map_.clear();
}

bool RobotInteraction::InteractionHandler::getPoseOffset(const RobotInteraction::EndEffector& eef, geometry_msgs::Pose& m)
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

bool RobotInteraction::InteractionHandler::getPoseOffset(const RobotInteraction::Joint& vj, geometry_msgs::Pose& m)
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

bool RobotInteraction::InteractionHandler::getLastEndEffectorMarkerPose(const RobotInteraction::EndEffector& eef, geometry_msgs::PoseStamped& ps)
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

bool RobotInteraction::InteractionHandler::getLastJointMarkerPose(const RobotInteraction::Joint& vj, geometry_msgs::PoseStamped& ps)
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

void RobotInteraction::InteractionHandler::clearLastEndEffectorMarkerPose(const RobotInteraction::EndEffector& eef)
{
  boost::mutex::scoped_lock slock(pose_map_lock_);
  pose_map_.erase(eef.eef_group);
}

void RobotInteraction::InteractionHandler::clearLastJointMarkerPose(const RobotInteraction::Joint& vj)
{
  boost::mutex::scoped_lock slock(pose_map_lock_);
  pose_map_.erase(vj.joint_name);
}

void RobotInteraction::InteractionHandler::clearLastMarkerPoses()
{
  boost::mutex::scoped_lock slock(pose_map_lock_);
  pose_map_.clear();
}

void RobotInteraction::InteractionHandler::setMenuHandler(const boost::shared_ptr<interactive_markers::MenuHandler>& mh)
{
  menu_handler_ = mh;
}

const boost::shared_ptr<interactive_markers::MenuHandler>& RobotInteraction::InteractionHandler::getMenuHandler()
{
  return menu_handler_;
}

void RobotInteraction::InteractionHandler::clearMenuHandler()
{
  menu_handler_.reset();
}

robot_state::RobotStateConstPtr RobotInteraction::InteractionHandler::getState() const
{
  boost::unique_lock<boost::mutex> ulock(state_lock_);
  while (!kstate_)
    state_available_condition_.wait(ulock);
  kstate_->update();
  return kstate_;
}

void RobotInteraction::InteractionHandler::setState(const robot_state::RobotState& kstate)
{
  boost::unique_lock<boost::mutex> ulock(state_lock_);
  while (!kstate_)
    state_available_condition_.wait(ulock);
  if (kstate_.unique())
    *kstate_ = kstate;
  else
    kstate_.reset(new robot_state::RobotState(kstate));
}

robot_state::RobotStatePtr RobotInteraction::InteractionHandler::getUniqueStateAccess()
{
  robot_state::RobotStatePtr result;
  {
    boost::unique_lock<boost::mutex> ulock(state_lock_);
    if (!kstate_)
    {
      do
      {
        state_available_condition_.wait(ulock);
      } while (!kstate_);
    }
    result.swap(kstate_);
  }
  if (!result.unique())
    result.reset(new robot_state::RobotState(*result));
  return result;
}

void RobotInteraction::InteractionHandler::setStateToAccess(robot_state::RobotStatePtr &state)
{
  boost::unique_lock<boost::mutex> ulock(state_lock_);
  if (state != kstate_)
    kstate_.swap(state);
  state_available_condition_.notify_all();
}

void RobotInteraction::InteractionHandler::handleGeneric(const RobotInteraction::Generic &g, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (g.process_feedback)
  {
    robot_state::RobotStatePtr state = getUniqueStateAccess();
    bool ok = g.process_feedback(*state, feedback);
    setStateToAccess(state);

    bool error_state_changed = false;
    if (!ok)
    {
      error_state_changed = inError(g) ? false : true;
      error_state_.insert(g.marker_name_suffix);
    }
    else
    {
      error_state_changed = inError(g) ? true : false;
      error_state_.erase(g.marker_name_suffix);
    }

    if (update_callback_)
      update_callback_(this, error_state_changed);
  }
}

void RobotInteraction::InteractionHandler::handleEndEffector(const robot_interaction::RobotInteraction::EndEffector &eef,
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

  robot_state::RobotStatePtr state = getUniqueStateAccess();

  kinematics::KinematicsQueryOptions options;
  if(!getKinematicsQueryOptionsForGroup(eef.parent_group, options))
    options = kinematics_query_options_;

  bool update_state_result = robot_interaction::RobotInteraction::updateState(*state, eef, tpose.pose, ik_attempts_, ik_timeout_, state_validity_callback_fn_, options);
  setStateToAccess(state);

  bool error_state_changed = false;
  if (!update_state_result)
  {
    error_state_changed = inError(eef) ? false : true;
    error_state_.insert(eef.parent_group);
  }
  else
  {
    error_state_changed = inError(eef) ? true : false;
    error_state_.erase(eef.parent_group);
  }

  if (update_callback_)
    update_callback_(this, error_state_changed);
}

void RobotInteraction::InteractionHandler::handleJoint(const robot_interaction::RobotInteraction::Joint &vj,
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

  if (!vj.parent_frame.empty() && !robot_state::Transforms::sameFrame(vj.parent_frame, planning_frame_))
  {
    robot_state::RobotStatePtr state = getUniqueStateAccess();
    Eigen::Affine3d p;
    tf::poseMsgToEigen(tpose.pose, p);
    tf::poseEigenToMsg(state->getGlobalLinkTransform(vj.parent_frame).inverse() * p, tpose.pose);
    robot_interaction::RobotInteraction::updateState(*state, vj, tpose.pose);
    setStateToAccess(state);
  }
  else
  {
    robot_state::RobotStatePtr state = getUniqueStateAccess();
    robot_interaction::RobotInteraction::updateState(*state, vj, tpose.pose);
    setStateToAccess(state);
  }

  if (update_callback_)
    update_callback_(this, false);
}

bool RobotInteraction::InteractionHandler::inError(const robot_interaction::RobotInteraction::EndEffector& eef) const
{
  return error_state_.find(eef.parent_group) != error_state_.end();
}

bool RobotInteraction::InteractionHandler::inError(const robot_interaction::RobotInteraction::Generic& g) const
{
  return error_state_.find(g.marker_name_suffix) != error_state_.end();
}

bool RobotInteraction::InteractionHandler::inError(const robot_interaction::RobotInteraction::Joint& vj) const
{
  return false;
}

void RobotInteraction::InteractionHandler::clearError(void)
{
  error_state_.clear();
}

bool RobotInteraction::InteractionHandler::transformFeedbackPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
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
        ROS_ERROR("Error transforming from frame '%s' to frame '%s'", tpose.header.frame_id.c_str(), planning_frame_.c_str());
        return false;
      }
    else
    {
      ROS_ERROR("Cannot transform from frame '%s' to frame '%s' (no TF instance provided)", tpose.header.frame_id.c_str(), planning_frame_.c_str());
      return false;
    }
  }
  return true;
}

}
