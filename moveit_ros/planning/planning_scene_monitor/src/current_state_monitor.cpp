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

#include <moveit/planning_scene_monitor/current_state_monitor.h>

#include <tf_conversions/tf_eigen.h>

#include <limits>

planning_scene_monitor::CurrentStateMonitor::CurrentStateMonitor(const robot_model::RobotModelConstPtr& robot_model,
                                                                 const boost::shared_ptr<tf::Transformer>& tf)
  : CurrentStateMonitor(robot_model, tf, ros::NodeHandle())
{
}

planning_scene_monitor::CurrentStateMonitor::CurrentStateMonitor(const robot_model::RobotModelConstPtr& robot_model,
                                                                 const boost::shared_ptr<tf::Transformer>& tf,
                                                                 ros::NodeHandle nh)
  : nh_(nh)
  , tf_(tf)
  , robot_model_(robot_model)
  , robot_state_(robot_model)
  , state_monitor_started_(false)
  , copy_dynamics_(false)
  , error_(std::numeric_limits<double>::epsilon())
{
  robot_state_.setToDefaultValues();
}

planning_scene_monitor::CurrentStateMonitor::~CurrentStateMonitor()
{
  stopStateMonitor();
}

robot_state::RobotStatePtr planning_scene_monitor::CurrentStateMonitor::getCurrentState() const
{
  boost::mutex::scoped_lock slock(state_update_lock_);
  robot_state::RobotState* result = new robot_state::RobotState(robot_state_);
  return robot_state::RobotStatePtr(result);
}

ros::Time planning_scene_monitor::CurrentStateMonitor::getCurrentStateTime() const
{
  boost::mutex::scoped_lock slock(state_update_lock_);
  return current_state_time_;
}

std::pair<robot_state::RobotStatePtr, ros::Time>
planning_scene_monitor::CurrentStateMonitor::getCurrentStateAndTime() const
{
  boost::mutex::scoped_lock slock(state_update_lock_);
  robot_state::RobotState* result = new robot_state::RobotState(robot_state_);
  return std::make_pair(robot_state::RobotStatePtr(result), current_state_time_);
}

std::map<std::string, double> planning_scene_monitor::CurrentStateMonitor::getCurrentStateValues() const
{
  std::map<std::string, double> m;
  boost::mutex::scoped_lock slock(state_update_lock_);
  const double* pos = robot_state_.getVariablePositions();
  const std::vector<std::string>& names = robot_state_.getVariableNames();
  for (std::size_t i = 0; i < names.size(); ++i)
    m[names[i]] = pos[i];
  return m;
}

void planning_scene_monitor::CurrentStateMonitor::setToCurrentState(robot_state::RobotState& upd) const
{
  boost::mutex::scoped_lock slock(state_update_lock_);
  const double* pos = robot_state_.getVariablePositions();
  upd.setVariablePositions(pos);
}

void planning_scene_monitor::CurrentStateMonitor::addUpdateCallback(const JointStateUpdateCallback& fn)
{
  if (fn)
    update_callbacks_.push_back(fn);
}

void planning_scene_monitor::CurrentStateMonitor::clearUpdateCallbacks()
{
  update_callbacks_.clear();
}

void planning_scene_monitor::CurrentStateMonitor::startStateMonitor(const std::string& joint_states_topic)
{
  if (!state_monitor_started_ && robot_model_)
  {
    joint_time_.clear();
    if (joint_states_topic.empty())
      ROS_ERROR("The joint states topic cannot be an empty string");
    else
      joint_state_subscriber_ = nh_.subscribe(joint_states_topic, 25, &CurrentStateMonitor::jointStateCallback, this);
    if (tf_ && robot_model_->getMultiDOFJointModels().size() > 0)
    {
      tf_connection_.reset(
          new TFConnection(tf_->addTransformsChangedListener(boost::bind(&CurrentStateMonitor::tfCallback, this))));
    }
    state_monitor_started_ = true;
    monitor_start_time_ = ros::Time::now();
    ROS_DEBUG("Listening to joint states on topic '%s'", nh_.resolveName(joint_states_topic).c_str());
  }
}

bool planning_scene_monitor::CurrentStateMonitor::isActive() const
{
  return state_monitor_started_;
}

void planning_scene_monitor::CurrentStateMonitor::stopStateMonitor()
{
  if (state_monitor_started_)
  {
    joint_state_subscriber_.shutdown();
    if (tf_ && tf_connection_)
    {
      tf_->removeTransformsChangedListener(*tf_connection_);
      tf_connection_.reset();
    }
    ROS_DEBUG("No longer listening for joint states");
    state_monitor_started_ = false;
  }
}

std::string planning_scene_monitor::CurrentStateMonitor::getMonitoredTopic() const
{
  if (joint_state_subscriber_)
    return joint_state_subscriber_.getTopic();
  else
    return "";
}

bool planning_scene_monitor::CurrentStateMonitor::haveCompleteState() const
{
  bool result = true;
  const std::vector<const moveit::core::JointModel*>& joints = robot_model_->getJointModels();
  boost::mutex::scoped_lock slock(state_update_lock_);
  for (const moveit::core::JointModel* joint : joints)
    if (joint_time_.find(joint) == joint_time_.end())
    {
      if (!joint->isPassive() && !joint->getMimic())
      {
        ROS_DEBUG("Joint '%s' has never been updated", joint->getName().c_str());
        result = false;
      }
    }
  return result;
}

bool planning_scene_monitor::CurrentStateMonitor::haveCompleteState(std::vector<std::string>& missing_states) const
{
  bool result = true;
  const std::vector<const moveit::core::JointModel*>& joints = robot_model_->getJointModels();
  boost::mutex::scoped_lock slock(state_update_lock_);
  for (const moveit::core::JointModel* joint : joints)
    if (joint_time_.find(joint) == joint_time_.end())
      if (!joint->isPassive() && !joint->getMimic())
      {
        missing_states.push_back(joint->getName());
        result = false;
      }
  return result;
}

bool planning_scene_monitor::CurrentStateMonitor::haveCompleteState(const ros::Duration& age) const
{
  bool result = true;
  const std::vector<const moveit::core::JointModel*>& joints = robot_model_->getJointModels();
  ros::Time now = ros::Time::now();
  ros::Time old = now - age;
  boost::mutex::scoped_lock slock(state_update_lock_);
  for (const moveit::core::JointModel* joint : joints)
  {
    if (joint->isPassive() || joint->getMimic())
      continue;
    std::map<const moveit::core::JointModel*, ros::Time>::const_iterator it = joint_time_.find(joint);
    if (it == joint_time_.end())
    {
      ROS_DEBUG("Joint '%s' has never been updated", joint->getName().c_str());
      result = false;
    }
    else if (it->second < old)
    {
      ROS_DEBUG("Joint '%s' was last updated %0.3lf seconds ago (older than the allowed %0.3lf seconds)",
                joint->getName().c_str(), (now - it->second).toSec(), age.toSec());
      result = false;
    }
  }
  return result;
}

bool planning_scene_monitor::CurrentStateMonitor::haveCompleteState(const ros::Duration& age,
                                                                    std::vector<std::string>& missing_states) const
{
  bool result = true;
  const std::vector<const moveit::core::JointModel*>& joints = robot_model_->getJointModels();
  ros::Time now = ros::Time::now();
  ros::Time old = now - age;
  boost::mutex::scoped_lock slock(state_update_lock_);
  for (const moveit::core::JointModel* joint : joints)
  {
    if (joint->isPassive() || joint->getMimic())
      continue;
    std::map<const moveit::core::JointModel*, ros::Time>::const_iterator it = joint_time_.find(joint);
    if (it == joint_time_.end())
    {
      ROS_DEBUG("Joint '%s' has never been updated", joint->getName().c_str());
      missing_states.push_back(joint->getName());
      result = false;
    }
    else if (it->second < old)
    {
      ROS_DEBUG("Joint '%s' was last updated %0.3lf seconds ago (older than the allowed %0.3lf seconds)",
                joint->getName().c_str(), (now - it->second).toSec(), age.toSec());
      missing_states.push_back(joint->getName());
      result = false;
    }
  }
  return result;
}

bool planning_scene_monitor::CurrentStateMonitor::waitForCurrentState(const ros::Time t, double wait_time) const
{
  ros::WallTime start = ros::WallTime::now();
  ros::WallDuration elapsed(0, 0);
  ros::WallDuration timeout(wait_time);

  boost::mutex::scoped_lock lock(state_update_lock_);
  while (current_state_time_ < t)
  {
    state_update_condition_.wait_for(lock, boost::chrono::nanoseconds((timeout - elapsed).toNSec()));
    elapsed = ros::WallTime::now() - start;
    if (elapsed > timeout)
      return false;
  }
  return true;
}

bool planning_scene_monitor::CurrentStateMonitor::waitForCompleteState(double wait_time) const
{
  double slept_time = 0.0;
  double sleep_step_s = std::min(0.05, wait_time / 10.0);
  ros::Duration sleep_step(sleep_step_s);
  while (!haveCompleteState() && slept_time < wait_time)
  {
    sleep_step.sleep();
    slept_time += sleep_step_s;
  }
  return haveCompleteState();
}
bool planning_scene_monitor::CurrentStateMonitor::waitForCurrentState(double wait_time) const
{
  waitForCompleteState(wait_time);
}

bool planning_scene_monitor::CurrentStateMonitor::waitForCompleteState(const std::string& group, double wait_time) const
{
  if (waitForCompleteState(wait_time))
    return true;
  bool ok = true;

  // check to see if we have a fully known state for the joints we want to record
  std::vector<std::string> missing_joints;
  if (!haveCompleteState(missing_joints))
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
    {
      std::set<std::string> mj;
      mj.insert(missing_joints.begin(), missing_joints.end());
      const std::vector<std::string>& names = jmg->getJointModelNames();
      bool ok = true;
      for (std::size_t i = 0; ok && i < names.size(); ++i)
        if (mj.find(names[i]) != mj.end())
          ok = false;
    }
    else
      ok = false;
  }
  return ok;
}

bool planning_scene_monitor::CurrentStateMonitor::waitForCurrentState(const std::string& group, double wait_time) const
{
  waitForCompleteState(group, wait_time);
}

void planning_scene_monitor::CurrentStateMonitor::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
{
  if (joint_state->name.size() != joint_state->position.size())
  {
    ROS_ERROR_THROTTLE(1, "State monitor received invalid joint state (number of joint names does not match number of "
                          "positions)");
    return;
  }
  bool update = false;

  {
    boost::mutex::scoped_lock _(state_update_lock_);
    // read the received values, and update their time stamps
    std::size_t n = joint_state->name.size();
    current_state_time_ = joint_state->header.stamp;
    for (std::size_t i = 0; i < n; ++i)
    {
      const moveit::core::JointModel* jm = robot_model_->getJointModel(joint_state->name[i]);
      if (!jm)
        continue;
      // ignore fixed joints, multi-dof joints (they should not even be in the message)
      if (jm->getVariableCount() != 1)
        continue;

      joint_time_[jm] = joint_state->header.stamp;

      if (robot_state_.getJointPositions(jm)[0] != joint_state->position[i])
      {
        update = true;
        robot_state_.setJointPositions(jm, &(joint_state->position[i]));

        // optionally copy velocities and effort
        if (copy_dynamics_)
        {
          // check if velocities exist
          if (joint_state->name.size() == joint_state->velocity.size())
          {
            robot_state_.setJointVelocities(jm, &(joint_state->velocity[i]));

            // check if effort exist. assume they are not useful if no velocities were passed in
            if (joint_state->name.size() == joint_state->effort.size())
            {
              robot_state_.setJointEfforts(jm, &(joint_state->effort[i]));
            }
          }
        }

        // continuous joints wrap, so we don't modify them (even if they are outside bounds!)
        if (jm->getType() == moveit::core::JointModel::REVOLUTE)
          if (static_cast<const moveit::core::RevoluteJointModel*>(jm)->isContinuous())
            continue;

        const robot_model::VariableBounds& b =
            jm->getVariableBounds()[0];  // only one variable in the joint, so we get its bounds

        // if the read variable is 'almost' within bounds (up to error_ difference), then consider it to be within
        // bounds
        if (joint_state->position[i] < b.min_position_ && joint_state->position[i] >= b.min_position_ - error_)
          robot_state_.setJointPositions(jm, &b.min_position_);
        else if (joint_state->position[i] > b.max_position_ && joint_state->position[i] <= b.max_position_ + error_)
          robot_state_.setJointPositions(jm, &b.max_position_);
      }
    }
  }

  // callbacks, if needed
  if (update)
    for (std::size_t i = 0; i < update_callbacks_.size(); ++i)
      update_callbacks_[i](joint_state);

  // notify waitForCurrentState() *after* potential update callbacks
  state_update_condition_.notify_all();
}

void planning_scene_monitor::CurrentStateMonitor::tfCallback()
{
  // read multi-dof joint states from TF, if needed
  const std::vector<const moveit::core::JointModel*>& multi_dof_joints = robot_model_->getMultiDOFJointModels();

  ros::Time latest_updates[multi_dof_joints.size()];

  for (size_t i = 0; i < multi_dof_joints.size(); i++)
  {
    const moveit::core::JointModel* joint = multi_dof_joints[i];
    const std::string& child_frame = joint->getChildLinkModel()->getName();
    const std::string& parent_frame =
        joint->getParentLinkModel() ? joint->getParentLinkModel()->getName() : robot_model_->getModelFrame();

    std::string err;
    if (tf_->getLatestCommonTime(parent_frame, child_frame, latest_updates[i], &err) != tf::NO_ERROR)
    {
      ROS_DEBUG_STREAM_THROTTLE(1, "Unable to update multi-dof joint '"
                                       << joint->getName() << "': tf has no common time between '"
                                       << parent_frame.c_str() << "' and '" << child_frame.c_str() << "': " << err);
      latest_updates[i] = ros::Time(0);
      continue;
    }
  }

  bool update = false;
  bool changes = false;

  {
    boost::mutex::scoped_lock _(state_update_lock_);

    for (size_t i = 0; i < multi_dof_joints.size(); i++)
    {
      const moveit::core::JointModel* joint = multi_dof_joints[i];

      if (latest_updates[i] <= joint_time_[joint])
        continue;

      const std::string& child_frame = joint->getChildLinkModel()->getName();
      const std::string& parent_frame =
          joint->getParentLinkModel() ? joint->getParentLinkModel()->getName() : robot_model_->getModelFrame();

      tf::StampedTransform transf;
      try
      {
        tf_->lookupTransform(parent_frame, child_frame, latest_updates[i], transf);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR_STREAM_THROTTLE(1, "Unable to update multi-dof joint '" << joint->getName()
                                                                          << "'. TF exception: " << ex.what());
        continue;
      }
      joint_time_[joint] = latest_updates[i];

      Eigen::Affine3d eigen_transf;
      tf::transformTFToEigen(transf, eigen_transf);

      double new_values[joint->getStateSpaceDimension()];
      joint->computeVariablePositions(eigen_transf, new_values);

      if (joint->distance(new_values, robot_state_.getJointPositions(joint)) > 1e-5)
      {
        changes = true;
      }

      robot_state_.setJointPositions(joint, new_values);
      update = true;
    }
  }

  // callbacks, if needed
  if (changes)
  {
    // stub joint state: multi-dof joints are not modelled in the message,
    // but we should still trigger the update callbacks
    sensor_msgs::JointStatePtr joint_state(new sensor_msgs::JointState);
    for (std::size_t i = 0; i < update_callbacks_.size(); ++i)
      update_callbacks_[i](joint_state);
  }

  if (update)
  {
    // notify waitForCurrentState() *after* potential update callbacks
    state_update_condition_.notify_all();
  }
}
