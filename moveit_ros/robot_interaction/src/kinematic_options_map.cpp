/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, SRI International
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

/* Author: Acorn Pooley */

#include <moveit/robot_interaction/kinematic_options_map.h>

robot_interaction::KinematicOptionsMap::KinematicOptionsMap()
{}

robot_interaction::KinematicOptions
robot_interaction::KinematicOptionsMap::getOptions(
      const std::string& key) const
{
  boost::mutex::scoped_lock lock(lock_);
  M_options::const_iterator it = options_.find(key);
  if (it == options_.end())
    return defaults_;
  return it->second;
}

void robot_interaction::KinematicOptionsMap::setOptions(
      const std::string& key,
      const KinematicOptions& options)
{
  boost::mutex::scoped_lock lock(lock_);
  options_[key] = options;
}

robot_interaction::KinematicOptions
robot_interaction::KinematicOptionsMap::getDefaultOptions() const
{
  boost::mutex::scoped_lock lock(lock_);
  return defaults_;
}

void robot_interaction::KinematicOptionsMap::setDefaultOptions(
      const KinematicOptions& options)
{
  boost::mutex::scoped_lock lock(lock_);
  defaults_ = options;
}

void robot_interaction::KinematicOptionsMap::clear()
{
  boost::mutex::scoped_lock lock(lock_);
  options_.clear();
}

void robot_interaction::KinematicOptionsMap::getKeys(
      std::vector<std::string>& keys) const
{
  boost::mutex::scoped_lock lock(lock_);
  keys.clear();
  keys.reserve(options_.size());
  for (M_options::const_iterator it = options_.begin() ;
       it != options_.end() ;
       ++it)
  {
    keys.push_back(it->first);
  }
}

// This is intended to be called as a ModifyStateFunction to modify the state
// maintained by a LockedRobotState in place.
void robot_interaction::KinematicOptionsMap::setStateFromIK(
      robot_state::RobotState* state,
      const std::string* key,
      const std::string* group,
      const std::string* tip,
      const geometry_msgs::Pose* pose,
      bool* result) const
{
  // copy options so lock is not needed during IK solve.
  KinematicOptions options = getOptions(*key);
  options.setStateFromIK(state, group, tip, pose, result);
}

