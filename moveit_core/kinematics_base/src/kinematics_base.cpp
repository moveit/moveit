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

/* Author: Sachin Chitta, Dave Coleman */

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/joint_model_group.h>

namespace kinematics
{
const double KinematicsBase::DEFAULT_SEARCH_DISCRETIZATION = 0.1;
const double KinematicsBase::DEFAULT_TIMEOUT = 1.0;

void KinematicsBase::setValues(const std::string& robot_description, const std::string& group_name,
                               const std::string& base_frame, const std::string& tip_frame,
                               double search_discretization)
{
  robot_description_ = robot_description;
  group_name_ = group_name;
  base_frame_ = removeSlash(base_frame);
  tip_frame_ = removeSlash(tip_frame);  // for backwards compatibility
  tip_frames_.push_back(removeSlash(tip_frame));
  search_discretization_ = search_discretization;
  setSearchDiscretization(search_discretization);
}

void KinematicsBase::setValues(const std::string& robot_description, const std::string& group_name,
                               const std::string& base_frame, const std::vector<std::string>& tip_frames,
                               double search_discretization)
{
  robot_description_ = robot_description;
  group_name_ = group_name;
  base_frame_ = removeSlash(base_frame);
  search_discretization_ = search_discretization;
  setSearchDiscretization(search_discretization);

  // Copy tip frames to local vector after stripping slashes
  tip_frames_.clear();
  for (std::size_t i = 0; i < tip_frames.size(); ++i)
    tip_frames_.push_back(removeSlash(tip_frames[i]));

  // Copy tip frames to our legacy variable if only one tip frame is passed in the input vector. Remove eventually.
  if (tip_frames.size() == 1)
    tip_frame_ = removeSlash(tip_frames[0]);
}

bool KinematicsBase::setRedundantJoints(const std::vector<unsigned int>& redundant_joint_indices)
{
  for (std::size_t i = 0; i < redundant_joint_indices.size(); ++i)
  {
    if (redundant_joint_indices[i] >= getJointNames().size())
    {
      return false;
    }
  }
  redundant_joint_indices_ = redundant_joint_indices;
  setSearchDiscretization(DEFAULT_SEARCH_DISCRETIZATION);

  return true;
}

bool KinematicsBase::setRedundantJoints(const std::vector<std::string>& redundant_joint_names)
{
  const std::vector<std::string>& jnames = getJointNames();
  std::vector<unsigned int> redundant_joint_indices;
  for (std::size_t i = 0; i < redundant_joint_names.size(); ++i)
    for (std::size_t j = 0; j < jnames.size(); ++j)
      if (jnames[j] == redundant_joint_names[i])
      {
        redundant_joint_indices.push_back(j);
        break;
      }
  return redundant_joint_indices.size() == redundant_joint_names.size() ? setRedundantJoints(redundant_joint_indices) :
                                                                          false;
}

std::string KinematicsBase::removeSlash(const std::string& str) const
{
  return (!str.empty() && str[0] == '/') ? removeSlash(str.substr(1)) : str;
}

bool KinematicsBase::supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error_text_out) const
{
  // Default implementation for legacy solvers:
  if (!jmg->isChain())
  {
    if (error_text_out)
    {
      *error_text_out = "This plugin only supports joint groups which are chains";
    }
    return false;
  }

  return true;
}

bool KinematicsBase::getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                   const std::vector<double>& ik_seed_state,
                                   std::vector<std::vector<double> >& solutions, KinematicsResult& result,
                                   const KinematicsQueryOptions& options) const
{
  std::vector<double> solution;
  result.solution_percentage = 0.0;

  if (std::find(supported_methods_.begin(), supported_methods_.end(), options.discretization_method) ==
      supported_methods_.end())
  {
    result.kinematic_error = KinematicErrors::UNSUPORTED_DISCRETIZATION_REQUESTED;
    return false;
  }

  if (ik_poses.size() != 1)
  {
    ROS_ERROR_NAMED("kinematics_base", "This kinematic solver does not support getPositionIK for multiple poses");
    result.kinematic_error = KinematicErrors::MULTIPLE_TIPS_NOT_SUPPORTED;
    return false;
  }

  if (ik_poses.empty())
  {
    ROS_ERROR_NAMED("kinematics_base", "Input ik_poses array is empty");
    result.kinematic_error = KinematicErrors::EMPTY_TIP_POSES;
    return false;
  }

  moveit_msgs::MoveItErrorCodes error_code;
  if (getPositionIK(ik_poses[0], ik_seed_state, solution, error_code, options))
  {
    solutions.resize(1);
    solutions[0] = solution;
    result.kinematic_error = KinematicErrors::OK;
    result.solution_percentage = 1.0f;
  }
  else
  {
    result.kinematic_error = KinematicErrors::NO_SOLUTION;
    return false;
  }

  return true;
}

}  // end of namespace kinematics
