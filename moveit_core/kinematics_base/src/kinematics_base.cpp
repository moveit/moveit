/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Sachin Chitta
*********************************************************************/

#include <moveit/kinematics_base/kinematics_base.h>

const double kinematics::KinematicsBase::DEFAULT_SEARCH_DISCRETIZATION = 0.1;
const double kinematics::KinematicsBase::DEFAULT_TIMEOUT = 1.0;

void kinematics::KinematicsBase::setValues(const std::string& robot_description,
                       const std::string& group_name,
                       const std::string& base_frame,
                       const std::string& tip_frame,
                       double search_discretization)
{
  robot_description_ = robot_description;
  group_name_ = group_name;
  base_frame_ = removeSlash(base_frame);
  tip_frame_ = removeSlash(tip_frame);
  search_discretization_ = search_discretization;
}

bool kinematics::KinematicsBase::setRedundantJoints(const std::vector<unsigned int> &redundant_joint_indices)
{
  for(std::size_t i = 0; i < redundant_joint_indices.size(); ++i)
  {
    if(redundant_joint_indices[i] >= getJointNames().size())
    {
      return false;
    }
  }
  redundant_joint_indices_ = redundant_joint_indices;
  return true;
}

bool kinematics::KinematicsBase::setRedundantJoints(const std::vector<std::string> &redundant_joint_names)
{
  const std::vector<std::string> &jnames = getJointNames();
  std::vector<unsigned int> redundant_joint_indices;
  for (std::size_t i = 0 ; i < redundant_joint_names.size() ; ++i)
    for (std::size_t j = 0 ; j < jnames.size() ; ++j)
      if (jnames[j] == redundant_joint_names[i])
      {
    redundant_joint_indices.push_back(j);
    break;
      }
  return redundant_joint_indices.size() == redundant_joint_names.size() ? setRedundantJoints(redundant_joint_indices) : false;
}

std::string kinematics::KinematicsBase::removeSlash(const std::string &str) const
{
  return (!str.empty() && str[0] == '/') ? removeSlash(str.substr(1)) : str;
}
