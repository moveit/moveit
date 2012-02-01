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

/* Author: Ioan Sucan */

#include <planning_models/conversions.h>
#include <ros/console.h>
#include <set>

namespace planning_models
{
static bool jointStateToKinematicState(const sensor_msgs::JointState &joint_state, KinematicState& state, std::set<std::string> *missing)
{
  if (joint_state.name.size() != joint_state.position.size())
  {
    ROS_ERROR_STREAM("Different number of names and positions in JointState message: " << joint_state.name.size()
                     << ", " << joint_state.position.size());
    return false;
  }
  
  std::map<std::string, double> joint_state_map;
  for (unsigned int i = 0 ; i < joint_state.name.size(); ++i)
    joint_state_map[joint_state.name[i]] = joint_state.position[i];
  
  if (missing == NULL)
    state.setStateValues(joint_state_map);
  else
  {
    std::vector<std::string> missing_variables;
    state.setStateValues(joint_state_map, missing_variables);
    missing->clear();
    for (unsigned int i = 0; i < missing_variables.size(); ++i)
      missing->insert(missing_variables[i]);
  }
  return true;
}

static bool multiDOFJointsToKinematicState(const moveit_msgs::MultiDOFJointState &mjs, KinematicState &state, const Transforms *tf)
{
  if (mjs.joint_names.size() != mjs.frame_ids.size() || mjs.joint_names.size() != mjs.child_frame_ids.size() ||
      mjs.joint_names.size() != mjs.poses.size())
  {
    ROS_ERROR("Different number of names, values or frames in MultiDOFJointState message.");
    return false;
  }
  
  std::vector<Eigen::Affine3d> transf(mjs.joint_names.size());
  bool tf_problem = false;
  bool error = false;
  
  for (unsigned int i = 0 ; i < mjs.joint_names.size(); ++i)
  {
    if (!poseFromMsg(mjs.poses[i], transf[i]))
      ROS_WARN("MultiDOFJointState message has incorrect quaternion specification for joint '%s'. Assuming identity.",
               mjs.joint_names[i].c_str());
    
    // if frames do not mach, attempt to transform
    if (mjs.frame_ids[i] != state.getKinematicModel()->getModelFrame())
    {
      bool ok = true;
      if (tf)
      {
        try
        {
          // find the transform that takes the given frame_id to the desired fixed frame
          const Eigen::Affine3d &t2fixed_frame = tf->getTransform(mjs.frame_ids[i]);
          // we update the value of the transform so that it transforms from the known fixed frame to the desired child link
          transf[i] = transf[i]*t2fixed_frame.inverse();
        }
        catch (std::runtime_error&)
        {
          ok = false;
        }
      }
      else
        ok = false;
      if (!ok)
      {
        tf_problem = true;
        ROS_WARN("The transform for joint '%s' was specified in frame '%s' but it was not possible to update that transform to frame '%s'",
                 mjs.joint_names[i].c_str(), mjs.frame_ids[i].c_str(), state.getKinematicModel()->getModelFrame().c_str());
      }
    }
  }
  
  for (unsigned int i = 0 ; i < mjs.joint_names.size(); ++i)
  {
    const std::string &joint_name = mjs.joint_names[i];
    if (!state.hasJointState(joint_name))
    {
      ROS_WARN("No joint matching multi-dof joint '%s'", joint_name.c_str());
      error = true;
      continue;
    }
    planning_models::KinematicState::JointState* joint_state = state.getJointState(joint_name);
    
    if (mjs.child_frame_ids[i] != joint_state->getJointModel()->getChildLinkModel()->getName())
    {
      ROS_WARN_STREAM("Robot state msg has bad multi_dof transform - child frame_ids do not match up with joint");
      tf_problem = true;
    }
    
    joint_state->setVariableValues(transf[i]);
  }
  
  return !tf_problem && !error;
}

static bool robotStateToKinematicStateHelper(const Transforms *tf, const moveit_msgs::RobotState &robot_state, KinematicState& state)
{
  std::set<std::string> missing;
  bool result1 = jointStateToKinematicState(robot_state.joint_state, state, &missing);
  bool result2 = multiDOFJointsToKinematicState(robot_state.multi_dof_joint_state, state, tf);
  state.updateLinkTransforms();
  
  if (result1 && result2)
  {
    if (!missing.empty())
      for (unsigned int i = 0 ; i < robot_state.multi_dof_joint_state.joint_names.size(); ++i)
      {
        const KinematicModel::JointModel *jm = state.getKinematicModel()->getJointModel(robot_state.multi_dof_joint_state.joint_names[i]);
        if (jm)
        {
          const std::vector<std::string> &vnames = jm->getVariableNames();
          for (unsigned int i = 0 ; i < vnames.size(); ++i)
            missing.erase(vnames[i]);
        }
      }
    
    return missing.empty();
  }
  else
    return false;
}
}


bool planning_models::jointStateToKinematicState(const sensor_msgs::JointState &joint_state, KinematicState& state)
{
  bool result = jointStateToKinematicState(joint_state, state, NULL);
  state.updateLinkTransforms();
  return result;
}

bool planning_models::robotStateToKinematicState(const moveit_msgs::RobotState &robot_state, KinematicState& state)
{
  return robotStateToKinematicStateHelper(NULL, robot_state, state);
}

bool planning_models::robotStateToKinematicState(const Transforms &tf, const moveit_msgs::RobotState &robot_state, KinematicState& state)
{
  return robotStateToKinematicStateHelper(&tf, robot_state, state);
}

void planning_models::kinematicStateToRobotState(const KinematicState& state, moveit_msgs::RobotState &robot_state)
{
  kinematicStateToJointState(state, robot_state.joint_state);
  const std::vector<KinematicState::JointState*> &js = state.getJointStateVector();
  robot_state.multi_dof_joint_state = moveit_msgs::MultiDOFJointState();
  for (std::size_t i = 0 ; i < js.size() ; ++i)
    if (js[i]->getVariableCount() > 1)
    {
      geometry_msgs::Pose p;
      msgFromPose(js[i]->getVariableTransform(), p);
      robot_state.multi_dof_joint_state.joint_names.push_back(js[i]->getName());
      robot_state.multi_dof_joint_state.frame_ids.push_back(state.getKinematicModel()->getModelFrame());
      robot_state.multi_dof_joint_state.child_frame_ids.push_back(js[i]->getJointModel()->getChildLinkModel()->getName());
      robot_state.multi_dof_joint_state.poses.push_back(p);
    }
}

void planning_models::kinematicStateToJointState(const KinematicState& state, sensor_msgs::JointState &joint_state)
{
  const std::vector<KinematicState::JointState*> &js = state.getJointStateVector();
  joint_state = sensor_msgs::JointState();
  
  for (std::size_t i = 0 ; i < js.size() ; ++i)
    if (js[i]->getVariableCount() == 1)
    {
      joint_state.name.push_back(js[i]->getName());
      joint_state.position.push_back(js[i]->getVariableValues()[0]);
    }
  
  joint_state.header.frame_id = state.getKinematicModel()->getModelFrame();
}

void planning_models::robotTrajectoryPointToRobotState(const moveit_msgs::RobotTrajectory &rt, std::size_t index, moveit_msgs::RobotState &rs)
{
  if (rt.joint_trajectory.points.size() > index)
  {
    rs.joint_state.header = rt.joint_trajectory.header;
    rs.joint_state.header.stamp = rs.joint_state.header.stamp + rt.joint_trajectory.points[index].time_from_start;
    rs.joint_state.name = rt.joint_trajectory.joint_names;
    rs.joint_state.position = rt.joint_trajectory.points[index].positions;
    rs.joint_state.velocity = rt.joint_trajectory.points[index].velocities;
  }
  
  if (rt.multi_dof_joint_trajectory.points.size() > index)
  {
    rs.multi_dof_joint_state.joint_names = rt.multi_dof_joint_trajectory.joint_names;
    rs.multi_dof_joint_state.frame_ids = rt.multi_dof_joint_trajectory.frame_ids;
    rs.multi_dof_joint_state.child_frame_ids = rt.multi_dof_joint_trajectory.child_frame_ids;
    rs.multi_dof_joint_state.stamp = rt.joint_trajectory.header.stamp + rt.multi_dof_joint_trajectory.points[index].time_from_start;
    rs.multi_dof_joint_state.poses = rt.multi_dof_joint_trajectory.points[index].poses;
  }
}
