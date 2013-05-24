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

#ifndef MOVEIT_ROBOT_STATE_CONVERSIONS_
#define MOVEIT_ROBOT_STATE_CONVERSIONS_

#include <moveit/robot_state/robot_state.h>
#include <moveit/transforms/transforms.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace robot_state
{
/**
 * @brief Convert a joint state to a kinematic state
 * @param joint_state The input joint state to be converted
 * @param state The resultant kinematic state
 * @return True if successful, false if failed for any reason
 */
bool jointStateToRobotState(const sensor_msgs::JointState &joint_state, RobotState& state);

/**
 * @brief Convert a robot state (with accompanying extra transforms) to a kinematic state
 * @param tf An instance of a transforms object
 * @param robot_state The input robot state
 * @param state The resultant kinematic state
 * @return True if successful, false if failed for any reason
 */
bool robotStateMsgToRobotState(const Transforms &tf, const moveit_msgs::RobotState &robot_state, RobotState& state, bool copy_attached_bodies = true);
/**
 * @brief Convert a robot state (with accompanying extra transforms) to a kinematic state
 * @param robot_state The input robot state
 * @param state The resultant kinematic state
 * @return True if successful, false if failed for any reason
 */
bool robotStateMsgToRobotState(const moveit_msgs::RobotState &robot_state, RobotState& state, bool copy_attached_bodies = true);

/**
 * @brief Convert a kinematic state to a robot state message
 * @param state The input kinematic state object
 * @param robot_state The resultant RobotState *message
 */
void robotStateToRobotStateMsg(const RobotState& state, moveit_msgs::RobotState &robot_state, bool copy_attached_bodies = true);

/**
 * @brief Convert a kinematic state to a joint state message
 * @param state The input kinematic state object
 * @param robot_state The resultant JointState message
 */
void robotStateToJointStateMsg(const RobotState& state, sensor_msgs::JointState &joint_state);

}

#endif
