/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef MOVEIT_TRAJECTORY_PROCESSING_TRAJECTORY_TOOLS_
#define MOVEIT_TRAJECTORY_PROCESSING_TRAJECTORY_TOOLS_

#include <moveit/kinematic_state/kinematic_state.h>
#include <moveit/kinematic_state/transforms.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>

namespace trajectory_processing
{

void convertToKinematicStates(std::vector<kinematic_state::KinematicStatePtr> &states,
                              const moveit_msgs::RobotState &start_state, const moveit_msgs::RobotTrajectory &trajectory,
                              const kinematic_state::KinematicState &reference_state, const kinematic_state::TransformsConstPtr &transforms);

void convertToRobotTrajectory(moveit_msgs::RobotTrajectory &trajectory,
                              const std::vector<kinematic_state::KinematicStateConstPtr> &states,
                              const std::vector<ros::Duration> &stamps = std::vector<ros::Duration>(),
                              const std::string &group = std::string());

void convertToRobotTrajectory(moveit_msgs::RobotTrajectory &trajectory,
                              const std::vector<kinematic_state::KinematicStateConstPtr> &states, 
                              const std::string &group);

void addPrefixState(const kinematic_state::KinematicState &prefix, moveit_msgs::RobotTrajectory &trajectory,
                    double dt_offset, const kinematic_state::TransformsConstPtr &transforms);

bool isTrajectoryEmpty(const moveit_msgs::RobotTrajectory &trajectory);

double getTrajectoryDuration(const moveit_msgs::RobotTrajectory &trajectory);

double averageSegmentDuration(const moveit_msgs::RobotTrajectory &trajectory);

std::size_t trajectoryPointCount(const moveit_msgs::RobotTrajectory &trajectory);

/**
 * @brief Convert a RobotTrajectoryPoint (in a RobotTrajectory) to a RobotState message
 * @param rt The input RobotTrajectory
 * @param index The index of the point that we want to convert
 * @param rs The resultant robot state
 * @return True if any data was copied to \e rs. If \e index is out of range, no data is copied and false is returned.
 */
bool robotTrajectoryPointToRobotState(const moveit_msgs::RobotTrajectory &trajectory, std::size_t index, moveit_msgs::RobotState &rs);

}

#endif

