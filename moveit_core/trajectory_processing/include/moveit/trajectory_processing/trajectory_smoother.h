/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan, E. Gil Jones */

#ifndef TRAJECTORY_SMOOTHER_H_
#define TRAJECTORY_SMOOTHER_H_

#include <vector>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/JointLimits.h>
#include <moveit_msgs/RobotState.h>

namespace trajectory_processing
{

/**
 * \brief Abstract base class for trajectory smoothing.
 *
 * A trajectory smoother is a class that takes in a JointTrajectory message, potentially
 * containing no velocities / accelerations, and fills them in using some rules. Example
 * implementations are in "ClampedCubicTrajectorySmoother", "FritschButlandTrajectorySmoother" and
 * "NumericalDifferentiationTrajectorySmoother", each of which uses a different set of rules
 * to fill in the velocities and accelerations.
 *
 */
class TrajectorySmoother
{
public:
  TrajectorySmoother(){}
  virtual ~TrajectorySmoother(){}

  /**
   * \brief Smooths the input position trajectory by generating velocities and accelerations at the waypoints.
   *
   * This virtual method needs to implemented by the derived class.
   * \return true if successful, false if not
   */
  virtual bool smooth(const trajectory_msgs::JointTrajectory& trajectory_in,
                      trajectory_msgs::JointTrajectory& trajectory_out,
                      const std::vector<moveit_msgs::JointLimits>& joint_limits) const = 0;


  /**
   * \brief Smooths the input position trajectory by generating velocities and accelerations at the waypoints.
   *        Uses the start_state to allow for non-zero starting joint velocities.
   *
   * This virtual method needs to implemented by the derived class.
   * \return true if successful, false if not
   */
  virtual bool smooth(const trajectory_msgs::JointTrajectory& trajectory_in,
                      trajectory_msgs::JointTrajectory& trajectory_out,
                      const std::vector<moveit_msgs::JointLimits>& joint_limits,
                      const moveit_msgs::RobotState& start_state) const = 0;
};

}

#endif /* TRAJECTORY_SMOOTHER_H_ */
