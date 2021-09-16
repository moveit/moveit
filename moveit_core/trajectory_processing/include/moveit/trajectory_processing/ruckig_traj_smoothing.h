/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Jack Center, Wyatt Rees, Andy Zelenak */

#pragma once

#include <Eigen/Core>
#include <list>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ruckig/ruckig.hpp>

namespace trajectory_processing
{
class RuckigSmoothing
{
public:
  static bool applySmoothing(robot_trajectory::RobotTrajectory& trajectory,
                             const double max_velocity_scaling_factor = 1.0,
                             const double max_acceleration_scaling_factor = 1.0);

private:
  /**
   * \brief Feed previous output back as input for next iteration. Get next target state from the next waypoint.
   */
  static void getNextCurrentTargetStates(ruckig::InputParameter<0>& ruckig_input,
                                         ruckig::OutputParameter<0>& ruckig_output,
                                         const moveit::core::RobotStatePtr& next_waypoint, size_t num_dof,
                                         const std::vector<int>& idx);

  /**
   * \brief Check for lagging motion of any joint at a waypoint.
   *
   * \return true if lagging motion is detected on any joint
   */
  static bool checkForLaggingMotion(const size_t num_dof, const ruckig::InputParameter<0>& ruckig_input,
                                    const ruckig::OutputParameter<0>& ruckig_output);

  /**
   * \brief Return L2-norm of velocity, taking all joints into account.
   */
  static double getTargetVelocityMagnitude(const ruckig::InputParameter<0>& ruckig_input, size_t num_dof);
};
}  // namespace trajectory_processing
