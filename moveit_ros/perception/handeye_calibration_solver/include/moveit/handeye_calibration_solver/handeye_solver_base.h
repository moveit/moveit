/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019,  Intel Corporation.
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

/* Author: Yu Yan */

#pragma once

#include <tf2_eigen/tf2_eigen.h>

namespace moveit_handeye_calibration
{
enum SensorMountType
{
  EYE_TO_HAND = 0,
  EYE_IN_HAND = 1,
};

class HandEyeSolverBase
{
public:
  HandEyeSolverBase() = default;
  virtual ~HandEyeSolverBase() = default;

  virtual void initialize() = 0;

  /**
   * @brief Get the names of available algorithms that can be used from the plugin.
   * @return A vector storing the names.
   */
  virtual const std::vector<std::string>& getSolverNames() const = 0;

  /**
   * @brief Calculate camera-robot transform from the input pose samples.
   * @param effector_wrt_world End-effector pose (4X4 transform) with respect to the world (or robot base).
   * @param object_wrt_sensor Object (calibration board) pose (4X4 transform) with respect to the camera.
   * @param setup Camera mount type, {EYE_TO_HAND, EYE_IN_HAND}.
   * @param solver_name The algorithm used in the calculation.
   * @return If the calculation succeeds, return true. Otherwise, return false.
   */
  virtual bool solve(const std::vector<Eigen::Isometry3d>& effector_wrt_world,
                     const std::vector<Eigen::Isometry3d>& object_wrt_sensor, SensorMountType setup = EYE_TO_HAND,
                     const std::string& solver_name = "") = 0;

  /**
   * @brief Get the result of the calibration, i.e. the camera pose with respect to the robot.
   * @return A 4X4 transform indicating the pose.
   */
  virtual const Eigen::Isometry3d& getCameraRobotPose() const = 0;
};

}  // namespace moveit_handeye_calibration