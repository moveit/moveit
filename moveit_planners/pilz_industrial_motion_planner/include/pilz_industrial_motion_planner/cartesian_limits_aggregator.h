/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#pragma once

#include "pilz_industrial_motion_planner/cartesian_limit.h"

namespace pilz_industrial_motion_planner
{
/**
 * @brief Obtains cartesian limits from the parameter server
 */
class CartesianLimitsAggregator
{
public:
  /**
   * @brief Loads cartesian limits from the parameter server
   *
   * The parameters are expected to be under "~/cartesian_limits" of the given
   * node handle.
   * The following limits can be specified:
   * - "max_trans_vel", the maximum translational velocity [m/s]
   * - "max_trans_acc, the maximum translational acceleration [m/s^2]
   * - "max_trans_dec", the maximum translational deceleration (<= 0) [m/s^2]
   * - "max_rot_vel", the maximum rotational velocity [rad/s]
   * - "max_rot_acc", the maximum rotational acceleration [rad/s^2]
   * - "max_rot_dec", the maximum rotational deceleration (<= 0)[rad/s^2]
   * @param nh node handle to access the parameters
   * @return the obtained cartesian limits
   */
  static CartesianLimit getAggregatedLimits(const ros::NodeHandle& nh);
};

}  // namespace pilz_industrial_motion_planner
