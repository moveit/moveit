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

/** \author Sachin Chitta */

#ifndef CUBIC_PARAMETERIZED_TRAJECTORY_H_
#define CUBIC_PARAMETERIZED_TRAJECTORY_H_

#include <trajectory_processing/trajectory_processing_utils.h>
#include <trajectory_processing/cubic_trajectory.h>

namespace trajectory_processing
{
/**
 * \brief 
 */
  class CubicParameterizedTrajectory 
  {
    public:

    CubicParameterizedTrajectory();

    bool parameterize(const trajectory_msgs::JointTrajectory& trajectory_in, 
                      const std::vector<moveit_msgs::JointLimits> &limits,
                      spline_msgs::SplineTrajectory& spline);

  private:

    double getDistance(const trajectory_msgs::JointTrajectoryPoint &start, 
                       const trajectory_msgs::JointTrajectoryPoint &end,
                       const std::vector<moveit_msgs::JointLimits> &limits);

    double getVelocityLimit(const trajectory_msgs::JointTrajectoryPoint &start, 
                            const trajectory_msgs::JointTrajectoryPoint &end,
                            const std::vector<moveit_msgs::JointLimits> &limits);

    double getAccelerationLimit(const trajectory_msgs::JointTrajectoryPoint &start, 
                                const trajectory_msgs::JointTrajectoryPoint &end,
                                const std::vector<moveit_msgs::JointLimits> &limits);

    bool hasAccelerationLimits(const std::vector<moveit_msgs::JointLimits> &limits);

    void getLimit(const trajectory_msgs::JointTrajectoryPoint &start, 
                  const trajectory_msgs::JointTrajectoryPoint &end,
                  const std::vector<moveit_msgs::JointLimits> &limits,
                  moveit_msgs::JointLimits &limit_out);

    double jointDiff(const double &start, 
                     const double &end,
                     const moveit_msgs::JointLimits &limit);
    
    void solveCubicSpline(const double &q0,
                          const double &q1,
                          const double &v0, 
                          const double &v1,
                          const double &dt,
                          std::vector<double> &coefficients);

    static const double EPS_TRAJECTORY = 1.0e-8;

    bool apply_limits_;
  };
}

#endif /* CUBIC_PARAMETERIZED_TRAJECTORY_H_ */
