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

#include <gtest/gtest.h>
#include <trajectory_processing/linear_spline_velocity_scaler.h>

using namespace trajectory_processing;

TEST(TestLinearSplineVelocityScaler, TestLinearSplineVelocityScaler1)
{
  // create the input:
  int length = 4;
  int joints = 2;

  trajectory_msgs::JointTrajectory trajectory;
  trajectory_msgs::JointTrajectory trajectory_out;
  std::vector<moveit_msgs::JointLimits> limits;

  trajectory.points.resize(length);
  trajectory.joint_names.resize(joints);
  trajectory.joint_names[0] = std::string("test0");
  trajectory.joint_names[1] = std::string("test1");

  limits.resize(joints);
  limits[0].max_velocity = 0.5;
  limits[0].has_velocity_limits = 1;

  limits[1].max_velocity = 0.25;
  limits[1].has_velocity_limits = 1;

  for (int i=0; i<length; i++)
  {
    trajectory.points[i].positions.resize(joints);
    trajectory.points[i].velocities.resize(joints);
    trajectory.points[i].accelerations.resize(joints);
    for(int j=0; j<joints; j++)
    {
      trajectory.points[i].positions[j] = i+j;
      trajectory.points[i].velocities[j] = 0.0;
      trajectory.points[i].accelerations[j] = 0.0;
    }
    trajectory.points[i].time_from_start = ros::Duration(0.0);
  }
  
  LinearSplineVelocityScaler scaler;
  scaler.smooth(trajectory, trajectory_out, limits);

  // verify that velocities are 0:
  EXPECT_NEAR(trajectory_out.points[0].time_from_start.toSec(), 0.0, 1e-8);
  EXPECT_NEAR(trajectory_out.points[1].time_from_start.toSec(), 4.0, 1e-8);
  EXPECT_NEAR(trajectory_out.points[2].time_from_start.toSec(), 8.0, 1e-8);
  EXPECT_NEAR(trajectory_out.points[3].time_from_start.toSec(), 12.0, 1e-8);

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_linear_spline_velocity_scaler");
  return RUN_ALL_TESTS();
}
