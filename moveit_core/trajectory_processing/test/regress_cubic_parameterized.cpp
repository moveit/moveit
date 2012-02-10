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
#include <trajectory_processing/cubic_parameterized_trajectory.h>
#include <random_numbers/random_numbers.h>

TEST(TestCubicParameterizedTrajectory, TestCubicParameterizedTrajectoryWithWrapAround)
{
  trajectory_processing::CubicParameterizedTrajectory traj;

  // create the input:
  int length = 2;
  int joints = 1;

  trajectory_msgs::JointTrajectory trajectory;
  std::vector<moveit_msgs::JointLimits> limits;
  trajectory.points.resize(length);
  trajectory.joint_names.resize(joints);
  trajectory.joint_names[0] = std::string("test0");

  limits.resize(joints);
  limits[0].max_velocity = 0.5;
  limits[0].has_velocity_limits = 1;

  //  limits[1].max_velocity = 0.25;
  //  limits[1].has_velocity_limits = 1;

  for (int i=0; i<length; i++)
  {
    trajectory.points[i].positions.resize(joints);
    trajectory.points[i].velocities.resize(joints);
    trajectory.points[i].accelerations.resize(joints);
    for(int j=0; j<joints; j++)
    {
      trajectory.points[i].positions[j] = 0.0;
      trajectory.points[i].velocities[j] = 0.0;
      trajectory.points[i].accelerations[j] = 0.0;
    }
    trajectory.points[i].time_from_start = ros::Duration(0.0);
  }

  trajectory.points[0].positions[0] = M_PI;
  trajectory.points[1].positions[0] = -M_PI;

  spline_msgs::SplineTrajectory spline;
  success = traj.parameterize(trajectory,limits,spline);

  double total_time;
  bool ss = trajectory_processing::getTotalTime(spline,total_time);
  EXPECT_TRUE(ss);

  trajectory_msgs::JointTrajectory wpt_out;
  std::vector<double> times_out;
  times_out.resize(length);
  times_out[0] = 0.0;
  times_out[1] = total_time;
  trajectory_processing::sampleSplineTrajectory(spline,times_out,wpt_out);
  EXPECT_NEAR(wpt_out.points[0].positions[0], M_PI, 1e-5);
  EXPECT_NEAR(wpt_out.points[1].positions[0], -M_PI, 1e-5);
}


TEST(TestCubicParameterizedTrajectory, TestCubicParameterizedTrajectory)
{
  trajectory_processing::CubicParameterizedTrajectory traj;

  // create the input:
  int length = 4;
  int joints = 2;

  trajectory_msgs::JointTrajectory trajectory;
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

  spline_msgs::SplineTrajectory spline;
  bool success = traj.parameterize(trajectory,limits,spline);

  double total_time;
  bool ss = trajectory_processing::getTotalTime(spline,total_time);
  EXPECT_TRUE(ss);

  double dT = 0.01;
  int sample_length = (int) (total_time/dT);

  std::vector<double> times;
  times.resize(sample_length);
  for (int i=0; i<sample_length; i++)
  {
    times[i] = dT*i;
  }

  EXPECT_TRUE(success);

  trajectory_msgs::JointTrajectory wpt_out;
  std::vector<double> times_out;
  times_out.resize(length);
  for (int i=0; i<length; i++)
  {
    times_out[i] = i;
  }
  trajectory_processing::sampleSplineTrajectory(spline,times_out,wpt_out);
  EXPECT_NEAR(wpt_out.points[0].positions[0], 0.0, 1e-5);
  EXPECT_NEAR(wpt_out.points[1].positions[0], 0.0740741, 1e-5);
  EXPECT_NEAR(wpt_out.points[2].positions[0], 0.259259, 1e-5);
  EXPECT_NEAR(wpt_out.points[3].positions[0], 0.5, 1e-5);

  EXPECT_NEAR(wpt_out.points[0].positions[1], 1.0, 1e-5);
  EXPECT_NEAR(wpt_out.points[1].positions[1], 1.0740741, 1e-5);
  EXPECT_NEAR(wpt_out.points[2].positions[1], 1.259259, 1e-5);
  EXPECT_NEAR(wpt_out.points[3].positions[1], 1.5, 1e-5);
}

TEST(TestCubicParameterizedTrajectory, TestWithAccelerationLimits1)
{
  trajectory_processing::CubicParameterizedTrajectory traj;

  // create the input:
  int length = 2;
  int joints = 1;

  trajectory_msgs::JointTrajectory trajectory;
  std::vector<moveit_msgs::JointLimits> limits;
  trajectory.points.resize(length);
  trajectory.joint_names.resize(joints);
  trajectory.joint_names[0] = std::string("test0");

  limits.resize(joints);
  limits[0].max_velocity = 0.2;
  limits[0].has_velocity_limits = 1;

  limits[0].max_acceleration = 0.1;
  limits[0].has_acceleration_limits = 1;

  for (int i=0; i<length; i++)
  {
    trajectory.points[i].positions.resize(joints);
    trajectory.points[i].velocities.resize(joints);
    trajectory.points[i].accelerations.resize(joints);
    trajectory.points[i].time_from_start = ros::Duration(0.0);
  }
  trajectory.points[1].positions[0] = 1.0;
  spline_msgs::SplineTrajectory spline;
  bool success = traj.parameterize(trajectory,limits,spline);
  EXPECT_TRUE(success);

  double total_time;
  bool ss = trajectory_processing::getTotalTime(spline,total_time);
  EXPECT_TRUE(ss);
  EXPECT_NEAR(total_time,7.74597,1e-5);

  trajectory_msgs::JointTrajectory wpt_out;
  std::vector<double> times_out;
  times_out.push_back(total_time);
  trajectory_processing::sampleSplineTrajectory(spline,times_out,wpt_out);

  EXPECT_NEAR(trajectory.points[1].positions[0],wpt_out.points[0].positions[0],1e-5);
  EXPECT_NEAR(trajectory.points[1].velocities[0],wpt_out.points[0].velocities[0],1e-5);
}

TEST(TestCubicParameterizedTrajectory, TestWithAccelerationLimits2)
{
  trajectory_processing::CubicParameterizedTrajectory traj;
  srand ( time(NULL) ); // initialize random seed: 

  // create the input:
  int length = 2;
  int joints = 1;

  trajectory_msgs::JointTrajectory trajectory;
  std::vector<moveit_msgs::JointLimits> limits;
  trajectory.points.resize(length);
  trajectory.joint_names.resize(joints);
  trajectory.joint_names[0] = std::string("test0");

  limits.resize(joints);
  limits[0].max_velocity = 0.2;
  limits[0].has_velocity_limits = 1;

  limits[0].max_acceleration = 0.1;
  limits[0].has_acceleration_limits = 1;

  for (int i=0; i<length; i++)
  {
    trajectory.points[i].positions.resize(joints);
    trajectory.points[i].velocities.resize(joints);
    trajectory.points[i].accelerations.resize(joints);
    trajectory.points[i].time_from_start = ros::Duration(0.0);
  }
  trajectory.points[1].positions[0] = 1.0;
  trajectory.points[1].velocities[0] = 0.0;
  spline_msgs::SplineTrajectory spline;
  bool success = traj.parameterize(trajectory,limits,spline);
  EXPECT_TRUE(success);

  double total_time;
  bool ss = trajectory_processing::getTotalTime(spline,total_time);
  EXPECT_TRUE(ss);
  //  EXPECT_NEAR(total_time,12.717798,1e-5);

  ROS_INFO("Next test");
  trajectory.points[1].velocities[0] = 0.0;
  success = traj.parameterize(trajectory,limits,spline);
  EXPECT_TRUE(success);
  ss = trajectory_processing::getTotalTime(spline,total_time);
  EXPECT_TRUE(ss);
  //  EXPECT_NEAR(total_time,7.5,1e-5);

  trajectory_msgs::JointTrajectory wpt_out;
  std::vector<double> times_out;
  times_out.push_back(total_time);
  trajectory_processing::sampleSplineTrajectory(spline,times_out,wpt_out);

  //  EXPECT_NEAR(trajectory.points[1].positions[0],wpt_out.points[0].positions[0],1e-5);
  //  EXPECT_NEAR(trajectory.points[1].velocities[0],wpt_out.points[0].velocities[0],1e-5);

  trajectory.points[0].positions[0] = -0.000720;
  trajectory.points[1].positions[0] = -0.0000080;
  trajectory.points[0].velocities[0] = .000028;
  trajectory.points[1].velocities[0] = 0.000034;
  limits[0].max_velocity = 1.0;
  limits[0].max_acceleration = 0.5;

  success = traj.parameterize(trajectory,limits,spline);
  EXPECT_TRUE(success);
  ss = trajectory_processing::getTotalTime(spline,total_time);
  EXPECT_TRUE(ss);
  EXPECT_NEAR(total_time,0.092254,1e-3);

  random_numbers::RandomNumberGenerator rng;
  double eps = 1e-2;
  for(unsigned int i=0; i < 2000; i++)
  {
    trajectory.points[0].positions[0] = rng.uniformReal(-100.0,100.0);
    trajectory.points[1].positions[0] = rng.uniformReal(-100.0,100.0);
    limits[0].max_velocity = fabs(rng.uniformReal(-100.0,100.0));
    limits[0].max_acceleration = fabs(rng.uniformReal(-100.0,100.0));
    //    trajectory.points[0].velocities[0] = rng.uniformReal(-limits[0].max_velocity,limits[0].max_velocity);
    //    trajectory.points[1].velocities[0] = rng.uniformReal(-limits[0].max_velocity,limits[0].max_velocity);
    trajectory.points[0].velocities[0] = 0.0;
    trajectory.points[1].velocities[0] = 0.0;
    if(trajectory.points[0].positions[0] == trajectory.points[1].positions[0])
      continue;
    success = traj.parameterize(trajectory,limits,spline);
    EXPECT_TRUE(success);
    ss = trajectory_processing::getTotalTime(spline,total_time);
    std::vector<double> test_times;
    double dT = 0.0;
    while(dT < total_time)
    {
      test_times.push_back(dT);
      dT += 0.01;
    }
    test_times.push_back(total_time);
    trajectory_msgs::JointTrajectory test_trajectory;
    trajectory_processing::sampleSplineTrajectory(spline,test_times,test_trajectory);
    for(unsigned int i=0; i < test_trajectory.points.size(); i++)
    {
      double vel_error = fabs(test_trajectory.points[i].velocities[0]) - limits[0].max_velocity;
      double acc_error = fabs(test_trajectory.points[i].accelerations[0]) - limits[0].max_acceleration;
      if(!(vel_error <= eps) || isnan(vel_error))
      {
        ROS_INFO("error: %f %f",vel_error,acc_error);      
        ROS_INFO("positions: %f, %f, velocities: %f, %f",trajectory.points[0].positions[0],
                 trajectory.points[1].positions[0],
                 trajectory.points[0].velocities[0],
                 trajectory.points[1].velocities[0]);
        ROS_INFO("Limits: %f, %f",limits[0].max_velocity,limits[0].max_acceleration);
        ROS_INFO(" ");
        ROS_INFO(" ");
        ROS_INFO(" ");
      }
      if(!(acc_error <= eps))
        ROS_INFO("error: %f %f",vel_error,acc_error);      

      EXPECT_TRUE(vel_error <= eps);
      EXPECT_TRUE(acc_error <= eps);      
    }
  }
  
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
