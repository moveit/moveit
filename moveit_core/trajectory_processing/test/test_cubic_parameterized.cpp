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

#include <ros/ros.h>
#include <moveit_msgs/JointTrajectoryWithLimits.h>
#include <spline_smoother/cubic_parameterized_trajectory.h>

double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}


//TEST(TestCubicParameterizedTrajectory, TestWithAccelerationLimits2)
void run_test(int num_times)
{
  spline_smoother::CubicParameterizedTrajectory traj;
  srand ( time(NULL) ); // initialize random seed: 

  bool success, ss;
  double total_time;

  // create the input:
  int length = 2;
  int joints = 1;

  moveit_msgs::JointTrajectoryWithLimits wpt;
  wpt.trajectory.points.resize(length);
  wpt.trajectory.joint_names.resize(joints);
  wpt.trajectory.joint_names[0] = std::string("test0");

  wpt.limits.resize(joints);
  wpt.limits[0].has_velocity_limits = 1;
  wpt.limits[0].has_acceleration_limits = 1;
  wpt.limits[0].max_velocity = 1.0;
  wpt.limits[0].max_acceleration = 0.5;

  for (int i=0; i<length; i++)
  {
    wpt.trajectory.points[i].positions.resize(joints);
    wpt.trajectory.points[i].velocities.resize(joints);
    wpt.trajectory.points[i].accelerations.resize(joints);
    wpt.trajectory.points[i].time_from_start = ros::Duration(0.0);
  }
  spline_smoother::SplineTrajectory spline;
  double eps = 1e-1;
  for(unsigned int i=0; i < num_times; i++)
  {
    //24.752475, -66.336634, velocities: 0.000000, 0.000000
    //[ INFO] [WallTime: 1285288331.834028213]: Limits: 14.851485, 6.930693
    wpt.trajectory.points[0].positions[0] = -24.752475;
    wpt.trajectory.points[1].positions[0] = 66.336634;
    wpt.limits[0].max_velocity = 14.851485;
    wpt.limits[0].max_acceleration =  6.930693;
    wpt.trajectory.points[0].velocities[0] = 0.0;
    wpt.trajectory.points[1].velocities[0] = 0.0;
    if(wpt.trajectory.points[0].positions[0] == wpt.trajectory.points[1].positions[0])
      continue;
    success = traj.parameterize(wpt.trajectory,wpt.limits,spline);
  
    ss = spline_smoother::getTotalTime(spline,total_time);
    std::vector<double> test_times;
    double dT = 0.0;
    while(dT < total_time)
    {
      test_times.push_back(dT);
      dT += 0.01;
    }
    test_times.push_back(total_time);
    trajectory_msgs::JointTrajectory test_trajectory;
    spline_smoother::sampleSplineTrajectory(spline,test_times,test_trajectory);
    
    for(unsigned int i=0; i < test_trajectory.points.size(); i++)
    {
      double vel_error = fabs(test_trajectory.points[i].velocities[0]) - wpt.limits[0].max_velocity;
      double acc_error = fabs(test_trajectory.points[i].accelerations[0]) - wpt.limits[0].max_acceleration;
      if(!(vel_error <= eps) || isnan(vel_error))
      {
        ROS_INFO("Time: %f",test_trajectory.points[i].time_from_start.toSec());
        ROS_INFO("Actual: %f %f",test_trajectory.points[i].velocities[0],test_trajectory.points[i].accelerations[0]);
        ROS_INFO("error: %f %f",vel_error,acc_error);      
        ROS_INFO("positions: %f, %f, velocities: %f, %f",wpt.trajectory.points[0].positions[0],
                 wpt.trajectory.points[1].positions[0],
                 wpt.trajectory.points[0].velocities[0],
                 wpt.trajectory.points[1].velocities[0]);
        ROS_INFO("Limits: %f, %f",wpt.limits[0].max_velocity,wpt.limits[0].max_acceleration);
        ROS_INFO(" ");
        ROS_INFO(" ");
        ROS_INFO(" ");
        break;
      }
      if(!(acc_error <= eps))
      {
        ROS_INFO("Time: %f",test_trajectory.points[i].time_from_start.toSec());
        ROS_INFO("error: %f %f",vel_error,acc_error);      
        ROS_INFO(" ");
        ROS_INFO(" ");
        ROS_INFO(" ");
        break;
      }
    }
  }
  
}


int main(int argc, char** argv)
{
  int num_times = 10;
  if(argc > 1)
    num_times = atoi(argv[1]);
  run_test(num_times);
  //  testing::InitGoogleTest(&argc, argv);
  //  return RUN_ALL_TESTS();
}
