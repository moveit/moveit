/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2019, PickNik Consulting
 *  All rights reserved.
 *
 *********************************************************************/

/* Author: Omid Heidari, Dave Coleman
   Desc:   Test the trajectory for various numbers of timesteps
 */

// C++
#include <iostream>
#include <string>

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

namespace trackpose_cpp
{
class TrajectoryTest : public ::testing::Test
{
public:
  TrajectoryTest()
  {
  }

  const int IS_SUCCESSFUL = 0;

protected:

  bool is_successful_;

  double max_duration_ = 1;  // seconds
};   // class TrajectoryTest

TEST_F(TrajectoryTest, OneTimestep)
{
  double timestep = 0.05;
  double desired_duration = 1 * timestep;

  EXPECT_EQ(trackpose_.run(timestep, desired_duration, max_duration_, current_state_input, goal_state_input,
                           default_cartesian_limits_, output_trajectory_),
            IS_SUCCESSFUL);

}


}  // namespace trackpose_cpp

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "x_timesteptest");

  int result = RUN_ALL_TESTS();

  ros::shutdown();

  return result;
}
