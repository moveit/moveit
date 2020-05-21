/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Dave Coleman
   Desc:   Stub test for the C++ interface to JogArm
*/

// C++
#include <string>

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// Main class
#include <moveit_jog_arm/jog_cpp_interface.h>

static const std::string LOGNAME = "jog_cpp_interface_test";

namespace moveit_jog_arm
{
class TestJogCppInterface : public ::testing::Test
{
public:
  void SetUp() override
  {
    nh_.reset(new ros::NodeHandle("~"));

    // Load the planning scene monitor
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor();
  }
  void TearDown() override
  {
  }

protected:
  std::unique_ptr<ros::NodeHandle> nh_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
};  // class TestJogCppInterface

TEST_F(TestJogCppInterface, InitTest)
{
  moveit_jog_arm::JogCppInterface jog_cpp_interface(planning_scene_monitor_);
  ros::Duration(1).sleep();  // Give the started thread some time to run
}

// TODO(davetcoleman): due to many blocking checks for ROS messages, and
// an abundance of threads, unit tests are not currently feasible for the
// cpp interface. This should be addressed in future re-factors

}  // namespace moveit_jog_arm

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "jog_cpp_interface_test_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();

  spinner.stop();
  ros::shutdown();
  return result;
}
