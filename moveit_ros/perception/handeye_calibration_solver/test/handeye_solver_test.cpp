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

#include <fstream>
#include <ros/package.h>
#include <gtest/gtest.h>
#include <jsoncpp/json/json.h>
#include <pluginlib/class_loader.hpp>
#include <moveit/handeye_calibration_solver/handeye_solver_base.h>

class MoveItHandEyeSolverTester : public ::testing::Test
{
protected:
  void SetUp() override
  {
    solver_ok_ = false;
    try
    {
      solver_plugins_loader_.reset(new pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeSolverBase>(
          "moveit_ros_perception", "moveit_handeye_calibration::HandEyeSolverBase"));
      solver_ = solver_plugins_loader_->createUniqueInstance("crigroup");
      solver_->initialize();
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Exception while creating handeye target plugin: " << ex.what());
      return;
    }

    Json::Reader reader;
    std::string moveit_ros_perception_package_path = ros::package::getPath("moveit_ros_perception");
    moveit_ros_perception_package_path += "/handeye_calibration_solver/test/pose_samples.json";
    std::ifstream ifs(moveit_ros_perception_package_path);

    if (ifs)
    {
      if (reader.parse(ifs, root_))
      {
        solver_ok_ = true;
      }
      else
        ROS_ERROR_STREAM("Can't parse json file: ./pose_samples.json");
    }
    else
      ROS_ERROR_STREAM("Can't load file: ./pose_samples.json");
  }

  void TearDown() override
  {
  }

protected:
  pluginlib::UniquePtr<moveit_handeye_calibration::HandEyeSolverBase> solver_;
  std::unique_ptr<pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeSolverBase> > solver_plugins_loader_;
  Json::Value root_;
  bool solver_ok_;
};

TEST_F(MoveItHandEyeSolverTester, InitOK)
{
  ASSERT_TRUE(solver_);
  ASSERT_TRUE(solver_ok_);
  ASSERT_EQ(root_.size(), 50);
}

TEST_F(MoveItHandEyeSolverTester, GetSolverNames)
{
  const std::vector<std::string>& solver_names = solver_->getSolverNames();
  ASSERT_EQ(solver_names.size(), 3);
  ASSERT_EQ(solver_names[0], "Daniilidis1999");
  ASSERT_EQ(solver_names[1], "ParkBryan1994");
  ASSERT_EQ(solver_names[2], "TsaiLenz1989");
}

TEST_F(MoveItHandEyeSolverTester, SolveAXEQXB)
{
  std::vector<Eigen::Isometry3d> eef_wrt_world(root_.size(), Eigen::Isometry3d::Identity());
  std::vector<Eigen::Isometry3d> obj_wrt_sensor(root_.size(), Eigen::Isometry3d::Identity());

  for (int i = 0; i < root_.size(); ++i)
  {
    Json::Value json_eef_wrt_world = root_[i][0];
    ASSERT_EQ(json_eef_wrt_world.size(), 4);
    for (int m = 0; m < json_eef_wrt_world.size(); ++m)
    {
      ASSERT_EQ(json_eef_wrt_world[m].size(), 4);
      for (int n = 0; n < json_eef_wrt_world[m].size(); ++n)
        eef_wrt_world[i](m, n) = json_eef_wrt_world[m][n].asDouble();
    }

    Json::Value json_obj_wrt_sensor = root_[i][1];
    ASSERT_EQ(json_obj_wrt_sensor.size(), 4);
    for (int m = 0; m < json_obj_wrt_sensor.size(); ++m)
    {
      ASSERT_EQ(json_obj_wrt_sensor[m].size(), 4);
      for (int n = 0; n < json_obj_wrt_sensor[m].size(); ++n)
        obj_wrt_sensor[i](m, n) = json_obj_wrt_sensor[m][n].asDouble();
    }
  }

  const std::vector<std::string>& solver_names = solver_->getSolverNames();
  for (const std::string& name : solver_names)
  {
    // Fake test for EYE_IN_HAND to check if segfault happens when loaded multiple times
    bool res = solver_->solve(eef_wrt_world, obj_wrt_sensor, moveit_handeye_calibration::EYE_IN_HAND, name);
    ASSERT_TRUE(res);
    // Test EYE_TO_HAND from real data
    res = solver_->solve(eef_wrt_world, obj_wrt_sensor, moveit_handeye_calibration::EYE_TO_HAND, name);
    ASSERT_TRUE(res);
    Eigen::Vector3d t(0.659, -0.249, 0.830);
    Eigen::Vector3d r(0.230, -2.540, 1.950);
    Eigen::Isometry3d ret = solver_->getCameraRobotPose();
    ASSERT_TRUE(ret.translation().isApprox(t, 0.01));
    ASSERT_TRUE(ret.rotation().eulerAngles(0, 1, 2).isApprox(r, 0.01));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
