/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, CITEC Bielefeld
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

/* Author: Robert Haschke */
#include <moveit_resources/config.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <gtest/gtest.h>
#include <ctime>

class PR2 : public testing::Test
{
protected:
  void SetUp() override
  {
    model_ = moveit::core::loadTestingRobotModel("pr2_description");
  };

  void TearDown() override
  {
  }

  robot_model::RobotModelPtr model_;
};

TEST_F(PR2, StateUpdateTiming)
{
  ASSERT_TRUE(bool(model_));
  robot_state::RobotState state(model_);
  clock_t begin = clock();
  for (unsigned i = 0; i < 10000; ++i)
  {
    state.setToRandomPositions();
    state.update();
  }
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cerr << "time for 10,000 random RobotState updates: " << elapsed_secs << "s" << std::endl;
}

TEST(EigenTransform, Timing)
{
  Eigen::Isometry3d iso = Eigen::Translation3d(1, 2, 3) * Eigen::AngleAxisd(0.13 * M_PI, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(0.29 * M_PI, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(0.42 * M_PI, Eigen::Vector3d::UnitZ());
  Eigen::Affine3d affine(iso.matrix());
  Eigen::Matrix4d result;
  Eigen::Isometry3d id = Eigen::Isometry3d::Identity();

  size_t runs = 1e6;
  clock_t begin = clock();
  for (size_t i = 0; i < runs; ++i)
    EXPECT_TRUE((iso.inverse() * iso).isApprox(id));
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cerr << "Isometry3d::inverse(): " << elapsed_secs << "s" << std::endl;

  begin = clock();
  for (size_t i = 0; i < runs; ++i)
    EXPECT_TRUE((affine.inverse(Eigen::Isometry) * affine).isApprox(id));
  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cerr << "Affine3d::inverse(Eigen::Isometry): " << elapsed_secs << "s" << std::endl;

  begin = clock();
  for (size_t i = 0; i < runs; ++i)
    EXPECT_TRUE((affine.inverse() * affine).isApprox(id));
  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cerr << "Affine3d::inverse(): " << elapsed_secs << "s" << std::endl;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
