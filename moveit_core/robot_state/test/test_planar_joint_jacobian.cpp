/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC.
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
 *   * Neither the name of the PickNik nor the names of its
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

/* Author: Ivo Vatavuk */

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/utils/eigen_test_utils.h>

using namespace moveit::core;

class SimplePlanarRobot : public testing::Test
{
protected:
  void SetUp() override
  {
    RobotModelBuilder builder("simple", "a");
    builder.addChain("a->b", "planar");
    builder.addGroupChain("a", "b", "group");
    robot_model_ = builder.build();
    robot_state_ = std::make_shared<RobotState>(robot_model_);
  }

  void TearDown() override
  {
  }

protected:
  RobotModelPtr robot_model_;
  RobotStatePtr robot_state_;
};

TEST_F(SimplePlanarRobot, testSimplePlanarRobot)
{
  std::cout << "Testing simple planar robot jacobian\n";

  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  auto joint_model_group = robot_model_->getJointModelGroup("group");

  std::vector<double> q_test{ 0.0, 0.0, 0.0 };
  //-----------------------Set robot state-----------------------
  robot_state_->setJointGroupPositions(joint_model_group, q_test);
  robot_state_->updateLinkTransforms();

  //-----------------------Calculate Jacobian-----------------------
  Eigen::MatrixXd jacobian;
  robot_state_->getJacobian(joint_model_group, robot_state_->getLinkModel("b"), reference_point_position, jacobian);

  //-----------------------Move first axis to 10 m-----------------------
  q_test[0] = 10.0;
  //-----------------------Set robot state-----------------------
  robot_state_->setJointGroupPositions(joint_model_group, q_test);
  robot_state_->updateLinkTransforms();

  //-----------------------Calculate Jacobian-----------------------
  Eigen::MatrixXd jacobian_2;
  robot_state_->getJacobian(joint_model_group, robot_state_->getLinkModel("b"), reference_point_position, jacobian_2);

  std::cout << "First Jacobian = \n" << jacobian << "\n";
  std::cout << "Second Jacobian = \n" << jacobian_2 << "\n";

  EXPECT_EIGEN_NEAR(jacobian, jacobian_2, 1e-5);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
