/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Bianca Homberg */
#include <moveit_resources/config.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <urdf_parser/urdf_parser.h>
#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>
#include <ctype.h>

class OneRobot : public testing::Test
{
protected:
  void SetUp() override
  {
    static const std::string MODEL2 =
        "<?xml version=\"1.0\" ?>"
        "<robot name=\"one_robot\">"
        "<link name=\"base_link\">"
        "  <inertial>"
        "    <mass value=\"2.81\"/>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
        "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
        "  </inertial>"
        "  <collision name=\"my_collision\">"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "<joint name=\"joint_a\" type=\"continuous\">"
        "   <axis xyz=\"0 0 1\"/>"
        "   <parent link=\"base_link\"/>"
        "   <child link=\"link_a\"/>"
        "   <origin rpy=\" 0.0 0 0 \" xyz=\"0.0 0 0 \"/>"
        "</joint>"
        "<link name=\"link_a\">"
        "  <inertial>"
        "    <mass value=\"1.0\"/>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
        "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
        "  </inertial>"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "<joint name=\"joint_b\" type=\"fixed\">"
        "  <parent link=\"link_a\"/>"
        "  <child link=\"link_b\"/>"
        "  <origin rpy=\" 0.0 -0.42 0 \" xyz=\"0.0 0.5 0 \"/>"
        "</joint>"
        "<link name=\"link_b\">"
        "  <inertial>"
        "    <mass value=\"1.0\"/>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
        "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
        "  </inertial>"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "  <joint name=\"joint_c\" type=\"prismatic\">"
        "    <axis xyz=\"1 0 0\"/>"
        "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.09\" velocity=\"0.2\"/>"
        "    <safety_controller k_position=\"20.0\" k_velocity=\"500.0\" soft_lower_limit=\"0.0\" "
        "soft_upper_limit=\"0.089\"/>"
        "    <parent link=\"link_b\"/>"
        "    <child link=\"link_c\"/>"
        "    <origin rpy=\" 0.0 0.42 0.0 \" xyz=\"0.0 -0.1 0 \"/>"
        "  </joint>"
        "<link name=\"link_c\">"
        "  <inertial>"
        "    <mass value=\"1.0\"/>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 .0\"/>"
        "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
        "  </inertial>"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "  <joint name=\"mim_f\" type=\"prismatic\">"
        "    <axis xyz=\"1 0 0\"/>"
        "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.19\" velocity=\"0.2\"/>"
        "    <parent link=\"link_c\"/>"
        "    <child link=\"link_d\"/>"
        "    <origin rpy=\" 0.0 0.0 0.0 \" xyz=\"0.1 0.1 0 \"/>"
        "    <mimic joint=\"joint_f\" multiplier=\"1.5\" offset=\"0.1\"/>"
        "  </joint>"
        "  <joint name=\"joint_f\" type=\"prismatic\">"
        "    <axis xyz=\"1 0 0\"/>"
        "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.19\" velocity=\"0.2\"/>"
        "    <parent link=\"link_d\"/>"
        "    <child link=\"link_e\"/>"
        "    <origin rpy=\" 0.0 0.0 0.0 \" xyz=\"0.1 0.1 0 \"/>"
        "  </joint>"
        "<link name=\"link_d\">"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 1 0\" xyz=\"0 0.1 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "<link name=\"link_e\">"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 1 0\" xyz=\"0 0.1 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "</robot>";

    static const std::string SMODEL2 =
        "<?xml version=\"1.0\" ?>"
        "<robot name=\"one_robot\">"
        "<virtual_joint name=\"base_joint\" child_link=\"base_link\" parent_frame=\"odom_combined\" type=\"planar\"/>"
        "<group name=\"base_from_joints\">"
        "<joint name=\"base_joint\"/>"
        "<joint name=\"joint_a\"/>"
        "<joint name=\"joint_c\"/>"
        "</group>"
        "<group name=\"mim_joints\">"
        "<joint name=\"joint_f\"/>"
        "<joint name=\"mim_f\"/>"
        "</group>"
        "<group name=\"base_with_subgroups\">"
        "<group name=\"base_from_base_to_tip\"/>"
        "<joint name=\"joint_c\"/>"
        "</group>"
        "<group name=\"base_from_base_to_tip\">"
        "<chain base_link=\"base_link\" tip_link=\"link_b\"/>"
        "<joint name=\"base_joint\"/>"
        "</group>"
        "<group name=\"base_from_base_to_e\">"
        "<chain base_link=\"base_link\" tip_link=\"link_e\"/>"
        "<joint name=\"base_joint\"/>"
        "</group>"
        "<group name=\"base_with_bad_subgroups\">"
        "<group name=\"error\"/>"
        "</group>"
        "</robot>";

    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(MODEL2);
    srdf::ModelSharedPtr srdf_model(new srdf::Model());
    srdf_model->initString(*urdf_model, SMODEL2);
    robot_model.reset(new moveit::core::RobotModel(urdf_model, srdf_model));
  }

  void TearDown() override
  {
  }

protected:
  moveit::core::RobotModelConstPtr robot_model;
};


std::size_t generateTestTraj(robot_trajectory::RobotTrajectory& traj,
                             const moveit::core::RobotModelConstPtr& robot_model,
                             const robot_model::JointModelGroup* joint_model_group)
{
  traj.clear();

  std::shared_ptr<robot_state::RobotState> robot_state(new robot_state::RobotState(robot_model));
  robot_state->update();

  // 3 waypoints with default joints
  for (std::size_t traj_ix = 0; traj_ix < 3; ++traj_ix)
  {
    // robot_state.reset(new robot_state::RobotState(*robot_state));
    traj.addSuffixWayPoint(robot_state::RobotStatePtr(new robot_state::RobotState(*robot_state)), 1.0);
  }
  return traj.size();
}

TEST_F(OneRobot, testGenerateTrajectory)
{
  const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("base_from_base_to_e");
  robot_trajectory::RobotTrajectory traj(robot_model, joint_model_group);

  // The full trajectory should be of length 3
  const std::size_t expected_full_traj_len = 3;

  // Generate a test trajectory
  std::size_t full_traj_len = generateTestTraj(traj, robot_model, joint_model_group);

  // Test the generateTestTraj still generates a trajectory of length 3
  EXPECT_EQ(full_traj_len, expected_full_traj_len);  // full traj should be 3 waypoints long
}

TEST_F(OneRobot, testGetFirstWayPointAndAddPrefix)
{
  const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("base_from_base_to_e");
  robot_trajectory::RobotTrajectory traj(robot_model, joint_model_group);
  generateTestTraj(traj, robot_model, joint_model_group);

  robot_state::RobotState new_robot_state(robot_model);
  new_robot_state.update();
  traj.addPrefixWayPoint(new_robot_state, 1.0);
  robot_state::RobotState first_robot_state = traj.getFirstWayPoint();

  EXPECT_EQ(first_robot_state, new_robot_state);
}

TEST_F(OneRobot, testIterators)
{
  const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("base_from_base_to_e");
  robot_trajectory::RobotTrajectory traj(robot_model, joint_model_group);
  generateTestTraj(traj, robot_model, joint_model_group);

  robot_state::RobotState traj_robot_state(robot_model);
  traj_robot_state.update();

  for (auto pair : traj)
  {
    // Compare waypoints
    EXPECT_EQ(*pair.first, traj_robot_state);
    // Compare duration_from_previous
    EXPECT_EQ(pair.second, 1.0);
  }

  // Consistency checks
  EXPECT_EQ(traj.begin(), traj.begin());
  EXPECT_EQ(traj.end(), traj.end());

  // traj has length 3; incrementing begin 3 times should reach the end
  EXPECT_NE(traj.begin(), traj.end());
  EXPECT_NE(++traj.begin(), traj.end());
  EXPECT_NE(++(++traj.begin()), traj.end());
  EXPECT_EQ(++(++(++traj.begin())), traj.end());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
