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

/* Author: Ioan Sucan */

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>
#include <geometric_shapes/shapes.h>
#include <moveit/profiler/profiler.h>
#include <moveit_resources/config.h>

class LoadPlanningModelsPr2 : public testing::Test
{
protected:
  void SetUp() override
  {
    boost::filesystem::path res_path(MOVEIT_TEST_RESOURCES_DIR);

    srdf_model.reset(new srdf::Model());
    std::string xml_string;
    std::fstream xml_file((res_path / "pr2_description/urdf/robot.xml").string().c_str(), std::fstream::in);
    if (xml_file.is_open())
    {
      while (xml_file.good())
      {
        std::string line;
        std::getline(xml_file, line);
        xml_string += (line + "\n");
      }
      xml_file.close();
      urdf_model = urdf::parseURDF(xml_string);
    }
    srdf_model->initFile(*urdf_model, (res_path / "pr2_description/srdf/robot.xml").string());
    robot_model.reset(new moveit::core::RobotModel(urdf_model, srdf_model));
  };

  void TearDown() override
  {
  }

protected:
  urdf::ModelInterfaceSharedPtr urdf_model;
  srdf::ModelSharedPtr srdf_model;
  moveit::core::RobotModelConstPtr robot_model;
};

TEST_F(LoadPlanningModelsPr2, InitOK)
{
  ASSERT_EQ(urdf_model->getName(), "pr2");
  ASSERT_EQ(srdf_model->getName(), "pr2");
}

TEST_F(LoadPlanningModelsPr2, ModelInit)
{
  srdf::ModelSharedPtr srdfModel(new srdf::Model());

  // with no world multidof we should get a fixed joint
  moveit::core::RobotModel robot_model0(urdf_model, srdfModel);
  EXPECT_TRUE(robot_model0.getRootJoint()->getVariableCount() == 0);

  static const std::string SMODEL1 = "<?xml version=\"1.0\" ?>"
                                     "<robot name=\"pr2\">"
                                     "<virtual_joint name=\"base_joint\" child_link=\"base_footprint\" "
                                     "parent_frame=\"base_footprint\" type=\"planar\"/>"
                                     "</robot>";
  srdfModel->initString(*urdf_model, SMODEL1);

  moveit::core::RobotModel robot_model1(urdf_model, srdfModel);
  ASSERT_TRUE(robot_model1.getRootJoint() != nullptr);
  EXPECT_EQ(robot_model1.getModelFrame(), "/base_footprint");

  static const std::string SMODEL2 = "<?xml version=\"1.0\" ?>"
                                     "<robot name=\"pr2\">"
                                     "<virtual_joint name=\"world_joint\" child_link=\"base_footprint\" "
                                     "parent_frame=\"odom_combined\" type=\"floating\"/>"
                                     "</robot>";
  srdfModel->initString(*urdf_model, SMODEL2);

  moveit::core::RobotModel robot_model2(urdf_model, srdfModel);
  ASSERT_TRUE(robot_model2.getRootJoint() != nullptr);
  EXPECT_EQ(robot_model2.getModelFrame(), "/odom_combined");
}

TEST_F(LoadPlanningModelsPr2, GroupInit)
{
  static const std::string SMODEL1 = "<?xml version=\"1.0\" ?>"
                                     "<robot name=\"pr2\">"
                                     "<virtual_joint name=\"base_joint\" child_link=\"base_footprint\" "
                                     "parent_frame=\"base_footprint\" type=\"planar\"/>"
                                     "<group name=\"left_arm_base_tip\">"
                                     "<chain base_link=\"monkey_base\" tip_link=\"monkey_tip\"/>"
                                     "</group>"
                                     "<group name=\"left_arm_joints\">"
                                     "<joint name=\"l_monkey_pan_joint\"/>"
                                     "<joint name=\"l_monkey_fles_joint\"/>"
                                     "</group>"
                                     "</robot>";

  srdf::ModelSharedPtr srdfModel(new srdf::Model());
  srdfModel->initString(*urdf_model, SMODEL1);
  moveit::core::RobotModel robot_model1(urdf_model, srdfModel);

  const moveit::core::JointModelGroup* left_arm_base_tip_group = robot_model1.getJointModelGroup("left_arm_base_tip");
  ASSERT_TRUE(left_arm_base_tip_group == nullptr);

  const moveit::core::JointModelGroup* left_arm_joints_group = robot_model1.getJointModelGroup("left_arm_joints");
  ASSERT_TRUE(left_arm_joints_group == nullptr);

  static const std::string SMODEL2 = "<?xml version=\"1.0\" ?>"
                                     "<robot name=\"pr2\">"
                                     "<virtual_joint name=\"base_joint\" child_link=\"base_footprint\" "
                                     "parent_frame=\"base_footprint\" type=\"planar\"/>"
                                     "<group name=\"left_arm_base_tip\">"
                                     "<chain base_link=\"torso_lift_link\" tip_link=\"l_wrist_roll_link\"/>"
                                     "</group>"
                                     "<group name=\"left_arm_joints\">"
                                     "<joint name=\"l_shoulder_pan_joint\"/>"
                                     "<joint name=\"l_shoulder_lift_joint\"/>"
                                     "<joint name=\"l_upper_arm_roll_joint\"/>"
                                     "<joint name=\"l_elbow_flex_joint\"/>"
                                     "<joint name=\"l_forearm_roll_joint\"/>"
                                     "<joint name=\"l_wrist_flex_joint\"/>"
                                     "<joint name=\"l_wrist_roll_joint\"/>"
                                     "</group>"
                                     "</robot>";
  srdfModel->initString(*urdf_model, SMODEL2);

  moveit::core::RobotModelPtr robot_model2(new moveit::core::RobotModel(urdf_model, srdfModel));

  left_arm_base_tip_group = robot_model2->getJointModelGroup("left_arm_base_tip");
  ASSERT_TRUE(left_arm_base_tip_group != nullptr);

  left_arm_joints_group = robot_model2->getJointModelGroup("left_arm_joints");
  ASSERT_TRUE(left_arm_joints_group != nullptr);

  EXPECT_EQ(left_arm_base_tip_group->getJointModels().size(), 9);
  EXPECT_EQ(left_arm_joints_group->getJointModels().size(), 7);

  EXPECT_EQ(left_arm_joints_group->getVariableNames().size(), left_arm_joints_group->getVariableCount());
  EXPECT_EQ(left_arm_joints_group->getVariableCount(), 7);

  EXPECT_EQ(robot_model2->getVariableNames().size(), robot_model2->getVariableCount());

  bool found_shoulder_pan_link = false;
  bool found_wrist_roll_link = false;
  for (unsigned int i = 0; i < left_arm_base_tip_group->getLinkModels().size(); i++)
  {
    if (left_arm_base_tip_group->getLinkModels()[i]->getName() == "l_shoulder_pan_link")
    {
      EXPECT_TRUE(!found_shoulder_pan_link);
      found_shoulder_pan_link = true;
    }
    if (left_arm_base_tip_group->getLinkModels()[i]->getName() == "l_wrist_roll_link")
    {
      EXPECT_TRUE(!found_wrist_roll_link);
      found_wrist_roll_link = true;
    }
    EXPECT_TRUE(left_arm_base_tip_group->getLinkModels()[i]->getName() != "torso_lift_link");
  }
  EXPECT_TRUE(found_shoulder_pan_link);
  EXPECT_TRUE(found_wrist_roll_link);

  moveit::core::RobotState ks(robot_model2);
  ks.setToDefaultValues();
  std::map<std::string, double> jv;
  jv["base_joint/x"] = 0.433;
  jv["base_joint/theta"] = -0.5;
  ks.setVariablePositions(jv);
  moveit_msgs::RobotState robot_state;
  moveit::core::robotStateToRobotStateMsg(ks, robot_state);

  moveit::core::RobotState ks2(robot_model2);
  moveit::core::robotStateMsgToRobotState(robot_state, ks2);

  const double* v1 = ks.getVariablePositions();
  const double* v2 = ks2.getVariablePositions();
  for (std::size_t i = 0; i < ks.getVariableCount(); ++i)
    EXPECT_NEAR(v1[i], v2[i], 1e-5);
}

TEST_F(LoadPlanningModelsPr2, SubgroupInit)
{
  moveit::core::RobotModel robot_model(urdf_model, srdf_model);
  const moveit::core::JointModelGroup* jmg = robot_model.getJointModelGroup("arms");
  ASSERT_TRUE(jmg);
  EXPECT_EQ(jmg->getSubgroupNames().size(), 2);
  EXPECT_TRUE(jmg->isSubgroup("right_arm"));

  const moveit::core::JointModelGroup* jmg2 = robot_model.getJointModelGroup("whole_body");
  EXPECT_EQ(jmg2->getSubgroupNames().size(), 5);
  EXPECT_TRUE(jmg2->isSubgroup("arms"));
  EXPECT_TRUE(jmg2->isSubgroup("right_arm"));
}

TEST_F(LoadPlanningModelsPr2, AssociatedFixedLinks)
{
  moveit::core::RobotModelPtr model(new moveit::core::RobotModel(urdf_model, srdf_model));
  EXPECT_TRUE(model->getLinkModel("r_gripper_palm_link")->getAssociatedFixedTransforms().size() > 1);
}

TEST_F(LoadPlanningModelsPr2, FullTest)
{
  moveit::core::RobotModelPtr robot_model(new moveit::core::RobotModel(urdf_model, srdf_model));

  moveit::core::RobotState ks(robot_model);
  ks.setToDefaultValues();

  moveit::core::RobotState ks2(robot_model);
  ks2.setToDefaultValues();

  std::vector<shapes::ShapeConstPtr> shapes;
  EigenSTL::vector_Affine3d poses;
  shapes::Shape* shape = new shapes::Box(.1, .1, .1);
  shapes.push_back(shapes::ShapeConstPtr(shape));
  poses.push_back(Eigen::Affine3d::Identity());
  std::set<std::string> touch_links;

  trajectory_msgs::JointTrajectory empty_state;
  moveit::core::AttachedBody* attached_body = new moveit::core::AttachedBody(
      robot_model->getLinkModel("r_gripper_palm_link"), "box", shapes, poses, touch_links, empty_state);
  ks.attachBody(attached_body);

  std::vector<const moveit::core::AttachedBody*> attached_bodies_1;
  ks.getAttachedBodies(attached_bodies_1);
  ASSERT_EQ(attached_bodies_1.size(), 1);

  std::vector<const moveit::core::AttachedBody*> attached_bodies_2;
  ks2 = ks;
  ks2.getAttachedBodies(attached_bodies_2);
  ASSERT_EQ(attached_bodies_2.size(), 1);

  ks.clearAttachedBody("box");
  attached_bodies_1.clear();
  ks.getAttachedBodies(attached_bodies_1);
  ASSERT_EQ(attached_bodies_1.size(), 0);

  ks2 = ks;
  attached_bodies_2.clear();
  ks2.getAttachedBodies(attached_bodies_2);
  ASSERT_EQ(attached_bodies_2.size(), 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
