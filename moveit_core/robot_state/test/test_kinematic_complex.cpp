/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/** Author E. Gil Jones, Ioan Sucan */

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <gtest/gtest.h>

class LoadPlanningModelsPr2 : public testing::Test
{
protected:

    virtual void SetUp()
    {
        srdf_model_.reset(new srdf::Model());

        std::string xml_string;
        std::fstream xml_file("test/urdf/robot.xml", std::fstream::in);
        if (xml_file.is_open())
        {
          while ( xml_file.good() )
          {
            std::string line;
            std::getline( xml_file, line);
            xml_string += (line + "\n");
          }
          xml_file.close();
          urdf_model_ = urdf::parseURDF(xml_string);
          urdf_ok_ = urdf_model_;
        }
        else
          urdf_ok_ = false;
        srdf_ok_ = srdf_model_->initFile(*urdf_model_, "test/srdf/robot.xml");
    };

    virtual void TearDown()
    {
    }

protected:

    boost::shared_ptr<urdf::ModelInterface> urdf_model_;
    boost::shared_ptr<srdf::Model>          srdf_model_;
    bool                                    urdf_ok_;
    bool                                    srdf_ok_;

};

TEST_F(LoadPlanningModelsPr2, InitOK)
{
    ASSERT_TRUE(urdf_ok_);
    ASSERT_TRUE(srdf_ok_);
    ASSERT_EQ(urdf_model_->getName(), "pr2_test");
    ASSERT_EQ(srdf_model_->getName(), "pr2_test");
}

TEST_F(LoadPlanningModelsPr2, MultidofInit)
{
    boost::shared_ptr<srdf::Model> srdfModel(new srdf::Model());

    // with no world multidof we should get a fixed joint
    robot_model::RobotModel kin_model0(urdf_model_, srdfModel);
    EXPECT_TRUE(kin_model0.getRoot()->getVariableCount() == 0);

    static const std::string SMODEL1 =
        "<?xml version=\"1.0\" ?>"
        "<robot name=\"pr2_test\">"
        "<virtual_joint name=\"base_joint\" child_link=\"base_footprint\" parent_frame=\"base_footprint\" type=\"planar\"/>"
        "</robot>";
    srdfModel->initString(*urdf_model_, SMODEL1);

    robot_model::RobotModel kin_model1(urdf_model_, srdfModel);
    ASSERT_TRUE(kin_model1.getRoot() != NULL);
    EXPECT_EQ(kin_model1.getModelFrame(), "base_footprint");

    static const std::string SMODEL2 =
        "<?xml version=\"1.0\" ?>"
        "<robot name=\"pr2_test\">"
        "<virtual_joint name=\"world_joint\" child_link=\"base_footprint\" parent_frame=\"odom_combined\" type=\"floating\"/>"
        "</robot>";
    srdfModel->initString(*urdf_model_, SMODEL2);

    robot_model::RobotModel kin_model2(urdf_model_, srdfModel);
    ASSERT_TRUE(kin_model2.getRoot() != NULL);
    EXPECT_EQ(kin_model2.getModelFrame(), "odom_combined");
}

TEST_F(LoadPlanningModelsPr2, GroupInit)
{
    static const std::string SMODEL1 =
        "<?xml version=\"1.0\" ?>"
        "<robot name=\"pr2_test\">"
        "<virtual_joint name=\"base_joint\" child_link=\"base_footprint\" parent_frame=\"base_footprint\" type=\"planar\"/>"
        "<group name=\"left_arm_base_tip\">"
        "<chain base_link=\"monkey_base\" tip_link=\"monkey_tip\"/>"
        "</group>"
        "<group name=\"left_arm_joints\">"
        "<joint name=\"l_monkey_pan_joint\"/>"
        "<joint name=\"l_monkey_fles_joint\"/>"
        "</group>"
        "</robot>";

    boost::shared_ptr<srdf::Model> srdfModel(new srdf::Model());
    srdfModel->initString(*urdf_model_, SMODEL1);
    robot_model::RobotModel kin_model1(urdf_model_, srdfModel);

    const robot_model::JointModelGroup* left_arm_base_tip_group = kin_model1.getJointModelGroup("left_arm_base_tip");
    ASSERT_TRUE(left_arm_base_tip_group == NULL);

    const robot_model::JointModelGroup* left_arm_joints_group = kin_model1.getJointModelGroup("left_arm_joints");
    ASSERT_TRUE(left_arm_joints_group == NULL);

    static const std::string SMODEL2 =
        "<?xml version=\"1.0\" ?>"
        "<robot name=\"pr2_test\">"
        "<virtual_joint name=\"base_joint\" child_link=\"base_footprint\" parent_frame=\"base_footprint\" type=\"planar\"/>"
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
    srdfModel->initString(*urdf_model_, SMODEL2);

    robot_model::RobotModelPtr kin_model2(new robot_model::RobotModel(urdf_model_, srdfModel));

    left_arm_base_tip_group = kin_model2->getJointModelGroup("left_arm_base_tip");
    ASSERT_TRUE(left_arm_base_tip_group != NULL);

    left_arm_joints_group = kin_model2->getJointModelGroup("left_arm_joints");
    ASSERT_TRUE(left_arm_joints_group != NULL);

    EXPECT_EQ(left_arm_base_tip_group->getJointModels().size(), 7);
    EXPECT_EQ(left_arm_joints_group->getJointModels().size(), 7);

    EXPECT_EQ(left_arm_joints_group->getVariableNames().size(), left_arm_joints_group->getVariableCount());
    EXPECT_EQ(left_arm_joints_group->getVariableCount(), 7);

    EXPECT_EQ(kin_model2->getVariableNames().size(), kin_model2->getVariableCount());

    bool found_shoulder_pan_link = false;
    bool found_wrist_roll_link = false;
    for(unsigned int i = 0; i < left_arm_base_tip_group->getLinkModels().size(); i++)
    {
        if (left_arm_base_tip_group->getLinkModels()[i]->getName() == "l_shoulder_pan_link")
        {
            EXPECT_TRUE(found_shoulder_pan_link == false);
            found_shoulder_pan_link = true;
        }
        if (left_arm_base_tip_group->getLinkModels()[i]->getName() == "l_wrist_roll_link")
        {
            EXPECT_TRUE(found_wrist_roll_link == false);
            found_wrist_roll_link = true;
        }
        EXPECT_TRUE(left_arm_base_tip_group->getLinkModels()[i]->getName() != "torso_lift_link");
    }
    EXPECT_TRUE(found_shoulder_pan_link);
    EXPECT_TRUE(found_wrist_roll_link);


    robot_state::RobotState ks(kin_model2);
    ks.setToDefaultValues();
    std::map<std::string, double> jv;
    jv["base_joint.x"] = 0.433;
    jv["base_joint.theta"] = -0.5;
    ks.setStateValues(jv);
    moveit_msgs::RobotState robot_state;
    robot_state::robotStateToRobotStateMsg(ks, robot_state);

    robot_state::RobotState ks2(kin_model2);
    robotStateMsgToRobotState(robot_state, ks2);
    std::vector<double> v1;
    ks.getStateValues(v1);
    std::vector<double> v2;
    ks2.getStateValues(v2);
    EXPECT_TRUE(v1.size() == v2.size());
    for (unsigned int i = 0; i < v1.size(); ++i)
        EXPECT_NEAR(v1[i], v2[i], 1e-5);

    std::vector<double> state_double_vector;
    ks.getStateValues(state_double_vector);
    ASSERT_TRUE(ks.setStateValues(state_double_vector));
}

TEST_F(LoadPlanningModelsPr2, SubgroupInit)
{
  robot_model::RobotModel kmodel(urdf_model_, srdf_model_);
  const robot_model::JointModelGroup* jmg = kmodel.getJointModelGroup("arms");
  ASSERT_TRUE(jmg);
  EXPECT_EQ(jmg->getSubgroupNames().size(), 2);
  EXPECT_TRUE(jmg->isSubgroup("right_arm"));

  const robot_model::JointModelGroup* jmg2 = kmodel.getJointModelGroup("whole_body");
  EXPECT_EQ(jmg2->getSubgroupNames().size(), 4);
  EXPECT_TRUE(jmg2->isSubgroup("arms"));
  EXPECT_TRUE(jmg2->isSubgroup("right_arm"));
}

TEST_F(LoadPlanningModelsPr2, AssociatedFixedLinks)
{
  boost::shared_ptr<robot_model::RobotModel> kmodel(new robot_model::RobotModel(urdf_model_, srdf_model_));

  EXPECT_TRUE(kmodel->getLinkModel("r_gripper_palm_link")->getAssociatedFixedTransforms().size() > 1);
}

//TEST_F(LoadPlanningModelsPr2, robot_state::RobotState *Copy)
TEST_F(LoadPlanningModelsPr2, FullTest)
{
  robot_model::RobotModelPtr kmodel(new robot_model::RobotModel(urdf_model_, srdf_model_));

  robot_state::RobotState ks(kmodel);
  ks.setToDefaultValues();

  robot_state::RobotState ks2(kmodel);
  ks2.setToDefaultValues();

  std::vector<shapes::ShapeConstPtr> shapes;
  EigenSTL::vector_Affine3d poses;
  shapes::Shape* shape = new shapes::Box(.1,.1,.1);
  shapes.push_back(shapes::ShapeConstPtr(shape));
  poses.push_back(Eigen::Affine3d::Identity());
  std::set<std::string> touch_links;

  sensor_msgs::JointState empty_state;
  robot_state::AttachedBody attached_body(ks.getLinkState("r_gripper_palm_link")->getLinkModel(), "box", shapes, poses, touch_links, empty_state);

  ks.attachBody(&attached_body);

  std::vector<const robot_state::AttachedBody*> attached_bodies_1;
  ks.getAttachedBodies(attached_bodies_1);
  ASSERT_EQ(attached_bodies_1.size(),1);

  std::vector<const robot_state::AttachedBody*> attached_bodies_2;
  ks2 = ks;
  ks2.getAttachedBodies(attached_bodies_2);
  ASSERT_EQ(attached_bodies_2.size(),1);

  ks.clearAttachedBody("box");
  attached_bodies_1.clear();
  ks.getAttachedBodies(attached_bodies_1);
  ASSERT_EQ(attached_bodies_1.size(),0);

  ks2 = ks;
  attached_bodies_2.clear();
  ks2.getAttachedBodies(attached_bodies_2);
  ASSERT_EQ(attached_bodies_2.size(),0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
