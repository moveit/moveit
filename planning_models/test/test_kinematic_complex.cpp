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

/** \author E. Gil Jones, Ioan Sucan */

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_models/conversions.h>
#include <planning_models/semantic_model.h>
#include <gtest/gtest.h>

class LoadPlanningModelsPr2 : public testing::Test
{
protected:

    virtual void SetUp()
    {
        urdf_model_.reset(new urdf::Model());
        srdf_model_.reset(new srdf::Model());
        urdf_ok_ = urdf_model_->initFile("test/urdf/robot.xml");
        srdf_ok_ = srdf_model_->initFile(*urdf_model_, "test/srdf/robot.xml");
    };

    virtual void TearDown()
    {
    }

protected:

    boost::shared_ptr<urdf::Model> urdf_model_;
    boost::shared_ptr<srdf::Model> srdf_model_;
    bool                           urdf_ok_;
    bool                           srdf_ok_;

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
    planning_models::KinematicModel kin_model0(urdf_model_, srdfModel);
    EXPECT_TRUE(kin_model0.getRoot()->getVariableCount() == 0);

    static const std::string SMODEL1 =
        "<?xml version=\"1.0\" ?>"
        "<robot name=\"pr2_test\">"
        "<virtual_joint name=\"base_joint\" child_link=\"base_footprint\" parent_frame=\"base_footprint\" type=\"planar\"/>"
        "</robot>";
    srdfModel->initString(*urdf_model_, SMODEL1);

    planning_models::KinematicModel kin_model1(urdf_model_, srdfModel);
    ASSERT_TRUE(kin_model1.getRoot() != NULL);
    EXPECT_EQ(kin_model1.getModelFrame(), "base_footprint");

    static const std::string SMODEL2 =
        "<?xml version=\"1.0\" ?>"
        "<robot name=\"pr2_test\">"
        "<virtual_joint name=\"world_joint\" child_link=\"base_footprint\" parent_frame=\"odom_combined\" type=\"floating\"/>"
        "</robot>";
    srdfModel->initString(*urdf_model_, SMODEL2);

    planning_models::KinematicModel kin_model2(urdf_model_, srdfModel);
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
    planning_models::KinematicModel kin_model1(urdf_model_, srdfModel);

    const planning_models::KinematicModel::JointModelGroup* left_arm_base_tip_group = kin_model1.getJointModelGroup("left_arm_base_tip");
    ASSERT_TRUE(left_arm_base_tip_group == NULL);

    const planning_models::KinematicModel::JointModelGroup* left_arm_joints_group = kin_model1.getJointModelGroup("left_arm_joints");
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

    planning_models::KinematicModelPtr kin_model2(new planning_models::KinematicModel(urdf_model_, srdfModel));

    left_arm_base_tip_group = kin_model2->getJointModelGroup("left_arm_base_tip");
    ASSERT_TRUE(left_arm_base_tip_group != NULL);

    left_arm_joints_group = kin_model2->getJointModelGroup("left_arm_joints");
    ASSERT_TRUE(left_arm_joints_group != NULL);

    EXPECT_EQ(left_arm_base_tip_group->getJointModels().size(), 7);
    EXPECT_EQ(left_arm_joints_group->getJointModels().size(), 7);

    EXPECT_EQ(left_arm_joints_group->getActiveDOFNames().size(), left_arm_joints_group->getVariableCount());
    EXPECT_EQ(left_arm_joints_group->getVariableCount(), 7);

    EXPECT_EQ(kin_model2->getActiveDOFNames().size(), kin_model2->getVariableCount());

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


    planning_models::KinematicState ks(kin_model2);
    ks.setToDefaultValues();
    std::map<std::string, double> jv;
    jv["base_joint.x"] = 0.433;
    jv["base_joint.theta"] = -0.5;
    ks.setStateValues(jv);
    moveit_msgs::RobotState robot_state;
    planning_models::kinematicStateToRobotState(ks, robot_state);

    planning_models::KinematicState ks2(kin_model2);
    robotStateToKinematicState(robot_state, ks2);
    std::vector<double> v1;
    ks.getStateValues(v1);
    std::vector<double> v2;
    ks2.getStateValues(v2);
    EXPECT_TRUE(v1.size() == v2.size());
    for (unsigned int i = 0; i < v1.size(); ++i)
	EXPECT_NEAR(v1[i], v2[i], 1e-5);

    geometry_msgs::Quaternion q;
    q.x = q.y = q.z = q.w = 0.0;
    Eigen::Quaterniond tq;
    EXPECT_FALSE(planning_models::quatFromMsg(q, tq));
    EXPECT_TRUE(tq.w() == 1.0);
}

TEST_F(LoadPlanningModelsPr2, SubgroupInit)
{
  planning_models::KinematicModel kmodel(urdf_model_, srdf_model_);  
  const planning_models::KinematicModel::JointModelGroup* jmg = kmodel.getJointModelGroup("arms");
  ASSERT_TRUE(jmg);
  EXPECT_EQ(jmg->getSubgroupNames().size(), 2);
  EXPECT_TRUE(jmg->isSubgroup("right_arm"));

  const planning_models::KinematicModel::JointModelGroup* jmg2 = kmodel.getJointModelGroup("whole_body");
  EXPECT_EQ(jmg2->getSubgroupNames().size(), 4);
  EXPECT_TRUE(jmg2->isSubgroup("arms"));
  EXPECT_TRUE(jmg2->isSubgroup("right_arm"));
  EXPECT_EQ(jmg2->getDisjointSubgroupNames().size(), 2);
}

TEST_F(LoadPlanningModelsPr2, SemanticInit)
{
  boost::shared_ptr<planning_models::KinematicModel> kmodel(new planning_models::KinematicModel(urdf_model_, srdf_model_));  
  planning_models::SemanticModel smodel(kmodel, srdf_model_);
  
  EXPECT_TRUE(smodel.isArm("right_arm"));
  EXPECT_TRUE(smodel.hasEndEffector("right_arm"));
  EXPECT_FALSE(smodel.isEndEffector("right_arm"));
  EXPECT_EQ(smodel.getEndEffector("right_arm"), "r_end_effector");
  EXPECT_EQ(smodel.getTipLink("right_arm"), "r_wrist_roll_link");
  EXPECT_EQ(smodel.getBaseLink("right_arm"), "torso_lift_link");
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
