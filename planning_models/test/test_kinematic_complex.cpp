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
#include <gtest/gtest.h>
#include <sstream>
#include <ctype.h>
#include <ros/package.h>

//urdf location relative to the planning_models path
static const std::string rel_path = "/test_urdf/robot.xml";

class LoadPlanningModelsPr2 : public testing::Test 
{
protected:
    
    virtual void SetUp()
    {
	full_path_ = ros::package::getPath("planning_models")+rel_path;
	urdf_ok_ = urdf_model_.initFile(full_path_);
    };
    
    virtual void TearDown() 
    {
    }
    
protected:
    
    urdf::Model urdf_model_;
    bool        urdf_ok_;
    std::string full_path_;
    
};

TEST_F(LoadPlanningModelsPr2, InitOK) 
{
    ASSERT_TRUE(urdf_ok_) << full_path_;
    ASSERT_EQ(urdf_model_.getName(), "pr2_test");
}

TEST_F(LoadPlanningModelsPr2, MultidofInit)
{
    //hack:
    srdf::Model srdfModel;    
    //end hack
    
    // with no world multidof we should get a fixed joint
    planning_models::KinematicModel kin_model0(urdf_model_, srdfModel);    
    EXPECT_TRUE(kin_model0.getRoot()->getVariableCount() == 0);
    
    // hack:
    srdf::Model::VirtualJoint config;
    config.name_ = "base_joint";
    config.type_ = "planar";
    config.parent_frame_ = "base_footprint";
    config.child_link_ = "base_footprint";
    srdfModel.virtual_joints_.push_back(config);
    // end hack

    planning_models::KinematicModel kin_model1(urdf_model_, srdfModel);    
    ASSERT_TRUE(kin_model1.getRoot() != NULL);
    EXPECT_EQ(kin_model1.getModelFrame(), "base_footprint");
    
    //now this should work with an non-identity transform
    config.name_ = "world_joint";
    config.type_ = "floating";
    config.parent_frame_ = "odom_combined";
    config.child_link_ = "base_footprint";
    srdfModel.virtual_joints_[0] = config;
    
    planning_models::KinematicModel kin_model2(urdf_model_, srdfModel);    
    ASSERT_TRUE(kin_model2.getRoot() != NULL);
    EXPECT_EQ(kin_model2.getModelFrame(), "odom_combined");
}

TEST_F(LoadPlanningModelsPr2, GroupInit) 
{ 
    //hack:
    srdf::Model srdfModel;    
  
    srdf::Model::VirtualJoint config;
    config.name_ = "base_joint";
    config.type_ = "planar";
    config.parent_frame_ = "base_footprint";
    config.child_link_ = "base_footprint";
    srdfModel.virtual_joints_.push_back(config);


    srdf::Model::Group g1;
    g1.name_ = "left_arm_base_tip";
    g1.chains_.push_back(std::make_pair("monkey_base", "monkey_tip"));
    srdfModel.groups_.push_back(g1);
    
    srdf::Model::Group g2;
    g2.name_ = "left_arm_joints";
    g2.joints_.push_back("l_monkey_pan_joint");
    g2.joints_.push_back("l_monkey_lift_joint");
    g2.joints_.push_back("l_monkey_arm_roll_joint");
    g2.joints_.push_back("l_monkey_flex_joint");
    g2.joints_.push_back("l_monkey_roll_joint");
    g2.joints_.push_back("l_monkey_flex_link");
    g2.joints_.push_back("l_monkey_roll_link");
    
    srdfModel.groups_.push_back(g2);
    // end hack
    
    planning_models::KinematicModel kin_model1(urdf_model_, srdfModel);
    
    const planning_models::KinematicModel::JointModelGroup* left_arm_base_tip_group = kin_model1.getJointModelGroup("left_arm_base_tip");
    ASSERT_TRUE(left_arm_base_tip_group == NULL);
    
    const planning_models::KinematicModel::JointModelGroup* left_arm_joints_group = kin_model1.getJointModelGroup("left_arm_joints");
    ASSERT_TRUE(left_arm_joints_group == NULL);
    
    srdf::Model::Group g3;
    g3.name_ = "left_arm_base_tip";
    g3.chains_.push_back(std::make_pair("l_shoulder_pan_link", "l_wrist_roll_link"));

    srdf::Model::Group g4;
    g4.name_ = "left_arm_joints";
    g4.joints_.push_back("l_shoulder_pan_joint");
    g4.joints_.push_back("l_shoulder_lift_joint");
    g4.joints_.push_back("l_upper_arm_roll_joint");
    g4.joints_.push_back("l_elbow_flex_joint");
    g4.joints_.push_back("l_forearm_roll_joint");
    g4.joints_.push_back("l_wrist_flex_joint");
    g4.joints_.push_back("l_wrist_roll_joint");
    
    srdfModel.groups_[0] = g3;
    srdfModel.groups_[1] = g4;
    
    planning_models::KinematicModelPtr kin_model2(new planning_models::KinematicModel(urdf_model_, srdfModel));
    
    left_arm_base_tip_group = kin_model2->getJointModelGroup("left_arm_base_tip");
    ASSERT_TRUE(left_arm_base_tip_group != NULL);
    
    left_arm_joints_group = kin_model2->getJointModelGroup("left_arm_joints");
    ASSERT_TRUE(left_arm_joints_group != NULL);
    
    EXPECT_EQ(left_arm_base_tip_group->getJointModels().size(), 7);
    EXPECT_EQ(left_arm_joints_group->getJointModels().size(), 7);
    
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
    ks.setDefaultValues();
    std::map<std::string, double> jv;
    jv["base_joint.x"] = 0.433;
    ks.setStateValues(jv);
    moveit_msgs::RobotState robot_state;
    planning_models::kinematicStateToRobotState(ks, robot_state); 
    std::cout << robot_state << std::endl;
    
    planning_models::KinematicState ks2(kin_model2);
    robotStateToKinematicState(robot_state, ks2);
    std::vector<double> v1;
    ks.getStateValues(v1);
    std::vector<double> v2;
    ks2.getStateValues(v2);
    EXPECT_TRUE(v1 == v2);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
