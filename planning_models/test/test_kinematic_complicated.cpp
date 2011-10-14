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

/** \author E. Gil Jones */

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <gtest/gtest.h>
#include <sstream>
#include <ctype.h>
#include <ros/package.h>

//urdf location relative to the planning_models path
static const std::string rel_path = "/test_urdf/robot.xml";

class LoadPlanningModelsPr2 : public testing::Test {
protected:
  
  virtual void SetUp() {

    full_path_ = ros::package::getPath("planning_models")+rel_path;
    
    urdf_ok_ = urdf_model_.initFile(full_path_);

  };

  virtual void TearDown() {

  }

protected:

  urdf::Model urdf_model_;
  bool urdf_ok_;
  std::string full_path_;

};

TEST_F(LoadPlanningModelsPr2, InitOK) 
{
  ASSERT_TRUE(urdf_ok_) << full_path_;
  ASSERT_EQ(urdf_model_.getName(),"pr2_test");
}

TEST_F(LoadPlanningModelsPr2, MultidofInit)
{
  std::vector<planning_models::KinematicModel::GroupConfig> gcs;

  {
    std::vector<planning_models::KinematicModel::MultiDofConfig> multi_dof_configs;
    //with no world multidof we should fail
    planning_models::KinematicModel kin_model(urdf_model_,gcs, multi_dof_configs);    
    ASSERT_TRUE(kin_model.getRoot() == NULL);
  }

  {
    std::vector<planning_models::KinematicModel::MultiDofConfig> multi_dof_configs;
    //now this should work with an identity transform
    planning_models::KinematicModel::MultiDofConfig config("base_joint");
    config.type = "Planar";
    config.parent_frame_id = "base_footprint";
    config.child_frame_id = "base_footprint";
    multi_dof_configs.push_back(config);
    
    planning_models::KinematicModel kin_model(urdf_model_,gcs, multi_dof_configs);    
    ASSERT_TRUE(kin_model.getRoot() != NULL);
    EXPECT_EQ(kin_model.getRoot()->getParentFrameId(), "base_footprint");
  }

  {
    std::vector<planning_models::KinematicModel::MultiDofConfig> multi_dof_configs;

    //now this should work with an non-identity transform
    planning_models::KinematicModel::MultiDofConfig config("world_joint");
    config.type = "Floating";
    config.parent_frame_id = "odom_combined";
    config.child_frame_id = "base_footprint";
    multi_dof_configs.push_back(config);
    
    planning_models::KinematicModel kin_model(urdf_model_,gcs, multi_dof_configs);    
    ASSERT_TRUE(kin_model.getRoot() != NULL);
    EXPECT_EQ(kin_model.getRoot()->getParentFrameId(), "odom_combined");
  }

  //now we test joint state equivalents
}

TEST_F(LoadPlanningModelsPr2, GroupInit) 
{
  std::vector<planning_models::KinematicModel::MultiDofConfig> multi_dof_configs;
  planning_models::KinematicModel::MultiDofConfig config("base_joint");
  config.type = "Planar";
  config.parent_frame_id = "base_footprint";
  config.child_frame_id = "base_footprint";
  multi_dof_configs.push_back(config);

  {
    std::vector<planning_models::KinematicModel::GroupConfig> gcs;
    //first make sure we can intialize with no groups

    planning_models::KinematicModel kin_model(urdf_model_,gcs, multi_dof_configs);    
    ASSERT_TRUE(kin_model.getRoot() != NULL);
  }

  {
    //if we screw something up we get no group
    std::vector<planning_models::KinematicModel::GroupConfig> gcs;
    planning_models::KinematicModel::GroupConfig left_arm_base_tip_gc("left_arm_base_tip",
                                     "monkey_base",
                                     "monkey_tip");
    gcs.push_back(left_arm_base_tip_gc);
    
    std::vector<std::string> left_arm_joints;
    left_arm_joints.push_back("l_monkey_pan_joint");
    left_arm_joints.push_back("l_monkey_lift_joint");
    left_arm_joints.push_back("l_monkey_arm_roll_joint");
    left_arm_joints.push_back("l_monkey_flex_joint");
    left_arm_joints.push_back("l_monkey_roll_joint");
    left_arm_joints.push_back("l_monkey_flex_link");
    left_arm_joints.push_back("l_monkey_roll_link");

    std::vector<std::string> subgroups;
    
    planning_models::KinematicModel::GroupConfig left_arm_joints_gc("left_arm_joints",
                                                                    left_arm_joints,
                                                                    subgroups);
    
    gcs.push_back(left_arm_base_tip_gc);
    gcs.push_back(left_arm_joints_gc);

    planning_models::KinematicModel kin_model(urdf_model_,gcs, multi_dof_configs);
    
    const planning_models::KinematicModel::JointModelGroup* left_arm_base_tip_group = kin_model.getModelGroup("left_arm_base_tip");
    ASSERT_TRUE(left_arm_base_tip_group == NULL);

    const planning_models::KinematicModel::JointModelGroup* left_arm_joints_group = kin_model.getModelGroup("left_arm_joints");
    ASSERT_TRUE(left_arm_joints_group == NULL);
  }

  {
    std::vector<planning_models::KinematicModel::GroupConfig> gcs;
    planning_models::KinematicModel::GroupConfig left_arm_base_tip_gc("left_arm_base_tip",
                                                                      "torso_lift_link",
                                                                      "l_wrist_roll_link");
    
    std::vector<std::string> left_arm_joints;
    left_arm_joints.push_back("l_shoulder_pan_joint");
    left_arm_joints.push_back("l_shoulder_lift_joint");
    left_arm_joints.push_back("l_upper_arm_roll_joint");
    left_arm_joints.push_back("l_elbow_flex_joint");
    left_arm_joints.push_back("l_forearm_roll_joint");
    left_arm_joints.push_back("l_wrist_flex_joint");
    left_arm_joints.push_back("l_wrist_roll_joint");

    std::vector<std::string> subgroups;
    
    planning_models::KinematicModel::GroupConfig left_arm_joints_gc("left_arm_joints",
                                                                    left_arm_joints,
                                                                    subgroups);
    
    gcs.push_back(left_arm_base_tip_gc);
    gcs.push_back(left_arm_joints_gc);

    planning_models::KinematicModel kin_model(urdf_model_,gcs, multi_dof_configs);
    
    const planning_models::KinematicModel::JointModelGroup* left_arm_base_tip_group = kin_model.getModelGroup("left_arm_base_tip");
    ASSERT_TRUE(left_arm_base_tip_group != NULL);

    const planning_models::KinematicModel::JointModelGroup* left_arm_joints_group = kin_model.getModelGroup("left_arm_joints");
    ASSERT_TRUE(left_arm_joints_group != NULL);

    for(unsigned int i = 0; i < std::min(left_arm_base_tip_group->getJointModels().size(),
                                         left_arm_joints_group->getJointModels().size()); i++) {
      EXPECT_EQ(left_arm_base_tip_group->getJointModels()[i]->getName(),
                left_arm_joints_group->getJointModels()[i]->getName());
    } 

    EXPECT_EQ(left_arm_base_tip_group->getJointModels().size(), 7);
    EXPECT_EQ(left_arm_joints_group->getJointModels().size(), 7);
    
    bool found_shoulder_pan_link = false;
    bool found_wrist_roll_link = false;
    for(unsigned int i = 0; i < left_arm_base_tip_group->getGroupLinkModels().size(); i++) {
      if(left_arm_base_tip_group->getGroupLinkModels()[i]->getName() == "l_shoulder_pan_link") {
        EXPECT_TRUE(found_shoulder_pan_link == false);
        found_shoulder_pan_link = true;
      }
      if(left_arm_base_tip_group->getGroupLinkModels()[i]->getName() == "l_wrist_roll_link") {
        EXPECT_TRUE(found_wrist_roll_link == false);
        found_wrist_roll_link = true;
      }
      EXPECT_TRUE(left_arm_base_tip_group->getGroupLinkModels()[i]->getName() != "torso_lift_link");
    }
    EXPECT_TRUE(found_shoulder_pan_link);
    EXPECT_TRUE(found_wrist_roll_link);
  }
  
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

