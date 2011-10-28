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

/** \author Ioan Sucan, E. Gil Jones */

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>
#include <ctype.h>

static bool sameStringIgnoringWS(const std::string &s1, const std::string &s2)
{
    unsigned int i1 = 0;
    unsigned int i2 = 0;
    while (i1 < s1.size() && isspace(s1[i1])) i1++;
    while (i2 < s2.size() && isspace(s2[i2])) i2++;
    while (i1 < s1.size() && i2 < s2.size())
    {
	if (i1 < s1.size() && i2 < s2.size())
	{
	    if (s1[i1] != s2[i2])
		return false;
	    i1++;
	    i2++;
	}
	while (i1 < s1.size() && isspace(s1[i1])) i1++;
	while (i2 < s2.size() && isspace(s2[i2])) i2++;
    }
    return i1 == s1.size() && i2 == s2.size();
}

TEST(Loading, SimpleRobot)
{
    static const std::string MODEL0 = 
	"<?xml version=\"1.0\" ?>" 
	"<robot name=\"myrobot\">" 
	"  <link name=\"base_link\">"
	"    <collision name=\"base_collision\">"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0.165\"/>"
	"    <geometry name=\"base_collision_geom\">"
	"      <box size=\"0.65 0.65 0.23\"/>"
	"    </geometry>"
	"    </collision>"
	"   </link>"
	"</robot>";
    
    srdf::Model srdfModel;

    // hack:
    srdf::Model::VirtualJoint config;
    config.name_ = "base_joint";
    config.type_ = "floating";
    config.parent_frame_ = "odom_combined";
    config.child_link_ = "base_link";
    srdfModel.virtual_joints_.push_back(config);
    // end hack

    urdf::Model urdfModel;
    urdfModel.initString(MODEL0);
    
    
    planning_models::KinematicModelPtr model(new planning_models::KinematicModel(urdfModel, srdfModel));
    planning_models::KinematicState state(model);
    
    state.setDefaultValues();
    
    //make sure that this copy constructor works
    planning_models::KinematicState new_state(state);
    
    //(0,0,0,0) isn't a valid quaternion, so the w should be 1
    std::map<std::string, double> state_values;
    new_state.getStateValues(state_values);
    
    EXPECT_EQ(state_values["base_joint.rot_w"], 1.0);
    
    EXPECT_EQ(std::string("myrobot"), model->getName());
    EXPECT_EQ((unsigned int)7, new_state.getVariableCount());
    
    const std::vector<planning_models::KinematicModel::LinkModel*>& links = model->getLinkModels();
    EXPECT_EQ((unsigned int)1, links.size());
    
    const std::vector<planning_models::KinematicModel::JointModel*>& joints = model->getJointModels();
    EXPECT_EQ((unsigned int)1, joints.size());
    
    const std::vector<std::string>& pgroups = model->getJointModelGroupNames();    
    EXPECT_EQ((unsigned int)0, pgroups.size());
}

TEST(LoadingAndFK, SimpleRobot)
{   
    static const std::string MODEL1 = 
	"<?xml version=\"1.0\" ?>" 
	"<robot name=\"myrobot\">" 
	"<link name=\"base_link\">"
	"  <inertial>"
	"    <mass value=\"2.81\"/>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0.099 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision name=\"my_collision\">"
	"    <origin rpy=\"0 0 -1\" xyz=\"-0.1 0 0\"/>"
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
	"</robot>";
    
    static const std::string MODEL1_INFO = 
	"Model myrobot in frame odom_combined, of dimension 3\n"
	"Joint values bounds:\n"
	"   base_joint.x [DBL_MIN, DBL_MAX]\n"
	"   base_joint.y [DBL_MIN, DBL_MAX]\n"
	"   base_joint.theta [-3.14159, 3.14159]\n"
	"Available groups: \n"
	"   base (of dimension 3):\n"
	"    joints:\n"
	"      base_joint\n"
	"    links:\n"
	"      base_link\n"
	"    roots:\n"
	"      base_joint";
    
    srdf::Model srdfModel;

    // hack:
    srdf::Model::VirtualJoint config;
    config.name_ = "base_joint";
    config.type_ = "planar";
    config.parent_frame_ = "odom_combined";
    config.child_link_ = "base_link";
    srdfModel.virtual_joints_.push_back(config);

    srdf::Model::Group gb;
    gb.name_ = "base";
    gb.joints_.push_back(config.name_);
    srdfModel.groups_.push_back(gb);
    // end hack

    urdf::Model urdfModel;
    urdfModel.initString(MODEL1);
    
    planning_models::KinematicModelPtr model(new planning_models::KinematicModel(urdfModel, srdfModel));
    planning_models::KinematicState state(model);
    
    EXPECT_EQ((unsigned int)3, state.getVariableCount());
    
    state.setDefaultValues();
    
    const std::vector<planning_models::KinematicState::JointState*>& joint_states = state.getJointStateVector();
    EXPECT_EQ((unsigned int)1, joint_states.size());
    EXPECT_EQ((unsigned int)3, joint_states[0]->getVariableValues().size());
    
    
    std::stringstream ssi;
    model->printModelInfo(ssi);
    EXPECT_TRUE(sameStringIgnoringWS(MODEL1_INFO, ssi.str())) << ssi.str();
    
    
    std::map<std::string, double> joint_values;
    joint_values["base_joint.x"] = 10.0;
    joint_values["base_joint.y"] = 8.0;
    
    //testing incomplete state
    std::vector<std::string> missing_states;
    state.setStateValues(joint_values, missing_states);
    ASSERT_EQ(missing_states.size(), 1);
    EXPECT_EQ(missing_states[0], std::string("base_joint.theta"));
    joint_values["base_joint.theta"] = 0.0;
    
    state.setStateValues(joint_values, missing_states);
    ASSERT_EQ(missing_states.size(), 0);
    
    EXPECT_NEAR(10.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().x(), 1e-5);
    EXPECT_NEAR(8.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().z(), 1e-5);
    
    //making sure that values get copied
    planning_models::KinematicState new_state(state);
    EXPECT_NEAR(10.0, new_state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().x(), 1e-5);
    EXPECT_NEAR(8.0, new_state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, new_state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().z(), 1e-5);
    
    const std::map<std::string, unsigned int>& ind_map = model->getJointVariablesIndexMap();
    std::vector<double> jv(state.getVariableCount(), 0.0);
    jv[ind_map.at("base_joint.x")] = 10.0;
    jv[ind_map.at("base_joint.y")] = 8.0;
    jv[ind_map.at("base_joint.theta")] = 0.0;
    
    state.setStateValues(jv);
    EXPECT_NEAR(10.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().x(), 1e-5);
    EXPECT_NEAR(8.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().z(), 1e-5);
}


TEST(FK, OneRobot)
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
	"    <safety_controller k_position=\"20.0\" k_velocity=\"500.0\" soft_lower_limit=\"0.0\" soft_upper_limit=\"0.089\"/>"
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
	"</robot>";
    
    static const std::string MODEL2_INFO = 
	"Model one_robot in frame odom_combined, of dimension 5\n"
	"Joint values bounds: \n"
	"   base_joint.x [DBL_MIN, DBL_MAX]\n"
	"   base_joint.y [DBL_MIN, DBL_MAX]\n"
	"   base_joint.theta [-3.14159, 3.14159]\n"
	"   joint_a [-3.14159, 3.14159]\n"
	"   joint_c [0.00000, 0.08900]\n"
	"\n"
	"Available groups: \n"
	"   base_from_base_to_tip (of dimension 4):\n"
	"     joints:\n"
	"      base_joint\n"
	"      joint_a\n"
	"     links:\n"
	"      base_link\n"
	"      link_a\n"
	"      link_b\n"
	"     roots:\n"
	"      base_joint\n"
	"   base_from_joints (of dimension 5):\n"
	"     joints:\n"
	"      base_joint\n"
	"      joint_a\n"
	"      joint_c\n"
	"     links:\n"
	"      base_link\n"
	"      link_a\n"
	"      link_c\n"
	"     roots:\n"
	"      base_joint\n"
	"   base_with_subgroups (of dimension 5):\n"
	"     joints:\n"
	"      base_joint\n"
	"      joint_a\n"
	"      joint_c\n"
	"     links:\n"
	"      base_link\n"
	"      link_a\n"
	"      link_c\n"
	"     roots:\n"
	"      base_joint";
    
    urdf::Model urdfModel;
    urdfModel.initString(MODEL2);
    
    srdf::Model srdfModel;

    // hack:
    srdf::Model::VirtualJoint config;
    config.name_ = "base_joint";
    config.type_ = "planar";
    config.parent_frame_ = "odom_combined";
    config.child_link_ = "base_link";
    srdfModel.virtual_joints_.push_back(config);

    srdf::Model::Group gbj;
    gbj.name_ = "base_from_joints";
    gbj.joints_.push_back("base_joint");
    gbj.joints_.push_back("joint_a");
    gbj.joints_.push_back("joint_c");
    srdfModel.groups_.push_back(gbj);

    srdf::Model::Group gbs;
    gbs.name_ = "base_with_subgroups";
    gbs.joints_.push_back("joint_c");
    gbs.subgroups_.push_back("base_from_base_to_tip");
    srdfModel.groups_.push_back(gbs);

    srdf::Model::Group gbc;
    gbc.name_ = "base_from_base_to_tip";
    gbc.chains_.push_back(std::make_pair("base_link", "link_b"));
    srdfModel.groups_.push_back(gbc);
    
    gbs.subgroups_.push_back("error");
    gbs.name_ = "base_with_bad_subgroups";
    srdfModel.groups_.push_back(gbs);

    
    // end hack

    planning_models::KinematicModelPtr model(new planning_models::KinematicModel(urdfModel, srdfModel));

    //testing that the two planning groups are the same
    const planning_models::KinematicModel::JointModelGroup* g_one = model->getJointModelGroup("base_from_joints");
    const planning_models::KinematicModel::JointModelGroup* g_two = model->getJointModelGroup("base_from_base_to_tip");
    const planning_models::KinematicModel::JointModelGroup* g_three = model->getJointModelGroup("base_with_subgroups");
    const planning_models::KinematicModel::JointModelGroup* g_four = model->getJointModelGroup("base_with_bad_subgroups");
    
    ASSERT_TRUE(g_one != NULL);
    ASSERT_TRUE(g_two != NULL);
    ASSERT_TRUE(g_three != NULL);
    ASSERT_TRUE(g_four == NULL);
    
    //joint_b is a fixed joint, so no one should have it
    ASSERT_EQ(g_one->getJointModelNames().size(), 3); 
    ASSERT_EQ(g_two->getJointModelNames().size(), 2); 
    ASSERT_EQ(g_three->getJointModelNames().size(), 3); 
    
    //only the links in between the joints, and the children of the leafs
    ASSERT_EQ(g_one->getLinkModelNames().size(), 3);
    //g_two only has three links
    ASSERT_EQ(g_two->getLinkModelNames().size(), 3);
    ASSERT_EQ(g_three->getLinkModelNames().size(), 3);
    
    std::vector<std::string> jmn = g_one->getJointModelNames();
    std::sort(jmn.begin(), jmn.end());
    EXPECT_EQ(jmn[0],"base_joint");
    EXPECT_EQ(jmn[1],"joint_a");
    EXPECT_EQ(jmn[2],"joint_c");
    jmn = g_two->getJointModelNames();
    std::sort(jmn.begin(), jmn.end());
    EXPECT_EQ(jmn[0],"base_joint");
    EXPECT_EQ(jmn[1],"joint_a");
    jmn = g_three->getJointModelNames();
    std::sort(jmn.begin(), jmn.end());
    EXPECT_EQ(jmn[0],"base_joint");
    EXPECT_EQ(jmn[1],"joint_a");
    EXPECT_EQ(jmn[2],"joint_c");
    
    //but they should have the same links to be updated
    ASSERT_EQ(g_one->getUpdatedLinkModels().size(), 4);
    ASSERT_EQ(g_two->getUpdatedLinkModels().size(), 4);
    ASSERT_EQ(g_three->getUpdatedLinkModels().size(), 4);

    EXPECT_EQ(g_one->getUpdatedLinkModels()[0]->getName(),"base_link");
    EXPECT_EQ(g_one->getUpdatedLinkModels()[1]->getName(),"link_a");
    EXPECT_EQ(g_one->getUpdatedLinkModels()[2]->getName(),"link_b");
    EXPECT_EQ(g_one->getUpdatedLinkModels()[3]->getName(),"link_c");
    
    EXPECT_EQ(g_two->getUpdatedLinkModels()[0]->getName(),"base_link");
    EXPECT_EQ(g_two->getUpdatedLinkModels()[1]->getName(),"link_a");
    EXPECT_EQ(g_two->getUpdatedLinkModels()[2]->getName(),"link_b");
    EXPECT_EQ(g_two->getUpdatedLinkModels()[3]->getName(),"link_c");
    
    EXPECT_EQ(g_three->getUpdatedLinkModels()[0]->getName(),"base_link");
    EXPECT_EQ(g_three->getUpdatedLinkModels()[1]->getName(),"link_a");
    EXPECT_EQ(g_three->getUpdatedLinkModels()[2]->getName(),"link_b");
    EXPECT_EQ(g_three->getUpdatedLinkModels()[3]->getName(),"link_c");
    
    //bracketing so the state gets destroyed before we bring down the model
    
    planning_models::KinematicState state(model);
    
    EXPECT_EQ((unsigned int)5, state.getVariableCount());
    
    state.setDefaultValues();
    
    std::map<std::string, double> joint_values;
    joint_values["base_joint.x"]=1.0;
    joint_values["base_joint.y"]=1.0;
    joint_values["base_joint.theta"]=0.5;
    joint_values["joint_a"] = -0.5;
    joint_values["joint_c"] = 0.1;
    state.getJointStateGroup("base_from_joints")->setStateValues(joint_values);
    
    //double param[5] = { 1, 1, 0.5, -0.5, 0.1 };
    //model->getGroup("base")->computeTransforms(param);
    
    std::stringstream ss0;
    model->printModelInfo(ss0);
    EXPECT_TRUE(sameStringIgnoringWS(MODEL2_INFO, ss0.str())) << MODEL2_INFO << "\n" << ss0.str();
    
    std::stringstream ss1;
    state.printTransforms(ss1);

    //make sure it works the same way for the whole robot
    state.setStateValues(joint_values);
    std::stringstream ss2;
    state.printTransforms(ss2);
    
    EXPECT_EQ(ss1.str(), ss2.str());
    
    EXPECT_NEAR(1.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().x(), 1e-5);
    EXPECT_NEAR(1.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().z(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("base_link")->getGlobalLinkTransform().getRotation().x(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("base_link")->getGlobalLinkTransform().getRotation().y(), 1e-5);
    EXPECT_NEAR(0.247404, state.getLinkState("base_link")->getGlobalLinkTransform().getRotation().z(), 1e-5);
    EXPECT_NEAR(0.968912, state.getLinkState("base_link")->getGlobalLinkTransform().getRotation().w(), 1e-5);
    
    EXPECT_NEAR(1.0, state.getLinkState("link_a")->getGlobalLinkTransform().getOrigin().x(), 1e-5);
    EXPECT_NEAR(1.0, state.getLinkState("link_a")->getGlobalLinkTransform().getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_a")->getGlobalLinkTransform().getOrigin().z(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_a")->getGlobalLinkTransform().getRotation().x(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_a")->getGlobalLinkTransform().getRotation().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_a")->getGlobalLinkTransform().getRotation().z(), 1e-5);
    EXPECT_NEAR(1.0, state.getLinkState("link_a")->getGlobalLinkTransform().getRotation().w(), 1e-5);
    
    EXPECT_NEAR(1.0, state.getLinkState("link_b")->getGlobalLinkTransform().getOrigin().x(), 1e-5);
    EXPECT_NEAR(1.5, state.getLinkState("link_b")->getGlobalLinkTransform().getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_b")->getGlobalLinkTransform().getOrigin().z(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_b")->getGlobalLinkTransform().getRotation().x(), 1e-5);
    EXPECT_NEAR(-0.20846, state.getLinkState("link_b")->getGlobalLinkTransform().getRotation().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_b")->getGlobalLinkTransform().getRotation().z(), 1e-5);
    EXPECT_NEAR(0.978031, state.getLinkState("link_b")->getGlobalLinkTransform().getRotation().w(), 1e-5);
    
    EXPECT_NEAR(1.1, state.getLinkState("link_c")->getGlobalLinkTransform().getOrigin().x(), 1e-5);
    EXPECT_NEAR(1.4, state.getLinkState("link_c")->getGlobalLinkTransform().getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_c")->getGlobalLinkTransform().getOrigin().z(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_c")->getGlobalLinkTransform().getRotation().x(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_c")->getGlobalLinkTransform().getRotation().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_c")->getGlobalLinkTransform().getRotation().z(), 1e-5);
    EXPECT_NEAR(1.0, state.getLinkState("link_c")->getGlobalLinkTransform().getRotation().w(), 1e-5);
    
    //bonus bounds lookup test
    std::vector<std::string> jn;
    jn.push_back("base_joint");
    EXPECT_TRUE(state.satisfiesBounds(jn));
    
    jn.push_back("monkey");
    EXPECT_FALSE(state.satisfiesBounds(jn));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

