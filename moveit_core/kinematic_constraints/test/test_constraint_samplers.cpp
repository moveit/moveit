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

/* Author Ioan Sucan */

#include <pluginlib/class_loader.h>

#include <kinematic_constraints/kinematic_constraint.h>
#include <kinematic_constraints/constraint_samplers.h>
#include <gtest/gtest.h>

class LoadPlanningModelsPr2 : public testing::Test
{
protected:

    virtual void SetUp()
    {
	srdf::Model::Group g_weird;
	g_weird.name_ = "weird_group";
	g_weird.joints_.push_back("head_pan_joint");
	g_weird.joints_.push_back("torso_lift_joint");
	g_weird.joints_.push_back("r_shoulder_pan_joint");
	g_weird.joints_.push_back("r_wrist_roll_joint");
	g_weird.joints_.push_back("l_shoulder_pan_joint");
	g_weird.joints_.push_back("l_wrist_roll_joint");
	srdf_model.groups_.push_back(g_weird);	

	srdf::Model::Group l_arm;
	l_arm.name_ = "left_arm";
	l_arm.chains_.push_back(std::make_pair("l_shoulder_pan_link", "l_wrist_roll_link"));
	srdf_model.groups_.push_back(l_arm);
	
        urdf_model.initFile("../planning_models/test/urdf/robot.xml");
        kmodel.reset(new planning_models::KinematicModel(urdf_model, srdf_model));
	kinematics_loader_.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("kinematics_base","kinematics::KinematicsBase"));
	
	ik_solver_name_ = "pr2_arm_kinematics/PR2ArmKinematicsPlugin";
	if (kinematics_loader_->isClassAvailable(ik_solver_name_))
	    ik_allocator_ = boost::bind(&LoadPlanningModelsPr2::allocIKSolver, this, _1);
	else
	    ROS_ERROR("PR2 IK solver plugin not found");
    };

    virtual void TearDown()
    {
    }

    boost::shared_ptr<kinematics::KinematicsBase> allocIKSolver(const planning_models::KinematicModel::JointModelGroup *jmg)
    {
	boost::shared_ptr<kinematics::KinematicsBase> result;
	if (kinematics_loader_)
	{
	    result.reset(kinematics_loader_->createClassInstance(ik_solver_name_));
	    if (result)
		result->initialize(jmg->getName());
	}
	return result;
    }
    
protected:

    urdf::Model                                                            urdf_model;
    srdf::Model                                                            srdf_model;
    planning_models::KinematicModelPtr                                     kmodel; 
    std::string                                                            ik_solver_name_;
    boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematics_loader_;
    kinematic_constraints::IKAllocator                                     ik_allocator_;
};

TEST_F(LoadPlanningModelsPr2, JointConstraintsSampler)
{
    planning_models::KinematicState ks(kmodel);
    ks.setDefaultValues();
    planning_models::Transforms tf(kmodel->getModelFrame());

    kinematic_constraints::JointConstraint jc1(*kmodel, tf);
    moveit_msgs::JointConstraint jcm1;
    jcm1.joint_name = "head_pan_joint";
    jcm1.position = 0.42;
    jcm1.tolerance_above = 0.01;
    jcm1.tolerance_below = 0.05;
    jcm1.weight = 1.0;
    EXPECT_TRUE(jc1.use(jcm1));

    kinematic_constraints::JointConstraint jc2(*kmodel, tf);
    moveit_msgs::JointConstraint jcm2;
    jcm2.joint_name = "l_shoulder_pan_joint";
    jcm2.position = 0.9;
    jcm2.tolerance_above = 0.1;
    jcm2.tolerance_below = 0.05;
    jcm2.weight = 1.0;
    EXPECT_TRUE(jc2.use(jcm2));

    kinematic_constraints::JointConstraint jc3(*kmodel, tf);
    moveit_msgs::JointConstraint jcm3;
    jcm3.joint_name = "r_wrist_roll_joint";
    jcm3.position = 0.7;
    jcm3.tolerance_above = 0.14;
    jcm3.tolerance_below = 0.005;
    jcm3.weight = 1.0;
    EXPECT_TRUE(jc3.use(jcm3));
    
    kinematic_constraints::JointConstraint jc4(*kmodel, tf);
    moveit_msgs::JointConstraint jcm4;
    jcm4.joint_name = "torso_lift_joint";
    jcm4.position = 0.2;
    jcm4.tolerance_above = 0.09;
    jcm4.tolerance_below = 0.01;
    jcm4.weight = 1.0;
    EXPECT_TRUE(jc4.use(jcm4));
    
    std::vector<kinematic_constraints::JointConstraint> js;
    js.push_back(jc1);
    js.push_back(jc2);
    js.push_back(jc3);
    js.push_back(jc4);
    
    kinematic_constraints::JointConstraintSampler jcs(kmodel->getJointModelGroup("weird_group"), js);
    EXPECT_EQ(jcs.getConstrainedJointCount(), 4);
    EXPECT_EQ(jcs.getUnconstrainedJointCount(), 2);

    for (int t = 0 ; t < 1000 ; ++t)
    {
	std::vector<double> values;
	EXPECT_TRUE(jcs.sample(values));
	ks.getJointStateGroup("weird_group")->setStateValues(values);
	for (unsigned int i = 0 ; i < js.size() ; ++i)
	    EXPECT_TRUE(js[i].decide(ks).first);
    }
}

TEST_F(LoadPlanningModelsPr2, PoseConstraintsSampler)
{
    planning_models::KinematicState ks(kmodel);
    ks.setDefaultValues();
    planning_models::Transforms tf(kmodel->getModelFrame());

    kinematic_constraints::PositionConstraint pc(*kmodel, tf);
    moveit_msgs::PositionConstraint pcm;

    pcm.link_name = "l_wrist_roll_link";
    pcm.target_point_offset.x = 0;
    pcm.target_point_offset.y = 0;
    pcm.target_point_offset.z = 0;
    pcm.constraint_region_shape.type = moveit_msgs::Shape::SPHERE;
    pcm.constraint_region_shape.dimensions.push_back(0.001);

    pcm.constraint_region_pose.header.frame_id = kmodel->getModelFrame();
    pcm.constraint_region_pose.pose.position.x = 0.55;
    pcm.constraint_region_pose.pose.position.y = 0.2;
    pcm.constraint_region_pose.pose.position.z = 1.25;
    pcm.constraint_region_pose.pose.orientation.x = 0.0;
    pcm.constraint_region_pose.pose.orientation.y = 0.0;
    pcm.constraint_region_pose.pose.orientation.z = 0.0;
    pcm.constraint_region_pose.pose.orientation.w = 1.0;
    pcm.weight = 1.0;

    EXPECT_TRUE(pc.use(pcm));
    
    kinematic_constraints::OrientationConstraint oc(*kmodel, tf);
    moveit_msgs::OrientationConstraint ocm;

    ocm.link_name = "l_wrist_roll_link";
    ocm.orientation.header.frame_id = kmodel->getModelFrame();
    ocm.orientation.quaternion.x = 0.0;
    ocm.orientation.quaternion.y = 0.0;
    ocm.orientation.quaternion.z = 0.0;
    ocm.orientation.quaternion.w = 1.0;
    ocm.absolute_roll_tolerance = 0.2;
    ocm.absolute_pitch_tolerance = 0.1;
    ocm.absolute_yaw_tolerance = 0.4;
    ocm.weight = 1.0;

    EXPECT_TRUE(oc.use(ocm));

    kinematic_constraints::IKConstraintSampler iks1(ik_allocator_, kmodel->getJointModelGroup("left_arm"), pc, oc);
    for (int t = 0 ; t < 100 ; ++t)
    {
	std::vector<double> values;
	EXPECT_TRUE(iks1.sample(values, 100, &ks));
	ks.getJointStateGroup("left_arm")->setStateValues(values);    
	EXPECT_TRUE(pc.decide(ks).first);
	EXPECT_TRUE(oc.decide(ks).first);
    }

    kinematic_constraints::IKConstraintSampler iks2(ik_allocator_, kmodel->getJointModelGroup("left_arm"), pc);
    for (int t = 0 ; t < 100 ; ++t)
    {
	std::vector<double> values;
	EXPECT_TRUE(iks2.sample(values, 100, &ks));
	ks.getJointStateGroup("left_arm")->setStateValues(values);    
	EXPECT_TRUE(pc.decide(ks).first);
    }
    
    kinematic_constraints::IKConstraintSampler iks3(ik_allocator_, kmodel->getJointModelGroup("left_arm"), oc);
    for (int t = 0 ; t < 100 ; ++t)
    {
	std::vector<double> values;
	EXPECT_TRUE(iks3.sample(values, 100, &ks));
	ks.getJointStateGroup("left_arm")->setStateValues(values);    
	EXPECT_TRUE(oc.decide(ks).first);
    }    
}


TEST_F(LoadPlanningModelsPr2, OrientationConstraintsSampler)
{
    planning_models::KinematicState ks(kmodel);
    ks.setDefaultValues();
    planning_models::Transforms tf(kmodel->getModelFrame());

    kinematic_constraints::OrientationConstraint oc(*kmodel, tf);
    moveit_msgs::OrientationConstraint ocm;

    ocm.link_name = "r_wrist_roll_link";
    ocm.orientation.header.frame_id = kmodel->getModelFrame();
    ocm.orientation.quaternion.x = 0.0;
    ocm.orientation.quaternion.y = 0.0;
    ocm.orientation.quaternion.z = 0.0;
    ocm.orientation.quaternion.w = 1.0;
    ocm.absolute_roll_tolerance = 0.01;
    ocm.absolute_pitch_tolerance = 0.01;
    ocm.absolute_yaw_tolerance = 0.01;
    ocm.weight = 1.0;

    EXPECT_TRUE(oc.use(ocm));

    const std::pair<bool, double> &p1 = oc.decide(ks);
    EXPECT_FALSE(p1.first);

    ocm.orientation.header.frame_id = ocm.link_name;
    EXPECT_TRUE(oc.use(ocm));
    const std::pair<bool, double> &p2 = oc.decide(ks);
    EXPECT_TRUE(p2.first);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "iks_test");
    
    return RUN_ALL_TESTS();
}
