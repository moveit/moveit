/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

#include <planning_scene_monitor/planning_scene_monitor.h>
#include <kinematic_constraints/kinematic_constraint.h>
#include <kinematic_constraints/constraint_samplers.h>
#include <kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <planning_models/conversions.h>

#include <ros/ros.h>
#include <gtest/gtest.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

class ConstraintSamplerTestBase : public testing::Test
{
protected:
  
  virtual void SetUp()
  {
    psm_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, NULL));
    kmodel_ = psm_->getPlanningScene()->getKinematicModel();
  };
  
  virtual void TearDown()
  {
  }
  
protected:
  
  ros::NodeHandle nh_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  planning_models::KinematicModelConstPtr kmodel_;
};


TEST_F(ConstraintSamplerTestBase, JointConstraintsSampler)
{
  planning_models::KinematicState ks(kmodel_);
  ks.setToDefaultValues();
  planning_models::TransformsPtr tf = psm_->getPlanningScene()->getTransforms();
  
  kinematic_constraints::JointConstraint jc1(kmodel_, tf);
  moveit_msgs::JointConstraint jcm1;
  jcm1.joint_name = "head_pan_joint";
  jcm1.position = 0.42;
  jcm1.tolerance_above = 0.01;
  jcm1.tolerance_below = 0.05;
  jcm1.weight = 1.0;
  EXPECT_TRUE(jc1.configure(jcm1));

  kinematic_constraints::JointConstraint jc2(kmodel_, tf);
  moveit_msgs::JointConstraint jcm2;
  jcm2.joint_name = "l_shoulder_pan_joint";
  jcm2.position = 0.9;
  jcm2.tolerance_above = 0.1;
  jcm2.tolerance_below = 0.05;
  jcm2.weight = 1.0;
  EXPECT_TRUE(jc2.configure(jcm2));
  
  kinematic_constraints::JointConstraint jc3(kmodel_, tf);
  moveit_msgs::JointConstraint jcm3;
  jcm3.joint_name = "r_wrist_roll_joint";
  jcm3.position = 0.7;
  jcm3.tolerance_above = 0.14;
  jcm3.tolerance_below = 0.005;
  jcm3.weight = 1.0;
  EXPECT_TRUE(jc3.configure(jcm3));
  
  kinematic_constraints::JointConstraint jc4(kmodel_, tf);
  moveit_msgs::JointConstraint jcm4;
  jcm4.joint_name = "torso_lift_joint";
  jcm4.position = 0.2;
  jcm4.tolerance_above = 0.09;
  jcm4.tolerance_below = 0.01;
  jcm4.weight = 1.0;
  EXPECT_TRUE(jc4.configure(jcm4));

  std::vector<kinematic_constraints::JointConstraint> js;
  js.push_back(jc1);
  js.push_back(jc2);
  js.push_back(jc3);
  js.push_back(jc4);
  
  kinematic_constraints::JointConstraintSampler jcs(kmodel_->getJointModelGroup("arms"), js);
  EXPECT_EQ(jcs.getConstrainedJointCount(), 2);
  EXPECT_EQ(jcs.getUnconstrainedJointCount(), 12);

  for (int t = 0 ; t < 1000 ; ++t)
  {
    std::vector<double> values;
    EXPECT_TRUE(jcs.sample(values, ks));
    ks.getJointStateGroup("arms")->setStateValues(values);
    double distance(0.0);
    EXPECT_TRUE(jc2.decide(ks,distance));
    EXPECT_TRUE(jc3.decide(ks,distance));
  }

  kinematics_plugin_loader::KinematicsPluginLoader kinematics_loader;
  kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_loader.getLoaderFunction();
  
  // test the automatic construction of constraint sampler
  moveit_msgs::Constraints c;
  
  // no constraints should give no sampler
  kinematic_constraints::ConstraintSamplerPtr s0 = kinematic_constraints::constructConstraintsSampler
    (kmodel_->getJointModelGroup("arms"), c, kmodel_, tf, kinematics_allocator);
  EXPECT_TRUE(s0.get() == NULL);

  // add the constraints
  c.joint_constraints.push_back(jcm1);
  c.joint_constraints.push_back(jcm2);
  c.joint_constraints.push_back(jcm3);
  c.joint_constraints.push_back(jcm4);
  
  kinematic_constraints::ConstraintSamplerPtr s = kinematic_constraints::constructConstraintsSampler
    (kmodel_->getJointModelGroup("arms"), c, kmodel_, tf, kinematics_allocator);
  EXPECT_TRUE(s.get() != NULL);
  
  // test the generated sampler
  for (int t = 0 ; t < 1000 ; ++t)
  {
    std::vector<double> values;
    EXPECT_TRUE(s->sample(values, ks));
    ks.getJointStateGroup("arms")->setStateValues(values);
    double distance;
    EXPECT_TRUE(jc2.decide(ks,distance));
    EXPECT_TRUE(jc3.decide(ks,distance));
  }
}

TEST_F(ConstraintSamplerTestBase, OrientationConstraintsSampler)
{
  planning_models::KinematicState ks(kmodel_);
  ks.setToDefaultValues();
  planning_models::TransformsPtr tf = psm_->getPlanningScene()->getTransforms();
  
  kinematic_constraints::OrientationConstraint oc(kmodel_, tf);
  moveit_msgs::OrientationConstraint ocm;
  
  ocm.link_name = "r_wrist_roll_link";
  ocm.orientation.header.frame_id = ocm.link_name; 
  ocm.orientation.quaternion.x = 0.5;
  ocm.orientation.quaternion.y = 0.5;
  ocm.orientation.quaternion.z = 0.5;
  ocm.orientation.quaternion.w = 0.5;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = 0.01;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  
  EXPECT_TRUE(oc.configure(ocm));
  
  double distance = 0.0;
  bool p1 = oc.decide(ks, distance);
  EXPECT_FALSE(p1);
  
  ocm.orientation.header.frame_id = kmodel_->getModelFrame();
  EXPECT_TRUE(oc.configure(ocm));
  
  kinematics_plugin_loader::KinematicsPluginLoader kinematics_loader;
  kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_loader.getLoaderFunction();
  
  kinematic_constraints::IKConstraintSampler iks(kmodel_->getJointModelGroup("right_arm"), kinematics_allocator,
                                                 kinematic_constraints::IKSamplingPose(oc));
  for (int t = 0 ; t < 100 ; ++t)
  {
    std::vector<double> values;
    EXPECT_TRUE(iks.sample(values, ks, 100));
    ks.getJointStateGroup("right_arm")->setStateValues(values);
    double distance = 0.0;
    EXPECT_TRUE(oc.decide(ks, distance));
  }
  
}

TEST_F(ConstraintSamplerTestBase, PoseConstraintsSampler)
{
  planning_models::KinematicState ks(kmodel_);
  ks.setToDefaultValues();
  planning_models::TransformsPtr tf = psm_->getPlanningScene()->getTransforms();
  
  kinematic_constraints::PositionConstraint pc(kmodel_, tf);
  moveit_msgs::PositionConstraint pcm;
  
  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region_shape.type = moveit_msgs::Shape::SPHERE;
  pcm.constraint_region_shape.dimensions.push_back(0.001);
  
  pcm.constraint_region_pose.header.frame_id = kmodel_->getModelFrame();
  pcm.constraint_region_pose.pose.position.x = 0.55;
  pcm.constraint_region_pose.pose.position.y = 0.2;
  pcm.constraint_region_pose.pose.position.z = 1.25;
  pcm.constraint_region_pose.pose.orientation.x = 0.0;
  pcm.constraint_region_pose.pose.orientation.y = 0.0;
  pcm.constraint_region_pose.pose.orientation.z = 0.0;
  pcm.constraint_region_pose.pose.orientation.w = 1.0;
  pcm.weight = 1.0;
  
  EXPECT_TRUE(pc.configure(pcm));
  
  kinematic_constraints::OrientationConstraint oc(kmodel_, tf);
  moveit_msgs::OrientationConstraint ocm;
  
  ocm.link_name = "l_wrist_roll_link";
  ocm.orientation.header.frame_id = kmodel_->getModelFrame();
  ocm.orientation.quaternion.x = 0.0;
  ocm.orientation.quaternion.y = 0.0;
  ocm.orientation.quaternion.z = 0.0;
  ocm.orientation.quaternion.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.4;
  ocm.weight = 1.0;
  
  EXPECT_TRUE(oc.configure(ocm));

  kinematics_plugin_loader::KinematicsPluginLoader kinematics_loader;
  kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_loader.getLoaderFunction();

  kinematic_constraints::IKConstraintSampler iks1(kmodel_->getJointModelGroup("left_arm"), kinematics_allocator,
                                                  kinematic_constraints::IKSamplingPose(pc, oc));
  for (int t = 0 ; t < 100 ; ++t)
  {
    std::vector<double> values;
    EXPECT_TRUE(iks1.sample(values, ks, 100));
    ks.getJointStateGroup("left_arm")->setStateValues(values);
    double distance(0.0);
    EXPECT_TRUE(pc.decide(ks,distance));
    EXPECT_TRUE(oc.decide(ks,distance));
  }
  
  kinematic_constraints::IKConstraintSampler iks2(kmodel_->getJointModelGroup("left_arm"), kinematics_allocator,
                                                  kinematic_constraints::IKSamplingPose(pc));
  for (int t = 0 ; t < 100 ; ++t)
  {
    std::vector<double> values;
    double distance(0.0);
    EXPECT_TRUE(iks2.sample(values, ks, 100));
    ks.getJointStateGroup("left_arm")->setStateValues(values);
    EXPECT_TRUE(pc.decide(ks,distance));
  }
  
  kinematic_constraints::IKConstraintSampler iks3(kmodel_->getJointModelGroup("left_arm"), kinematics_allocator,
                                                  kinematic_constraints::IKSamplingPose(oc));
  for (int t = 0 ; t < 100 ; ++t)
  {
    std::vector<double> values;
    double distance(0.0);
    EXPECT_TRUE(iks3.sample(values, ks, 100));
    ks.getJointStateGroup("left_arm")->setStateValues(values);
    EXPECT_TRUE(oc.decide(ks,distance));
  }
  
  
  // test the automatic construction of constraint sampler
  moveit_msgs::Constraints c;
  c.position_constraints.push_back(pcm);
  c.orientation_constraints.push_back(ocm);
  
  kinematic_constraints::ConstraintSamplerPtr s = kinematic_constraints::constructConstraintsSampler
    (kmodel_->getJointModelGroup("left_arm"), c, kmodel_, tf, kinematics_allocator);
  EXPECT_TRUE(s.get() != NULL);
  for (int t = 0 ; t < 100 ; ++t)
  {
    std::vector<double> values;
    double distance(0.0);
    EXPECT_TRUE(s->sample(values, ks, 100));
    ks.getJointStateGroup("left_arm")->setStateValues(values);
    EXPECT_TRUE(pc.decide(ks,distance));
    EXPECT_TRUE(oc.decide(ks,distance));
  }
}

TEST_F(ConstraintSamplerTestBase, GenericConstraintsSampler)
{
  moveit_msgs::Constraints c;
  
  moveit_msgs::PositionConstraint pcm;
  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region_shape.type = moveit_msgs::Shape::SPHERE;
  pcm.constraint_region_shape.dimensions.push_back(0.001);
  
  pcm.constraint_region_pose.header.frame_id = kmodel_->getModelFrame();
  pcm.constraint_region_pose.pose.position.x = 0.55;
  pcm.constraint_region_pose.pose.position.y = 0.2;
  pcm.constraint_region_pose.pose.position.z = 1.25;
  pcm.constraint_region_pose.pose.orientation.x = 0.0;
  pcm.constraint_region_pose.pose.orientation.y = 0.0;
  pcm.constraint_region_pose.pose.orientation.z = 0.0;
  pcm.constraint_region_pose.pose.orientation.w = 1.0;
  pcm.weight = 1.0;
  c.position_constraints.push_back(pcm);
  
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "l_wrist_roll_link";
  ocm.orientation.header.frame_id = kmodel_->getModelFrame();
  ocm.orientation.quaternion.x = 0.0;
  ocm.orientation.quaternion.y = 0.0;
  ocm.orientation.quaternion.z = 0.0;
  ocm.orientation.quaternion.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.4;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);
  
  ocm.link_name = "r_wrist_roll_link";
  ocm.orientation.header.frame_id = kmodel_->getModelFrame();
  ocm.orientation.quaternion.x = 0.0;
  ocm.orientation.quaternion.y = 0.0;
  ocm.orientation.quaternion.z = 0.0;
  ocm.orientation.quaternion.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = 0.01;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);
  
  kinematics_plugin_loader::KinematicsPluginLoader kinematics_loader;
  kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_loader.getLoaderFunction();
  kinematic_constraints::KinematicsSubgroupAllocator sa;
  sa[kmodel_->getJointModelGroup("left_arm")] = kinematics_allocator;
  sa[kmodel_->getJointModelGroup("right_arm")] = kinematics_allocator;
  
  planning_models::TransformsPtr tf = psm_->getPlanningScene()->getTransforms();
  kinematic_constraints::ConstraintSamplerPtr s = kinematic_constraints::constructConstraintsSampler
    (kmodel_->getJointModelGroup("arms"), c, kmodel_, tf, kinematic_constraints::KinematicsAllocator(), sa);
  
  EXPECT_TRUE(s.get() != NULL);
  
  kinematic_constraints::KinematicConstraintSet kset(kmodel_, tf);
  kset.add(c);
  
  planning_models::KinematicState ks(kmodel_);
  ks.setToDefaultValues();
  for (int t = 0 ; t < 100 ; ++t)
  {
    double distance;
    std::vector<double> values;
    EXPECT_TRUE(s->sample(values, ks, 1000));
    ks.getJointStateGroup("arms")->setStateValues(values);
    EXPECT_TRUE(kset.decide(ks, distance));
  }
}

TEST_F(ConstraintSamplerTestBase, DisplayGenericConstraintsSamples)
{
  moveit_msgs::Constraints c;
  /*
  moveit_msgs::PositionConstraint pcm;
  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region_shape.type = moveit_msgs::Shape::BOX;
  pcm.constraint_region_shape.dimensions.push_back(0.5);
  pcm.constraint_region_shape.dimensions.push_back(1.0);
  pcm.constraint_region_shape.dimensions.push_back(0.7);
  
  pcm.constraint_region_pose.header.frame_id = kmodel_->getModelFrame();
  pcm.constraint_region_pose.pose.position.x = 0.8;
  pcm.constraint_region_pose.pose.position.y = 0.4;
  pcm.constraint_region_pose.pose.position.z = 0.7;
  pcm.constraint_region_pose.pose.orientation.x = 0.0;
  pcm.constraint_region_pose.pose.orientation.y = 0.0;
  pcm.constraint_region_pose.pose.orientation.z = 0.0;
  pcm.constraint_region_pose.pose.orientation.w = 1.0;
  pcm.weight = 1.0;
  c.position_constraints.push_back(pcm);
  */
  moveit_msgs::PositionConstraint pcm2;
  pcm2.link_name = "r_wrist_roll_link";
  pcm2.target_point_offset.x = 0.7;
  pcm2.target_point_offset.y = 0;
  pcm2.target_point_offset.z = 0;
  pcm2.constraint_region_shape.type = moveit_msgs::Shape::BOX;
  pcm2.constraint_region_shape.dimensions.push_back(0.01);
  pcm2.constraint_region_shape.dimensions.push_back(0.01);
  pcm2.constraint_region_shape.dimensions.push_back(0.01);
  
  pcm2.constraint_region_pose.header.frame_id = "l_wrist_roll_link";
  pcm2.constraint_region_pose.pose.position.x = 0.0;
  pcm2.constraint_region_pose.pose.position.y = 0.0;
  pcm2.constraint_region_pose.pose.position.z = 0.0;
  pcm2.constraint_region_pose.pose.orientation.x = 0.0;
  pcm2.constraint_region_pose.pose.orientation.y = 0.0;
  pcm2.constraint_region_pose.pose.orientation.z = 0.0;
  pcm2.constraint_region_pose.pose.orientation.w = 1.0;
  pcm2.weight = 1.0;
  c.position_constraints.push_back(pcm2);


  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "l_wrist_roll_link";
  ocm.orientation.header.frame_id = kmodel_->getModelFrame();
  ocm.orientation.quaternion.x = 0.5;
  ocm.orientation.quaternion.y = 0.5;
  ocm.orientation.quaternion.z = 0.5;
  ocm.orientation.quaternion.w = 0.5;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = M_PI;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);

  ocm.link_name = "r_wrist_roll_link";
  ocm.orientation.header.frame_id = "l_wrist_roll_link";
  ocm.orientation.quaternion.x = 0.0;
  ocm.orientation.quaternion.y = 0.0;
  ocm.orientation.quaternion.z = 1.0;
  ocm.orientation.quaternion.w = 0.0;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = 0.01;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);

  kinematics_plugin_loader::KinematicsPluginLoader kinematics_loader;
  kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_loader.getLoaderFunction();
  kinematic_constraints::KinematicsSubgroupAllocator sa;
  sa[kmodel_->getJointModelGroup("left_arm")] = kinematics_allocator;
  sa[kmodel_->getJointModelGroup("right_arm")] = kinematics_allocator;

  ros::NodeHandle nh;
  ros::Publisher pub_state = nh.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 20);
  
  planning_models::TransformsPtr tf = psm_->getPlanningScene()->getTransforms();
  kinematic_constraints::ConstraintSamplerPtr s = kinematic_constraints::constructConstraintsSampler
    (kmodel_->getJointModelGroup("arms"), c, kmodel_, tf, kinematic_constraints::KinematicsAllocator(), sa);
  
  EXPECT_TRUE(s.get() != NULL);
  
  kinematic_constraints::KinematicConstraintSet kset(kmodel_, tf);
  kset.add(c);
  
  planning_models::KinematicState ks(kmodel_);
  ks.setToDefaultValues();
  for (int t = 0 ; t < 150000 ; ++t)
  {
    double distance;
    std::vector<double> values;
    if ((s->sample(values, ks, 3)))
    {
      ks.getJointStateGroup("arms")->setStateValues(values);
      bool valid = kset.decide(ks, distance);
      EXPECT_TRUE(valid);
      if (valid)
      {
        moveit_msgs::DisplayTrajectory d;
        d.model_id = kmodel_->getName();
        planning_models::kinematicStateToRobotState(ks, d.robot_state);
        pub_state.publish(d);
        //        ros::WallDuration(1.0).sleep();
      }
    }
  }
}

/*
TEST_F(ConstraintSamplerTestBase, VisibilityConstraint)
{
  ros::NodeHandle nh;
  planning_models::KinematicState ks(kmodel_);
  ks.setDefaultValues();
  planning_models::TransformsPtr tf = psm_->getPlanningScene()->getTransforms();
  
  kinematic_constraints::VisibilityConstraint vc(kmodel_, tf);
  moveit_msgs::VisibilityConstraint vcm;
  vcm.target_radius = 0.1;
  vcm.cone_sides = 3;
  
  vcm.target_pose.header.frame_id = "r_wrist_roll_link";
  vcm.target_pose.pose.position.x = 0;
  vcm.target_pose.pose.position.y = 0;
  vcm.target_pose.pose.position.z = 0;
  vcm.target_pose.pose.orientation.x = 0;
  vcm.target_pose.pose.orientation.y = 0;
  vcm.target_pose.pose.orientation.z = 0;
  vcm.target_pose.pose.orientation.w = 1;
  
  vcm.sensor_pose.header.frame_id = "head_pan_link";
  vcm.sensor_pose.pose.position.x = 0;
  vcm.sensor_pose.pose.position.y = 0;
  vcm.sensor_pose.pose.position.z = 0;
  vcm.sensor_pose.pose.orientation.x = 0;
  vcm.sensor_pose.pose.orientation.y = 0;
  vcm.sensor_pose.pose.orientation.z = 0;
  vcm.sensor_pose.pose.orientation.w = 1;
  vcm.weight = 1.0;
  
  EXPECT_TRUE(vc.configure(vcm));
  
  shapes::Mesh *m = vc.getVisibilityCone(ks);
  visualization_msgs::Marker mk;
  shapes::constructMarkerFromShape(m, mk);
  delete m;
  mk.header.frame_id = kmodel_->getModelFrame();
  mk.header.stamp = ros::Time::now();
  mk.ns = "constraints";
  mk.id = 1;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.position.x = 0;
  mk.pose.position.y = 0;
  mk.pose.position.z = 0;
  mk.pose.orientation.x = 0;
  mk.pose.orientation.y = 0;
  mk.pose.orientation.z = 0;
  mk.pose.orientation.w = 1;
  mk.lifetime = ros::Duration(60);
  mk.color.a = 0.5;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  sleep(1);
  pub.publish(mk);
  ros::spin();
  }
*/
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ompl_planning");
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  bool result = RUN_ALL_TESTS();
  sleep(1);
  return result;
  
}
