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
 *   * Neither the name of Willow Garage nor the names of its
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

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <moveit/constraint_samplers/union_constraint_sampler.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/constraint_samplers/constraint_sampler_tools.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_resources/config.h>

#include <geometric_shapes/shape_operations.h>
#include <visualization_msgs/MarkerArray.h>

#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <boost/bind.hpp>
#include <boost/filesystem/path.hpp>

#include "pr2_arm_kinematics_plugin.h"

class LoadPlanningModelsPr2 : public testing::Test
{
protected:
  kinematics::KinematicsBasePtr getKinematicsSolverRightArm(const robot_model::JointModelGroup* jmg)
  {
    {
      return pr2_kinematics_plugin_right_arm_;
    }
  }

  kinematics::KinematicsBasePtr getKinematicsSolverLeftArm(const robot_model::JointModelGroup* jmg)
  {
    {
      return pr2_kinematics_plugin_left_arm_;
    }
  }

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
    kmodel.reset(new robot_model::RobotModel(urdf_model, srdf_model));

    pr2_kinematics_plugin_right_arm_.reset(new pr2_arm_kinematics::PR2ArmKinematicsPlugin);

    pr2_kinematics_plugin_right_arm_->setRobotModel(urdf_model);
    pr2_kinematics_plugin_right_arm_->initialize("", "right_arm", "torso_lift_link", "r_wrist_roll_link", .01);

    pr2_kinematics_plugin_left_arm_.reset(new pr2_arm_kinematics::PR2ArmKinematicsPlugin);

    pr2_kinematics_plugin_left_arm_->setRobotModel(urdf_model);
    pr2_kinematics_plugin_left_arm_->initialize("", "left_arm", "torso_lift_link", "l_wrist_roll_link", .01);

    func_right_arm = boost::bind(&LoadPlanningModelsPr2::getKinematicsSolverRightArm, this, _1);
    func_left_arm = boost::bind(&LoadPlanningModelsPr2::getKinematicsSolverLeftArm, this, _1);

    std::map<std::string, robot_model::SolverAllocatorFn> allocators;
    allocators["right_arm"] = func_right_arm;
    allocators["left_arm"] = func_left_arm;
    allocators["whole_body"] = func_right_arm;
    allocators["base"] = func_left_arm;

    kmodel->setKinematicsAllocators(allocators);

    ps.reset(new planning_scene::PlanningScene(kmodel));
  };

  void TearDown() override
  {
  }

protected:
  urdf::ModelInterfaceSharedPtr urdf_model;
  srdf::ModelSharedPtr srdf_model;
  robot_model::RobotModelPtr kmodel;
  planning_scene::PlanningScenePtr ps;
  pr2_arm_kinematics::PR2ArmKinematicsPluginPtr pr2_kinematics_plugin_right_arm_;
  pr2_arm_kinematics::PR2ArmKinematicsPluginPtr pr2_kinematics_plugin_left_arm_;
  robot_model::SolverAllocatorFn func_right_arm;
  robot_model::SolverAllocatorFn func_left_arm;
};

TEST_F(LoadPlanningModelsPr2, JointConstraintsSamplerSimple)
{
  robot_state::RobotState ks(kmodel);
  ks.setToDefaultValues();

  robot_state::RobotState ks_const(kmodel);
  ks_const.setToDefaultValues();

  kinematic_constraints::JointConstraint jc1(kmodel);
  moveit_msgs::JointConstraint jcm1;
  // leaving off joint name
  jcm1.position = 0.42;
  jcm1.tolerance_above = 0.01;
  jcm1.tolerance_below = 0.05;
  jcm1.weight = 1.0;
  EXPECT_FALSE(jc1.configure(jcm1));

  std::vector<kinematic_constraints::JointConstraint> js;
  js.push_back(jc1);

  constraint_samplers::JointConstraintSampler jcs(ps, "right_arm");
  // no valid constraints
  EXPECT_FALSE(jcs.configure(js));

  // testing that this does the right thing
  jcm1.joint_name = "r_shoulder_pan_joint";
  EXPECT_TRUE(jc1.configure(jcm1));
  js.push_back(jc1);
  EXPECT_TRUE(jcs.configure(js));
  EXPECT_EQ(jcs.getConstrainedJointCount(), 1);
  EXPECT_EQ(jcs.getUnconstrainedJointCount(), 6);
  EXPECT_TRUE(jcs.sample(ks, ks, 1));

  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(jcs.sample(ks, ks_const, 1));
    EXPECT_TRUE(jc1.decide(ks).satisfied);
  }

  // redoing the configure leads to 6 unconstrained variables as well
  EXPECT_TRUE(jcs.configure(js));
  EXPECT_EQ(jcs.getUnconstrainedJointCount(), 6);

  kinematic_constraints::JointConstraint jc2(kmodel);

  moveit_msgs::JointConstraint jcm2;
  jcm2.joint_name = "r_shoulder_pan_joint";
  jcm2.position = 0.54;
  jcm2.tolerance_above = 0.01;
  jcm2.tolerance_below = 0.01;
  jcm2.weight = 1.0;
  EXPECT_TRUE(jc2.configure(jcm2));
  js.push_back(jc2);

  // creating a constraint that conflicts with the other (leaves no sampleable region)
  EXPECT_FALSE(jcs.configure(js));
  EXPECT_FALSE(jcs.sample(ks, ks_const, 1));

  // we can't sample for a different group
  constraint_samplers::JointConstraintSampler jcs2(ps, "arms");
  jcs2.configure(js);
  EXPECT_FALSE(jcs2.sample(ks, ks_const, 1));

  // not ok to not have any references to joints in this group in the constraints
  constraint_samplers::JointConstraintSampler jcs3(ps, "left_arm");
  EXPECT_FALSE(jcs3.configure(js));

  // testing that the most restrictive bounds are used
  js.clear();

  jcm1.position = .4;
  jcm1.tolerance_above = .05;
  jcm1.tolerance_below = .05;
  jcm2.position = .4;
  jcm2.tolerance_above = .1;
  jcm2.tolerance_below = .1;
  EXPECT_TRUE(jc1.configure(jcm1));
  EXPECT_TRUE(jc2.configure(jcm2));
  js.push_back(jc1);
  js.push_back(jc2);

  EXPECT_TRUE(jcs.configure(js));

  // should always be within narrower constraints
  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(jcs.sample(ks, ks_const, 1));
    EXPECT_TRUE(jc1.decide(ks).satisfied);
  }

  // too narrow range outside of joint limits
  js.clear();

  jcm1.position = -3.1;
  jcm1.tolerance_above = .05;
  jcm1.tolerance_below = .05;

  // the joint configuration corrects this
  EXPECT_TRUE(jc1.configure(jcm1));
  js.push_back(jc1);
  EXPECT_TRUE(jcs.configure(js));

  // partially overlapping regions will also work
  js.clear();
  jcm1.position = .35;
  jcm1.tolerance_above = .05;
  jcm1.tolerance_below = .05;
  jcm2.position = .45;
  jcm2.tolerance_above = .05;
  jcm2.tolerance_below = .05;
  EXPECT_TRUE(jc1.configure(jcm1));
  EXPECT_TRUE(jc2.configure(jcm2));
  js.push_back(jc1);
  js.push_back(jc2);

  // leads to a min and max of .4, so all samples should be exactly .4
  EXPECT_TRUE(jcs.configure(js));
  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(jcs.sample(ks, ks_const, 1));
    std::map<std::string, double> var_values;
    EXPECT_NEAR(ks.getVariablePosition("r_shoulder_pan_joint"), .4, std::numeric_limits<double>::epsilon());
    EXPECT_TRUE(jc1.decide(ks).satisfied);
    EXPECT_TRUE(jc2.decide(ks).satisfied);
  }

  // this leads to a larger sampleable region
  jcm1.position = .38;
  jcm2.position = .42;
  EXPECT_TRUE(jc1.configure(jcm1));
  EXPECT_TRUE(jc2.configure(jcm2));
  js.push_back(jc1);
  js.push_back(jc2);
  EXPECT_TRUE(jcs.configure(js));
  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(jcs.sample(ks, ks_const, 1));
    EXPECT_TRUE(jc1.decide(ks).satisfied);
    EXPECT_TRUE(jc2.decide(ks).satisfied);
  }
}

TEST_F(LoadPlanningModelsPr2, IKConstraintsSamplerSimple)
{
  robot_state::Transforms& tf = ps->getTransformsNonConst();

  kinematic_constraints::PositionConstraint pc(kmodel);
  moveit_msgs::PositionConstraint pcm;

  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[0] = 0.001;

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.55;
  pcm.constraint_region.primitive_poses[0].position.y = 0.2;
  pcm.constraint_region.primitive_poses[0].position.z = 1.25;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  EXPECT_FALSE(pc.configure(pcm, tf));

  constraint_samplers::IKConstraintSampler ik_bad(ps, "l_arm");
  EXPECT_FALSE(ik_bad.isValid());

  constraint_samplers::IKConstraintSampler iks(ps, "left_arm");
  EXPECT_FALSE(iks.configure(constraint_samplers::IKSamplingPose()));
  EXPECT_FALSE(iks.isValid());

  EXPECT_FALSE(iks.configure(constraint_samplers::IKSamplingPose(pc)));

  pcm.header.frame_id = kmodel->getModelFrame();
  EXPECT_TRUE(pc.configure(pcm, tf));
  EXPECT_TRUE(iks.configure(constraint_samplers::IKSamplingPose(pc)));

  // ik link not in this group
  constraint_samplers::IKConstraintSampler ik_bad_2(ps, "right_arm");
  EXPECT_FALSE(ik_bad_2.configure(constraint_samplers::IKSamplingPose(pc)));
  EXPECT_FALSE(ik_bad_2.isValid());

  // not the ik link
  pcm.link_name = "l_shoulder_pan_link";
  EXPECT_TRUE(pc.configure(pcm, tf));
  EXPECT_FALSE(iks.configure(constraint_samplers::IKSamplingPose(pc)));

  // solver for base doesn't cover group
  constraint_samplers::IKConstraintSampler ik_base(ps, "base");
  pcm.link_name = "l_wrist_roll_link";
  EXPECT_TRUE(pc.configure(pcm, tf));
  EXPECT_FALSE(ik_base.configure(constraint_samplers::IKSamplingPose(pc)));
  EXPECT_FALSE(ik_base.isValid());

  // shouldn't work as no direct constraint solver
  constraint_samplers::IKConstraintSampler ik_arms(ps, "arms");
  EXPECT_FALSE(iks.isValid());
}

TEST_F(LoadPlanningModelsPr2, OrientationConstraintsSampler)
{
  robot_state::RobotState ks(kmodel);
  ks.setToDefaultValues();
  ks.update();
  robot_state::RobotState ks_const(kmodel);
  ks_const.setToDefaultValues();
  ks_const.update();

  robot_state::Transforms& tf = ps->getTransformsNonConst();

  kinematic_constraints::OrientationConstraint oc(kmodel);
  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = ocm.link_name;
  ocm.orientation.x = 0.5;
  ocm.orientation.y = 0.5;
  ocm.orientation.z = 0.5;
  ocm.orientation.w = 0.5;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = 0.01;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;

  EXPECT_TRUE(oc.configure(ocm, tf));

  bool p1 = oc.decide(ks).satisfied;
  EXPECT_FALSE(p1);

  ocm.header.frame_id = kmodel->getModelFrame();
  EXPECT_TRUE(oc.configure(ocm, tf));

  constraint_samplers::IKConstraintSampler iks(ps, "right_arm");
  EXPECT_TRUE(iks.configure(constraint_samplers::IKSamplingPose(oc)));
  for (int t = 0; t < 100; ++t)
  {
    ks.update();
    EXPECT_TRUE(iks.sample(ks, ks_const, 100));
    EXPECT_TRUE(oc.decide(ks).satisfied);
  }
}

TEST_F(LoadPlanningModelsPr2, IKConstraintsSamplerValid)
{
  robot_state::RobotState ks(kmodel);
  ks.setToDefaultValues();
  ks.update();
  robot_state::RobotState ks_const(kmodel);
  ks_const.setToDefaultValues();
  ks_const.update();

  robot_state::Transforms& tf = ps->getTransformsNonConst();

  kinematic_constraints::PositionConstraint pc(kmodel);
  moveit_msgs::PositionConstraint pcm;

  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[0] = 0.001;

  pcm.header.frame_id = kmodel->getModelFrame();

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.55;
  pcm.constraint_region.primitive_poses[0].position.y = 0.2;
  pcm.constraint_region.primitive_poses[0].position.z = 1.25;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  EXPECT_TRUE(pc.configure(pcm, tf));

  kinematic_constraints::OrientationConstraint oc(kmodel);
  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "l_wrist_roll_link";
  ocm.header.frame_id = kmodel->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.4;
  ocm.weight = 1.0;

  EXPECT_TRUE(oc.configure(ocm, tf));

  constraint_samplers::IKConstraintSampler iks1(ps, "left_arm");
  EXPECT_TRUE(iks1.configure(constraint_samplers::IKSamplingPose(pc, oc)));
  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(iks1.sample(ks, ks_const, 100));
    EXPECT_TRUE(pc.decide(ks).satisfied);
    EXPECT_TRUE(oc.decide(ks).satisfied);
  }

  constraint_samplers::IKConstraintSampler iks2(ps, "left_arm");
  EXPECT_TRUE(iks2.configure(constraint_samplers::IKSamplingPose(pc)));
  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(iks2.sample(ks, ks_const, 100));
    EXPECT_TRUE(pc.decide(ks).satisfied);
  }

  constraint_samplers::IKConstraintSampler iks3(ps, "left_arm");
  EXPECT_TRUE(iks3.configure(constraint_samplers::IKSamplingPose(oc)));
  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(iks3.sample(ks, ks_const, 100));
    EXPECT_TRUE(oc.decide(ks).satisfied);
  }
}

TEST_F(LoadPlanningModelsPr2, UnionConstraintSampler)
{
  robot_state::RobotState ks(kmodel);
  ks.setToDefaultValues();
  ks.update();

  robot_state::RobotState ks_const(kmodel);
  ks_const.setToDefaultValues();
  ks_const.update();

  robot_state::Transforms& tf = ps->getTransformsNonConst();

  kinematic_constraints::JointConstraint jc1(kmodel);

  std::map<std::string, double> state_values;

  moveit_msgs::JointConstraint torso_constraint;
  torso_constraint.joint_name = "torso_lift_joint";
  torso_constraint.position = ks.getVariablePosition("torso_lift_joint");
  torso_constraint.tolerance_above = 0.01;
  torso_constraint.tolerance_below = 0.01;
  torso_constraint.weight = 1.0;
  EXPECT_TRUE(jc1.configure(torso_constraint));

  kinematic_constraints::JointConstraint jc2(kmodel);
  moveit_msgs::JointConstraint jcm2;
  jcm2.joint_name = "r_elbow_flex_joint";
  jcm2.position = ks.getVariablePosition("r_elbow_flex_joint");
  jcm2.tolerance_above = 0.01;
  jcm2.tolerance_below = 0.01;
  jcm2.weight = 1.0;
  EXPECT_TRUE(jc2.configure(jcm2));

  moveit_msgs::PositionConstraint pcm;

  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[0] = 0.001;

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.55;
  pcm.constraint_region.primitive_poses[0].position.y = 0.2;
  pcm.constraint_region.primitive_poses[0].position.z = 1.25;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  pcm.header.frame_id = kmodel->getModelFrame();

  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "l_wrist_roll_link";
  ocm.header.frame_id = kmodel->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.4;
  ocm.weight = 1.0;

  std::vector<kinematic_constraints::JointConstraint> js;
  js.push_back(jc1);

  constraint_samplers::JointConstraintSamplerPtr jcsp(
      new constraint_samplers::JointConstraintSampler(ps, "arms_and_torso"));
  EXPECT_TRUE(jcsp->configure(js));

  std::vector<kinematic_constraints::JointConstraint> js2;
  js2.push_back(jc2);

  constraint_samplers::JointConstraintSamplerPtr jcsp2(new constraint_samplers::JointConstraintSampler(ps, "arms"));
  EXPECT_TRUE(jcsp2->configure(js2));

  kinematic_constraints::PositionConstraint pc(kmodel);
  EXPECT_TRUE(pc.configure(pcm, tf));

  kinematic_constraints::OrientationConstraint oc(kmodel);
  EXPECT_TRUE(oc.configure(ocm, tf));

  constraint_samplers::IKConstraintSamplerPtr iksp(new constraint_samplers::IKConstraintSampler(ps, "left_arm"));
  EXPECT_TRUE(iksp->configure(constraint_samplers::IKSamplingPose(pc, oc)));
  EXPECT_TRUE(iksp->isValid());

  std::vector<constraint_samplers::ConstraintSamplerPtr> cspv;
  cspv.push_back(jcsp2);
  cspv.push_back(iksp);
  cspv.push_back(jcsp);

  constraint_samplers::UnionConstraintSampler ucs(ps, "arms_and_torso", cspv);

  // should have reordered to place whole body first
  constraint_samplers::JointConstraintSampler* jcs =
      dynamic_cast<constraint_samplers::JointConstraintSampler*>(ucs.getSamplers()[0].get());
  EXPECT_TRUE(jcs);
  EXPECT_EQ(jcs->getJointModelGroup()->getName(), "arms_and_torso");

  constraint_samplers::JointConstraintSampler* jcs2 =
      dynamic_cast<constraint_samplers::JointConstraintSampler*>(ucs.getSamplers()[1].get());
  EXPECT_TRUE(jcs2);
  EXPECT_EQ(jcs2->getJointModelGroup()->getName(), "arms");

  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(ucs.sample(ks, ks_const, 100));
    ks.update();
    ks.updateLinkTransforms();  // Returned samples have dirty link transforms.
    ks_const.update();
    EXPECT_TRUE(jc1.decide(ks).satisfied);
    EXPECT_TRUE(jc2.decide(ks).satisfied);
    EXPECT_TRUE(pc.decide(ks).satisfied);
  }

  // now we add a position constraint on right arm
  pcm.link_name = "r_wrist_roll_link";
  ocm.link_name = "r_wrist_roll_link";
  cspv.clear();

  kinematic_constraints::PositionConstraint pc2(kmodel);
  EXPECT_TRUE(pc2.configure(pcm, tf));

  kinematic_constraints::OrientationConstraint oc2(kmodel);
  EXPECT_TRUE(oc2.configure(ocm, tf));

  constraint_samplers::IKConstraintSamplerPtr iksp2(new constraint_samplers::IKConstraintSampler(ps, "right_arm"));
  EXPECT_TRUE(iksp2->configure(constraint_samplers::IKSamplingPose(pc2, oc2)));
  EXPECT_TRUE(iksp2->isValid());

  // totally disjoint, so should break ties based on alphabetical order
  cspv.clear();
  cspv.push_back(iksp2);
  cspv.push_back(iksp);

  constraint_samplers::UnionConstraintSampler ucs2(ps, "arms_and_torso", cspv);

  constraint_samplers::IKConstraintSampler* ikcs_test =
      dynamic_cast<constraint_samplers::IKConstraintSampler*>(ucs2.getSamplers()[0].get());
  ASSERT_TRUE(ikcs_test);
  EXPECT_EQ(ikcs_test->getJointModelGroup()->getName(), "left_arm");

  // now we make left depends on right, right should stay first
  pcm.link_name = "l_wrist_roll_link";
  ocm.link_name = "l_wrist_roll_link";
  pcm.header.frame_id = "r_wrist_roll_link";
  ocm.header.frame_id = "r_wrist_roll_link";
  EXPECT_TRUE(pc.configure(pcm, tf));
  EXPECT_TRUE(oc.configure(ocm, tf));
  ASSERT_TRUE(iksp->configure(constraint_samplers::IKSamplingPose(pc, oc)));

  cspv.clear();
  cspv.push_back(iksp2);
  cspv.push_back(iksp);

  constraint_samplers::UnionConstraintSampler ucs3(ps, "arms_and_torso", cspv);

  ikcs_test = dynamic_cast<constraint_samplers::IKConstraintSampler*>(ucs3.getSamplers()[0].get());
  EXPECT_TRUE(ikcs_test);
  EXPECT_EQ(ikcs_test->getJointModelGroup()->getName(), "right_arm");
}

TEST_F(LoadPlanningModelsPr2, PoseConstraintSamplerManager)
{
  robot_state::RobotState ks(kmodel);
  ks.setToDefaultValues();
  ks.update();
  robot_state::RobotState ks_const(kmodel);
  ks_const.setToDefaultValues();
  ks_const.update();

  kinematic_constraints::PositionConstraint pc(kmodel);

  moveit_msgs::PositionConstraint pcm;
  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[0] = 0.001;

  pcm.header.frame_id = kmodel->getModelFrame();

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.55;
  pcm.constraint_region.primitive_poses[0].position.y = 0.2;
  pcm.constraint_region.primitive_poses[0].position.z = 1.25;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "l_wrist_roll_link";
  ocm.header.frame_id = kmodel->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.4;
  ocm.weight = 1.0;

  // test the automatic construction of constraint sampler
  moveit_msgs::Constraints c;
  c.position_constraints.push_back(pcm);
  c.orientation_constraints.push_back(ocm);

  constraint_samplers::ConstraintSamplerPtr s =
      constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "left_arm", c);
  EXPECT_TRUE(s != nullptr);
  constraint_samplers::IKConstraintSampler* iks = dynamic_cast<constraint_samplers::IKConstraintSampler*>(s.get());
  ASSERT_TRUE(iks);
  ASSERT_TRUE(static_cast<bool>(iks->getPositionConstraint()));
  ASSERT_TRUE(static_cast<bool>(iks->getOrientationConstraint()));

  static const int NT = 100;
  int succ = 0;
  for (int t = 0; t < NT; ++t)
  {
    EXPECT_TRUE(s->sample(ks, ks_const, 100));
    EXPECT_TRUE(iks->getPositionConstraint()->decide(ks).satisfied);
    EXPECT_TRUE(iks->getOrientationConstraint()->decide(ks).satisfied);
    if (s->sample(ks, ks_const, 1))
      succ++;
  }
  ROS_INFO("Success rate for IK Constraint Sampler with position & orientation constraints for one arm: %lf",
           (double)succ / (double)NT);

  // add additional ocm with smaller volume
  ocm.absolute_x_axis_tolerance = 0.1;

  c.orientation_constraints.push_back(ocm);

  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "left_arm", c);
  EXPECT_TRUE(s != nullptr);

  iks = dynamic_cast<constraint_samplers::IKConstraintSampler*>(s.get());
  ASSERT_TRUE(iks);
  ASSERT_TRUE(static_cast<bool>(iks->getOrientationConstraint()));
  EXPECT_NEAR(iks->getOrientationConstraint()->getXAxisTolerance(), .1, .0001);
}

TEST_F(LoadPlanningModelsPr2, JointVersusPoseConstraintSamplerManager)
{
  robot_state::RobotState ks(kmodel);
  ks.setToDefaultValues();
  ks.update();

  moveit_msgs::Constraints con;
  con.joint_constraints.resize(1);

  con.joint_constraints[0].joint_name = "l_shoulder_pan_joint";
  con.joint_constraints[0].position = 0.54;
  con.joint_constraints[0].tolerance_above = 0.01;
  con.joint_constraints[0].tolerance_below = 0.01;
  con.joint_constraints[0].weight = 1.0;

  constraint_samplers::ConstraintSamplerPtr s =
      constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "right_arm", con);
  EXPECT_FALSE(static_cast<bool>(s));
  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "left_arm", con);
  EXPECT_TRUE(static_cast<bool>(s));

  con.joint_constraints.resize(7);

  con.joint_constraints[1].joint_name = "l_shoulder_lift_joint";
  con.joint_constraints[1].position = 0.54;
  con.joint_constraints[1].tolerance_above = 0.01;
  con.joint_constraints[1].tolerance_below = 0.01;
  con.joint_constraints[1].weight = 1.0;

  con.joint_constraints[2].joint_name = "l_upper_arm_roll_joint";
  con.joint_constraints[2].position = 0.54;
  con.joint_constraints[2].tolerance_above = 0.01;
  con.joint_constraints[2].tolerance_below = 0.01;
  con.joint_constraints[2].weight = 1.0;

  con.joint_constraints[3].joint_name = "l_elbow_flex_joint";
  con.joint_constraints[3].position = -0.54;
  con.joint_constraints[3].tolerance_above = 0.01;
  con.joint_constraints[3].tolerance_below = 0.01;
  con.joint_constraints[3].weight = 1.0;

  con.joint_constraints[4].joint_name = "l_forearm_roll_joint";
  con.joint_constraints[4].position = 0.54;
  con.joint_constraints[4].tolerance_above = 0.01;
  con.joint_constraints[4].tolerance_below = 0.01;
  con.joint_constraints[4].weight = 1.0;

  con.joint_constraints[5].joint_name = "l_wrist_flex_joint";
  con.joint_constraints[5].position = -0.54;
  con.joint_constraints[5].tolerance_above = 0.05;
  con.joint_constraints[5].tolerance_below = 0.05;
  con.joint_constraints[5].weight = 1.0;

  // an extra constraint on one link, but this shouldn't change anything
  con.joint_constraints[6].joint_name = "l_wrist_flex_joint";
  con.joint_constraints[6].position = -0.56;
  con.joint_constraints[6].tolerance_above = 0.01;
  con.joint_constraints[6].tolerance_below = 0.01;
  con.joint_constraints[6].weight = 1.0;

  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "left_arm", con);
  EXPECT_TRUE(static_cast<bool>(s));

  con.position_constraints.resize(1);

  // intentionally making wrong wrist
  con.position_constraints[0].link_name = "r_wrist_roll_link";
  con.position_constraints[0].target_point_offset.x = 0;
  con.position_constraints[0].target_point_offset.y = 0;
  con.position_constraints[0].target_point_offset.z = 0;
  con.position_constraints[0].constraint_region.primitives.resize(1);
  con.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  con.position_constraints[0].constraint_region.primitives[0].dimensions.resize(1);
  con.position_constraints[0].constraint_region.primitives[0].dimensions[0] = 0.001;

  con.position_constraints[0].header.frame_id = kmodel->getModelFrame();

  con.position_constraints[0].constraint_region.primitive_poses.resize(1);
  con.position_constraints[0].constraint_region.primitive_poses[0].position.x = 0.55;
  con.position_constraints[0].constraint_region.primitive_poses[0].position.y = 0.2;
  con.position_constraints[0].constraint_region.primitive_poses[0].position.z = 1.25;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.x = 0.0;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.y = 0.0;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.z = 0.0;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.w = 1.0;
  con.position_constraints[0].weight = 1.0;

  // this still works, but we should get a JointConstraintSampler
  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "left_arm", con);
  EXPECT_TRUE(static_cast<bool>(s));
  constraint_samplers::JointConstraintSampler* jcs =
      dynamic_cast<constraint_samplers::JointConstraintSampler*>(s.get());
  EXPECT_TRUE(jcs);

  con.position_constraints[0].link_name = "l_wrist_roll_link";
  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "left_arm", con);
  EXPECT_TRUE(static_cast<bool>(s));
  jcs = dynamic_cast<constraint_samplers::JointConstraintSampler*>(s.get());
  EXPECT_FALSE(jcs);
  constraint_samplers::IKConstraintSampler* iks = dynamic_cast<constraint_samplers::IKConstraintSampler*>(s.get());
  EXPECT_FALSE(iks);

  // we should get a union constraint sampler
  constraint_samplers::UnionConstraintSampler* ucs =
      dynamic_cast<constraint_samplers::UnionConstraintSampler*>(s.get());
  EXPECT_TRUE(ucs);

  con.orientation_constraints.resize(1);

  // again, screwing this up intentionally
  con.orientation_constraints[0].link_name = "r_wrist_roll_link";
  con.orientation_constraints[0].header.frame_id = kmodel->getModelFrame();
  con.orientation_constraints[0].orientation.x = 0.0;
  con.orientation_constraints[0].orientation.y = 0.0;
  con.orientation_constraints[0].orientation.z = 0.0;
  con.orientation_constraints[0].orientation.w = 1.0;
  con.orientation_constraints[0].absolute_x_axis_tolerance = 0.2;
  con.orientation_constraints[0].absolute_y_axis_tolerance = 0.1;
  con.orientation_constraints[0].absolute_z_axis_tolerance = 0.4;
  con.orientation_constraints[0].weight = 1.0;

  // we still get an IK sampler with just the position constraint
  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "left_arm", con);
  EXPECT_TRUE(static_cast<bool>(s));
  ucs = dynamic_cast<constraint_samplers::UnionConstraintSampler*>(s.get());
  ASSERT_TRUE(ucs);
  jcs = dynamic_cast<constraint_samplers::JointConstraintSampler*>(ucs->getSamplers()[0].get());
  iks = dynamic_cast<constraint_samplers::IKConstraintSampler*>(ucs->getSamplers()[1].get());

  ASSERT_TRUE(iks);
  ASSERT_TRUE(jcs);
  EXPECT_TRUE(static_cast<bool>(iks->getPositionConstraint()));
  EXPECT_FALSE(iks->getOrientationConstraint());

  con.orientation_constraints[0].link_name = "l_wrist_roll_link";

  // now they both are good
  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "left_arm", con);
  EXPECT_TRUE(static_cast<bool>(s));
  ucs = dynamic_cast<constraint_samplers::UnionConstraintSampler*>(s.get());
  iks = dynamic_cast<constraint_samplers::IKConstraintSampler*>(ucs->getSamplers()[1].get());
  ASSERT_TRUE(iks);
  EXPECT_TRUE(static_cast<bool>(iks->getPositionConstraint()));
  EXPECT_TRUE(static_cast<bool>(iks->getOrientationConstraint()));

  // now just the orientation constraint is good
  con.position_constraints[0].link_name = "r_wrist_roll_link";
  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "left_arm", con);
  ASSERT_TRUE(static_cast<bool>(s));
  ucs = dynamic_cast<constraint_samplers::UnionConstraintSampler*>(s.get());
  iks = dynamic_cast<constraint_samplers::IKConstraintSampler*>(ucs->getSamplers()[1].get());
  ASSERT_TRUE(iks);
  EXPECT_FALSE(iks->getPositionConstraint());
  EXPECT_TRUE(static_cast<bool>(iks->getOrientationConstraint()));

  // now if we constraint all the joints, we get a joint constraint sampler
  con.joint_constraints.resize(8);
  con.joint_constraints[7].joint_name = "l_wrist_roll_joint";
  con.joint_constraints[7].position = 0.54;
  con.joint_constraints[7].tolerance_above = 0.01;
  con.joint_constraints[7].tolerance_below = 0.01;
  con.joint_constraints[7].weight = 1.0;

  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "left_arm", con);
  EXPECT_TRUE(static_cast<bool>(s));
  jcs = dynamic_cast<constraint_samplers::JointConstraintSampler*>(s.get());
  ASSERT_TRUE(jcs);
}

TEST_F(LoadPlanningModelsPr2, MixedJointAndIkSamplerManager)
{
  robot_state::RobotState ks(kmodel);
  ks.setToDefaultValues();
  ks.update();
  robot_state::RobotState ks_const(kmodel);
  ks_const.setToDefaultValues();
  ks_const.update();

  moveit_msgs::Constraints con;
  con.joint_constraints.resize(1);

  con.joint_constraints[0].joint_name = "torso_lift_joint";
  con.joint_constraints[0].position = ks.getVariablePosition("torso_lift_joint");
  con.joint_constraints[0].tolerance_above = 0.01;
  con.joint_constraints[0].tolerance_below = 0.01;
  con.joint_constraints[0].weight = 1.0;

  kinematic_constraints::JointConstraint jc(kmodel);
  EXPECT_TRUE(jc.configure(con.joint_constraints[0]));

  con.position_constraints.resize(1);

  con.position_constraints[0].link_name = "l_wrist_roll_link";
  con.position_constraints[0].target_point_offset.x = 0;
  con.position_constraints[0].target_point_offset.y = 0;
  con.position_constraints[0].target_point_offset.z = 0;
  con.position_constraints[0].constraint_region.primitives.resize(1);
  con.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  con.position_constraints[0].constraint_region.primitives[0].dimensions.resize(1);
  con.position_constraints[0].constraint_region.primitives[0].dimensions[0] = 0.001;

  con.position_constraints[0].constraint_region.primitive_poses.resize(1);
  con.position_constraints[0].constraint_region.primitive_poses[0].position.x = 0.55;
  con.position_constraints[0].constraint_region.primitive_poses[0].position.y = 0.2;
  con.position_constraints[0].constraint_region.primitive_poses[0].position.z = 1.25;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.x = 0.0;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.y = 0.0;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.z = 0.0;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.w = 1.0;
  con.position_constraints[0].weight = 1.0;

  con.position_constraints[0].header.frame_id = kmodel->getModelFrame();

  con.orientation_constraints.resize(1);
  con.orientation_constraints[0].link_name = "l_wrist_roll_link";
  con.orientation_constraints[0].header.frame_id = kmodel->getModelFrame();
  con.orientation_constraints[0].orientation.x = 0.0;
  con.orientation_constraints[0].orientation.y = 0.0;
  con.orientation_constraints[0].orientation.z = 0.0;
  con.orientation_constraints[0].orientation.w = 1.0;
  con.orientation_constraints[0].absolute_x_axis_tolerance = 0.2;
  con.orientation_constraints[0].absolute_y_axis_tolerance = 0.1;
  con.orientation_constraints[0].absolute_z_axis_tolerance = 0.4;
  con.orientation_constraints[0].weight = 1.0;

  constraint_samplers::ConstraintSamplerPtr s =
      constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "arms_and_torso", con);

  constraint_samplers::UnionConstraintSampler* ucs =
      dynamic_cast<constraint_samplers::UnionConstraintSampler*>(s.get());
  ASSERT_TRUE(ucs);

  constraint_samplers::IKConstraintSampler* ikcs_test =
      dynamic_cast<constraint_samplers::IKConstraintSampler*>(ucs->getSamplers()[1].get());
  ASSERT_TRUE(ikcs_test);

  for (int t = 0; t < 1; ++t)
  {
    EXPECT_TRUE(s->sample(ks, ks_const, 100));
    EXPECT_TRUE(jc.decide(ks).satisfied);
    EXPECT_TRUE(ikcs_test->getPositionConstraint()->decide(ks).satisfied);
    EXPECT_TRUE(ikcs_test->getOrientationConstraint()->decide(ks).satisfied);
  }
}

TEST_F(LoadPlanningModelsPr2, SubgroupJointConstraintsSamplerManager)
{
  robot_state::RobotState ks(kmodel);
  ks.setToDefaultValues();
  ks.update();
  robot_state::RobotState ks_const(kmodel);
  ks_const.setToDefaultValues();
  ks_const.update();

  kinematic_constraints::JointConstraint jc1(kmodel);
  moveit_msgs::JointConstraint jcm1;
  jcm1.joint_name = "head_pan_joint";
  jcm1.position = 0.42;
  jcm1.tolerance_above = 0.01;
  jcm1.tolerance_below = 0.05;
  jcm1.weight = 1.0;
  EXPECT_TRUE(jc1.configure(jcm1));

  kinematic_constraints::JointConstraint jc2(kmodel);
  moveit_msgs::JointConstraint jcm2;
  jcm2.joint_name = "l_shoulder_pan_joint";
  jcm2.position = 0.9;
  jcm2.tolerance_above = 0.1;
  jcm2.tolerance_below = 0.05;
  jcm2.weight = 1.0;
  EXPECT_TRUE(jc2.configure(jcm2));

  kinematic_constraints::JointConstraint jc3(kmodel);
  moveit_msgs::JointConstraint jcm3;
  jcm3.joint_name = "r_wrist_roll_joint";
  jcm3.position = 0.7;
  jcm3.tolerance_above = 0.14;
  jcm3.tolerance_below = 0.005;
  jcm3.weight = 1.0;
  EXPECT_TRUE(jc3.configure(jcm3));

  kinematic_constraints::JointConstraint jc4(kmodel);
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

  constraint_samplers::JointConstraintSampler jcs(ps, "arms");
  jcs.configure(js);
  EXPECT_EQ(jcs.getConstrainedJointCount(), 2);
  EXPECT_EQ(jcs.getUnconstrainedJointCount(), 12);

  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(jcs.sample(ks, ks_const, 1));
    EXPECT_TRUE(jc2.decide(ks).satisfied);
    EXPECT_TRUE(jc3.decide(ks).satisfied);
  }

  // test the automatic construction of constraint sampler
  moveit_msgs::Constraints c;

  // no constraints should give no sampler
  constraint_samplers::ConstraintSamplerPtr s0 =
      constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "arms", c);
  EXPECT_TRUE(s0 == nullptr);

  // add the constraints
  c.joint_constraints.push_back(jcm1);
  c.joint_constraints.push_back(jcm2);
  c.joint_constraints.push_back(jcm3);
  c.joint_constraints.push_back(jcm4);

  constraint_samplers::ConstraintSamplerPtr s =
      constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "arms", c);
  EXPECT_TRUE(s != nullptr);

  // test the generated sampler
  for (int t = 0; t < 1000; ++t)
  {
    EXPECT_TRUE(s->sample(ks, ks_const, 1));
    EXPECT_TRUE(jc2.decide(ks).satisfied);
    EXPECT_TRUE(jc3.decide(ks).satisfied);
  }
}

TEST_F(LoadPlanningModelsPr2, SubgroupPoseConstraintsSampler)
{
  moveit_msgs::Constraints c;

  moveit_msgs::PositionConstraint pcm;
  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;

  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[0] = 0.001;

  pcm.header.frame_id = kmodel->getModelFrame();

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.55;
  pcm.constraint_region.primitive_poses[0].position.y = 0.2;
  pcm.constraint_region.primitive_poses[0].position.z = 1.25;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;
  c.position_constraints.push_back(pcm);

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "l_wrist_roll_link";
  ocm.header.frame_id = kmodel->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.4;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);

  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = kmodel->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = 0.01;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);

  robot_state::Transforms& tf = ps->getTransformsNonConst();
  constraint_samplers::ConstraintSamplerPtr s =
      constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps, "arms", c);
  EXPECT_TRUE(static_cast<bool>(s));
  constraint_samplers::UnionConstraintSampler* ucs =
      dynamic_cast<constraint_samplers::UnionConstraintSampler*>(s.get());
  EXPECT_TRUE(ucs);

  kinematic_constraints::KinematicConstraintSet kset(kmodel);
  kset.add(c, tf);

  robot_state::RobotState ks(kmodel);
  ks.setToDefaultValues();
  ks.update();
  robot_state::RobotState ks_const(kmodel);
  ks_const.setToDefaultValues();
  ks_const.update();

  static const int NT = 100;
  int succ = 0;
  for (int t = 0; t < NT; ++t)
  {
    EXPECT_TRUE(s->sample(ks, ks_const, 1000));
    EXPECT_TRUE(kset.decide(ks).satisfied);
    if (s->sample(ks, ks_const, 1))
      succ++;
  }
  ROS_INFO_NAMED("pr2_arm_kinematics_plugin",
                 "Success rate for IK Constraint Sampler with position & orientation constraints for both arms: %lf",
                 (double)succ / (double)NT);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
