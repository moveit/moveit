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

#include <kinematic_constraints/kinematic_constraint.h>
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>

class LoadPlanningModelsPr2 : public testing::Test
{
protected:
  
  virtual void SetUp()
  {
    srdf_model.reset(new srdf::Model());
    std::string xml_string;
    std::fstream xml_file("../planning_models/test/urdf/robot.xml", std::fstream::in);
    if (xml_file.is_open())
    {
      while ( xml_file.good() )
      {
        std::string line;
        std::getline( xml_file, line);
        xml_string += (line + "\n");
      }
      xml_file.close();
      urdf_model = urdf::parseURDF(xml_string);
    }
    kmodel.reset(new planning_models::KinematicModel(urdf_model, srdf_model));
  };
  
  virtual void TearDown()
  {
  }
  
protected:
  
  boost::shared_ptr<urdf::ModelInterface>     urdf_model;
  boost::shared_ptr<srdf::Model>     srdf_model;
  planning_models::KinematicModelPtr kmodel;
};

TEST_F(LoadPlanningModelsPr2, JointConstraintsSimple)
{
    planning_models::KinematicState ks(kmodel);
    ks.setToDefaultValues();
    planning_models::TransformsPtr tf(new planning_models::Transforms(kmodel->getModelFrame()));

    kinematic_constraints::JointConstraint jc(kmodel, tf);
    moveit_msgs::JointConstraint jcm;
    jcm.joint_name = "head_pan_joint";
    jcm.position = 0.4;
    jcm.tolerance_above = 0.1;
    jcm.tolerance_below = 0.05;
    jcm.weight = 1.0;

    EXPECT_TRUE(jc.configure(jcm));
    kinematic_constraints::ConstraintEvaluationResult p1 = jc.decide(ks);
    EXPECT_FALSE(p1.satisfied);
    EXPECT_NEAR(p1.distance, jcm.position, 1e-6);

    std::map<std::string, double> jvals;
    jvals[jcm.joint_name] = 0.41;
    ks.setStateValues(jvals);

    kinematic_constraints::ConstraintEvaluationResult p2 = jc.decide(ks);
    EXPECT_TRUE(p2.satisfied);
    EXPECT_NEAR(p2.distance, 0.01, 1e-6);

    jvals[jcm.joint_name] = 0.46;
    ks.setStateValues(jvals);
    EXPECT_TRUE(jc.decide(ks).satisfied);

    jvals[jcm.joint_name] = 0.501;
    ks.setStateValues(jvals);
    EXPECT_FALSE(jc.decide(ks).satisfied);

    jvals[jcm.joint_name] = 0.39;
    ks.setStateValues(jvals);
    EXPECT_TRUE(jc.decide(ks).satisfied);

    jvals[jcm.joint_name] = 0.34;
    ks.setStateValues(jvals);
    EXPECT_FALSE(jc.decide(ks).satisfied);
    EXPECT_TRUE(jc.equal(jc, 1e-12));
}

TEST_F(LoadPlanningModelsPr2, JointConstraintsCont)
{
    planning_models::KinematicState ks(kmodel);
    ks.setToDefaultValues();
    planning_models::TransformsPtr tf(new planning_models::Transforms(kmodel->getModelFrame()));

    kinematic_constraints::JointConstraint jc(kmodel, tf);
    moveit_msgs::JointConstraint jcm;

    jcm.joint_name = "l_wrist_roll_joint";
    jcm.position = 3.14;
    jcm.tolerance_above = 0.04;
    jcm.tolerance_below = 0.02;
    jcm.weight = 1.0;

    EXPECT_TRUE(jc.configure(jcm));

    std::map<std::string, double> jvals;
    jvals[jcm.joint_name] = 3.17;
    ks.setStateValues(jvals);

    kinematic_constraints::ConstraintEvaluationResult p1 = jc.decide(ks);
    EXPECT_TRUE(p1.satisfied);
    EXPECT_NEAR(p1.distance, 0.03, 1e-6);


    jvals[jcm.joint_name] = -3.14;
    ks.setStateValues(jvals);

    kinematic_constraints::ConstraintEvaluationResult p2 = jc.decide(ks);
    EXPECT_TRUE(p2.satisfied);
    EXPECT_NEAR(p2.distance, 0.003185, 1e-4);
}

TEST_F(LoadPlanningModelsPr2, PositionConstraintsFixed)
{
    planning_models::KinematicState ks(kmodel);
    ks.setToDefaultValues();
    planning_models::TransformsPtr tf(new planning_models::Transforms(kmodel->getModelFrame()));

    kinematic_constraints::PositionConstraint pc(kmodel, tf);
    moveit_msgs::PositionConstraint pcm;

    pcm.link_name = "l_wrist_roll_link";
    pcm.target_point_offset.x = 0;
    pcm.target_point_offset.y = 0;
    pcm.target_point_offset.z = 0;
    pcm.constraint_region.primitives.resize(1);
    pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
    pcm.constraint_region.primitives[0].dimensions.resize(1);
    pcm.constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.2;

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

    EXPECT_TRUE(pc.configure(pcm));

    EXPECT_TRUE(pc.decide(ks).satisfied);

    std::map<std::string, double> jvals;
    jvals["torso_lift_joint"] = 0.4;
    ks.setStateValues(jvals);
    EXPECT_FALSE(pc.decide(ks).satisfied); 
    EXPECT_TRUE(pc.equal(pc, 1e-12));
}

TEST_F(LoadPlanningModelsPr2, PositionConstraintsMobile)
{
    planning_models::KinematicState ks(kmodel);
    ks.setToDefaultValues();
    planning_models::TransformsPtr tf(new planning_models::Transforms(kmodel->getModelFrame()));

    kinematic_constraints::PositionConstraint pc(kmodel, tf);
    moveit_msgs::PositionConstraint pcm;

    pcm.link_name = "l_wrist_roll_link";
    pcm.target_point_offset.x = 0;
    pcm.target_point_offset.y = 0;
    pcm.target_point_offset.z = 0;

    pcm.constraint_region.primitives.resize(1);
    pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
    pcm.constraint_region.primitives[0].dimensions.resize(1);
    pcm.constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.38 * 2.0;

    pcm.header.frame_id = "r_wrist_roll_link"; 

    pcm.constraint_region.primitive_poses.resize(1);
    pcm.constraint_region.primitive_poses[0].position.x = 0.0;
    pcm.constraint_region.primitive_poses[0].position.y = 0.0;
    pcm.constraint_region.primitive_poses[0].position.z = 0.0;
    pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
    pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
    pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
    pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
    pcm.weight = 1.0;

    EXPECT_FALSE(tf->isFixedFrame(pcm.link_name));
    EXPECT_TRUE(pc.configure(pcm));

    EXPECT_TRUE(pc.decide(ks).satisfied);

    pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    pcm.constraint_region.primitives[0].dimensions.resize(3);
    pcm.constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.2;
    pcm.constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.25;
    pcm.constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1;
    EXPECT_TRUE(pc.configure(pcm));

    std::map<std::string, double> jvals;
    jvals["l_shoulder_pan_joint"] = 0.4;
    ks.setStateValues(jvals);
    EXPECT_TRUE(pc.decide(ks).satisfied);
    EXPECT_TRUE(pc.equal(pc, 1e-12));
}


TEST_F(LoadPlanningModelsPr2, OrientationConstraintsSimple)
{
    planning_models::KinematicState ks(kmodel);
    ks.setToDefaultValues();
    planning_models::TransformsPtr tf(new planning_models::Transforms(kmodel->getModelFrame()));

    kinematic_constraints::OrientationConstraint oc(kmodel, tf);
    moveit_msgs::OrientationConstraint ocm;

    ocm.link_name = "r_wrist_roll_link";
    ocm.header.frame_id = kmodel->getModelFrame();
    ocm.orientation.x = 0.0;
    ocm.orientation.y = 0.0;
    ocm.orientation.z = 0.0;
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    EXPECT_TRUE(oc.configure(ocm));
    
    EXPECT_FALSE(oc.decide(ks).satisfied);

    ocm.header.frame_id = ocm.link_name;
    EXPECT_TRUE(oc.configure(ocm));
    EXPECT_TRUE(oc.decide(ks).satisfied);  
    EXPECT_TRUE(oc.equal(oc, 1e-12));
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
