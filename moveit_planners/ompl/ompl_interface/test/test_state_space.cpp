/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space.h>
#include <moveit_resources/config.h>

#include <urdf_parser/urdf_parser.h>

#include <ompl/util/Exception.h>
#include <moveit/robot_state/conversions.h>
#include <gtest/gtest.h>
#include <fstream>
#include <boost/filesystem/path.hpp>

class LoadPlanningModelsPr2 : public testing::Test
{
protected:
  virtual void SetUp()
  {
    boost::filesystem::path res_path(MOVEIT_TEST_RESOURCES_DIR);

    srdf_model_.reset(new srdf::Model());
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
      urdf_model_ = urdf::parseURDF(xml_string);
    }
    srdf_model_->initFile(*urdf_model_, (res_path / "pr2_description/srdf/robot.xml").string());
    robot_model_.reset(new moveit::core::RobotModel(urdf_model_, srdf_model_));
  };

  virtual void TearDown()
  {
  }

protected:
  robot_model::RobotModelPtr robot_model_;
  urdf::ModelInterfaceSharedPtr urdf_model_;
  srdf::ModelSharedPtr srdf_model_;
  bool urdf_ok_;
  bool srdf_ok_;
};

TEST_F(LoadPlanningModelsPr2, StateSpace)
{
  ompl_interface::ModelBasedStateSpaceSpecification spec(robot_model_, "whole_body");
  ompl_interface::JointModelStateSpace ss(spec);
  ss.setPlanningVolume(-1, 1, -1, 1, -1, 1);
  ss.setup();
  std::ofstream fout("ompl_interface_test_state_space_diagram1.dot");
  ss.diagram(fout);
  bool passed = false;
  try
  {
    ss.sanityChecks();
    passed = true;
  }
  catch (ompl::Exception& ex)
  {
    ROS_ERROR("Sanity checks did not pass: %s", ex.what());
  }
  EXPECT_TRUE(passed);
}

TEST_F(LoadPlanningModelsPr2, StateSpaces)
{
  ompl_interface::ModelBasedStateSpaceSpecification spec1(robot_model_, "right_arm");
  ompl_interface::ModelBasedStateSpace ss1(spec1);
  ss1.setup();

  ompl_interface::ModelBasedStateSpaceSpecification spec2(robot_model_, "left_arm");
  ompl_interface::ModelBasedStateSpace ss2(spec2);
  ss2.setup();

  ompl_interface::ModelBasedStateSpaceSpecification spec3(robot_model_, "whole_body");
  ompl_interface::ModelBasedStateSpace ss3(spec3);
  ss3.setup();

  ompl_interface::ModelBasedStateSpaceSpecification spec4(robot_model_, "arms");
  ompl_interface::ModelBasedStateSpace ss4(spec4);
  ss4.setup();

  std::ofstream fout("ompl_interface_test_state_space_diagram2.dot");
  ompl::base::StateSpace::Diagram(fout);
}

TEST_F(LoadPlanningModelsPr2, StateSpaceCopy)
{
  ompl_interface::ModelBasedStateSpaceSpecification spec(robot_model_, "right_arm");
  ompl_interface::JointModelStateSpace ss(spec);
  ss.setPlanningVolume(-1, 1, -1, 1, -1, 1);
  ss.setup();
  std::ofstream fout("ompl_interface_test_state_space_diagram1.dot");
  ss.diagram(fout);
  bool passed = false;
  try
  {
    ss.sanityChecks();
    passed = true;
  }
  catch (ompl::Exception& ex)
  {
    ROS_ERROR("Sanity checks did not pass: %s", ex.what());
  }
  EXPECT_TRUE(passed);

  robot_state::RobotState kstate(robot_model_);
  kstate.setToRandomPositions();
  EXPECT_TRUE(kstate.distance(kstate) < 1e-12);
  ompl::base::State* state = ss.allocState();
  for (int i = 0; i < 10; ++i)
  {
    robot_state::RobotState kstate2(kstate);
    EXPECT_TRUE(kstate.distance(kstate2) < 1e-12);
    ss.copyToOMPLState(state, kstate);
    kstate.setToRandomPositions(kstate.getRobotModel()->getJointModelGroup(ss.getJointModelGroupName()));
    std::cout << (kstate.getGlobalLinkTransform("r_wrist_roll_link").translation() -
                  kstate2.getGlobalLinkTransform("r_wrist_roll_link").translation())
              << std::endl;
    EXPECT_TRUE(kstate.distance(kstate2) > 1e-12);
    ss.copyToRobotState(kstate, state);
    std::cout << (kstate.getGlobalLinkTransform("r_wrist_roll_link").translation() -
                  kstate2.getGlobalLinkTransform("r_wrist_roll_link").translation())
              << std::endl;
    EXPECT_TRUE(kstate.distance(kstate2) < 1e-12);
  }

  ss.freeState(state);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
