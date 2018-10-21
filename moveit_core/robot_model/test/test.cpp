/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
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

#include <moveit/robot_model/robot_model.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>
#include <moveit/profiler/profiler.h>
#include <moveit_resources/config.h>

class LoadPlanningModelsPr2 : public testing::Test
{
protected:
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
    robot_model.reset(new moveit::core::RobotModel(urdf_model, srdf_model));
  };

  void TearDown() override
  {
  }

protected:
  urdf::ModelInterfaceSharedPtr urdf_model;
  srdf::ModelSharedPtr srdf_model;
  moveit::core::RobotModelConstPtr robot_model;
};

TEST_F(LoadPlanningModelsPr2, InitOK)
{
  ASSERT_EQ(urdf_model->getName(), "pr2");
  ASSERT_EQ(srdf_model->getName(), "pr2");
}

TEST_F(LoadPlanningModelsPr2, Model)
{
  // robot_model->printModelInfo(std::cout);

  const std::vector<const moveit::core::JointModel*>& joints = robot_model->getJointModels();
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    ASSERT_EQ(joints[i]->getJointIndex(), i);
    ASSERT_EQ(robot_model->getJointModel(joints[i]->getName()), joints[i]);
  }
  const std::vector<const moveit::core::LinkModel*>& links = robot_model->getLinkModels();
  for (std::size_t i = 0; i < links.size(); ++i)
  {
    ASSERT_EQ(links[i]->getLinkIndex(), i);
    //    std::cout << joints[i]->getName() << std::endl;
  }
  moveit::tools::Profiler::Status();
}

TEST(SiblingAssociateLinks, SimpleYRobot)
{
  /* base_link - a - b - c
                  \
                   - d ~ e          */
  const std::string MODEL = "<?xml version=\"1.0\" ?>"
                            "<robot name=\"one_robot\">"
                            "<link name=\"base_link\"/>"
                            "<joint name=\"joint_a\" type=\"continuous\">"
                            "  <parent link=\"base_link\"/>"
                            "  <child link=\"link_a\"/>"
                            "  <axis xyz=\"0 0 1\"/>"
                            "  <origin rpy=\" 0.0 0 0 \" xyz=\"0.0 0 0 \"/>"
                            "</joint>"
                            "<link name=\"link_a\"/>"
                            "<joint name=\"joint_b\" type=\"fixed\">"
                            "  <parent link=\"link_a\"/>"
                            "  <child link=\"link_b\"/>"
                            "  <origin rpy=\" 0.0 -0.42 0 \" xyz=\"0.0 0.5 0 \"/>"
                            "</joint>"
                            "<link name=\"link_b\"/>"
                            "<joint name=\"joint_c\" type=\"fixed\">"
                            "  <parent link=\"link_b\"/>"
                            "  <child link=\"link_c\"/>"
                            "  <origin rpy=\" 0.0 0.42 0.0 \" xyz=\"0.0 0.5 0 \"/>"
                            "</joint>"
                            "<link name=\"link_c\"/>"
                            "<joint name=\"joint_d\" type=\"fixed\">"
                            "  <parent link=\"link_a\"/>"
                            "  <child link=\"link_d\"/>"
                            "  <origin rpy=\" 0.0 -0.42 0 \" xyz=\"0.0 0.5 0 \"/>"
                            "</joint>"
                            "<link name=\"link_d\"/>"
                            "<joint name=\"joint_e\" type=\"continuous\">"
                            "  <parent link=\"link_d\"/>"
                            "  <child link=\"link_e\"/>"
                            "  <axis xyz=\"0 0 1\"/>"
                            "  <origin rpy=\" 0.0 0 0 \" xyz=\"0.0 0 0 \"/>"
                            "</joint>"
                            "<link name=\"link_e\"/>"
                            "</robot>";

  const std::string SMODEL =
      "<?xml version=\"1.0\" ?>"
      "<robot name=\"one_robot\">"
      "<virtual_joint name=\"base_joint\" child_link=\"base_link\" parent_frame=\"odom\" type=\"planar\"/>"
      "<group name=\"base_joint\"><joint name=\"base_joint\"/></group>"
      "</robot>";
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(MODEL);
  srdf::ModelSharedPtr srdf_model(new srdf::Model());
  srdf_model->initString(*urdf_model, SMODEL);
  moveit::core::RobotModelConstPtr robot_model;
  robot_model.reset(new moveit::core::RobotModel(urdf_model, srdf_model));

  const std::string a = "link_a", b = "link_b", c = "link_c", d = "link_d";
  auto connected = { a, b, c, d };  // these are rigidly connected with each other
  moveit::core::LinkTransformMap map;

  for (const std::string& root : connected)
  {
    SCOPED_TRACE("root: " + root);
    std::set<std::string> expected_set(connected);
    expected_set.erase(root);
    std::set<std::string> actual_set;
    for (const auto& item : robot_model->getLinkModel(root)->getAssociatedFixedTransforms())
      actual_set.insert(item.first->getName());

    std::ostringstream expected, actual;
    std::copy(expected_set.begin(), expected_set.end(), std::ostream_iterator<std::string>(expected, " "));
    std::copy(actual_set.begin(), actual_set.end(), std::ostream_iterator<std::string>(actual, " "));

    EXPECT_EQ(expected.str(), actual.str());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
