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

/** \author Martin Pecka*/

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_resources/config.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <gtest/gtest.h>

class TestAABB : public testing::Test
{
protected:
  std::string readFileToString(boost::filesystem::path path) const
  {
    std::string file_string;
    std::fstream file(path.string().c_str(), std::fstream::in);
    if (file.is_open())
    {
      std::string line;
      while (file.good())
      {
        std::getline(file, line);
        file_string += (line + "\n");
      }
      file.close();
    }
    return file_string;
  }

  virtual void SetUp(){};

  robot_state::RobotState loadModel(const std::string urdf, const std::string srdf)
  {
    urdf::ModelInterfaceSharedPtr parsed_urdf(urdf::parseURDF(urdf));
    if (!parsed_urdf)
      throw std::runtime_error("Cannot parse URDF.");

    srdf::ModelSharedPtr parsed_srdf(new srdf::Model());
    bool srdf_ok = parsed_srdf->initString(*parsed_urdf, srdf);
    if (!srdf_ok)
      throw std::runtime_error("Cannot parse URDF.");

    robot_model::RobotModelPtr model(new robot_model::RobotModel(parsed_urdf, parsed_srdf));
    robot_state::RobotState robot_state = robot_state::RobotState(model);
    robot_state.setToDefaultValues();
    robot_state.update(true);

    return robot_state;
  }

  virtual void TearDown()
  {
  }
};

TEST_F(TestAABB, TestPR2)
{
  // Contains a link with mesh geometry that is not centered

  boost::filesystem::path res_path(MOVEIT_TEST_RESOURCES_DIR);

  const std::string PR2_URDF = this->readFileToString(res_path / "pr2_description/urdf/robot.xml");
  const std::string PR2_SRDF = this->readFileToString(res_path / "pr2_description/srdf/robot.xml");

  robot_state::RobotState pr2_state = this->loadModel(PR2_URDF, PR2_SRDF);

  const Eigen::Vector3d& extentsBaseFootprint = pr2_state.getLinkModel("base_footprint")->getShapeExtentsAtOrigin();
  // values taken from moveit_resources/pr2_description/urdf/robot.xml
  EXPECT_NEAR(extentsBaseFootprint[0], 0.001, 1e-4);
  EXPECT_NEAR(extentsBaseFootprint[1], 0.001, 1e-4);
  EXPECT_NEAR(extentsBaseFootprint[2], 0.001, 1e-4);

  const Eigen::Vector3d& offsetBaseFootprint = pr2_state.getLinkModel("base_footprint")->getCenteredBoundingBoxOffset();
  EXPECT_NEAR(offsetBaseFootprint[0], 0.0, 1e-4);
  EXPECT_NEAR(offsetBaseFootprint[1], 0.0, 1e-4);
  EXPECT_NEAR(offsetBaseFootprint[2], 0.071, 1e-4);

  const Eigen::Vector3d& extentsBaseLink = pr2_state.getLinkModel("base_link")->getShapeExtentsAtOrigin();
  // values computed from moveit_resources/pr2_description/urdf/meshes/base_v0/base_L.stl in e.g. Meshlab
  EXPECT_NEAR(extentsBaseLink[0], 0.668242, 1e-4);
  EXPECT_NEAR(extentsBaseLink[1], 0.668242, 1e-4);
  EXPECT_NEAR(extentsBaseLink[2], 0.656175, 1e-4);

  const Eigen::Vector3d& offsetBaseLink = pr2_state.getLinkModel("base_link")->getCenteredBoundingBoxOffset();
  EXPECT_NEAR(offsetBaseLink[0], 0.0, 1e-4);
  EXPECT_NEAR(offsetBaseLink[1], 0.0, 1e-4);
  EXPECT_NEAR(offsetBaseLink[2], 0.656175 / 2, 1e-4);  // The 3D mesh isn't centered, but is whole above z axis

  std::vector<double> pr2_aabb;
  pr2_state.computeAABB(pr2_aabb);
  ASSERT_EQ(pr2_aabb.size(), 6);

  EXPECT_NEAR(pr2_aabb[0], -0.3376, 1e-4);
  EXPECT_NEAR(pr2_aabb[1], 0.6397, 1e-4);
  EXPECT_NEAR(pr2_aabb[2], -0.6682 / 2, 1e-4);
  EXPECT_NEAR(pr2_aabb[3], 0.6682 / 2, 1e-4);
  EXPECT_NEAR(pr2_aabb[4], 0.0044, 1e-4);
  EXPECT_NEAR(pr2_aabb[5], 1.6328, 1e-4);

  // To visualize bbox of the PR2, you can uncomment this code.
  /*
  for (std::size_t i = 0; i < 3; ++i) {
    double dim = pr2_aabb[2*i+1] - pr2_aabb[2*i];
    double center = dim/2;
    std::cout << dim << ", " << (pr2_aabb[2*i+1] - center) << std::endl;
  }
  // */
}

TEST_F(TestAABB, TestSimple)
{
  // Contains a link with simple geometry and an offset in the collision link

  const std::string SIMPLE_URDF =
      "<?xml version='1.0' ?>"
      "<robot name='simple'>"
      "  <link name='base_link'>"
      "    <collision>"
      "      <origin rpy='0 0 0' xyz='0 0 0'/>"
      "      <geometry>"
      "        <mesh filename='package://moveit_resources/pr2_description/urdf/meshes/base_v0/base_L.stl'/>"
      "      </geometry>"
      "    </collision>"
      "  </link>"
      "  <link name='base_footprint'>"
      "    <collision>"
      "      <origin rpy='0 0 0' xyz='0 0 0.071'/>"
      "      <geometry>"
      "        <box size='0.001 0.001 0.001'/>"
      "      </geometry>"
      "    </collision>"
      "  </link>"
      "  <joint name='base_footprint_joint' type='fixed'>"
      "    <origin rpy='0 0 0' xyz='0 0 0.051'/>"
      "    <child link='base_link'/>"
      "    <parent link='base_footprint'/>"
      "  </joint>"
      "</robot>";

  const std::string SIMPLE_SRDF =
      "<?xml version='1.0'?>"
      "<robot name='simple'>  "
      "  <virtual_joint name='world_joint' type='planar' parent_frame='odom_combined' child_link='base_footprint'/>   "
      "  <group name='base'>"
      "    <joint name='world_joint'/>"
      "  </group>    "
      "</robot>";

  robot_state::RobotState simple_state = this->loadModel(SIMPLE_URDF, SIMPLE_SRDF);

  std::vector<double> simple_aabb;
  simple_state.computeAABB(simple_aabb);

  ASSERT_EQ(simple_aabb.size(), 6);
  EXPECT_NEAR(simple_aabb[0], -0.6682 / 2, 1e-4);
  EXPECT_NEAR(simple_aabb[1], 0.6682 / 2, 1e-4);
  EXPECT_NEAR(simple_aabb[2], -0.6682 / 2, 1e-4);
  EXPECT_NEAR(simple_aabb[3], 0.6682 / 2, 1e-4);
  EXPECT_NEAR(simple_aabb[4], 0.0510, 1e-4);
  EXPECT_NEAR(simple_aabb[5], 0.7071, 1e-4);
}

TEST_F(TestAABB, TestComplex)
{
  // Contains a link with simple geometry and an offset and rotation in the collision link

  const std::string COMPLEX_URDF = "<?xml version='1.0' ?>"
                                   "<robot name='complex'>"
                                   "  <link name='base_link'>"
                                   "    <collision>"
                                   "      <origin rpy='0 0 1.5708' xyz='5.0 0 1.0'/>"
                                   "      <geometry>"
                                   "        <box size='1.0 0.1 0.1' />"
                                   "      </geometry>"
                                   "    </collision>"
                                   "    <collision>"
                                   "      <origin rpy='0 0 1.5708' xyz='4.0 0 1.0'/>"
                                   "      <geometry>"
                                   "        <box size='1.0 0.1 0.1' />"
                                   "      </geometry>"
                                   "    </collision>"
                                   "  </link>"
                                   "  <link name='base_footprint'>"
                                   "    <collision>"
                                   "      <origin rpy='0 1.5708 0' xyz='-5.0 0 -1.0'/>"
                                   "      <geometry>"
                                   "        <box size='0.1 1.0 0.1' />"
                                   "      </geometry>"
                                   "    </collision>"
                                   "  </link>"
                                   "  <joint name='base_footprint_joint' type='fixed'>"
                                   "    <origin rpy='0 0 1.5708' xyz='0 0 1'/>"
                                   "    <child link='base_link'/>"
                                   "    <parent link='base_footprint'/>"
                                   "  </joint>"
                                   "</robot>";

  const std::string COMPLEX_SRDF =
      "<?xml version='1.0'?>"
      "<robot name='complex'>  "
      "  <virtual_joint name='world_joint' type='planar' parent_frame='odom_combined' child_link='base_footprint'/>   "
      "  <group name='base'>"
      "    <joint name='world_joint'/>"
      "  </group>    "
      "</robot>";

  robot_state::RobotState complex_state = this->loadModel(COMPLEX_URDF, COMPLEX_SRDF);

  EXPECT_NEAR(complex_state.getLinkModel("base_footprint")->getShapeExtentsAtOrigin()[0], 0.1, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_footprint")->getShapeExtentsAtOrigin()[1], 1.0, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_footprint")->getShapeExtentsAtOrigin()[2], 0.1, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_footprint")->getCenteredBoundingBoxOffset()[0], -5.0, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_footprint")->getCenteredBoundingBoxOffset()[1], 0.0, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_footprint")->getCenteredBoundingBoxOffset()[2], -1.0, 1e-4);

  EXPECT_NEAR(complex_state.getLinkModel("base_link")->getShapeExtentsAtOrigin()[0], 1.1, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_link")->getShapeExtentsAtOrigin()[1], 1.0, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_link")->getShapeExtentsAtOrigin()[2], 0.1, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_link")->getCenteredBoundingBoxOffset()[0], 4.5, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_link")->getCenteredBoundingBoxOffset()[1], 0.0, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_link")->getCenteredBoundingBoxOffset()[2], 1.0, 1e-4);

  std::vector<double> complex_aabb;
  complex_state.computeAABB(complex_aabb);

  ASSERT_EQ(complex_aabb.size(), 6);
  EXPECT_NEAR(complex_aabb[0], -5.05, 1e-4);
  EXPECT_NEAR(complex_aabb[1], 0.5, 1e-4);
  EXPECT_NEAR(complex_aabb[2], -0.5, 1e-4);
  EXPECT_NEAR(complex_aabb[3], 5.05, 1e-4);
  EXPECT_NEAR(complex_aabb[4], -1.05, 1e-4);
  EXPECT_NEAR(complex_aabb[5], 2.05, 1e-4);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
