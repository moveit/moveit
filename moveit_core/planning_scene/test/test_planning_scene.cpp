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

#include <gtest/gtest.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/utils/message_checks.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <sstream>
#include <string>
#include <boost/filesystem/path.hpp>
#include <ros/package.h>

// This function needs to return void so the gtest FAIL() macro inside
// it works right.
void loadModelFile(std::string filename, std::string& file_content)
{
  boost::filesystem::path res_path(ros::package::getPath("moveit_resources"));
  std::string xml_string;
  std::fstream xml_file((res_path / filename).string().c_str(), std::fstream::in);
  EXPECT_TRUE(xml_file.is_open());
  while (xml_file.good())
  {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  file_content = xml_string;
}

void loadRobotModel(urdf::ModelInterfaceSharedPtr& robot_model_out)
{
  std::string xml_string;
  loadModelFile("pr2_description/urdf/robot.xml", xml_string);
  robot_model_out = urdf::parseURDF(xml_string);
}
void loadRobotModels(urdf::ModelInterfaceSharedPtr& robot_model_out, srdf::ModelSharedPtr& srdf_model_out)
{
  loadRobotModel(robot_model_out);
  std::string xml_string;
  loadModelFile("pr2_description/srdf/robot.xml", xml_string);
  srdf_model_out->initString(*robot_model_out, xml_string);
}

TEST(PlanningScene, LoadRestore)
{
  urdf::ModelInterfaceSharedPtr urdf_model;
  loadRobotModel(urdf_model);
  srdf::ModelSharedPtr srdf_model(new srdf::Model());
  planning_scene::PlanningScene ps(urdf_model, srdf_model);
  moveit_msgs::PlanningScene ps_msg;
  ps.getPlanningSceneMsg(ps_msg);
  EXPECT_EQ(ps.getName(), ps_msg.name);
  EXPECT_EQ(ps.getRobotModel()->getName(), ps_msg.robot_model_name);
  ps.setPlanningSceneMsg(ps_msg);
  EXPECT_EQ(ps.getName(), ps_msg.name);
  EXPECT_EQ(ps.getRobotModel()->getName(), ps_msg.robot_model_name);
}

TEST(PlanningScene, LoadRestoreDiff)
{
  urdf::ModelInterfaceSharedPtr urdf_model;
  loadRobotModel(urdf_model);
  srdf::ModelSharedPtr srdf_model(new srdf::Model());

  planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene(urdf_model, srdf_model));

  collision_detection::World& world = *ps->getWorldNonConst();
  Eigen::Isometry3d id = Eigen::Isometry3d::Identity();
  world.addToObject("sphere", shapes::ShapeConstPtr(new shapes::Sphere(0.4)), id);

  moveit_msgs::PlanningScene ps_msg;
  ps_msg.robot_state.is_diff = true;
  EXPECT_TRUE(moveit::core::isEmpty(ps_msg));
  ps->getPlanningSceneMsg(ps_msg);
  ps->setPlanningSceneMsg(ps_msg);
  EXPECT_FALSE(moveit::core::isEmpty(ps_msg));
  EXPECT_TRUE(world.hasObject("sphere"));

  planning_scene::PlanningScenePtr next = ps->diff();
  EXPECT_TRUE(next->getWorld()->hasObject("sphere"));
  next->getWorldNonConst()->addToObject("sphere2", shapes::ShapeConstPtr(new shapes::Sphere(0.5)), id);
  EXPECT_EQ(next->getWorld()->size(), 2u);
  EXPECT_EQ(ps->getWorld()->size(), 1u);
  next->getPlanningSceneDiffMsg(ps_msg);
  EXPECT_EQ(ps_msg.world.collision_objects.size(), 1u);
  next->decoupleParent();
  moveit_msgs::PlanningScene ps_msg2;
  next->getPlanningSceneDiffMsg(ps_msg2);
  EXPECT_EQ(ps_msg2.world.collision_objects.size(), 0u);
  next->getPlanningSceneMsg(ps_msg);
  EXPECT_EQ(ps_msg.world.collision_objects.size(), 2u);
  ps->setPlanningSceneMsg(ps_msg);
  EXPECT_EQ(ps->getWorld()->size(), 2u);
}

TEST(PlanningScene, MakeAttachedDiff)
{
  srdf::ModelSharedPtr srdf_model(new srdf::Model());
  urdf::ModelInterfaceSharedPtr urdf_model;
  loadRobotModel(urdf_model);

  planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene(urdf_model, srdf_model));

  collision_detection::World& world = *ps->getWorldNonConst();
  Eigen::Isometry3d id = Eigen::Isometry3d::Identity();
  world.addToObject("sphere", shapes::ShapeConstPtr(new shapes::Sphere(0.4)), id);

  planning_scene::PlanningScenePtr attached_object_diff_scene = ps->diff();

  moveit_msgs::AttachedCollisionObject att_obj;
  att_obj.link_name = "r_wrist_roll_link";
  att_obj.object.operation = moveit_msgs::CollisionObject::ADD;
  att_obj.object.id = "sphere";

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  attached_object_diff_scene->processAttachedCollisionObjectMsg(att_obj);
  attached_object_diff_scene->checkCollision(req, res);
  ps->checkCollision(req, res);
}

TEST(PlanningScene, isStateValid)
{
  srdf::ModelSharedPtr srdf_model(new srdf::Model());
  urdf::ModelInterfaceSharedPtr urdf_model;
  loadRobotModels(urdf_model, srdf_model);

  planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene(urdf_model, srdf_model));
  robot_state::RobotState current_state = ps->getCurrentState();
  if (ps->isStateColliding(current_state, "left_arm"))
  {
    EXPECT_FALSE(ps->isStateValid(current_state, "left_arm"));
  }
}

TEST(PlanningScene, loadGoodSceneGeometry)
{
  srdf::ModelSharedPtr srdf_model(new srdf::Model());
  urdf::ModelInterfaceSharedPtr urdf_model;
  loadRobotModels(urdf_model, srdf_model);
  planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene(urdf_model, srdf_model));

  std::istringstream good_scene_geometry;
  good_scene_geometry.str("foobar_scene\n"
                          "* foo\n"
                          "1\n"
                          "box\n"
                          "2.58 1.36 0.31\n"
                          "1.49257 1.00222 0.170051\n"
                          "0 0 4.16377e-05 1\n"
                          "0 0 1 0.3\n"
                          "* bar\n"
                          "1\n"
                          "cylinder\n"
                          "0.02 0.0001\n"
                          "0.453709 0.499136 0.355051\n"
                          "0 0 4.16377e-05 1\n"
                          "1 0 0 1\n"
                          ".\n");
  EXPECT_TRUE(ps->loadGeometryFromStream(good_scene_geometry));
  EXPECT_EQ(ps->getName(), "foobar_scene");
  EXPECT_TRUE(ps->getWorld()->hasObject("foo"));
  EXPECT_TRUE(ps->getWorld()->hasObject("bar"));
  EXPECT_FALSE(ps->getWorld()->hasObject("baz"));  // Sanity check.
}

TEST(PlanningScene, loadBadSceneGeometry)
{
  srdf::ModelSharedPtr srdf_model(new srdf::Model());
  urdf::ModelInterfaceSharedPtr urdf_model;
  loadRobotModels(urdf_model, srdf_model);
  planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene(urdf_model, srdf_model));
  std::istringstream empty_scene_geometry;

  // This should fail since there is no planning scene name and no end of geometry marker.
  EXPECT_FALSE(ps->loadGeometryFromStream(empty_scene_geometry));

  std::istringstream malformed_scene_geometry;
  malformed_scene_geometry.str("malformed_scene_geometry\n"
                               "* foo\n"
                               "1\n"
                               "box\n"
                               "2.58 1.36\n" /* Only two tokens; should be 3 */
                               "1.49257 1.00222 0.170051\n"
                               "0 0 4.16377e-05 1\n"
                               "0 0 1 0.3\n"
                               ".\n");
  EXPECT_FALSE(ps->loadGeometryFromStream(malformed_scene_geometry));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
