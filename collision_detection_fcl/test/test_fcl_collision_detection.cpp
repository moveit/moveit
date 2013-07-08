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

#include <moveit/test_resources/config.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>

#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shape_operations.h>

#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>

#include <boost/filesystem.hpp>

typedef collision_detection::CollisionWorldFCL DefaultCWorldType;
typedef collision_detection::CollisionRobotFCL DefaultCRobotType;

static std::string urdf_file = (boost::filesystem::path(MOVEIT_TEST_RESOURCES_DIR) / "urdf/robot.xml").string();
static std::string srdf_file = (boost::filesystem::path(MOVEIT_TEST_RESOURCES_DIR) / "srdf/robot.xml").string();
static std::string kinect_dae_file = (boost::filesystem::path(MOVEIT_TEST_RESOURCES_DIR) / "urdf/meshes/sensors/kinect_v0/kinect.dae").string();

class FclCollisionDetectionTester : public testing::Test
{

protected:

  virtual void SetUp()
  {
    srdf_model_.reset(new srdf::Model());
    std::string xml_string;
    std::fstream xml_file(urdf_file.c_str(), std::fstream::in);

    if (xml_file.is_open())
    {
      while ( xml_file.good() )
      {
        std::string line;
        std::getline( xml_file, line);
        xml_string += (line + "\n");
      }
      xml_file.close();
      urdf_model_ = urdf::parseURDF(xml_string);
      urdf_ok_ = urdf_model_;
    }
    else
    {
      EXPECT_EQ("FAILED TO OPEN FILE", urdf_file);
      urdf_ok_ = false;
    }
    srdf_ok_ = srdf_model_->initFile(*urdf_model_, srdf_file);

    kmodel_.reset(new robot_model::RobotModel(urdf_model_, srdf_model_));

    acm_.reset(new collision_detection::AllowedCollisionMatrix(kmodel_->getLinkModelNames(), true));

    crobot_.reset(new DefaultCRobotType(kmodel_));
    cworld_.reset(new DefaultCWorldType());
  }

  virtual void TearDown()
  {

  }

protected:

  bool urdf_ok_;
  bool srdf_ok_;

  boost::shared_ptr<urdf::ModelInterface>  urdf_model_;
  boost::shared_ptr<srdf::Model>           srdf_model_;

  robot_model::RobotModelPtr               kmodel_;

  boost::shared_ptr<collision_detection::CollisionRobot>        crobot_;
  boost::shared_ptr<collision_detection::CollisionWorld>        cworld_;

  collision_detection::AllowedCollisionMatrixPtr acm_;

};


TEST_F(FclCollisionDetectionTester, InitOK)
{
  ASSERT_TRUE(urdf_ok_);
  ASSERT_TRUE(srdf_ok_);
}

TEST_F(FclCollisionDetectionTester, DefaultNotInCollision)
{
  robot_state::RobotState kstate(kmodel_);
  kstate.setToDefaultValues();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  crobot_->checkSelfCollision(req, res, kstate, *acm_);
  ASSERT_FALSE(res.collision);
}


TEST_F(FclCollisionDetectionTester, LinksInCollision)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res1;
  collision_detection::CollisionResult res2;
  collision_detection::CollisionResult res3;
  //req.contacts = true;
  //req.max_contacts = 100;

  robot_state::RobotState kstate(kmodel_);
  kstate.setToDefaultValues();

  Eigen::Affine3d offset = Eigen::Affine3d::Identity();
  offset.translation().x() = .01;

  //  kstate.getLinkState("base_link")->updateGivenGlobalLinkTransform(Eigen::Affine3d::Identity());
  //  kstate.getLinkState("base_bellow_link")->updateGivenGlobalLinkTransform(offset);
  kstate.updateStateWithLinkAt("base_link", Eigen::Affine3d::Identity());
  kstate.updateStateWithLinkAt("base_bellow_link", offset);

  acm_->setEntry("base_link", "base_bellow_link", false);
  crobot_->checkSelfCollision(req, res1, kstate, *acm_);
  ASSERT_TRUE(res1.collision);

  acm_->setEntry("base_link", "base_bellow_link", true);
  crobot_->checkSelfCollision(req, res2, kstate, *acm_);
  ASSERT_FALSE(res2.collision);

  //  req.verbose = true;
  //  kstate.getLinkState("r_gripper_palm_link")->updateGivenGlobalLinkTransform(Eigen::Affine3d::Identity());
  //  kstate.getLinkState("l_gripper_palm_link")->updateGivenGlobalLinkTransform(offset);
  kstate.updateStateWithLinkAt("r_gripper_palm_link", Eigen::Affine3d::Identity());
  kstate.updateStateWithLinkAt("l_gripper_palm_link", offset);

  acm_->setEntry("r_gripper_palm_link", "l_gripper_palm_link", false);
  crobot_->checkSelfCollision(req, res3, kstate, *acm_);
  ASSERT_TRUE(res3.collision);
}


TEST_F(FclCollisionDetectionTester, ContactReporting)
{
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 1;

  robot_state::RobotState kstate(kmodel_);
  kstate.setToDefaultValues();

  Eigen::Affine3d offset = Eigen::Affine3d::Identity();
  offset.translation().x() = .01;

  //  kstate.getLinkState("base_link")->updateGivenGlobalLinkTransform(Eigen::Affine3d::Identity());
  //  kstate.getLinkState("base_bellow_link")->updateGivenGlobalLinkTransform(offset);
  //  kstate.getLinkState("r_gripper_palm_link")->updateGivenGlobalLinkTransform(Eigen::Affine3d::Identity());
  //  kstate.getLinkState("l_gripper_palm_link")->updateGivenGlobalLinkTransform(offset);

  kstate.updateStateWithLinkAt("base_link", Eigen::Affine3d::Identity());
  kstate.updateStateWithLinkAt("base_bellow_link", offset);
  kstate.updateStateWithLinkAt("r_gripper_palm_link", Eigen::Affine3d::Identity());
  kstate.updateStateWithLinkAt("l_gripper_palm_link", offset);

  acm_->setEntry("base_link", "base_bellow_link", false);
  acm_->setEntry("r_gripper_palm_link", "l_gripper_palm_link", false);

  collision_detection::CollisionResult res;
  crobot_->checkSelfCollision(req, res, kstate, *acm_);
  ASSERT_TRUE(res.collision);
  EXPECT_EQ(res.contacts.size(),1);
  EXPECT_EQ(res.contacts.begin()->second.size(),1);

  res.clear();
  req.max_contacts = 2;
  req.max_contacts_per_pair = 1;
  //  req.verbose = true;
  crobot_->checkSelfCollision(req, res, kstate, *acm_);
  ASSERT_TRUE(res.collision);
  EXPECT_EQ(res.contacts.size(), 2);
  EXPECT_EQ(res.contacts.begin()->second.size(),1);

  res.contacts.clear();
  res.contact_count = 0;

  req.max_contacts = 10;
  req.max_contacts_per_pair = 2;
  acm_.reset(new collision_detection::AllowedCollisionMatrix(kmodel_->getLinkModelNames(), false));
  crobot_->checkSelfCollision(req, res, kstate, *acm_);
  ASSERT_TRUE(res.collision);
  EXPECT_LE(res.contacts.size(), 10);
  EXPECT_LE(res.contact_count, 10);

}

TEST_F(FclCollisionDetectionTester, ContactPositions)
{
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 1;

  robot_state::RobotState kstate(kmodel_);
  kstate.setToDefaultValues();

  Eigen::Affine3d pos1 = Eigen::Affine3d::Identity();
  Eigen::Affine3d pos2 = Eigen::Affine3d::Identity();

  pos1.translation().x() = 5.0;
  pos2.translation().x() = 5.01;

  //  kstate.getLinkState("r_gripper_palm_link")->updateGivenGlobalLinkTransform(pos1);
  //  kstate.getLinkState("l_gripper_palm_link")->updateGivenGlobalLinkTransform(pos2);
  kstate.updateStateWithLinkAt("r_gripper_palm_link", pos1);
  kstate.updateStateWithLinkAt("l_gripper_palm_link", pos2);

  acm_->setEntry("r_gripper_palm_link", "l_gripper_palm_link", false);

  collision_detection::CollisionResult res;
  crobot_->checkSelfCollision(req, res, kstate, *acm_);
  ASSERT_TRUE(res.collision);
  ASSERT_EQ(res.contacts.size(),1);
  ASSERT_EQ(res.contacts.begin()->second.size(),1);

  for(collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin();
      it != res.contacts.end();
      it++) {
    EXPECT_NEAR(it->second[0].pos.x(), 5.0, .33);
  }

  pos1 = Eigen::Affine3d(Eigen::Translation3d(3.0,0.0,0.0)*Eigen::Quaterniond::Identity());
  pos2 = Eigen::Affine3d(Eigen::Translation3d(3.0,0.0,0.0)*Eigen::Quaterniond(0.965, 0.0, 0.258, 0.0));
  //  kstate.getLinkState("r_gripper_palm_link")->updateGivenGlobalLinkTransform(pos1);
  //  kstate.getLinkState("l_gripper_palm_link")->updateGivenGlobalLinkTransform(pos2);
  kstate.updateStateWithLinkAt("r_gripper_palm_link", pos1);
  kstate.updateStateWithLinkAt("l_gripper_palm_link", pos2);

  collision_detection::CollisionResult res2;
  crobot_->checkSelfCollision(req, res2, kstate, *acm_);
  ASSERT_TRUE(res2.collision);
  ASSERT_EQ(res2.contacts.size(),1);
  ASSERT_EQ(res2.contacts.begin()->second.size(),1);

  for(collision_detection::CollisionResult::ContactMap::const_iterator it = res2.contacts.begin();
      it != res2.contacts.end();
      it++) {
    EXPECT_NEAR(it->second[0].pos.x(), 3.0, 0.33);
  }

  pos1 = Eigen::Affine3d(Eigen::Translation3d(3.0,0.0,0.0)*Eigen::Quaterniond::Identity());
  pos2 = Eigen::Affine3d(Eigen::Translation3d(3.0,0.0,0.0)*Eigen::Quaterniond(M_PI/4.0, 0.0, M_PI/4.0, 0.0));
  //  kstate.getLinkState("r_gripper_palm_link")->updateGivenGlobalLinkTransform(pos1);
  //  kstate.getLinkState("l_gripper_palm_link")->updateGivenGlobalLinkTransform(pos2);
  kstate.updateStateWithLinkAt("r_gripper_palm_link", pos1);
  kstate.updateStateWithLinkAt("l_gripper_palm_link", pos2);

  collision_detection::CollisionResult res3;
  crobot_->checkSelfCollision(req, res2, kstate, *acm_);
  ASSERT_FALSE(res3.collision);
}

TEST_F(FclCollisionDetectionTester, AttachedBodyTester) {
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  acm_.reset(new collision_detection::AllowedCollisionMatrix(kmodel_->getLinkModelNames(), true));

  robot_state::RobotState kstate(kmodel_);
  kstate.setToDefaultValues();

  Eigen::Affine3d pos1 = Eigen::Affine3d::Identity();
  pos1.translation().x() = 5.0;

  //  kstate.getLinkState("r_gripper_palm_link")->updateGivenGlobalLinkTransform(pos1);
  kstate.updateStateWithLinkAt("r_gripper_palm_link", pos1);
  crobot_->checkSelfCollision(req, res, kstate, *acm_);
  ASSERT_FALSE(res.collision);

  shapes::Shape* shape = new shapes::Box(.1,.1,.1);
  cworld_->getWorld()->addToObject("box", shapes::ShapeConstPtr(shape), pos1);

  res = collision_detection::CollisionResult();
  cworld_->checkRobotCollision(req, res, *crobot_, kstate, *acm_);
  ASSERT_TRUE(res.collision);

  //deletes shape
  cworld_->getWorld()->removeObject("box");

  shape = new shapes::Box(.1,.1,.1);
  std::vector<shapes::ShapeConstPtr> shapes;
  EigenSTL::vector_Affine3d poses;
  shapes.push_back(shapes::ShapeConstPtr(shape));
  poses.push_back(Eigen::Affine3d::Identity());
  std::vector<std::string> touch_links;
  kstate.attachBody("box", shapes, poses, touch_links, "r_gripper_palm_link");

  res = collision_detection::CollisionResult();
  crobot_->checkSelfCollision(req, res, kstate, *acm_);
  ASSERT_TRUE(res.collision);

  //deletes shape
  kstate.clearAttachedBody("box");

  touch_links.push_back("r_gripper_palm_link");
  shapes[0].reset(new shapes::Box(.1,.1,.1));
  kstate.attachBody("box", shapes, poses, touch_links, "r_gripper_palm_link");

  res = collision_detection::CollisionResult();
  crobot_->checkSelfCollision(req, res, kstate, *acm_);
  ASSERT_FALSE(res.collision);

  pos1.translation().x() = 5.01;
  shapes::Shape* coll = new shapes::Box(.1, .1, .1);
  cworld_->getWorld()->addToObject("coll", shapes::ShapeConstPtr(coll), pos1);
  res = collision_detection::CollisionResult();
  cworld_->checkRobotCollision(req, res, *crobot_, kstate, *acm_);
  ASSERT_TRUE(res.collision);

  acm_->setEntry("coll", "r_gripper_palm_link", true);
  res = collision_detection::CollisionResult();
  cworld_->checkRobotCollision(req, res, *crobot_, kstate, *acm_);
  ASSERT_TRUE(res.collision);
}

TEST_F(FclCollisionDetectionTester, DiffSceneTester)
{
  robot_state::RobotState kstate(kmodel_);
  kstate.setToDefaultValues();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  collision_detection::CollisionRobotFCL new_crobot(*(dynamic_cast<collision_detection::CollisionRobotFCL*>(crobot_.get())));

  ros::WallTime before = ros::WallTime::now();
  new_crobot.checkSelfCollision(req,res,kstate);
  double first_check = (ros::WallTime::now()-before).toSec();
  before = ros::WallTime::now();
  new_crobot.checkSelfCollision(req,res,kstate);
  double second_check = (ros::WallTime::now()-before).toSec();

  EXPECT_LT(fabs(first_check-second_check), .05);

  std::vector<shapes::ShapeConstPtr> shapes;
  shapes.resize(1);

  boost::filesystem::path path(boost::filesystem::current_path());

  shapes[0].reset(shapes::createMeshFromResource("file://"+path.string()+"/"+kinect_dae_file));

  EigenSTL::vector_Affine3d poses;
  poses.push_back(Eigen::Affine3d::Identity());

  std::vector<std::string> touch_links;
  kstate.attachBody("kinect", shapes, poses, touch_links, "r_gripper_palm_link");

  before = ros::WallTime::now();
  new_crobot.checkSelfCollision(req,res,kstate);
  first_check = (ros::WallTime::now()-before).toSec();
  before = ros::WallTime::now();
  new_crobot.checkSelfCollision(req,res,kstate);
  second_check = (ros::WallTime::now()-before).toSec();

  //the first check is going to take a while, as data must be constructed
  EXPECT_LT(second_check, .1);

  collision_detection::CollisionRobotFCL other_new_crobot(*(dynamic_cast<collision_detection::CollisionRobotFCL*>(crobot_.get())));
  before = ros::WallTime::now();
  new_crobot.checkSelfCollision(req,res,kstate);
  first_check = (ros::WallTime::now()-before).toSec();
  before = ros::WallTime::now();
  new_crobot.checkSelfCollision(req,res,kstate);
  second_check = (ros::WallTime::now()-before).toSec();

  EXPECT_LT(fabs(first_check-second_check), .05);

}

TEST_F(FclCollisionDetectionTester, ConvertObjectToAttached)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  boost::filesystem::path path(boost::filesystem::current_path());
  shapes::ShapeConstPtr shape(shapes::createMeshFromResource("file://"+path.string()+"/"+kinect_dae_file));
  Eigen::Affine3d pos1 = Eigen::Affine3d::Identity();
  Eigen::Affine3d pos2 = Eigen::Affine3d::Identity();
  pos2.translation().x() = 10.0;

  cworld_->getWorld()->addToObject("kinect", shape, pos1);

  robot_state::RobotState kstate(kmodel_);
  kstate.setToDefaultValues();

  ros::WallTime before = ros::WallTime::now();
  cworld_->checkRobotCollision(req, res, *crobot_, kstate);
  double first_check = (ros::WallTime::now()-before).toSec();
  before = ros::WallTime::now();
  cworld_->checkRobotCollision(req, res, *crobot_, kstate);
  double second_check = (ros::WallTime::now()-before).toSec();

  EXPECT_LT(second_check, .05);

  collision_detection::CollisionWorld::ObjectPtr object = cworld_->getWorld()->getObject("kinect");
  cworld_->getWorld()->removeObject("kinect");

  robot_state::RobotState kstate1(kmodel_);
  robot_state::RobotState kstate2(kmodel_);
  kstate1.setToDefaultValues();
  kstate2.setToDefaultValues();

  std::vector<std::string> touch_links;
  kstate1.attachBody("kinect", object->shapes_, object->shape_poses_, touch_links, "r_gripper_palm_link");

  EigenSTL::vector_Affine3d other_poses;
  other_poses.push_back(pos2);

  // This creates a new set of constant properties for the attached body, which happens to be the same as the one above;
  kstate2.attachBody("kinect", object->shapes_, object->shape_poses_, touch_links, "r_gripper_palm_link");

  //going to take a while, but that's fine
  res = collision_detection::CollisionResult();
  crobot_->checkSelfCollision(req,res,kstate1);

  EXPECT_TRUE(res.collision);

  before = ros::WallTime::now();
  crobot_->checkSelfCollision(req,res,kstate1, *acm_);
  first_check = (ros::WallTime::now()-before).toSec();
  before = ros::WallTime::now();
  req.verbose = true;
  res = collision_detection::CollisionResult();
  crobot_->checkSelfCollision(req,res,kstate2, *acm_);
  second_check = (ros::WallTime::now()-before).toSec();

  EXPECT_LT(first_check, .05);
  EXPECT_LT(fabs(first_check-second_check), .1);
}



TEST_F(FclCollisionDetectionTester, TestCollisionMapAdditionSpeed)
{
  EigenSTL::vector_Affine3d poses;
  std::vector<shapes::ShapeConstPtr> shapes;
  for(unsigned int i = 0; i < 10000; i++) {
    poses.push_back(Eigen::Affine3d::Identity());
    shapes.push_back(shapes::ShapeConstPtr(new shapes::Box(.01, .01, .01)));
  }
  ros::WallTime start = ros::WallTime::now();
  cworld_->getWorld()->addToObject("map", shapes, poses);
  double t = (ros::WallTime::now()-start).toSec();
  EXPECT_GE(1.0, t);
  // this is not really a failure; it is just that slow;
  // looking into doing collision checking with a voxel grid.
  logInform("Adding boxes took %g", t);
}


TEST_F(FclCollisionDetectionTester, MoveMesh)
{
  robot_state::RobotState kstate1(kmodel_);
  kstate1.setToDefaultValues();

  Eigen::Affine3d kinect_pose;
  kinect_pose.setIdentity();
  shapes::ShapePtr kinect_shape;
  boost::filesystem::path path(boost::filesystem::current_path());
  kinect_shape.reset(shapes::createMeshFromResource("file://"+path.string()+"/"+kinect_dae_file));

  cworld_->getWorld()->addToObject("kinect", kinect_shape, kinect_pose);

  Eigen::Affine3d np;
  for(unsigned int i = 0; i < 5 ; i++)
  {
    np = Eigen::Translation3d(i*.001, i*.001, i*.001)*Eigen::Quaterniond::Identity();
    cworld_->getWorld()->moveShapeInObject("kinect", kinect_shape, np);
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    cworld_->checkCollision(req, res, *crobot_, kstate1, *acm_);
  }

  // SUCCEED();
}

TEST_F(FclCollisionDetectionTester, TestChangingShapeSize)
{
  robot_state::RobotState kstate1(kmodel_);
  kstate1.setToDefaultValues();

  collision_detection::CollisionRequest req1;
  collision_detection::CollisionResult res1;


  ASSERT_FALSE(res1.collision);

  EigenSTL::vector_Affine3d poses;
  std::vector<shapes::ShapeConstPtr> shapes;
  for(unsigned int i = 0; i < 5; i++)
  {
    cworld_->getWorld()->removeObject("shape");
    shapes.clear();
    poses.clear();
    shapes.push_back(shapes::ShapeConstPtr(new shapes::Box(1+i*.0001, 1+i*.0001, 1+i*.0001)));
    poses.push_back(Eigen::Affine3d::Identity());
    cworld_->getWorld()->addToObject("shape", shapes, poses);
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    cworld_->checkCollision(req, res, *crobot_, kstate1, *acm_);
    ASSERT_TRUE(res.collision);
  }

  Eigen::Affine3d kinect_pose;
  shapes::ShapePtr kinect_shape;
  boost::filesystem::path path(boost::filesystem::current_path());
  kinect_shape.reset(shapes::createMeshFromResource("file://"+path.string()+"/"+kinect_dae_file));
  cworld_->getWorld()->addToObject("kinect", kinect_shape, kinect_pose);
  collision_detection::CollisionRequest req2;
  collision_detection::CollisionResult res2;
  cworld_->checkCollision(req2, res2, *crobot_, kstate1, *acm_);
  ASSERT_TRUE(res2.collision);
  for(unsigned int i = 0; i < 5; i++) {
    cworld_->getWorld()->removeObject("shape");
    shapes.clear();
    poses.clear();
    shapes.push_back(shapes::ShapeConstPtr(new shapes::Box(1+i*.0001, 1+i*.0001, 1+i*.0001)));
    poses.push_back(Eigen::Affine3d::Identity());
    cworld_->getWorld()->addToObject("shape", shapes, poses);
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    cworld_->checkCollision(req, res, *crobot_, kstate1, *acm_);
    ASSERT_TRUE(res.collision);
  }
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
