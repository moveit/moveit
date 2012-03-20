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

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_models/transforms.h>
#include <collision_detection/fcl/collision_world.h>
#include <collision_detection/fcl/collision_robot.h>
#include <geometric_shapes/shape_operations.h>

#include <boost/filesystem.hpp>


typedef collision_detection::CollisionWorldFCL DefaultCWorldType;
typedef collision_detection::CollisionRobotFCL DefaultCRobotType;

class FclCollisionDetectionTester : public testing::Test{

protected:

  virtual void SetUp() 
  {
    urdf_model_.reset(new urdf::Model());
    srdf_model_.reset(new srdf::Model());
    urdf_ok_ = urdf_model_->initFile("../planning_models/test/urdf/robot.xml");
    srdf_ok_ = srdf_model_->initFile(*urdf_model_, "../planning_models/test/srdf/robot.xml");

    kmodel_.reset(new planning_models::KinematicModel(urdf_model_, srdf_model_));

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

  boost::shared_ptr<urdf::Model>           urdf_model_;
  boost::shared_ptr<srdf::Model>           srdf_model_;
  
  planning_models::KinematicModelPtr             kmodel_;
  
  planning_models::TransformsPtr                 ftf_;
  planning_models::TransformsConstPtr            ftf_const_;
  
  boost::shared_ptr<DefaultCRobotType>        crobot_;
  boost::shared_ptr<DefaultCWorldType>        cworld_;
  
  collision_detection::AllowedCollisionMatrixPtr acm_;

};

TEST_F(FclCollisionDetectionTester, InitOK)
{
    ASSERT_TRUE(urdf_ok_);
    ASSERT_TRUE(srdf_ok_);
}

TEST_F(FclCollisionDetectionTester, DefaultNotInCollision)
{
  planning_models::KinematicState kstate(kmodel_);
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

  planning_models::KinematicState kstate(kmodel_);
  kstate.setToDefaultValues();

  Eigen::Affine3d offset = Eigen::Affine3d::Identity();
  offset.translation().x() = .01;

  kstate.getLinkState("base_link")->updateGivenGlobalLinkTransform(Eigen::Affine3d::Identity());
  kstate.getLinkState("base_bellow_link")->updateGivenGlobalLinkTransform(offset);
  acm_->setEntry("base_link", "base_bellow_link", false);
  crobot_->checkSelfCollision(req, res1, kstate, *acm_);
  ASSERT_TRUE(res1.collision);

  acm_->setEntry("base_link", "base_bellow_link", true);
  crobot_->checkSelfCollision(req, res2, kstate, *acm_);
  ASSERT_FALSE(res2.collision);
  
  //  req.verbose = true;
  kstate.getLinkState("r_gripper_palm_link")->updateGivenGlobalLinkTransform(Eigen::Affine3d::Identity());
  kstate.getLinkState("l_gripper_palm_link")->updateGivenGlobalLinkTransform(offset);

  acm_->setEntry("r_gripper_palm_link", "l_gripper_palm_link", false);
  crobot_->checkSelfCollision(req, res3, kstate, *acm_);
  ASSERT_TRUE(res3.collision);
}

TEST_F(FclCollisionDetectionTester, ContactReporting)
{
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 1;

  planning_models::KinematicState kstate(kmodel_);
  kstate.setToDefaultValues();

  Eigen::Affine3d offset = Eigen::Affine3d::Identity();
  offset.translation().x() = .01;

  kstate.getLinkState("base_link")->updateGivenGlobalLinkTransform(Eigen::Affine3d::Identity());
  kstate.getLinkState("base_bellow_link")->updateGivenGlobalLinkTransform(offset);
  kstate.getLinkState("r_gripper_palm_link")->updateGivenGlobalLinkTransform(Eigen::Affine3d::Identity());
  kstate.getLinkState("l_gripper_palm_link")->updateGivenGlobalLinkTransform(offset);

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

  planning_models::KinematicState kstate(kmodel_);
  kstate.setToDefaultValues();

  Eigen::Affine3d pos1 = Eigen::Affine3d::Identity();
  Eigen::Affine3d pos2 = Eigen::Affine3d::Identity();

  pos1.translation().x() = 5.0;
  pos2.translation().x() = 5.01;

  kstate.getLinkState("r_gripper_palm_link")->updateGivenGlobalLinkTransform(pos1);
  kstate.getLinkState("l_gripper_palm_link")->updateGivenGlobalLinkTransform(pos2);

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
  kstate.getLinkState("r_gripper_palm_link")->updateGivenGlobalLinkTransform(pos1);
  kstate.getLinkState("l_gripper_palm_link")->updateGivenGlobalLinkTransform(pos2);

  collision_detection::CollisionResult res2;
  crobot_->checkSelfCollision(req, res2, kstate, *acm_);
  ASSERT_TRUE(res2.collision);
  ASSERT_EQ(res2.contacts.size(),1);
  ASSERT_EQ(res2.contacts.begin()->second.size(),1);

  for(collision_detection::CollisionResult::ContactMap::const_iterator it = res2.contacts.begin();
      it != res2.contacts.end();
      it++) {
    ROS_INFO_STREAM("Col x is " << it->second[0].pos.x());
    EXPECT_NEAR(it->second[0].pos.x(), 3.0, 0.33);
  }

  pos1 = Eigen::Affine3d(Eigen::Translation3d(3.0,0.0,0.0)*Eigen::Quaterniond::Identity());
  pos2 = Eigen::Affine3d(Eigen::Translation3d(3.0,0.0,0.0)*Eigen::Quaterniond(M_PI/4.0, 0.0, M_PI/4.0, 0.0));
  kstate.getLinkState("r_gripper_palm_link")->updateGivenGlobalLinkTransform(pos1);
  kstate.getLinkState("l_gripper_palm_link")->updateGivenGlobalLinkTransform(pos2);

  collision_detection::CollisionResult res3;
  crobot_->checkSelfCollision(req, res2, kstate, *acm_);
  ASSERT_FALSE(res3.collision);
}

TEST_F(FclCollisionDetectionTester, AttachedBodyTester) {
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  acm_.reset(new collision_detection::AllowedCollisionMatrix(kmodel_->getLinkModelNames(), true)); 

  planning_models::KinematicState kstate(kmodel_);
  kstate.setToDefaultValues();

  Eigen::Affine3d pos1 = Eigen::Affine3d::Identity();
  pos1.translation().x() = 5.0;

  kstate.getLinkState("r_gripper_palm_link")->updateGivenGlobalLinkTransform(pos1);
  crobot_->checkSelfCollision(req, res, kstate, *acm_);
  ASSERT_FALSE(res.collision);  

  shapes::Shape* shape = new shapes::Box(.1,.1,.1);
  cworld_->addToObject("box", shape, pos1);
  
  res = collision_detection::CollisionResult();
  cworld_->checkRobotCollision(req, res, *crobot_, kstate, *acm_);
  ASSERT_TRUE(res.collision);  

  //deletes shape
  cworld_->removeObject("box");

  shape = new shapes::Box(.1,.1,.1);
  std::vector<shapes::Shape*> shapes;
  std::vector<Eigen::Affine3d> poses;
  shapes.push_back(shape);
  poses.push_back(Eigen::Affine3d::Identity());
  std::vector<std::string> touch_links;
  kstate.getLinkState("r_gripper_palm_link")->attachBody("box", shapes, poses, touch_links);

  res = collision_detection::CollisionResult();
  crobot_->checkSelfCollision(req, res, kstate, *acm_);
  ASSERT_TRUE(res.collision);  

  //deletes shape
  kstate.getLinkState("r_gripper_palm_link")->clearAttachedBody("box");

  touch_links.push_back("r_gripper_palm_link");
  shapes[0] = new shapes::Box(.1,.1,.1); 
  kstate.getLinkState("r_gripper_palm_link")->attachBody("box", shapes, poses, touch_links);

  res = collision_detection::CollisionResult();
  crobot_->checkSelfCollision(req, res, kstate, *acm_);
  ASSERT_FALSE(res.collision);  

  pos1.translation().x() = 5.01;
  shapes::Shape* coll = new shapes::Box(.1, .1, .1);
  cworld_->addToObject("coll", coll, pos1);  
  res = collision_detection::CollisionResult();
  cworld_->checkRobotCollision(req, res, *crobot_, kstate, *acm_);
  ASSERT_TRUE(res.collision);  

  acm_->setEntry("coll", "r_gripper_palm_link", true);
  res = collision_detection::CollisionResult();
  cworld_->checkRobotCollision(req, res, *crobot_, kstate, *acm_);
  ASSERT_TRUE(res.collision);  
}

TEST_F(FclCollisionDetectionTester, DiffSceneTester) {

  planning_models::KinematicState kstate(kmodel_);
  kstate.setToDefaultValues();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  collision_detection::CollisionRobotFCL new_crobot(*crobot_.get());

  ros::WallTime before = ros::WallTime::now();
  new_crobot.checkSelfCollision(req,res,kstate);
  double first_check = (ros::WallTime::now()-before).toSec();
  before = ros::WallTime::now();
  new_crobot.checkSelfCollision(req,res,kstate);
  double second_check = (ros::WallTime::now()-before).toSec();

  EXPECT_LT(fabs(first_check-second_check), .0001);
  
  ROS_INFO_STREAM("Diff " << first_check-second_check << " first " << first_check << " second " << second_check);

  std::vector<shapes::Shape*> shapes;
  shapes.resize(1);

  boost::filesystem::path path(boost::filesystem::current_path());
  
  shapes[0] = shapes::createMeshFromFilename("file://"+path.string()+"/../planning_models/test/kinect.dae");

  std::vector<Eigen::Affine3d> poses;
  poses.push_back(Eigen::Affine3d::Identity());

  std::vector<std::string> touch_links;
  kstate.getLinkState("r_gripper_palm_link")->attachBody("kinect", shapes, poses, touch_links);

  before = ros::WallTime::now();
  new_crobot.checkSelfCollision(req,res,kstate);
  first_check = (ros::WallTime::now()-before).toSec();
  before = ros::WallTime::now();
  new_crobot.checkSelfCollision(req,res,kstate);
  second_check = (ros::WallTime::now()-before).toSec();

  //the first check is going to take a while, as data must be constructed
  EXPECT_LT(second_check, .0001);

  ROS_INFO_STREAM("Attached diff " << first_check-second_check << " first " << first_check << " second " << second_check);
  
  collision_detection::CollisionRobotFCL other_new_crobot(*crobot_.get());
  before = ros::WallTime::now();
  new_crobot.checkSelfCollision(req,res,kstate);
  first_check = (ros::WallTime::now()-before).toSec();
  before = ros::WallTime::now();
  new_crobot.checkSelfCollision(req,res,kstate);
  second_check = (ros::WallTime::now()-before).toSec();

  EXPECT_LT(fabs(first_check-second_check), .0001);

  ROS_INFO_STREAM("Attached robot second diff " << first_check-second_check << " first " << first_check << " second " << second_check);

}

TEST_F(FclCollisionDetectionTester, ConvertObjectToAttached) {

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  boost::filesystem::path path(boost::filesystem::current_path());
  shapes::Shape* shape = shapes::createMeshFromFilename("file://"+path.string()+"/../planning_models/test/kinect.dae"); 
  Eigen::Affine3d pos1 = Eigen::Affine3d::Identity();

  cworld_->addToObject("kinect", shape, pos1);

  planning_models::KinematicState kstate(kmodel_);
  kstate.setToDefaultValues();

  ros::WallTime before = ros::WallTime::now();
  cworld_->checkRobotCollision(req, res, *crobot_, kstate);
  double first_check = (ros::WallTime::now()-before).toSec();
  before = ros::WallTime::now();
  cworld_->checkRobotCollision(req, res, *crobot_, kstate);
  double second_check = (ros::WallTime::now()-before).toSec();

  EXPECT_LT(second_check, .0001);

  ROS_INFO_STREAM("Check diff " << first_check-second_check << " first " << first_check << " second " << second_check);
  
  collision_detection::CollisionWorld::ObjectPtr object = cworld_->getObject("kinect");
  cworld_->removeObject("kinect");
  
  planning_models::KinematicState kstate1(kmodel_);
  planning_models::KinematicState kstate2(kmodel_);
  kstate1.setToDefaultValues();
  kstate2.setToDefaultValues();
  
  std::vector<std::string> touch_links;
  kstate1.getLinkState("r_gripper_palm_link")->attachBody("kinect", object->shapes_, object->shape_poses_, touch_links);
  kstate2.getLinkState("r_gripper_palm_link")->attachBody("kinect", object->shapes_, object->shape_poses_, touch_links);
  
  object->shapes_.clear();

  //going to take a while, but that's fine
  crobot_->checkSelfCollision(req,res,kstate1);

  before = ros::WallTime::now();
  crobot_->checkSelfCollision(req,res,kstate1);
  first_check = (ros::WallTime::now()-before).toSec();
  before = ros::WallTime::now();
  crobot_->checkSelfCollision(req,res,kstate2);
  second_check = (ros::WallTime::now()-before).toSec();

  EXPECT_LT(fabs(first_check-second_check), .0001);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
