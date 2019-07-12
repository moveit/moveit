/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Jens Petit
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Jens Petit */

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection_bullet/collision_world_bullet.h>
#include <moveit/collision_detection_bullet/collision_robot_bullet.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shape_operations.h>

#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>

typedef collision_detection::CollisionWorldBullet DefaultCWorldType;
typedef collision_detection::CollisionRobotBullet DefaultCRobotType;

/** \brief Brings the panda robot in user defined home position */
inline void setToHome(robot_state::RobotState& panda_state)
{
  panda_state.setToDefaultValues();
  double joint2 = -0.785;
  double joint4 = -2.356;
  double joint6 = 1.571;
  double joint7 = 0.785;
  panda_state.setJointPositions("panda_joint2", &joint2);
  panda_state.setJointPositions("panda_joint4", &joint4);
  panda_state.setJointPositions("panda_joint6", &joint6);
  panda_state.setJointPositions("panda_joint7", &joint7);
  panda_state.update();
}

class BulletCollisionDetectionTester : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    robot_model_ok_ = static_cast<bool>(robot_model_);

    acm_.reset(new collision_detection::AllowedCollisionMatrix(robot_model_->getLinkModelNames(), false));

    acm_->setEntry("panda_link0", "panda_link1", true);
    acm_->setEntry("panda_link1", "panda_link2", true);
    acm_->setEntry("panda_link2", "panda_link3", true);
    acm_->setEntry("panda_link3", "panda_link4", true);
    acm_->setEntry("panda_link4", "panda_link5", true);
    acm_->setEntry("panda_link5", "panda_link6", true);
    acm_->setEntry("panda_link6", "panda_link7", true);
    acm_->setEntry("panda_link7", "panda_hand", true);
    acm_->setEntry("panda_hand", "panda_rightfinger", true);
    acm_->setEntry("panda_hand", "panda_leftfinger", true);
    acm_->setEntry("panda_rightfinger", "panda_leftfinger", true);
    acm_->setEntry("panda_link5", "panda_link7", true);
    acm_->setEntry("panda_link6", "panda_hand", true);

    crobot_.reset(new DefaultCRobotType(robot_model_));
    cworld_.reset(new DefaultCWorldType());

    robot_state_.reset(new robot_state::RobotState(robot_model_));

    setToHome(*robot_state_);
  }

  void TearDown() override
  {
  }

protected:
  bool robot_model_ok_;

  robot_model::RobotModelPtr robot_model_;

  collision_detection::CollisionWorldPtr cworld_;
  collision_detection::CollisionRobotPtr crobot_;

  collision_detection::AllowedCollisionMatrixPtr acm_;

  robot_state::RobotStatePtr robot_state_;
};

/** \brief Correct setup testing. */
TEST_F(BulletCollisionDetectionTester, InitOK)
{
  ASSERT_TRUE(robot_model_ok_);
}

/** \brief Tests the default values specified in the SRDF if they are collision free. */
TEST_F(BulletCollisionDetectionTester, DefaultNotInCollision)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  crobot_->checkSelfCollision(req, res, *robot_state_, *acm_);
  ASSERT_FALSE(res.collision);
}

/** \brief A configuration where the robot should collide with itself. */
TEST_F(BulletCollisionDetectionTester, LinksInCollision)
{
  // Sets the joint values to zero which is a colliding configuration
  robot_state_->setToDefaultValues();
  robot_state_->update();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  crobot_->checkSelfCollision(req, res, *robot_state_, *acm_);
  ASSERT_TRUE(res.collision);
}

/** \brief Two boxes in collision in the world environment. */
TEST_F(BulletCollisionDetectionTester, WorldToWorldCollision)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  shapes::Shape* shape = new shapes::Box(.5, .5, .5);
  shapes::ShapeConstPtr shape_ptr(shape);

  Eigen::Isometry3d pos_1 = Eigen::Isometry3d::Identity();
  pos_1.translation().x() = 1;
  cworld_->getWorld()->addToObject("box", shape_ptr, pos_1);

  Eigen::Isometry3d pos_2 = Eigen::Isometry3d::Identity();
  pos_2.translation().x() = 1.2;
  cworld_->getWorld()->addToObject("box_2", shape_ptr, pos_2);

  cworld_->checkRobotCollision(req, res, *crobot_, *robot_state_, *acm_);

  ASSERT_TRUE(res.collision);
}

/** \brief Adding obstacles to the world which are tested against the robot. Simple cases. */
TEST_F(BulletCollisionDetectionTester, RobotWorldCollision_1)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  shapes::Shape* shape = new shapes::Box(.1, .1, .1);
  shapes::ShapeConstPtr shape_ptr(shape);

  Eigen::Isometry3d pos1 = Eigen::Isometry3d::Identity();
  pos1.translation().z() = 0.3;
  cworld_->getWorld()->addToObject("box", shape_ptr, pos1);

  crobot_->checkSelfCollision(req, res, *robot_state_, *acm_);
  ASSERT_FALSE(res.collision);
  res.clear();

  cworld_->checkRobotCollision(req, res, *crobot_, *robot_state_, *acm_);
  ASSERT_TRUE(res.collision);
  res.clear();

  cworld_->getWorld()->moveObject("box", pos1);
  cworld_->checkRobotCollision(req, res, *crobot_, *robot_state_, *acm_);
  ASSERT_TRUE(res.collision);
  res.clear();

  cworld_->getWorld()->moveObject("box", pos1);
  ASSERT_FALSE(res.collision);
}

/** \brief Adding obstacles to the world which are tested against the robot. */
TEST_F(BulletCollisionDetectionTester, RobotWorldCollision_2)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.max_contacts = 10;
  req.contacts = true;
  req.verbose = true;

  shapes::Shape* shape = new shapes::Box(.4, .4, .4);
  shapes::ShapeConstPtr shape_ptr(shape);

  Eigen::Isometry3d pos1 = Eigen::Isometry3d::Identity();
  pos1.translation().z() = 0.3;
  cworld_->getWorld()->addToObject("box", shape_ptr, pos1);
  cworld_->checkRobotCollision(req, res, *crobot_, *robot_state_, *acm_);
  ASSERT_TRUE(res.collision);
  ASSERT_GE(res.contact_count, 3u);
  res.clear();
}

/** \brief Tests the padding through expanding the link geometry in such a way that a collision occurs. */
TEST_F(BulletCollisionDetectionTester, PaddingTest)
{
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 10;
  collision_detection::CollisionResult res;

  cworld_->checkRobotCollision(req, res, *crobot_, *robot_state_, *acm_);
  ASSERT_FALSE(res.collision);
  res.clear();

  // Adding the box right in front of the robot hand
  shapes::Shape* shape = new shapes::Box(0.1, 0.1, 0.1);
  shapes::ShapeConstPtr shape_ptr(shape);

  Eigen::Isometry3d pos{ Eigen::Isometry3d::Identity() };
  pos.translation().x() = 0.43;
  pos.translation().y() = 0;
  pos.translation().z() = 0.55;
  cworld_->getWorld()->addToObject("box", shape_ptr, pos);

  crobot_->setLinkPadding("panda_hand", 0.08);
  cworld_->checkRobotCollision(req, res, *crobot_, *robot_state_, *acm_);
  ASSERT_TRUE(res.collision);
  res.clear();

  crobot_->setLinkPadding("panda_hand", 0.0);
  cworld_->checkRobotCollision(req, res, *crobot_, *robot_state_, *acm_);
  ASSERT_FALSE(res.collision);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
