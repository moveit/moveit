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
  
  collision_detection::CollisionRobotPtr         crobot_;
  collision_detection::CollisionWorldPtr         cworld_;
  
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


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
