/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, MakinaRocks, Inc.
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
 *   * Neither the name of MakinaRocks nor the names of its
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

/* Author: Vinnam Kim */

#include <moveit/collision_detection_fcl/mrx_custom/collision_detector_allocator_mrx.h>
#include <moveit/collision_detection/test_collision_common_pr2.h>

INSTANTIATE_TYPED_TEST_CASE_P(FCLCollisionCheck, CollisionDetectorTest,
                              collision_detection::CollisionDetectorAllocatorMRX);

TYPED_TEST_SUITE(CollisionDetectorTest, ::testing::Types<collision_detection::CollisionDetectorAllocatorMRX>);

TYPED_TEST(CollisionDetectorTest, DoesBlah)
{
  moveit::core::RobotState robot_state(this->robot_model_);
  robot_state.setToDefaultValues();
  robot_state.update();

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation().x() = 0.01;

  robot_state.updateStateWithLinkAt("base_link", Eigen::Isometry3d::Identity());
  robot_state.updateStateWithLinkAt("base_bellow_link", offset);
  robot_state.updateStateWithLinkAt("r_gripper_palm_link", Eigen::Isometry3d::Identity());
  robot_state.updateStateWithLinkAt("l_gripper_palm_link", offset);
  robot_state.update();

  this->acm_->setEntry("r_gripper_palm_link", "l_gripper_palm_link", false);

  {
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    this->cenv_->checkSelfCollision(req, res, robot_state, *this->acm_);
    // They should be collided because r_gripper_palm_link is at (0,0,0) and l_gripper_palm_link is at (0.01,0,0).
    ASSERT_TRUE(res.collision);
  }

  {
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;

    // Mesh explosion.
    // l_gripper_palm_link becomes super large and has no triangle collided with r_gripper_palm_link
    this->cenv_->setPadding(1.0);
    req.group_name = "l_end_effector";

    this->cenv_->checkSelfCollision(req, res, robot_state, *this->acm_);
    ASSERT_FALSE(res.collision);
  }
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
