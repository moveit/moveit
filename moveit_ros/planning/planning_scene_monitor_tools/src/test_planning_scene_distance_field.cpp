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

#include <gtest/gtest.h>
#include <planning_scene/planning_scene.h>
#include <collision_distance_field/collision_detector_allocator_distance_field.h>

TEST(PlanningScene, LoadRestore)
{
    boost::shared_ptr<urdf::Model> urdf_model(new urdf::Model());
    boost::shared_ptr<srdf::Model> srdf_model(new srdf::Model());
    urdf_model->initFile("../../moveit/planning_models/test/urdf/robot.xml");
    planning_scene::PlanningScene ps;
    ps.setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorDistanceField::create());
    ps.configure(urdf_model, srdf_model);
    EXPECT_TRUE(ps.isConfigured());
    moveit_msgs::PlanningScene ps_msg;
    ps.getPlanningSceneMsg(ps_msg);
    ps.setPlanningSceneMsg(ps_msg);
}

TEST(PlanningScene, LoadRestoreDiff)
{
    boost::shared_ptr<urdf::Model> urdf_model(new urdf::Model());
    boost::shared_ptr<srdf::Model> srdf_model(new srdf::Model());
    urdf_model->initFile("../../moveit/planning_models/test/urdf/robot.xml");
    planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene());
    ps->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorDistanceField::create());
    ps->configure(urdf_model, srdf_model);
    EXPECT_TRUE(ps->isConfigured());

    collision_detection::CollisionWorld &world = *ps->getWorldNonConst();
    Eigen::Affine3d id = Eigen::Affine3d::Identity();
    world.addToObject("sphere", shapes::ShapeConstPtr(new shapes::Sphere(0.4)), id);
    
    moveit_msgs::PlanningScene ps_msg;
    EXPECT_TRUE(planning_scene::PlanningScene::isEmpty(ps_msg));
    ps->getPlanningSceneMsg(ps_msg);
    ps->setPlanningSceneMsg(ps_msg);
    EXPECT_FALSE(planning_scene::PlanningScene::isEmpty(ps_msg));
    EXPECT_TRUE(ps->getWorld()->hasObject("sphere"));
    
    planning_scene::PlanningScene next(ps);
    EXPECT_TRUE(next.isConfigured());
    EXPECT_TRUE(next.getWorld()->hasObject("sphere"));
    next.getWorldNonConst()->addToObject("sphere2", shapes::ShapeConstPtr(new shapes::Sphere(0.5)), id);
    EXPECT_EQ(next.getWorld()->size(), 2);
    EXPECT_EQ(ps->getWorld()->size(), 1);
    next.getPlanningSceneDiffMsg(ps_msg);
    EXPECT_EQ(ps_msg.world.collision_objects.size(), 1);
    next.decoupleParent();
    moveit_msgs::PlanningScene ps_msg2;
    next.getPlanningSceneDiffMsg(ps_msg2);	
    EXPECT_EQ(ps_msg2.world.collision_objects.size(), 0);
    next.getPlanningSceneMsg(ps_msg);	
    EXPECT_EQ(ps_msg.world.collision_objects.size(), 2);
    ps->setPlanningSceneMsg(ps_msg);
    EXPECT_EQ(ps->getWorld()->size(), 2);
}

TEST(PlanningScene, MakeAttachedDiff)
{
  boost::shared_ptr<urdf::Model> urdf_model(new urdf::Model());
  boost::shared_ptr<srdf::Model> srdf_model(new srdf::Model());
  urdf_model->initFile("../../moveit/planning_models/test/urdf/robot.xml");
  planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene());
  ps->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorDistanceField::create());
  ps->configure(urdf_model, srdf_model);
  EXPECT_TRUE(ps->isConfigured());
  
  collision_detection::CollisionWorld &world = *ps->getWorldNonConst();
  Eigen::Affine3d id = Eigen::Affine3d::Identity();
  world.addToObject("sphere", shapes::ShapeConstPtr(new shapes::Sphere(0.4)), id);

  planning_scene::PlanningScenePtr attached_object_diff_scene(new planning_scene::PlanningScene(ps));
  
  moveit_msgs::AttachedCollisionObject att_obj;
  att_obj.link_name = "r_wrist_roll_link";
  att_obj.object.operation = moveit_msgs::CollisionObject::ADD;
  att_obj.object.id = "sphere";
  
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = "right_arm";
  attached_object_diff_scene->processAttachedCollisionObjectMsg(att_obj);
  attached_object_diff_scene->checkCollision(req,res);
  ps->checkCollision(req, res);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
