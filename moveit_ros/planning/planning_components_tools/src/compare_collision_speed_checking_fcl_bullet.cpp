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

/* Author: Ioan Sucan, Sachin Chitta */

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bt.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

static const std::string ROBOT_DESCRIPTION = "robot_description";

void runCollisionDetection(unsigned int trials, const planning_scene::PlanningScene* scene,
                           const robot_state::RobotState state)
{
  ROS_INFO("Starting...");
  collision_detection::CollisionRequest req;
  ros::WallTime start = ros::WallTime::now();
  for (unsigned int i = 0; i < trials; ++i)
  {
    collision_detection::CollisionResult res;
    scene->checkSelfCollision(req, res, state);
  }
  double duration = (ros::WallTime::now() - start).toSec();
  ROS_INFO("Performed %lf collision checks per second", (double)trials / duration);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compare_collision_checking_speed");

  unsigned int trials = 1000000;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBt::create());
  planning_scene_monitor::PlanningSceneMonitor psm(planning_scene, ROBOT_DESCRIPTION);
  psm.startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  psm.startSceneMonitor();

  if (psm.getPlanningScene())
  {
    ros::Duration(0.5).sleep();
    std::vector<robot_state::RobotState> states;
    ROS_INFO("Sampling valid states...");

    // sample a valid state
    robot_state::RobotState& current_state = planning_scene->getCurrentStateNonConst();
    collision_detection::CollisionRequest req;
    do
    {
      current_state.setToRandomPositions();
      current_state.update();
      collision_detection::CollisionResult res;
      psm.getPlanningScene()->checkSelfCollision(req, res);
      ROS_INFO_STREAM("Found state " << (res.collision ? "in collision" : "not in collision"));
      if (!res.collision)
        break;
    } while (true);

    ROS_INFO_STREAM("Using Bullet");
    runCollisionDetection(trials, planning_scene.get(), current_state);

    planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
    ROS_INFO_STREAM("Using FCL");
    runCollisionDetection(trials, planning_scene.get(), current_state);
  }
  else
    ROS_ERROR("Planning scene not configured");

  return 0;
}
