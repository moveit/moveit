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

void runCollisionDetection(unsigned int id, unsigned int trials, const planning_scene::PlanningScene* scene,
                           const robot_state::RobotState state)
{
  ROS_INFO("Starting thread %u", id);
  collision_detection::CollisionRequest req;
  ros::WallTime start = ros::WallTime::now();
  for (unsigned int i = 0; i < trials; ++i)
  {
    collision_detection::CollisionResult res;
    scene->checkCollision(req, res, state);
  }
  double duration = (ros::WallTime::now() - start).toSec();
  ROS_INFO("Thread %u performed %lf collision checks per second", id, (double)trials / duration);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compare_collision_checking_speed");

  unsigned int nthreads = 2;
  unsigned int trials = 10000;
  boost::program_options::options_description desc;
  desc.add_options()("nthreads", boost::program_options::value<unsigned int>(&nthreads)->default_value(nthreads),
                     "Number of threads to use")(
      "trials", boost::program_options::value<unsigned int>(&trials)->default_value(trials),
      "Number of collision checks to perform with each thread")("wait",
                                                                "Wait for a user command (so the planning scene can be "
                                                                "updated in thre background)")("help", "this screen");
  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po = boost::program_options::parse_command_line(argc, argv, desc);
  boost::program_options::store(po, vm);
  boost::program_options::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 0;
  }

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
    if (vm.count("wait"))
    {
      psm.startWorldGeometryMonitor();
      psm.startSceneMonitor();
      std::cout << "Listening to planning scene updates. Press Enter to continue ..." << std::endl;
      std::cin.get();
    }
    else
      ros::Duration(0.5).sleep();

    std::vector<robot_state::RobotState> states;
    ROS_INFO("Sampling %u valid states...", nthreads);
    for (unsigned int i = 0; i < nthreads; ++i)
    {
      // sample a valid state
      robot_state::RobotState& current_state = planning_scene->getCurrentStateNonConst();

      collision_detection::CollisionRequest req;
      do
      {
        current_state.setToRandomPositions();
        current_state.update();
        collision_detection::CollisionResult res;
        res.clear();
        psm.getPlanningScene()->checkSelfCollision(req, res);
        ROS_INFO_STREAM((res.collision ? "In collision" : "Not in collision") << " for thread " << i);
        if (!res.collision)
          break;
      } while (true);
      states.push_back(robot_state::RobotState(current_state));
    }

    std::vector<boost::thread*> threads;

    for (unsigned int i = 0; i < states.size(); ++i)
      threads.push_back(new boost::thread(
          boost::bind(&runCollisionDetection, i, trials, psm.getPlanningScene().get(), states[i])));

    for (unsigned int i = 0; i < states.size(); ++i)
    {
      threads[i]->join();
      delete threads[i];
    }
  }
  else
    ROS_ERROR("Planning scene not configured");

  return 0;
}
