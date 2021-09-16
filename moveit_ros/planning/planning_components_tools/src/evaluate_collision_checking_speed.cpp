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

#include <chrono>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

static const std::string ROBOT_DESCRIPTION = "robot_description";

void runCollisionDetection(unsigned int trials, const planning_scene_monitor::LockedPlanningSceneRO& scene,
                           moveit::core::RobotStatePtr& state, const moveit::core::JointModelGroup* jmg)
{
  ROS_INFO("Starting benchmark");
  collision_detection::CollisionRequest req;
  if (jmg != nullptr)
    req.group_name = jmg->getName();

  auto duration = std::chrono::duration<double>::zero();

  for (unsigned int i = 0; i < trials; ++i)
  {
    collision_detection::CollisionResult res;

    if (jmg == nullptr)
      state->setToRandomPositions();
    else
      state->setToRandomPositions(jmg);

    const auto start = std::chrono::system_clock::now();
    scene->checkCollision(req, res, *state);
    duration += std::chrono::system_clock::now() - start;
  }

  ROS_INFO("Performed %lf collision checks per second", (double)trials / duration.count());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "evaluate_collision_checking_speed");

  unsigned int iters = 5;
  unsigned int trials = 10000;
  std::string group_name("");
  boost::program_options::options_description desc;
  desc.add_options()("trials", boost::program_options::value<unsigned int>(&trials)->default_value(trials),
                     "Number of collision checks for each iteration")(
      "iters", boost::program_options::value<unsigned int>(&iters)->default_value(iters), "Number of iterations")(
      "groupname", boost::program_options::value<std::string>(&group_name)->default_value(group_name),
      "Joint group name")("help", "this screen");
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

  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(ROBOT_DESCRIPTION);
  psm->stopPublishingPlanningScene();
  psm->stopSceneMonitor();
  psm->stopStateMonitor();
  psm->stopWorldGeometryMonitor();

  if (psm->getPlanningScene())
  {
    if (!psm->requestPlanningSceneState())
      throw std::runtime_error("Failed to request scene.");

    planning_scene_monitor::LockedPlanningSceneRO ps(psm);

    for (const auto& id : ps->getWorld()->getObjectIds())
    {
      const size_t shape_size = ps->getWorld()->getObject(id)->shapes_.size();
      ROS_INFO("id : %s, shape_size = %zu", id.c_str(), shape_size);
    }

    moveit::core::RobotStatePtr state;
    state.reset(new moveit::core::RobotState(ps->getRobotModel()));
    state->setRandomNumberGenerator(3003);

    const moveit::core::JointModelGroup* jmg = nullptr;

    if (group_name != "")
      jmg = state->getJointModelGroup(group_name);

    ROS_INFO_COND(jmg != nullptr, "Group name : %s, JMG name : %s", group_name.c_str(), jmg->getName().c_str());

    for (size_t i = 0; i < iters; i++)
      runCollisionDetection(trials, ps, state, jmg);
  }
  else
    ROS_ERROR("Planning scene not configured");

  return 0;
}
