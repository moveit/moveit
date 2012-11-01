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

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <boost/algorithm/string/join.hpp>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/variables_map.hpp>
#include <ros/console.h>

static const std::string ROBOT_DESCRIPTION="robot_description";
static bool ALLOW_DUPLICATE_SCENE_NAMES = false;

void onSceneUpdate(planning_scene_monitor::PlanningSceneMonitor *psm, moveit_warehouse::PlanningSceneStorage *pss)
{
  ROS_INFO("Received an update to the planning scene...");
  
  if (!psm->getPlanningScene()->getName().empty())
  {
    if (ALLOW_DUPLICATE_SCENE_NAMES || !pss->hasPlanningScene(psm->getPlanningScene()->getName()))
    {
      moveit_msgs::PlanningScene psmsg;
      psm->getPlanningScene()->getPlanningSceneMsg(psmsg);
      pss->addPlanningScene(psmsg);
    }
    else
      ROS_INFO("Scene '%s' was previously added. Not adding again.", psm->getPlanningScene()->getName().c_str());
  }
  else
    ROS_INFO("Scene name is empty. Not saving.");
}

void onConstraints(const moveit_msgs::ConstraintsConstPtr &msg, moveit_warehouse::ConstraintsStorage *cs)
{
  if (msg->name.empty())
  {
    ROS_INFO("No name specified for constraints. Not saving.");
    return;
  }
  
  if (cs->hasConstraints(msg->name))
  {
    ROS_INFO("Replacing constraints '%s'", msg->name.c_str());    
    cs->removeConstraints(msg->name);
    cs->addConstraints(*msg);
  }
  else
  {
    ROS_INFO("Adding constraints '%s'", msg->name.c_str());    
    cs->addConstraints(*msg);
  }
}

void onMotionPlanRequest(const moveit_msgs::MotionPlanRequestConstPtr &req,
                         planning_scene_monitor::PlanningSceneMonitor *psm,
                         moveit_warehouse::PlanningSceneStorage *pss)
{
  if (psm->getPlanningScene()->getName().empty())
  {
    ROS_INFO("Scene name is empty. Not saving planning request.");
    return;
  }
  pss->addPlanningRequest(*req, psm->getPlanningScene()->getName());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_to_warehouse", ros::init_options::AnonymousName);
  
  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("allow-duplicate-scene-names", "Allow adding scenes with same name.")
    ("host", boost::program_options::value<std::string>(), "Host for the MongoDB.")
    ("port", boost::program_options::value<std::size_t>(), "Port for the MongoDB.");
  
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);
  
  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }
  
  if (vm.count("allow-duplicate-scene-names"))
  {
    ALLOW_DUPLICATE_SCENE_NAMES = true;
  }
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::NodeHandle nh;
  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, tf);
  if (!psm.getPlanningScene() || !psm.getPlanningScene()->isConfigured())
  {
    ROS_ERROR("Unable to initialize PlanningSceneMonitor");
    return 1;
  }
  
  psm.startSceneMonitor();
  psm.startWorldGeometryMonitor();
  moveit_warehouse::PlanningSceneStorage pss(vm.count("host") ? vm["host"].as<std::string>() : "",
                                             vm.count("port") ? vm["port"].as<std::size_t>() : 0);
  moveit_warehouse::ConstraintsStorage cs(vm.count("host") ? vm["host"].as<std::string>() : "",
                                          vm.count("port") ? vm["port"].as<std::size_t>() : 0);
  std::vector<std::string> names;
  std::vector<ros::Time> times;
  pss.getPlanningSceneNamesAndTimes(names, times);
  if (names.empty())
    ROS_INFO("There are no previously stored scenes");
  else
  {
    ROS_INFO("Previously stored scenes:");
    for (std::size_t i = 0 ; i < names.size() ; ++i)
      ROS_INFO(" * %s", names[i].c_str());
  }
  
  psm.addUpdateCallback(boost::bind(&onSceneUpdate, &psm, &pss));
  boost::function<void(const moveit_msgs::MotionPlanRequestConstPtr&)> callback1 = boost::bind(&onMotionPlanRequest, _1, &psm, &pss);
  ros::Subscriber mplan_req_sub = nh.subscribe("motion_plan_request", 100, callback1);
  boost::function<void(const moveit_msgs::ConstraintsConstPtr&)> callback2 = boost::bind(&onConstraints, _1, &cs);
  ros::Subscriber constr_sub = nh.subscribe("constraints", 100, callback2);
  std::vector<std::string> topics;
  psm.getMonitoredTopics(topics);
  ROS_INFO_STREAM("Listening for scene updates on topics " << boost::algorithm::join(topics, ", "));
  ROS_INFO_STREAM("Listening for planning requests on topic " << mplan_req_sub.getTopic());
  ROS_INFO_STREAM("Listening for named constraints on topic " << constr_sub.getTopic());

  ros::waitForShutdown();
  return 0;
}
