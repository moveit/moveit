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
#include <moveit/warehouse/state_storage.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <boost/algorithm/string/join.hpp>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/variables_map.hpp>
#include <ros/ros.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "initialize_demo_db", ros::init_options::AnonymousName);

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
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
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::NodeHandle nh;
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
  if (!psm.getPlanningScene())
  {
    ROS_ERROR("Unable to initialize PlanningSceneMonitor");
    return 1;
  }
  
  moveit_warehouse::PlanningSceneStorage pss(vm.count("host") ? vm["host"].as<std::string>() : "",
                                             vm.count("port") ? vm["port"].as<std::size_t>() : 0);
  moveit_warehouse::ConstraintsStorage cs(vm.count("host") ? vm["host"].as<std::string>() : "",
                                          vm.count("port") ? vm["port"].as<std::size_t>() : 0);
  moveit_warehouse::RobotStateStorage rs(vm.count("host") ? vm["host"].as<std::string>() : "",
                                         vm.count("port") ? vm["port"].as<std::size_t>() : 0);
  
  // add default planning scenes
  moveit_msgs::PlanningScene psmsg;
  psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
  psmsg.name = "default1";
  pss.addPlanningScene(psmsg);
  psmsg.name = "default2";
  pss.addPlanningScene(psmsg);

  moveit_msgs::RobotState rsmsg;
  robot_state::robotStateToRobotStateMsg(psm.getPlanningScene()->getCurrentState(), rsmsg);
  rs.addRobotState(rsmsg, "S1");
  rs.addRobotState(rsmsg, "S2");

  const std::vector<std::string> &gnames = psm.getRobotModel()->getJointModelGroupNames();
  if (gnames.empty())
  {
    moveit_msgs::Constraints cmsg = kinematic_constraints::constructGoalConstraints(psm.getPlanningScene()->getCurrentState().getJointStateGroup(gnames.front()));
    cmsg.name = "Constr1";
    cs.addConstraints(cmsg);
    cmsg = kinematic_constraints::constructGoalConstraints(psm.getPlanningScene()->getCurrentState().getJointStateGroup(gnames.back()));
    cmsg.name = "Constr2";
    cs.addConstraints(cmsg);
  }
  
  return 0;
}
