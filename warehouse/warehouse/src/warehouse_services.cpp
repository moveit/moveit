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

/* Author: Dan Greenwalducan */

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <boost/algorithm/string/join.hpp>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <ros/ros.h>
#include <moveit_msgs/SaveRobotStateToWarehouse.h>
#include <moveit_msgs/ListRobotStatesInWarehouse.h>
#include <moveit_msgs/GetRobotStateFromWarehouse.h>
#include <moveit_msgs/CheckIfRobotStateExistsInWarehouse.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

bool store_state(moveit_msgs::SaveRobotStateToWarehouse::Request&  request,
                 moveit_msgs::SaveRobotStateToWarehouse::Response& response,
                 moveit_warehouse::RobotStateStorage* rs)
{
  const moveit_msgs::RobotState& state = request.state;
  if ("" == request.name)
  {
    ROS_ERROR("You must specify a name to store a state");
    return response.success = false;
  }
  rs->addRobotState(request.state, request.name, request.robot);
  return response.success = true;
}

bool list_states(moveit_msgs::ListRobotStatesInWarehouse::Request&  request,
                  moveit_msgs::ListRobotStatesInWarehouse::Response& response,
                  moveit_warehouse::RobotStateStorage* rs)
{
  if ("" == request.regex)
  {
    rs->getKnownRobotStates(response.states, request.robot);
  }
  else
  {
    rs->getKnownRobotStates(request.regex, response.states, request.robot);
  }
  return true;
}

bool has_state(moveit_msgs::CheckIfRobotStateExistsInWarehouse::Request&  request,
               moveit_msgs::CheckIfRobotStateExistsInWarehouse::Response& response,
               moveit_warehouse::RobotStateStorage* rs)
{
  response.exists = rs->hasRobotState(request.name, request.robot);
  return true;
}

bool get_state(moveit_msgs::GetRobotStateFromWarehouse::Request&  request,
               moveit_msgs::GetRobotStateFromWarehouse::Response& response,
               moveit_warehouse::RobotStateStorage* rs)
{
  moveit_warehouse::RobotStateWithMetadata state_buffer;
  rs->getRobotState(state_buffer, request.name, request.robot);
  response.state = static_cast<const moveit_msgs::RobotState&>(*state_buffer);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_warehouse_services");

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

  //  TODO(dg-shadow) check that host + port available. check db is working

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit_warehouse::RobotStateStorage rs(vm.count("host") ? vm["host"].as<std::string>() : "",
                                         vm.count("port") ? vm["port"].as<std::size_t>() : 0);
  std::vector<std::string> names;
  rs.getKnownRobotStates(names);
  if (names.empty())
    ROS_INFO("There are no previously stored robot states");
  else
  {
    ROS_INFO("Previously stored robot states:");
    for (std::size_t i = 0 ; i < names.size() ; ++i)
      ROS_INFO(" * %s", names[i].c_str());
  }

  boost::function<bool(moveit_msgs::SaveRobotStateToWarehouse::Request&  request,
                       moveit_msgs::SaveRobotStateToWarehouse::Response& response)>
    save_cb = boost::bind(&store_state, _1, _2, &rs);

  boost::function<bool(moveit_msgs::ListRobotStatesInWarehouse::Request&  request,
                       moveit_msgs::ListRobotStatesInWarehouse::Response& response)>
    list_cb = boost::bind(&list_states, _1, _2, &rs);

  boost::function<bool(moveit_msgs::GetRobotStateFromWarehouse::Request&  request,
                       moveit_msgs::GetRobotStateFromWarehouse::Response& response)>
    get_cb = boost::bind(&get_state, _1, _2, &rs);

  boost::function<bool(moveit_msgs::CheckIfRobotStateExistsInWarehouse::Request&  request,
                       moveit_msgs::CheckIfRobotStateExistsInWarehouse::Response& response)>
    has_cb = boost::bind(&has_state, _1, _2, &rs);



  ros::NodeHandle node("~");
  ros::ServiceServer save_state_server  = node.advertiseService("save_robot_state",  save_cb);
  ros::ServiceServer list_states_server = node.advertiseService("list_robot_states", list_cb);
  ros::ServiceServer get_state_server   = node.advertiseService("get_robot_state",   get_cb);
  ros::ServiceServer has_state_server   = node.advertiseService("has_robot_state",   has_cb);

//  ros::Subscriber state_sub = nh.subscribe("robot_state", 100, save_cb);
/*
  std::vector<std::string> topics; psm.getMonitoredTopics(topics);
  ROS_INFO_STREAM("Listening for scene updates on topics " <<
  boost::algorithm::join(topics, ", ")); ROS_INFO_STREAM("Listening
  for planning requests on topic " << mplan_req_sub.getTopic());
  ROS_INFO_STREAM("Listening for named constraints on topic " <<
  constr_sub.getTopic()); ROS_INFO_STREAM("Listening for states on
  topic " << state_sub.getTopic());

*/
  ros::waitForShutdown();
  return 0;
}
