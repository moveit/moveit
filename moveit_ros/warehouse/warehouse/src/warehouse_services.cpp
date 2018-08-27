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

/* Author: Dan Greenwald */

#include <ros/ros.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit_msgs/SaveRobotStateToWarehouse.h>
#include <moveit_msgs/ListRobotStatesInWarehouse.h>
#include <moveit_msgs/GetRobotStateFromWarehouse.h>
#include <moveit_msgs/CheckIfRobotStateExistsInWarehouse.h>
#include <moveit_msgs/DeleteRobotStateFromWarehouse.h>
#include <moveit_msgs/RenameRobotStateInWarehouse.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

bool storeState(moveit_msgs::SaveRobotStateToWarehouse::Request& request,
                moveit_msgs::SaveRobotStateToWarehouse::Response& response, moveit_warehouse::RobotStateStorage* rs)
{
  if (request.name.empty())
  {
    ROS_ERROR("You must specify a name to store a state");
    return (response.success = false);
  }
  rs->addRobotState(request.state, request.name, request.robot);
  return (response.success = true);
}

bool listStates(moveit_msgs::ListRobotStatesInWarehouse::Request& request,
                moveit_msgs::ListRobotStatesInWarehouse::Response& response, moveit_warehouse::RobotStateStorage* rs)
{
  if (request.regex.empty())
  {
    rs->getKnownRobotStates(response.states, request.robot);
  }
  else
  {
    rs->getKnownRobotStates(request.regex, response.states, request.robot);
  }
  return true;
}

bool hasState(moveit_msgs::CheckIfRobotStateExistsInWarehouse::Request& request,
              moveit_msgs::CheckIfRobotStateExistsInWarehouse::Response& response,
              moveit_warehouse::RobotStateStorage* rs)
{
  response.exists = rs->hasRobotState(request.name, request.robot);
  return true;
}

bool getState(moveit_msgs::GetRobotStateFromWarehouse::Request& request,
              moveit_msgs::GetRobotStateFromWarehouse::Response& response, moveit_warehouse::RobotStateStorage* rs)
{
  if (!rs->hasRobotState(request.name, request.robot))
  {
    ROS_ERROR_STREAM("No state called '" << request.name << "' for robot '" << request.robot << "'.");
    moveit_msgs::RobotState dummy;
    response.state = dummy;
    return false;
  }
  moveit_warehouse::RobotStateWithMetadata state_buffer;
  rs->getRobotState(state_buffer, request.name, request.robot);
  response.state = static_cast<const moveit_msgs::RobotState&>(*state_buffer);
  return true;
}

bool renameState(moveit_msgs::RenameRobotStateInWarehouse::Request& request,
                 moveit_msgs::RenameRobotStateInWarehouse::Response& response, moveit_warehouse::RobotStateStorage* rs)
{
  if (!rs->hasRobotState(request.old_name, request.robot))
  {
    ROS_ERROR_STREAM("No state called '" << request.old_name << "' for robot '" << request.robot << "'.");
    return false;
  }
  rs->renameRobotState(request.old_name, request.new_name, request.robot);
  return true;
}

bool deleteState(moveit_msgs::DeleteRobotStateFromWarehouse::Request& request,
                 moveit_msgs::DeleteRobotStateFromWarehouse::Response& response,
                 moveit_warehouse::RobotStateStorage* rs)
{
  if (!rs->hasRobotState(request.name, request.robot))
  {
    ROS_ERROR_STREAM("No state called '" << request.name << "' for robot '" << request.robot << "'.");
    return false;
  }
  rs->removeRobotState(request.name, request.robot);
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_warehouse_services");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node;
  std::string host;

  int port;
  float connection_timeout;
  int connection_retries;

  node.param<std::string>("warehouse_host", host, "localhost");
  node.param<int>("warehouse_port", port, 33829);
  node.param<float>("warehouse_db_connection_timeout", connection_timeout, 5.0);
  node.param<int>("warehouse_db_connection_retries", connection_retries, 5);

  warehouse_ros::DatabaseConnection::Ptr conn;

  try
  {
    conn = moveit_warehouse::loadDatabase();
    conn->setParams(host, port, connection_timeout);

    ROS_INFO("Connecting to warehouse on %s:%d", host.c_str(), port);
    int tries = 0;
    while (!conn->connect())
    {
      ++tries;
      ROS_WARN("Failed to connect to DB on %s:%d (try %d/%d).", host.c_str(), port, tries, connection_retries);
      if (tries == connection_retries)
      {
        ROS_FATAL("Failed to connect too many times, giving up");
        return 1;
      }
    }
  }
  catch (std::exception& ex)
  {
    ROS_ERROR("%s", ex.what());
    return 1;
  }

  moveit_warehouse::RobotStateStorage rs(conn);

  std::vector<std::string> names;
  rs.getKnownRobotStates(names);
  if (names.empty())
    ROS_INFO("There are no previously stored robot states");
  else
  {
    ROS_INFO("Previously stored robot states:");
    for (std::size_t i = 0; i < names.size(); ++i)
      ROS_INFO(" * %s", names[i].c_str());
  }

  boost::function<bool(moveit_msgs::SaveRobotStateToWarehouse::Request & request,
                       moveit_msgs::SaveRobotStateToWarehouse::Response & response)>
      save_cb = boost::bind(&storeState, _1, _2, &rs);

  boost::function<bool(moveit_msgs::ListRobotStatesInWarehouse::Request & request,
                       moveit_msgs::ListRobotStatesInWarehouse::Response & response)>
      list_cb = boost::bind(&listStates, _1, _2, &rs);

  boost::function<bool(moveit_msgs::GetRobotStateFromWarehouse::Request & request,
                       moveit_msgs::GetRobotStateFromWarehouse::Response & response)>
      get_cb = boost::bind(&getState, _1, _2, &rs);

  boost::function<bool(moveit_msgs::CheckIfRobotStateExistsInWarehouse::Request & request,
                       moveit_msgs::CheckIfRobotStateExistsInWarehouse::Response & response)>
      has_cb = boost::bind(&hasState, _1, _2, &rs);

  boost::function<bool(moveit_msgs::RenameRobotStateInWarehouse::Request & request,
                       moveit_msgs::RenameRobotStateInWarehouse::Response & response)>
      rename_cb = boost::bind(&renameState, _1, _2, &rs);

  boost::function<bool(moveit_msgs::DeleteRobotStateFromWarehouse::Request & request,
                       moveit_msgs::DeleteRobotStateFromWarehouse::Response & response)>
      delete_cb = boost::bind(&deleteState, _1, _2, &rs);

  ros::ServiceServer save_state_server = node.advertiseService("save_robot_state", save_cb);
  ros::ServiceServer list_states_server = node.advertiseService("list_robot_states", list_cb);
  ros::ServiceServer get_state_server = node.advertiseService("get_robot_state", get_cb);
  ros::ServiceServer has_state_server = node.advertiseService("has_robot_state", has_cb);
  ros::ServiceServer rename_state_server = node.advertiseService("rename_robot_state", rename_cb);
  ros::ServiceServer delete_state_server = node.advertiseService("delete_robot_state", delete_cb);

  ros::waitForShutdown();
  return 0;
}
