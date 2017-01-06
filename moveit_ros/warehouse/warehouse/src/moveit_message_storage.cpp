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

/* Author: Ioan Sucan */

#include <moveit/warehouse/moveit_message_storage.h>
#include <boost/regex.hpp>
#include <ros/ros.h>

moveit_warehouse::MoveItMessageStorage::MoveItMessageStorage(const std::string& host, const unsigned int port,
                                                             double wait_seconds)
  : db_host_(host), db_port_(port), timeout_(wait_seconds)
{
  // if we are using default values for initialization, attempt to use ROS params for initialization
  if (db_host_.empty() || db_port_ == 0)
  {
    ros::NodeHandle nh("~");
    // search for the warehouse_port parameter in the local name space of the node, and up the tree of namespaces;
    // if the desired param is not found, make a final attempt to look fro the param in the default namespace defined
    // above
    if (db_port_ == 0)
    {
      std::string paramName;
      if (!nh.searchParam("warehouse_port", paramName))
        paramName = "warehouse_port";
      int param_port;
      if (nh.getParam(paramName, param_port))
        db_port_ = param_port;
    }
    if (db_host_.empty())
    {
      std::string paramName;
      if (!nh.searchParam("warehouse_host", paramName))
        paramName = "warehouse_host";
      std::string param_host;
      if (nh.getParam(paramName, param_host))
        db_host_ = param_host;
    }
  }
  ROS_DEBUG("Connecting to MongoDB on host '%s' port '%u'...", db_host_.c_str(), db_port_);
}

moveit_warehouse::MoveItMessageStorage::~MoveItMessageStorage()
{
}

void moveit_warehouse::MoveItMessageStorage::drop(const std::string& db)
{
  mongo_ros::dropDatabase(db, db_host_, db_port_, timeout_);
  ROS_DEBUG("Dropped database '%s'", db.c_str());
}

void moveit_warehouse::MoveItMessageStorage::filterNames(const std::string& regex,
                                                         std::vector<std::string>& names) const
{
  if (!regex.empty())
  {
    std::vector<std::string> fnames;
    boost::regex r(regex);
    for (std::size_t i = 0; i < names.size(); ++i)
    {
      boost::cmatch match;
      if (boost::regex_match(names[i].c_str(), match, r))
        fnames.push_back(names[i]);
    }
    names.swap(fnames);
  }
}
