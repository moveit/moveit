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

#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/profiler/profiler.h>
#include <ros/ros.h>

rdf_loader::RDFLoader::RDFLoader(const std::string& robot_description)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RDFLoader(robot_description)");

  ros::WallTime start = ros::WallTime::now();
  ros::NodeHandle nh("~");
  std::string content;

  if (!nh.searchParam(robot_description, robot_description_) || !nh.getParam(robot_description_, content))
  {
    ROS_ERROR("Robot model parameter not found! Did you remap '%s'?", robot_description.c_str());
    return;
  }

  urdf::Model* umodel = new urdf::Model();
  if (!umodel->initString(content))
  {
    ROS_ERROR("Unable to parse URDF from parameter '%s'", robot_description_.c_str());
    return;
  }
  urdf_.reset(umodel);

  const std::string srdf_description(robot_description_ + "_semantic");
  std::string scontent;
  if (!nh.getParam(srdf_description, scontent))
  {
    ROS_ERROR("Robot semantic description not found. Did you forget to define or remap '%s'?",
              srdf_description.c_str());
    return;
  }

  srdf_.reset(new srdf::Model());
  if (!srdf_->initString(*urdf_, scontent))
  {
    ROS_ERROR("Unable to parse SRDF from parameter '%s'", srdf_description.c_str());
    srdf_.reset();
    return;
  }

  ROS_DEBUG_STREAM_NAMED("rdf", "Loaded robot model in " << (ros::WallTime::now() - start).toSec() << " seconds");
}

rdf_loader::RDFLoader::RDFLoader(const std::string& urdf_string, const std::string& srdf_string)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RDFLoader(string)");

  urdf::Model* umodel = new urdf::Model();
  urdf_.reset(umodel);
  if (umodel->initString(urdf_string))
  {
    srdf_.reset(new srdf::Model());
    if (!srdf_->initString(*urdf_, srdf_string))
    {
      ROS_ERROR("Unable to parse SRDF");
      srdf_.reset();
    }
  }
  else
  {
    ROS_ERROR("Unable to parse URDF");
    urdf_.reset();
  }
}

rdf_loader::RDFLoader::RDFLoader(TiXmlDocument* urdf_doc, TiXmlDocument* srdf_doc)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RDFLoader(XML)");

  urdf::Model* umodel = new urdf::Model();
  urdf_.reset(umodel);
  if (umodel->initXml(urdf_doc))
  {
    srdf_.reset(new srdf::Model());
    if (!srdf_->initXml(*urdf_, srdf_doc))
    {
      ROS_ERROR("Unable to parse SRDF");
      srdf_.reset();
    }
  }
  else
  {
    ROS_ERROR("Unable to parse URDF");
    urdf_.reset();
  }
}
