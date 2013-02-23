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

/* Author: Jon Binney, Ioan Sucan */

#include <ros/ros.h>
#include <moveit_msgs/SaveMap.h>
#include <moveit_msgs/LoadMap.h>
#include <moveit/occupancy_map_monitor/occupancy_map.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <moveit/occupancy_map_monitor/point_cloud_occupancy_map_updater.h>

namespace occupancy_map_monitor
{

OccupancyMapMonitor::OccupancyMapMonitor(double map_resolution) :
  nh_("~"),
  map_resolution_(map_resolution)
{
  initialize();
}

OccupancyMapMonitor::OccupancyMapMonitor(const boost::shared_ptr<tf::Transformer> &tf, const std::string &map_frame, double map_resolution) :
  nh_("~"),
  tf_(tf),
  map_frame_(map_frame),
  map_resolution_(map_resolution)
{ 
  initialize();
}

void OccupancyMapMonitor::initialize()
{ 
  /* load params from param server */
  if (map_resolution_ <= std::numeric_limits<double>::epsilon())
    if (!nh_.getParam("octomap_resolution", map_resolution_))
    {
      map_resolution_ = 0.1;
      ROS_WARN("Resolution not specified for Octomap. Assuming resolution = %g instead", map_resolution_);
    }
  ROS_DEBUG("Using resolution = %lf m for building octomap", map_resolution_);
  
  if (map_frame_.empty())
    if (!nh_.getParam("octomap_frame", map_frame_))
      if (tf_)
        ROS_WARN("No target frame specified for Octomap. No transforms will be applied to received data.");
  
  if (!tf_ && !map_frame_.empty())
    ROS_WARN("Target frame specified but no TF instance specified. No transforms will be applied to received data.");
  
  tree_.reset(new OccMapTree(map_resolution_));
  tree_const_ = tree_;
  
  XmlRpc::XmlRpcValue sensor_list;
  if (nh_.getParam("sensors", sensor_list))
  {
    if (sensor_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      for (int32_t i = 0; i < sensor_list.size(); ++i)
      {
        if (!sensor_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          ROS_ERROR("Params for sensor %d not a struct, ignoring sensor", i);
          continue;
        }
        
        if (!sensor_list[i].hasMember ("sensor_type"))
        {
          ROS_ERROR("No sensor type for sensor %d; ignoring.", i);
          continue;
        }
        
        std::string sensor_type = std::string(sensor_list[i]["sensor_type"]);
        OccupancyMapUpdaterPtr up;
        if (sensor_type == "point_cloud_sensor")
        {
          up.reset(new PointCloudOccupancyMapUpdater(this));
        }
        else
        {
          ROS_ERROR("Sensor %d has unknown type %s; ignoring.", i, sensor_type.c_str());
          continue;
        }
        
        /* pass the params struct directly in to the updater */
        if (!up->setParams(sensor_list[i]))
        {
          ROS_ERROR("Failed to load sensor %d of type %s", i, sensor_type.c_str());
          continue;
        }
        
        if (up->initialize())
          map_updaters_.push_back(up);
      }
    else
      ROS_ERROR("List of sensors must be an array!");
  }
  
  /* advertise a service for loading octomaps from disk */
  save_map_srv_ = nh_.advertiseService("save_map", &OccupancyMapMonitor::saveMapCallback, this);
  load_map_srv_ = nh_.advertiseService("load_map", &OccupancyMapMonitor::loadMapCallback, this);
}

bool OccupancyMapMonitor::saveMapCallback(moveit_msgs::SaveMap::Request& request, moveit_msgs::SaveMap::Response& response)
{
  ROS_INFO("Writing map to %s", request.filename.c_str());
  tree_->lockRead();
  try
  {
    response.success = tree_->writeBinary(request.filename);
  }
  catch (...)
  {
    response.success = false;
  }
  tree_->unlockRead();
  return true;
}

bool OccupancyMapMonitor::loadMapCallback(moveit_msgs::LoadMap::Request& request, moveit_msgs::LoadMap::Response& response)
{
  ROS_INFO("Reading map from %s", request.filename.c_str());
  
  /* load the octree from disk */
  tree_->lockWrite();
  try
  {
    response.success = tree_->readBinary(request.filename);
  }
  catch (...)
  { 
    ROS_ERROR("Failed to load map from file");
    response.success = false;
  }
  tree_->unlockWrite();
  
  return true;
}

void OccupancyMapMonitor::startMonitor()
{
  /* initialize all of the occupancy map updaters */
  for (std::size_t i = 0 ; i < map_updaters_.size() ; ++i)
    map_updaters_[i]->start();
}

void OccupancyMapMonitor::stopMonitor()
{  
  for (std::size_t i = 0 ; i < map_updaters_.size() ; ++i)
    map_updaters_[i]->stop();
}

OccupancyMapMonitor::~OccupancyMapMonitor()
{
  stopMonitor();
}

}
