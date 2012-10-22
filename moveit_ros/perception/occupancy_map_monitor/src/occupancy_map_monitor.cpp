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

/* Author: Jon Binney */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <occupancy_map_monitor/occupancy_map.h>
#include <occupancy_map_monitor/occupancy_map_monitor.h>
#include <occupancy_map_monitor/point_cloud_occupancy_map_updater.h>
#include <occupancy_map_monitor/octomap_markers.h>

#include <octomap_msgs/conversions.h>

namespace occupancy_map_monitor
{

OccupancyMapMonitor::OccupancyMapMonitor(const Options &opt, const boost::shared_ptr<tf::Transformer> &tf) : nh_("~")
{
  initialize(opt, tf);
}

OccupancyMapMonitor::OccupancyMapMonitor(const boost::shared_ptr<tf::Transformer> &tf) : nh_("~")
{ 
  Options opt; // empty set of options; this will lead to having everything read from the param server
  initialize(opt, tf);
}

void OccupancyMapMonitor::initialize(const Options &input_opt, const boost::shared_ptr<tf::Transformer> &tf)
{ 
  tree_update_thread_running_ = false;
  opt_ = input_opt; // we need to be able to update options
  
  /* load params from param server */
  if (opt_.map_resolution <= 0.0)
    if (!nh_.getParam("octomap_resolution", opt_.map_resolution))
    {
      opt_.map_resolution = 0.1;
      ROS_WARN("Resolution not specified for Octomap.");
    }
  ROS_DEBUG("Using resolution = %lf m for building octomap", opt_.map_resolution);
  
  if (opt_.map_frame.empty())
    if (!nh_.getParam("octomap_frame", opt_.map_frame))
      ROS_WARN("No target frame specified for Octomap. No transforms will be applied to received data.");
  
  tree_.reset(new octomap::OcTree(opt_.map_resolution));
  tree_const_ = tree_;
  
  XmlRpc::XmlRpcValue sensor_list;
  if (nh_.getParam("sensors", sensor_list))
  {
    if(!sensor_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("List of sensors must be an array!");
      return;
    }
    for (int32_t i = 0; i < sensor_list.size(); ++i)
    {
      if(!sensor_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR("Params for sensor %d not a struct, ignoring sensor", i);
        continue;
      }

      if(!sensor_list[i].hasMember ("sensor_type"))
      {
        ROS_ERROR("No sensor type for sensor %d, ignoring sensor", i);
        continue;
      }

      std::string sensor_type = std::string (sensor_list[i]["sensor_type"]);
      boost::shared_ptr<OccupancyMapUpdater> up;
      if(sensor_type == "point_cloud_sensor")
      {
        up.reset(new PointCloudOccupancyMapUpdater(tf, opt_.map_frame));
      }
      else
      {
        ROS_ERROR("Sensor %d has unknown type %s, ignoring sensor", i, sensor_type.c_str());
        continue;
      }

      /* pass the params struct directly in to the updater */
      if(!up->setParams(sensor_list[i]))
      {
        ROS_ERROR("Failed to load sensor %d of type %s", i, sensor_type.c_str());
        continue;
      }

      up->setNotifyFunction(boost::bind(&OccupancyMapMonitor::updateReady, this, _1));
      map_updaters_.push_back(up);
    }
  }
  occupied_marker_pub_ = root_nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells", 1);
  free_marker_pub_ = root_nh_.advertise<visualization_msgs::MarkerArray>("free_cells", 1);
  octree_binary_pub_ = root_nh_.advertise<octomap_msgs::OctomapBinary>("octomap_binary", 1);
}

void OccupancyMapMonitor::treeUpdateThread(void)
{
  std::set<OccupancyMapUpdater*> ready;
  while (tree_update_thread_running_)
  {
    {
      boost::mutex::scoped_lock update_lock(update_mut_);
      if (update_cond_.timed_wait(update_lock, boost::posix_time::milliseconds(100)))
        updates_available_.swap(ready);
    }
    if (tree_update_thread_running_ && !ready.empty())
    {
      ROS_DEBUG("Calling updaters");
      {
        boost::lock_guard<boost::mutex> _lock(tree_mutex_);
        for (std::set<OccupancyMapUpdater*>::iterator it = ready.begin() ; it != ready.end() ; ++it)
          (*it)->process(tree_);
      }
      if (update_callback_)
        update_callback_();
      ready.clear();

      publish_markers();
      publish_octomap_binary();
    }
  }
}

void OccupancyMapMonitor::updateReady(OccupancyMapUpdater *updater)
{ 
  {
    boost::mutex::scoped_lock update_lock(update_mut_);
    updates_available_.insert(updater);
  }
  update_cond_.notify_all();
}

void OccupancyMapMonitor::lockOcTree(void)
{
  tree_mutex_.lock();
}

void OccupancyMapMonitor::unlockOcTree(void)
{
  tree_mutex_.unlock();
}

void OccupancyMapMonitor::publish_markers(void)
{
  boost::lock_guard<boost::mutex> _lock(tree_mutex_);
  
  visualization_msgs::MarkerArray occupied_nodes_arr, free_nodes_arr;
  
  make_occupied_cells_marker_array(tree_, ros::Time::now(), opt_.map_frame, "occupied_cells", occupied_nodes_arr);
  //make_free_cells_marker_array(tree_, ros::Time::now(), map_frame_, "free_cells", free_nodes_arr);
  
  occupied_marker_pub_.publish(occupied_nodes_arr);
  //free_marker_pub_.publish(free_nodes_arr);
}

void OccupancyMapMonitor::publish_octomap_binary(void)
{
  octomap_msgs::OctomapBinary map;

  map.header.frame_id = opt_.map_frame;
  map.header.stamp = ros::Time::now();

  if (octomap_msgs::binaryMapToMsgData(*tree_, map.data))
  {
    octree_binary_pub_.publish(map);
  }
  else
  {
    ROS_ERROR("Could not generate OctoMap message");
  }
}

void OccupancyMapMonitor::startMonitor(void)
{
  if (!tree_update_thread_running_)
  {
    /* initialize all of the occupancy map updaters */
    std::vector<boost::shared_ptr<OccupancyMapUpdater> >::iterator it;
    for (it = map_updaters_.begin(); it != map_updaters_.end(); it++)
      (*it)->initialize();
    
    /* start a dedicated thread for updating the occupancy map */
    tree_update_thread_running_ = true;
    tree_update_thread_.reset(new boost::thread(&OccupancyMapMonitor::treeUpdateThread, this));
  }
}

void OccupancyMapMonitor::stopMonitor(void)
{ 
  if (tree_update_thread_running_)
  {
    tree_update_thread_running_ = false;
    tree_update_thread_->join();
    tree_update_thread_.reset();
  }
}

OccupancyMapMonitor::~OccupancyMapMonitor(void)
{
  stopMonitor();
}

}
