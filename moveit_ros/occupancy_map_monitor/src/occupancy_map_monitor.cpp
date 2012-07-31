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

namespace occupancy_map_monitor
{


  OccupancyMapMonitor::OccupancyMapMonitor(const boost::shared_ptr<tf::Transformer> &tf) :
    nh_("~")
  {
    double map_resolution;

    /* load params from param server */
    nh_.param<std::string>("map_frame", map_frame_, "base_link"); /// \todo CAN't USE pr2 stuff
    nh_.param<double>("map_resolution", map_resolution, 0.1);

    tree_.reset(new octomap::OcTree(map_resolution));

    XmlRpc::XmlRpcValue sensor_list;
    if (nh_.getParam("sensors", sensor_list))
    {
      ROS_ASSERT(sensor_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int32_t i = 0; i < sensor_list.size(); ++i)
      {
        ROS_ASSERT(sensor_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        ROS_ASSERT(sensor_list[i].hasMember ("sensor_type"));
        std::string sensor_type = std::string (sensor_list[i]["sensor_type"]);
        
        if(sensor_type == "point_cloud_sensor")
        {
          ROS_ASSERT(sensor_list[i].hasMember("point_cloud_topic"));
          std::string point_cloud_topic = std::string (sensor_list[i]["point_cloud_topic"]);
          
          ROS_ASSERT(sensor_list[i].hasMember("max_range"));
          double max_range = double (sensor_list[i]["max_range"]);
          
          ROS_ASSERT(sensor_list[i].hasMember("frame_subsample"));
          size_t frame_subsample = int (sensor_list[i]["frame_subsample"]);
          
          ROS_ASSERT(sensor_list[i].hasMember("point_subsample"));
          size_t point_subsample = int (sensor_list[i]["point_subsample"]);
          
          boost::shared_ptr<OccupancyMapUpdater> up(new PointCloudOccupancyMapUpdater(tf, map_frame_, point_cloud_topic, max_range, frame_subsample, point_subsample));
          up->setNotifyFunction(boost::bind(&OccupancyMapMonitor::updateReady, this, _1));
          map_updaters_.push_back(up);
        }
        else
        {
          ROS_ERROR_STREAM("Ignoring unknown sensor type" << sensor_type << "in occupancy map params");
        }
      }
    }
    
    occupied_marker_pub_ = root_nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells", 1);
    free_marker_pub_ = root_nh_.advertise<visualization_msgs::MarkerArray>("free_cells", 1);
  }

  void OccupancyMapMonitor::treeUpdateThread(void)
  {
    std::set<OccupancyMapUpdater*> ready;
    while (tree_update_thread_)
    {
      {
        boost::mutex::scoped_lock update_lock(update_mut_);
        if (update_cond_.timed_wait(update_lock, boost::posix_time::milliseconds(100)))
          updates_available_.swap(ready);        
      }

      for (std::set<OccupancyMapUpdater*>::iterator it = ready.begin() ; it != ready.end() ; ++it)
      {
        ROS_DEBUG("Calling updater");
        boost::lock_guard<boost::mutex> _lock(tree_mutex_);
        (*it)->process(tree_);
      }
      ready.clear();
      publish_markers();
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

  OccMapTreePtr OccupancyMapMonitor::getTreePtr()
  {
    return tree_;
  }

  void OccupancyMapMonitor::lockTree()
  {
    tree_mutex_.lock();
  }

  void OccupancyMapMonitor::unlockTree()
  {
    tree_mutex_.unlock();
  }

  void OccupancyMapMonitor::publish_markers(void)
  {
    boost::lock_guard<boost::mutex> _lock(tree_mutex_);

    visualization_msgs::MarkerArray occupied_nodes_arr, free_nodes_arr;

    make_occupied_cells_marker_array(tree_, ros::Time::now(), map_frame_, "occupied_cells", occupied_nodes_arr);
    //make_free_cells_marker_array(tree_, ros::Time::now(), map_frame_, "free_cells", free_nodes_arr);

    occupied_marker_pub_.publish(occupied_nodes_arr);
    //free_marker_pub_.publish(free_nodes_arr);
  }

  void OccupancyMapMonitor::startMonitor()
  {
    if (!tree_update_thread_)
    {
      /* initialize all of the occupancy map updaters */
      std::vector<boost::shared_ptr<OccupancyMapUpdater> >::iterator it;
      for (it = map_updaters_.begin(); it != map_updaters_.end(); it++)
        (*it)->initialize();
      
      /* start a dedicated thread for updating the occupancy map */
      tree_update_thread_.reset(new boost::thread(&OccupancyMapMonitor::treeUpdateThread, this));
    }
  }

void OccupancyMapMonitor::stopMonitor()
{ 
  if (tree_update_thread_)
  {
    boost::thread *copy = tree_update_thread_.get();
    tree_update_thread_.reset();
    copy->join();
  }
}

  OccupancyMapMonitor::~OccupancyMapMonitor(void)
  {
    stopMonitor();
  }
}
