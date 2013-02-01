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

#ifndef MOVEIT_POINTCLOUD_OCCUPANCY_MAP_UPDATER_H_
#define MOVEIT_POINTCLOUD_OCCUPANCY_MAP_UPDATER_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit/robot_self_filter/self_mask.h>
#include <moveit/occupancy_map_monitor/occupancy_map.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <boost/thread.hpp>

namespace occupancy_map_monitor
{
  class PointCloudOccupancyMapUpdater : public OccupancyMapUpdater
  {
  public:
    PointCloudOccupancyMapUpdater(const boost::shared_ptr<tf::Transformer> &tf, const std::string &map_frame);
    virtual ~PointCloudOccupancyMapUpdater();
      
    virtual bool setParams(XmlRpc::XmlRpcValue &params);
    virtual bool setParams(const std::string &point_cloud_topic, double max_range,  size_t frame_subsample,
                           size_t point_subsample, const std::vector<robot_self_filter::LinkInfo> see_links);
    virtual void initialize();
    virtual void process(const OccMapTreePtr &tree);
      
  private:

    virtual void cloudMsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
    virtual void processCloud(const OccMapTreePtr &tree, const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
    
    ros::NodeHandle root_nh_;
      
    boost::shared_ptr<tf::Transformer> tf_;
      
    /* params */
    std::string map_frame_;
    std::string point_cloud_topic_;
    double max_range_;
    size_t frame_subsample_;
    size_t point_subsample_;
      
    message_filters::Subscriber<sensor_msgs::PointCloud2> *point_cloud_subscriber_;
    tf::MessageFilter<sensor_msgs::PointCloud2> *point_cloud_filter_;
    sensor_msgs::PointCloud2::ConstPtr last_point_cloud_;
    boost::mutex last_point_cloud_mutex_;
      
    /* used to store all cells in the map which a given ray passes through during raycasting.
       we cache this here because it dynamically pre-allocates a lot of memory in its contsructor */
    octomap::KeyRay key_ray_;

    boost::shared_ptr<robot_self_filter::SelfMask> self_mask_;
  };
}

#endif /* MOVEIT_POINTCLOUD_OCCUPANCY_MAP_UPDATER_H_ */
