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

#include <moveit/occupancy_map_monitor/point_cloud_occupancy_map_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <message_filters/subscriber.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <moveit/robot_self_filter/self_mask.h>

namespace occupancy_map_monitor
{

PointCloudOccupancyMapUpdater::PointCloudOccupancyMapUpdater(OccupancyMapMonitor *monitor)
  : OccupancyMapUpdater(monitor, "PointCloudUpdater"),
    tf_(monitor->getTFClient()),
    point_cloud_subscriber_(NULL),
    point_cloud_filter_(NULL)
{
}

PointCloudOccupancyMapUpdater::~PointCloudOccupancyMapUpdater()
{
  stopHelper();
}

bool PointCloudOccupancyMapUpdater::setParams(XmlRpc::XmlRpcValue &params)
{
  if (!params.hasMember("point_cloud_topic"))
    return false;
  std::string point_cloud_topic = std::string (params["point_cloud_topic"]);

  if(!params.hasMember("max_range"))
    return false;
  double max_range = double (params["max_range"]);

  if(!params.hasMember("frame_subsample"))
    return false;
  size_t frame_subsample = int (params["frame_subsample"]);

  if(!params.hasMember("point_subsample"))
    return false;
  size_t point_subsample = int (params["point_subsample"]);

  std::vector<robot_self_filter::LinkInfo> links;
  if(params.hasMember("self_mask"))
  {
    ROS_INFO("Configuring self mask");
    if(!robot_self_filter::createLinksFromParams(params["self_mask"], links))
    {
      ROS_ERROR("Failed to load self mask");
      return false;
    }
  }

  return this->setParams(point_cloud_topic, max_range, frame_subsample, point_subsample, links);
}

bool PointCloudOccupancyMapUpdater::setParams(const std::string &point_cloud_topic, double max_range, size_t frame_subsample,
                                              size_t point_subsample, const std::vector<robot_self_filter::LinkInfo> &links)
{
  point_cloud_topic_ = point_cloud_topic;
  max_range_ = max_range;
  frame_subsample_ = frame_subsample;
  point_subsample_ = point_subsample;
  if (tf_)
    self_mask_.reset(new robot_self_filter::SelfMask(*tf_, links));
  return true;
}

bool PointCloudOccupancyMapUpdater::initialize()
{
  return true;
}

void PointCloudOccupancyMapUpdater::start()
{
  if (point_cloud_subscriber_)
    return;
  /* subscribe to point cloud topic using tf filter*/
  point_cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(root_nh_, point_cloud_topic_, 5);
  if (tf_ && !monitor_->getMapFrame().empty())
  {
    point_cloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(*point_cloud_subscriber_, *tf_, monitor_->getMapFrame(), 5);
    point_cloud_filter_->registerCallback(boost::bind(&PointCloudOccupancyMapUpdater::cloudMsgCallback, this, _1));
    ROS_INFO("Listening to '%s' using message filter with target frame '%s'", point_cloud_topic_.c_str(), point_cloud_filter_->getTargetFramesString().c_str());
  }
  else
  {
    point_cloud_subscriber_->registerCallback(boost::bind(&PointCloudOccupancyMapUpdater::cloudMsgCallback, this, _1));
    ROS_INFO("Listening to '%s'", point_cloud_topic_.c_str());
  }
}

void PointCloudOccupancyMapUpdater::stopHelper()
{ 
  delete point_cloud_filter_;
  delete point_cloud_subscriber_;
}

void PointCloudOccupancyMapUpdater::stop()
{ 
  stopHelper();
  point_cloud_filter_ = NULL;
  point_cloud_subscriber_ = NULL;
}

mesh_filter::MeshHandle PointCloudOccupancyMapUpdater::excludeShape(const shapes::ShapeConstPtr &shape)
{
  mesh_filter::MeshHandle h = 0;
  return h;
}

void PointCloudOccupancyMapUpdater::forgetShape(mesh_filter::MeshHandle handle)
{
}

void PointCloudOccupancyMapUpdater::cloudMsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
  ROS_DEBUG("Received a new point cloud message");
  ros::WallTime start = ros::WallTime::now();

  if (monitor_->getMapFrame().empty())
    monitor_->setMapFrame(cloud_msg->header.frame_id);
  
  /* get transform for cloud into map frame */
  tf::StampedTransform map_H_sensor;
  if (monitor_->getMapFrame() == cloud_msg->header.frame_id)
    map_H_sensor.setIdentity();
  else
  {
    if (tf_)
    {
      try
      {
        tf_->lookupTransform(monitor_->getMapFrame(), cloud_msg->header.frame_id, cloud_msg->header.stamp, map_H_sensor);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << "; quitting callback");
        return;
      }
    }
    else
      return;
  } 

  /* convert cloud message to pcl cloud object */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);

  /* compute sensor origin in map frame */
  tf::Vector3 sensor_origin_tf = map_H_sensor.getOrigin();
  octomap::point3d sensor_origin(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());

  /* mask out points on the robot */
  std::vector<int> mask;
  if (self_mask_)
    self_mask_->maskContainment(cloud, mask);

  OccMapTreePtr tree = monitor_->getOcTreePtr();
  octomap::KeySet free_cells, occupied_cells;

  tree->lockRead();
  
  try
  {
    /* do ray tracing to find which cells this point cloud indicates should be free, and which it indicates
     * should be occupied */
    unsigned int row, col;
    for (row = 0; row < cloud.height; row += point_subsample_)
    {
      for (col = 0; col < cloud.width; col += point_subsample_)
      {
        bool self_point = false;
        if (!mask.empty() && mask[row*cloud.width + col] == robot_self_filter::INSIDE)
          self_point = true;
        
        pcl::PointXYZ p = cloud(col, row);
        
        /* check for NaN */
        if (!((p.x == p.x) && (p.y == p.y) && (p.z == p.z)))
          continue;
        
        /* transform to map frame */
        tf::Vector3 point_tf = map_H_sensor * tf::Vector3(p.x, p.y, p.z);
        octomap::point3d point(point_tf.getX(), point_tf.getY(), point_tf.getZ());
        
        /* free cells along ray */
        if (tree->computeRayKeys(sensor_origin, point, key_ray_))
          free_cells.insert(key_ray_.begin(), key_ray_.end());
        
        /* occupied cell at ray endpoint if ray is shorter than max range and this point
           isn't on a part of the robot*/
        if(!self_point)
        {
          double range = (point - sensor_origin).norm();
          if(range < max_range_)
          {
            octomap::OcTreeKey key;
            if (tree->coordToKeyChecked(point, key))
              occupied_cells.insert(key);
          }
        }
      }
    }
  }
  catch (...)
  { 
    tree->unlockRead();
    return;
  }

  ROS_DEBUG("Marking free cells in octomap");
  
  tree->unlockRead();
  tree->lockWrite();

  try
  {
    
    /* mark free cells only if not seen occupied in this cloud */
    for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
      /* this check seems unnecessary since we would just overwrite them in the next loop? -jbinney */
      if (occupied_cells.find(*it) == occupied_cells.end())
        tree->updateNode(*it, false);
    
    ROS_DEBUG("Marking occupied cells in octomap");
    
    /* now mark all occupied cells */
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++)
      tree->updateNode(*it, true);
  }
  catch (...)
  {
  }
  tree->unlockWrite();
  ROS_INFO("Processed point cloud in %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0);
  triggerUpdateCallback();
}

}
