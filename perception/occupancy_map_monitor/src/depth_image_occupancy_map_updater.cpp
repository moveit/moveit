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

/* Author: Ioan Sucan, Suat Gedikli, Jon Binney */

#include <moveit/occupancy_map_monitor/depth_image_occupancy_map_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>


namespace occupancy_map_monitor
{

DepthImageOccupancyMapUpdater::DepthImageOccupancyMapUpdater(OccupancyMapMonitor *monitor) :
  OccupancyMapUpdater(monitor, "DepthImageUpdater"),
  nh_("~"),
  tf_(monitor->getTFClient()),
  input_depth_transport_(nh_),
  queue_size_(5),
  near_clipping_plane_distance_(0.4),
  far_clipping_plane_distance_(5.0),
  shadow_threshold_(0.3),
  padding_coefficient_0_(0.0035),
  padding_coefficient_1_(0.0),
  padding_coefficient_2_(0.001)
{ 
}

DepthImageOccupancyMapUpdater::~DepthImageOccupancyMapUpdater()
{
  stopHelper();
}

bool DepthImageOccupancyMapUpdater::setParams(XmlRpc::XmlRpcValue &params)
{
  if (params.hasMember("queue_size"))
    queue_size_ = (int)params["queue_size"];
  if (params.hasMember("near_clipping_plane_distance"))
    near_clipping_plane_distance_ = (double) params["near_clipping_plane_distance"];
  if (params.hasMember("far_clipping_plane_distance"))
    far_clipping_plane_distance_ = (double) params["far_clipping_plane_distance"];
  if (params.hasMember("shadow_threshold"))
    shadow_threshold_ = (double) params["shadow_threshold"];
  if (params.hasMember("padding_coefficient_1"))
    padding_coefficient_0_ = (double) params["padding_coefficient_1"];
  if (params.hasMember("padding_coefficient_2"))
    padding_coefficient_1_ = (double) params["padding_coefficient_2"];
  if (params.hasMember("padding_coefficient_3"))
    padding_coefficient_2_ = (double) params["padding_coefficient_3"];
}

bool DepthImageOccupancyMapUpdater::initialize()
{
  return true;
}

void DepthImageOccupancyMapUpdater::start()
{
  image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
  sub_depth_image_ = input_depth_transport_.subscribeCamera("depth", queue_size_, &DepthImageOccupancyMapUpdater::depthImageCallback, this, hints);
}

void DepthImageOccupancyMapUpdater::stop()
{ 
  stopHelper();
}

void DepthImageOccupancyMapUpdater::stopHelper()
{ 
  sub_depth_image_.shutdown();
}

void DepthImageOccupancyMapUpdater::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  
  ROS_DEBUG("Received a new point cloud message");
  
  if (monitor_->getMapFrame().empty())
    monitor_->setMapFrame(depth_msg->header.frame_id);
  
  /* get transform for cloud into map frame */
  tf::StampedTransform map_H_sensor;
  if (monitor_->getMapFrame() == depth_msg->header.frame_id)
    map_H_sensor.setIdentity();
  else
  {
    if (tf_)
    {
      try
      {
        tf_->lookupTransform(monitor_->getMapFrame(), depth_msg->header.frame_id, depth_msg->header.stamp, map_H_sensor);
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

  

  // call the mesh filter




  // Use correct principal point from calibration
  double px = info_msg->K[2];
  double py = info_msg->K[5];
  
  double inv_fx = 1.0 / info_msg->K[0];
  double inv_fy = 1.0 / info_msg->K[4];
  
  const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
  
  // Pre-compute some constants
  if (x_cache_.size() < depth_msg->width)
    x_cache_.resize(depth_msg->width);
  if (y_cache_.size() < depth_msg->height)
    y_cache_.resize(depth_msg->height);
  
  for (int x = 0; x < depth_msg->width; ++x)
    x_cache_[x] = (x - px) * inv_fx;
  
  for (int y = 0; y < depth_msg->height; ++y)
    y_cache_[y] = (y - py) * inv_fy;

  tf::Vector3 sensor_origin_tf = map_H_sensor.getOrigin();
  octomap::point3d sensor_origin(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());
  
  OccMapTreePtr tree = monitor_->getOcTreePtr();
  octomap::KeySet free_cells, occupied_cells;

  tree->lockRead();
  
  try
  {
    const float* depth_row = reinterpret_cast<const float*>(depth_msg->data[0]);
    for (int y = 0; y < depth_msg->height ; ++y, depth_row += depth_msg->width)
      if (y_cache_[y] == y_cache_[y]) // if not NaN
        for (int x = 0; x < depth_msg->width; ++x)
        {
          float zz = depth_row[x];
          if (zz == zz) // check for NaN
          {
            float yy = y_cache_[y] * zz;
            float xx = x_cache_[x] * zz;
            if (xx == xx)
            {
              /* transform to map frame */
              tf::Vector3 point_tf = map_H_sensor * tf::Vector3(xx, yy, zz);
              octomap::point3d point(point_tf.getX(), point_tf.getY(), point_tf.getZ());
              
              /* free cells along ray */
              if (tree->computeRayKeys(sensor_origin, point, key_ray_))
                free_cells.insert(key_ray_.begin(), key_ray_.end());
              
              
              /* occupied cell at ray endpoint if ray is shorter than max range and this point
                 isn't on a part of the robot*/
              octomap::OcTreeKey key;
              if (tree->coordToKeyChecked(point, key))
                occupied_cells.insert(key);
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
}


}
