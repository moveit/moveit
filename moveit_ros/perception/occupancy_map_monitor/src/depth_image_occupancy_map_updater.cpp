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

#include <moveit/occupancy_map_monitor/depth_image_occupancy_map_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>


namespace occupancy_map_monitor
{

DepthImageOccupancyMapUpdater::DepthImageOccupancyMapUpdater(OccupancyMapMonitor *monitor) :
  OccupancyMapUpdater(monitor, "DepthImageUpdater"),
  nh_("~"),
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
  // call the mesh filter

}


}
