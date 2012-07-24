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

#include <occupancy_map_server/point_cloud_occupancy_map_updater.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

namespace occupancy_map_server
{
	PointCloudOccupancyMapUpdater::PointCloudOccupancyMapUpdater(const std::string &map_frame, const std::string &point_cloud_topic, boost::shared_ptr<tf::Transformer> tf) :
			map_frame_(map_frame), point_cloud_topic_(point_cloud_topic), tf_(tf)
		{}

	PointCloudOccupancyMapUpdater::~PointCloudOccupancyMapUpdater(void)
	{
		delete point_cloud_subscriber_;
		delete point_cloud_filter_;
	}

	void PointCloudOccupancyMapUpdater::initialize()
	{
		// subscribe to point cloud topic using tf filter
		point_cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(root_nh_, point_cloud_topic_, 1024);
		point_cloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(*point_cloud_subscriber_, *tf_, map_frame_, 1024);
		point_cloud_filter_->registerCallback(boost::bind(&PointCloudOccupancyMapUpdater::cloudMsgCallback, this, _1));

		ROS_INFO("Listening to '%s' using message notifier with target frame '%s'", point_cloud_topic_.c_str(), point_cloud_filter_->getTargetFramesString().c_str());

	}

	void PointCloudOccupancyMapUpdater::cloudMsgCallback(const sensor_msgs::PointCloud2::ConstPtr cloud_msg)
	{
		if(this->isActive())
			ROS_DEBUG("Got a point cloud message, updating occupancy map");
		else
			ROS_DEBUG("Updater not active; ignoring pointcloud message");

		if(!this->beginUpdateTree())
		{
			ROS_ERROR("Updater activated but no octree to update");
			return;
		}

		this->endUpdateTree();

		//TODO: update the octree using the point cloud message
	}
}
