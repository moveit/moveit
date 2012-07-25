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
#include <occupancy_map_server/occupancy_map_server.h>
#include <occupancy_map_server/point_cloud_occupancy_map_updater.h>

namespace occupancy_map_server
{

  OccupancyMapServer::OccupancyMapServer(const boost::shared_ptr<tf::Transformer> tf) :
      tree_(0.1), map_frame_("base_link")
  {
    /* This will eventually load different updaters depending on config params */
    boost::shared_ptr<OccupancyMapUpdater> up = boost::make_shared<PointCloudOccupancyMapUpdater>(map_frame_,
        "/camera/rgb/points", 30.0, tf);
    map_updaters_.push_back(up);

    marker_pub_ = root_nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells", 1);
  }

  void OccupancyMapServer::treeUpdateThread(void)
  {
    ros::Rate r(100);
    while (ros::ok())
    {
      /* run any occupancy map updater which is ready */
      std::vector<boost::shared_ptr<OccupancyMapUpdater> >::iterator it;
      for (it = map_updaters_.begin(); it != map_updaters_.end(); it++)
        (*it)->process(&tree_);

      publish_markers();

      r.sleep();
    }
  }

  void OccupancyMapServer::publish_markers(void)
  {
    visualization_msgs::MarkerArray occupied_nodes_arr;
    int tree_depth = tree_.getTreeDepth();

    /* each array stores all cubes of a different size, one for each depth level */
    occupied_nodes_arr.markers.resize(tree_depth + 1);

    /* now, traverse all leaves in the tree */
    for (octomap::OcTree::iterator it = tree_.begin(tree_depth), end = tree_.end(); it != end; ++it)
    {
      if (tree_.isNodeOccupied(*it))
      {
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();

        unsigned idx = it.getDepth();
        assert(idx < occupied_nodes_arr.markers.size());

        geometry_msgs::Point cube_center;
        cube_center.x = x;
        cube_center.y = y;
        cube_center.z = z;

        occupied_nodes_arr.markers[idx].points.push_back(cube_center);
      }
    }

    /* finish MarkerArray */
    for (unsigned i = 0; i < occupied_nodes_arr.markers.size(); ++i)
    {
      double size = tree_.getNodeSize(i);

      occupied_nodes_arr.markers[i].header.frame_id = map_frame_;
      occupied_nodes_arr.markers[i].header.stamp = ros::Time::now();
      occupied_nodes_arr.markers[i].ns = "occupancy_map";
      occupied_nodes_arr.markers[i].id = i;
      occupied_nodes_arr.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupied_nodes_arr.markers[i].scale.x = size;
      occupied_nodes_arr.markers[i].scale.y = size;
      occupied_nodes_arr.markers[i].scale.z = size;
      occupied_nodes_arr.markers[i].color.r = 0.0;
      occupied_nodes_arr.markers[i].color.g = 0.0;
      occupied_nodes_arr.markers[i].color.b = 1.0;
      occupied_nodes_arr.markers[i].color.a = 0.5;

      if (occupied_nodes_arr.markers[i].points.size() > 0)
        occupied_nodes_arr.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupied_nodes_arr.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    marker_pub_.publish(occupied_nodes_arr);
  }

  void OccupancyMapServer::run()
  {
    /* initialize all of the occupancy map updaters */
    std::vector<boost::shared_ptr<OccupancyMapUpdater> >::iterator it;
    for (it = map_updaters_.begin(); it != map_updaters_.end(); it++)
      (*it)->initialize();

    /* start a dedicated thread for updating the occupancy map */
    tree_update_thread_ = boost::thread(&OccupancyMapServer::treeUpdateThread, this);

    /* give control to main ros loop */
    ros::spin();
  }

  OccupancyMapServer::~OccupancyMapServer(void)
  {
  }
}
