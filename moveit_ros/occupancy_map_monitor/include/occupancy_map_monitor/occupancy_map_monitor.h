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

#ifndef MOVEIT_OCCUPANCY_MAP_MONITOR_H_
#define MOVEIT_OCCUPANCY_MAP_MONITOR_H_

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include <occupancy_map_monitor/occupancy_map.h>
#include <occupancy_map_monitor/occupancy_map_updater.h>


namespace occupancy_map_monitor
{

  class OccupancyMapMonitor
  {
  public:
    OccupancyMapMonitor(const boost::shared_ptr<tf::Transformer> tf);
    ~OccupancyMapMonitor(void);

    /** @brief run the monitor. does not return (gives control to main ROS loop) */
    void run(void);

    /** @brief get a pointer to the underlying octree for this monitor. lock the
     *  tree before reading or writing using this pointer */
    OccMapTreePtr getTreePtr(void);

    /** @brief lock the underlying octree. it will not be read or written by the
     *  monitor until unlockTree() is called */
    void lockTree(void);
    void unlockTree(void);


  private:
    void treeUpdateThread(void);
    void publish_markers(void);

    OccMapTreePtr tree_;
    boost::mutex tree_mutex_;
    boost::thread tree_update_thread_;

    std::string map_frame_;

    std::vector<boost::shared_ptr<OccupancyMapUpdater> > map_updaters_;

    ros::NodeHandle root_nh_;
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
  };

}

#endif /* MOVEIT_OCCUPANCY_MAP_MONITOR_H_ */
