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

#ifndef MOVEIT_OCCUPANCY_MAP_UPDATER_H_
#define MOVEIT_OCCUPANCY_MAP_UPDATER_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <ros/ros.h>
#include <moveit/occupancy_map_monitor/occupancy_map.h>

namespace occupancy_map_monitor
{
	/**
	 * @class OccupancyMapUpdater
   * Base class for classes which update the occupancy map.
   */
class OccupancyMapUpdater
{
public:
  /** @brief Constructor */
  OccupancyMapUpdater() {}
  virtual ~OccupancyMapUpdater() {}

  /** @brief Server calls this
       *  @param notify_func Function which updater should call when ready to update the map
       */
  void setNotifyFunction(const boost::function<void(OccupancyMapUpdater*)> &notify_func) { notify_func_ = notify_func; }

  /** @brief Set updater params using struct that comes from parsing a yaml string*/
  virtual bool setParams(XmlRpc::XmlRpcValue &params) = 0;

  /** @brief Do any necessary setup (subscribe to ros topics, etc.)*/
  virtual void initialize() = 0;

  /** @brief Update the map
       *  @param tree Pointer to octree which represents the occupancy map
       */
  virtual void process(const OccMapTreePtr &tree) = 0;

protected:

  /** @brief Updater calls this to notify the server that it is ready to modify the map */
  void notifyUpdateReady()
  {
    if (notify_func_)
      notify_func_(this);
  }
  
private:
  boost::function<void(OccupancyMapUpdater*)> notify_func_;
};
}

#endif /* MOVEIT_OCCUPANCY_MAP_UPDATER_H_ */
