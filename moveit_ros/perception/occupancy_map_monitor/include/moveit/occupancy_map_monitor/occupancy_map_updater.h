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

#ifndef MOVEIT_OCCUPANCY_MAP_MONITOR_OCCUPANCY_MAP_UPDATER__
#define MOVEIT_OCCUPANCY_MAP_MONITOR_OCCUPANCY_MAP_UPDATER__

#include <moveit/occupancy_map_monitor/occupancy_map.h>
#include <moveit/mesh_filter/mesh_filter_base.h>
#include <geometric_shapes/shapes.h>
#include <boost/shared_ptr.hpp>

namespace occupancy_map_monitor
{

class OccupancyMapMonitor;

/** \brief Base class for classes which update the occupancy map.
 */
class OccupancyMapUpdater
{
public:

  OccupancyMapUpdater(OccupancyMapMonitor *monitor, const std::string &type) :
    monitor_(monitor)
  {
  }
  
  virtual ~OccupancyMapUpdater()
  {
  }

  /** @brief Set updater params using struct that comes from parsing a yaml string*/
  virtual bool setParams(XmlRpc::XmlRpcValue &params) = 0;

  /** @brief Do any necessary setup (subscribe to ros topics, etc.)*/
  virtual bool initialize() = 0;

  virtual void start() = 0;
  
  virtual void stop() = 0;

  virtual mesh_filter::MeshHandle excludeShape(const shapes::ShapeConstPtr &shape) = 0;

  const std::string& getType() const
  {
    return type_;
  } 

  virtual void setTransformCallback(const boost::function<bool(mesh_filter::MeshHandle, Eigen::Affine3d&)> &transform_callback)
  {
    transform_provider_callback_ = transform_callback;
  }

  void setUpdateCallback(const boost::function<void()> &update_callback)
  {
    update_callback_ = update_callback;
  }
  
protected:
  
  OccupancyMapMonitor *monitor_;
  std::string type_;  
  boost::function<void()> update_callback_;
  boost::function<bool(mesh_filter::MeshHandle, Eigen::Affine3d&)> transform_provider_callback_;
  
  void triggerUpdateCallback(void)
  {
    if (update_callback_)
      update_callback_();
  }
  
};

typedef boost::shared_ptr<OccupancyMapUpdater> OccupancyMapUpdaterPtr;
typedef boost::shared_ptr<const OccupancyMapUpdater> OccupancyMapUpdaterConstPtr;

}

#endif /* MOVEIT_OCCUPANCY_MAP_UPDATER_H_ */
