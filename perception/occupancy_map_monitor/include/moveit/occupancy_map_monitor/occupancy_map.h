/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Jon Binney */

#ifndef MOVEIT_OCCUPANCY_MAP_MONITOR_OCCUPANCY_MAP_
#define MOVEIT_OCCUPANCY_MAP_MONITOR_OCCUPANCY_MAP_

#include <octomap/octomap.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/function.hpp>

namespace occupancy_map_monitor
{

typedef octomap::OcTreeNode OccMapNode;

class OccMapTree : public octomap::OcTree
{
public:
  
  OccMapTree(double resolution) : octomap::OcTree(resolution)
  {
  }

  OccMapTree(const std::string &filename) : octomap::OcTree(filename)
  {
  }
  
  /** @brief lock the underlying octree. it will not be read or written by the
   *  monitor until unlockTree() is called */
  void lockRead()
  {
    tree_mutex_.lock_shared();
  }
  
  /** @brief unlock the underlying octree. */
  void unlockRead()
  {
    tree_mutex_.unlock_shared();
  }
  
  /** @brief lock the underlying octree. it will not be read or written by the
   *  monitor until unlockTree() is called */
  void lockWrite()
  {  
    tree_mutex_.lock();
  }
  
  /** @brief unlock the underlying octree. */
  void unlockWrite()
  {  
    tree_mutex_.unlock();
  }

  void triggerUpdateCallback(void)
  {
    if (update_callback_)
      update_callback_();
  }
  
  /** @brief Set the callback to trigger when updates are received */
  void setUpdateCallback(const boost::function<void()> &update_callback)
  {
    update_callback_ = update_callback;
  }
  
private:
  boost::shared_mutex tree_mutex_;
  boost::function<void()> update_callback_;
};

typedef boost::shared_ptr<OccMapTree> OccMapTreePtr;
typedef boost::shared_ptr<const OccMapTree> OccMapTreeConstPtr;

}

#endif 
