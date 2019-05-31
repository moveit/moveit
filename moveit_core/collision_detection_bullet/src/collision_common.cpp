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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Jia Pan */

#include <moveit/collision_detection_bullet/collision_common.h>
#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection_bullet/fcl_compat.h>

#if (MOVEIT_FCL_VERSION >= FCL_VERSION_CHECK(0, 6, 0))
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/octree/octree.h>
#else
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/octree.h>
#endif

#include <boost/thread/mutex.hpp>
#include <memory>

namespace collision_detection
{
struct FCLShapeCache
{
  using ShapeKey = shapes::ShapeConstWeakPtr;
  using ShapeMap = std::map<ShapeKey, FCLGeometryConstPtr, std::owner_less<ShapeKey>>;

  FCLShapeCache() : clean_count_(0)
  {
  }

  void bumpUseCount(bool force = false)
  {
    clean_count_++;

    // clean-up for cache (we don't want to keep infinitely large number of weak ptrs stored)
    if (clean_count_ > MAX_CLEAN_COUNT || force)
    {
      clean_count_ = 0;
      for (auto it = map_.begin(); it != map_.end();)
      {
        auto nit = it;
        ++nit;
        if (it->first.expired())
          map_.erase(it);
        it = nit;
      }
      //      ROS_DEBUG_NAMED("collision_detection.fcl", "Cleaning up cache for FCL objects that correspond to static
      //      shapes. Cache size
      //      reduced from %u
      //      to %u", from, (unsigned int)map_.size());
    }
  }

  static const unsigned int MAX_CLEAN_COUNT = 100;  // every this many uses of the cache, a cleaning operation is
                                                    // executed (this is only removal of expired entries)
  ShapeMap map_;
  unsigned int clean_count_;
};

/* We template the function so we get a different cache for each of the template arguments combinations */
template <typename BV, typename T>
FCLShapeCache& GetShapeCache()
{
  /* The cache is created thread_local, that is each thread calling
   * this quasi-singleton function will get its own instance. Once
   * the thread joins/exits, the cache gets deleted.
   * Reasoning is that multi-threaded planners (eg OMPL) or user-code
   * will often need to do collision checks with the same object
   * simultaneously (especially true for attached objects). Having only
   * one global cache leads to many cache misses. Also as the cache can
   * only be accessed by one thread we don't need any locking.
   */
  static thread_local FCLShapeCache cache;
  return cache;
}

void cleanCollisionGeometryCache()
{
  FCLShapeCache& cache1 = GetShapeCache<fcl::OBBRSSd, World::Object>();
  {
    cache1.bumpUseCount(true);
  }
  FCLShapeCache& cache2 = GetShapeCache<fcl::OBBRSSd, robot_state::AttachedBody>();
  {
    cache2.bumpUseCount(true);
  }
}

void CollisionData::enableGroup(const robot_model::RobotModelConstPtr& robot_model)
{
  if (robot_model->hasJointModelGroup(req_->group_name))
    active_components_only_ = &robot_model->getJointModelGroup(req_->group_name)->getUpdatedLinkModelsSet();
  else
    active_components_only_ = nullptr;
}

bool acmEvaluate(const collision_detection::AllowedCollisionMatrix* acm)
{
  return true;
}

}  // namespace collision_detection
