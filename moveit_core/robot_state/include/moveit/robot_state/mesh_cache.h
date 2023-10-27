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

#pragma once

#include <shape_msgs/Mesh.h>
#include <geometric_shapes/shape_operations.h>

namespace moveit
{
namespace core
{
using MeshHash = std::size_t;
using TimeHash = std::pair<int, int>;

std::string meshString(const shape_msgs::Mesh& mesh);

MeshHash hashMeshString(const std::string& mesh_string);

MeshHash hashMesh(const shape_msgs::Mesh& mesh);

TimeHash hashTime(const ros::Time& time);

struct MeshCacheControlBlock
{
  std::string mesh_string_;
  MeshHash mesh_hash_;
  shapes::ShapeConstPtr shape_;
  ros::Time last_used_;
  TimeHash last_used_hash_;
  std::size_t approximateMemoryUse() const;
};

class MeshCache
{
public:
  MeshCache(std::size_t min_size_to_cache = 0, std::size_t max_cache_size = std::numeric_limits<std::size_t>::max());
  static MeshCache& threadLocalCache();
  shapes::ShapeConstPtr getShape(const shape_msgs::Mesh& mesh);

private:
  void cacheControlBlock(std::shared_ptr<MeshCacheControlBlock> control_block);
  void removeControlBlock(std::shared_ptr<MeshCacheControlBlock> control_block);
  void updateLastUsed(std::shared_ptr<MeshCacheControlBlock> control_block);

  const std::size_t min_size_to_cache_;
  const std::size_t max_cache_size_;
  std::map<MeshHash, std::shared_ptr<MeshCacheControlBlock>> cache_by_mesh_hash_;
  std::map<TimeHash, std::shared_ptr<MeshCacheControlBlock>> cache_by_last_used_;
  std::size_t cache_size_;
};
}  // namespace core
}  // namespace moveit