/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Julius Kammerl */

#ifndef MOVEIT_VISUALIZATION_SCENE_DISPLAY_RVIZ_OCTOMAP_RENDER_
#define MOVEIT_VISUALIZATION_SCENE_DISPLAY_RVIZ_OCTOMAP_RENDER_

#include <vector>
#include <rviz/ogre_helpers/point_cloud.h>

namespace octomap
{
class OcTree;
}

namespace Ogre
{
class SceneManager;
class SceneNode;
class AxisAlignedBox;
}

namespace moveit_rviz_plugin
{

class OcTreeRender
{

public:
  OcTreeRender(const boost::shared_ptr<const octomap::OcTree> &octree, Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = NULL, std::size_t max_octree_depth = 0);
  virtual ~OcTreeRender();

private:
  void setColor( double z_pos, double min_z, double max_z, double color_factor, rviz::PointCloud::Point* point);
  void octreeDecoding (const boost::shared_ptr<const octomap::OcTree> &octree);

  // Ogre-rviz point clouds
  std::vector<rviz::PointCloud*> cloud_;
  boost::shared_ptr<const octomap::OcTree> octree_;
  
  Ogre::SceneNode* scene_node_;
  Ogre::SceneManager* scene_manager_;

  double colorFactor_;

  std::size_t octree_depth_;

};

}
#endif

