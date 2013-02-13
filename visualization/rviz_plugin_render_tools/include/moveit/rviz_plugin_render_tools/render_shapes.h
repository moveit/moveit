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

/* Author: Ioan Sucan */

#ifndef MOVEIT_VISUALIZATION_SCENE_DISPLAY_RVIZ_RENDER_SHAPES_
#define MOVEIT_VISUALIZATION_SCENE_DISPLAY_RVIZ_RENDER_SHAPES_

#include <geometric_shapes/shapes.h>
#include <rviz/helpers/color.h>
#include <OGRE/OgreMaterial.h>
#include <Eigen/Geometry>
#include <string>
#include <boost/shared_ptr.hpp>
#include <moveit/rviz_plugin_render_tools/octomap_render.h>

namespace Ogre
{
class Entity;
class SceneNode;
class ManualObject;
}

namespace rviz
{
class DisplayContext;
class Shape;
}

namespace moveit_rviz_plugin
{

// forward delcaration
class OcTreeRender;

class RenderShapes
{
public:

  RenderShapes(rviz::DisplayContext *context);
  ~RenderShapes();

  void renderShape(Ogre::SceneNode *node,
                   const shapes::Shape *s,
                   const Eigen::Affine3d &p,
                   OctreeVoxelRenderMode octree_voxel_rendering,
                   OctreeVoxelColorMode octree_color_mode,
                   const rviz::Color &color,
                   float alpha);
  void clear();
  
private:

  rviz::DisplayContext *context_;
  
  std::vector< boost::shared_ptr<rviz::Shape> > scene_shapes_;
  std::vector< Ogre::MovableObject* > movable_objects_;
  std::vector< boost::shared_ptr<OcTreeRender> > octree_voxel_grids_;

  std::vector<Ogre::MaterialPtr> materials_;


};

typedef boost::shared_ptr<RenderShapes> RenderShapesPtr;
typedef boost::shared_ptr<const RenderShapes> RenderShapesConstPtr;

}

#endif

