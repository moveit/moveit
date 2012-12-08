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
#include <OGRE/OgreMovableObject.h>
#include <geometric_shapes/shapes.h>

#include "rviz/ogre_helpers/point_cloud.h"

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

class OcTreeRender : public Ogre::MovableObject
{

public:
  OcTreeRender(const shapes::Shape *shape, Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = NULL);
  virtual ~OcTreeRender();

  virtual const Ogre::String& getMovableType() const;
  virtual const Ogre::AxisAlignedBox& getBoundingBox() const;
  virtual float getBoundingRadius() const;
  virtual void getWorldTransforms( Ogre::Matrix4* xform ) const;
  virtual void _updateRenderQueue( Ogre::RenderQueue* queue );
  virtual void _notifyCurrentCamera( Ogre::Camera* camera );
  virtual void _notifyAttached(Ogre::Node *parent, bool isTagPoint=false);
#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 6)
  virtual void visitRenderables(Ogre::Renderable::Visitor* visitor, bool debugRenderables);
#endif

private:
  void setColor( double z_pos, double min_z, double max_z, double color_factor, rviz::PointCloud::Point* point);
  void octreeDecoding (boost::shared_ptr<const octomap::OcTree> octree);

  // Ogre-rviz point clouds
  std::vector<rviz::PointCloud*> cloud_;

  const shapes::Shape *shape_;
  Ogre::SceneNode* scene_node_;
  Ogre::SceneManager* scene_manager_;

  Ogre::AxisAlignedBox bb_;

  double colorFactor_;

};

}
#endif

