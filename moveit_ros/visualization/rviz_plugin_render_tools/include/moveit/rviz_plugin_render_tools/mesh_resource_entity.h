/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#pragma once

#include <Eigen/Geometry>

#include <rviz/helpers/color.h>

#include <OGRE/OgreMaterial.h>

#include <vector>

namespace Ogre
{
class SceneNode;
class Entity;
}  // namespace Ogre

namespace rviz
{
class Color;
class DisplayContext;
}  // namespace rviz

namespace moveit_rviz_plugin
{
class MeshResourceEntity
{
public:
  MeshResourceEntity(rviz::DisplayContext* context, Ogre::SceneNode* parent_node);
  ~MeshResourceEntity();

  std::set<Ogre::MaterialPtr> getMaterials();

  void onNewMessage(const rviz::Color& color, float alpha, const std::string& mesh_resource,
                    const Eigen::Isometry3d& pose, double scale);

protected:
  void reset();

  Ogre::Entity* entity_;
  std::set<Ogre::MaterialPtr> materials_;

  //! Scaling factor to convert units. Currently relevant for Collada only.
  float unit_rescale_;

  rviz::DisplayContext* context_;
  Ogre::SceneNode* scene_node_;

  rviz::Color color_;
  float alpha_;
  std::string mesh_resource_;
};

}  // namespace moveit_rviz_plugin
