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

#include <geometric_shapes/check_isometry.h>

#include <moveit/rviz_plugin_render_tools/mesh_resource_entity.h>

#include <rviz/display_context.h>
#include <rviz/mesh_loader.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreSharedPtr.h>
#include <OGRE/OgreTechnique.h>

namespace moveit_rviz_plugin
{
MeshResourceEntity::MeshResourceEntity(rviz::DisplayContext* context, Ogre::SceneNode* parent_node)
  : entity_(nullptr), context_(context)
{
  scene_node_ = parent_node->createChildSceneNode();
}

MeshResourceEntity::~MeshResourceEntity()
{
  reset();
}

void MeshResourceEntity::reset()
{
  // destroy entity
  if (entity_)
  {
    context_->getSceneManager()->destroyEntity(entity_);
    entity_ = nullptr;
  }

  // destroy all the materials we've created
  std::set<Ogre::MaterialPtr>::iterator it;
  for (it = materials_.begin(); it != materials_.end(); it++)
  {
    Ogre::MaterialPtr material = *it;
    if (!material.isNull())
    {
      Ogre::MaterialManager::getSingleton().remove(material->getName());
    }
  }
  materials_.clear();
}

void MeshResourceEntity::onNewMessage(const rviz::Color& color, float alpha, const std::string& mesh_resource,
                                      const Eigen::Isometry3d& pose, double scale)
{
  // flag indicating if the mesh material color needs to be updated
  bool update_color = !entity_;

  scene_node_->setVisible(false);

  // Get the color information from the message
  float r = color.r_;
  float g = color.g_;
  float b = color.b_;
  float a = alpha;

  // todo: when should this be enabled?
  bool mesh_use_embedded_materials = false;

  if (!entity_ || mesh_resource_ != mesh_resource)
  {
    reset();

    mesh_resource_ = mesh_resource;

    if (mesh_resource.empty())
    {
      return;
    }

    if (rviz::loadMeshFromResource(mesh_resource).isNull())
    {
      std::stringstream ss;
      ss << "Mesh resource marker [" << /*getStringID() <<*/ "] could not load [" << mesh_resource << "]";
      ROS_DEBUG("%s", ss.str().c_str());
      return;
    }

    static uint32_t count = 0;
    std::stringstream ss;
    ss << "mesh_resource_entity_" << count++;
    std::string id = ss.str();
    entity_ = context_->getSceneManager()->createEntity(id, mesh_resource);
    scene_node_->attachObject(entity_);

    // create a default material for any sub-entities which don't have their own.
    ss << "Material";
    Ogre::MaterialPtr default_material =
        Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    default_material->setReceiveShadows(false);
    default_material->getTechnique(0)->setLightingEnabled(true);
    default_material->getTechnique(0)->setAmbient(0.5, 0.5, 0.5);
    materials_.insert(default_material);

    if (mesh_use_embedded_materials)
    {
      // make clones of all embedded materials so selection works correctly
      for (const Ogre::MaterialPtr& material : getMaterials())
      {
        if (material->getName() != "BaseWhiteNoLighting")
        {
          Ogre::MaterialPtr new_material = material->clone(id + material->getName());
          // add a new pass to every custom material to perform the color tinting
          Ogre::Pass* pass = new_material->getTechnique(0)->createPass();
          pass->setAmbient(0.0f, 0.0f, 0.0f);
          pass->setDiffuse(0.0f, 0.0f, 0.0f, 0.0f);
          pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
          pass->setDepthWriteEnabled(false);
          pass->setLightingEnabled(true);
          materials_.insert(new_material);
        }
      }

      // make sub-entities use cloned materials
      for (uint32_t i = 0; i < entity_->getNumSubEntities(); ++i)
      {
        std::string mat_name = entity_->getSubEntity(i)->getMaterialName();
        if (mat_name != "BaseWhiteNoLighting")
        {
          entity_->getSubEntity(i)->setMaterialName(id + mat_name);
        }
        else
        {
          // BaseWhiteNoLighting is the default material Ogre uses
          // when it sees a mesh with no material.  Here we replace
          // that with our default_material which gets colored with
          // new_message->color.
          entity_->getSubEntity(i)->setMaterial(default_material);
        }
      }
    }
    else
    {
      entity_->setMaterial(default_material);
    }
  }
  else
  {
    // underlying mesh resource has not changed but if the color has
    // then we need to update the materials color
    if (update_color || color_.r_ != r || color_.g_ != g || color_.b_ != b || alpha_ != a)
    {
      update_color = true;
      color_ = color;
      alpha_ = alpha;
    }
  }

  // update material color
  if (update_color)
  {
    bool depth_write = a >= 0.9998;
    Ogre::SceneBlendType blending = depth_write ? Ogre::SBT_REPLACE : Ogre::SBT_TRANSPARENT_ALPHA;
    bool tinting = mesh_use_embedded_materials;

    for (const Ogre::MaterialPtr& material : materials_)
    {
      Ogre::Technique* technique = material->getTechnique(0);
      Ogre::Pass* pass0 = technique->getPass(0);
      Ogre::Pass* passT = technique->getPass(technique->getNumPasses() - 1);
      if (tinting)
      {
        // modify material's original color to use given alpha value
        Ogre::ColourValue color = pass0->getDiffuse();
        color.a = a;
        pass0->setDiffuse(color);
        // tint by re-rendering with marker color
        passT->setAmbient(r * 0.5f, g * 0.5f, b * 0.5f);
        passT->setDiffuse(r, g, b, std::min(a, 0.5f));
      }
      else
      {
        pass0->setAmbient(r * 0.5f, g * 0.5f, b * 0.5f);
        pass0->setDiffuse(r, g, b, a);
      }

      pass0->setSceneBlending(blending);
      pass0->setDepthWriteEnabled(depth_write);
      pass0->setLightingEnabled(true);
    }
  }

  Eigen::Vector3d translation = pose.translation();
  Ogre::Vector3 position(translation.x(), translation.y(), translation.z());
  ASSERT_ISOMETRY(pose)  // unsanitized input, could contain a non-isometry
  Eigen::Quaterniond q(pose.linear());
  Ogre::Quaternion orientation(q.w(), q.x(), q.y(), q.z());

  scene_node_->setVisible(true);
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  scene_node_->setScale(scale, scale, scale);
}

std::set<Ogre::MaterialPtr> MeshResourceEntity::getMaterials()
{
  std::set<Ogre::MaterialPtr> materials;
  if (entity_)
  {
    uint32_t num_sub_entities = entity_->getNumSubEntities();
    for (uint32_t i = 0; i < num_sub_entities; ++i)
    {
      Ogre::SubEntity* sub = entity_->getSubEntity(i);
      const Ogre::MaterialPtr& material = sub->getMaterial();
      materials.insert(material);
    }
  }
  return materials;
}

}  // namespace moveit_rviz_plugin
