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

#include "moveit_rviz_plugin/planning_scene_render.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>

#include <rviz/ogre_helpers/shape.h>
#include <rviz/display_context.h>
#include <rviz/robot/robot.h>

#include "planning_link_updater.h"

moveit_rviz_plugin::PlanningSceneRender::PlanningSceneRender(rviz::DisplayContext *context, Ogre::SceneNode *node, rviz::Robot *robot) :
  context_(context), scene_node_(node), scene_robot_(robot)
{
}

void moveit_rviz_plugin::PlanningSceneRender::clear(void)
{
  scene_shapes_.clear();
  for (std::size_t i = 0 ; i < manual_objects_.size() ; ++i)
    context_->getSceneManager()->destroyManualObject(manual_objects_[i]);

  manual_objects_.clear();
  if (!material_name_.empty())
  {
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_->getName());
    material_name_ = "";
  }
}

void moveit_rviz_plugin::PlanningSceneRender::renderShape(Ogre::SceneNode *node, const shapes::Shape *s, const Eigen::Affine3d &p, const rviz::Color &color, float alpha)
{
  rviz::Shape* ogre_shape = NULL;
  switch (s->type)
  {
  case shapes::SPHERE:
    {
      ogre_shape = new rviz::Shape(rviz::Shape::Sphere,
                                   context_->getSceneManager(), node);
      double d = 2.0 * static_cast<const shapes::Sphere*>(s)->radius;
      ogre_shape->setScale(Ogre::Vector3(d, d, d));
    }
    break;
  case shapes::BOX:
    {
      ogre_shape = new rviz::Shape(rviz::Shape::Cube,
                                   context_->getSceneManager(), node);
      const double* sz = static_cast<const shapes::Box*>(s)->size;
      ogre_shape->setScale(Ogre::Vector3(sz[0], sz[1], sz[2]));
    }
    break;
  case shapes::CYLINDER:
    {
      ogre_shape = new rviz::Shape(rviz::Shape::Cylinder,
                                   context_->getSceneManager(), node);
      double d = 2.0 * static_cast<const shapes::Cylinder*>(s)->radius;
      double z = static_cast<const shapes::Cylinder*>(s)->length;
      ogre_shape->setScale(Ogre::Vector3(d, z, d)); // the shape has z as major axis, but the rendered cylinder has y as major axis (assuming z is upright);
    }
    break;
  case shapes::MESH:
    {
      const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(s);
      if (mesh->triangle_count > 0)
      {
        // check if we need to construct the material
        if (material_name_.empty())
        {
          material_name_ = "Planning Display Mesh Material";
          material_ = Ogre::MaterialManager::getSingleton().create( material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
          material_->setReceiveShadows(true);
          material_->getTechnique(0)->setLightingEnabled(true);
          material_->setCullingMode(Ogre::CULL_NONE);
          material_->getTechnique(0)->setAmbient(color.r_, color.g_, color.b_);
          material_->getTechnique(0)->setDiffuse(0, 0, 0, alpha);
          if (alpha < 0.9998)
          {
            material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
            material_->getTechnique(0)->setDepthWriteEnabled( false );
          }
          else
          {
            material_->getTechnique(0)->setSceneBlending( Ogre::SBT_REPLACE );
            material_->getTechnique(0)->setDepthWriteEnabled( true );
          }
        }

        std::string name = "Planning Display Mesh " + boost::lexical_cast<std::string>(manual_objects_.size());
        Ogre::ManualObject *manual_object = context_->getSceneManager()->createManualObject(name);
        manual_object->estimateVertexCount(mesh->triangle_count * 3);
        manual_object->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
        Eigen::Vector3d normal(0.0, 0.0, 0.0);
        for (unsigned int i = 0 ; i < mesh->triangle_count ; ++i)
        {
          unsigned int i3 = i * 3;
          if (mesh->normals)
            for (int k = 0 ; k < 3 ; ++k)
              normal[k] = mesh->normals[i3 + k];
          for (int k = 0 ; k < 3 ; ++k)
          {
            unsigned int vi = 3 * mesh->triangles[i3 + k];
            Eigen::Vector3d v = p * Eigen::Vector3d(mesh->vertices[vi], mesh->vertices[vi + 1], mesh->vertices[vi + 2]);
            manual_object->position(v.x(), v.y(), v.z());
            if (mesh->normals)
              manual_object->normal(normal.x(), normal.y(), normal.z());
          }
        }
        manual_object->end();
        node->attachObject(manual_object);
        manual_objects_.push_back(manual_object);
      }
    }
    break;
  default:
    break;
  }

  if (ogre_shape)
  {
    ogre_shape->setColor(color.r_, color.g_, color.b_, alpha);
    Ogre::Vector3 position(p.translation().x(), p.translation().y(), p.translation().z());
    Eigen::Quaterniond q(p.rotation());
    Ogre::Quaternion orientation(q.w(), q.x(), q.y(), q.z());

    if (s->type == shapes::CYLINDER)
    {
      // in geometric shapes, the z axis of the cylinder is it height;
      // for the rviz shape, the y axis is the height; we add a transform to fix this
      static Ogre::Quaternion fix(Ogre::Radian(M_PI/2.0), Ogre::Vector3(1.0, 0.0, 0.0));
      orientation = fix * orientation;
    }

    ogre_shape->setPosition(position);
    ogre_shape->setOrientation(orientation);
    scene_shapes_.push_back(boost::shared_ptr<rviz::Shape>(ogre_shape));
  }
}

// ******************************************************************************************
// Render Planning Scene
// ******************************************************************************************
void moveit_rviz_plugin::PlanningSceneRender::renderPlanningScene(const planning_scene::PlanningSceneConstPtr &scene, float scene_alpha, float robot_alpha)
{
  static rviz::Color env_color(0.2f, 0.9f, 0.2f);
  static rviz::Color attached_color(0.6f, 0.6f, 0.6f);

  if (!scene)
    return;

  clear();
  
  planning_models::KinematicStateConstPtr ks(new planning_models::KinematicState(scene->getCurrentState()));
  scene_robot_->update(PlanningLinkUpdater(ks));
  collision_detection::CollisionWorldConstPtr cworld = scene->getCollisionWorld();
  const std::vector<std::string> &ids = cworld->getObjectIds();
  for (std::size_t i = 0 ; i < ids.size() ; ++i)
  {
    collision_detection::CollisionWorld::ObjectConstPtr o = cworld->getObject(ids[i]);
    rviz::Color color = env_color;
    if (scene->hasColor(ids[i]))
    {
      const std_msgs::ColorRGBA &c = scene->getColor(ids[i]);
      color.r_ = c.r; color.g_ = c.g; color.b_ = c.b;
    }
    for (std::size_t j = 0 ; j < o->shapes_.size() ; ++j)
      renderShape(scene_node_, o->shapes_[j].get(), o->shape_poses_[j], color, scene_alpha);
  }
  
  std::vector<const planning_models::KinematicState::AttachedBody*> attached_bodies;
  scene->getCurrentState().getAttachedBodies(attached_bodies);
  for (std::size_t i = 0 ; i < attached_bodies.size() ; ++i)
  {
    rviz::Color color = attached_color;
    if (scene->hasColor(attached_bodies[i]->getName()))
    {
      const std_msgs::ColorRGBA &c = scene->getColor(attached_bodies[i]->getName());
      color.r_ = c.r; color.g_ = c.g; color.b_ = c.b;
    }
    const EigenSTL::vector_Affine3d &ab_t = attached_bodies[i]->getGlobalCollisionBodyTransforms();
    const std::vector<shapes::ShapeConstPtr> &ab_shapes = attached_bodies[i]->getShapes();
    for (std::size_t j = 0 ; j < ab_shapes.size() ; ++j)
    {
      renderShape(scene_robot_->getVisualNode(), ab_shapes[j].get(), ab_t[j], color, robot_alpha);
      renderShape(scene_robot_->getCollisionNode(), ab_shapes[j].get(), ab_t[j], color, robot_alpha);
    }
  }
}
