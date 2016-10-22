/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Fetch Robotics Inc.
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
 *   * Neither the name of Fetch Robotics nor the names of its
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

#ifndef MOVEIT_COLLISION_PLUGIN_LOADER_COLLISION_PLUGIN_LOADER_H
#define MOVEIT_COLLISION_PLUGIN_LOADER_COLLISION_PLUGIN_LOADER_H

#include <ros/ros.h>
#include <moveit/macros/class_forward.h>
#include <moveit/collision_detection/collision_plugin.h>

namespace collision_detection
{
/**
 * @brief This is used to load the collision plugin
 */
class CollisionPluginLoader
{
public:
  CollisionPluginLoader();
  ~CollisionPluginLoader();

  /** @brief This can be called on a new planning scene to setup the collision detector. */
  void setupScene(ros::NodeHandle& nh, const planning_scene::PlanningScenePtr& scene);

  /**
   * @brief Load a collision detection robot/world into a planning scene instance.
   * @param name The plugin name.
   * @param scene The planning scene instance.
   * @param exclusive If true, sets the new detection robot/world to be the only one.
   * @return True if collision robot/world were added to scene.
   */
  bool activate(const std::string& name, const planning_scene::PlanningScenePtr& scene, bool exclusive);

private:
  MOVEIT_CLASS_FORWARD(CollisionPluginLoaderImpl);
  CollisionPluginLoaderImplPtr loader_;
};

}  // namespace collision_detection

#endif  // MOVEIT_COLLISION_PLUGIN_LOADER_COLLISION_PLUGIN_LOADER_H
