/*
 * collision_detector_fcl_plugin_loader.h
 */

#pragma once

#include <moveit/collision_detection/collision_plugin.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>

namespace collision_detection
{
class CollisionDetectorFCLPluginLoader : public CollisionPlugin
{
public:
  virtual bool initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const;
};
}
