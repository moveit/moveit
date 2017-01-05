/*
 * collision_detector_hybrid_plugin_loader.h
 *
 *  Created on: 16-Aug-2016
 *      Author: ace
 */

#ifndef MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_DETECTOR_HYBRID_PLUGIN_LOADER_H_
#define MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_DETECTOR_HYBRID_PLUGIN_LOADER_H_

#include <moveit/collision_detection/collision_plugin.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>

namespace collision_detection
{
class CollisionDetectorHybridPluginLoader : public CollisionPlugin
{
public:
  virtual bool initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const;
};
}
#endif  // MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_DETECTOR_HYBRID_PLUGIN_LOADER_H_
