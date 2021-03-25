/*
 * collision_detector_fcl_plugin_loader.h
 */

#ifndef MOVEIT_COLLISION_DETECTION_FCL_COLLISION_DETECTOR_FCL_PLUGIN_LOADER_H_
#define MOVEIT_COLLISION_DETECTION_FCL_COLLISION_DETECTOR_FCL_PLUGIN_LOADER_H_

#include <moveit/collision_detection/collision_plugin.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>

namespace collision_detection
{
class CollisionDetectorFCLPluginLoader : public CollisionPlugin
{
public:
  virtual bool initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const;
};
}  // namespace collision_detection
#endif  // MOVEIT_COLLISION_DETECTION_FCL_COLLISION_DETECTOR_FCL_PLUGIN_LOADER_H_
