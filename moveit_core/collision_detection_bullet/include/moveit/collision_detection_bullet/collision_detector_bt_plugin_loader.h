/*
 * collision_detector_bt_plugin_loader.h
 */

#ifndef MOVEIT_COLLISION_DETECTION_BT_COLLISION_DETECTOR_BT_PLUGIN_LOADER_H_
#define MOVEIT_COLLISION_DETECTION_BT_COLLISION_DETECTOR_BT_PLUGIN_LOADER_H_

#include <moveit/collision_detection/collision_plugin.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bt.h>

namespace collision_detection
{
class CollisionDetectorBtPluginLoader : public CollisionPlugin
{
public:
  virtual bool initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const;
};
}
#endif  // MOVEIT_COLLISION_DETECTION_BT_COLLISION_DETECTOR_BT_PLUGIN_LOADER_H_
