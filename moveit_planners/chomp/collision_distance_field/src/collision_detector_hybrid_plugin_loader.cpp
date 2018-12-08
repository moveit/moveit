#include <moveit/collision_distance_field/collision_detector_hybrid_plugin_loader.h>
#include <pluginlib/class_list_macros.hpp>

namespace collision_detection
{
bool CollisionDetectorHybridPluginLoader::initialize(const planning_scene::PlanningScenePtr& scene,
                                                     bool exclusive) const
{
  scene->setActiveCollisionDetector(CollisionDetectorAllocatorHybrid::create(), exclusive);
  return true;
}
}

PLUGINLIB_EXPORT_CLASS(collision_detection::CollisionDetectorHybridPluginLoader, collision_detection::CollisionPlugin)
