#include <moveit/collision_detection_fcl/collision_detector_fcl_plugin_loader.h>
#include <pluginlib/class_list_macros.h>

namespace collision_detection
{
bool CollisionDetectorFCLPluginLoader::initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const
{
  scene->setActiveCollisionDetector(CollisionDetectorAllocatorFCL::create(), exclusive);
  return true;
}
}  // namespace collision_detection

PLUGINLIB_EXPORT_CLASS(collision_detection::CollisionDetectorFCLPluginLoader, collision_detection::CollisionPlugin)
