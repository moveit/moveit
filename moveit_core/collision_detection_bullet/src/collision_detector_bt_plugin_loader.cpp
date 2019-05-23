#include <moveit/collision_detection_bullet/collision_detector_bt_plugin_loader.h>
#include <pluginlib/class_list_macros.h>

namespace collision_detection
{
bool CollisionDetectorBtPluginLoader::initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const
{
  scene->setActiveCollisionDetector(CollisionDetectorAllocatorBt::create(), exclusive);
  return true;
}
}

PLUGINLIB_EXPORT_CLASS(collision_detection::CollisionDetectorBtPluginLoader, collision_detection::CollisionPlugin)
