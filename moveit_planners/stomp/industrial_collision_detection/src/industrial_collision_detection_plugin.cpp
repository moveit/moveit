/**
 * @file industrial_collision_detection_plugin.h
 * @brief The industrial FCL plugin loader
 *
 * @author Levi Armstrong
 * @date May 4, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @license Software License Agreement (Apache License)\n
 * \n
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at\n
 * \n
 * http://www.apache.org/licenses/LICENSE-2.0\n
 * \n
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <industrial_collision_detection/industrial_collision_detection_plugin.h>
#include <pluginlib/class_list_macros.h>

namespace collision_detection
{
  bool IndustrialFCLPluginLoader::initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const
  {
    scene->setActiveCollisionDetector(CollisionDetectorAllocatorIndustrial::create(), exclusive);
    return true;
  }
}

PLUGINLIB_EXPORT_CLASS(collision_detection::IndustrialFCLPluginLoader, collision_detection::CollisionPlugin)
