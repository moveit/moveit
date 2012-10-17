/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

// Author: E. Gil Jones

#include <moveit_visualization_ros/interactive_object_visualization_qt_wrapper.h>
#include <moveit_visualization_ros/qt_helper_functions.h>

namespace moveit_visualization_ros {

void InteractiveObjectVisualizationQtWrapper::addCubeSignalled() {
  ROS_INFO_STREAM("Trying to add");
  addCube();
}

void InteractiveObjectVisualizationQtWrapper::callUpdateCallback() {
  InteractiveObjectVisualization::callUpdateCallback();
  updatePlanningSceneSignal(planning_scene_diff_);
}

void InteractiveObjectVisualizationQtWrapper::addCollisionObjectSignalled(const moveit_msgs::CollisionObject& obj,
                                                                          const QColor& color)
{
  addObject(obj,
            convertQColorToRGBA(color));
}

void InteractiveObjectVisualizationQtWrapper::loadPlanningSceneSignalled(moveit_msgs::PlanningScenePtr planning_scene) {
  updateOriginalPlanningScene(planning_scene);
}

void InteractiveObjectVisualizationQtWrapper::deleteSignalled(std::string name) {
  deleteObject(name);
}

void InteractiveObjectVisualizationQtWrapper::attachCollisionObjectSignalled(const std::string& name,
                                                                             const std::string& link_name,
                                                                             const std::vector<std::string>& touch_links)
{
  attachObject(name,
               link_name,
               touch_links);
}

}

