/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#ifndef _PROXIMITY_VISUALIZATION_H_
#define _PROXIMITY_VISUALIZATION_H_

#include <ros/ros.h>
#include <planning_scene/planning_scene.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <collision_distance_field/collision_distance_field_types.h>
#include <collision_distance_field/collision_robot_distance_field.h>
#include <collision_distance_field/collision_world_distance_field.h>

namespace moveit_visualization_ros 
{

class ProximityVisualization {

public:

  ProximityVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                         //boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                         ros::Publisher& marker_publisher);

  void updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene);
  
  void groupChanged(const std::string& group_name);

  void stateChanged(const std::string& group,
                    const planning_models::KinematicState& state);
  
protected:
  
  std::string current_group_;

  std::vector<std::string> last_object_ids_;
  
  planning_scene::PlanningSceneConstPtr planning_scene_;
  collision_detection::AllowedCollisionMatrix distance_acm_;

  collision_distance_field::CollisionRobotDistanceField robot_;
  collision_distance_field::CollisionWorldDistanceField world_;

  ros::Publisher publisher_;

};

}

#endif
