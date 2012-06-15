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

#include <moveit_visualization_ros/proximity_visualization.h>

namespace moveit_visualization_ros
{

ProximityVisualization::ProximityVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                                     ros::Publisher& marker_publisher)
  :
  planning_scene_(planning_scene), 
  robot_(planning_scene->getKinematicModel()),
  world_(),
  publisher_(marker_publisher)
{
  distance_acm_ = planning_scene->getAllowedCollisionMatrix();
  distance_acm_.setEntry("r_shoulder_pan_link", "r_shoulder_pan_link", true);
  distance_acm_.setEntry("r_shoulder_lift_link", "r_shoulder_lift_link", true);
  distance_acm_.setEntry("r_upper_arm_link", "r_upper_arm_link", true);  
  distance_acm_.setEntry("l_shoulder_pan_link", "l_shoulder_pan_link", true);
  distance_acm_.setEntry("l_shoulder_lift_link", "l_shoulder_lift_link", true);
  distance_acm_.setEntry("l_upper_arm_link", "l_upper_arm_link", true);  
}

void ProximityVisualization::updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene)
{
  planning_scene_ = planning_scene;
  std::vector<std::string> object_ids = planning_scene_->getCollisionWorld()->getObjectIds();
  for(unsigned int i = 0; i < object_ids.size(); i++) {
    collision_detection::CollisionWorld::ObjectConstPtr other_obj = planning_scene_->getCollisionWorld()->getObject(object_ids[i]);
    if(!world_.hasObject(object_ids[i])) {
      world_.addToObject(object_ids[i], other_obj->shapes_[0], other_obj->shape_poses_[0]);
    } else {
      collision_detection::CollisionWorld::ObjectConstPtr our_obj = world_.getObject(object_ids[i]);
      if((our_obj->shape_poses_[0].translation()-other_obj->shape_poses_[0].translation()).norm() > .01) {
        world_.moveShapeInObject(our_obj->id_,
                                 our_obj->shapes_[0],
                                 other_obj->shape_poses_[0]);
      }
    }
  }
  groupChanged(current_group_);
}

void ProximityVisualization::groupChanged(const std::string& group) 
{
  current_group_ = group;
  stateChanged(group, planning_scene_->getCurrentState());
}

void ProximityVisualization::stateChanged(const std::string& group,
                                          const planning_models::KinematicState& state)
{
  if(group != current_group_) return;
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = current_group_;
  boost::shared_ptr<const collision_distance_field::CollisionRobotDistanceField::GroupStateRepresentation> world_gsr =
    world_.getCollisionGradients(req, 
                                 res, 
                                 robot_, 
                                 state,
                                 distance_acm_);

  visualization_msgs::MarkerArray arrow_markers;
  std_msgs::ColorRGBA col;
  col.b = 1.0;
  col.a = .8;
  collision_distance_field::getProximityGradientMarkers(planning_scene_->getPlanningFrame(),
                                                        "arrows",
                                                        ros::Duration(0.0),
                                                        world_gsr->link_body_decompositions_,
                                                        world_gsr->gradients_,
                                                        arrow_markers);

  publisher_.publish(arrow_markers);
}

}

