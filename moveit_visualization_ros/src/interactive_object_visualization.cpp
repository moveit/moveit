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

#include <moveit_visualization_ros/interactive_object_visualization.h>

namespace moveit_visualization_ros {

InteractiveObjectVisualization::InteractiveObjectVisualization(planning_scene::PlanningSceneConstPtr planning_scene,
                                                               boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server, 
                                                               const std_msgs::ColorRGBA& color) 
  : planning_scene_(planning_scene),
    interactive_marker_server_(interactive_marker_server),
    cube_counter_(0),
    sphere_counter_(0),
    cylinder_counter_(0)
{
  planning_scene_diff_.reset(new planning_scene::PlanningScene(planning_scene_));
}


planning_scene::PlanningSceneConstPtr InteractiveObjectVisualization::addCube(const std::string& name) {
  std::string name_to_pass;
  if(name.empty()) {
    name_to_pass = generateNewCubeName();
  } 
  return addObject(name_to_pass, 
                   new shapes::Box(DEFAULT_SCALE,
                                   DEFAULT_SCALE,
                                   DEFAULT_SCALE));
}

planning_scene::PlanningSceneConstPtr InteractiveObjectVisualization::addSphere(const std::string& name) {
  std::string name_to_pass;
  if(name.empty()) {
    name_to_pass = generateNewSphereName();
  } 
  return addObject(name_to_pass, 
                   new shapes::Sphere(DEFAULT_SCALE));
}

planning_scene::PlanningSceneConstPtr InteractiveObjectVisualization::addCylinder(const std::string& name) {
  std::string name_to_pass;
  if(name.empty()) {
    name_to_pass = generateNewCylinderName();
  } 
  return addObject(name_to_pass, 
                   new shapes::Cylinder(DEFAULT_SCALE,
                                        DEFAULT_SCALE));
}

planning_scene::PlanningSceneConstPtr InteractiveObjectVisualization::addObject(const std::string& name,
                                                                                shapes::Shape* shape) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = planning_scene_->getPlanningFrame();
  pose.pose.position.x = DEFAULT_X;
  pose.pose.position.y = DEFAULT_Y;
  pose.pose.position.z = DEFAULT_Z;
  pose.pose.orientation.w = 1.0;
  
  Eigen::Affine3d aff;
  planning_models::poseFromMsg(pose.pose, aff);
  planning_scene_diff_->getCollisionWorld()->addToObject(name, 
                                                         shape,
                                                         aff);
  
  visualization_msgs::InteractiveMarker marker = makeButtonBox(name,
                                                               pose,
                                                               DEFAULT_SCALE,
                                                               false, 
                                                               false);
  add6DofControl(marker, false);
  interactive_marker_server_->insert(marker);
  interactive_marker_server_->setCallback(marker.name, 
                                          boost::bind(&InteractiveObjectVisualization::processInteractiveMarkerFeedback, this, _1));
  
  interactive_marker_server_->applyChanges();
  return planning_scene_diff_;
}

void InteractiveObjectVisualization::setUpdateCallback(const boost::function<void(planning_scene::PlanningSceneConstPtr)>& callback) {
  update_callback_ = callback;
}

void InteractiveObjectVisualization::updateObjectPose(const std::string& name,
                                                      const geometry_msgs::Pose& pose) 
{
  collision_detection::CollisionWorld::ObjectConstPtr obj = planning_scene_diff_->getCollisionWorld()->getObject(name);
  if(!obj) {
    ROS_WARN_STREAM("No object with name " << name);
    return;
  }
  Eigen::Affine3d aff;
  planning_models::poseFromMsg(pose, aff);
  planning_scene_diff_->getCollisionWorld()->moveShapeInObject(name, obj->shapes_[0], aff);
  
  callUpdateCallback();
}

void InteractiveObjectVisualization::callUpdateCallback() {
  if(update_callback_) {
    update_callback_(planning_scene_diff_);
  }
}

void InteractiveObjectVisualization::processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) 
{    
  ROS_INFO_STREAM("Processing feedback for " << feedback->marker_name);
  switch (feedback->event_type) {
  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    updateObjectPose(feedback->marker_name, feedback->pose);
    break;
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    break;
  default:
    ROS_DEBUG_STREAM("Getting event type " << feedback->event_type);
  }
  interactive_marker_server_->applyChanges();
}; 


}

