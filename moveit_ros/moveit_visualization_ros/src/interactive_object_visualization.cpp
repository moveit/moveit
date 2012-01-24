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

  interactive_markers::MenuHandler::EntryHandle del_entry
    = default_menu_handler_.insert("Delete object", 
                                   boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));

  menu_handle_to_function_map_[del_entry] = boost::bind(&InteractiveObjectVisualization::deleteObject, this, _1);
}


planning_scene::PlanningSceneConstPtr InteractiveObjectVisualization::addCube(const std::string& name) {
  std::string name_to_pass;
  if(name.empty()) {
    name_to_pass = generateNewCubeName();
  } 
  return addObject(name_to_pass, 
                   moveit_msgs::Shape::BOX);
}

planning_scene::PlanningSceneConstPtr InteractiveObjectVisualization::addSphere(const std::string& name) {
  std::string name_to_pass;
  if(name.empty()) {
    name_to_pass = generateNewSphereName();
  } 
  return addObject(name_to_pass, 
                   moveit_msgs::Shape::SPHERE);
}

planning_scene::PlanningSceneConstPtr InteractiveObjectVisualization::addCylinder(const std::string& name) {
  std::string name_to_pass;
  if(name.empty()) {
    name_to_pass = generateNewCylinderName();
  } 
  return addObject(name_to_pass, 
                   moveit_msgs::Shape::CYLINDER);
}

planning_scene::PlanningSceneConstPtr InteractiveObjectVisualization::addObject(const std::string& name,
                                                                                const moveit_msgs::Shape::_type_type& type) {
  moveit_msgs::CollisionObject coll;
  
  coll.id = name;
  coll.operation = moveit_msgs::CollisionObject::ADD;
  coll.header.frame_id = planning_scene_->getPlanningFrame();
  coll.poses.resize(1);
  coll.poses[0].position.x = DEFAULT_X;
  coll.poses[0].position.y = DEFAULT_Y;
  coll.poses[0].position.z = DEFAULT_Z;
  coll.poses[0].orientation.w = 1.0;

  coll.shapes.resize(1);
  coll.shapes[0].type = type;
  if(type == moveit_msgs::Shape::SPHERE) {
    coll.shapes[0].dimensions.push_back(DEFAULT_SCALE);
  } else if(type == moveit_msgs::Shape::CYLINDER) {
    coll.shapes[0].dimensions.push_back(DEFAULT_SCALE);
    coll.shapes[0].dimensions.push_back(DEFAULT_SCALE);
  } else {
    coll.shapes[0].dimensions.push_back(DEFAULT_SCALE);
    coll.shapes[0].dimensions.push_back(DEFAULT_SCALE);
    coll.shapes[0].dimensions.push_back(DEFAULT_SCALE);
  }

  planning_scene_diff_->processCollisionObjectMsg(coll);
  
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = planning_scene_->getPlanningFrame();
  pose_stamped.pose = coll.poses[0];
  visualization_msgs::InteractiveMarker marker = makeButtonBox(name,
                                                               pose_stamped,
                                                               DEFAULT_SCALE*1.1,
                                                               false, 
                                                               false);
  add6DofControl(marker, false);
  interactive_marker_server_->insert(marker);
  interactive_marker_server_->setCallback(marker.name, 
                                          boost::bind(&InteractiveObjectVisualization::processInteractiveMarkerFeedback, this, _1));
  
  default_menu_handler_.apply(*interactive_marker_server_, marker.name);
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

void InteractiveObjectVisualization::deleteObject(const std::string& name) {
  moveit_msgs::CollisionObject coll;
  
  coll.id = name;
  coll.operation = moveit_msgs::CollisionObject::REMOVE;
  
  planning_scene_diff_->processCollisionObjectMsg(coll);

  interactive_marker_server_->erase(name);
  interactive_marker_server_->applyChanges();

  callUpdateCallback();
}

void InteractiveObjectVisualization::callUpdateCallback() {
  if(update_callback_) {
    update_callback_(planning_scene_diff_);
  }
}

void InteractiveObjectVisualization::processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) 
{    
  ROS_DEBUG_STREAM("Processing feedback for " << feedback->marker_name);
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

void InteractiveObjectVisualization::processInteractiveMenuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) 
{
  if(feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
    ROS_WARN_STREAM("Got something other than menu select on menu feedback function");
    return;
  }
  if(menu_handle_to_function_map_.find(feedback->menu_entry_id) == menu_handle_to_function_map_.end()) {
    ROS_WARN_STREAM("No callback associated with entry");
    return;
  }
  menu_handle_to_function_map_[feedback->menu_entry_id](feedback->marker_name);
}

}

