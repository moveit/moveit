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
#include <geometric_shapes/shape_operations.h>

static const double MIN_DIMENSION = .02;

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
  default_object_color_.r = default_object_color_.g = default_object_color_.b = .75;
  default_object_color_.a = 1.0;
}


void InteractiveObjectVisualization::addCube(const std::string& name) {
  std::string name_to_pass;
  if(name.empty()) {
    name_to_pass = generateNewCubeName();
  } 
  geometry_msgs::Pose pose;
  pose.position.x = DEFAULT_X;
  pose.position.y = DEFAULT_Y;
  pose.position.z = DEFAULT_Z;
  pose.orientation.w = 1.0;

  moveit_msgs::Shape shape;
  shape.type = moveit_msgs::Shape::BOX;
  shape.dimensions.resize(3, DEFAULT_SCALE);

  addObject(name_to_pass, 
            pose,
            shape);
}

void InteractiveObjectVisualization::addSphere(const std::string& name) {
  std::string name_to_pass;
  if(name.empty()) {
    name_to_pass = generateNewSphereName();
  } 
  geometry_msgs::Pose pose;
  pose.position.x = DEFAULT_X;
  pose.position.y = DEFAULT_Y;
  pose.position.z = DEFAULT_Z;
  pose.orientation.w = 1.0;

  moveit_msgs::Shape shape;
  shape.type = moveit_msgs::Shape::SPHERE;
  shape.dimensions.resize(1, DEFAULT_SCALE);

  addObject(name_to_pass, 
            pose,
            shape);
}

void InteractiveObjectVisualization::addCylinder(const std::string& name) {
  std::string name_to_pass;
  if(name.empty()) {
    name_to_pass = generateNewCylinderName();
  } 
  geometry_msgs::Pose pose;
  pose.position.x = DEFAULT_X;
  pose.position.y = DEFAULT_Y;
  pose.position.z = DEFAULT_Z;
  pose.orientation.w = 1.0;

  moveit_msgs::Shape shape;
  shape.type = moveit_msgs::Shape::CYLINDER;
  shape.dimensions.resize(2, DEFAULT_SCALE);

  addObject(name_to_pass, 
            pose,
            shape);               
}

void InteractiveObjectVisualization::addObject(const std::string& name,
                                               const geometry_msgs::Pose& pose_msg,
                                               const moveit_msgs::Shape& shape_msg)
{
  std_msgs::ColorRGBA col;
  col.r = col.b = col.g = .75;
  col.a = 1.0;
  addObject(name, pose_msg, shape_msg, col);
}
void InteractiveObjectVisualization::addObject(const std::string& name,
                                               const geometry_msgs::Pose& pose_msg,
                                               const moveit_msgs::Shape& shape_msg,
                                               const std_msgs::ColorRGBA& col) {
  visualization_msgs::InteractiveMarker tm;
  bool already_have = interactive_marker_server_->get(name, tm);
  
  interactive_markers::MenuHandler::CheckState off_state = interactive_markers::MenuHandler::CHECKED;
  interactive_markers::MenuHandler::CheckState grow_state, shrink_state;
  grow_state = shrink_state = interactive_markers::MenuHandler::UNCHECKED;

  std_msgs::ColorRGBA color_to_use = col;

  if(already_have) {
    moveit_msgs::CollisionObject rem;
    rem.id = name;
    rem.operation = moveit_msgs::CollisionObject::REMOVE;

    if(tm.controls.size() > 0 && tm.controls[0].markers.size() > 0) {
      if(tm.controls[0].markers[0].color.r != default_object_color_.r ||
         tm.controls[0].markers[0].color.g != default_object_color_.g ||
         tm.controls[0].markers[0].color.b != default_object_color_.b) {
        color_to_use = tm.controls[0].markers[0].color;
      }
    }

    object_menu_handlers_[name].getCheckState(menu_name_to_handle_maps_[name]["Off"], off_state);
    object_menu_handlers_[name].getCheckState(menu_name_to_handle_maps_[name]["Grow"], grow_state);
    object_menu_handlers_[name].getCheckState(menu_name_to_handle_maps_[name]["Shrink"], shrink_state);
  
    planning_scene_diff_->processCollisionObjectMsg(rem);
    object_menu_handlers_.erase(name);
    menu_name_to_handle_maps_.erase(name);
    menu_handle_to_function_maps_.erase(name);
  }

  moveit_msgs::CollisionObject coll;
  
  coll.id = name;
  coll.operation = moveit_msgs::CollisionObject::ADD;
  coll.header.frame_id = planning_scene_->getPlanningFrame();
  coll.poses.push_back(pose_msg);
  coll.shapes.push_back(shape_msg);
  
  planning_scene_diff_->processCollisionObjectMsg(coll);
  
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "/"+planning_scene_->getPlanningFrame();
  pose_stamped.pose = coll.poses[0];
  visualization_msgs::InteractiveMarker marker; 
  if(shape_msg.type == moveit_msgs::Shape::BOX) {
    marker = makeButtonBox(name,
                           pose_stamped,
                           shape_msg.dimensions[0],
                           shape_msg.dimensions[1],
                           shape_msg.dimensions[2],
                           false, 
                           false);
  } else if(shape_msg.type == moveit_msgs::Shape::CYLINDER) { 
    marker = makeButtonCylinder(name,
                                pose_stamped,
                                shape_msg.dimensions[0],
                                shape_msg.dimensions[1],
                                false, 
                                false);
  } else {
    marker = makeButtonSphere(name,
                              pose_stamped,
                              shape_msg.dimensions[0],
                              false, 
                              false);
  }
  if(dof_marker_enabled_.find(name) == dof_marker_enabled_.end()) {
    dof_marker_enabled_[name] = true;
  }
  if(dof_marker_enabled_[name]) {
    add6DofControl(marker, false);
  }
  recolorInteractiveMarker(marker, color_to_use);
  interactive_marker_server_->insert(marker);
  interactive_marker_server_->setCallback(marker.name, 
                                          boost::bind(&InteractiveObjectVisualization::processInteractiveMarkerFeedback, this, _1));

  interactive_markers::MenuHandler::EntryHandle del_entry
    = object_menu_handlers_[name].insert("Delete object", 
                                         boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));
  menu_handle_to_function_maps_[name][del_entry] = boost::bind(&InteractiveObjectVisualization::deleteObject, this, _1);
  
  interactive_markers::MenuHandler::EntryHandle resize_entry
    = object_menu_handlers_[name].insert("Resize Mode", 
                                         boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));
  
  interactive_markers::MenuHandler::EntryHandle off_entry = 
    object_menu_handlers_[name].insert(resize_entry, "Off", boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));
  object_menu_handlers_[name].setCheckState(off_entry, off_state);
  
  menu_name_to_handle_maps_[name]["Off"] = off_entry;
  menu_handle_to_function_maps_[name][off_entry] = boost::bind(&InteractiveObjectVisualization::setResizeModeOff, this, _1);
  
  interactive_markers::MenuHandler::EntryHandle grow_entry = 
    object_menu_handlers_[name].insert(resize_entry, "Grow", boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));
  object_menu_handlers_[name].setCheckState(grow_entry, grow_state);
  
  menu_name_to_handle_maps_[name]["Grow"] = grow_entry;
  menu_handle_to_function_maps_[name][grow_entry] = boost::bind(&InteractiveObjectVisualization::setResizeModeGrow, this, _1);
  
  interactive_markers::MenuHandler::EntryHandle shrink_entry = 
    object_menu_handlers_[name].insert(resize_entry, "Shrink", boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));
  object_menu_handlers_[name].setCheckState(shrink_entry, shrink_state);
  
  menu_name_to_handle_maps_[name]["Shrink"] = shrink_entry;
  menu_handle_to_function_maps_[name][shrink_entry] = boost::bind(&InteractiveObjectVisualization::setResizeModeShrink, this, _1);
  
  object_menu_handlers_[name].apply(*interactive_marker_server_, marker.name);

  interactive_marker_server_->applyChanges();
  callUpdateCallback();
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

void InteractiveObjectVisualization::growObject(const std::string& name,
                                                const geometry_msgs::Pose& new_pose_msg) 
{
  collision_detection::CollisionWorld::ObjectConstPtr obj = planning_scene_diff_->getCollisionWorld()->getObject(name);
  if(!obj) {
    ROS_WARN_STREAM("No object with name " << name);
    return;
  }
  Eigen::Affine3d new_pose;
  planning_models::poseFromMsg(new_pose_msg, new_pose);

  Eigen::Affine3d cur_pose = obj->shape_poses_[0];
  geometry_msgs::Pose cur_pose_msg;
  planning_models::msgFromPose(cur_pose, cur_pose_msg);

  Eigen::Affine3d diff = cur_pose.inverse()*new_pose;

  moveit_msgs::Shape shape;
  shapes::constructMsgFromShape(obj->shapes_[0], shape);
  if(shape.type == moveit_msgs::Shape::BOX) {
    shape.dimensions[0] += fabs(diff.translation().x());
    shape.dimensions[1] += fabs(diff.translation().y());
    shape.dimensions[2] += fabs(diff.translation().z());
  } else if(shape.type == moveit_msgs::Shape::CYLINDER) {
    shape.dimensions[0] += fmax(fabs(diff.translation().x()), fabs(diff.translation().y()));
    shape.dimensions[1] += fabs(diff.translation().z());
  } else if(shape.type == moveit_msgs::Shape::SPHERE) {
    shape.dimensions[0] += fmax(fmax(fabs(diff.translation().x()), fabs(diff.translation().y())), fabs(diff.translation().z()));
  }
  
  geometry_msgs::Pose diff_pose_msg;
  Eigen::Affine3d diff_pose_adj = diff;
  diff_pose_adj.translation() /= 2.0;
  planning_models::msgFromPose(cur_pose*diff_pose_adj, diff_pose_msg),

  addObject(name,
            diff_pose_msg,
            shape);

  callUpdateCallback();
}

void InteractiveObjectVisualization::shrinkObject(const std::string& name,
                                                  const geometry_msgs::Pose& new_pose_msg) 
{
  collision_detection::CollisionWorld::ObjectConstPtr obj = planning_scene_diff_->getCollisionWorld()->getObject(name);
  if(!obj) {
    ROS_WARN_STREAM("No object with name " << name);
    return;
  }
  Eigen::Affine3d new_pose;
  planning_models::poseFromMsg(new_pose_msg, new_pose);

  Eigen::Affine3d cur_pose = obj->shape_poses_[0];
  geometry_msgs::Pose cur_pose_msg;
  planning_models::msgFromPose(cur_pose, cur_pose_msg);

  Eigen::Affine3d diff = cur_pose.inverse()*new_pose;

  ROS_INFO_STREAM("Diff is " << diff.translation().x() << " " 
                  << diff.translation().y() << " " 
                  << diff.translation().z());

  if(diff.translation().x() > 0) {
    diff.translation().x() *= -1.0;
  }
  if(diff.translation().y() > 0) {
    diff.translation().y() *= -1.0;
  }
  if(diff.translation().z() > 0) {
    diff.translation().z() *= -1.0;
  }

  moveit_msgs::Shape shape;
  shapes::constructMsgFromShape(obj->shapes_[0], shape);
  if(shape.type == moveit_msgs::Shape::BOX) {
    diff.translation().x() = fmax(diff.translation().x(), -shape.dimensions[0]+MIN_DIMENSION);
    diff.translation().y() = fmax(diff.translation().y(), -shape.dimensions[1]+MIN_DIMENSION);
    diff.translation().z() = fmax(diff.translation().z(), -shape.dimensions[2]+MIN_DIMENSION);
    shape.dimensions[0] += diff.translation().x();
    shape.dimensions[1] += diff.translation().y();
    shape.dimensions[2] += diff.translation().z();
  } else if(shape.type == moveit_msgs::Shape::CYLINDER) {
    diff.translation().x() = fmax(diff.translation().x(), -shape.dimensions[0]+MIN_DIMENSION);
    diff.translation().y() = fmax(diff.translation().y(), -shape.dimensions[0]+MIN_DIMENSION);
    diff.translation().z() = fmax(diff.translation().z(), -shape.dimensions[1]+MIN_DIMENSION);
    shape.dimensions[0] += fmin(diff.translation().x(), diff.translation().y());
    shape.dimensions[1] += diff.translation().z();
  } else if(shape.type == moveit_msgs::Shape::SPHERE) {
    diff.translation().x() = fmax(diff.translation().x(), -shape.dimensions[0]+MIN_DIMENSION);
    diff.translation().y() = fmax(diff.translation().y(), -shape.dimensions[1]+MIN_DIMENSION);
    diff.translation().z() = fmax(diff.translation().z(), -shape.dimensions[2]+MIN_DIMENSION);
    shape.dimensions[0] += fmin(fmin(diff.translation().x(), diff.translation().y()), diff.translation().z());
  }
  
  geometry_msgs::Pose diff_pose_msg;
  Eigen::Affine3d diff_pose_adj = diff;
  diff_pose_adj.translation() /= 2.0;
  planning_models::msgFromPose(cur_pose*diff_pose_adj, diff_pose_msg),

  addObject(name,
            diff_pose_msg,
            shape);

  callUpdateCallback();
}


void InteractiveObjectVisualization::deleteObject(const std::string& name) {
  moveit_msgs::CollisionObject coll;
  coll.id = name;
  coll.operation = moveit_msgs::CollisionObject::REMOVE;
  
  planning_scene_diff_->processCollisionObjectMsg(coll);

  interactive_marker_server_->erase(name);
  interactive_marker_server_->applyChanges();

  dof_marker_enabled_.erase(name);
  object_menu_handlers_.erase(name);
  menu_name_to_handle_maps_.erase(name);
  menu_handle_to_function_maps_.erase(name);

  callUpdateCallback();
}

void InteractiveObjectVisualization::setResizeModeOff(const std::string& name)
{
  object_menu_handlers_[name].setCheckState(menu_name_to_handle_maps_[name]["Off"], interactive_markers::MenuHandler::CHECKED);
  object_menu_handlers_[name].setCheckState(menu_name_to_handle_maps_[name]["Grow"], interactive_markers::MenuHandler::UNCHECKED);
  object_menu_handlers_[name].setCheckState(menu_name_to_handle_maps_[name]["Shrink"], interactive_markers::MenuHandler::UNCHECKED);
  object_menu_handlers_[name].reApply(*interactive_marker_server_);
  interactive_marker_server_->applyChanges();
}

void InteractiveObjectVisualization::setResizeModeGrow(const std::string& name)
{
  object_menu_handlers_[name].setCheckState(menu_name_to_handle_maps_[name]["Off"], interactive_markers::MenuHandler::UNCHECKED);
  object_menu_handlers_[name].setCheckState(menu_name_to_handle_maps_[name]["Grow"], interactive_markers::MenuHandler::CHECKED);
  object_menu_handlers_[name].setCheckState(menu_name_to_handle_maps_[name]["Shrink"], interactive_markers::MenuHandler::UNCHECKED);
  object_menu_handlers_[name].reApply(*interactive_marker_server_);
  interactive_marker_server_->applyChanges();
}

void InteractiveObjectVisualization::setResizeModeShrink(const std::string& name)
{
  object_menu_handlers_[name].setCheckState(menu_name_to_handle_maps_[name]["Off"], interactive_markers::MenuHandler::UNCHECKED);
  object_menu_handlers_[name].setCheckState(menu_name_to_handle_maps_[name]["Grow"], interactive_markers::MenuHandler::UNCHECKED);
  object_menu_handlers_[name].setCheckState(menu_name_to_handle_maps_[name]["Shrink"], interactive_markers::MenuHandler::CHECKED);
  object_menu_handlers_[name].reApply(*interactive_marker_server_);
  interactive_marker_server_->applyChanges();
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
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    {
      //TODO - this can probably change when we can swap out markers and continue dragging
      interactive_markers::MenuHandler::CheckState off_state, grow_state, shrink_state;
      object_menu_handlers_[feedback->marker_name].getCheckState(menu_name_to_handle_maps_[feedback->marker_name]["Off"], off_state);
      object_menu_handlers_[feedback->marker_name].getCheckState(menu_name_to_handle_maps_[feedback->marker_name]["Grow"], grow_state);
      object_menu_handlers_[feedback->marker_name].getCheckState(menu_name_to_handle_maps_[feedback->marker_name]["Shrink"], shrink_state);
      if(shrink_state == interactive_markers::MenuHandler::CHECKED) {
        shrinkObject(feedback->marker_name, feedback->pose);
      } else if(grow_state == interactive_markers::MenuHandler::CHECKED) {
        growObject(feedback->marker_name, feedback->pose);
      } else {
        updateObjectPose(feedback->marker_name, feedback->pose);
      }
    }
    break;
  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      interactive_markers::MenuHandler::CheckState off_state;
      object_menu_handlers_[feedback->marker_name].getCheckState(menu_name_to_handle_maps_[feedback->marker_name]["Off"], off_state);
      if(off_state == interactive_markers::MenuHandler::CHECKED) {
        updateObjectPose(feedback->marker_name, feedback->pose);
      }      
    }
    break;
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    {
      dof_marker_enabled_[feedback->marker_name] = !dof_marker_enabled_[feedback->marker_name];
      collision_detection::CollisionWorld::ObjectConstPtr obj = planning_scene_diff_->getCollisionWorld()->getObject(feedback->marker_name);
      moveit_msgs::Shape shape;
      shapes::constructMsgFromShape(obj->shapes_[0], shape);
      geometry_msgs::Pose cur_pose_msg;
      planning_models::msgFromPose(obj->shape_poses_[0], cur_pose_msg);
      addObject(feedback->marker_name,
                cur_pose_msg,
                shape);
    }
    break;
  default:
    ROS_DEBUG_STREAM("Getting event type " << feedback->event_type);
  }
}; 

void InteractiveObjectVisualization::processInteractiveMenuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) 
{
  if(feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
    ROS_WARN_STREAM("Got something other than menu select on menu feedback function");
    return;
  }
  if(menu_handle_to_function_maps_.find(feedback->marker_name) == menu_handle_to_function_maps_.end()) {
    ROS_WARN_STREAM("No menu entry associated with " << feedback->marker_name);
    return;
  }

  if(menu_handle_to_function_maps_[feedback->marker_name].find(feedback->menu_entry_id) == 
     menu_handle_to_function_maps_[feedback->marker_name].end()) {
    ROS_WARN_STREAM("Got menu entry with no handle callback");
    return;
  }

  menu_handle_to_function_maps_[feedback->marker_name][feedback->menu_entry_id](feedback->marker_name);
}

}

