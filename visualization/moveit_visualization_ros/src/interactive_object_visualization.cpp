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

InteractiveObjectVisualization::InteractiveObjectVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                               boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server, 
                                                               const std_msgs::ColorRGBA& color) 
  : planning_scene_(planning_scene),
    interactive_marker_server_(interactive_marker_server),
    cube_counter_(0),
    sphere_counter_(0),
    cylinder_counter_(0)
{
  planning_scene_diff_ = planning_scene_->diff();
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

  shape_msgs::SolidPrimitive shape;
  shape.type = shape_msgs::SolidPrimitive::BOX;
  shape.dimensions.resize(3);
  shape.dimensions[0] = shape.dimensions[1] = shape.dimensions[2] = DEFAULT_SCALE;

  moveit_msgs::CollisionObject coll;
  coll.id = name_to_pass;
  coll.primitive_poses.push_back(pose);
  coll.primitives.push_back(shape);

  addObject(coll);
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

  shape_msgs::SolidPrimitive shape;
  shape.type = shape_msgs::SolidPrimitive::SPHERE;
  shape.dimensions.resize(1);
  shape.dimensions[0] = DEFAULT_SCALE;
  
  moveit_msgs::CollisionObject coll;
  coll.id = name_to_pass;
  coll.primitive_poses.push_back(pose);
  coll.primitives.push_back(shape);

  addObject(coll);
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

  shape_msgs::SolidPrimitive shape;
  shape.type = shape_msgs::SolidPrimitive::CYLINDER;
  shape.dimensions.resize(2);
  shape.dimensions[0] = DEFAULT_SCALE;
  shape.dimensions[1] = DEFAULT_SCALE;

  moveit_msgs::CollisionObject coll;
  coll.id = name_to_pass;
  coll.primitive_poses.push_back(pose);
  coll.primitives.push_back(shape);

  addObject(coll);
}

void InteractiveObjectVisualization::addObject(const moveit_msgs::CollisionObject& coll)
{
  std_msgs::ColorRGBA col;
  col.r = col.b = col.g = .75;
  col.a = 1.0;
  addObject(coll, col);
}

void InteractiveObjectVisualization::addObject(const moveit_msgs::CollisionObject& coll,
                                               const std_msgs::ColorRGBA& col) {
  visualization_msgs::InteractiveMarker tm;
  bool already_have = interactive_marker_server_->get(coll.id, tm);
  
  interactive_markers::MenuHandler::CheckState off_state = interactive_markers::MenuHandler::CHECKED;
  interactive_markers::MenuHandler::CheckState grow_state, shrink_state;
  grow_state = shrink_state = interactive_markers::MenuHandler::UNCHECKED;

  std_msgs::ColorRGBA color_to_use = col;

  if(already_have) {
    moveit_msgs::CollisionObject rem;
    rem.id = coll.id;
    rem.operation = moveit_msgs::CollisionObject::REMOVE;

    if(tm.controls.size() > 0 && tm.controls[0].markers.size() > 0) {
      if(tm.controls[0].markers[0].color.r != default_object_color_.r ||
         tm.controls[0].markers[0].color.g != default_object_color_.g ||
         tm.controls[0].markers[0].color.b != default_object_color_.b) {
        color_to_use = tm.controls[0].markers[0].color;
      }
    }

    object_menu_handlers_[coll.id].getCheckState(menu_name_to_handle_maps_[coll.id]["Off"], off_state);
    object_menu_handlers_[coll.id].getCheckState(menu_name_to_handle_maps_[coll.id]["Grow"], grow_state);
    object_menu_handlers_[coll.id].getCheckState(menu_name_to_handle_maps_[coll.id]["Shrink"], shrink_state);
  
    planning_scene_diff_->processCollisionObjectMsg(rem);
  }
  ros::WallTime first = ros::WallTime::now();
  if(coll.header.frame_id.empty()) {
    moveit_msgs::CollisionObject coll2 = coll;
    coll2.header.frame_id = planning_scene_->getPlanningFrame();
    planning_scene_diff_->processCollisionObjectMsg(coll2);
  } else {
    planning_scene_diff_->processCollisionObjectMsg(coll);
  }
  //ROS_INFO_STREAM("Insertion took " << (ros::WallTime::now()-first).toSec());
  
  if(coll.primitives.empty() && coll.meshes.empty()) {
    ROS_WARN_STREAM("No shapes");
    return;
  }
  if(coll.primitive_poses.empty() && coll.mesh_poses.empty()) {
    ROS_WARN_STREAM("No poses");
    return;
  }
  visualization_msgs::InteractiveMarker marker; 

  makeInteractiveMarkerButton(coll, color_to_use, marker);

  if(dof_marker_enabled_.find(coll.id) == dof_marker_enabled_.end()) {
    dof_marker_enabled_[coll.id] = true;
  }
  if(dof_marker_enabled_[coll.id]) {
    add6DofControl(marker, false);
  }
  recolorInteractiveMarker(marker, color_to_use);
  interactive_marker_server_->insert(marker);
  interactive_marker_server_->setCallback(coll.id, 
                                          boost::bind(&InteractiveObjectVisualization::processInteractiveMarkerFeedback, this, _1));

  if(menu_name_to_handle_maps_[coll.id].find("Delete object") == menu_name_to_handle_maps_[coll.id].end()) {
    interactive_markers::MenuHandler::EntryHandle del_entry
      = object_menu_handlers_[coll.id].insert("Delete object", 
                                              boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));
    menu_handle_to_function_maps_[coll.id][del_entry] = boost::bind(&InteractiveObjectVisualization::deleteObject, this, _1);
    menu_name_to_handle_maps_[coll.id]["Delete object"] = del_entry;
  }
  if(menu_name_to_handle_maps_[coll.id].find("Resize Mode") == menu_name_to_handle_maps_[coll.id].end()) {
    interactive_markers::MenuHandler::EntryHandle resize_entry
      = object_menu_handlers_[coll.id].insert("Resize Mode", 
                                           boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));
    
    menu_name_to_handle_maps_[coll.id]["Resize Mode"] = resize_entry;

    interactive_markers::MenuHandler::EntryHandle off_entry = 
      object_menu_handlers_[coll.id].insert(resize_entry, "Off", boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));
    object_menu_handlers_[coll.id].setCheckState(off_entry, off_state);
    
    menu_name_to_handle_maps_[coll.id]["Off"] = off_entry;
    menu_handle_to_function_maps_[coll.id][off_entry] = boost::bind(&InteractiveObjectVisualization::setResizeModeOff, this, _1);
    
    interactive_markers::MenuHandler::EntryHandle grow_entry = 
      object_menu_handlers_[coll.id].insert(resize_entry, "Grow", boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));
    object_menu_handlers_[coll.id].setCheckState(grow_entry, grow_state);
    
    menu_name_to_handle_maps_[coll.id]["Grow"] = grow_entry;
    menu_handle_to_function_maps_[coll.id][grow_entry] = boost::bind(&InteractiveObjectVisualization::setResizeModeGrow, this, _1);
    
    interactive_markers::MenuHandler::EntryHandle shrink_entry = 
      object_menu_handlers_[coll.id].insert(resize_entry, "Shrink", boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));
    object_menu_handlers_[coll.id].setCheckState(shrink_entry, shrink_state);
    
    menu_name_to_handle_maps_[coll.id]["Shrink"] = shrink_entry;
    menu_handle_to_function_maps_[coll.id][shrink_entry] = boost::bind(&InteractiveObjectVisualization::setResizeModeShrink, this, _1);
  }
  for(std::map<std::string, boost::function<void(const std::string&)> >::iterator it = all_callback_map_.begin();
      it != all_callback_map_.end(); 
      it++) {
    if(menu_name_to_handle_maps_[coll.id].find(it->first) == menu_name_to_handle_maps_[coll.id].end()) {
      interactive_markers::MenuHandler::EntryHandle eh = object_menu_handlers_[coll.id].insert(it->first,
                                                                                            boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));
      menu_name_to_handle_maps_[coll.id][it->first] = eh;
      menu_handle_to_function_maps_[coll.id][eh] = it->second;
    }
  }
  
  object_menu_handlers_[coll.id].apply(*interactive_marker_server_, coll.id);

  interactive_marker_server_->applyChanges();
  callUpdateCallback();
}

void InteractiveObjectVisualization::updateCurrentState(const planning_models::KinematicState& current_state) {
  planning_scene_diff_->setCurrentState(current_state);
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
  // we discard the obj (we don't need it) and this allows more efficient caching in the collision checking library
  shapes::ShapeConstPtr shape = obj->shapes_[0];
  obj.reset();
  
  Eigen::Affine3d aff;
  planning_models::poseFromMsg(pose, aff);
  planning_scene_diff_->getCollisionWorld()->moveShapeInObject(name, shape, aff);
  
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
  
  if (obj->shapes_[0]->type == shapes::SPHERE || obj->shapes_[0]->type == shapes::BOX || obj->shapes_[0]->type == shapes::CYLINDER)
  {
    Eigen::Affine3d new_pose;
    planning_models::poseFromMsg(new_pose_msg, new_pose);
    
    Eigen::Affine3d cur_pose = obj->shape_poses_[0];
    geometry_msgs::Pose cur_pose_msg;
    planning_models::msgFromPose(cur_pose, cur_pose_msg);
    
    Eigen::Affine3d diff = cur_pose.inverse()*new_pose;
    
    shapes::ShapeMsg shape_variant;
    shape_msgs::SolidPrimitive shape;
    if (shapes::constructMsgFromShape(obj->shapes_[0].get(), shape_variant))
    {
      shape = boost::get<shape_msgs::SolidPrimitive>(shape_variant);
      if(shape.type == shape_msgs::SolidPrimitive::BOX) {
        shape.dimensions[0] += fabs(diff.translation().x());
        shape.dimensions[1] += fabs(diff.translation().y());
        shape.dimensions[2] += fabs(diff.translation().z());
      } else if(shape.type == shape_msgs::SolidPrimitive::CYLINDER) {
        shape.dimensions[0] += 2.0 * fmax(fabs(diff.translation().x()), fabs(diff.translation().y()));
        shape.dimensions[1] += fabs(diff.translation().z());
      } else if(shape.type == shape_msgs::SolidPrimitive::SPHERE) {
        shape.dimensions[0] += 2.0 * fmax(fmax(fabs(diff.translation().x()), fabs(diff.translation().y())), fabs(diff.translation().z()));
      }
    }
    
    
    // we no longer need this instance, so we reset so that potential caching operations can avoid cloning the object
    obj.reset();
    
    geometry_msgs::Pose diff_pose_msg;
    Eigen::Affine3d diff_pose_adj = diff;
    diff_pose_adj.translation() /= 2.0;
    planning_models::msgFromPose(cur_pose*diff_pose_adj, diff_pose_msg);
    
    moveit_msgs::CollisionObject coll;
    coll.id = name;
    coll.primitive_poses.push_back(diff_pose_msg);
    coll.primitives.push_back(shape);
    
    addObject(coll);
    
    callUpdateCallback();
  }
}

void InteractiveObjectVisualization::shrinkObject(const std::string& name,
                                                  const geometry_msgs::Pose& new_pose_msg) 
{
  collision_detection::CollisionWorld::ObjectConstPtr obj = planning_scene_diff_->getCollisionWorld()->getObject(name);
  if(!obj) {
    ROS_WARN_STREAM("No object with name " << name);
    return;
  } 

  if (obj->shapes_[0]->type == shapes::SPHERE || obj->shapes_[0]->type == shapes::BOX || obj->shapes_[0]->type == shapes::CYLINDER)
  {
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
    
    shape_msgs::SolidPrimitive shape;
    shapes::ShapeMsg shape_variant;
    if (shapes::constructMsgFromShape(obj->shapes_[0].get(), shape_variant))
    {
      shape = boost::get<shape_msgs::SolidPrimitive>(shape_variant);

      if(shape.type == shape_msgs::SolidPrimitive::BOX) {
        diff.translation().x() = fmax(diff.translation().x(), -shape.dimensions[0]+MIN_DIMENSION);
        diff.translation().y() = fmax(diff.translation().y(), -shape.dimensions[1]+MIN_DIMENSION);
        diff.translation().z() = fmax(diff.translation().z(), -shape.dimensions[2]+MIN_DIMENSION);
        shape.dimensions[0] += diff.translation().x();
        shape.dimensions[1] += diff.translation().y();
        shape.dimensions[2] += diff.translation().z();
      } else if(shape.type == shape_msgs::SolidPrimitive::CYLINDER) {
        diff.translation().x() = fmax(diff.translation().x(), -shape.dimensions[0]+MIN_DIMENSION);
        diff.translation().y() = fmax(diff.translation().y(), -shape.dimensions[0]+MIN_DIMENSION);
        diff.translation().z() = fmax(diff.translation().z(), -shape.dimensions[1]+MIN_DIMENSION);
        shape.dimensions[0] += 2.0*fmin(diff.translation().x(), diff.translation().y());
        shape.dimensions[1] += diff.translation().z();
      } else if(shape.type == shape_msgs::SolidPrimitive::SPHERE) {
        diff.translation().x() = fmax(diff.translation().x(), -shape.dimensions[0]+MIN_DIMENSION);
        diff.translation().y() = fmax(diff.translation().y(), -shape.dimensions[0]+MIN_DIMENSION);
        diff.translation().z() = fmax(diff.translation().z(), -shape.dimensions[0]+MIN_DIMENSION);
        shape.dimensions[0] += 2.0*fmin(fmin(diff.translation().x(), diff.translation().y()), diff.translation().z());
      }
    }
    
    // we no longer need this instance, so we reset so that potential caching operations can avoid cloning the object
    obj.reset();
    
    geometry_msgs::Pose diff_pose_msg;
    Eigen::Affine3d diff_pose_adj = diff;
    diff_pose_adj.translation() /= 2.0;
    planning_models::msgFromPose(cur_pose*diff_pose_adj, diff_pose_msg);
    
    moveit_msgs::CollisionObject coll;
    coll.id = name;
    coll.primitive_poses.push_back(diff_pose_msg);
    coll.primitives.push_back(shape);
    
    addObject(coll);
    
    callUpdateCallback();
  }
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

void InteractiveObjectVisualization::attachObject(const std::string& name,
                                                  const std::string& link,
                                                  const std::vector<std::string>& touch_links)
{
  if(!planning_scene_diff_->getCollisionWorld()->hasObject(name)) {
    ROS_WARN_STREAM("No object " << name << " for attach");
    return;
  }
  moveit_msgs::AttachedCollisionObject att;
  planning_scene_diff_->getCollisionObjectMsg(name, att.object);
  att.link_name = link;
  att.touch_links = touch_links;
  planning_scene_diff_->processAttachedCollisionObjectMsg(att);
  callUpdateCallback();

  visualization_msgs::InteractiveMarker tm;
  bool already_have = interactive_marker_server_->get(name, tm);
  if(!already_have) {
    ROS_WARN_STREAM("Somehow don't have interactive marker");
    return;
  }

  std_msgs::ColorRGBA color_to_use;
  if(tm.controls.size() > 0 && tm.controls[0].markers.size() > 0) {
    if(tm.controls[0].markers[0].color.r != default_object_color_.r ||
       tm.controls[0].markers[0].color.g != default_object_color_.g ||
       tm.controls[0].markers[0].color.b != default_object_color_.b) {
      color_to_use = tm.controls[0].markers[0].color;
    }
  }
  dof_marker_enabled_.erase(name);
  object_menu_handlers_.erase(name);
  menu_name_to_handle_maps_.erase(name);
  menu_handle_to_function_maps_.erase(name);
  interactive_marker_server_->erase(name);
  interactive_marker_server_->applyChanges();
  visualization_msgs::InteractiveMarker marker; 
  makeInteractiveMarkerButton(att.object, color_to_use, marker, 1.001);
  recolorInteractiveMarker(marker, color_to_use);
  interactive_marker_server_->insert(marker);
  if(menu_name_to_handle_maps_[name].find("Detach object") == menu_name_to_handle_maps_[name].end()) {
    interactive_markers::MenuHandler::EntryHandle del_entry
      = object_menu_handlers_[name].insert("Detach object", 
                                           boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));
    menu_handle_to_function_maps_[name][del_entry] = boost::bind(&InteractiveObjectVisualization::detachObject, this, _1, link);
    menu_name_to_handle_maps_[name]["Detach object"] = del_entry;
  }
  object_menu_handlers_[name].apply(*interactive_marker_server_, name);
  interactive_marker_server_->applyChanges();
}

void InteractiveObjectVisualization::detachObject(const std::string& name,
                                                  const std::string& link) 
{
  visualization_msgs::InteractiveMarker tm;
  bool already_have = interactive_marker_server_->get(name, tm);
  if(!already_have) {
    ROS_WARN_STREAM("Somehow don't have interactive marker");
    return;
  }

  std_msgs::ColorRGBA color_to_use;
  if(tm.controls.size() > 0 && tm.controls[0].markers.size() > 0) {
    if(tm.controls[0].markers[0].color.r != default_object_color_.r ||
       tm.controls[0].markers[0].color.g != default_object_color_.g ||
       tm.controls[0].markers[0].color.b != default_object_color_.b) {
      color_to_use = tm.controls[0].markers[0].color;
    }
  }
  moveit_msgs::AttachedCollisionObject att;
  att.object.id = name;
  att.link_name = link;
  att.object.operation = moveit_msgs::CollisionObject::REMOVE;
  planning_scene_diff_->processAttachedCollisionObjectMsg(att);
  
  interactive_marker_server_->erase(name);
  interactive_marker_server_->applyChanges();

  dof_marker_enabled_.erase(name);
  object_menu_handlers_.erase(name);
  menu_name_to_handle_maps_.erase(name);
  menu_handle_to_function_maps_.erase(name);

  //should be back as object
  moveit_msgs::CollisionObject coll;
  if(planning_scene_diff_->getCollisionObjectMsg(name, coll)) {
    addObject(coll, color_to_use);
  } else {
    ROS_WARN_STREAM("Object disappeared");
  }
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

class ShapeVisitorAddToCollisionObject : public boost::static_visitor<void>
{
public:
  
  ShapeVisitorAddToCollisionObject(moveit_msgs::CollisionObject *obj) :
    boost::static_visitor<void>(), obj_(obj)
  {
  }

  void setPoseMessage(const geometry_msgs::Pose *pose)
  {
    pose_ = pose;
  }
    
  void operator()(const shape_msgs::Plane &shape_msg) const
  {
    obj_->planes.push_back(shape_msg);   
    obj_->plane_poses.push_back(*pose_);
  }
  
  void operator()(const shape_msgs::Mesh &shape_msg) const
  {
    obj_->meshes.push_back(shape_msg);
    obj_->mesh_poses.push_back(*pose_);
  }
  
  void operator()(const shape_msgs::SolidPrimitive &shape_msg) const
  {
    obj_->primitives.push_back(shape_msg);
    obj_->primitive_poses.push_back(*pose_);
  }
  
private:
  
  moveit_msgs::CollisionObject *obj_;
  const geometry_msgs::Pose *pose_;
};

void InteractiveObjectVisualization::processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) 
{    
  ROS_DEBUG_STREAM("Processing feedback for " << feedback->marker_name);
  switch (feedback->event_type) {
  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
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
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    {
      dof_marker_enabled_[feedback->marker_name] = !dof_marker_enabled_[feedback->marker_name];
      collision_detection::CollisionWorld::ObjectConstPtr obj = planning_scene_diff_->getCollisionWorld()->getObject(feedback->marker_name);
      shapes::ShapeMsg shape;
      shapes::constructMsgFromShape(obj->shapes_[0].get(), shape);
      geometry_msgs::Pose cur_pose_msg;
      planning_models::msgFromPose(obj->shape_poses_[0], cur_pose_msg);

      // we no longer need this instance, so we reset so that potential caching operations can avoid cloning the object
      obj.reset();
  
      moveit_msgs::CollisionObject coll;
      coll.id = feedback->marker_name;
      ShapeVisitorAddToCollisionObject sv(&coll);
      sv.setPoseMessage(&cur_pose_msg);
      boost::apply_visitor(sv, shape);
      
      addObject(coll);
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

void InteractiveObjectVisualization::updateOriginalPlanningScene(moveit_msgs::PlanningScenePtr& ptr) {
  //need to get rid of everything that's been added
  for(std::map<std::string, bool>::iterator it = dof_marker_enabled_.begin();
      it != dof_marker_enabled_.end();
      it++) {
    interactive_marker_server_->erase(it->first);
  }
  dof_marker_enabled_.clear();
  object_menu_handlers_.clear();
  menu_name_to_handle_maps_.clear();
  menu_handle_to_function_maps_.clear();
  interactive_marker_server_->applyChanges();
  planning_scene_diff_ = planning_scene_->diff();
  planning_scene_diff_->setPlanningSceneMsg(*ptr);
  for(unsigned int i = 0; i < ptr->world.collision_objects.size(); i++) {
    addObject(ptr->world.collision_objects[i]);
  }
  callUpdateCallback();
}

void InteractiveObjectVisualization::addMenuEntry(const std::string& menu_name,
                                                  const boost::function<void(const std::string&)>& callback)
{
  all_callback_map_[menu_name] = callback;
  for(std::map<std::string, interactive_markers::MenuHandler>::iterator it = object_menu_handlers_.begin();
      it != object_menu_handlers_.end();
      it++) {
    interactive_markers::MenuHandler::EntryHandle eh 
      = it->second.insert(menu_name,
                          boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));
    menu_name_to_handle_maps_[it->first][menu_name] = eh;
    menu_handle_to_function_maps_[it->first][eh] = callback;
    it->second.apply(*interactive_marker_server_, it->first);
  }
  interactive_marker_server_->applyChanges();
}

void InteractiveObjectVisualization::addMenuEntry(const std::string& object_name,
                                                  const std::string& menu_name,
                                                  const boost::function<void(const std::string&)>& callback)
{
  if(object_menu_handlers_.find(object_name) == object_menu_handlers_.end()) {
    ROS_WARN_STREAM("No object " << object_name << " for adding menu entry");
  }
  interactive_markers::MenuHandler& mh = object_menu_handlers_[object_name];
  interactive_markers::MenuHandler::EntryHandle eh = mh.insert(menu_name,
                                                               boost::bind(&InteractiveObjectVisualization::processInteractiveMenuFeedback, this, _1));
  menu_name_to_handle_maps_[object_name][menu_name] = eh;
  menu_handle_to_function_maps_[object_name][eh] = callback;
  mh.apply(*interactive_marker_server_, object_name);
  interactive_marker_server_->applyChanges();
}

void InteractiveObjectVisualization::makeInteractiveMarkerButton(const moveit_msgs::CollisionObject& coll,
                                                                 const std_msgs::ColorRGBA& color_to_use,
                                                                 visualization_msgs::InteractiveMarker& marker,
                                                                 double scale)
{
  if(coll.primitive_poses.size() > 1 && coll.primitives.size() == 1 && coll.meshes.empty()) {
    marker = makeButtonPointMass(coll.id,
                                 "/"+planning_scene_->getPlanningFrame(),
                                 coll.primitive_poses,
                                 color_to_use,
                                 coll.primitives[0].dimensions[0]*scale,
                                 false,
                                 false);
    ROS_INFO_STREAM("Made button mass");
  } else if(coll.primitives.size() + coll.meshes.size() > 1) {
    std::vector<shapes::ShapeMsg> shapes;
    std::vector<geometry_msgs::Pose> poses;
    for (std::size_t i = 0 ; i < coll.primitives.size() ; ++i)
    {
      shapes::ShapeMsg s = coll.primitives[i]; shapes.push_back(s);
      poses.push_back(coll.primitive_poses[i]);
    }
    for (std::size_t i = 0 ; i < coll.meshes.size() ; ++i)
    {
      shapes::ShapeMsg s = coll.meshes[i]; shapes.push_back(s);
      poses.push_back(coll.mesh_poses[i]);
    }
    marker = makeButtonCompoundShape(coll.id,
                                     "/"+planning_scene_->getPlanningFrame(),
                                     shapes,
                                     poses,
                                     color_to_use,
                                     1.0*scale,
                                     false,
                                     false);
    ROS_INFO_STREAM("Made compound object");
  } else {
    if (coll.meshes.empty())
    {
      const shape_msgs::SolidPrimitive& shape_msg = coll.primitives[0]; 
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = "/"+planning_scene_->getPlanningFrame();
      pose_stamped.pose = coll.primitive_poses[0];
      
      if(shape_msg.type == shape_msgs::SolidPrimitive::BOX) {
        marker = makeButtonBox(coll.id,
                               pose_stamped,
                               shape_msg.dimensions[0]*scale,
                               shape_msg.dimensions[1]*scale,
                               shape_msg.dimensions[2]*scale,
                               false, 
                               false);
      } else if(shape_msg.type == shape_msgs::SolidPrimitive::CYLINDER) { 
        marker = makeButtonCylinder(coll.id,
                                    pose_stamped,
                                    shape_msg.dimensions[0]*scale*2,
                                    shape_msg.dimensions[1]*scale,
                                    false, 
                                    false);
      } else if(shape_msg.type == shape_msgs::SolidPrimitive::SPHERE) {
        marker = makeButtonSphere(coll.id,
                                  pose_stamped,
                                  shape_msg.dimensions[0]*scale*2,
                                  false, 
                                  false);
      }
    }
    else  {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = "/"+planning_scene_->getPlanningFrame();
      pose_stamped.pose = coll.mesh_poses[0];
      marker = makeButtonMesh(coll.id,
                              coll.meshes[0],
                              pose_stamped,
                              color_to_use);
    }
  }
}

}

