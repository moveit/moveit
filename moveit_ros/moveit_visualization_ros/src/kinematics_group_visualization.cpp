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

#include <moveit_visualization_ros/kinematics_group_visualization.h>
#include <moveit_visualization_ros/interactive_marker_helper_functions.h>
#include <collision_detection/collision_tools.h>

namespace moveit_visualization_ros
{

KinematicsGroupVisualization::KinematicsGroupVisualization(planning_scene::PlanningSceneConstPtr planning_scene, 
                                                           boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server, 
                                                           const std::string& group_name, 
                                                           const std::string& suffix_name,
                                                           const std::string& kinematics_solver_name,
                                                           const std_msgs::ColorRGBA& good_color,
                                                           const std_msgs::ColorRGBA& bad_color,
                                                           ros::Publisher& marker_publisher) :
  group_name_(group_name),
  suffix_name_(suffix_name),
  interactive_marker_name_(group_name+"_interactive_kinematics_"+suffix_name),
  regular_marker_name_(group_name+"_kinematics_"+suffix_name),
  last_solution_good_(true),
  last_solution_changed_(false),
  good_color_(good_color),
  bad_color_(bad_color),
  planning_scene_(planning_scene),
  interactive_marker_server_(interactive_marker_server),
  state_(planning_scene_->getCurrentState()),
  marker_publisher_(marker_publisher),
  dof_marker_enabled_(false)
{
  const std::map<std::string, srdf::Model::Group>& group_map 
    = planning_scene_->getKinematicModel()->getJointModelGroupConfigMap();
  
  kinematics_loader_.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("kinematics_base", "kinematics::KinematicsBase"));    
  
  if(group_map.find(group_name) == group_map.end()) {
    ROS_ERROR_STREAM("No group named " << group_name);
    return;
  }
  
  const srdf::Model::Group& srdf_group = group_map.find(group_name)->second;
  
  if(srdf_group.chains_.size() == 0 ||
     srdf_group.chains_[0].first.empty() ||
     srdf_group.chains_[0].second.empty()) {
    ROS_ERROR_STREAM("Group name " << group_name << " has no or messed up chain definition");
    return;
  }

  kinematics::KinematicsBasePtr result;
  result.reset(kinematics_loader_->createClassInstance(kinematics_solver_name));
  
  ik_solver_.reset(new kinematics_constraint_aware::KinematicsSolverConstraintAware(result,
                                                                                    planning_scene_->getKinematicModel(),
                                                                                    group_name));

  const planning_models::KinematicModel::LinkModel* lm = state_.getKinematicModel()->getLinkModel(ik_solver_->getTipFrame());
  if(lm == NULL) {
    ROS_ERROR_STREAM("No link for tip frame " << ik_solver_->getTipFrame());
  }
  end_effector_link_names_ = state_.getKinematicModel()->getChildLinkModelNames(lm);
  //children will include tip link
  if(!end_effector_link_names_.size() > 1) {
    end_effector_link_names_.erase(end_effector_link_names_.begin());
  }
  enable6DOFControls();
  default_menu_handler_.apply(*interactive_marker_server_, interactive_marker_name_);
  interactive_marker_server_->applyChanges();
};

void KinematicsGroupVisualization::hideAllMarkers() {
  removeLastMarkers();
}

void KinematicsGroupVisualization::showAllMarkers() {
  for(unsigned int i = 0; i < last_marker_array_.markers.size(); i++) {
    last_marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
    last_marker_array_.markers[i].color.a = 1.0;
  }
}

void KinematicsGroupVisualization::setMarkerAlpha(double a) {
  for(unsigned int i = 0; i < last_marker_array_.markers.size(); i++) {
    last_marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
    last_marker_array_.markers[i].header.stamp = ros::Time::now();
    last_marker_array_.markers[i].color.a = a;
  }
  marker_publisher_.publish(last_marker_array_);
  std_msgs::ColorRGBA good_color = good_color_;
  good_color.a = a;
  makeInteractiveControlMarker(interactive_marker_name_,
                               good_color,
                               dof_marker_enabled_);
}

void KinematicsGroupVisualization::disable6DOFControls() {
  dof_marker_enabled_ = false;
  makeInteractiveControlMarker(interactive_marker_name_,
                               good_color_,
                               false);
}

void KinematicsGroupVisualization::enable6DOFControls() {
  dof_marker_enabled_ = true;
  makeInteractiveControlMarker(interactive_marker_name_,
                               good_color_,
                               true);
}

void KinematicsGroupVisualization::addButtonClickCallback(const boost::function<void(void)>& button_click_callback) {
  button_click_callback_ = button_click_callback;
}

void KinematicsGroupVisualization::addMenuEntry(const std::string& name, 
                                               const boost::function<void(void)>& callback) {
  interactive_markers::MenuHandler::EntryHandle eh
    = default_menu_handler_.insert(name, 
                                   boost::bind(&KinematicsGroupVisualization::processInteractiveMenuFeedback, this, _1));

  menu_handle_to_string_map_[eh] = name;
  default_callback_map_[name] = callback;
  default_menu_handler_.reApply(*interactive_marker_server_);
}

void KinematicsGroupVisualization::updatePlanningScene(planning_scene::PlanningSceneConstPtr planning_scene) {
  planning_scene_ = planning_scene;
  updateEndEffectorState(last_pose_);
}

void KinematicsGroupVisualization::processInteractiveMenuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if(feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
    ROS_WARN_STREAM("Got something other than menu select on menu feedback function");
    return;
  }
  if(menu_handle_to_string_map_.find(feedback->menu_entry_id) ==
     menu_handle_to_string_map_.end()) {
    ROS_WARN_STREAM("No handle found " << feedback->menu_entry_id);
    return;
  }
  std::string name = menu_handle_to_string_map_[feedback->menu_entry_id];
  if(default_callback_map_.find(name) == default_callback_map_.end()) {
    ROS_WARN_STREAM("No callback associated with name " << name);
    return;
  }
  default_callback_map_[name]();
}

void KinematicsGroupVisualization::removeLastMarkers() 
{
  if(last_marker_array_.markers.empty()) return;

  for(unsigned int i = 0; i < last_marker_array_.markers.size(); i++) {
    last_marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  marker_publisher_.publish(last_marker_array_);
}

void KinematicsGroupVisualization::sendCurrentMarkers()
{
  if(last_solution_good_) {
    std_msgs::ColorRGBA col;
    col = good_color_;
    state_.getRobotMarkers(col,
                           regular_marker_name_,
                           ros::Duration(0.0),
                           last_marker_array_,
                           ik_solver_->getLinkNames());

    marker_publisher_.publish(last_marker_array_);
  } else {
    removeLastMarkers();
    last_marker_array_.markers.clear();
    collision_detection::getCollisionMarkersFromContacts(last_marker_array_,
                                                         planning_scene_->getKinematicModel()->getModelFrame(),
                                                         ik_solver_->getLastInitialPoseCheckCollisionResult().contacts,
                                                         bad_color_,
                                                         ros::Duration(0.0));
    marker_publisher_.publish(last_marker_array_);
  }
}

void KinematicsGroupVisualization::updateEndEffectorState(const geometry_msgs::Pose& pose) {
  ros::WallTime start = ros::WallTime::now();

  last_pose_ = pose;
    
  //assuming pose is in world frame for now
  Eigen::Affine3d cur;
  planning_models::poseFromMsg(pose, cur);
  
  state_.updateStateWithLinkAt(ik_solver_->getTipFrame(), cur);
  
  //now need to get in base_frame
  Eigen::Affine3d base_in_world = state_.getLinkState(ik_solver_->getBaseFrame())->getGlobalLinkTransform();
  
  ROS_DEBUG_STREAM("Base x y z " << base_in_world.translation().x() << " " 
                   << base_in_world.translation().y() << " " 
                   << base_in_world.translation().z()); 

  Eigen::Quaterniond quat(base_in_world.rotation());

  ROS_DEBUG_STREAM("Base rot x y z w " << quat.x() << " " 
                   << quat.y() << " " 
                   << quat.z() << " " 
                   << quat.w()); 
  
  Eigen::Affine3d tip_in_base = base_in_world.inverse()*cur;

  quat = Eigen::Quaterniond(tip_in_base.rotation());

  ROS_DEBUG_STREAM("tip rot x y z w " << quat.x() << " " 
                   << quat.y() << " " 
                   << quat.z() << " " 
                   << quat.w()); 
  
  geometry_msgs::Pose np;
  planning_models::msgFromPose(tip_in_base, np);
  
  ROS_DEBUG_STREAM("X Y Z are " << np.position.x << " " 
                   << np.position.y << " " 
                   << np.position.z << " from " << ik_solver_->getBaseFrame()); 
  
  sensor_msgs::JointState sol;
  moveit_msgs::MoveItErrorCodes err;
  
  visualization_msgs::MarkerArray arr;
  moveit_msgs::Constraints emp_constraints;
  if(ik_solver_->findConstraintAwareSolution(np,
                                             emp_constraints,
                                             planning_scene_,
                                             sol,
                                             err, 
                                             true)) {
    if(last_solution_good_ == false) {
      last_solution_changed_ = true;
    } else {
      last_solution_changed_ = false;
    }
    last_solution_good_ = true;
    state_.setStateValues(sol);
  } else {
    if(last_solution_good_ == true) {
      last_solution_changed_ = true;
    } else {
      last_solution_changed_ = false;
    }
    last_solution_good_ = false; 
    ROS_INFO_STREAM("IK not ok " << err.val);
  }
  sendCurrentMarkers();
  // if(last_solution_changed_) {
  //   std_msgs::ColorRGBA col;
  //   if(last_solution_good_) {
  //     col = good_color_;
  //   } else {
  //     col = bad_color_;
  //   }
  //   makeInteractiveControlMarker(interactive_marker_name_,
  //                                state_.getLinkState(ik_solver_->getTipFrame())->getGlobalLinkTransform(),
  //                                col);
  // }
  ROS_INFO_STREAM("Total time is " << (ros::WallTime::now()-start));
}

void KinematicsGroupVisualization::processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) 
{    
  switch (feedback->event_type) {
  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    updateEndEffectorState(feedback->pose);
    break;
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    if(button_click_callback_) {
      button_click_callback_();
    }
    break;
  default:
    ROS_DEBUG_STREAM("Getting event type " << feedback->event_type);
  }
  interactive_marker_server_->applyChanges();
}; 

void KinematicsGroupVisualization::makeInteractiveControlMarker(const std::string& name,
                                                                const std_msgs::ColorRGBA& color,
                                                                bool add_6dof)
{
  visualization_msgs::InteractiveMarker marker = makeMeshButtonFromLinks(name,
                                                                         state_,
                                                                         end_effector_link_names_,
                                                                         color,
                                                                         true);
  if(add_6dof) {
    add6DofControl(marker, false);
  }
  marker.description=group_name_+"_"+suffix_name_;

  // make6DOFMarker(group_name_+"_ik",
  //                ps,
  //                .3,
  //                false,
  //                false);
  interactive_marker_server_->insert(marker);
  interactive_marker_server_->setCallback(marker.name, 
                                          boost::bind(&KinematicsGroupVisualization::processInteractiveMarkerFeedback, this, _1));
  interactive_marker_server_->applyChanges();
};

}


