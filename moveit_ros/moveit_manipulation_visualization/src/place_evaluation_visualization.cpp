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

#include <moveit_manipulation_visualization/place_evaluation_visualization.h>

namespace moveit_manipulation_visualization {

PlaceEvaluationVisualization::
PlaceEvaluationVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                             boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                             boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader>& kinematics_plugin_loader,
                             ros::Publisher& marker_publisher) :
  planning_scene_(planning_scene),
  marker_publisher_(marker_publisher)
{
  const boost::shared_ptr<const srdf::Model>& srdf = planning_scene->getSrdfModel();
  const std::vector<srdf::Model::Group>& srdf_groups = srdf->getGroups();

  kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_plugin_loader->getLoaderFunction();
  
  std::map<std::string, kinematics::KinematicsBasePtr> solver_map;
  for(unsigned int i = 0; i < srdf_groups.size(); i++) {
    if(srdf_groups[i].subgroups_.size() == 0 &&
       srdf_groups[i].chains_.size() == 1) {
      if(!kinematics_plugin_loader->isGroupKnown(srdf_groups[i].name_)) {
        ROS_WARN_STREAM("Really should have loader for " << srdf_groups[i].name_);
        continue;
      }
      const planning_models::KinematicModel::JointModelGroup* jmg
        = planning_scene_->getKinematicModel()->getJointModelGroup(srdf_groups[i].name_); 

      solver_map[srdf_groups[i].name_] = kinematics_allocator(jmg);
    }
  }

  place_evaluator_fast_.reset(new grasp_place_evaluation::PlaceEvaluatorFast(planning_scene_->getKinematicModel(),
                                                                             solver_map));
  
  joint_trajectory_visualization_.reset(new moveit_visualization_ros::JointTrajectoryVisualization(planning_scene,
                                                                                                   marker_publisher));
  
}

void PlaceEvaluationVisualization::resetPlaceExecutionInfo() {
  last_place_evaluation_info_.clear();
}

void PlaceEvaluationVisualization::evaluatePlaceLocations(const std::string& group_name,
                                                          const moveit_manipulation_msgs::PlaceGoal& goal,
                                                          const planning_models::KinematicState* seed_state,
                                                          const std::vector<geometry_msgs::PoseStamped>& place_locations)
{
  place_evaluator_fast_->testPlaceLocations(planning_scene_,
                                            seed_state,
                                            goal, 
                                            place_locations,
                                            last_place_evaluation_info_,
                                            true);
}

bool PlaceEvaluationVisualization::getEvaluatedPlace(unsigned int num,
                                                     grasp_place_evaluation::PlaceExecutionInfo& place) const {
  if(num >= last_place_evaluation_info_.size()) {
    return false;
  }
  place = last_place_evaluation_info_[num];
  return true;
}

void PlaceEvaluationVisualization::removeAllMarkers() {
  for(unsigned int i = 0; i < last_marker_array_.markers.size(); i++) {
    last_marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  marker_publisher_.publish(last_marker_array_);
  last_marker_array_.markers.clear();
}

void PlaceEvaluationVisualization::showPlacePose(unsigned int num,
                                                 bool show_place,
                                                 bool show_preplace,
                                                 bool show_retreat) {
  if(num >= last_place_evaluation_info_.size()) {
    return;
  }

  removeAllMarkers();
  
  std::vector<std::string> end_effector_links = 
    planning_scene_->getSemanticModel()->getGroupLinks(planning_scene_->getSemanticModel()->getEndEffector(last_place_evaluation_info_.place_goal_.arm_name));
  
  planning_models::KinematicState state(planning_scene_->getCurrentState());
  
  if(show_place) {
    state.setStateValues(last_place_evaluation_info_.place_goal_.grasp.grasp_posture);
    state.updateStateWithLinkAt(planning_scene_->getSemanticModel()->getTipLink(last_place_evaluation_info_.place_goal_.arm_name),
                                last_place_evaluation_info_[num].place_pose_);

    std_msgs::ColorRGBA col;
    col.g = col.a = 1.0;
    
    state.getRobotMarkers(col,
                          "place",
                          ros::Duration(0.0),
                          last_marker_array_,
                          end_effector_links);
  }
  if(show_preplace) {
    state.setStateValues(last_place_evaluation_info_.place_goal_.grasp.pre_grasp_posture);
    state.updateStateWithLinkAt(planning_scene_->getSemanticModel()->getTipLink(last_place_evaluation_info_.place_goal_.arm_name),
                                last_place_evaluation_info_[num].preplace_pose_);

    std_msgs::ColorRGBA col;
    col.b = col.a = 1.0;
    
    state.getRobotMarkers(col,
                          "preplace",
                          ros::Duration(0.0),
                          last_marker_array_,
                          end_effector_links);
  }

  if(show_retreat) {
    
    planning_models::KinematicState diff_state(last_place_evaluation_info_[num].detached_object_diff_scene_->getCurrentState());    

    diff_state.setStateValues(last_place_evaluation_info_.place_goal_.grasp.grasp_posture);

    diff_state.updateStateWithLinkAt(planning_scene_->getSemanticModel()->getTipLink(last_place_evaluation_info_.place_goal_.arm_name),
                                     last_place_evaluation_info_[num].retreat_pose_);

    std_msgs::ColorRGBA col;
    col.b = col.g = col.a = 1.0;
    
    diff_state.getRobotMarkers(col,
                               "retreat",
                               ros::Duration(0.0),
                               last_marker_array_,
                               end_effector_links);
  }

  marker_publisher_.publish(last_marker_array_);
}

void PlaceEvaluationVisualization::playInterpolatedTrajectories(unsigned int num,
                                                                bool play_approach,
                                                                bool play_retreat,
                                                                bool in_thread)
{
  if(num >= last_place_evaluation_info_.size()) {
    return;
  }

  if(last_place_evaluation_info_[num].result_.result_code != moveit_manipulation_msgs::PlaceLocationResult::SUCCESS) {
    return;
  }
  
  if(in_thread) {
    boost::thread(boost::bind(&PlaceEvaluationVisualization::playInterpolatedTrajectoriesThread, this, num, play_approach, play_retreat));
  } else {
    removeAllMarkers();
    playInterpolatedTrajectoriesThread(num, play_approach, play_retreat);
    showPlacePose(num, true, true, true);
  }
}

void PlaceEvaluationVisualization::playInterpolatedTrajectoriesThread(unsigned int num,
                                                                      bool play_approach,
                                                                      bool play_retreat)
{

  std_msgs::ColorRGBA col;
  col.b = col.r = col.a = 1.0;

  if(play_approach) {
    joint_trajectory_visualization_->updatePlanningScene(planning_scene_);
    planning_models::KinematicState state(planning_scene_->getCurrentState());
    state.setStateValues(last_place_evaluation_info_.place_goal_.grasp.grasp_posture);
    joint_trajectory_visualization_->setTrajectory(state,
                                                   last_place_evaluation_info_.place_goal_.arm_name,
                                                   last_place_evaluation_info_[num].approach_trajectory_,
                                                   col);
    if(play_retreat) {
      joint_trajectory_visualization_->playCurrentTrajectory(true);
    } else {
      joint_trajectory_visualization_->playCurrentTrajectory();
    }
  } 
  if(play_retreat) {
    last_place_evaluation_info_[num].detached_object_diff_scene_->getCurrentState().setStateValues(last_place_evaluation_info_.place_goal_.grasp.pre_grasp_posture);

    joint_trajectory_visualization_->updatePlanningScene(last_place_evaluation_info_[num].detached_object_diff_scene_);
    joint_trajectory_visualization_->setTrajectory(last_place_evaluation_info_[num].detached_object_diff_scene_->getCurrentState(),
                                                   last_place_evaluation_info_.place_goal_.arm_name,
                                                   last_place_evaluation_info_[num].retreat_trajectory_,
                                                   col);
    joint_trajectory_visualization_->playCurrentTrajectory(true);
  }
}

}
