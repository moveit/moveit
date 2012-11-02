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
PlaceEvaluationVisualization(ros::Publisher& marker_publisher) :
  marker_publisher_(marker_publisher)
{
}

void PlaceEvaluationVisualization::removeAllMarkers() {
  for(unsigned int i = 0; i < last_marker_array_.markers.size(); i++) {
    last_marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  marker_publisher_.publish(last_marker_array_);
  last_marker_array_.markers.clear();
}

void PlaceEvaluationVisualization::hideAllMarkers() {
  if(!last_marker_array_.markers.empty()) {
    saved_marker_array_ = last_marker_array_;
    removeAllMarkers();
  }
  //otherwise there's nothing to save or we'd previously hidden
}

void PlaceEvaluationVisualization::showHiddenMarkers() {
  last_marker_array_ = saved_marker_array_;
  for(unsigned int i = 0; i < last_marker_array_.markers.size(); i++) {
    last_marker_array_.markers[i].header.stamp = ros::Time::now();
  }
  marker_publisher_.publish(last_marker_array_);
}

void PlaceEvaluationVisualization::showPlacePose(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                 const grasp_place_evaluation::PlaceExecutionInfoVector& place_info,
                                                 unsigned int num,
                                                 bool show_place,
                                                 bool show_preplace,
                                                 bool show_retreat) {
  if(num >= place_info.size()) {
    return;
  }

  removeAllMarkers();

  std::vector<std::string> end_effector_links = planning_scene->getKinematicModel()->getJointModelGroup(planning_scene->getKinematicModel()->getJointModelGroup(place_info.place_goal_.arm_name)->getAttachedEndEffectorGroupName())->getLinkModelNames();
  
  planning_models::KinematicState state(planning_scene->getCurrentState());
  
  if(show_place) {
    state.setStateValues(place_info.place_goal_.grasp.grasp_posture);
    state.updateStateWithLinkAt(planning_scene->getKinematicModel()->getJointModelGroup(place_info.place_goal_.arm_name)->getLinkModelNames().back(),
                                place_info[num].place_pose_);

    std_msgs::ColorRGBA col;
    col.g = col.a = 1.0;
    
    state.getRobotMarkers(col,
                          "place",
                          ros::Duration(0.0),
                          last_marker_array_,
                          end_effector_links);
  }
  if(show_preplace) {
    state.setStateValues(place_info.place_goal_.grasp.grasp_posture);
    state.updateStateWithLinkAt(planning_scene->getKinematicModel()->getJointModelGroup(place_info.place_goal_.arm_name)->getLinkModelNames().back(),
                                place_info[num].preplace_pose_);

    std_msgs::ColorRGBA col;
    col.b = col.a = 1.0;
    
    state.getRobotMarkers(col,
                          "preplace",
                          ros::Duration(0.0),
                          last_marker_array_,
                          end_effector_links);
  }

  if(show_retreat && place_info[num].detached_object_diff_scene_) {
    
    planning_models::KinematicState diff_state(place_info[num].detached_object_diff_scene_->getCurrentState());    
    diff_state.setStateValues(place_info.place_goal_.grasp.pre_grasp_posture);

    diff_state.updateStateWithLinkAt(planning_scene->getKinematicModel()->getJointModelGroup(place_info.place_goal_.arm_name)->getLinkModelNames().back(),
                                     place_info[num].retreat_pose_);

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

void PlaceEvaluationVisualization::playInterpolatedTrajectories(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                                const grasp_place_evaluation::PlaceExecutionInfoVector& place_info,
                                                                boost::shared_ptr<moveit_visualization_ros::JointTrajectoryVisualization> joint_trajectory_visualization,
                                                                unsigned int num,
                                                                bool play_approach,
                                                                bool play_retreat,
                                                                bool in_thread,
                                                                bool hide_markers)
{
  if(num >= place_info.size()) {
    return;
  }

  if(place_info[num].result_.result_code != moveit_manipulation_msgs::PlaceLocationResult::SUCCESS) {
    return;
  }
  
  if(in_thread) {
    boost::thread(boost::bind(&PlaceEvaluationVisualization::playInterpolatedTrajectoriesThread, this, planning_scene, place_info, joint_trajectory_visualization, num, play_approach, play_retreat, hide_markers));
  } else {
    playInterpolatedTrajectoriesThread(planning_scene, place_info, joint_trajectory_visualization, num, play_approach, play_retreat, hide_markers);
  }
}

void PlaceEvaluationVisualization::playInterpolatedTrajectoriesThread(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                                      const grasp_place_evaluation::PlaceExecutionInfoVector& place_info,
                                                                      boost::shared_ptr<moveit_visualization_ros::JointTrajectoryVisualization> joint_trajectory_visualization,
                                                                      unsigned int num,
                                                                      bool play_approach,
                                                                      bool play_retreat,
                                                                      bool hide_markers)
{
  std_msgs::ColorRGBA col;
  col.b = col.r = col.a = 1.0;

  if(hide_markers) {
    hideAllMarkers();
  }

  if(play_approach) {
    joint_trajectory_visualization->updatePlanningScene(planning_scene);
    planning_models::KinematicState state(planning_scene->getCurrentState());
    state.setStateValues(place_info.place_goal_.grasp.grasp_posture);
    joint_trajectory_visualization->setTrajectory(state,
                                                   place_info.place_goal_.arm_name,
                                                   place_info[num].approach_trajectory_,
                                                   col);
    joint_trajectory_visualization->playCurrentTrajectory(true);
  } 
  if(play_retreat && place_info[num].detached_object_diff_scene_) {
    place_info[num].detached_object_diff_scene_->getCurrentState().setStateValues(place_info.place_goal_.grasp.pre_grasp_posture);

    
    // visualization_msgs::MarkerArray obj_array;
    // place_info[num].detached_object_diff_scene_->getCollisionObjectMarkers(obj_array,
    //                                                                        col,
    //                                                                        "detached_object",
    //                                                                        ros::Duration(0.0));
    // if(obj_array.markers.size() == 0) {
    //   ROS_WARN_STREAM("No collision object markers");
    // } else {
    //   vis_marker_array_publisher_.publish(obj_array);
    //   last_marker_array_.push_back(obj_array.markers[0]);
    // }
    joint_trajectory_visualization->updatePlanningScene(place_info[num].detached_object_diff_scene_);
    joint_trajectory_visualization->setTrajectory(place_info[num].detached_object_diff_scene_->getCurrentState(),
                                                  place_info.place_goal_.arm_name,
                                                  place_info[num].retreat_trajectory_,
                                                  col);
    joint_trajectory_visualization->playCurrentTrajectory(true);
  }
  if(hide_markers) {
    showHiddenMarkers();
  }
}

}
