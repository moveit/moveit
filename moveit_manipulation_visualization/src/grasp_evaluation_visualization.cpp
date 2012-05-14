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

#include <moveit_manipulation_visualization/grasp_evaluation_visualization.h>
#include <kinematics_plugin_loader/kinematics_plugin_loader_helpers.h>

namespace moveit_manipulation_visualization {

GraspEvaluationVisualization::
GraspEvaluationVisualization(ros::Publisher& marker_publisher) :
  marker_publisher_(marker_publisher)
{  
}

void GraspEvaluationVisualization::removeAllMarkers() {
  for(unsigned int i = 0; i < last_marker_array_.markers.size(); i++) {
    last_marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  marker_publisher_.publish(last_marker_array_);
  last_marker_array_.markers.clear();
}

void GraspEvaluationVisualization::showGraspPose(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                 const grasp_place_evaluation::GraspExecutionInfoVector& grasp_info,
                                                 unsigned int num,
                                                 bool show_grasp,
                                                 bool show_pregrasp,
                                                 bool show_lift) {
  if(num >= grasp_info.size()) {
    return;
  }

  removeAllMarkers();

  std::vector<std::string> end_effector_links = 
    planning_scene->getSemanticModel()->getGroupLinks(planning_scene->getSemanticModel()->getEndEffector(grasp_info.pickup_goal_.arm_name));
  
  planning_models::KinematicState state(planning_scene->getCurrentState());

  if(show_grasp) {
    state.setStateValues(grasp_info.grasps_[num].grasp_posture);
    state.updateStateWithLinkAt(planning_scene->getSemanticModel()->getTipLink(grasp_info.pickup_goal_.arm_name),
                                grasp_info[num].grasp_pose_);

    std_msgs::ColorRGBA col;
    col.g = col.a = 1.0;
    
    state.getRobotMarkers(col,
                          "grasp",
                          ros::Duration(0.0),
                          last_marker_array_,
                          end_effector_links);
  }
  if(show_pregrasp) {
    state.setStateValues(grasp_info.grasps_[num].pre_grasp_posture);
    state.updateStateWithLinkAt(planning_scene->getSemanticModel()->getTipLink(grasp_info.pickup_goal_.arm_name),
                                grasp_info[num].pregrasp_pose_);

    std_msgs::ColorRGBA col;
    col.b = col.a = 1.0;
    
    state.getRobotMarkers(col,
                          "pregrasp",
                          ros::Duration(0.0),
                          last_marker_array_,
                          end_effector_links);
  }

  if(show_lift && grasp_info[num].attached_object_diff_scene_) {

    planning_models::KinematicState diff_state(grasp_info[num].attached_object_diff_scene_->getCurrentState());    
    diff_state.setStateValues(grasp_info.grasps_[num].grasp_posture);

    diff_state.updateStateWithLinkAt(planning_scene->getSemanticModel()->getTipLink(grasp_info.pickup_goal_.arm_name),
                                     grasp_info[num].lift_pose_);

    std_msgs::ColorRGBA col;
    col.b = col.g = col.a = 1.0;
    
    diff_state.getRobotMarkers(col,
                          "lift",
                          ros::Duration(0.0),
                          last_marker_array_,
                          end_effector_links);
  }

  marker_publisher_.publish(last_marker_array_);
}

void GraspEvaluationVisualization::playInterpolatedTrajectories(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                                const grasp_place_evaluation::GraspExecutionInfoVector& grasp_info,
                                                                boost::shared_ptr<moveit_visualization_ros::JointTrajectoryVisualization> joint_trajectory_visualization,
                                                                unsigned int num,
                                                                bool play_approach,
                                                                bool play_lift,
                                                                bool in_thread) {
  if(num >= grasp_info.size()) {
    return;
  }

  if(grasp_info[num].result_.result_code != moveit_manipulation_msgs::GraspResult::SUCCESS) {
    return;
  }

  removeAllMarkers();
  
  if(in_thread) {
    boost::thread(boost::bind(&GraspEvaluationVisualization::playInterpolatedTrajectoriesThread, this, planning_scene, grasp_info, 
                              joint_trajectory_visualization, num, play_approach, play_lift));
  } else {
    removeAllMarkers();
    playInterpolatedTrajectoriesThread(planning_scene, grasp_info, joint_trajectory_visualization, num, play_approach, play_lift);
    //showGraspPose(planning_scene, grasp_info, num, true, true, true);
  }
}

void GraspEvaluationVisualization::playInterpolatedTrajectoriesThread(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                                      const grasp_place_evaluation::GraspExecutionInfoVector& grasp_info,
                                                                      boost::shared_ptr<moveit_visualization_ros::JointTrajectoryVisualization> joint_trajectory_visualization,
                                                                      unsigned int num,
                                                                      bool play_approach,
                                                                      bool play_lift)
{

  std_msgs::ColorRGBA col;
  col.b = col.r = col.a = 1.0;

  if(play_approach) {
    joint_trajectory_visualization->updatePlanningScene(planning_scene);
    planning_models::KinematicState state(planning_scene->getCurrentState());
    state.setStateValues(grasp_info.grasps_[num].pre_grasp_posture);
    if(grasp_info[num].approach_trajectory_.points.size() == 0) {
      ROS_WARN_STREAM("No points in approach trajectory");
    } else {
      joint_trajectory_visualization->setTrajectory(state,
                                                     grasp_info.pickup_goal_.arm_name,
                                                     grasp_info[num].approach_trajectory_,
                                                     col);
      if(play_lift) {
        joint_trajectory_visualization->playCurrentTrajectory(true);
      } else {
        joint_trajectory_visualization->playCurrentTrajectory();
      }
    }
  } 
  if(play_lift && grasp_info[num].attached_object_diff_scene_) {
    grasp_info[num].attached_object_diff_scene_->getCurrentState().setStateValues(grasp_info.grasps_[num].grasp_posture);

    if(grasp_info[num].lift_trajectory_.points.size() == 0) {
      ROS_WARN_STREAM("No points in lift trajectory");
    } else {
      
      joint_trajectory_visualization->updatePlanningScene(grasp_info[num].attached_object_diff_scene_);
      joint_trajectory_visualization->setTrajectory(grasp_info[num].attached_object_diff_scene_->getCurrentState(),
                                                     grasp_info.pickup_goal_.arm_name,
                                                     grasp_info[num].lift_trajectory_,
                                                     col);
      joint_trajectory_visualization->playCurrentTrajectory(true);
    }
  }
}

}
