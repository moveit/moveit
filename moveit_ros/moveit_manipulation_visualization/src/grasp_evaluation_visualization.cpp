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

namespace moveit_manipulation_visualization {

GraspEvaluationVisualization::
GraspEvaluationVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
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

  grasp_evaluator_fast_.reset(new grasp_place_evaluation::GraspEvaluatorFast(planning_scene_->getKinematicModel(),
                                                                             solver_map));

  joint_trajectory_visualization_.reset(new moveit_visualization_ros::JointTrajectoryVisualization(planning_scene,
                                                                                                   marker_publisher));
  
}

void GraspEvaluationVisualization::evaluateGrasps(const std::string& group_name,
                                                  const moveit_manipulation_msgs::PickupGoal& goal,
                                                  const planning_models::KinematicState* seed_state,
                                                  const std::vector<moveit_manipulation_msgs::Grasp>& grasps)
{
  grasp_evaluator_fast_->testGrasps(planning_scene_,
                                    seed_state,
                                    goal, 
                                    grasps,
                                    last_grasp_evaluation_info_,
                                    true);
  //showGraspPose(0, true, true, true);
  playInterpolationTrajectories(0,
                                true,
                                true);
}

void GraspEvaluationVisualization::showGraspPose(unsigned int num,
                                                 bool show_grasp,
                                                 bool show_pregrasp,
                                                 bool show_lift) {
  if(num >= last_grasp_evaluation_info_.size()) {
    return;
  }

  std::vector<std::string> end_effector_links;
  grasp_evaluator_fast_->getGroupLinks(grasp_evaluator_fast_->getEndEffectorName(planning_scene_->getSrdfModel(),
                                                                                 last_grasp_evaluation_info_.pickup_goal_.arm_name),
                                       end_effector_links);
  
  planning_models::KinematicState state(planning_scene_->getCurrentState());

  visualization_msgs::MarkerArray arr;
  
  if(show_grasp) {
    state.setStateValues(last_grasp_evaluation_info_.grasps_[num].grasp_posture);
    state.updateStateWithLinkAt(grasp_evaluator_fast_->getTipLink(last_grasp_evaluation_info_.pickup_goal_.arm_name),
                                last_grasp_evaluation_info_[num].grasp_pose_);

    std_msgs::ColorRGBA col;
    col.g = col.a = 1.0;
    
    state.getRobotMarkers(col,
                          "grasp",
                          ros::Duration(0.0),
                          arr,
                          end_effector_links);
  }
  if(show_pregrasp) {
    state.setStateValues(last_grasp_evaluation_info_.grasps_[num].pre_grasp_posture);
    state.updateStateWithLinkAt(grasp_evaluator_fast_->getTipLink(last_grasp_evaluation_info_.pickup_goal_.arm_name),
                                last_grasp_evaluation_info_[num].pregrasp_pose_);

    std_msgs::ColorRGBA col;
    col.b = col.a = 1.0;
    
    state.getRobotMarkers(col,
                          "pregrasp",
                          ros::Duration(0.0),
                          arr,
                          end_effector_links);
  }

  if(show_lift) {
    state.setStateValues(last_grasp_evaluation_info_.grasps_[num].grasp_posture);

    state.updateStateWithLinkAt(grasp_evaluator_fast_->getTipLink(last_grasp_evaluation_info_.pickup_goal_.arm_name),
                                last_grasp_evaluation_info_[num].lift_pose_);

    std_msgs::ColorRGBA col;
    col.b = col.g = col.a = 1.0;
    
    state.getRobotMarkers(col,
                          "lift",
                          ros::Duration(0.0),
                          arr,
                          end_effector_links);
  }

  marker_publisher_.publish(arr);
}

void GraspEvaluationVisualization::playInterpolationTrajectories(unsigned int num,
                                                                 bool play_approach,
                                                                 bool play_lift) {
  if(num >= last_grasp_evaluation_info_.size()) {
    return;
  }

  if(last_grasp_evaluation_info_[num].result_.result_code != moveit_manipulation_msgs::GraspResult::SUCCESS) {
    return;
  }

  std_msgs::ColorRGBA col;
  col.b = col.r = col.a = 1.0;

  if(play_approach) {
    joint_trajectory_visualization_->updatePlanningScene(planning_scene_);
    planning_models::KinematicState state(planning_scene_->getCurrentState());
    state.setStateValues(last_grasp_evaluation_info_.grasps_[num].pre_grasp_posture);
    joint_trajectory_visualization_->setTrajectory(state,
                                                   last_grasp_evaluation_info_.pickup_goal_.arm_name,
                                                   last_grasp_evaluation_info_[num].approach_trajectory_,
                                                   col);
    if(play_lift) {
      joint_trajectory_visualization_->playCurrentTrajectory(true);
    } else {
      joint_trajectory_visualization_->playCurrentTrajectory();
    }
  } 
  if(play_lift) {
    
    // boost::shared_ptr<planning_scene::PlanningScene> attached_object_diff_scene(new planning_scene::PlanningScene(planning_scene_));
    // planning_models::KinematicState state(attached_object_diff_scene->getCurrentState());
    // std::map<std::string, double> grasp_values;
    // for(unsigned int i = 0; i < last_grasp_evaluation_info_[num].lift_trajectory_.joint_names.size(); i++) {
    //   grasp_values[last_grasp_evaluation_info_[num].lift_trajectory_.joint_names[i]] 
    //     = last_grasp_evaluation_info_[num].lift_trajectory_.points.front().positions[i];
    // }
    // state.setStateValues(grasp_values);
    // attached_object_diff_scene->setCurrentState(state);
    
    moveit_msgs::AttachedCollisionObject att_obj;
    att_obj.link_name = grasp_evaluator_fast_->getAttachLink(grasp_evaluator_fast_->getEndEffectorName(planning_scene_->getSrdfModel(),
                                                                                                       last_grasp_evaluation_info_.pickup_goal_.arm_name));
    
    att_obj.object.operation = moveit_msgs::CollisionObject::ADD;
    att_obj.object.id = last_grasp_evaluation_info_.pickup_goal_.collision_object_name;
    grasp_evaluator_fast_->getGroupLinks(grasp_evaluator_fast_->getEndEffectorName(planning_scene_->getSrdfModel(),
                                                                                   last_grasp_evaluation_info_.pickup_goal_.arm_name),
                                         att_obj.touch_links);
    //attached_object_diff_scene->processAttachedCollisionObjectMsg(att_obj);

    last_grasp_evaluation_info_[num].attached_object_diff_scene_->getCurrentState().setStateValues(last_grasp_evaluation_info_.grasps_[num].pre_grasp_posture);

    joint_trajectory_visualization_->updatePlanningScene(last_grasp_evaluation_info_[num].attached_object_diff_scene_);
    joint_trajectory_visualization_->setTrajectory(last_grasp_evaluation_info_[num].attached_object_diff_scene_->getCurrentState(),
                                                   last_grasp_evaluation_info_.pickup_goal_.arm_name,
                                                   last_grasp_evaluation_info_[num].lift_trajectory_,
                                                   col);
    joint_trajectory_visualization_->playCurrentTrajectory(true);
  }
}

}
