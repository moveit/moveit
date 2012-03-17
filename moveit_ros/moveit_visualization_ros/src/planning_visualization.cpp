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

#include <moveit_visualization_ros/interactive_marker_helper_functions.h>
#include <moveit_visualization_ros/planning_visualization.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <kinematic_constraints/utils.h>
#include <planning_models/conversions.h>
#include <trajectory_processing/iterative_smoother.h>
#include <trajectory_processing/unnormalize_shortcutter.h>

namespace moveit_visualization_ros {

PlanningVisualization::PlanningVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                             const std::map<std::string, std::vector<moveit_msgs::JointLimits> >& group_joint_limit_map,
                                             boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                                             boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader>& kinematics_plugin_loader,
                                             ros::Publisher& marker_publisher)
  : planning_scene_(planning_scene), 
    ompl_interface_(planning_scene->getKinematicModel(), kinematics_plugin_loader),
    group_joint_limit_map_(group_joint_limit_map),
    last_trajectory_ok_(false)
{
  ompl_interface_.getPlanningContextManager().setMaximumSolutionSegmentLength(.1);

  const std::vector<srdf::Model::Group>& groups = planning_scene_->getSrdfModel()->getGroups();

  for(unsigned int i = 0; i < groups.size(); i++) {
    //special for arms for now
    if(groups[i].chains_.size() > 0 || groups[i].name_ == "arms") {
      group_visualization_map_[groups[i].name_].reset(new KinematicsStartGoalVisualization(planning_scene,
                                                                                           interactive_marker_server,
                                                                                           kinematics_plugin_loader,
                                                                                           groups[i].name_,
                                                                                           marker_publisher,
                                                                                           false));
      group_visualization_map_[groups[i].name_]->addMenuEntry("Plan", boost::bind(&PlanningVisualization::generatePlan, this, _1, true));
      group_visualization_map_[groups[i].name_]->addMenuEntry("Random start / goal", boost::bind(&PlanningVisualization::generateRandomStartEnd, this, _1));
      group_visualization_map_[groups[i].name_]->addMenuEntry("Reset start and goal", boost::bind(&PlanningVisualization::resetStartGoal, this, _1));
      group_visualization_map_[groups[i].name_]->setGoodBadMode(true);
    }
  }

  unnormalize_shortcutter_.reset(new trajectory_processing::UnnormalizeShortcutter(planning_scene->getKinematicModel()));
  trajectory_smoother_.reset(new trajectory_processing::IterativeParabolicSmoother());
 
  joint_trajectory_visualization_.reset(new JointTrajectoryVisualization(planning_scene,
                                                                         marker_publisher));

  ros::NodeHandle nh;
  display_traj_publisher_ = nh.advertise<moveit_msgs::DisplayTrajectory>("display_trajectory", 1);

}

void PlanningVisualization::updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene) {
  planning_scene_ = planning_scene;
  for(std::map<std::string, boost::shared_ptr<KinematicsStartGoalVisualization> >::iterator it = group_visualization_map_.begin();
      it != group_visualization_map_.end(); 
      it++) {
    it->second->updatePlanningScene(planning_scene);
  }
  joint_trajectory_visualization_->updatePlanningScene(planning_scene);
}

void PlanningVisualization::resetAllStartAndGoalStates() {
  for(std::map<std::string, boost::shared_ptr<KinematicsStartGoalVisualization> >::iterator it = group_visualization_map_.begin();
      it != group_visualization_map_.end(); 
      it++) {
    it->second->resetStartGoal();
  }
}

void PlanningVisualization::resetAllStartStates() {
  for(std::map<std::string, boost::shared_ptr<KinematicsStartGoalVisualization> >::iterator it = group_visualization_map_.begin();
      it != group_visualization_map_.end(); 
      it++) {
    it->second->resetStartState();
  }
}

void PlanningVisualization::addMenuEntry(const std::string& name, 
                                         const boost::function<void(const std::string&)>& callback)
{
  //For now adding to all groups
  for(std::map<std::string, boost::shared_ptr<KinematicsStartGoalVisualization> >::iterator it = group_visualization_map_.begin();
      it != group_visualization_map_.end(); 
      it++) {
    it->second->addMenuEntry(name, callback);
  }
}

void PlanningVisualization::hideAllGroups() {
  for(std::map<std::string, boost::shared_ptr<KinematicsStartGoalVisualization> >::iterator it = group_visualization_map_.begin();
      it != group_visualization_map_.end(); 
      it++) {
    it->second->hideAllMarkers();
  }
}

void PlanningVisualization::setGoalState(const std::string& group_name,
                                         const planning_models::KinematicState& state) 
{
  if(group_visualization_map_.find(group_name) == group_visualization_map_.end()) {
    ROS_WARN_STREAM("No group " << group_name << " for setting goal state");
    return;
  }
  group_visualization_map_.at(group_name)->setGoalState(state);
}

void PlanningVisualization::setStartState(const std::string& group_name,
                                         const planning_models::KinematicState& state) 
{
  if(group_visualization_map_.find(group_name) == group_visualization_map_.end()) {
    ROS_WARN_STREAM("No group " << group_name << " for setting goal state");
    return;
  }
  group_visualization_map_.at(group_name)->setStartState(state);
}

void PlanningVisualization::setAllStartChainModes(bool chain) {
  for(std::map<std::string, boost::shared_ptr<KinematicsStartGoalVisualization> >::iterator it = group_visualization_map_.begin();
      it != group_visualization_map_.end(); 
      it++) {
    it->second->setChainStartToCurrent(chain);
  }
}

void PlanningVisualization::selectGroup(const std::string& group) {
  if(current_group_ == group) return;
  if(group_visualization_map_.find(group) == group_visualization_map_.end()) {
    ROS_WARN_STREAM("No group name " << group);
  }
  if(!current_group_.empty()) {
    group_visualization_map_[current_group_]->hideAllMarkers();
  }
  current_group_ = group;
  group_visualization_map_[current_group_]->showAllMarkers();
}

void PlanningVisualization::generatePlan(const std::string& name, bool play) {

  ROS_INFO_STREAM("Planning for " << name);
  if(group_visualization_map_.find(name) == group_visualization_map_.end()) {
    ROS_INFO_STREAM("No group " << name << " so can't plan");
  }

  const planning_models::KinematicState& start_state = group_visualization_map_[name]->getStartState();
  const planning_models::KinematicState& goal_state = group_visualization_map_[name]->getGoalState();

  moveit_msgs::MoveItErrorCodes error_code;
  trajectory_msgs::JointTrajectory traj;
  if(generatePlanForScene(planning_scene_,
                  name,
                  &start_state,
                  &goal_state,
                  traj,
                  error_code)) {
    std_msgs::ColorRGBA col;
    col.a = .8;
    col.b = 1.0;

    joint_trajectory_visualization_->setTrajectory(start_state,
                                                   name,
                                                   traj,
                                                   col);
    if(play) {
      joint_trajectory_visualization_->playCurrentTrajectory();
    }
    moveit_msgs::DisplayTrajectory d;
    d.model_id = planning_scene_->getKinematicModel()->getName();
    planning_models::kinematicStateToRobotState(start_state, d.trajectory_start);
    d.trajectory.joint_trajectory = traj;
    display_traj_publisher_.publish(d);
    last_trajectory_ = traj;
    last_group_name_ = name;
    last_trajectory_ok_ = true;
  } else {
    last_trajectory_ok_ = false;
    ROS_INFO_STREAM("Planning failed");
  }
}

bool PlanningVisualization::generatePlanForScene(const planning_scene::PlanningSceneConstPtr& scene,
                                                 const std::string& group_name,
                                                 const planning_models::KinematicState* start_state,
                                                 const planning_models::KinematicState* goal_state,
                                                 trajectory_msgs::JointTrajectory& ret_traj,
                                                 moveit_msgs::MoveItErrorCodes& error_code) const
{
  moveit_msgs::GetMotionPlan::Request req;
  moveit_msgs::GetMotionPlan::Response res;

  req.motion_plan_request.group_name = group_name;
  planning_models::kinematicStateToRobotState(*start_state,req.motion_plan_request.start_state);
  req.motion_plan_request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state->getJointStateGroup(group_name),
                                                                                                     .001, .001));

  req.motion_plan_request.num_planning_attempts = 1;
  req.motion_plan_request.allowed_planning_time = ros::Duration(3.0);
  
  if(ompl_interface_.solve(scene, req, res)) {
    ROS_INFO_STREAM("Got " << res.trajectory.joint_trajectory.points.size());
    ROS_INFO_STREAM("Original last time " << res.trajectory.joint_trajectory.points.back().time_from_start);
    trajectory_msgs::JointTrajectory traj;
    moveit_msgs::MoveItErrorCodes error_code;
    moveit_msgs::Constraints emp_constraints;
    unnormalize_shortcutter_->shortcut(scene,
                                       group_name,
                                       start_state,
                                       group_joint_limit_map_.at(group_name),
                                       emp_constraints,
                                       emp_constraints,
                                       res.trajectory.joint_trajectory,
                                       ros::Duration(0.0),
                                       traj,
                                       error_code);
    trajectory_smoother_->smooth(traj,
                                 ret_traj,
                                 group_joint_limit_map_.at(group_name));
    ROS_INFO_STREAM("Smoothed last time " << ret_traj.points.back().time_from_start);
    return true;
  } else {
    return false;
  } 
}                                         
                                         

void PlanningVisualization::generateRandomStartEnd(const std::string& name) {

  ROS_INFO_STREAM("Getting request to set random start and end configurations");

  group_visualization_map_[name]->setRandomStartGoal();
}

void PlanningVisualization::resetStartGoal(const std::string& name) {

  ROS_INFO_STREAM("Getting request to reset start and end configurations");

  group_visualization_map_[name]->resetStartGoal();
}


} // namespace moveit_visualization_ros
