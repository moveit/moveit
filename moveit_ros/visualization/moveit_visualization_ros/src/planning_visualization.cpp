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

namespace moveit_visualization_ros {

PlanningVisualization::PlanningVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                             const boost::shared_ptr<planning_pipeline::PlanningPipeline>& move_group_pipeline,
                                             boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                                             boost::shared_ptr<planning_models_loader::KinematicModelLoader>& kinematic_model_loader,
                                             ros::Publisher& marker_publisher,
                                             boost::shared_ptr<tf::TransformBroadcaster>& broadcaster)
  : nh_(""),
    pnh_("~"),
    planning_scene_(planning_scene),
    move_group_pipeline_(move_group_pipeline),
    last_start_state_(planning_scene->getCurrentState()),
    last_trajectory_ok_(false),
    cycle_ok_(false)
{
  //ompl_interface_.getPlanningContextManager().setMaximumSolutionSegmentLength(.1);

  const std::vector<srdf::Model::Group>& groups = planning_scene_->getKinematicModel()->getSRDF()->getGroups();

  for(unsigned int i = 0; i < groups.size(); i++) {
    //special for arms for now
    if(groups[i].chains_.size() > 0 || groups[i].name_ == "arms") {
      group_visualization_map_[groups[i].name_].reset(new KinematicsStartGoalVisualization(planning_scene,
                                                                                           interactive_marker_server,
                                                                                           kinematic_model_loader,
                                                                                           groups[i].name_,
                                                                                           marker_publisher,
                                                                                           broadcaster,
                                                                                           false));
      group_visualization_map_[groups[i].name_]->addMenuEntry("Plan", boost::bind(&PlanningVisualization::generatePlan, this, _1, true));
      group_visualization_map_[groups[i].name_]->addMenuEntry("Plan out and back", boost::bind(&PlanningVisualization::generateOutAndBackPlan, this, _1, true));
      group_visualization_map_[groups[i].name_]->addMenuEntry("Play last trajectory", boost::bind(&PlanningVisualization::playLastTrajectory, this));
      group_visualization_map_[groups[i].name_]->addMenuEntry("Random start / goal", boost::bind(&PlanningVisualization::generateRandomStartEnd, this, _1));
      group_visualization_map_[groups[i].name_]->addMenuEntry("Reset start and goal", boost::bind(&PlanningVisualization::resetStartGoal, this, _1));
      group_visualization_map_[groups[i].name_]->setGoodBadMode(true);
    }
  }

  joint_trajectory_visualization_.reset(new JointTrajectoryVisualization(planning_scene,
                                                                         marker_publisher));
  display_traj_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("display_trajectory", 1);

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

void PlanningVisualization::addStateChangedCallback(const boost::function<void(const std::string&,
                                                                               const planning_models::KinematicState&)>& callback)
{
  for(std::map<std::string, boost::shared_ptr<KinematicsStartGoalVisualization> >::iterator it = group_visualization_map_.begin();
      it != group_visualization_map_.end();
      it++) {
    it->second->addStateChangedCallback(callback);
  }
}

void PlanningVisualization::setAllStartChainModes(bool chain) {
  for(std::map<std::string, boost::shared_ptr<KinematicsStartGoalVisualization> >::iterator it = group_visualization_map_.begin();
      it != group_visualization_map_.end();
      it++) {
    it->second->setChainStartToCurrent(chain);
  }
}

void PlanningVisualization::setAllStartInteractionModes(bool interaction_enabled) {
  for(std::map<std::string, boost::shared_ptr<KinematicsStartGoalVisualization> >::iterator it = group_visualization_map_.begin();
      it != group_visualization_map_.end();
      it++) {
    it->second->setStartInteractionEnabled(interaction_enabled);
  }
}

void PlanningVisualization::setAllStartVisibility(bool visible) {
  for(std::map<std::string, boost::shared_ptr<KinematicsStartGoalVisualization> >::iterator it = group_visualization_map_.begin();
      it != group_visualization_map_.end();
      it++) {
    it->second->setStartVisible(visible);
  }
}

void PlanningVisualization::setAllGoalInteractionModes(bool interaction_enabled) {
  for(std::map<std::string, boost::shared_ptr<KinematicsStartGoalVisualization> >::iterator it = group_visualization_map_.begin();
      it != group_visualization_map_.end();
      it++) {
    it->second->setGoalInteractionEnabled(interaction_enabled);
  }
}

void PlanningVisualization::setAllGoalVisibility(bool visible) {
  for(std::map<std::string, boost::shared_ptr<KinematicsStartGoalVisualization> >::iterator it = group_visualization_map_.begin();
      it != group_visualization_map_.end();
      it++) {
    it->second->setGoalVisible(visible);
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

void PlanningVisualization::selectPlanner(const std::string& planner) {
  if(current_planner_ == planner) 
    return;
  // TODO: not sure what this is suppose to do - error check? - DTC
  /*  if(planner_visualization_map_.find(planner) == planner_visualization_map_.end()) {
    ROS_WARN_STREAM("No planner name " << planner);
  }
  if(!current_planner_.empty()) {
    planner_visualization_map_[current_planner_]->hideAllMarkers();
    }*/
  current_planner_ = planner;
  //  planner_visualization_map_[current_planner_]->showAllMarkers();
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
    last_start_state_ = start_state;
    last_trajectory_ = traj;
    last_group_name_ = name;
    last_trajectory_ok_ = true;
    cycle_ok_ = true;
    if(play) {
      playLastTrajectory();
    }
  } else {
    last_trajectory_ok_ = false;
    ROS_INFO_STREAM("Planning failed");
  }
}

void PlanningVisualization::generateOutAndBackPlan(const std::string& name, bool play) {
  ROS_INFO_STREAM("Planning out and back for " << name);
  if(group_visualization_map_.find(name) == group_visualization_map_.end()) {
    ROS_INFO_STREAM("No group " << name << " so can't plan");
  }

  const planning_models::KinematicState& start_state = group_visualization_map_[name]->getStartState();
  const planning_models::KinematicState& goal_state = group_visualization_map_[name]->getGoalState();

  moveit_msgs::MoveItErrorCodes error_code;
  trajectory_msgs::JointTrajectory out_traj;
  trajectory_msgs::JointTrajectory back_traj;
  if(generatePlanForScene(planning_scene_,
                          name,
                          &start_state,
                          &goal_state,
                          out_traj,
                          error_code)) {
    if(generatePlanForScene(planning_scene_,
                            name,
                            &goal_state,
                            &start_state,
                            back_traj,
                            error_code)) {
      ros::Duration last_dur = out_traj.points.back().time_from_start;
      for(unsigned int i = 0; i < back_traj.points.size(); i++) {
        out_traj.points.push_back(back_traj.points[i]);
        out_traj.points.back().time_from_start += last_dur;
      }
      last_trajectory_ = out_traj;
      last_start_state_ = start_state;
      last_trajectory_ = out_traj;
      last_group_name_ = name;
      last_trajectory_ok_ = true;
      playLastTrajectory();
      cycle_ok_ = true;
    }
  } else {
    ROS_WARN_STREAM("Out portion of out and back failed");
    last_trajectory_ok_ = false;
    cycle_ok_ = false;
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

  // Manually define what planner to use - DTC
  req.motion_plan_request.planner_id = getCurrentPlanner();

  ROS_INFO_STREAM("USING MENU PLANNER " << getCurrentPlanner());

  if(!move_group_pipeline_->generatePlan(scene, req, res)) {
    ROS_WARN_STREAM("Response traj " << res.trajectory.joint_trajectory);
    return false;
  }
  ret_traj = res.trajectory.joint_trajectory;
  return true;
}

void PlanningVisualization::playLastTrajectory() {
  if(!last_trajectory_ok_) return;

  std_msgs::ColorRGBA col;
  col.a = .8;
  col.b = 1.0;

  joint_trajectory_visualization_->setTrajectory(last_start_state_,
                                                 last_group_name_,
                                                 last_trajectory_,
                                                 col);
  joint_trajectory_visualization_->playCurrentTrajectory();
  moveit_msgs::DisplayTrajectory d;
  d.model_id = planning_scene_->getKinematicModel()->getName();
  planning_models::kinematicStateToRobotState(last_start_state_, d.trajectory_start);
  d.trajectory.joint_trajectory = last_trajectory_;
  display_traj_publisher_.publish(d);
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
