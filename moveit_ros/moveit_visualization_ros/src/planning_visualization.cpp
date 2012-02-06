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
#include <kinematic_constraints/utils.h>
#include <planning_models/conversions.h>

namespace moveit_visualization_ros {

PlanningVisualization::PlanningVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                             boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                                             ros::Publisher& marker_publisher)
  : planning_scene_(planning_scene), ompl_interface_(planning_scene->getKinematicModel())
{
  const std::vector<srdf::Model::Group>& groups = planning_scene_->getSrdfModel()->getGroups();

  for(unsigned int i = 0; i < groups.size(); i++) {
    if(groups[i].chains_.size() > 0 && groups[i].subgroups_.size() == 0) {
      group_visualization_map_[groups[i].name_].reset(new KinematicsStartGoalVisualization(planning_scene,
                                                                                            interactive_marker_server,
                                                                                            groups[i].name_,
                                                                                            "pr2_arm_kinematics/PR2ArmKinematicsPlugin",
                                                                                            marker_publisher,
                                                                                            false));
      group_visualization_map_[groups[i].name_]->addMenuEntry("Plan", boost::bind(&PlanningVisualization::generatePlan, this, _1));
      group_visualization_map_[groups[i].name_]->addMenuEntry("Random start / goal", boost::bind(&PlanningVisualization::generateRandomStartEnd, this, _1));
      group_visualization_map_[groups[i].name_]->addMenuEntry("Reset start and goal", boost::bind(&PlanningVisualization::resetStartGoal, this, _1));
    }
  }
 
  joint_trajectory_visualization_.reset(new JointTrajectoryVisualization(planning_scene,
                                                                         marker_publisher));

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

void PlanningVisualization::generatePlan(const std::string& name) {

  ROS_INFO_STREAM("Planning for " << name);
  if(group_visualization_map_.find(name) == group_visualization_map_.end()) {
    ROS_INFO_STREAM("No group " << name << " so can't plan");
  }

  const planning_models::KinematicState& start_state = group_visualization_map_[name]->getStartState();

  const planning_models::KinematicState& goal_state = group_visualization_map_[name]->getGoalState();
  
  moveit_msgs::GetMotionPlan::Request req;
  moveit_msgs::GetMotionPlan::Response res;

  req.motion_plan_request.group_name = name;
  planning_models::kinematicStateToRobotState(start_state,req.motion_plan_request.start_state);
  req.motion_plan_request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state.getJointStateGroup(name),
                                                                                                     .001, .001));
  req.motion_plan_request.num_planning_attempts = 1;
  req.motion_plan_request.allowed_planning_time = ros::Duration(3.0);
  
  std_msgs::ColorRGBA col;
  col.a = .8;
  col.b = 1.0;

  ompl_interface_.solve(planning_scene_, req, res);
  joint_trajectory_visualization_->setTrajectory(start_state,
                                                 res.trajectory.joint_trajectory,
                                                 col);

  joint_trajectory_visualization_->playCurrentTrajectory();
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
