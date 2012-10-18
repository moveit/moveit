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

#include <moveit_visualization_ros/planning_visualization_qt_wrapper.h>

#include <QMetaType>

namespace moveit_visualization_ros {

PlanningVisualizationQtWrapper::
PlanningVisualizationQtWrapper(const planning_scene::PlanningSceneConstPtr& planning_scene,
                               const boost::shared_ptr<planning_pipeline::PlanningPipeline>& move_group_pipeline,
                               boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                               boost::shared_ptr<planning_models_loader::KinematicModelLoader>& kinematic_model_loader,
                               ros::Publisher& marker_publisher,
                               boost::shared_ptr<tf::TransformBroadcaster>& broadcaster) :
  PlanningVisualization(planning_scene, move_group_pipeline, interactive_marker_server, kinematic_model_loader, marker_publisher, broadcaster)
{
  qRegisterMetaType<planning_models::KinematicState*>("KinematicState");
  qRegisterMetaType<trajectory_msgs::JointTrajectory>("trajectory_msgs::JointTrajectory");
  qRegisterMetaType<moveit_msgs::MoveItErrorCodes>("moveit_msgs::MoveItErrorCodes");
  qRegisterMetaType<planning_scene::PlanningSceneConstPtr>("planning_scene::PlanningSceneConstPtr");
}


void PlanningVisualizationQtWrapper::newGroupSelected(const QString& new_group) {

  ROS_INFO_STREAM("new group: " << new_group.toStdString());
  selectGroup(new_group.toStdString());
}

void PlanningVisualizationQtWrapper::newPlannerSelected(const QString& new_planner) {

  ROS_INFO_STREAM("new planner: " << new_planner.toStdString());
  selectPlanner(new_planner.toStdString());
}

void PlanningVisualizationQtWrapper::generatePlanRequested(bool play) {
  generatePlan(current_group_, play);
  if(last_trajectory_ok_) {
    planGenerated(current_group_,
                  last_trajectory_);
  } else {
    moveit_msgs::MoveItErrorCodes failed_message;
    planFailed(failed_message);
  }
}

void PlanningVisualizationQtWrapper::generatePlanDiffSceneRequested(const std::string& group,
                                                                    const planning_scene::PlanningSceneConstPtr& scene,
                                                                    const planning_models::KinematicState* goal_state) 
{
  trajectory_msgs::JointTrajectory traj;
  moveit_msgs::MoveItErrorCodes error_code;
  if(generatePlanForScene(scene,
                          group,
                          &scene->getCurrentState(),
                          goal_state,
                          traj,
                          error_code)) {
    planGenerated(group,
                  traj);
  } else {
    planFailed(error_code);
  }
}


void PlanningVisualizationQtWrapper::setStartStateRequested(const std::string& group_name,
                                                            const planning_models::KinematicState* state)
{
  setStartState(group_name,
                *state);
}

void PlanningVisualizationQtWrapper::setGoalStateRequested(const std::string& group_name,
                                                           const planning_models::KinematicState* state)
{
  setGoalState(group_name,
               *state);
}

}

