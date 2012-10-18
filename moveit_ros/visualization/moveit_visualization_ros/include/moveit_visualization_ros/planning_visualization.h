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

#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <ros/ros.h>
#include <moveit_visualization_ros/kinematics_start_goal_visualization.h>
#include <moveit_visualization_ros/joint_trajectory_visualization.h>
#include <boost/function.hpp>
#include <planning_models_loader/kinematic_model_loader.h>
#include <planning_pipeline/planning_pipeline.h>

namespace moveit_visualization_ros
{

class PlanningVisualization 
{
public:

  PlanningVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                        const boost::shared_ptr<planning_pipeline::PlanningPipeline>& move_group_pipeline,
                        boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                        boost::shared_ptr<planning_models_loader::KinematicModelLoader>& kinematics_plugin_loader,
                        ros::Publisher& marker_publisher,
                        boost::shared_ptr<tf::TransformBroadcaster>& broadcaster);
  
  virtual void updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene);

  virtual void resetAllStartStates();
  virtual void resetAllStartAndGoalStates();

  virtual void addMenuEntry(const std::string& name,
                    const boost::function<void(const std::string&)>& callback);
  
  virtual void selectGroup(const std::string& name);

  virtual void selectPlanner(const std::string& name);

  virtual void hideAllGroups();

  virtual bool getLastTrajectory(std::string& group_name,
                         trajectory_msgs::JointTrajectory& traj) const
  {
    if(!last_trajectory_ok_) return false;
    group_name = last_group_name_;
    traj = last_trajectory_;
    return true;
  }

  virtual bool cycleOk() const {
    return cycle_ok_;
  }

  /** If true, start groups always follow the current state of the robot */
  virtual void setAllStartChainModes(bool chain);

  /** If false, all interactive markers are disabled */
  virtual void setAllStartInteractionModes(bool interaction_enabled);

  /** If false, all markers are hidden */
  virtual void setAllStartVisibility(bool visible);

  /** If false, all interactive markers are disabled */
  virtual void setAllGoalInteractionModes(bool interaction_enabled);

  /** If false, all markers are hidden */
  virtual void setAllGoalVisibility(bool visible);

  virtual std::string getCurrentGroup() const {
    return current_group_;
  }

  virtual std::string getCurrentPlanner() const {
    return current_planner_;
  }

  virtual void setGoalState(const std::string& group_name,
                    const planning_models::KinematicState& state);

  virtual void setStartState(const std::string& group_name,
                     const planning_models::KinematicState& state);

  virtual void addStateChangedCallback(const boost::function<void(const std::string&,
                                                          const planning_models::KinematicState&)>& callback);

protected:

  virtual void generatePlan(const std::string& name, bool play=true);
  virtual void generateOutAndBackPlan(const std::string& name, bool play=true);
  virtual bool generatePlanForScene(const planning_scene::PlanningSceneConstPtr& scene,
                            const std::string& arm_name,
                            const planning_models::KinematicState* start_state,
                            const planning_models::KinematicState* goal_state,
                            trajectory_msgs::JointTrajectory& traj,
                            moveit_msgs::MoveItErrorCodes& error_code) const;

  virtual void generateRandomStartEnd(const std::string& name);
  virtual void resetStartGoal(const std::string& name);
  virtual void playLastTrajectory();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  planning_scene::PlanningSceneConstPtr planning_scene_;
  boost::shared_ptr<planning_pipeline::PlanningPipeline> move_group_pipeline_;

  std::string current_group_;
  std::string current_planner_;
  std::map<std::string, boost::shared_ptr<KinematicsStartGoalVisualization> > group_visualization_map_;
  boost::shared_ptr<JointTrajectoryVisualization> joint_trajectory_visualization_;
  ros::Publisher display_traj_publisher_;
  
  std::string last_group_name_;
  trajectory_msgs::JointTrajectory last_trajectory_;
  planning_models::KinematicState last_start_state_;
  bool last_trajectory_ok_;
  bool cycle_ok_;

};

}

#endif
