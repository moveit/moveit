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

#ifndef _JOINT_TRAJECTORY_VISUALIZATION_H_
#define _JOINT_TRAJECTORY_VISUALIZATION_H_

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <planning_scene_monitor/planning_scene_monitor.h>


namespace moveit_visualization_ros
{

class JointTrajectoryVisualization 
{

public:
  JointTrajectoryVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                               ros::Publisher& marker_publisher);

  void updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene);
     
  void setTrajectory(const planning_models::KinematicState& start_state,
                     const std::string& group_name,
                     const trajectory_msgs::JointTrajectory& traj,
                     const std_msgs::ColorRGBA& color);

  void playCurrentTrajectory(bool block = false);

protected:

  void advanceTrajectory();

  planning_scene::PlanningSceneConstPtr planning_scene_;
  ros::Publisher marker_publisher_;

  planning_models::KinematicState current_state_;
  trajectory_msgs::JointTrajectory current_joint_trajectory_;
  std_msgs::ColorRGBA marker_color_;

  std::vector<std::string> link_model_names_;

  boost::shared_ptr<boost::thread> playback_thread_;

  boost::condition_variable trajectory_finished_;
  boost::mutex trajectory_playing_mutex_;
  bool playback_happening_;

  
};

}

#endif
