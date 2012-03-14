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

#include <moveit_visualization_ros/joint_trajectory_visualization.h>

static const ros::WallDuration sleep_time = ros::WallDuration(.01);

namespace moveit_visualization_ros
{

JointTrajectoryVisualization::JointTrajectoryVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                           ros::Publisher& marker_publisher)
  : planning_scene_(planning_scene),
    marker_publisher_(marker_publisher),
    current_state_(planning_scene_->getCurrentState())
{
  
}; 

void JointTrajectoryVisualization::setTrajectory(const planning_models::KinematicState& start_state,
                                                 const std::string& group_name,
                                                 const trajectory_msgs::JointTrajectory& traj,
                                                 const std_msgs::ColorRGBA& color)
{
  link_model_names_ = planning_scene_->getKinematicModel()->getJointModelGroup(group_name)->getUpdatedLinkModelNames();
  current_state_ = start_state;
  current_joint_trajectory_ = traj;
  marker_color_ = color;
}

void JointTrajectoryVisualization::updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene)
{  
  planning_scene_ = planning_scene;
}

void JointTrajectoryVisualization::playCurrentTrajectory(bool block)
{
  if(playback_thread_) {
    ROS_DEBUG_STREAM("Cancelling current playback");
    playback_thread_->interrupt();
    playback_thread_->join();
    ROS_DEBUG_STREAM("Cancelling completed");
  }

  if(current_joint_trajectory_.points.size() == 0) {
    ROS_WARN_STREAM("No points in trajectory, not playing");
    return;
  }

  if(block) {
    boost::unique_lock<boost::mutex> lock(trajectory_playing_mutex_);
    playback_happening_ = true;
    playback_thread_.reset(new boost::thread(boost::bind(&JointTrajectoryVisualization::advanceTrajectory, this)));
    while(playback_happening_) {
      trajectory_finished_.wait(lock);
    }
  } else {
    playback_thread_.reset(new boost::thread(boost::bind(&JointTrajectoryVisualization::advanceTrajectory, this)));
  }
}

void JointTrajectoryVisualization::advanceTrajectory() {
  visualization_msgs::MarkerArray arr;
  try {
    unsigned int current_point = 0;
    ros::WallTime playback_start_time = ros::WallTime::now();
    bool use_time = false;
    if(current_joint_trajectory_.points.back().time_from_start > ros::Duration(0.0)) {
      use_time = true;
    }
    while(ros::ok() && current_point < current_joint_trajectory_.points.size()) {
      arr.markers.clear();
      std::map<std::string, double> joint_state;
      for(unsigned int i = 0; i < current_joint_trajectory_.joint_names.size(); i++) {
        joint_state[current_joint_trajectory_.joint_names[i]] =
          current_joint_trajectory_.points[current_point].positions[i];
      }
      current_state_.setStateValues(joint_state);
      current_state_.getRobotMarkers(marker_color_,
                                     "joint_trajectory",
                                     ros::Duration(0.0),
                                     arr,
                                     link_model_names_);
      marker_publisher_.publish(arr);
      boost::this_thread::sleep((ros::WallTime::now()+ros::WallDuration(.05)).toBoost());
      if(!use_time) {
        current_point++;
      } else {
        for( ; current_point < current_joint_trajectory_.points.size(); current_point++) {
          if(ros::WallDuration(current_joint_trajectory_.points[current_point].time_from_start.toSec()) >= 
             (ros::WallTime::now()- playback_start_time)) {
            break;
          }
        }
      } 
    } 
    ROS_INFO_STREAM("Last time " << (ros::WallTime::now()- playback_start_time));
  } catch(...) {
    ROS_DEBUG_STREAM("Playback interrupted");
    
    return;
  }
  if(!arr.markers.empty()) {
    for(unsigned int i = 0; i < arr.markers.size(); i++) {
      arr.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    marker_publisher_.publish(arr);
  }
  {
    boost::lock_guard<boost::mutex> lock(trajectory_playing_mutex_);
    playback_happening_=false;
  }
  trajectory_finished_.notify_all();
}

}
