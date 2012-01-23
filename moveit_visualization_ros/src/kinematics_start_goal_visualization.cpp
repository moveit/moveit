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

#include <moveit_visualization_ros/kinematics_start_goal_visualization.h>
#include <moveit_visualization_ros/interactive_marker_helper_functions.h>

namespace moveit_visualization_ros
{

KinematicsStartGoalVisualization::KinematicsStartGoalVisualization(planning_scene::PlanningSceneConstPtr planning_scene,
                                                                   boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server, 
                                                                   const std::string& group_name, 
                                                                   const std::string& kinematics_solver_name,
                                                                   ros::Publisher& marker_publisher)
{
  std_msgs::ColorRGBA good_color;
  good_color.g = good_color.a = 1.0;

  std_msgs::ColorRGBA bad_color;
  bad_color.r = bad_color.a = 1.0;
  
  start_.reset(new KinematicsGroupVisualization(planning_scene,
                                                interactive_marker_server,
                                                group_name,
                                                "start_position",
                                                kinematics_solver_name,
                                                makeRandomColor(.2,1.0),
                                                bad_color,
                                                marker_publisher));

  goal_.reset(new KinematicsGroupVisualization(planning_scene,
                                               interactive_marker_server,
                                               group_name,
                                               "end_position",
                                               kinematics_solver_name,
                                               makeRandomColor(.2,1.0),
                                               bad_color,
                                               marker_publisher));

  start_->addButtonClickCallback(boost::bind(&KinematicsStartGoalVisualization::startOn, this));
  goal_->addButtonClickCallback(boost::bind(&KinematicsStartGoalVisualization::goalOn, this));

  goalOn();

}

void KinematicsStartGoalVisualization::startOn() {
  goal_->disable6DOFControls();
  goal_->setMarkerAlpha(0.25);
  start_->enable6DOFControls();
  start_->setMarkerAlpha(1.0);
}

void KinematicsStartGoalVisualization::goalOn() {
  start_->disable6DOFControls();
  start_->setMarkerAlpha(0.25);
  goal_->enable6DOFControls();
  goal_->setMarkerAlpha(1.0);
}

}
