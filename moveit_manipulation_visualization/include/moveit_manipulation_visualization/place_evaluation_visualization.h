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

#ifndef _PLACE_EVALUATION_VISUALIZATION_H_
#define _PLACE_EVALUATION_VISUALIZATION_H_

#include <planning_scene/planning_scene.h>
#include <grasp_place_evaluation/place_evaluator.h>
#include <moveit_visualization_ros/joint_trajectory_visualization.h>

namespace moveit_manipulation_visualization {

class PlaceEvaluationVisualization {
  
public:

  PlaceEvaluationVisualization(ros::Publisher& marker_publisher);
  
  ~PlaceEvaluationVisualization() {};
  
  void removeAllMarkers();

  void hideAllMarkers();

  void showHiddenMarkers();

  void showPlacePose(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const grasp_place_evaluation::PlaceExecutionInfoVector& place_info,
                     unsigned int num,
                     bool show_place,
                     bool show_preplace,
                     bool show_retreat);
  
  void playInterpolatedTrajectories(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                    const grasp_place_evaluation::PlaceExecutionInfoVector& place_info,
                                    boost::shared_ptr<moveit_visualization_ros::JointTrajectoryVisualization> joint_trajectory_visualization,
                                    unsigned int num,
                                    bool play_approach,
                                    bool play_retreat,
                                    bool in_thread = true,
                                    bool hide_markers = true);
  
protected:

  void playInterpolatedTrajectoriesThread(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                          const grasp_place_evaluation::PlaceExecutionInfoVector& place_info,
                                          boost::shared_ptr<moveit_visualization_ros::JointTrajectoryVisualization> joint_trajectory_visualization,
                                          unsigned int num,
                                          bool play_approach,
                                          bool play_retreat,
                                          bool hide_markers = true);

  ros::Publisher marker_publisher_;
  visualization_msgs::MarkerArray last_marker_array_;  
  visualization_msgs::MarkerArray saved_marker_array_;
};

}
#endif
