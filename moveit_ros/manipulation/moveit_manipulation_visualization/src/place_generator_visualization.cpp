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

#include <moveit_manipulation_visualization/place_generator_visualization.h>

namespace moveit_manipulation_visualization {

PlaceGeneratorVisualization::PlaceGeneratorVisualization(ros::Publisher& marker_publisher) :
  place_generator_(new PlaceGeneratorDummy()),
  marker_publisher_(marker_publisher)
{
}

bool PlaceGeneratorVisualization::generatePlaces(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                 const std::string& obj,
                                                 const std::string& support,
                                                 std::vector<geometry_msgs::PoseStamped>& place_locations)
{
  return place_generator_->generatePlaceLocations(planning_scene,
                                                  obj, 
                                                  support,
                                                  place_locations);
}


void PlaceGeneratorVisualization::showPlace(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                            const std::string& arm_name,
                                            const moveit_manipulation_msgs::Grasp& grasp,
                                            const geometry_msgs::PoseStamped& place_location)
{
  planning_models::KinematicState state(planning_scene->getCurrentState());

  state.setStateValues(grasp.grasp_posture);

  Eigen::Affine3d place_pose;
  planning_models::poseFromMsg(place_location.pose, place_pose);

  Eigen::Affine3d grasp_pose;
  planning_models::poseFromMsg(grasp.grasp_pose,
                               grasp_pose);

  Eigen::Affine3d tip_pose = place_pose*grasp_pose;

  state.updateStateWithLinkAt(planning_scene->getKinematicModel()->getJointModelGroup(arm_name)->getLinkModelNames().back(), tip_pose);
  std_msgs::ColorRGBA col;
  col.r = col.g = col.a = 1.0;

  std::vector<std::string> end_effector_links = planning_scene->getKinematicModel()->getJointModelGroup(planning_scene->getKinematicModel()->getJointModelGroup(arm_name)->getAttachedEndEffectorGroupName())->getLinkModelNames();

  state.getRobotMarkers(col,
                        "place",
                        ros::Duration(0.0),
                        last_markers_,
                        end_effector_links);
  marker_publisher_.publish(last_markers_);
}

void PlaceGeneratorVisualization::removeAllMarkers() {

  for(unsigned int i = 0; i < last_markers_.markers.size(); i++) {
    last_markers_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  marker_publisher_.publish(last_markers_);
  last_markers_.markers.clear();
}

}
