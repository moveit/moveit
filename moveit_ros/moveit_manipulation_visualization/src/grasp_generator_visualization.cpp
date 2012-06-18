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

#include <moveit_manipulation_visualization/grasp_generator_visualization.h>

namespace moveit_manipulation_visualization {

GraspGeneratorVisualization::GraspGeneratorVisualization(ros::Publisher& marker_publisher) :
  grasp_generator_(new GraspGeneratorDummy()),
  marker_publisher_(marker_publisher)
{
  
}

bool GraspGeneratorVisualization::generateGrasps(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                 const std::string& obj,
                                                 const std::string& arm_name,
                                                 std::vector<moveit_manipulation_msgs::Grasp>& grasps,
                                                 std::string& frame_name)
{
  return grasp_generator_->generateGrasps(planning_scene,
                                          arm_name,
                                          obj, 
                                          grasps,
                                          frame_name);
}


void GraspGeneratorVisualization::showGrasp(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                            const std::string& arm_name,
                                            const std::string& obj,
                                            const std::string& grasp_frame,
                                            moveit_manipulation_msgs::Grasp& grasp)
{
  moveit_msgs::CollisionObject co;
  if(!planning_scene->getCollisionObjectMsg(obj, 
                                            co)) {
    ROS_WARN_STREAM("Don't appear to have object " << obj << " in planning scene");
    return;
  }

  Eigen::Affine3d gp;
  planning_models::poseFromMsg(grasp.grasp_pose, gp);

  Eigen::Affine3d scene_pose;

  if(grasp_frame == obj) {
    Eigen::Affine3d obj_pose;
    planning_models::poseFromMsg(co.primitive_poses.empty() ? co.mesh_poses[0] : co.primitive_poses[0], obj_pose);
    scene_pose = obj_pose*gp;
  } else if(grasp_frame != planning_scene->getPlanningFrame()) {
    planning_scene->getTransforms()->transformPose(planning_scene->getCurrentState(),
                                                   grasp_frame,
                                                   gp,
                                                   scene_pose);
    
  } else {
    planning_models::poseFromMsg(grasp.grasp_pose, scene_pose);
  }
  planning_models::KinematicState state(planning_scene->getCurrentState());
  state.setStateValues(grasp.grasp_posture);
  ROS_DEBUG_STREAM("Grasp posture " << grasp.grasp_posture);
  ROS_DEBUG_STREAM("Pre-grasp posture " << grasp.pre_grasp_posture);
  state.updateStateWithLinkAt(planning_scene->getKinematicModel()->getJointModelGroup(arm_name)->getLinkModelNames().back(), scene_pose);
  std_msgs::ColorRGBA col;
  col.g = col.a = 1.0;

  std::vector<std::string> end_effector_links = planning_scene->getKinematicModel()->getJointModelGroup(planning_scene->getKinematicModel()->getJointModelGroup(arm_name)->getAttachedEndEffectorGroupName())->getLinkModelNames();
  state.getRobotMarkers(col,
                        "grasp",
                        ros::Duration(0.0),
                        last_markers_,
                        end_effector_links);
  marker_publisher_.publish(last_markers_);
}

void GraspGeneratorVisualization::removeAllMarkers() {

  for(unsigned int i = 0; i < last_markers_.markers.size(); i++) {
    last_markers_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  marker_publisher_.publish(last_markers_);
  last_markers_.markers.clear();
}

}
