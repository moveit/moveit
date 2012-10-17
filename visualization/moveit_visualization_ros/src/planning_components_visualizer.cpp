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

#include <ros/ros.h>
#include <moveit_visualization_ros/interactive_object_visualization.h>
#include <moveit_visualization_ros/planning_visualization.h>
#include <moveit_visualization_ros/kinematic_state_joint_state_publisher.h>

using namespace moveit_visualization_ros;

static const std::string VIS_TOPIC_NAME = "planning_components_visualization";

boost::shared_ptr<KinematicStateJointStatePublisher> joint_state_publisher_;
boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
boost::shared_ptr<PlanningVisualization> pv_;
boost::shared_ptr<InteractiveObjectVisualization> iov_;

void publisher_function() {
  ros::WallRate r(10.0);
  while(ros::ok())
  {
    joint_state_publisher_->broadcastRootTransform(planning_scene_monitor_->getPlanningScene()->getCurrentState());
    joint_state_publisher_->publishKinematicState(planning_scene_monitor_->getPlanningScene()->getCurrentState());
    r.sleep();
  }
}

void updateCallback(planning_scene::PlanningSceneConstPtr planning_scene) {
  pv_->updatePlanningScene(planning_scene);
}

void addCubeCallback() {
  iov_->addCube();
}

void addCylinderCallback() {
  iov_->addCylinder();
}

void addSphereCallback() {
  iov_->addSphere();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_components_visualizer", ros::init_options::NoSigintHandler);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  interactive_marker_server_.reset(new interactive_markers::InteractiveMarkerServer("interactive_kinematics_visualization", "", false));
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  joint_state_publisher_.reset(new KinematicStateJointStatePublisher());

  boost::thread publisher_thread(boost::bind(&publisher_function));

  ros::NodeHandle nh;

  ros::Publisher vis_marker_array_publisher;
  ros::Publisher vis_marker_publisher;

  vis_marker_publisher = nh.advertise<visualization_msgs::Marker> (VIS_TOPIC_NAME, 128);
  vis_marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray> (VIS_TOPIC_NAME + "_array", 128);

  std_msgs::ColorRGBA col;
  col.r = col.g = col.b = .5;
  col.a = 1.0;

  std_msgs::ColorRGBA good_color;
  good_color.a = 1.0;    
  good_color.g = 1.0;    
  
  std_msgs::ColorRGBA bad_color;
  bad_color.a = 1.0;    
  bad_color.r = 1.0;    

  boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader> 
    kinematics_plugin_loader(new kinematics_plugin_loader::KinematicsPluginLoader());

  pv_.reset(new PlanningVisualization(planning_scene_monitor_->getPlanningScene(),
                                      planning_scene_monitor_->getGroupJointLimitsMap(),
                                      interactive_marker_server_,
                                      kinematics_plugin_loader,
                                      vis_marker_array_publisher));
  
  iov_.reset(new InteractiveObjectVisualization(planning_scene_monitor_->getPlanningScene(),
                                                interactive_marker_server_,
                                                col));
  
  iov_->setUpdateCallback(boost::bind(&updateCallback, _1));

  pv_->addMenuEntry("Add Cube", boost::bind(&addCubeCallback));
  pv_->addMenuEntry("Add Cylinder", boost::bind(&addCylinderCallback));
  pv_->addMenuEntry("Add Sphere", boost::bind(&addSphereCallback));
  
  ros::waitForShutdown();
}

