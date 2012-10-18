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
#include <moveit_visualization_ros/kinematics_group_visualization.h>
#include <planning_scene_monitor_tools/kinematic_state_joint_state_publisher.h>

using namespace moveit_visualization_ros;

static const std::string VIS_TOPIC_NAME = "kinematics_visualization";

boost::shared_ptr<KinematicStateJointStatePublisher> joint_state_publisher_;
boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

void publisher_function() {
  ros::WallRate r(10.0);
  while(ros::ok())
  {
    joint_state_publisher_->broadcastRootTransform(planning_scene_monitor_->getPlanningScene()->getCurrentState());
    joint_state_publisher_->publishKinematicState(planning_scene_monitor_->getPlanningScene()->getCurrentState());
    r.sleep();
  }
}

void simplePrintClick() {
  ROS_INFO_STREAM("Getting called click");
}

void simplePrintMenu1(const std::string& name) {
  ROS_INFO_STREAM("Getting called menu 1");
}

void simplePrintMenu2(const std::string& name) {
  ROS_INFO_STREAM("Getting called menu 2");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinematics_visualization_test", ros::init_options::NoSigintHandler);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server;
  interactive_marker_server.reset(new interactive_markers::InteractiveMarkerServer("interactive_kinematics_visualization", "", false));
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  joint_state_publisher_.reset(new KinematicStateJointStatePublisher());

  boost::thread publisher_thread(boost::bind(&publisher_function));

  ros::NodeHandle nh;

  ros::Publisher vis_marker_array_publisher;
  ros::Publisher vis_marker_publisher;

  boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;

  vis_marker_publisher = nh.advertise<visualization_msgs::Marker> (VIS_TOPIC_NAME, 128);
  vis_marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray> (VIS_TOPIC_NAME + "_array", 128);

  std_msgs::ColorRGBA good_color;
  good_color.a = 1.0;    
  good_color.g = 1.0;    
  
  std_msgs::ColorRGBA bad_color;
  bad_color.a = 1.0;    
  bad_color.r = 1.0;    

  // planning_models::KinematicState kstate(planning_scene_monitor_->getPlanningScene()->getKinematicModel());
  // kstate.setToDefaultValues();

  // Eigen::Affine3d pos1 = Eigen::Affine3d(Eigen::Translation3d(3.0,0.0,0.0)*Eigen::Quaterniond::Identity());
  // Eigen::Affine3d pos2 = Eigen::Affine3d(Eigen::Translation3d(3.0,0.0,0.0)*Eigen::Quaterniond(0.965, 0.0, 0.258, 0.0));
  // //Eigen::Affine3d(Eigen::Translation3d(3.0,0.0,0.0)*Eigen::Quaterniond(M_PI/4.0, 0.0, M_PI/4.0, 0.0));
  // kstate.getLinkState("r_gripper_palm_link")->updateGivenGlobalLinkTransform(pos1);
  // kstate.getLinkState("l_gripper_palm_link")->updateGivenGlobalLinkTransform(pos2);

  // std::vector<std::string> links;
  // links.push_back("r_gripper_palm_link");
  // links.push_back("l_gripper_palm_link");

  // visualization_msgs::MarkerArray arr;
  // kstate.getRobotMarkers(good_color,
  //                        "temp",
  //                        ros::Duration(0.0),
  //                        arr,
  //                        links);
  // while(ros::ok()) {
  //   vis_marker_array_publisher.publish(arr);
  //   ros::WallDuration(0.2).sleep();
  // }

  boost::shared_ptr<planning_models_loader::KinematicModelLoader> 
    kinematic_model_loader = planning_scene_monitor_->getKinematicModelLoader();
  

  KinematicsGroupVisualization kv(planning_scene_monitor_->getPlanningScene(),
                                                                interactive_marker_server,
                                                                kinematic_model_loader,
                                                                "arms",
                                                                "state",
                                                                good_color,
                                                                bad_color,
                                                                vis_marker_array_publisher,
                                                                tf_broadcaster);

  kv.addButtonClickCallback(boost::bind(&simplePrintClick));
  kv.addMenuEntry("monkey1", boost::bind(&simplePrintMenu1, _1));
  kv.addMenuEntry("monkey2", boost::bind(&simplePrintMenu2, _1));
 
  // geometry_msgs::PoseStamped pose;
  // pose.pose.position.x = .58;
  // pose.pose.position.y = -.18;
  // pose.pose.position.z = .75;
  // pose.pose.orientation.x = 0.0;
  // pose.pose.orientation.y = 0.88793;
  // pose.pose.orientation.z = 0.0;
  // pose.pose.orientation.w = -.459979;

  // kv.updateEndEffectorState("right_arm",
  //                           pose);

  ros::waitForShutdown();
}

