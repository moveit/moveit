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

// Author: E. Gil Jones, Ken Anderson

#include <ros/ros.h>
#include <distance_field/propagation_distance_field.h>
#include <planning_scene_monitor_tools/kinematic_state_joint_state_publisher.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <collision_distance_field/collision_distance_field_types.h>
#include <collision_distance_field/collision_robot_distance_field.h>
#include <collision_distance_field/collision_world_distance_field.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <moveit_visualization_ros/interactive_marker_helper_functions.h>
#include <planning_models/transforms.h>

using namespace moveit_visualization_ros;

static const std::string VIS_TOPIC_NAME = "distance_field_visualization";

boost::shared_ptr<KinematicStateJointStatePublisher> joint_state_publisher_;
boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
//boost::shared_ptr<InteractiveObjectVisualization> iov_;

void publisher_function() {
  ros::WallRate r(10.0);
  while(ros::ok())
  {
    joint_state_publisher_->broadcastRootTransform(planning_scene_monitor_->getPlanningScene()->getCurrentState());
    joint_state_publisher_->publishKinematicState(planning_scene_monitor_->getPlanningScene()->getCurrentState());
    r.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_object_visualization", ros::init_options::NoSigintHandler);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  // interactive_marker_server_.reset(new interactive_markers::InteractiveMarkerServer("interactive_distance_field_visualization", "", false));
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  joint_state_publisher_.reset(new KinematicStateJointStatePublisher());

  std::map<std::string, double> joint_vals;
  joint_vals["r_shoulder_pan_joint"] = -.5;
  planning_scene_monitor_->getPlanningScene()->getCurrentState().setStateValues(joint_vals);

  boost::thread publisher_thread(boost::bind(&publisher_function));

  ros::NodeHandle nh;

  ros::Publisher vis_marker_array_publisher;
  ros::Publisher vis_marker_publisher;

  vis_marker_publisher = nh.advertise<visualization_msgs::Marker> (VIS_TOPIC_NAME, 128);
  vis_marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray> (VIS_TOPIC_NAME + "_array", 128);

  collision_distance_field::CollisionRobotDistanceField coll(planning_scene_monitor_->getPlanningScene()->getKinematicModel());
  collision_distance_field::CollisionWorldDistanceField world;
  shapes::Shape* shape = new shapes::Box(.1,.1,.5);
  Eigen::Affine3d pos1 = Eigen::Affine3d::Identity();
  pos1.translation().x() = .4;
  pos1.translation().z() = 1.0;
  world.addToObject("box", shapes::ShapeConstPtr(shape), pos1);

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = "right_arm";
  collision_detection::AllowedCollisionMatrix acm = planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix();
  acm.setEntry("r_shoulder_pan_link", "r_shoulder_pan_link", true);
  acm.setEntry("r_shoulder_lift_link", "r_shoulder_lift_link", true);
  acm.setEntry("r_upper_arm_link", "r_upper_arm_link", true);

  boost::shared_ptr<const collision_distance_field::CollisionRobotDistanceField::GroupStateRepresentation> world_gsr =
    world.getCollisionGradients(req, 
                                res, 
                                coll, 
                                planning_scene_monitor_->getPlanningScene()->getCurrentState(), 
                                acm);

  // world.checkRobotCollision(req,
  //                           res,
  //                           coll,
  //                           planning_scene_monitor_->getPlanningScene()->getCurrentState(), 
  //                           acm);
                            
  visualization_msgs::Marker inf_marker;
  world.getDistanceField()->getIsoSurfaceMarkers(0.0,.01,
                                                  planning_scene_monitor_->getPlanningScene()->getPlanningFrame(),
                                                  ros::Time::now(),
                                                  Eigen::Affine3d::Identity(),
                                                  inf_marker);
  //boost::shared_ptr<const collision_distance_field::CollisionRobotDistanceField::GroupStateRepresentation> world_gsr = world.getLastGroupStateRepresentation();

  visualization_msgs::MarkerArray arrow_markers;
  std_msgs::ColorRGBA col;
  col.b = 1.0;
  col.a = .8;
  collision_distance_field::getProximityGradientMarkers(planning_scene_monitor_->getPlanningScene()->getPlanningFrame(),
                                                        "arrows",
                                                        ros::Duration(0.0),
                                                        world_gsr->link_body_decompositions_,
                                                        world_gsr->gradients_,
                                                        arrow_markers);

  // //req.contacts = true;
  // coll.checkSelfCollision(req, res, planning_scene_monitor_->getPlanningScene()->getCurrentState(), acm);
  // boost::shared_ptr<const collision_distance_field::CollisionRobotDistanceField::DistanceFieldCacheEntry> dfce = coll.getLastDistanceFieldEntry();
  // if(!dfce) {
  //   ROS_WARN_STREAM("no dfce");
  //   exit(-1);
  // }
  // boost::shared_ptr<const collision_distance_field::CollisionRobotDistanceField::GroupStateRepresentation> gsr = coll.getLastGroupStateRepresentation();
  // // visualization_msgs::MarkerArray sphere_markers;
  // // std_msgs::ColorRGBA col;
  // // col.g = 1.0;
  // // col.a = .8;
  // // collision_distance_field::getCollisionSphereMarkers(col,
  // //                                                     planning_scene_monitor_->getPlanningScene()->getPlanningFrame(),
  // //                                                     "spheres",
  // //                                                     ros::Duration(0.0),
  // //                                                     gsr->link_body_decompositions_,
  // //                                                     sphere_markers);

  // visualization_msgs::MarkerArray arrow_markers;
  // // std_msgs::ColorRGBA col;
  // // col.b = 1.0;
  // // col.a = .8;
  // // collision_distance_field::getProximityGradientMarkers(col,
  // //                                                      planning_scene_monitor_->getPlanningScene()->getPlanningFrame(),
  // //                                                      "arrows",
  // //                                                      ros::Duration(0.0),
  // //                                                      gsr->link_body_decompositions_,
  // //                                                      gsr->gradients_,
  // //                                                      arrow_markers);

  // visualization_msgs::Marker inf_marker;
  // dfce->distance_field_->getIsoSurfaceMarkers(0.0,.01,
  //                                             planning_scene_monitor_->getPlanningScene()->getPlanningFrame(),
  //                                             ros::Time::now(),
  //                                             Eigen::Affine3d::Identity(),
  //                                             inf_marker);
  // bodies::Body* b = bodies::createBodyFromShape(planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getLinkModel("base_link")->getShape().get());
  // b->setPose(planning_scene_monitor_->getPlanningScene()->getCurrentState().getLinkState("base_link")->getGlobalLinkTransform());
  // bodies::ConvexMesh* cm = dynamic_cast<bodies::ConvexMesh*>(b);
  // inf_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  // inf_marker.scale.x = inf_marker.scale.y = inf_marker.scale.z = 1.0;
  // inf_marker.color.a = inf_marker.color.b = 1.0;
  // inf_marker.header.frame_id = planning_scene_monitor_->getPlanningScene()->getPlanningFrame(); 
  // inf_marker.pose.orientation.w = 1.0;
  // for(unsigned int i = 0; i < cm->getTriangles().size(); i+=3) {
  //   // ROS_INFO_STREAM("Triangle " << i << " is " << cm->getTriangles()[i] << " " 
  //   //                 << cm->getTriangles()[i+1] << " " 
  //   //                 << cm->getTriangles()[i+2]); 
  //   geometry_msgs::Point p;
  //   p.x = cm->getVertices()[cm->getTriangles()[i]].x();
  //   p.y = cm->getVertices()[cm->getTriangles()[i]].y();
  //   p.z = cm->getVertices()[cm->getTriangles()[i]].z();
  //   inf_marker.points.push_back(p);
  //   p.x = cm->getVertices()[cm->getTriangles()[i+1]].x();
  //   p.y = cm->getVertices()[cm->getTriangles()[i+1]].y();
  //   p.z = cm->getVertices()[cm->getTriangles()[i+1]].z();
  //   inf_marker.points.push_back(p);
  //   p.x = cm->getVertices()[cm->getTriangles()[i+2]].x();
  //   p.y = cm->getVertices()[cm->getTriangles()[i+2]].y();
  //   p.z = cm->getVertices()[cm->getTriangles()[i+2]].z();
  //   inf_marker.points.push_back(p);
  // }
  ros::WallRate r(1.0);
  while(ros::ok()) {
    vis_marker_publisher.publish(inf_marker);
    //vis_marker_array_publisher.publish(sphere_markers);
    vis_marker_array_publisher.publish(arrow_markers);
    r.sleep();
  }

  // std_msgs::ColorRGBA col;
  // col.r = col.g = col.b = .5;
  // col.a = 1.0;

  // std_msgs::ColorRGBA good_color;
  // good_color.a = 1.0;    
  // good_color.g = 1.0;    
  
  // std_msgs::ColorRGBA bad_color;
  // bad_color.a = 1.0;    
  // bad_color.r = 1.0;    

  // iov_.reset(new InteractiveObjectVisualization(planning_scene_monitor_->getPlanningScene(),
  //                                               interactive_marker_server_,
  //                                               col));
  
  // kv_->addMenuEntry("Add Cube", boost::bind(&addCubeCallback, _1));

  // iov_->setUpdateCallback(boost::bind(&updateCallback, _1));
   

  ros::waitForShutdown();
}

