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
#include <distance_field/propagation_distance_field.h>
#include <moveit_visualization_ros/kinematic_state_joint_state_publisher.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <collision_distance_field/collision_distance_field_types.h>
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

class MovingMarkers {

public:

  MovingMarkers(boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                const std::string& world_frame,
                double x_size, double y_size, double z_size) :
    interactive_marker_server_(interactive_marker_server),
    world_frame_(world_frame),
    x_size_(x_size), y_size_(y_size), z_size_(z_size)
  {
    update_thread_.reset(new boost::thread(boost::bind(&MovingMarkers::updateAllMarkers, this)));
  }
  
  void addSphere() {
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = world_frame_;
    ps.pose.position.x = 1.0;
    ps.pose.position.y = 1.0;
    ps.pose.position.z = 1.0;
    ps.pose.orientation.w = 1.0;
    markers_["sphere"] = makeButtonSphere("sphere",
                                          ps,
                                          .1,
                                          false,
                                          false);
    interactive_marker_server_->insert(markers_["sphere"]);
    velocities_["sphere"] = Eigen::Translation3d(1.0,0.0,0.0);
  }    

  // TODO- add mutex
  bool getFirstSpherePosition(Eigen::Translation3d& translation)
  {
    std::map<std::string, visualization_msgs::InteractiveMarker>::iterator it = markers_.begin();
    if( it != markers_.end() )
    {
      Eigen::Affine3d pose_e;
      planning_models::poseFromMsg(it->second.pose, pose_e);
      translation.x() = pose_e.translation().x();
      translation.y() = pose_e.translation().y();
      translation.z() = pose_e.translation().z();
      //transform.translation = pose_e.translation;
      return true;
    }
    return false;
  }

  void updateAllMarkers() {
    while(ros::ok()) {
      for(std::map<std::string, visualization_msgs::InteractiveMarker>::iterator it = markers_.begin();
          it != markers_.end();
          it++) {
        Eigen::Affine3d pose_e; 
        planning_models::poseFromMsg(it->second.pose, pose_e);
        Eigen::Translation3d m(velocities_[it->first].x()*.05,
                               velocities_[it->first].y()*.05,
                               velocities_[it->first].z()*.05);
        pose_e = pose_e*m;
        if(pose_e.translation().x() > x_size_ || pose_e.translation().x() < -x_size_) {
          velocities_[it->first].x() *= -1;
        }
        if(pose_e.translation().y() > y_size_ || pose_e.translation().y() < -y_size_) {
          velocities_[it->first].y() *= -1;
        }
        if(pose_e.translation().z() > z_size_ || pose_e.translation().z() < -z_size_) {
          velocities_[it->first].z() *= -1;
        }
        planning_models::msgFromPose(pose_e, it->second.pose);
        interactive_marker_server_->insert(it->second);
        interactive_marker_server_->applyChanges();
      }
      boost::this_thread::sleep((ros::WallTime::now()+ros::WallDuration(.05)).toBoost());
    }
  }
  
protected:

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;  
  std::string world_frame_;
  double x_size_, y_size_, z_size_;
  boost::shared_ptr<boost::thread> update_thread_;
  std::map<std::string, Eigen::Translation3d> velocities_; 
  std::map<std::string, visualization_msgs::InteractiveMarker> markers_;

};

// void updateCallback(planning_scene::PlanningSceneConstPtr planning_scene) {
//   kv_->updatePlanningScene(planning_scene);
// }

// void addCubeCallback(const std::string& name) {
//   iov_->addCube();
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_object_visualization", ros::init_options::NoSigintHandler);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  interactive_marker_server_.reset(new interactive_markers::InteractiveMarkerServer("interactive_distance_field_visualization", "", false));
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  joint_state_publisher_.reset(new KinematicStateJointStatePublisher());

  boost::thread publisher_thread(boost::bind(&publisher_function));

  ros::NodeHandle nh;

  ros::Publisher vis_marker_array_publisher;
  ros::Publisher vis_marker_publisher;

  vis_marker_publisher = nh.advertise<visualization_msgs::Marker> (VIS_TOPIC_NAME, 128);
  vis_marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray> (VIS_TOPIC_NAME + "_array", 128);

  time_t start, diff;
  const double resolution = 0.025;
  const double max_distance = 0.5;
  const bool 	 iterative = true;
  distance_field::PropagationDistanceField distance_field(3.0, 3.0, 4.0, resolution, -1.0, -1.5, -2.0, max_distance);

  shapes::Box* box = new shapes::Box(2.5, 2.5, 0.4);
  collision_distance_field::BodyDecomposition bd("box", box, resolution, 0.0);
  Eigen::Affine3d trans(Eigen::Translation3d(0.0,0.0,-1.0)*Eigen::Quaterniond::Identity());
  bd.updatePose(trans);
  std::vector<Eigen::Vector3d> table_points = bd.getCollisionPoints();
  std::vector<Eigen::Vector3d> sphere_points;
  //sphere_points.push_back(bd.getCollisionPoints()[0]);// TODO: this seems the only way to not segfault...
  //sphere_points.clear();

  //ROS_INFO_STREAM("Adding " << table_points.size() << " to field");

  distance_field.addPointsToField(table_points);

  visualization_msgs::Marker mark;

  MovingMarkers mm(interactive_marker_server_,
                   planning_scene_monitor_->getPlanningScene()->getPlanningFrame(),
                   1.5,1.5,2.0);
  mm.addSphere();

  shapes::Sphere* sphere = new shapes::Sphere(0.1);
  collision_distance_field::BodyDecomposition sbd("sphere", sphere, 0.025, 0.0);

  int i = 0;
  while(1) {
    Eigen::Translation3d translation;
    if( mm.getFirstSpherePosition(translation) )
    {
      // If using only the sphere's center point
      //Eigen::Vector3d point(translation.x(), translation.y(), translation.z());
      //ROS_INFO_STREAM( "transform="<<point.x()<<","<<point.y()<<","<<point.z()<<std::endl );

      // If using the fully decomposed sphere body full sphere bodies
      Eigen::Affine3d trans(translation*Eigen::Quaterniond::Identity());
      sbd.updatePose(trans);

      start = clock();

      if( iterative )
      {
        // Just add/remove the sphere points
        if( sphere_points.size() >0 )
        {
          distance_field.removePointsFromField(sphere_points);
          sphere_points.clear();
        }

        sphere_points = sbd.getCollisionPoints();
        distance_field.addPointsToField(sphere_points);
      }
      else
      {
        // Update the whole voxel map
        sphere_points = sbd.getCollisionPoints();
        distance_field.updatePointsInField(table_points, false);
        distance_field.addPointsToField(sphere_points);
      }

      diff = start-clock();
      std::cout<< "timeToUpdate=" << (double)diff/CLOCKS_PER_SEC << std::endl;
      start = clock();
      distance_field.getProjectionPlanes( planning_scene_monitor_->getPlanningScene()->getPlanningFrame(),
                                          ros::Time::now(), max_distance,
                                          mark);
      //distance_field.getIsoSurfaceMarkers(0.0000,0.5,planning_scene_monitor_->getPlanningScene()->getPlanningFrame(),
      //                                    ros::Time::now(),
      //                                    Eigen::Affine3d::Identity(),
      //                                    mark);

      diff = start-clock();
      std::cout<< "timeToProject=" << (double)diff/CLOCKS_PER_SEC << std::endl;

      ROS_INFO_STREAM( "num_points="<<table_points.size()+sphere_points.size()<<std::endl );
    }

    vis_marker_publisher.publish(mark);
    i++;
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

