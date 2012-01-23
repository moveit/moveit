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

#ifndef _INTERACTIVE_OBJECT_VISUALIZATION_H_
#define _INTERACTIVE_OBJECT_VISUALIZATION_H_

#include <ros/ros.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <moveit_visualization_ros/interactive_marker_helper_functions.h>

static const double DEFAULT_SCALE = .1;
static const double DEFAULT_X = .5;
static const double DEFAULT_Y = 0.0;
static const double DEFAULT_Z = .5;

namespace moveit_visualization_ros
{

class InteractiveObjectVisualization {
public:
  
  InteractiveObjectVisualization(planning_scene::PlanningSceneConstPtr planning_scene,
                                 boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server, 
                                 const std_msgs::ColorRGBA& color) 
    : planning_scene_(planning_scene),
      interactive_marker_server_(interactive_marker_server)
  {
    planning_scene_diff_.reset(new planning_scene::PlanningScene(planning_scene_));
  }

  ~InteractiveObjectVisualization() {
  }

  planning_scene::PlanningSceneConstPtr addCube(void) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = planning_scene_->getPlanningFrame();
    pose.pose.position.x = DEFAULT_X;
    pose.pose.position.y = DEFAULT_Y;
    pose.pose.position.z = DEFAULT_Z;
    pose.pose.orientation.w = 1.0;

    Eigen::Affine3d aff;
    planning_models::poseFromMsg(pose.pose, aff);
    planning_scene_diff_->getCollisionWorld()->addToObject("cube1", new shapes::Box(DEFAULT_SCALE,
                                                                                    DEFAULT_SCALE,
                                                                                    DEFAULT_SCALE),
                                                          aff);

    visualization_msgs::InteractiveMarker marker = makeButtonBox("cube1",
                                                                 pose,
                                                                 DEFAULT_SCALE,
                                                                 false, 
                                                                 false);
    add6DofControl(marker, false);
    interactive_marker_server_->insert(marker);
    interactive_marker_server_->setCallback(marker.name, 
                                            boost::bind(&InteractiveObjectVisualization::processInteractiveMarkerFeedback, this, _1));

    interactive_marker_server_->applyChanges();
    return planning_scene_diff_;
  }

  void setUpdateCallback(const boost::function<void(planning_scene::PlanningSceneConstPtr)>& callback) {
    update_callback_ = callback;
  }

  void updateObjectPose(const std::string& name,
                        const geometry_msgs::Pose& pose) 
  {
    collision_detection::CollisionWorld::ObjectConstPtr obj = planning_scene_diff_->getCollisionWorld()->getObject(name);
    if(!obj) {
      ROS_WARN_STREAM("No object with name " << name);
      return;
    }
    Eigen::Affine3d aff;
    planning_models::poseFromMsg(pose, aff);
    planning_scene_diff_->getCollisionWorld()->moveShapeInObject(name, obj->shapes_[0], aff);

    callUpdateCallback();
  }

protected:

  void callUpdateCallback() {
    if(update_callback_) {
      update_callback_(planning_scene_diff_);
    }
  }

  void processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) 
  {    
    ROS_INFO_STREAM("Processing feedback for " << feedback->marker_name);
    switch (feedback->event_type) {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      updateObjectPose(feedback->marker_name, feedback->pose);
      break;
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      break;
    default:
      ROS_DEBUG_STREAM("Getting event type " << feedback->event_type);
    }
    interactive_marker_server_->applyChanges();
  }; 
  
  planning_scene::PlanningSceneConstPtr planning_scene_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;

  planning_scene::PlanningScenePtr planning_scene_diff_;

  boost::function<void(planning_scene::PlanningSceneConstPtr)> update_callback_;

};

}

#endif
