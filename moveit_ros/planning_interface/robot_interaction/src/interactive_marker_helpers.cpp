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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#include <moveit/robot_interaction/interactive_marker_helpers.h>
#include <tf/transform_datatypes.h>

namespace robot_interaction
{

void addArrowMarker(visualization_msgs::InteractiveMarker &im)
{
  // create a grey box marker
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.scale.x = 0.8 * im.scale;
  m.scale.y = 0.15 * im.scale;
  m.scale.z = 0.15 * im.scale;
  m.ns = "goal_pose_arrow_marker";
  m.id = 1;
  m.action = visualization_msgs::Marker::ADD;
  m.header = im.header;
  m.pose = im.pose;
  //Arrow points along Z
  tf::Quaternion imq;
  tf::quaternionMsgToTF(m.pose.orientation, imq);
  imq=imq*tf::createQuaternionFromRPY(0, -M_PI_2, 0);
  tf::quaternionTFToMsg(imq, m.pose.orientation);
  m.color.r = 0.0f;
  m.color.g = 1.0f;
  m.color.b = 0.0f;
  m.color.a = 1.0f;
  
  visualization_msgs::InteractiveMarkerControl m_control;
  m_control.always_visible = true;
  m_control.interaction_mode=m_control.BUTTON;
  m_control.markers.push_back(m);

  // add the control to the interactive marker
  im.controls.push_back(m_control);
}

void addErrorMarker(visualization_msgs::InteractiveMarker &im)
{
  // create a grey box marker
  visualization_msgs::Marker err;
  err.type = visualization_msgs::Marker::MESH_RESOURCE;
  err.scale.x = 0.002 * im.scale;
  err.scale.y = 0.002 * im.scale;
  err.scale.z = 0.002 * im.scale;
  err.mesh_resource = "package://moveit_ros_planning_interface/resources/access-denied.dae";
  err.ns = "robot_interaction_error";
  err.id = 1;
  err.action = visualization_msgs::Marker::ADD;
  err.header = im.header;
  err.pose = im.pose;  
  err.pose.orientation.x = err.pose.orientation.y = 0.7071067811865476;
  err.pose.orientation.z = err.pose.orientation.w = 0.0;
  err.color.r = 1.0f;
  err.color.g = 0.0f;
  err.color.b = 0.0f;
  err.color.a = 1.0f;
  
  visualization_msgs::InteractiveMarkerControl err_control;
  err_control.always_visible = false;
  err_control.markers.push_back(err);

  // add the control to the interactive marker
  im.controls.push_back(err_control);
}

visualization_msgs::InteractiveMarker make3DOFMarker(const std::string& name,
                                                     const geometry_msgs::PoseStamped &stamped, 
                                                     double scale,
                                                     bool fixed)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;
  
  visualization_msgs::InteractiveMarkerControl control;
  
  if (fixed)
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  
  return int_marker;
}

visualization_msgs::InteractiveMarker make6DOFMarker(const std::string& name,
                                                     const geometry_msgs::PoseStamped &stamped, 
                                                     double scale,
                                                     bool fixed)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;
  
  visualization_msgs::InteractiveMarkerControl control;
  
  if (fixed)
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  
  return int_marker;
}

}
