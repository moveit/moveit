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

// Author: E. Gil Jones, Adam Leeper

#ifndef _INTERACTIVE_MARKER_HELPER_FUNCTIONS_H_
#define _INTERACTIVE_MARKER_HELPER_FUNCTIONS_H_

#include <cmath>

#include <interactive_markers/tools.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MenuEntry.h>
#include <geometry_msgs/PoseStamped.h>
#include <planning_models/kinematic_state.h>
#include <planning_models/transforms.h>

static bool done_seed = false;

namespace moveit_visualization_ros
{

// class MenuEntryHelper {

//   MenuEntryMap(const std::string& name) :
//     name_(name),
//     has_children_(false),
//     is_leaf_(false)
//   {
//   };

//   MenuEntryMap(const std::string<std::vector>& lineage,
//                const std::string& name,
//                const boost::function<void(const std::vector<std::string>&) callback) :
//     name_(name),
//     full_lineage_(parent->getLineage()),
//     is_leaf_(true)
//   {
//     full_lineage_.push_back(name);
//   };

//   void addChild(const std::string& child_name) {
//     has_children_ = true;
//     children_[child_name] = MenuEntryHelper(child_name);
//   }

//   void addChild(const std::string& child_name,
//                 const boost::function<void(const std::vector<std::string>&) callback) {
//     has_children_ = true;
//     children_[child_name] = MenuEntryHelper(full_lineage_
//   }
  
// protected:

//   std::vector<std::string> name_;
//   std::vector<std::string> full_lineage_;

//   bool has_children_;
//   std::map<std::string, MenuEntryHelper> children_;
  
//   boost::function<void(const std::vector<std::string>&)> callback_function_; 

// };

inline std_msgs::ColorRGBA makeRandomColor(float brightness, float alpha)
{
  if(!done_seed) {
    done_seed = true;
    srand(time(0));
  }

  std_msgs::ColorRGBA toReturn;
  toReturn.a = alpha;

  toReturn.r = ((float)(random()) / (float)RAND_MAX) * (1.0f - brightness) + brightness;
  toReturn.g = ((float)(random()) / (float)RAND_MAX) * (1.0f - brightness) + brightness;
  toReturn.b = ((float)(random()) / (float)RAND_MAX) * (1.0f - brightness) + brightness;

  toReturn.r = fmin(toReturn.r, 1.0f);
  toReturn.g = fmin(toReturn.g, 1.0f);
  toReturn.b = fmin(toReturn.b, 1.0f);

  return toReturn;
}


inline visualization_msgs::Marker makeBox( float scale )
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  return marker;
}

inline visualization_msgs::Marker makeSphere( float scale )
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  return marker;
}

inline void add6DofControl( visualization_msgs::InteractiveMarker &msg, bool fixed )
{
  visualization_msgs::InteractiveMarkerControl control;

  if(fixed)
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control);
}


inline visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg.scale) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

inline visualization_msgs::InteractiveMarkerControl& makeSphereControl( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeSphere(msg.scale) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

inline visualization_msgs::MenuEntry makeMenuEntry(const char *title)
{
  visualization_msgs::MenuEntry m;
  m.title = title;
  m.command = title;
  return m;
}

inline visualization_msgs::MenuEntry makeMenuEntry(const char *title, const char *command, int type  )
{
  visualization_msgs::MenuEntry m;
  m.title = title;
  m.command = command;
  m.command_type = type;
  return m;
}

inline visualization_msgs::InteractiveMarker makeMeshMarker(const std::string &name, 
                                                     const std::string &mesh_resource,
                                                     const geometry_msgs::PoseStamped &stamped, 
                                                     float scale, 
                                                     const std_msgs::ColorRGBA &color, 
                                                     bool use_color)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.pose = stamped.pose;
  int_marker.name = name;
  int_marker.scale = scale;

  visualization_msgs::Marker mesh;
  mesh.color = color;
  mesh.mesh_resource = mesh_resource;
  mesh.mesh_use_embedded_materials = !use_color;
  mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh.scale.x = scale;
  mesh.scale.y = scale;
  mesh.scale.z = scale;

  visualization_msgs::InteractiveMarkerControl control;
  control.markers.push_back( mesh );
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  int_marker.controls.push_back( control );

  return int_marker;
}

inline visualization_msgs::InteractiveMarker makeMeshMarker(const std::string &name, 
                                                     const std::string &mesh_resource,
                                                     const geometry_msgs::PoseStamped &stamped, 
                                                     float scale)
{
  std_msgs::ColorRGBA color;
  return makeMeshMarker(name, mesh_resource, stamped, scale, color, false);
}

inline visualization_msgs::InteractiveMarker makeMeshMarker(const std::string &name, 
                                                     const std::string &mesh_resource,
                                                     const geometry_msgs::PoseStamped &stamped, 
                                                     float scale, 
                                                     const std_msgs::ColorRGBA &color)
{
  return makeMeshMarker(name, mesh_resource, stamped, scale, color, true);
}

inline visualization_msgs::InteractiveMarker makeButtonBox(const std::string& name, 
                                                    const geometry_msgs::PoseStamped &stamped, 
                                                    float scale, 
                                                    bool fixed, 
                                                    bool view_facing)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;
  //int_marker.description = "This is the marker.";

  visualization_msgs::InteractiveMarkerControl &control = makeBoxControl(int_marker);
  //control.description = "This is the control";
  control.always_visible = false;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

  return int_marker;
}

inline visualization_msgs::InteractiveMarker makeButtonSphere(const std::string& name, 
                                                       const geometry_msgs::PoseStamped &stamped,
                                                       float scale, 
                                                       bool fixed, 
                                                       bool view_facing,
                                                       std_msgs::ColorRGBA color)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;
  //int_marker.description = "This is the marker.";

  visualization_msgs::InteractiveMarkerControl &control = makeSphereControl(int_marker);
  //control.description = "This is the control";
  control.always_visible = false;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.markers.back().color = color;

  return int_marker;
}

inline visualization_msgs::InteractiveMarker makeButtonSphere(const std::string& name, 
                                                       const geometry_msgs::PoseStamped &stamped,
                                                       float scale, 
                                                       bool fixed, 
                                                       bool view_facing)
{
  std_msgs::ColorRGBA color;
  return makeButtonSphere(name, stamped, scale, fixed, view_facing, color);
}

inline visualization_msgs::InteractiveMarker make6DOFMarker(const std::string& name, 
                                                     const geometry_msgs::PoseStamped &stamped, 
                                                     float scale, 
                                                     bool fixed, 
                                                     bool view_facing)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;

  if ( view_facing )
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
    control.orientation.w = 1;
    int_marker.controls.push_back(control);

    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    control.markers.push_back( makeSphere(scale*0.5) );
    int_marker.controls.push_back(control);
  }
  else
  {
    add6DofControl(int_marker, fixed);
  }

  return int_marker;
}

inline visualization_msgs::InteractiveMarker makeMeshButtonFromLinks(const std::string& marker_name,
                                                                     const planning_models::KinematicState& state,
                                                                     const std::vector<std::string>& links,
                                                                     const std_msgs::ColorRGBA& color, 
                                                                     const double scale, 
                                                                     bool use_color) {
  
  visualization_msgs::InteractiveMarker int_marker;
  if(links.size() == 0) return int_marker;

  //assumes that center is first marker
  const planning_models::KinematicState::LinkState* first_link = state.getLinkState(links[0]);
  Eigen::Affine3d first_pose = first_link->getGlobalCollisionBodyTransform();
  planning_models::msgFromPose(first_pose, int_marker.pose);
  
  int_marker.header.frame_id = "/"+state.getKinematicModel()->getModelFrame();
  int_marker.name = marker_name;
  int_marker.scale = .5; //scale;
  
  //  add6DofControl(int_marker, false);

  visualization_msgs::InteractiveMarkerControl control;

  visualization_msgs::Marker mesh;
  //header intentionally left empty, meaning that poses are relative to marker

  mesh.mesh_use_embedded_materials = !use_color;
  mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh.scale.x = 1.0;
  mesh.scale.y = 1.0;
  mesh.scale.z = 1.0;
  mesh.color = color;

  for(unsigned int i = 0; i < links.size(); i++) {

    const planning_models::KinematicState::LinkState* ls = state.getLinkState(links[i]);
    if(ls == NULL) {
      ROS_WARN_STREAM("No link state for requested link " << links[i]);
      continue;
    }
    if(ls->getLinkModel()->getFilename().empty()) {
      ROS_DEBUG_STREAM("No filename for " << links[i]);
      continue;
    }

    mesh.mesh_resource = ls->getLinkModel()->getFilename();
    //getting pose relative to first pose
    Eigen::Affine3d ret_pose = first_pose.inverse()*ls->getGlobalCollisionBodyTransform();

    planning_models::msgFromPose(ret_pose, mesh.pose);
    control.markers.push_back(mesh);
  }

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  int_marker.controls.push_back( control );

  return int_marker;

}

}
#endif
