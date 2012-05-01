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

#include <interactive_markers/tools.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MenuEntry.h>
#include <geometry_msgs/PoseStamped.h>
#include <planning_models/kinematic_state.h>
#include <planning_models/transforms.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_utils/shape_extents.h>
#include <shape_conversions/shape_to_marker.h>
#include <shape_msgs/Shape.h>

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

std_msgs::ColorRGBA makeRandomColor(float brightness, float alpha);

geometry_msgs::Pose determinePoseCentroidAndExtents(const std::vector<geometry_msgs::Pose>& poses,
                                                    double& xex,
                                                    double& yex,
                                                    double& zex);

visualization_msgs::Marker makeBox( float x, float y, float z);

visualization_msgs::Marker makeCylinder( float x, float z);

visualization_msgs::Marker makeSphere( float scale );

void add6DofControl( visualization_msgs::InteractiveMarker &msg, bool fixed );

void removeAxisControls(visualization_msgs::InteractiveMarker &msg);

visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg);

visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg,
                                                              float x, float y, float z);

visualization_msgs::InteractiveMarkerControl& makeCylinderControl( visualization_msgs::InteractiveMarker &msg);

visualization_msgs::InteractiveMarkerControl& makeCylinderControl(visualization_msgs::InteractiveMarker &msg,
                                                                  float x, float z);

visualization_msgs::InteractiveMarkerControl& makeSphereControl(visualization_msgs::InteractiveMarker &msg);

visualization_msgs::MenuEntry makeMenuEntry(const char *title);

visualization_msgs::MenuEntry makeMenuEntry(const char *title, const char *command, int type);

visualization_msgs::InteractiveMarker makeMeshMarker(const std::string &name, 
                                                     const std::string &mesh_resource,
                                                     const geometry_msgs::PoseStamped &stamped, 
                                                     float scale, 
                                                     const std_msgs::ColorRGBA &color, 
                                                     bool use_color);

visualization_msgs::InteractiveMarker makeMeshMarker(const std::string &name, 
                                                     const std::string &mesh_resource,
                                                     const geometry_msgs::PoseStamped &stamped, 
                                                     float scale);

visualization_msgs::InteractiveMarker makeMeshMarker(const std::string &name, 
                                                     const std::string &mesh_resource,
                                                     const geometry_msgs::PoseStamped &stamped, 
                                                     float scale, 
                                                     const std_msgs::ColorRGBA &color);

visualization_msgs::InteractiveMarker makeButtonBox(const std::string& name, 
                                                    const geometry_msgs::PoseStamped &stamped, 
                                                    float scale, 
                                                    bool fixed, 
                                                    bool view_facing);

visualization_msgs::InteractiveMarker makeButtonBox(const std::string& name, 
                                                    const geometry_msgs::PoseStamped &stamped, 
                                                    float x, float y, float z, 
                                                    bool fixed, 
                                                    bool view_facing);

visualization_msgs::InteractiveMarker makeButtonCylinder(const std::string& name, 
                                                         const geometry_msgs::PoseStamped &stamped, 
                                                         float scale, 
                                                         bool fixed, 
                                                         bool view_facing);

visualization_msgs::InteractiveMarker makeButtonCylinder(const std::string& name, 
                                                         const geometry_msgs::PoseStamped &stamped, 
                                                         float x, float z,  
                                                         bool fixed, 
                                                         bool view_facing);

visualization_msgs::InteractiveMarker makeButtonSphere(const std::string& name, 
                                                       const geometry_msgs::PoseStamped &stamped,
                                                       float scale, 
                                                       bool fixed, 
                                                       bool view_facing);

visualization_msgs::InteractiveMarker makeButtonPointMass(const std::string& name, 
                                                          const std::string& frame_id,
                                                          const std::vector<geometry_msgs::Pose>& points,
                                                          const std_msgs::ColorRGBA& color, 
                                                          float scale, 
                                                          bool fixed, 
                                                          bool view_facing);

visualization_msgs::InteractiveMarker makeButtonCompoundShape(const std::string& name, 
                                                              const std::string& frame_id,
                                                              const std::vector<shape_msgs::Shape>& shapes,
                                                              const std::vector<geometry_msgs::Pose>& poses,
                                                              const std_msgs::ColorRGBA& color, 
                                                              float scale, 
                                                              bool fixed, 
                                                              bool view_facing);

visualization_msgs::InteractiveMarker make6DOFMarker(const std::string& name, 
                                                     const geometry_msgs::PoseStamped &stamped, 
                                                     float scale, 
                                                     bool fixed, 
                                                     bool view_facing);

visualization_msgs::InteractiveMarker makeButtonMesh(const std::string& marker_name,
                                                     const shape_msgs::Shape& mesh_shape,
                                                     const geometry_msgs::PoseStamped &stamped,
                                                     const std_msgs::ColorRGBA& color);

visualization_msgs::InteractiveMarker makeMeshButtonFromLinks(const std::string& marker_name,
                                                              const planning_models::KinematicState& state,
                                                              const std::string& parent_link,
                                                              const std::vector<std::string>& links,
                                                              const std_msgs::ColorRGBA& color, 
                                                              const double scale, 
                                                              bool use_color, 
                                                              Eigen::Affine3d& relative_transform);

void recolorInteractiveMarker(visualization_msgs::InteractiveMarker& marker,
                              const std_msgs::ColorRGBA& color); 

}
#endif
