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

#include <cmath>
#include <float.h>

#include <moveit_visualization_ros/interactive_marker_helper_functions.h>
#include <ros/console.h>

static bool done_seed = false;

namespace moveit_visualization_ros
{

std_msgs::ColorRGBA makeRandomColor(float brightness, float alpha)
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

geometry_msgs::Pose determinePoseCentroidAndExtents(const std::vector<geometry_msgs::Pose>& poses,
                                                    double& xex,
                                                    double& yex,
                                                    double& zex)
{
  double x = 0.0, y = 0.0, z = 0.0;
  double xmin = DBL_MAX, ymin = DBL_MAX, zmin = DBL_MAX;
  double xmax = -DBL_MAX, ymax = -DBL_MAX, zmax = -DBL_MAX;
  for(unsigned int i = 0; i < poses.size(); i++) {
    double xval = poses[i].position.x;
    double yval = poses[i].position.y;
    double zval = poses[i].position.z;
    x += xval;
    y += yval;
    z += zval;
    if(xval < xmin) {
      xmin = xval; 
    }
    if(xval > xmax) {
      xmax = xval; 
    }
    
    if(yval < ymin) {
      ymin = yval; 
    }
    if(yval > ymax) {
      ymax = yval; 
    }
    
    if(zval < zmin) {
      zmin = zval; 
    }
    if(zval > zmax) {
      zmax = zval; 
    }
  }

  xex = fabs(xmax-xmin);
  yex = fabs(ymax-ymin);
  zex = fabs(zmax-zmin);

  geometry_msgs::Pose ret_pose;
  ret_pose.orientation.w = 1.0;

  ret_pose.position.x = (x/(poses.size()*1.0));
  ret_pose.position.y = (y/(poses.size()*1.0));
  ret_pose.position.z = (z/(poses.size()*1.0));

  return ret_pose;
}

visualization_msgs::Marker makeBox( float x, float y, float z)
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = x;
  marker.scale.y = y;
  marker.scale.z = z;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::Marker makeCylinder( float x, float z)
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.scale.x = x;
  marker.scale.y = x;
  marker.scale.z = z;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::Marker makeSphere( float scale )
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

visualization_msgs::Marker makeArrow( const geometry_msgs::Point &base, const geometry_msgs::Point &tip )
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::ARROW;
  float length = sqrt( pow(tip.x-base.x,2) + pow(tip.y-base.y,2) + pow(tip.z-base.z,2));
  marker.scale.x = length/12.0;  //shaft radius
  marker.scale.y = length/7.0;  // head radius  // what about head length?
  //marker.scale.z = scale;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  marker.points.push_back(base);
  marker.points.push_back(tip);

  return marker;
}

void add6DofControl( visualization_msgs::InteractiveMarker &msg, bool fixed )
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

void removeAxisControls(visualization_msgs::InteractiveMarker &msg)
{
  std::vector<visualization_msgs::InteractiveMarkerControl>::iterator it = msg.controls.begin();
  while(it != msg.controls.end()) {
    if(it->interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS ||
       it->interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_AXIS) {
      it = msg.controls.erase(it);
    } else {
      it++;    
    }  
  }
}

visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg.scale, msg.scale, msg.scale) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg,
                                                                     float x, float y, float z)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(x, y, z) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

visualization_msgs::InteractiveMarkerControl& makeCylinderControl( visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeCylinder(msg.scale, msg.scale) );
  msg.controls.push_back( control );

  return msg.controls.back();
}


visualization_msgs::InteractiveMarkerControl& makeCylinderControl( visualization_msgs::InteractiveMarker &msg,
                                                                          float x, float z)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeCylinder(x, z) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

visualization_msgs::InteractiveMarkerControl& makeSphereControl( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeSphere(msg.scale) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

visualization_msgs::MenuEntry makeMenuEntry(const char *title)
{
  visualization_msgs::MenuEntry m;
  m.title = title;
  m.command = title;
  return m;
}

visualization_msgs::MenuEntry makeMenuEntry(const char *title, const char *command, int type  )
{
  visualization_msgs::MenuEntry m;
  m.title = title;
  m.command = command;
  m.command_type = type;
  return m;
}

visualization_msgs::InteractiveMarker makeMeshMarker(const std::string &name, 
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

visualization_msgs::InteractiveMarker makeMeshMarker(const std::string &name, 
                                                     const std::string &mesh_resource,
                                                     const geometry_msgs::PoseStamped &stamped, 
                                                     float scale)
{
  std_msgs::ColorRGBA color;
  return makeMeshMarker(name, mesh_resource, stamped, scale, color, false);
}

visualization_msgs::InteractiveMarker makeMeshMarker(const std::string &name, 
                                                     const std::string &mesh_resource,
                                                     const geometry_msgs::PoseStamped &stamped, 
                                                     float scale, 
                                                     const std_msgs::ColorRGBA &color)
{
  return makeMeshMarker(name, mesh_resource, stamped, scale, color, true);
}

visualization_msgs::InteractiveMarker makeButtonBox(const std::string& name, 
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

visualization_msgs::InteractiveMarker makeButtonBox(const std::string& name, 
                                                           const geometry_msgs::PoseStamped &stamped, 
                                                           float x, float y, float z, 
                                                           bool fixed, 
                                                           bool view_facing)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = fmax(x, fmax(y, z));
  int_marker.pose = stamped.pose;
  //int_marker.description = "This is the marker.";

  visualization_msgs::InteractiveMarkerControl &control = makeBoxControl(int_marker,
                                                                         x, y, z);
  //control.description = "This is the control";
  control.always_visible = false;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

  return int_marker;
}

visualization_msgs::InteractiveMarker makeButtonCylinder(const std::string& name, 
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

  visualization_msgs::InteractiveMarkerControl &control = makeCylinderControl(int_marker);
  //control.description = "This is the control";
  control.always_visible = false;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

  return int_marker;
}

visualization_msgs::InteractiveMarker makeButtonCylinder(const std::string& name, 
                                                                const geometry_msgs::PoseStamped &stamped, 
                                                                float x, float z,  
                                                                bool fixed, 
                                                                bool view_facing)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = fmax(x, z);
  int_marker.pose = stamped.pose;
  //int_marker.description = "This is the marker.";

  visualization_msgs::InteractiveMarkerControl &control = makeCylinderControl(int_marker, x, z);
  //control.description = "This is the control";
  control.always_visible = false;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

  return int_marker;
}

visualization_msgs::InteractiveMarker makeButtonSphere(const std::string& name, 
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

  visualization_msgs::InteractiveMarkerControl &control = makeSphereControl(int_marker);
  //control.description = "This is the control";
  control.always_visible = false;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

  return int_marker;
}

visualization_msgs::InteractiveMarker makeButtonPointMass(const std::string& name, 
                                                          const std::string& frame_id,
                                                          const std::vector<geometry_msgs::Pose>& points,
                                                          const std_msgs::ColorRGBA& color, 
                                                          float scale, 
                                                          bool fixed, 
                                                          bool view_facing)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.name = name;
  //int_marker.description = "This is the marker.";

  visualization_msgs::InteractiveMarkerControl control;
  visualization_msgs::Marker sphere_list;
  sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
  sphere_list.color = color;
  sphere_list.scale.x = sphere_list.scale.y = sphere_list.scale.z = scale;
  sphere_list.pose.orientation.w = 1.0;

  double xex, yex, zex;
  int_marker.pose = determinePoseCentroidAndExtents(points,
                                                    xex, yex, zex);
  int_marker.scale = fmax(xex, fmax(yex, zex))*1.05;

  Eigen::Affine3d trans;
  planning_models::poseFromMsg(int_marker.pose, trans);

  for(unsigned int i = 0; i < points.size(); i++) {
    Eigen::Affine3d point_pose(Eigen::Translation3d(points[i].position.x, points[i].position.y, points[i].position.z)*Eigen::Quaterniond::Identity());
    Eigen::Affine3d trans_pose = trans.inverse()*point_pose;
    geometry_msgs::Pose conv;
    planning_models::msgFromPose(trans_pose, conv);
    sphere_list.points.push_back(conv.position);
  }
  control.markers.push_back(sphere_list);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  int_marker.controls.push_back( control );
  return int_marker;
}

class ShapeVisitorMarker : public boost::static_visitor<void>
{
public:
  
  ShapeVisitorMarker(visualization_msgs::Marker *marker, bool use_mesh_triangle_list) :
    boost::static_visitor<void>(),
    use_mesh_triangle_list_(use_mesh_triangle_list),
    marker_(marker)
  {
  }
  
  void operator()(const shape_msgs::Plane &shape_msg) const
  {
    throw std::runtime_error("No visual markers can be constructed for planes");
  }
  
  void operator()(const shape_msgs::Mesh &shape_msg) const
  {
    shape_tools::constructMarkerFromShape(shape_msg, *marker_, use_mesh_triangle_list_);
  }
  
  void operator()(const shape_msgs::SolidPrimitive &shape_msg) const
  {
    shape_tools::constructMarkerFromShape(shape_msg, *marker_);
  }
  
private:
  
  bool use_mesh_triangle_list_;
  visualization_msgs::Marker *marker_;
};

visualization_msgs::InteractiveMarker makeButtonCompoundShape(const std::string& name, 
                                                              const std::string& frame_id,
                                                              const std::vector<shapes::ShapeMsg>& shapes,
                                                              const std::vector<geometry_msgs::Pose>& poses,
                                                              const std_msgs::ColorRGBA& color, 
                                                              float scale, 
                                                              bool fixed, 
                                                              bool view_facing)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.name = name;
  //int_marker.description = "This is the marker.";
  
  visualization_msgs::InteractiveMarkerControl control;
  double xex, yex, zex;
  int_marker.pose = determinePoseCentroidAndExtents(poses,
                                                    xex, yex, zex);
  int_marker.scale = fmax(xex, fmax(yex, zex))*1.05;
  Eigen::Affine3d trans;
  planning_models::poseFromMsg(int_marker.pose, trans);

  for(unsigned int i = 0; i < shapes.size(); i++) {
    visualization_msgs::Marker mk;
    try
    {      
      boost::apply_visitor(ShapeVisitorMarker(&mk, false), shapes[i]);
    }
    catch(std::runtime_error &ex)
    {
      ROS_WARN_STREAM("Problem with shape index " << i << ": " << ex.what());
      continue;
    }    
    Eigen::Affine3d shape_pose;
    planning_models::poseFromMsg(poses[i], shape_pose);
    Eigen::Affine3d trans_pose = trans.inverse()*shape_pose;
    planning_models::msgFromPose(trans_pose, mk.pose);
    control.markers.push_back(mk);
  }
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  int_marker.controls.push_back( control );
  return int_marker;
}

visualization_msgs::InteractiveMarker make6DOFMarker(const std::string& name, 
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

visualization_msgs::InteractiveMarker makeButtonMesh(const std::string& marker_name,
                                                     const shape_msgs::Mesh& mesh_shape,
                                                     const geometry_msgs::PoseStamped &stamped,
                                                     const std_msgs::ColorRGBA& color)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = stamped.header;
  int_marker.name = marker_name;
  int_marker.pose = stamped.pose;

  visualization_msgs::InteractiveMarkerControl control;

  visualization_msgs::Marker mesh_mark;
  mesh_mark.mesh_use_embedded_materials = false;
  
  try
  {
    shape_tools::constructMarkerFromShape(mesh_shape, mesh_mark, true);
  }
  catch(std::runtime_error &ex)
  {
    ROS_WARN_STREAM("Some problem constructing mesh marker " << marker_name << ": " << ex.what());
    return int_marker;
  }
  double x, y, z;
  shape_tools::getShapeExtents(mesh_shape, x, y, z);
  if( x*y*z <= 0.0) {
    ROS_WARN_STREAM("Some problem with marker extents");
    int_marker.scale = 1.0;
  } else {
    double max;
    max = fmax(x, fmax(y, z));
    int_marker.scale = .05+max;
  }
  mesh_mark.color = color;
  control.markers.push_back(mesh_mark);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  int_marker.controls.push_back( control );
  return int_marker;
}

visualization_msgs::InteractiveMarker makeMeshButtonFromLinks(const std::string& marker_name,
                                                              const planning_models::KinematicState& state,
                                                              const std::string& parent_link,
                                                              const std::vector<std::string>& links,
                                                              const std_msgs::ColorRGBA& color, 
                                                              const double scale, 
                                                              bool use_color, 
                                                              Eigen::Affine3d& relative_transform) {
  
  visualization_msgs::InteractiveMarker int_marker;
  if(links.size() == 0) {
    ROS_WARN_STREAM("No links for marker creation");
    return int_marker;
  }

  const planning_models::KinematicState::LinkState* parent_link_state = state.getLinkState(parent_link);
  Eigen::Affine3d parent_pose = parent_link_state->getGlobalLinkTransform();

  const planning_models::KinematicState::LinkState* first_link_state = state.getLinkState(links[0]);
  //Eigen::Affine3d first_pose = first_link_state->getGlobalLinkTransform();

  //ROS_INFO_STREAM("Parent link " << parent_link << " first link " << links[0]);
  
  //Eigen::Affine3d init_transform = parent_pose.inverse()*first_pose;
  Eigen::Affine3d init_transform = Eigen::Affine3d::Identity();

  int_marker.header.frame_id = "/"+state.getKinematicModel()->getModelFrame();
  int_marker.name = marker_name;
  
  //  add6DofControl(int_marker, false);

  visualization_msgs::InteractiveMarkerControl control;

  visualization_msgs::Marker mesh;
  //header intentionally left empty, meaning that poses are relative to marker

  mesh.mesh_use_embedded_materials = false;
  mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh.scale.x = 1.0;
  mesh.scale.y = 1.0;
  mesh.scale.z = 1.0;
  mesh.color = color;

  std::vector<const bodies::Body*> bodies;
  double x = 0.0, y = 0.0, z = 0.0;
  double xmin = DBL_MAX, ymin = DBL_MAX, zmin = DBL_MAX;
  double xmax = -DBL_MAX, ymax = -DBL_MAX, zmax = -DBL_MAX;
  unsigned int count = 0;
  for(unsigned int i = 0; i < links.size(); i++) {
    
    const planning_models::KinematicState::LinkState* ls = state.getLinkState(links[i]);
    if(ls == NULL) {
      ROS_WARN_STREAM("No link state for requested link " << links[i]);
      continue;
    }

    if(ls->getLinkModel()->getShape() == NULL) {
      ROS_DEBUG_STREAM("No shape for " << links[i]);
      continue;
    }
    count++;
    double xval = ls->getGlobalCollisionBodyTransform().translation().x();
    double yval = ls->getGlobalCollisionBodyTransform().translation().y();
    double zval = ls->getGlobalCollisionBodyTransform().translation().z();

    //ROS_INFO_STREAM("Link " << ls->getName() << xval << " " << yval << " " << zval);
    
    x += xval;
    y += yval;
    z += zval;
    if(xval < xmin) {
      xmin = xval; 
    }
    if(xval > xmax) {
      xmax = xval; 
    }

    if(yval < ymin) {
      ymin = yval; 
    }
    if(yval > ymax) {
      ymax = yval; 
    }

    if(zval < zmin) {
      zmin = zval; 
    }
    if(zval > zmax) {
      zmax = zval; 
    }
    //bodies::Body* body = bodies::createBodyFromShape(&(*(ls->getLinkModel()->getShape())));
    //body->setPose(ls->getGlobalCollisionBodyTransform());
    //bodies.push_back(body);
  }

  if(count == 0) {
    ROS_WARN_STREAM("No end effector links have geometry");

    relative_transform.setIdentity();
    int_marker.scale = .25;

    mesh.mesh_resource = parent_link_state->getLinkModel()->getVisualFilename();
    //getting pose relative to first pose

    planning_models::msgFromPose(parent_pose, int_marker.pose);
    planning_models::msgFromPose(relative_transform, mesh.pose);
    control.markers.push_back(mesh);

  } else {

    Eigen::Translation3d trans(x/(count*1.0), y/(count*1.0), z/(count*1.0));
    //ROS_INFO_STREAM("Min " << xmin << " " << ymin << " " << zmin);
    //ROS_INFO_STREAM("Max " << xmax << " " << ymax << " " << zmax);
    //bodies::BoundingSphere merged_sphere;
    //bodies::computeBoundingSphere(bodies, merged_sphere);
    double dia = fmax((xmax-xmin), fmax(ymax-ymin, zmax-zmin));
    //TODO - better way to add padding?
    int_marker.scale = fmax(.3, dia+.05);
    //int_marker.scale = merged_sphere.rad*2.0;
    
    Eigen::Affine3d bound_pose = (trans*init_transform)*Eigen::Quaterniond(parent_pose.rotation());
    planning_models::msgFromPose(bound_pose, int_marker.pose);
    
    relative_transform = bound_pose.inverse()*parent_pose;
    geometry_msgs::Pose tp;
    planning_models::msgFromPose(relative_transform, tp);
    for(unsigned int i = 0; i < links.size(); i++) {
      
      const planning_models::KinematicState::LinkState* ls = state.getLinkState(links[i]);
      if(ls == NULL) {
        ROS_WARN_STREAM("No link state for requested link " << links[i]);
        continue;
      }
      
      if(ls->getLinkModel()->getVisualFilename().empty()) {
        ROS_DEBUG_STREAM("No filename for " << links[i]);
        
        continue;
      }
      
      mesh.mesh_resource = ls->getLinkModel()->getVisualFilename();
      //getting pose relative to first pose
      Eigen::Affine3d ret_pose = bound_pose.inverse()*ls->getGlobalCollisionBodyTransform();
      
      planning_models::msgFromPose(ret_pose, mesh.pose);
      control.markers.push_back(mesh);
    }
  }

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  int_marker.controls.push_back( control );

  return int_marker;

}

void recolorInteractiveMarker(visualization_msgs::InteractiveMarker& marker,
                                     const std_msgs::ColorRGBA& color) {
  for(unsigned int i = 0; i < marker.controls.size(); i++) {
    for(unsigned int j = 0; j < marker.controls[i].markers.size(); j++) {
      marker.controls[i].markers[j].color = color;
    }
  }
}

}

