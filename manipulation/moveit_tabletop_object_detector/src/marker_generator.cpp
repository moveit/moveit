/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author(s): Marius Muja and Matei Ciocarlie

#include <moveit_tabletop_object_detector/marker_generator.h>

//for random colors
#include <stdlib.h>
#include <time.h>

#include <geometry_msgs/Pose.h>

#include <tf/transform_datatypes.h>

#include <moveit_tabletop_object_detector/model_fitter.h>

namespace moveit_tabletop_object_detector {

visualization_msgs::Marker MarkerGenerator::getFitMarker(const shape_msgs::Mesh &mesh, double rank)
{
  //create the marker
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.type = visualization_msgs::Marker::POINTS;
  //marker will be published in object frame, so its pose is identity
  marker.color.a = 1.0;

  marker.color.r = rank;
  marker.color.g = 1 - rank;
  marker.color.b = 0.0;

  marker.scale.x = 0.003;
  marker.scale.y = 0.003;
  marker.points.insert(marker.points.begin(), mesh.vertices.begin(), mesh.vertices.end());

  for (int i=0; i<40; i++)
  {
    geometry_msgs::Point p;
    p.x = 0.0005 * i;
    p.y = p.z = 0.0;
    marker.points.push_back(p);
  }

  return marker;
}

/*! The point cloud is a set of points belonging to the plane, in the plane coordinate system
  (with the origin in the plane and the z axis normal to the plane).

  It is the responsibility of the caller to set the appropriate pose for the marker so that
  it shows up in the right reference frame.
 */
visualization_msgs::Marker MarkerGenerator::getTableMarker(float xmin, float xmax, float ymin, float ymax)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.lifetime = ros::Duration();

  //create the marker in the table reference frame
  //the caller is responsible for setting the pose of the marker to match

  marker.scale.x = 0.001;
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;

  marker.points.resize(5);
  marker.points[0].x = xmin;
  marker.points[0].y = ymin;
  marker.points[0].z = 0;
  
  marker.points[1].x = xmin;
  marker.points[1].y = ymax;
  marker.points[1].z = 0;
  
  marker.points[2].x = xmax;
  marker.points[2].y = ymax;
  marker.points[2].z = 0;
  
  marker.points[3].x = xmax;
  marker.points[3].y = ymin;
  marker.points[3].z = 0;
  
  marker.points[4].x = xmin;
  marker.points[4].y = ymin;
  marker.points[4].z = 0;

  marker.points.resize(6);
  marker.points[5].x = xmin;
  marker.points[5].y = ymin;
  marker.points[5].z = 0.02;
   
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  return marker;
}

/*! Create a marker for a convex hull table
  It is the responsibility of the caller to set the appropriate pose for the marker so that
  it shows up in the right reference frame.
 */
visualization_msgs::Marker MarkerGenerator::getConvexHullTableMarker(const shape_msgs::Mesh &mesh)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.001;
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;
  for(size_t i=0; i<mesh.triangles.size(); i++)
  {
    marker.points.push_back(mesh.vertices[mesh.triangles[i].vertex_indices[0]]);
    marker.points.push_back(mesh.vertices[mesh.triangles[i].vertex_indices[1]]);
    marker.points.push_back(mesh.vertices[mesh.triangles[i].vertex_indices[2]]);
  }
  return marker;
}


/*! Create a generic Marker 
 */
visualization_msgs::Marker MarkerGenerator::createMarker(std::string frame_id, double duration, double xdim, double ydim, double zdim,
							 double r, double g, double b, int type, int id, std::string ns, geometry_msgs::Pose pose)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(duration);
  marker.type = type;
  marker.id = id;
  marker.ns = ns;
  marker.pose = pose;
  marker.header.frame_id = frame_id;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.scale.x = xdim;
  marker.scale.y = ydim;
  marker.scale.z = zdim;
  return marker;
}


} //namespace tabletop_object_detector
