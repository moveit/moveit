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

#ifndef _MARKER_GENERATOR_H_
#define _MARKER_GENERATOR_H_

#include <vector>

#include <boost/ptr_container/ptr_vector.hpp>

#include <visualization_msgs/Marker.h>

#include <geometric_shapes/shape_messages.h>
#include <geometry_msgs/Pose.h>

namespace moveit_tabletop_object_detector {

class ModelFitInfo;

//! A convenience class for generating markers based on various clustering and fitting data
/*! Just a place to group all the different debug marker generators
  so they don't polute other classes.
*/
class MarkerGenerator {
 public:
  //! Create a line strip marker that goes around a detected table
  static visualization_msgs::Marker getTableMarker(float xmin, float xmax, float ymin, float ymax);
  //! A marker with all the points in a cloud in a random color
  template <class PointCloudType>
  static visualization_msgs::Marker getCloudMarker(const PointCloudType& cloud);
  //! A marker showing where a fit model is believed to be
  static visualization_msgs::Marker getFitMarker(const shape_msgs::Mesh &mesh, double rank);  
  //! A marker showing where a convex hull table is
  static visualization_msgs::Marker getConvexHullTableMarker(const shape_msgs::Mesh &mesh);
  //! Create a generic Marker
  static visualization_msgs::Marker createMarker(std::string frame_id, double duration, double xdim, double ydim, double zdim,
					  double r, double g, double b, int type, int id, std::string ns, geometry_msgs::Pose pose);
};

/*!
  It is the responsibility of the caller to set the appropriate pose for the marker so that
  it shows up in the right reference frame.
*/
template <class PointCloudType>
visualization_msgs::Marker MarkerGenerator::getCloudMarker(const PointCloudType& cloud)
{
  static bool first_time = true;
  if (first_time) {
    srand ( time(NULL) );
    first_time = false;
  }

  //create the marker
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();

  marker.type = visualization_msgs::Marker::POINTS;
  marker.scale.x = 0.002;
  marker.scale.y = 0.002;
  marker.scale.z = 1.0;

  marker.color.r = ((double)rand())/RAND_MAX;
  marker.color.g = ((double)rand())/RAND_MAX;
  marker.color.b = ((double)rand())/RAND_MAX;
  marker.color.a = 1.0;

  for(size_t i=0; i<cloud.points.size(); i++) {
    geometry_msgs::Point p;
    p.x = cloud.points[i].x;
    p.y = cloud.points[i].y;
    p.z = cloud.points[i].z;
    marker.points.push_back(p);
  }

  //the caller must decide the header; we are done here
  return marker;
}


}//namespace

#endif
