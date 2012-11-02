/*********************************************************************
 * Software License Agreement (BSD License)
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

#include <moveit_tabletop_object_detector/model_fitter.h>

#include <distance_field/propagation_distance_field.h>
#include <moveit_tabletop_object_detector/iterative_distance_fitter.h>

namespace moveit_tabletop_object_detector {

DistanceFieldFitter::DistanceFieldFitter() : distance_voxel_grid_(NULL) 
{
  truncate_value_ = 0.05;
  distance_field_resolution_ = 0.002;
}

DistanceFieldFitter::~DistanceFieldFitter() 
{
  delete distance_voxel_grid_;
}

void DistanceFieldFitter::initializeFromEigenVectors(const EigenSTL::vector_Vector3d &points) 
{
  delete distance_voxel_grid_;
  distance_voxel_grid_ = NULL;

  if (points.size() == 0) {
    return;
  }

  Eigen::Vector3d min = points[0], max = points[0];
  for (size_t i=0; i<points.size(); ++i) 
  {
    for (size_t j=0; j<3; ++j) 
    {
      if (min[j] > points[i][j]) 
      {
	min[j] = points[i][j];
      }
      if (max[j] < points[i][j]) 
      {
	max[j] = points[i][j];
      }
    }
  }
  
  ROS_DEBUG("Size: (%g,%g,%g, %g, %g, %g)",min[0], min[1], min[2], max[0], max[1], max[2]);

  //the distance field is initialized as follows: match the size of the object, but add
  //padding equal to the truncate_value_ on each side. Resolution is constant regardless 
  //of the size of the object.

  //the only difference in along negative Z where we add only a very small padding, making
  //the assumptions that we are pretty sure where the table is (Z=0 by convention), all objects
  //have the origin on the bottom, and nothing is below the table
  //allow just two cells under the table, to deal with noise and such
  double table_padding = 2.5 * distance_field_resolution_;
  distance_voxel_grid_ = new distance_field::PropagationDistanceField(max[0]-min[0] + 2*truncate_value_, 
								      max[1]-min[1] + 2*truncate_value_,
								      max[2]-min[2] + truncate_value_ + table_padding, 
								      distance_field_resolution_, 
								      min[0] - truncate_value_, 
								      min[1] - truncate_value_,  
								      min[2] - table_padding,
								      2 * truncate_value_ );
  distance_voxel_grid_->reset();
  distance_voxel_grid_->addPointsToField(points);
}

double dist(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1)
{
  return sqrt( (v1.x()-v0.x())*(v1.x()-v0.x()) + 
	       (v1.y()-v0.y())*(v1.y()-v0.y()) +  
	       (v1.z()-v0.z())*(v1.z()-v0.z()) );
}

/*! Given a triangle defined by three vertices, returns a set of points obtained
  by sampling the surface of the triangle. Points are obtained by barycentric
  interpolation, with a guarantee that, along the interpolated directions, the
  distance between interpolated locations is not greater than min_res  (could be 
  smaller if one of the 0-1 and 0-2 edges of the triangle is significantly shorter 
  than the other).

  The vertices themselves are NOT returned in the set of points.
*/
EigenSTL::vector_Vector3d interpolateTriangle(Eigen::Vector3d v0, 
                                              Eigen::Vector3d v1, 
                                              Eigen::Vector3d v2, double min_res)
{
  EigenSTL::vector_Vector3d vectors;

  //find out the interpolation resolution for the first coordinate
  //based on the size of the 0-1 and 0-2 edges
  double d01 = dist(v0, v1);
  double d02 = dist(v0, v2);
  double res_0 = min_res / std::max(d01, d02);

  //perform the first interpolation
  //we do not want the vertices themselves, so we don't start at 0 
  double t0=res_0;
  bool done = false;
  while (!done)
  {
    if (t0 >= 1.0)
    {
      t0 = 1.0;
      done = true;
    }
    //compute the resolution for the second interpolation
    Eigen::Vector3d p1 = t0*v0 + (1-t0) * v1;
    Eigen::Vector3d p2 = t0*v0 + (1-t0) * v2;
    double d12 = dist(p1, p2);
    double res_12 = min_res / d12;

    //perform the second interpolation
    double t12 = 0;
    bool done12 = false;
    while (!done12)
    {
      if (t12 >= 1.0)
      {
	t12 = 1.0;
	done12 = true;
      }
      //actual point insertion
      //do not insert the vertices themselves
      if (t0!=1.0 || (t12!=0.0 && t12!=1.0))
      {
	vectors.push_back( t12*p1 + (1.0 - t12)*p2 );
      }
      t12 += res_12;
    }
    
    t0 += res_0;
  }
  return vectors;
}

void ModelToCloudFitter::sampleMesh(const shape_msgs::Mesh &mesh, 
                                    EigenSTL::vector_Vector3d& points,
				    double resolution)
{
  points.reserve(mesh.vertices.size());
  //vertices themselves get inserted explicitly here. If we inserted them
  //as part of triangles, we would insert each vertex multiple times
  typedef std::vector<geometry_msgs::Point>::const_iterator I;
  for (I i=mesh.vertices.begin(); i!=mesh.vertices.end(); i++) 
  {
    points.push_back(Eigen::Vector3d(i->x,i->y,i->z));
  }
  
  //sample triangle surfaces at a specified min-resolution 
  //and insert the resulting points
  for (size_t i=0; i<mesh.triangles.size(); i++)
  {
    Eigen::Vector3d v0( mesh.vertices.at( mesh.triangles.at(i).vertex_indices[0]).x,
                        mesh.vertices.at( mesh.triangles.at(i).vertex_indices[0]).y,
                        mesh.vertices.at( mesh.triangles.at(i).vertex_indices[0]).z);
    Eigen::Vector3d v1( mesh.vertices.at( mesh.triangles.at(i).vertex_indices[1]).x,
		  mesh.vertices.at( mesh.triangles.at(i).vertex_indices[1]).y,
		  mesh.vertices.at( mesh.triangles.at(i).vertex_indices[1]).z);
    Eigen::Vector3d v2( mesh.vertices.at( mesh.triangles.at(i).vertex_indices[2]).x,
		  mesh.vertices.at( mesh.triangles.at(i).vertex_indices[2]).y,
		  mesh.vertices.at( mesh.triangles.at(i).vertex_indices[2]).z);
    EigenSTL::vector_Vector3d triangle_vectors = interpolateTriangle(v0, v1, v2, resolution);
    points.insert(points.begin(), triangle_vectors.begin(), triangle_vectors.end());
  }
}


void DistanceFieldFitter::initializeFromMesh(const shape_msgs::Mesh &mesh)
{
  EigenSTL::vector_Vector3d points;
  //we use a slightly larger resolution than the distance field, in an attempt to bring
  //down pre-computation time
  sampleMesh(mesh, points,  1.5 * distance_field_resolution_ ); 
  initializeFromEigenVectors(points);
}


} //namespace
