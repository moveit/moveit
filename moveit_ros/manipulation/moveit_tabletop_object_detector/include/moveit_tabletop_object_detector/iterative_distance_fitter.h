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

// Author(s): Marius Muja, Matei Ciocarlie and Romain Thibaux

#include <moveit_tabletop_object_detector/model_fitter.h>

#include <math.h>
#include <distance_field/propagation_distance_field.h>

#include <moveit_tabletop_object_detector/marker_generator.h>

namespace moveit_tabletop_object_detector {

//! Does an ICP-like fitting only in the X and Y translation DOFs
class IterativeTranslationFitter : public DistanceFieldFitter
{
 private:

  //! Helper function for fitting
  template <class PointCloudType>
    geometry_msgs::Point32 centerOfSupport(const PointCloudType& cloud);

  //! Inner loop when doing translation fitting
  template <class PointCloudType>
    double getFitScoreAndGradient(const PointCloudType& cloud, 
				  const geometry_msgs::Point32& location, 
				  geometry_msgs::Point32& vector,
				  double &maxDist);
 public:
  //! Stub, just calls super's constructor
  IterativeTranslationFitter() : DistanceFieldFitter() {}
  //! Empty stub
  ~IterativeTranslationFitter() {}

  //! Main fitting function
  template <class PointCloudType>
    ModelFitInfo fitPointCloud(const PointCloudType& cloud);
};

//------------------------- Implementation follows ----------------------------------------

/*! Computes the point at the bottom of the point cloud vertical to the center of
  gravity. This is the point where the table supports the object. 
*/
template <class PointCloudType>
geometry_msgs::Point32 IterativeTranslationFitter::centerOfSupport(const PointCloudType& cloud) 
{
  geometry_msgs::Point32 center;
  center.x = center.y = center.z = 0;
  if (cloud.points.empty())
  {
    return center;
  }
  for (unsigned int i=0; i<cloud.points.size(); ++i) 
  {
    center.x += cloud.points[i].x;
    center.y += cloud.points[i].y;
  }
  center.x /= cloud.points.size();
  center.y /= cloud.points.size();
  return center;
}


template <class PointCloudType>
double IterativeTranslationFitter::getFitScoreAndGradient(const PointCloudType& cloud, 
							  const geometry_msgs::Point32& location, 
							  geometry_msgs::Point32& vector,
							  double &max_dist)
{
  double score = 0;
  max_dist = 0;
  
  vector.x = 0;
  vector.y = 0;
  vector.z = 0;
  int cnt = 0;
  
  for (size_t i=0;i<cloud.points.size();i++) 
  {
    double wx = cloud.points[i].x-location.x;
    double wy = cloud.points[i].y-location.y;
    double wz = cloud.points[i].z-location.z;
    
    int x, y, z;
    double val = truncate_value_;
    if (distance_voxel_grid_->worldToGrid(wx,wy,wz,x,y,z)) 
    {
      const distance_field::PropDistanceFieldVoxel& voxel = distance_voxel_grid_->getCell(x,y,z);
      double cx, cy, cz;
      if (voxel.closest_point_[0] != distance_field::PropDistanceFieldVoxel::UNINITIALIZED) 
      {
	distance_voxel_grid_->gridToWorld(voxel.closest_point_[0],
					  voxel.closest_point_[1],
					  voxel.closest_point_[2],
					  cx,cy,cz);
	val = distance_voxel_grid_->getDistanceFromCell(x,y,z);
	vector.x += (cx-wx);
	vector.y += (cy-wy);
	vector.z += (cz-wz);
	cnt++;
	if (val>truncate_value_) 
        {
	  val = truncate_value_;
	}
      }
      else
      {
      }
    }
    else
    {
    }    
    max_dist = std::max(max_dist,val);
    //score += val*val;
    score += val;
  }
  score /= (cloud.points.size());
  if (cnt!=0) 
  {
    vector.x /=  cnt;
    vector.y /=  cnt;
    vector.z /=  cnt;
  }
  
  return score;
}

/*! Iterates over the inner loop of \a getFitScoreAndGradient, then moves in the direction
  of the computed gradient. Does this until the score stops decreasing.

  The fit is initialized to the centroid of the cloud along x and y, and 0 along z. This 
  assumes that the meshes are all such that the origin is at the bottom, with the z axis 
  pointing up. It also assumes the points have been translated to a coordinate frame
  where z=0 is the table plane.

  For the same reason. there is no iteration done along z at all.
*/
template <class PointCloudType>
ModelFitInfo IterativeTranslationFitter::fitPointCloud(const PointCloudType& cloud)
{
  if (cloud.points.empty()) 
  {
    ROS_ERROR("Attempt to fit model to empty point cloud");
    geometry_msgs::Pose bogus_pose;
    return ModelFitInfo(model_id_, bogus_pose, -1.0);
  }
  
  // compute center of point cloud
  geometry_msgs::Point32 center = centerOfSupport<PointCloudType>(cloud);

  geometry_msgs::Point32 location = center;
  geometry_msgs::Point32 vector;
  double max_dist;
  geometry_msgs::Pose pose;
    
  double score = getFitScoreAndGradient<PointCloudType>(cloud, location, vector, max_dist);
  double old_score = score + 1;
 
  double EPS = 1.0e-6;
  int max_iterations = 100;
  int iter = 0;
  while (score < old_score - EPS && iter < max_iterations) 
  {
    old_score = score;
    location.x -= vector.x;
    location.y -= vector.y;
    // see above comment on search along z
    // location.z -= vector.z;
    score = getFitScoreAndGradient<PointCloudType>(cloud, location, vector, max_dist);
    iter++;
  }

  if (iter == max_iterations) 
  {
    ROS_WARN("Maximum iterations reached in model fitter");
  }

  pose.position.x = location.x;
  pose.position.y = location.y;
  pose.position.z = location.z;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  return ModelFitInfo(model_id_, pose, old_score);
}

} //namespace
