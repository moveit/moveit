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

/** \author Mrinal Kalakrishnan, Ken Anderson */

#ifndef DF_DISTANCE_FIELD_H_
#define DF_DISTANCE_FIELD_H_

#include <distance_field/voxel_grid.h>
#include <vector>
#include <list>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometric_shapes/eigen_types.h>
#include <moveit_msgs/CollisionMap.h>

namespace distance_field
{

/// \brief Structure the holds the location of voxels withing the voxel map
typedef Eigen::Vector3i int3;

/// \brief The plane to visualize
enum PlaneVisualizationType
{
 XYPlane,
 XZPlane,
 YZPlane
};

/**
* \brief A VoxelGrid that can convert a set of obstacle points into a distance field.
*
* It computes the distance transform of the input points, and stores the distance to
* the closest obstacle in each voxel. Also available is the location of the closest point,
* and the gradient of the field at a point. Expansion of obstacles is performed upto a given
* radius.
*
* This is an abstract base class, current implementations include PropagationDistanceField
* and PFDistanceField.
*/
class DistanceField
{
public:

  /**
   * \brief Constructor for distance field
   *
   */
  DistanceField(double resolution);

  virtual ~DistanceField();


  /**
   * \brief Add (and expand) a set of points to the distance field.
   *
   * This function will incrementally add the given points and update the distance field
   * correspondingly. Use the reset() function if you need to remove all points and start
   * afresh.
   */
  virtual void addPointsToField(const EigenSTL::vector_Vector3d &points)=0;

  virtual void removePointsFromField(const EigenSTL::vector_Vector3d &points)=0;

  /**
   * \brief Adds the points in a collision map to the distance field.
   */
  void addCollisionMapToField(const moveit_msgs::CollisionMap &collision_map);

  /**
   * \brief Resets the distance field to the max_distance.
   */
  virtual void reset()=0;

  /**
   * \brief Gets the distance to the closest obstacle at the given location.
   */
  virtual double getDistance(double x, double y, double z) const = 0;

  /**
   * \brief Gets the distance at a location and the gradient of the field.
   */
  double getDistanceGradient(double x, double y, double z, double& gradient_x, double& gradient_y, double& gradient_z, bool& in_bounds) const;

  /**
   * \brief Gets the distance to the closest obstacle at the given integer cell location.
   */
  virtual double getDistanceFromCell(int x, int y, int z) const = 0;

  //pass-throughs to voxel grid
  virtual bool isCellValid(int x, int y, int z) const = 0;
  virtual int getXNumCells() const = 0;
  virtual int getYNumCells() const = 0;
  virtual int getZNumCells() const = 0;
  virtual bool gridToWorld(int x, int y, int z, double& world_x, double& world_y, double& world_z) const = 0;
  virtual bool worldToGrid(double world_x, double world_y, double world_z, int& x, int& y, int& z) const = 0;

  /**
   * \brief Get an iso-surface for visualizaion in rviz.
   *
   * Gets an iso-surface containing points between min_radius and max_radius
   * as visualization markers.
   * \param marker the marker to be published
   */
  void getIsoSurfaceMarkers(double min_radius, double max_radius,
                            const std::string & frame_id, const ros::Time stamp,
                            const Eigen::Affine3d& cur,
                            visualization_msgs::Marker& marker ) const;

  /**
   * \brief Get an array of markers that can be published to rviz
   *
   * Gets the gradient of the distance field as visualization markers for rviz.
   * \param markers the marker array to be published
   */
  void getGradientMarkers(double min_radius, double max_radius,
                          const std::string & frame_id, const ros::Time stamp,
                          std::vector<visualization_msgs::Marker>& markers ) const;

  /**
   * \brief Gets a set of markers to rviz along the specified plane.
   *
   * \param type the plane to show (XZ, XY, YZ)
   * \param length the size along the first axis in meters.
   * \param width the size along the second axis in meters.
   * \param height the position along the orthogonal axis to the plane, in meters.
   * \param marker the marker to be published
   */
  void getPlaneMarkers(PlaneVisualizationType type, double length, double width, double height, const Eigen::Vector3d& origin,
                       const std::string & frame_id, const ros::Time stamp,
                       visualization_msgs::Marker& marker ) const;

  // TODO - doc
  void getProjectionPlanes( const std::string & frame_id, const ros::Time stamp, const double max_distance,
                            visualization_msgs::Marker& marker ) const;

  double getResolution() const {
    return resolution_;
  }

protected:
  void setPoint(const int xCell, const int yCell, const int zCell,
                const double dist, geometry_msgs::Point & points, std_msgs::ColorRGBA & color,
                const double max_distance) const;

private:
  double resolution_;
  int inv_twice_resolution_;
};

}
#endif /* DF_DISTANCE_FIELD_H_ */
