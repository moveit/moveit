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

/* Author: Mrinal Kalakrishnan, Ken Anderson */

#ifndef MOVEIT_DISTANCE_FIELD_DISTANCE_FIELD_
#define MOVEIT_DISTANCE_FIELD_DISTANCE_FIELD_

#include <moveit/distance_field/voxel_grid.h>
#include <vector>
#include <list>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit_msgs/CollisionMap.h>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <geometric_shapes/shape_messages.h>

namespace distance_field
{

/// \brief The plane to visualize
enum PlaneVisualizationType
{
 XYPlane,
 XZPlane,
 YZPlane
};

/**
* \brief DistanceField is an abstract base class for computing
* distances from sets of 3D obstacle points.  The distance assigned to
* a freespace cell should be the distance to the closest obstacle
* cell.  
*
* Inherited classes must contain methods for holding a dense set of 3D
* voxels as well as methods for computing the required distances.
*
*/
class DistanceField
{
public:

  /** 
   * \brief Constructor
   * 
   * @param [in] size_x The X dimension of the volume to represent
   * @param [in] size_y The Y dimension of the volume to represent
   * @param [in] size_z The Z dimension of the volume to represent
   * @param [in] resolution The resolution of the volume
   * @param [in] origin_x The lower X corner of the volume
   * @param [in] origin_y The lower Y corner of the volume
   * @param [in] origin_z The lower Z corner of the volume
   */
  DistanceField(double size_x, double size_y, double size_z, double resolution,
                double origin_x, double origin_y, double origin_z);

  virtual ~DistanceField();

  virtual void updatePointsInField(const EigenSTL::vector_Vector3d& old_points,
                                   const EigenSTL::vector_Vector3d& new_points) = 0;

  /** 
   * \brief Add a set of points to the distance field, updating distance values accordingly
   * 
   * This function will incrementally add the given points and update
   * the distance field correspondingly. Use the \ref reset() function
   * if you need to remove all points and start afresh.
   *
   * @param [in] points The set of points to add
   */
  virtual void addPointsToField(const EigenSTL::vector_Vector3d &points)=0;

  /** 
   * \brief Removes a set of points to the distance field, updating distance values accordingly
   * 
   * This function will remove the given points and update the
   * distance field correspondingly. Use the \ref reset() function if
   * you need to remove all points and start afresh.
   *
   * @param [in] points The set of points to remove
   */
  virtual void removePointsFromField(const EigenSTL::vector_Vector3d &points)=0;

  void addShapeToField(const shapes::ShapeMsg& shape,
                       const geometry_msgs::Pose& pose);

  void moveShapeInField(const shapes::ShapeMsg& shape,
                        const geometry_msgs::Pose& old_pose,
                        const geometry_msgs::Pose& new_pose);
  
  void removeShapeFromField(const shapes::ShapeMsg& shape,
                            const geometry_msgs::Pose& pose);

  /**
   * \brief Resets the distance field to an unitialized value
   */
  virtual void reset()=0;
  
  /** 
   * \brief Gets the distance to the closest obstacle at the given location.
   * 
   * The particular implementation may return a max distance value if
   * a cell is far away from all obstacles.  Values of 0.0 should
   * represent that a cell is an obstacle, and some implementations
   * may return a signed value representing distance to the surface
   * when a cell is deep inside an obstacle volume.  An implementation
   * may also return some value to represent when a location is
   * outside the represented volume.
   *
   * @param [in] x The cell's X value
   * @param [in] y The cell's Y value
   * @param [in] z The cell's Z value
   * 
   * @return The distance to the closest obstacle
   */
  virtual double getDistance(double x, double y, double z) const = 0;
  
  /**
   * \brief Gets the distance at a location and the gradient of the field.
   */
  double getDistanceGradient(double x, double y, double z, 
                             double& gradient_x, double& gradient_y, double& gradient_z, 
                             bool& in_bounds) const;
  /**
   * \brief Gets the distance to the closest obstacle at the given integer cell location.
   */
  virtual double getDistanceFromCell(int x, int y, int z) const = 0;

  //pass-throughs to underlying container
  virtual bool isCellValid(int x, int y, int z) const = 0;

  virtual int getXNumCells() const = 0;

  virtual int getYNumCells() const = 0;

  virtual int getZNumCells() const = 0;

  virtual bool gridToWorld(int x, int y, int z, 
                           double& world_x, double& world_y, double& world_z) const = 0;
  virtual bool worldToGrid(double world_x, double world_y, double world_z, 
                           int& x, int& y, int& z) const = 0;

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
#endif
