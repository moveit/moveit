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

/** \author Mrinal Kalakrishnan */

#ifndef DF_DISTANCE_FIELD_H_
#define DF_DISTANCE_FIELD_H_

#include <distance_field/voxel_grid.h>
#include <vector>
#include <list>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

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
template <typename T>
class DistanceField: public VoxelGrid<T>
{
public:

  /**
   * \brief Constructor for the VoxelGrid.
   *
   * @param size_x Size (x axis) of the container in meters
   * @param size_y Size (y axis) of the container in meters
   * @param size_z Size (z axis) of the container in meters
   * @param resolution: resolution (size of a single cell) in meters
   * @param origin_x Origin (x axis) of the container
   * @param origin_y Origin (y axis) of the container
   * @param origin_z Origin (z axis) of the container
   * @param default_object The object to return for an out-of-bounds query
   */
  DistanceField(double size_x, double size_y, double size_z, double resolution,
      double origin_x, double origin_y, double origin_z, T default_object);

  virtual ~DistanceField();


  /**
   * \brief Add (and expand) a set of points to the distance field.
   *
   * This function will incrementally add the given points and update the distance field
   * correspondingly. Use the reset() function if you need to remove all points and start
   * afresh.
   */
  virtual void addPointsToField(const std::vector<Eigen::Vector3d> &points)=0;

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
  double getDistance(double x, double y, double z) const;

  /**
   * \brief Gets the distance at a location and the gradient of the field.
   */
  double getDistanceGradient(double x, double y, double z, double& gradient_x, double& gradient_y, double& gradient_z) const;

  /**
   * \brief Gets the distance to the closest obstacle at the given integer cell location.
   */
  double getDistanceFromCell(int x, int y, int z) const;

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
                            visualization_msgs::Marker& marker );

  /**
   * \brief Get an array of markers that can be published to rviz
   *
   * Gets the gradient of the distance field as visualization markers for rviz.
   * \param markers the marker array to be published
   */
  void getGradientMarkers(double min_radius, double max_radius,
                          const std::string & frame_id, const ros::Time stamp,
                          std::vector<visualization_msgs::Marker>& markers );

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
                       visualization_msgs::Marker& marker );

protected:
  virtual double getDistance(const T& object) const=0;

private:
  int inv_twice_resolution_;
};

//////////////////////////// template function definitions follow //////////////

template <typename T>
DistanceField<T>::~DistanceField()
{

}

template <typename T>
DistanceField<T>::DistanceField(double size_x, double size_y, double size_z, double resolution,
    double origin_x, double origin_y, double origin_z, T default_object):
      VoxelGrid<T>(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, default_object)
{
  inv_twice_resolution_ = 1.0/(2.0*resolution);
}

template <typename T>
double DistanceField<T>::getDistance(double x, double y, double z) const
{
  return getDistance((*this)(x,y,z));
}

template <typename T>
double DistanceField<T>::getDistanceGradient(double x, double y, double z, double& gradient_x, double& gradient_y, double& gradient_z) const
{
  int gx, gy, gz;

  this->worldToGrid(x, y, z, gx, gy, gz);

  // if out of bounds, return 0 distance, and 0 gradient
  // we need extra padding of 1 to get gradients
  if (gx<1 || gy<1 || gz<1 || gx>=this->num_cells_[this->DIM_X]-1 || gy>=this->num_cells_[this->DIM_Y]-1 || gz>=this->num_cells_[this->DIM_Z]-1)
  {
    gradient_x = 0.0;
    gradient_y = 0.0;
    gradient_z = 0.0;
    return 0;
  }

  gradient_x = (getDistanceFromCell(gx+1,gy,gz) - getDistanceFromCell(gx-1,gy,gz))*inv_twice_resolution_;
  gradient_y = (getDistanceFromCell(gx,gy+1,gz) - getDistanceFromCell(gx,gy-1,gz))*inv_twice_resolution_;
  gradient_z = (getDistanceFromCell(gx,gy,gz+1) - getDistanceFromCell(gx,gy,gz-1))*inv_twice_resolution_;

  return getDistanceFromCell(gx,gy,gz);

}

template <typename T>
double DistanceField<T>::getDistanceFromCell(int x, int y, int z) const
{
  return getDistance(this->getCell(x,y,z));
}

template <typename T>
void DistanceField<T>::getIsoSurfaceMarkers(double min_radius, double max_radius,
                                            const std::string & frame_id, const ros::Time stamp,
                                            const Eigen::Affine3d& cur,
                                            visualization_msgs::Marker& inf_marker )
{
  inf_marker.points.clear();
  inf_marker.header.frame_id = frame_id;
  inf_marker.header.stamp = stamp;
  inf_marker.ns = "distance_field";
  inf_marker.id = 1;
  inf_marker.type = visualization_msgs::Marker::CUBE_LIST;
  inf_marker.action = 0;
  inf_marker.scale.x = this->resolution_[VoxelGrid<T>::DIM_X];
  inf_marker.scale.y = this->resolution_[VoxelGrid<T>::DIM_Y];
  inf_marker.scale.z = this->resolution_[VoxelGrid<T>::DIM_Z];
  inf_marker.color.r = 1.0;
  inf_marker.color.g = 0.0;
  inf_marker.color.b = 0.0;
  inf_marker.color.a = 0.1;
  //inf_marker.lifetime = ros::Duration(30.0);

  inf_marker.points.reserve(100000);
  int num_total_cells =
    this->num_cells_[VoxelGrid<T>::DIM_X]*
    this->num_cells_[VoxelGrid<T>::DIM_Y]*
    this->num_cells_[VoxelGrid<T>::DIM_Z];
  for (int x = 0; x < this->num_cells_[VoxelGrid<T>::DIM_X]; ++x)
  {
    for (int y = 0; y < this->num_cells_[VoxelGrid<T>::DIM_Y]; ++y)
    {
      for (int z = 0; z < this->num_cells_[VoxelGrid<T>::DIM_Z]; ++z)
      {
        double dist = getDistanceFromCell(x,y,z);
        if (dist >= min_radius && dist <= max_radius)
        {
          int last = inf_marker.points.size();
          inf_marker.points.resize(last + 1);
          double nx, ny, nz;
          this->gridToWorld(x,y,z,
                            nx, ny, nz);
          Eigen::Translation3d vec(nx,ny,nz);
          //Eigen::Translation3d nv = cur.rotation()*vec;
          inf_marker.points[last].x = vec.x();
          inf_marker.points[last].y = vec.y();
          inf_marker.points[last].z = vec.z();
        }
      }
    }
  }
}

template <typename T>
void DistanceField<T>::getGradientMarkers( double min_radius, double max_radius,
                                           const std::string & frame_id, const ros::Time stamp,
                                           std::vector<visualization_msgs::Marker>& markers )
{
  Eigen::Vector3d unitX(1, 0, 0);
  Eigen::Vector3d unitY(0, 1, 0);
  Eigen::Vector3d unitZ(0, 0, 1);

  int id = 0;

  for (int x = 0; x < this->num_cells_[this->DIM_X]; ++x)
  {
    for (int y = 0; y < this->num_cells_[this->DIM_Y]; ++y)
    {
      for (int z = 0; z < this->num_cells_[this->DIM_Z]; ++z)
      {
        double worldX, worldY, worldZ;
        this->gridToWorld(x, y, z, worldX, worldY, worldZ);

        double gradientX, gradientY, gradientZ;
        double distance = getDistanceGradient(worldX, worldY, worldZ, gradientX, gradientY, gradientZ);
        Eigen::Vector3d gradient(gradientX, gradientY, gradientZ);

        if (distance >= min_radius && distance <= max_radius && gradient.norm() > 0)
        {
          visualization_msgs::Marker marker;

          marker.header.frame_id = frame_id;
          marker.header.stamp = stamp;

          marker.ns = "distance_field_gradient";
          marker.id = id++;
          marker.type = visualization_msgs::Marker::ARROW;
          marker.action = visualization_msgs::Marker::ADD;

          marker.pose.position.x = worldX;
          marker.pose.position.y = worldY;
          marker.pose.position.z = worldZ;

          Eigen::Vector3d axis = gradient.cross(unitX).norm() > 0 ? gradient.cross(unitX) : unitY;
          //double angle = -gradient.angle(unitX);
          //Eigen::AngleAxisd rotation(angle, axis);

          // marker.pose.orientation.x = rotation.rotation().x();
          // marker.pose.orientation.y = rotation.rotation().y();
          // marker.pose.orientation.z = rotation.rotation().z();
          // marker.pose.orientation.w = rotation.rotation().w();

          marker.scale.x = this->resolution_[this->DIM_X];
          marker.scale.y = this->resolution_[this->DIM_Y];
          marker.scale.z = this->resolution_[this->DIM_Z];

          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          marker.color.a = 1.0;

          markers.push_back(marker);
        }
      }
    }
  }
}

template <typename T>
void DistanceField<T>::addCollisionMapToField(const moveit_msgs::CollisionMap &collision_map)
{
  size_t num_boxes = collision_map.boxes.size();
  std::vector<Eigen::Vector3d> points;
  points.reserve(num_boxes);
  for (size_t i=0; i<num_boxes; ++i)
  {
    points.push_back(Eigen::Vector3d(
        collision_map.boxes[i].pose.position.x,
        collision_map.boxes[i].pose.position.y,
        collision_map.boxes[i].pose.position.z
        ));
  }
  addPointsToField(points);
}

template <typename T>
void DistanceField<T>::getPlaneMarkers(distance_field::PlaneVisualizationType type, double length, double width,
                                       double height, const Eigen::Vector3d& origin,
                                       const std::string & frame_id, const ros::Time stamp,
                                       visualization_msgs::Marker& plane_marker )
{
  plane_marker.header.frame_id = frame_id;
  plane_marker.header.stamp = stamp;
  plane_marker.ns = "distance_field_plane";
  plane_marker.id = 1;
  plane_marker.type = visualization_msgs::Marker::CUBE_LIST;
  plane_marker.action = visualization_msgs::Marker::ADD;
  plane_marker.scale.x = this->resolution_[VoxelGrid<T>::DIM_X];
  plane_marker.scale.y = this->resolution_[VoxelGrid<T>::DIM_Y];
  plane_marker.scale.z = this->resolution_[VoxelGrid<T>::DIM_Z];
  //plane_marker.lifetime = ros::Duration(30.0);

  plane_marker.points.reserve(100000);
  plane_marker.colors.reserve(100000);

  double minX = 0;
  double maxX = 0;
  double minY = 0;
  double maxY = 0;
  double minZ = 0;
  double maxZ = 0;

  switch(type)
  {
    case XYPlane:
      minZ = height;
      maxZ = height;

      minX = -length/2.0;
      maxX = length/2.0;
      minY = -width/2.0;
      maxY = width/2.0;
      break;
    case XZPlane:
      minY = height;
      maxY = height;

      minX = -length/2.0;
      maxX = length/2.0;
      minZ = -width/2.0;
      maxZ = width/2.0;
      break;
    case YZPlane:
      minX = height;
      maxX = height;

      minY = -length/2.0;
      maxY = length/2.0;
      minZ = -width/2.0;
      maxZ = width/2.0;
      break;
  }

  minX += origin.x();
  minY += origin.y();
  minZ += origin.z();

  maxX += origin.x();
  maxY += origin.y();
  maxZ += origin.z();

  int minXCell = 0;
  int maxXCell = 0;
  int minYCell = 0;
  int maxYCell = 0;
  int minZCell = 0;
  int maxZCell = 0;

  this->worldToGrid(minX,minY,minZ, minXCell, minYCell, minZCell);
  this->worldToGrid(maxX,maxY,maxZ, maxXCell, maxYCell, maxZCell);
  plane_marker.color.a = 1.0;
  for(int x = minXCell; x <= maxXCell; ++x)
  {
    for(int y = minYCell; y <= maxYCell; ++y)
    {
      for(int z = minZCell; z <= maxZCell; ++z)
      {
        if(!this->isCellValid(x,y,z))
        {
          continue;
        }
        double dist = getDistanceFromCell(x, y, z);
        int last = plane_marker.points.size();
        plane_marker.points.resize(last + 1);
        plane_marker.colors.resize(last + 1);
        double nx, ny, nz;
        this->gridToWorld(x, y, z, nx, ny, nz);
        Eigen::Vector3d vec(nx, ny, nz);
        plane_marker.points[last].x = vec.x();
        plane_marker.points[last].y = vec.y();
        plane_marker.points[last].z = vec.z();
        if(dist < 0.0)
        {
          plane_marker.colors[last].r = fmax(fmin(0.1/fabs(dist), 1.0), 0.0);
          plane_marker.colors[last].g = fmax(fmin(0.05/fabs(dist), 1.0), 0.0);
          plane_marker.colors[last].b = fmax(fmin(0.01/fabs(dist), 1.0), 0.0);

        }
        else
        {
          plane_marker.colors[last].b = fmax(fmin(0.1/(dist+0.001), 1.0),0.0);
          plane_marker.colors[last].g = fmax(fmin(0.05/(dist+0.001), 1.0),0.0);
          plane_marker.colors[last].r = fmax(fmin(0.01/(dist+0.001), 1.0),0.0);
        }
      }
    }
  }
}

}
#endif /* DF_DISTANCE_FIELD_H_ */
