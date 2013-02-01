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

#ifndef MOVEIT_DISTANCE_FIELD_PROPAGATION_DISTANCE_FIELD_
#define MOVEIT_DISTANCE_FIELD_PROPAGATION_DISTANCE_FIELD_

#include <moveit/distance_field/voxel_grid.h>
#include <moveit/distance_field/distance_field.h>
#include <vector>
#include <list>
#include <Eigen/Core>
#include <set>
#include <octomap/octomap.h>

namespace distance_field
{

/**
 * \brief Struct for sorting type Eigen::Vector3i for use in sorted
 * std containers.  Sorts in z order, then y order, then x order.  
 */
struct compareEigen_Vector3i
{
  bool operator()(Eigen::Vector3i loc_1, Eigen::Vector3i loc_2) const
  {
    if( loc_1.z() != loc_2.z() )
      return ( loc_1.z() < loc_2.z() );
    else if( loc_1.y() != loc_2.y() )
      return ( loc_1.y() < loc_2.y() );
    else if( loc_1.x() != loc_2.x() )
      return ( loc_1.x() < loc_2.x() );
    return false;
  }
};

/**
 * \brief Structure that holds voxel information for the
 * DistanceField.  Will be used in VoxelGrid.
 */
struct PropDistanceFieldVoxel
{

  /** 
   * \brief Constructor.  All fields left uninitialized.
   * 
   * 
   */
  PropDistanceFieldVoxel();

  /** 
   * \brief Constructor.  Sets values for distance_sq_ and
   * negative_distance_square_, and sets all remaining internal values
   * to uninitialized.
   * 
   * @param distance_sq_positive Value to which to initialize
   * distance_sq_ for distance to closest obstalce
   * 
   * @param distance_sq_negative Value to which to initialize
   * distance_sq_negative_ for distance to nearest non-obstacle cell
   * 
   */
  PropDistanceFieldVoxel(int distance_sq_positive, int distance_sq_negative);

  int distance_square_;         /**< \brief Distance in cells to the closest obstacle, squared */
  int negative_distance_square_; /**< \brief Distance in cells to the nearest unoccupied cell, squared */
  Eigen::Vector3i closest_point_; /**< \brief Closest occupied cell */
  Eigen::Vector3i closest_negative_point_; /**< \brief Closest unoccupied cell */
  int update_direction_;        /**< \brief Direction from which this voxel was updated for occupied distance propogation */
  int negative_update_direction_;        /**< \brief Direction from which this voxel was updated  for negative distance propogation*/

  static const int UNINITIALIZED=-1; /**< \brief Value that represents an unitialized voxel */
};

/**
 * \brief A DistanceField implementation that uses a vector
 * propagation method.  Distances propogate outward from occupied
 * cells, or inwards from unoccupied cells if negative distances are
 * to be computed, which is optional.  Outward and inward propagation
 * only occur to a desired maximum distance - cells that are more than
 * this maximum distance from the nearest cell will have maximum
 * distance measurements. 
 * 
 * Uses a \ref VoxelGrid to hold all data
 */
class PropagationDistanceField: public DistanceField
{
public:


  /** 
   * \brief Constructor that initializes entire distance field to
   * empty - all cells will be assigned maximum distance values.  All
   * units are arbitrary but are assumed for documentation purposes to
   * represent meters.
   * 
   * @param [in] size_x The X dimension in meters of the volume to represent
   * @param [in] size_y The Y dimension in meters of the volume to represent
   * @param [in] size_z The Z dimension in meters of the volume to represent
   * @param [in] resolution The resolution in meters of the volume
   * @param [in] origin_x The minimum X point of the volume
   * @param [in] origin_y The minimum Y point of the volume
   * @param [in] origin_z The minimum Z point of the volume

   * @param [in] max_distance The maximum distance to which to
   * propagate distance values.  Cells that are greater than this
   * distance will be assigned the maximum distance value.
   *
   * @param propagate_negative_distances Whether or not to propagate negative distances.  
   * 
   * @return 
   */
  PropagationDistanceField(double size_x, 
                           double size_y, 
                           double size_z, 
                           double resolution,
                           double origin_x, double origin_y, double origin_z, 
                           double max_distance,
                           bool propagate_negative_distances=false);

  PropagationDistanceField(const octomap::OcTree& octree,
                           const octomap::point3d& bbx_min,
                           const octomap::point3d& bbx_max,
                           double max_distance,
                           bool propagate_negative_distances=false);

  PropagationDistanceField(std::istream& stream,
                           double max_distance,
                           bool propagate_negative_distances=false);
  
  virtual ~PropagationDistanceField();
  
  /**
   * \brief Change the set of obstacle points and recalculate the distance field (if there are any changes).
   */
  virtual void updatePointsInField(const EigenSTL::vector_Vector3d& old_points,
                                   const EigenSTL::vector_Vector3d& new_points);

  /**
   * \brief Add (and expand) a set of points to the distance field.
   */
  virtual void addPointsToField(const EigenSTL::vector_Vector3d& points);

  /**
   * \brief Incrementally remove the set of points in the distance field.
   */
  virtual void removePointsFromField(const EigenSTL::vector_Vector3d& points);

  /**
   * \brief Resets the distance field to the max_distance.
   */
  virtual void reset();

  /**
   * \brief Gets the distance to the closest obstacle at the given location.
   */
  virtual double getDistance(double x, double y, double z) const;

  /**
   * \brief Gets the distance to the closest obstacle at the given integer cell location.
   */
  virtual double getDistanceFromCell(int x, int y, int z) const;

  //pass-throughs to voxel grid
  virtual bool isCellValid(int x, int y, int z) const;
  virtual int getXNumCells() const;
  virtual int getYNumCells() const;
  virtual int getZNumCells() const;
  virtual bool gridToWorld(int x, int y, int z, double& world_x, double& world_y, double& world_z) const;
  virtual bool worldToGrid(double world_x, double world_y, double world_z, int& x, int& y, int& z) const;

  virtual bool writeToStream(std::ostream& stream) const;

  virtual bool readFromStream(std::istream& stream);

  const PropDistanceFieldVoxel& getCell(int x, int y, int z) const {
    return voxel_grid_->getCell(x, y, z);
  }

private:

  void initialize();

  bool propagate_negative_;

  boost::shared_ptr<VoxelGrid<PropDistanceFieldVoxel> > voxel_grid_;

  /// \brief The set of all the obstacle voxels
  typedef std::set<Eigen::Vector3i, compareEigen_Vector3i> VoxelSet;

  /// \brief Structure used to hold propagation frontier
  std::vector<std::vector<Eigen::Vector3i> > bucket_queue_;
  std::vector<std::vector<Eigen::Vector3i> > negative_bucket_queue_;
  double max_distance_;
  int max_distance_sq_;

  std::vector<double> sqrt_table_;

  // neighborhoods:
  // [0] - for expansion of d=0
  // [1] - for expansion of d>=1
  // Under this, we have the 27 directions
  // Then, a list of neighborhoods for each direction
  std::vector<std::vector<std::vector<Eigen::Vector3i > > > neighborhoods_;

  std::vector<Eigen::Vector3i > direction_number_to_direction_;

  void addNewObstacleVoxels(const std::vector<Eigen::Vector3i>& voxel_points);
  //void removeObstacleVoxels(const VoxelSet& points);
  void removeObstacleVoxels(const std::vector<Eigen::Vector3i>& voxel_points);

  // starting with the voxels on the queue, propagate values to neighbors up to a certain distance.
  void propagatePositive();
  void propagateNegative();
  virtual double getDistance(const PropDistanceFieldVoxel& object) const;
  int getDirectionNumber(int dx, int dy, int dz) const;
  Eigen::Vector3i getLocationDifference(int directionNumber) const;	// TODO- separate out neighborhoods
  void initNeighborhoods();
  static int eucDistSq(Eigen::Vector3i point1, Eigen::Vector3i point2);
  void print(const VoxelSet & set);
  void print(const EigenSTL::vector_Vector3d& points);
};

////////////////////////// inline functions follow ////////////////////////////////////////

inline PropDistanceFieldVoxel::PropDistanceFieldVoxel(int distance_sq, int negative_distance_sq):
  distance_square_(distance_sq),
  negative_distance_square_(negative_distance_sq)
{
  closest_point_.x() = PropDistanceFieldVoxel::UNINITIALIZED;
  closest_point_.y() = PropDistanceFieldVoxel::UNINITIALIZED;
  closest_point_.z() = PropDistanceFieldVoxel::UNINITIALIZED;
  closest_negative_point_.x() = PropDistanceFieldVoxel::UNINITIALIZED;
  closest_negative_point_.y() = PropDistanceFieldVoxel::UNINITIALIZED;
  closest_negative_point_.z() = PropDistanceFieldVoxel::UNINITIALIZED;
}

inline PropDistanceFieldVoxel::PropDistanceFieldVoxel()
{
}

inline double PropagationDistanceField::getDistance(const PropDistanceFieldVoxel& object) const
{
  return sqrt_table_[object.distance_square_]-sqrt_table_[object.negative_distance_square_];
}

}

#endif
