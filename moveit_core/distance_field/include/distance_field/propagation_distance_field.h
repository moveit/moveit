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

#ifndef DF_PROPAGATION_DISTANCE_FIELD_H_
#define DF_PROPAGATION_DISTANCE_FIELD_H_

#include <distance_field/voxel_grid.h>
#include <distance_field/distance_field.h>
#include <vector>
#include <list>
#include <ros/ros.h>
#include <Eigen/Core>
#include <set>

namespace distance_field
{

// less-than Comparison
bool lessThan(int3 loc_1, int3 loc_2);

int equal(int3 loc_1, int3 loc_2);

// Class
struct compareInt3
{
  bool operator()(int3 loc_1, int3 loc_2) const
  {
    return lessThan(loc_1,loc_2);
  }
};


/**
 * \brief Structure that holds voxel information for the DistanceField.
 */
struct PropDistanceFieldVoxel
{
  PropDistanceFieldVoxel();
  PropDistanceFieldVoxel(int distance_sq);

  int distance_square_;         /**< Squared distance from the closest obstacle */
  int3 closest_point_;	        /**< Closes obstacle from this voxel */
  int update_direction_;        /**< Direction from which this voxel was updated */

  static const int UNINITIALIZED=-1;
};

struct SignedPropDistanceFieldVoxel : public PropDistanceFieldVoxel
{
    SignedPropDistanceFieldVoxel();
    SignedPropDistanceFieldVoxel(int distance_sq_positive, int distance_sq_negative);
    int positive_distance_square_;
    int negative_distance_square_;
    int3 closest_positive_point_;
    int3 closest_negative_point_;

    static const int UNINITIALIZED=-999;
};

/**
 * \brief A DistanceField implementation that uses a vector propagation method.
 *
 * It computes the distance transform of the input points, and stores the distance to
 * the closest obstacle in each voxel. Also available is the location of the closest point,
 * and the gradient of the field at a point. Expansion of obstacles is performed upto a given
 * radius.
 */
class PropagationDistanceField: public DistanceField
{
public:


  /**
   * \brief Constructor for the DistanceField.
   */
  PropagationDistanceField(double size_x, double size_y, double size_z, double resolution,
                           double origin_x, double origin_y, double origin_z, double max_distance);
  
  virtual ~PropagationDistanceField();
  
  /**
   * \brief Change the set of obstacle points and recalculate the distance field (if there are any changes).
   * \param iterative Calculate the changes in the object voxels, and propogate the changes outward.
   *        Otherwise, clear the distance map and recalculate the entire voxel map.
   */
  virtual void updatePointsInField(const EigenSTL::vector_Vector3d& points, const bool iterative=true);

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

  const PropDistanceFieldVoxel& getCell(int x, int y, int z) const {
    return voxel_grid_.getCell(x, y, z);
  }

private:

  VoxelGrid<PropDistanceFieldVoxel> voxel_grid_;

  /// \brief The set of all the obstacle voxels
  typedef std::set<int3, compareInt3> VoxelSet;
  VoxelSet object_voxel_locations_;

  /// \brief Structure used to hold propogation frontier
  std::vector<std::vector<int3> > bucket_queue_;
  double max_distance_;
  int max_distance_sq_;

  std::vector<double> sqrt_table_;

  // neighborhoods:
  // [0] - for expansion of d=0
  // [1] - for expansion of d>=1
  // Under this, we have the 27 directions
  // Then, a list of neighborhoods for each direction
  std::vector<std::vector<std::vector<int3 > > > neighborhoods_;

  std::vector<int3 > direction_number_to_direction_;

  void addNewObstacleVoxels(const VoxelSet& points);
  void removeObstacleVoxels(const VoxelSet& points);
  // starting with the voxels on the queue, propogate values to neighbors up to a certain distance.
  void propogate();
  virtual double getDistance(const PropDistanceFieldVoxel& object) const;
  int getDirectionNumber(int dx, int dy, int dz) const;
  int3 getLocationDifference(int directionNumber) const;	// TODO- separate out neighborhoods
  void initNeighborhoods();
  static int eucDistSq(int3 point1, int3 point2);
  void print(const VoxelSet & set);
  void print(const EigenSTL::vector_Vector3d& points);
};

////////////////////////// inline functions follow ////////////////////////////////////////

inline PropDistanceFieldVoxel::PropDistanceFieldVoxel(int distance_sq):
  distance_square_(distance_sq)
{
  closest_point_.x() = PropDistanceFieldVoxel::UNINITIALIZED;
  closest_point_.y() = PropDistanceFieldVoxel::UNINITIALIZED;
  closest_point_.z() = PropDistanceFieldVoxel::UNINITIALIZED;
}

inline PropDistanceFieldVoxel::PropDistanceFieldVoxel()
{
}

inline double PropagationDistanceField::getDistance(const PropDistanceFieldVoxel& object) const
{
  return sqrt_table_[object.distance_square_];
}


class SignedPropagationDistanceField : public DistanceField
{
  public:
  
  SignedPropagationDistanceField(double size_x, double size_y, double size_z, double resolution, double origin_x,
                                 double origin_y, double origin_z, double max_distance);
  virtual ~SignedPropagationDistanceField();
  
  virtual void addPointsToField(const EigenSTL::vector_Vector3d &points);

  virtual void removePointsFromField(const EigenSTL::vector_Vector3d &points)
  {
    reset();
  }
  
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

private:

  VoxelGrid<SignedPropDistanceFieldVoxel> voxel_grid_;

  std::vector<std::vector<int3> > positive_bucket_queue_;
  std::vector<std::vector<int3> > negative_bucket_queue_;
  double max_distance_;
  int max_distance_sq_;
  
  std::vector<double> sqrt_table_;
  
  // [0] - for expansion of d=0
  // [1] - for expansion of d>=1
  // Under this, we have the 27 directions
  // Then, a list of neighborhoods for each direction
  std::vector<std::vector<std::vector<int3 > > > neighborhoods_;
  
  std::vector<int3 > direction_number_to_direction_;
  
  virtual double getDistance(const SignedPropDistanceFieldVoxel& object) const;
  int getDirectionNumber(int dx, int dy, int dz) const;
  void initNeighborhoods();
  static int eucDistSq(int3 point1, int3 point2);
};



inline SignedPropDistanceFieldVoxel::SignedPropDistanceFieldVoxel(int distance_sq_positive, int distance_sq_negative):
  positive_distance_square_(distance_sq_positive),
  negative_distance_square_(distance_sq_negative),
  closest_positive_point_(SignedPropDistanceFieldVoxel::UNINITIALIZED),
  closest_negative_point_(SignedPropDistanceFieldVoxel::UNINITIALIZED)
{
}

inline SignedPropDistanceFieldVoxel::SignedPropDistanceFieldVoxel()
{
}

inline double SignedPropagationDistanceField::getDistance(const SignedPropDistanceFieldVoxel& object) const
{
  if(object.negative_distance_square_ != 0)
  {
    //ROS_INFO("Positive Distance %d, Negative Distance %d",object.positive_distance_square_,object.negative_distance_square_ );
  }
  return sqrt_table_[object.positive_distance_square_] - sqrt_table_[object.negative_distance_square_];
}

}

#endif /* DF_PROPAGATION_DISTANCE_FIELD_H_ */
