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

/* Author: Mrinal Kalakrishnan */

#ifndef MOVEIT_DISTANCE_FIELD_VOXEL_GRID_
#define MOVEIT_DISTANCE_FIELD_VOXEL_GRID_

#include <algorithm>
#include <cmath>

namespace distance_field
{

/// \brief Specifies dimension of different axes
enum Dimension {
  DIM_X = 0,
  DIM_Y = 1,
  DIM_Z = 2
};

/**
 * \brief VoxelGrid holds a dense 3D, axis-aligned set of data at a
 * given resolution, where the data is supplied as a template
 * parameter.
 *
 */
template <typename T>
class VoxelGrid
{
public:
  /**
   * \brief Constructor for the VoxelGrid.
   *
   * Constructs a dense representation of a 3D, axis-aligned volume at
   * a given resolution.  The volume can be represented in any
   * consistent set of units, but for the sake of documentation we
   * assume that the units are meters.  The size of the the volume is
   * given along each of the X, Y, and Z axes.  The volume begins at
   * the minimum point in each dimension, as specified by the origin
   * parameters.  The data structure will remain unintialized until
   * the \ref VoxelGrid::reset function is called.
   *
   * @param [in] size_x Size of the X axis in meters
   * @param [in] size_y Size of the Y axis in meters
   * @param [in] size_z Size of the Z axis in meters
   *
   * @param [in] resolution Resolution of a single cell in meters
   *
   * @param [in] origin_x Minimum point along the X axis of the volume
   * @param [in] origin_y Minimum point along the Y axis of the volume
   * @param [in] origin_z Minimum point along the Z axis of the volume
   *
   * @param [in] default_object An object that will be returned for any
   * future queries that are not valid
   */
  VoxelGrid(double size_x, double size_y, double size_z, double resolution,
            double origin_x, double origin_y, double origin_z, T default_object);
  virtual ~VoxelGrid();

  /**
   * \brief Operator that gets the value of the given location (x, y,
   * z) given the discretization of the volume.  The location
   * represents a location in the original coordinate frame used to
   * construct the voxel grid.
   *
   * @param [in] x X component of the desired location
   * @param [in] y Y component of the desired location
   * @param [in] z Z component of the desired location
   *
   * @return The data stored at that location, or a default value
   * supplied in the constructor if the location is not valid.
   */
  const T& operator()(double x, double y, double z) const;

  /**
   * \brief Gives the value of the given location (x,y,z) in the
   * discretized voxel grid space.
   *
   * The address here is in the discretized space of the voxel grid,
   * where the cell indicated by the constructor arguments (origin_x,
   * origin_y, origin_z) is cell (0,0,0), and the cell indicated by
   * (origin_x+x_size, origin_y+y_size, origin_z+z_size) will be
   * (size_x/resolution, size_y/resolution, size_z/resolution).
   *
   * @param [in] x The X index of the desired cell
   * @param [in] y The Y index of the desired cell
   * @param [in] z The Z index of the desired cell
   *
   * @return The data in the indicated cell, or a default value
   * supplied in the constructor if the location is not valid.
   */
  T& getCell(int x, int y, int z);

  /**
   * \brief Sets the value of the given location (x,y,z) in the
   * discretized voxel grid space to supplied value.
   *
   * The address here is in the discretized space of the voxel grid,
   * where the cell indicated by the constructor arguments (origin_x,
   * origin_y, origin_z) is cell (0,0,0), and the cell indicated by
   * (origin_x+x_size, origin_y+y_size, origin_z+z_size) will be
   * (size_x/resolution, size_y/resolution, size_z/resolution).
   *
   * If the arguments do not indicate a valid cell, no action is
   * taken.
   *
   * @param [in] x The X index of the desired cell
   * @param [in] y The Y index of the desired cell
   * @param [in] z The Z index of the desired cell
   * @param [out] obj The data to place into the given cell
   */
  void setCell(int x, int y, int z, T& obj);

  /**
   * The const version of the the function in \ref VoxelGrid::getCell(int x, int y, int z).
   */
  const T& getCell(int x, int y, int z) const;

  /**
   * \brief Sets every cell in the voxel grid to the supplied data
   *
   * @param [in] initial The template variable to which to set the data
   */
  void reset(const T& initial);

  /**
   * \brief Gets the size in arbitrary units of the indicated dimension
   *
   * @param [in] dim The dimension for the query
   *
   * @return The size in meters
   */
  double getSize(Dimension dim) const;

  /**
   * \brief Gets the resolution of the indicated dimension in arbitrary consistent units
   *
   * @param [in] dim The dimension for the query
   *
   * @return The resolution in meters
   */
  double getResolution(Dimension dim) const;

  /**
   * \brief Gets the origin (miniumum point) of the indicated dimension
   *
   * @param [in] dim The dimension for the query
   *
   * @return The indicated axis origin
   */
  double getOrigin(Dimension dim) const;

  /**
   * \brief Gets the number of cells in the indicated dimension
   *
   * @param [in] dim The dimension for the query
   *
   * @return The number of cells for the indicated dimension
   */
  int getNumCells(Dimension dim) const;

  /**
   * \brief Converts grid coordinates to world coordinates.
   */
  /**
   * \brief Converts from an set of integer indices to a world
   * location given the origin and resolution parameters.  There is no
   * check whether or not the cell or world locations lie within the
   * represented region.
   *
   * @param [in] x The integer X location
   * @param [in] y The integer Y location
   * @param [in] z The integer Z location
   * @param [out] world_x The computed world X location
   * @param [out] world_y The computed world X location
   * @param [out] world_z The computed world X location
   *
   * @return True, as there is no check that the integer locations are valid
   */
  bool gridToWorld(int x, int y, int z, double& world_x, double& world_y, double& world_z) const;

  /**
   * \brief Converts from a world location to a set of integer
   * indices.  Does check whether or not the cell being returned is
   * valid.  The returned indices will be computed even if they are
   * invalid.
   *
   * @param [in] world_x The world X location
   * @param [in] world_y The world Y location
   * @param [in] world_z The world Z location
   * @param [out] x The computed integer X location
   * @param [out] y The computed integer X location
   * @param [out] z The computed integer X location
   *
   * @return True if all the world values result in integer indices
   * that pass a validity check; otherwise False.
   */
  bool worldToGrid(double world_x, double world_y, double world_z, int& x, int& y, int& z) const;

  /**
   * \brief Checks if the given cell in integer coordinates is within the voxel grid
   *
   * @param [in] x The integer X location
   * @param [in] y The integer Y location
   * @param [in] z The integer Z location
   *
   * @return True if the cell lies within the voxel grid; otherwise False.
   */
  bool isCellValid(int x, int y, int z) const;

  /**
   * \brief Checks if the indicated index is valid along a particular dimension.
   *
   * @param [in] dim The dimension for the query
   * @param [in] cell The index along that dimension
   *
   * @return True if the cell is valid along that dimension; otherwise False.
   */
  bool isCellValid(Dimension dim, int cell) const;

protected:
  T* data_;                     /**< \brief Storage for the full set of data elements */
  T default_object_;            /**< \brief The default object to return in case of out-of-bounds query */
  T*** data_ptrs_;              /**< \brief 3D array of pointers to the data elements */
  double size_[3];              /**< \brief The size of each dimension in meters (in Dimension order) */
  double resolution_[3];        /**< \brief The resolution of each dimension in meters (in Dimension order) */
  double origin_[3];            /**< \brief The origin (minumum point) of each dimension in meters (in Dimension order) */
  int num_cells_[3];            /**< \brief The number of cells in each dimension (in Dimension order) */
  int num_cells_total_;         /**< \brief The total number of voxels in the grid */
  int stride1_;                 /**< \brief The step to take when stepping between consecutive X members in the 1D array */
  int stride2_;                 /**< \brief The step to take when stepping between consecutive Y members given an X in the 1D array */

  /**
   * \brief Gets the 1D index into the array, with no validity check.
   *
   * @param [in] x The integer X location
   * @param [in] y The integer Y location
   * @param [in] z The integer Z location
   *
   * @return The computed 1D index
   */
  int ref(int x, int y, int z) const;

  /**
   * \brief Gets the cell number from the location
   */
  /**
   * \brief Gets the cell number in a given dimension given a world
   * value.  No validity check.
   *
   * @param [in] dim The dimension of the query
   * @param [in] loc The world location along that dimension
   *
   * @return The computed cell index along the given dimension
   */
  int getCellFromLocation(Dimension dim, double loc) const;

  /**
   * \brief Gets the center of the cell in world coordinates along the
   * given dimension.  No validity check.
   *
   * @param [in] dim The dimension of the query
   * @param [in] cell The cell along the given dimension
   *
   * @return The world coordinate of the center of the cell
   */
  double getLocationFromCell(Dimension dim, int cell) const;

};

//////////////////////////// template function definitions follow //////////////////

template<typename T>
VoxelGrid<T>::VoxelGrid(double size_x, double size_y, double size_z, double resolution,
    double origin_x, double origin_y, double origin_z, T default_object)
{
  size_[DIM_X] = size_x;
  size_[DIM_Y] = size_y;
  size_[DIM_Z] = size_z;
  origin_[DIM_X] = origin_x;
  origin_[DIM_Y] = origin_y;
  origin_[DIM_Z] = origin_z;
  num_cells_total_ = 1;
  for (int i=DIM_X; i<=DIM_Z; ++i)
  {
    resolution_[i] = resolution;
    num_cells_[i] = size_[i] / resolution_[i];
    num_cells_total_ *= num_cells_[i];
  }
  default_object_ = default_object;

  stride1_ = num_cells_[DIM_Y]*num_cells_[DIM_Z];
  stride2_ = num_cells_[DIM_Z];

  // initialize the data:
  data_ = new T[num_cells_total_];

}

template<typename T>
VoxelGrid<T>::~VoxelGrid()
{
  delete[] data_;
}

template<typename T>
inline bool VoxelGrid<T>::isCellValid(int x, int y, int z) const
{
  return (
      x>=0 && x<num_cells_[DIM_X] &&
      y>=0 && y<num_cells_[DIM_Y] &&
      z>=0 && z<num_cells_[DIM_Z]);
}

template<typename T>
inline bool VoxelGrid<T>::isCellValid(Dimension dim, int cell) const
{
  return cell>=0 && cell<num_cells_[dim];
}

template<typename T>
inline int VoxelGrid<T>::ref(int x, int y, int z) const
{
  return x*stride1_ + y*stride2_ + z;
}

template<typename T>
inline double VoxelGrid<T>::getSize(Dimension dim) const
{
  return size_[dim];
}

template<typename T>
inline double VoxelGrid<T>::getResolution(Dimension dim) const
{
  return resolution_[dim];
}

template<typename T>
inline double VoxelGrid<T>::getOrigin(Dimension dim) const
{
  return origin_[dim];
}

template<typename T>
inline int VoxelGrid<T>::getNumCells(Dimension dim) const
{
  return num_cells_[dim];
}

template<typename T>
inline const T& VoxelGrid<T>::operator()(double x, double y, double z) const
{
  int cellX = getCellFromLocation(DIM_X, x);
  int cellY = getCellFromLocation(DIM_Y, y);
  int cellZ = getCellFromLocation(DIM_Z, z);
  if (!isCellValid(cellX, cellY, cellZ))
    return default_object_;
  return getCell(cellX, cellY, cellZ);
}

template<typename T>
inline T& VoxelGrid<T>::getCell(int x, int y, int z)
{
  return data_[ref(x,y,z)];
}

template<typename T>
inline const T& VoxelGrid<T>::getCell(int x, int y, int z) const
{
  return data_[ref(x,y,z)];
}

template<typename T>
inline void VoxelGrid<T>::setCell(int x, int y, int z, T& obj)
{
  data_[ref(x,y,z)] = obj;
}

template<typename T>
inline int VoxelGrid<T>::getCellFromLocation(Dimension dim, double loc) const
{
  return int(floor((loc-origin_[dim])/resolution_[dim] + 0.5));
}

template<typename T>
inline double VoxelGrid<T>::getLocationFromCell(Dimension dim, int cell) const
{
  return origin_[dim] + resolution_[dim]*(double(cell));
}


template<typename T>
inline void VoxelGrid<T>::reset(const T& initial)
{
  std::fill(data_, data_+num_cells_total_, initial);
}

template<typename T>
inline bool VoxelGrid<T>::gridToWorld(int x, int y, int z, double& world_x, double& world_y, double& world_z) const
{
  world_x = getLocationFromCell(DIM_X, x);
  world_y = getLocationFromCell(DIM_Y, y);
  world_z = getLocationFromCell(DIM_Z, z);
  return true;
}

template<typename T>
inline bool VoxelGrid<T>::worldToGrid(double world_x, double world_y, double world_z, int& x, int& y, int& z) const
{
  x = getCellFromLocation(DIM_X, world_x);
  y = getCellFromLocation(DIM_Y, world_y);
  z = getCellFromLocation(DIM_Z, world_z);
  return isCellValid(x,y,z);
}

} // namespace distance_field
#endif
