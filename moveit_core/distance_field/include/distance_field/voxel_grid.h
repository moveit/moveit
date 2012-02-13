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

#ifndef DF_VOXEL_GRID_H_
#define DF_VOXEL_GRID_H_

#include <algorithm>

namespace distance_field
{

/**
 * \brief Generic container for a discretized 3D voxel grid for any class/structure
 */
template <typename T>
class VoxelGrid
{
public:
  /**
   * Constructor for the VoxelGrid.
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
  VoxelGrid(double size_x, double size_y, double size_z, double resolution,
      double origin_x, double origin_y, double origin_z, T default_object);
  virtual ~VoxelGrid();

  /**
   * \brief Gets the value at the given position (returns the value from the closest discretized location).
   */
  const T& operator()(double x, double y, double z) const;

  /**
   * \brief Gets the value at a given integer location.
   */
  T& getCell(int x, int y, int z);

  /**
   * \brief Sets the value at an integer location.
   */
  void setCell(int x, int y, int z, T& obj);

  /**
   * \brief Gets the value at a given integer location (const version).
   */
  const T& getCell(int x, int y, int z) const;

  /**
   * \brief Reset the entire grid to the given initial value.
   */
  void reset(T initial);

  enum Dimension
  {
    DIM_X = 0,
    DIM_Y = 1,
    DIM_Z = 2
  };

  /**
   * \brief Gets the size of the given dimension.
   */
  double getSize(Dimension dim) const;

  /**
   * \brief Gets the resolution of the given dimension.
   */
  double getResolution(Dimension dim) const;

  /**
   * \brief Gets the origin of the given dimension.
   */
  double getOrigin(Dimension dim) const;

  /**
   * \brief Gets the number of cells of the given dimension.
   */
  int getNumCells(Dimension dim) const;

  /**
   * \brief Converts grid coordinates to world coordinates.
   */
  bool gridToWorld(int x, int y, int z, double& world_x, double& world_y, double& world_z) const;

  /**
   * \brief Converts world coordinates to grid coordinates.
   */
  bool worldToGrid(double world_x, double world_y, double world_z, int& x, int& y, int& z) const;

protected:
  T* data_;			/**< Storage for data elements */
  T default_object_;		/**< The default object to return in case of out-of-bounds query */
  T*** data_ptrs_;
  double size_[3];
  double resolution_[3];
  double origin_[3];
  int num_cells_[3];
  int num_cells_total_;
  int stride1_;
  int stride2_;

  /**
   * \brief Gets the reference in the data_ array for the given integer x,y,z location
   */
  int ref(int x, int y, int z) const;

  /**
   * \brief Gets the cell number from the location
   */
  int getCellFromLocation(Dimension dim, double loc) const;

  /**
   * \brief Gets the location from the cell number
   */
  double getLocationFromCell(Dimension dim, int cell) const;

  /**
   * \brief Checks if the given cell is within the voxel grid
   */
  bool isCellValid(int x, int y, int z) const;

  /**
   * \brief Checks validity of the given cell for a particular dimension
   */
  bool isCellValid(Dimension dim, int cell) const;
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
  return int(round((loc-origin_[dim])/resolution_[dim]));
}

template<typename T>
inline double VoxelGrid<T>::getLocationFromCell(Dimension dim, int cell) const
{
  return origin_[dim] + resolution_[dim]*(double(cell));
}


template<typename T>
inline void VoxelGrid<T>::reset(T initial)
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
#endif /* DF_VOXEL_GRID_H_ */
