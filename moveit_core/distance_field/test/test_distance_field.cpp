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

#include <gtest/gtest.h>

#include <moveit/distance_field/voxel_grid.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <console_bridge/console.h>

using namespace distance_field;

static const double width = 0.5;//2.0;
static const double height = 0.5;//2.0;
static const double depth = 0.5;//1.0;
static const double resolution = 0.1;
static const double origin_x = 0.0;
static const double origin_y = 0.0;
static const double origin_z = 0.0;
static const double max_dist = 0.3;

static const int max_dist_in_voxels = max_dist/resolution+0.5;
static const int max_dist_sq_in_voxels = max_dist_in_voxels*max_dist_in_voxels;

static const Eigen::Vector3d point1(0.0,0.0,0.0);
static const Eigen::Vector3d point2(0.0,0.1,0.2);
static const Eigen::Vector3d point3(0.4,0.0,0.0);


int dist_sq(int x, int y, int z)
{
  return x*x + y*y +z*z;
}

void print( PropagationDistanceField& pdf, int numX, int numY, int numZ)
{
  for (int x=0; x<numX; x++) {
    for (int y=0; y<numY; y++) {
      for (int z=0; z<numZ; z++) {
        std::cout << pdf.getCell(x,y,z).distance_square_ << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
}


void check_distance_field(const PropagationDistanceField & df, const EigenSTL::vector_Vector3d& points, int numX, int numY, int numZ)
{
  // Check after adding point(s)
  // Fairly heavy computation.  Try to keep voxel grid small when doing this test
  for (int x=0; x<numX; x++) {
    for (int y=0; y<numY; y++) {
      for (int z=0; z<numZ; z++) {
        int min_dist_square = max_dist_sq_in_voxels;
        for( unsigned int i=0; i<points.size(); i++) {
          int dx = points[i].x()/resolution - x;
          int dy = points[i].y()/resolution - y;
          int dz = points[i].z()/resolution - z;
          int dist_square = dist_sq(dx,dy,dz);
          min_dist_square = std::min(dist_square, min_dist_square);
        }
        ASSERT_EQ(df.getCell(x, y, z).distance_square_, min_dist_square);
      }
    }
  }
}


TEST(TestPropagationDistanceField, TestAddPoints)
{

  PropagationDistanceField df( width, height, depth, resolution, origin_x, origin_y, origin_z, max_dist);

  // Check size
  int numX = df.getXNumCells();
  int numY = df.getYNumCells();
  int numZ = df.getZNumCells();
  
  EXPECT_EQ( numX, (int)(width/resolution+0.5) );
  EXPECT_EQ( numY, (int)(height/resolution+0.5) );
  EXPECT_EQ( numZ, (int)(depth/resolution+0.5) );

  // Error checking
  //print(df, numX, numY, numZ);

  // TODO - check initial values
  //EXPECT_EQ( df.getCell(0,0,0).distance_square_, max_dist_sq_in_voxels );

  // Add points to the grid
  EigenSTL::vector_Vector3d points;
  points.push_back(point1);
  points.push_back(point2);
  df.reset();
  logInform("Adding %u points", points.size());
  df.addPointsToField(points);
  print(df, numX, numY, numZ);

  // Error checking
  //print(df, numX, numY, numZ);

  // Check correctness
  check_distance_field( df, points, numX, numY, numZ);

  // Update - iterative
  points.clear();
  points.push_back(point1);
  points.push_back(point3);
  df.updatePointsInField(points,true);
  //print(df, numX, numY, numZ);
  check_distance_field( df, points, numX, numY, numZ);

  // Remove
  points.clear();
  points.push_back(point1);
  df.removePointsFromField(points);
  //print(df, numX, numY, numZ);
  points.clear();
  points.push_back(point3);
  check_distance_field( df, points, numX, numY, numZ);


  // Update - not iterative
  points.clear();
  points.push_back(point2);
  points.push_back(point3);
  df.updatePointsInField(points,false);
  print(df, numX, numY, numZ);
  check_distance_field( df, points, numX, numY, numZ);

  // TODO - test gradient and closest point location

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
