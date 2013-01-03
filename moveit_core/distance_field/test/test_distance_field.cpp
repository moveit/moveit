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

static const double width = 1.0;
static const double height = 1.0;
static const double depth = 1.0;
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

void printNeg(PropagationDistanceField& pdf, int numX, int numY, int numZ)
{
  for (int x=0; x<numX; x++) {
    for (int y=0; y<numY; y++) {
      for (int z=0; z<numZ; z++) {
        std::cout << pdf.getCell(x,y,z).negative_distance_square_ << " ";
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
  points.push_back(point3);
  EigenSTL::vector_Vector3d old_points;
  old_points.push_back(point1);
  df.updatePointsInField(old_points, points);
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

}

// TEST(TestSignedPropagationDistanceField, TestAddPoints)
// {

//   PropagationDistanceField df( width, height, depth, resolution, origin_x, origin_y, origin_z, max_dist, true);

//   // Check size
//   int numX = df.getXNumCells();
//   int numY = df.getYNumCells();
//   int numZ = df.getZNumCells();
  
//   EXPECT_EQ( numX, (int)(width/resolution+0.5) );
//   EXPECT_EQ( numY, (int)(height/resolution+0.5) );
//   EXPECT_EQ( numZ, (int)(depth/resolution+0.5) );

//   // Error checking
//   //print(df, numX, numY, numZ);

//   // TODO - check initial values
//   //EXPECT_EQ( df.getCell(0,0,0).distance_square_, max_dist_sq_in_voxels );

//   // Add points to the grid
//   double lwx, lwy, lwz;
//   double hwx, hwy, hwz;
//   df.gridToWorld(1,1,1,lwx,lwy,lwz);
//   df.gridToWorld(8,8,8,hwx,hwy,hwz);

//   EigenSTL::vector_Vector3d points;
//   for(double x = lwx; x <= hwx; x+= .1) {
//     for(double y = lwy; y <= hwy; y+= .1) {
//       for(double z = lwz; z <= hwz; z+= .1) {
//         points.push_back(Eigen::Vector3d(x,y,z));
//       }
//     }
//   }

//   df.reset();
//   logInform("Adding %u points", points.size());
//   df.addPointsToField(points);
//   print(df, numX, numY, numZ);
//   printNeg(df, numX, numY, numZ);

//   double cx, cy, cz;
//   df.gridToWorld(5,5,5,cx,cy,cz);

//   Eigen::Vector3d center_point(cx,cy,cz);

//   EigenSTL::vector_Vector3d rem_points;
//   rem_points.push_back(center_point);
//   df.removePointsFromField(rem_points);
//   std::cout << "Pos "<< std::endl;
//   print(df, numX, numY, numZ);
//   std::cout << "Neg "<< std::endl;
//   printNeg(df, numX, numY, numZ);
// }


// TEST(TestSignedPropagationDistanceField, TestShape)
// {

//   PropagationDistanceField df( width, height, depth, resolution, origin_x, origin_y, origin_z, max_dist, true);

//   int numX = df.getXNumCells();
//   int numY = df.getYNumCells();
//   int numZ = df.getZNumCells();
  
//   shape_msgs::SolidPrimitive sphere;
//   sphere.type = shape_msgs::SolidPrimitive::SPHERE;
//   sphere.dimensions.resize(1);
//   sphere.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = .25;
  
//   geometry_msgs::Pose p;
//   p.orientation.w = 1.0;
//   p.position.x = .5;
//   p.position.y = .5;
//   p.position.z = .5;

//   df.addShapeToField(sphere, p);

//   std::cout << "Shape pos "<< std::endl;
//   print(df, numX, numY, numZ);
//   std::cout << "Shape neg "<< std::endl;
//   printNeg(df, numX, numY, numZ);

//   EXPECT_EQ(df.getCell(5, 5, 5).distance_square_, 0);
//   EXPECT_EQ(df.getCell(5, 5, 5).negative_distance_square_, 3);  

//   geometry_msgs::Pose np;
//   np.orientation.w = 1.0;
//   np.position.x = .7;
//   np.position.y = .7;
//   np.position.z = .7;  
  
//   df.moveShapeInField(sphere, p, np);

//   EXPECT_GT(df.getCell(5, 5, 5).distance_square_, 0);
//   EXPECT_LT(df.getCell(5, 5, 5).negative_distance_square_, 3);  

//   std::cout << "Shape pos move "<< std::endl;
//   print(df, numX, numY, numZ);
//   std::cout << "Shape neg move "<< std::endl;
//   printNeg(df, numX, numY, numZ);

// }

// static const double PERF_WIDTH = 3.0;
// static const double PERF_HEIGHT = 3.0;
// static const double PERF_DEPTH = 4.0;
// static const double PERF_RESOLUTION = 0.02;
// static const double PERF_ORIGIN_X = 0.0;
// static const double PERF_ORIGIN_Y = 0.0;
// static const double PERF_ORIGIN_Z = 0.0;
// static const double PERF_MAX_DIST = .25;

// TEST(TestSignedPropagationDistanceField, TestPerformance)
// {
//   std::cout << "Creating distance field with " <<
//     (PERF_WIDTH/PERF_RESOLUTION)*(PERF_HEIGHT/PERF_RESOLUTION)*(PERF_DEPTH/PERF_RESOLUTION)
//             << " entries" << std::endl;

//   ros::WallTime dt = ros::WallTime::now();
//   PropagationDistanceField df( PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, 
//                                PERF_ORIGIN_X, PERF_ORIGIN_Y, PERF_ORIGIN_Z, PERF_MAX_DIST, false);
//   std::cout << "Creating unsigned took " << (ros::WallTime::now()-dt).toSec() << std::endl;

//   dt = ros::WallTime::now();
//   PropagationDistanceField sdf( PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, 
//                                 PERF_ORIGIN_X, PERF_ORIGIN_Y, PERF_ORIGIN_Z, PERF_MAX_DIST, true);

//   std::cout << "Creating signed took " << (ros::WallTime::now()-dt).toSec() << std::endl;

//   shape_msgs::SolidPrimitive big_table;
//   big_table.type = shape_msgs::SolidPrimitive::BOX;
//   big_table.dimensions.resize(3);
//   big_table.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.5;
//   big_table.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
//   big_table.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = .25;
  
//   geometry_msgs::Pose p;
//   p.orientation.w = 1.0;
//   p.position.x = PERF_WIDTH/2.0;
//   p.position.y = PERF_DEPTH/2.0;
//   p.position.z = PERF_HEIGHT/2.0;

//   geometry_msgs::Pose np;
//   np.orientation.w = 1.0;
//   np.position.x = p.position.x+.1;
//   np.position.y = p.position.y;
//   np.position.z = p.position.z;

//   std::cout << "Adding " << (1.5/PERF_RESOLUTION)*(1.5/PERF_RESOLUTION)*(.25/PERF_RESOLUTION) << " points" << std::endl;

//   dt = ros::WallTime::now();
//   df.addShapeToField(big_table, p);
//   std::cout << "Adding to unsigned took " << (ros::WallTime::now()-dt).toSec() << std::endl;  

//   dt = ros::WallTime::now();
//   df.addShapeToField(big_table, p);
//   std::cout << "Re-adding to unsigned took " << (ros::WallTime::now()-dt).toSec() << std::endl;  

//   dt = ros::WallTime::now();
//   sdf.addShapeToField(big_table, p);
//   std::cout << "Adding to signed took " << (ros::WallTime::now()-dt).toSec() << std::endl;  
  
//   dt = ros::WallTime::now();
//   df.removeShapeFromField(big_table, p);
//   std::cout << "Removing from unsigned took " << (ros::WallTime::now()-dt).toSec() << std::endl;  
  
//   dt = ros::WallTime::now();
//   sdf.removeShapeFromField(big_table, p);
//   std::cout << "Removing from signed took " << (ros::WallTime::now()-dt).toSec() << std::endl;  

//   shape_msgs::SolidPrimitive small_table;
//   small_table.type = shape_msgs::SolidPrimitive::BOX;
//   small_table.dimensions.resize(3);
//   small_table.dimensions[shape_msgs::SolidPrimitive::BOX_X] = .25;
//   small_table.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = .25;
//   small_table.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = .05;

//   std::cout << "Adding " << (13)*(13)*(3) << " points" << std::endl;

//   dt = ros::WallTime::now();
//   df.addShapeToField(small_table, p);
//   std::cout << "Adding to unsigned took " << (ros::WallTime::now()-dt).toSec() << std::endl;  

//   dt = ros::WallTime::now();
//   sdf.addShapeToField(small_table, p);
//   std::cout << "Adding to signed took " << (ros::WallTime::now()-dt).toSec() << std::endl;  

//   dt = ros::WallTime::now();
//   df.moveShapeInField(small_table, p, np);
//   std::cout << "Moving in unsigned took " << (ros::WallTime::now()-dt).toSec() << std::endl;  

//   dt = ros::WallTime::now();
//   sdf.moveShapeInField(small_table, p, np);
//   std::cout << "Moving in signed took " << (ros::WallTime::now()-dt).toSec() << std::endl;  

// }


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
