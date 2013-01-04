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
#include <moveit/distance_field/distance_field_common.h>
#include <console_bridge/console.h>
#include <geometric_shapes/body_operations.h>

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

static const Eigen::Vector3d point1(0.1,0.0,0.0);
static const Eigen::Vector3d point2(0.0,0.1,0.2);
static const Eigen::Vector3d point3(0.4,0.0,0.0);

int dist_sq(int x, int y, int z)
{
  return x*x + y*y +z*z;
}

void print( PropagationDistanceField& pdf, int numX, int numY, int numZ)
{
  for (int z=0; z<numZ; z++) {
    for (int x=0; x<numX; x++) {
      for (int y=0; y<numY; y++) {
        std::cout << pdf.getCell(x,y,z).distance_square_ << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
  for (int z=0; z<numZ; z++) {
    for (int x=0; x<numX; x++) {
      for (int y=0; y<numY; y++) {
        if(pdf.getCell(x,y,z).distance_square_ == 0) {
          //logInform("Obstacle cell %d %d %d", x, y, z);
        }
      }
    }
  }
}

void printNeg(PropagationDistanceField& pdf, int numX, int numY, int numZ)
{
  for (int z=0; z<numZ; z++) {
    for (int x=0; x<numX; x++) {
      for (int y=0; y<numY; y++) {
        std::cout << pdf.getCell(x,y,z).negative_distance_square_ << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
}

bool areDistanceFieldsDistancesEqual(const PropagationDistanceField& df1,
                                     const PropagationDistanceField& df2)
{
  if(df1.getXNumCells() != df2.getXNumCells()) return false;
  if(df1.getYNumCells() != df2.getYNumCells()) return false;
  if(df1.getZNumCells() != df2.getZNumCells()) return false;
  for (int z=0; z<df1.getYNumCells(); z++) {
    for (int x=0; x<df1.getXNumCells(); x++) {
      for (int y=0; y<df1.getYNumCells(); y++) {
        if(df1.getCell(x,y,z).distance_square_ != df2.getCell(x,y,z).distance_square_) {
          logInform("Cell %d %d %d distances not equal %d %d",x,y,z,
                    df1.getCell(x,y,z).distance_square_,df2.getCell(x,y,z).distance_square_);
          return false;
        }
        if(df1.getCell(x,y,z).negative_distance_square_ != df2.getCell(x,y,z).negative_distance_square_) {
          logInform("Cell %d %d %d neg distances not equal %d %d",x,y,z,
                    df1.getCell(x,y,z).negative_distance_square_,df2.getCell(x,y,z).negative_distance_square_);
          return false;
        }
      }
    }
  }
  return true;
}

//points should contain all occupied points
void check_distance_field(const PropagationDistanceField & df, 
                          const EigenSTL::vector_Vector3d& points, 
                          int numX, int numY, int numZ, 
                          bool do_negs)
{
  std::vector<Eigen::Vector3i> points_ind(points.size());
  for(unsigned int i = 0; i < points.size(); i++) {
    Eigen::Vector3i loc;
    bool valid = df.worldToGrid(points[i].x(), points[i].y(), points[i].z(),
                                loc.x(), loc.y(), loc.z() );
    points_ind[i] = loc;
  }

  for (int x=0; x<numX; x++) {
    for (int y=0; y<numY; y++) {
      for (int z=0; z<numZ; z++) {
        double dsq = df.getCell(x, y, z).distance_square_;
        double ndsq = df.getCell(x, y, z).negative_distance_square_;
        if(dsq == 0) {
          bool found = false;
          for(unsigned int i = 0; i < points_ind.size(); i++) {
            if(points_ind[i].x() == x && points_ind[i].y() == y && points_ind[i].z() == z) {
              found = true;
              break;
            }
          }
          if(do_negs) {
            ASSERT_GT(ndsq, 0) << "Obstacle point " << x << " " << y << " " << z << " has zero negative value";
          }
          ASSERT_TRUE(found) << "Obstacle point " << x << " " << y << " " << z << " not found";
        }
      }
    }
  }
}


TEST(TestPropagationDistanceField, TestAddRemovePoints)
{
  PropagationDistanceField df( width, height, depth, resolution, origin_x, origin_y, origin_z, max_dist);

  // Check size
  int numX = df.getXNumCells();
  int numY = df.getYNumCells();
  int numZ = df.getZNumCells();
  
  EXPECT_EQ( numX, (int)(width/resolution+0.5) );
  EXPECT_EQ( numY, (int)(height/resolution+0.5) );
  EXPECT_EQ( numZ, (int)(depth/resolution+0.5) );

  // Add points to the grid
  EigenSTL::vector_Vector3d points;
  points.push_back(point1);
  points.push_back(point2);
  logInform("Adding %u points", points.size());
  df.addPointsToField(points);
  //print(df, numX, numY, numZ);

  // Update - iterative
  points.clear();
  points.push_back(point2);
  points.push_back(point3);
  EigenSTL::vector_Vector3d old_points;
  old_points.push_back(point1);
  df.updatePointsInField(old_points, points);
  //std::cout << "One removal, one addition" << std::endl;
  //print(df, numX, numY, numZ);
  //printNeg(df, numX, numY, numZ);
  check_distance_field(df, points, numX, numY, numZ, false);

  // Remove
  points.clear();
  points.push_back(point2);
  df.removePointsFromField(points);
  points.clear();
  points.push_back(point3);
  check_distance_field( df, points, numX, numY, numZ, false);
}

TEST(TestSignedPropagationDistanceField, TestSignedAddRemovePoints)
{

  PropagationDistanceField df( width, height, depth, resolution, origin_x, origin_y, origin_z, max_dist, true);

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
  double lwx, lwy, lwz;
  double hwx, hwy, hwz;
  df.gridToWorld(1,1,1,lwx,lwy,lwz);
  df.gridToWorld(8,8,8,hwx,hwy,hwz);

  EigenSTL::vector_Vector3d points;
  for(double x = lwx; x <= hwx; x+= .1) {
    for(double y = lwy; y <= hwy; y+= .1) {
      for(double z = lwz; z <= hwz; z+= .1) {
        points.push_back(Eigen::Vector3d(x,y,z));
      }
    }
  }

  df.reset();
  logInform("Adding %u points", points.size());
  df.addPointsToField(points);
  //print(df, numX, numY, numZ);
  //printNeg(df, numX, numY, numZ);

  double cx, cy, cz;
  df.gridToWorld(5,5,5,cx,cy,cz);

  Eigen::Vector3d center_point(cx,cy,cz);

  EigenSTL::vector_Vector3d rem_points;
  rem_points.push_back(center_point);
  df.removePointsFromField(rem_points);
  //std::cout << "Pos "<< std::endl;
  //print(df, numX, numY, numZ);
  //std::cout << "Neg "<< std::endl;
  //printNeg(df, numX, numY, numZ);

  //testing equality with initial add of points without the center point
  PropagationDistanceField test_df( width, height, depth, resolution, origin_x, origin_y, origin_z, max_dist, true);
  EigenSTL::vector_Vector3d test_points;
  for(unsigned int i = 0; i < points.size(); i++) {
    if(points[i].x() != center_point.x() ||
       points[i].y() != center_point.y() ||
       points[i].z() != center_point.z())
    {
      test_points.push_back(points[i]);
    }
  }
  test_df.addPointsToField(test_points);  
  ASSERT_TRUE(areDistanceFieldsDistancesEqual(df, test_df));
}


TEST(TestSignedPropagationDistanceField, TestShape)
{

  PropagationDistanceField df( width, height, depth, resolution, origin_x, origin_y, origin_z, max_dist, true);

  int numX = df.getXNumCells();
  int numY = df.getYNumCells();
  int numZ = df.getZNumCells();
  
  shape_msgs::SolidPrimitive sphere;
  sphere.type = shape_msgs::SolidPrimitive::SPHERE;
  sphere.dimensions.resize(1);
  sphere.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = .25;
  
  geometry_msgs::Pose p;
  p.orientation.w = 1.0;
  p.position.x = .5;
  p.position.y = .5;
  p.position.z = .5;

  geometry_msgs::Pose np;
  np.orientation.w = 1.0;
  np.position.x = .7;
  np.position.y = .7;
  np.position.z = .7;  

  df.addShapeToField(sphere, p);

  bodies::Body* body = bodies::constructBodyFromMsg(sphere, p);
  EigenSTL::vector_Vector3d point_vec = distance_field::determineCollisionPoints(body, resolution);
  delete body;
  check_distance_field(df, point_vec,
                       numX, numY, numZ, true);

  // std::cout << "Shape pos "<< std::endl;
  // print(df, numX, numY, numZ);
  // std::cout << "Shape neg "<< std::endl;
  // printNeg(df, numX, numY, numZ);

  df.addShapeToField(sphere, np);

  body = bodies::constructBodyFromMsg(sphere, np);
  EigenSTL::vector_Vector3d point_vec_2 = distance_field::determineCollisionPoints(body, resolution);
  delete body;
  EigenSTL::vector_Vector3d point_vec_union = point_vec_2;
  point_vec_union.insert(point_vec_union.end(),
                         point_vec.begin(),
                         point_vec.end());
  check_distance_field(df, point_vec_union,
                       numX, numY, numZ, true);

  //should get rid of old pose
  df.moveShapeInField(sphere, p, np);

  check_distance_field(df, point_vec_2,
                       numX, numY, numZ, true);

  //should be equivalent to just adding second shape
  PropagationDistanceField test_df( width, height, depth, resolution, origin_x, origin_y, origin_z, max_dist, true);
  test_df.addShapeToField(sphere, np);
  ASSERT_TRUE(areDistanceFieldsDistancesEqual(df, test_df));
}

static const double PERF_WIDTH = 3.0;
static const double PERF_HEIGHT = 3.0;
static const double PERF_DEPTH = 4.0;
static const double PERF_RESOLUTION = 0.02;
static const double PERF_ORIGIN_X = 0.0;
static const double PERF_ORIGIN_Y = 0.0;
static const double PERF_ORIGIN_Z = 0.0;
static const double PERF_MAX_DIST = .25;
static const unsigned int UNIFORM_DISTANCE = 10;

TEST(TestSignedPropagationDistanceField, TestPerformance)
{
  std::cout << "Creating distance field with " <<
    (PERF_WIDTH/PERF_RESOLUTION)*(PERF_HEIGHT/PERF_RESOLUTION)*(PERF_DEPTH/PERF_RESOLUTION)
            << " entries" << std::endl;

  ros::WallTime dt = ros::WallTime::now();
  PropagationDistanceField df( PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, 
                               PERF_ORIGIN_X, PERF_ORIGIN_Y, PERF_ORIGIN_Z, PERF_MAX_DIST, false);
  std::cout << "Creating unsigned took " << (ros::WallTime::now()-dt).toSec() << std::endl;

  dt = ros::WallTime::now();
  PropagationDistanceField sdf( PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, 
                                PERF_ORIGIN_X, PERF_ORIGIN_Y, PERF_ORIGIN_Z, PERF_MAX_DIST, true);

  std::cout << "Creating signed took " << (ros::WallTime::now()-dt).toSec() << std::endl;

  shape_msgs::SolidPrimitive big_table;
  big_table.type = shape_msgs::SolidPrimitive::BOX;
  big_table.dimensions.resize(3);
  big_table.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.0;
  big_table.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.0;
  big_table.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = .5;
  
  geometry_msgs::Pose p;
  p.orientation.w = 1.0;
  p.position.x = PERF_WIDTH/2.0;
  p.position.y = PERF_DEPTH/2.0;
  p.position.z = PERF_HEIGHT/2.0;

  geometry_msgs::Pose np;
  np.orientation.w = 1.0;
  np.position.x = p.position.x+.01;
  np.position.y = p.position.y;
  np.position.z = p.position.z;

  unsigned int big_num_points = ceil(2.0/PERF_RESOLUTION)*ceil(2.0/PERF_RESOLUTION)*ceil(.5/PERF_RESOLUTION);

  std::cout << "Adding " << big_num_points << " points" << std::endl; 

  dt = ros::WallTime::now();
  df.addShapeToField(big_table, p);
  std::cout << "Adding to unsigned took " 
            << (ros::WallTime::now()-dt).toSec() 
            << " avg " << (ros::WallTime::now()-dt).toSec()/(big_num_points*1.0) 
            << std::endl;  

  dt = ros::WallTime::now();
  df.addShapeToField(big_table, p);
  std::cout << "Re-adding to unsigned took " << (ros::WallTime::now()-dt).toSec() << std::endl;  

  dt = ros::WallTime::now();
  sdf.addShapeToField(big_table, p);
  std::cout << "Adding to signed took " 
            << (ros::WallTime::now()-dt).toSec() 
            << " avg " << (ros::WallTime::now()-dt).toSec()/(big_num_points*1.0) 
            << std::endl;  

  dt = ros::WallTime::now();
  df.moveShapeInField(big_table, p, np);
  std::cout << "Moving in unsigned took " << (ros::WallTime::now()-dt).toSec() << std::endl;  

  dt = ros::WallTime::now();
  sdf.moveShapeInField(big_table, p, np);
  std::cout << "Moving in signed took " << (ros::WallTime::now()-dt).toSec() << std::endl;  
    
  dt = ros::WallTime::now();
  df.removeShapeFromField(big_table, np);
  std::cout << "Removing from unsigned took " << (ros::WallTime::now()-dt).toSec() << std::endl;  
  
  dt = ros::WallTime::now();
  sdf.removeShapeFromField(big_table, np);
  std::cout << "Removing from signed took " << (ros::WallTime::now()-dt).toSec() << std::endl;  

  dt = ros::WallTime::now();
  df.reset();

  shape_msgs::SolidPrimitive small_table;
  small_table.type = shape_msgs::SolidPrimitive::BOX;
  small_table.dimensions.resize(3);
  small_table.dimensions[shape_msgs::SolidPrimitive::BOX_X] = .25;
  small_table.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = .25;
  small_table.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = .05;

  unsigned int small_num_points = (13)*(13)*(3);

  std::cout << "Adding " << small_num_points << " points" << std::endl;
  df.addShapeToField(big_table, p);

  dt = ros::WallTime::now();
  df.addShapeToField(small_table, p);
  std::cout << "Adding to unsigned took " 
            << (ros::WallTime::now()-dt).toSec() 
            << " avg " << (ros::WallTime::now()-dt).toSec()/(small_num_points*1.0) 
            << std::endl;  

  dt = ros::WallTime::now();
  sdf.addShapeToField(small_table, p);
  std::cout << "Adding to signed took " 
            << (ros::WallTime::now()-dt).toSec() 
            << " avg " << (ros::WallTime::now()-dt).toSec()/(small_num_points*1.0) 
            << std::endl;  

  dt = ros::WallTime::now();
  df.moveShapeInField(small_table, p, np);
  std::cout << "Moving in unsigned took " << (ros::WallTime::now()-dt).toSec() << std::endl;  

  dt = ros::WallTime::now();
  sdf.moveShapeInField(small_table, p, np);
  std::cout << "Moving in signed took " << (ros::WallTime::now()-dt).toSec() << std::endl;  

  //uniformly spaced points - a worst case scenario
  PropagationDistanceField worstdfu(PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, 
                                   PERF_ORIGIN_X, PERF_ORIGIN_Y, PERF_ORIGIN_Z, PERF_MAX_DIST, false);

  PropagationDistanceField worstdfs(PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, 
                                   PERF_ORIGIN_X, PERF_ORIGIN_Y, PERF_ORIGIN_Z, PERF_MAX_DIST, true);

  
  
  EigenSTL::vector_Vector3d bad_vec;
  unsigned int count = 0;
  for(unsigned int z = UNIFORM_DISTANCE; z < worstdfu.getZNumCells()-UNIFORM_DISTANCE; z += UNIFORM_DISTANCE) {
    for(unsigned int x = UNIFORM_DISTANCE; x < worstdfu.getXNumCells()-UNIFORM_DISTANCE; x += UNIFORM_DISTANCE) {
      for(unsigned int y = UNIFORM_DISTANCE; y < worstdfu.getYNumCells()-UNIFORM_DISTANCE; y += UNIFORM_DISTANCE) {
        count++;
        Eigen::Vector3d loc;
        bool valid = worstdfu.gridToWorld(x,y,z,
                                          loc.x(), loc.y(), loc.z());
        
        if(!valid) {
          logWarn("Something wrong");
          continue;
        }
        bad_vec.push_back(loc);
      }
    }
  }
  
  dt = ros::WallTime::now();
  worstdfu.addPointsToField(bad_vec);
  ros::WallDuration wd = ros::WallTime::now()-dt; 
  printf("Time for unsigned adding %d uniform points is %g average %g\n", bad_vec.size(), wd.toSec(), wd.toSec()/(bad_vec.size()*1.0));
  dt = ros::WallTime::now();
  worstdfs.addPointsToField(bad_vec);
  wd = ros::WallTime::now()-dt; 
  printf("Time for signed adding %d uniform points is %g average %g\n", bad_vec.size(), wd.toSec(), wd.toSec()/(bad_vec.size()*1.0));

}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
