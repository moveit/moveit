/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

using trajectory_processing::Path;
using trajectory_processing::Trajectory;

TEST(time_optimal_trajectory_generation, test1)
{
  Eigen::VectorXd waypoint(4);
  std::list<Eigen::VectorXd> waypoints;

  waypoint << 1424.0, 984.999694824219, 2126.0, 0.0;
  waypoints.push_back(waypoint);
  waypoint << 1423.0, 985.000244140625, 2126.0, 0.0;
  waypoints.push_back(waypoint);

  Eigen::VectorXd max_velocities(4);
  max_velocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd max_accelerations(4);
  max_accelerations << 0.00249, 0.00249, 0.00249, 0.00249;

  Trajectory trajectory(Path(waypoints, 100.0), max_velocities, max_accelerations, 10.0);
  EXPECT_TRUE(trajectory.isValid());
  EXPECT_DOUBLE_EQ(40.080256821829849, trajectory.getDuration());

  // Test start matches
  EXPECT_DOUBLE_EQ(1424.0, trajectory.getPosition(0.0)[0]);
  EXPECT_DOUBLE_EQ(984.999694824219, trajectory.getPosition(0.0)[1]);
  EXPECT_DOUBLE_EQ(2126.0, trajectory.getPosition(0.0)[2]);
  EXPECT_DOUBLE_EQ(0.0, trajectory.getPosition(0.0)[3]);

  // Test end matches
  EXPECT_DOUBLE_EQ(1423.0, trajectory.getPosition(trajectory.getDuration())[0]);
  EXPECT_DOUBLE_EQ(985.000244140625, trajectory.getPosition(trajectory.getDuration())[1]);
  EXPECT_DOUBLE_EQ(2126.0, trajectory.getPosition(trajectory.getDuration())[2]);
  EXPECT_DOUBLE_EQ(0.0, trajectory.getPosition(trajectory.getDuration())[3]);
}

TEST(time_optimal_trajectory_generation, test2)
{
  Eigen::VectorXd waypoint(4);
  std::list<Eigen::VectorXd> waypoints;

  waypoint << 1427.0, 368.0, 690.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 1427.0, 368.0, 790.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 952.499938964844, 433.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 951.0, 90.0;
  waypoints.push_back(waypoint);

  Eigen::VectorXd max_velocities(4);
  max_velocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd max_accelerations(4);
  max_accelerations << 0.002, 0.002, 0.002, 0.002;

  Trajectory trajectory(Path(waypoints, 100.0), max_velocities, max_accelerations, 10.0);
  EXPECT_TRUE(trajectory.isValid());
  EXPECT_DOUBLE_EQ(1922.1418427445944, trajectory.getDuration());

  // Test start matches
  EXPECT_DOUBLE_EQ(1427.0, trajectory.getPosition(0.0)[0]);
  EXPECT_DOUBLE_EQ(368.0, trajectory.getPosition(0.0)[1]);
  EXPECT_DOUBLE_EQ(690.0, trajectory.getPosition(0.0)[2]);
  EXPECT_DOUBLE_EQ(90.0, trajectory.getPosition(0.0)[3]);

  // Test end matches
  EXPECT_DOUBLE_EQ(452.5, trajectory.getPosition(trajectory.getDuration())[0]);
  EXPECT_DOUBLE_EQ(533.0, trajectory.getPosition(trajectory.getDuration())[1]);
  EXPECT_DOUBLE_EQ(951.0, trajectory.getPosition(trajectory.getDuration())[2]);
  EXPECT_DOUBLE_EQ(90.0, trajectory.getPosition(trajectory.getDuration())[3]);
}

TEST(time_optimal_trajectory_generation, test3)
{
  Eigen::VectorXd waypoint(4);
  std::list<Eigen::VectorXd> waypoints;

  waypoint << 1427.0, 368.0, 690.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 1427.0, 368.0, 790.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 952.499938964844, 433.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 951.0, 90.0;
  waypoints.push_back(waypoint);

  Eigen::VectorXd max_velocities(4);
  max_velocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd max_accelerations(4);
  max_accelerations << 0.002, 0.002, 0.002, 0.002;

  Trajectory trajectory(Path(waypoints, 100.0), max_velocities, max_accelerations, 10.0);
  EXPECT_TRUE(trajectory.isValid());
  EXPECT_DOUBLE_EQ(1919.5597888812974, trajectory.getDuration());

  // Test start matches
  EXPECT_DOUBLE_EQ(1427.0, trajectory.getPosition(0.0)[0]);
  EXPECT_DOUBLE_EQ(368.0, trajectory.getPosition(0.0)[1]);
  EXPECT_DOUBLE_EQ(690.0, trajectory.getPosition(0.0)[2]);
  EXPECT_DOUBLE_EQ(90.0, trajectory.getPosition(0.0)[3]);

  // Test end matches
  EXPECT_DOUBLE_EQ(452.5, trajectory.getPosition(trajectory.getDuration())[0]);
  EXPECT_DOUBLE_EQ(533.0, trajectory.getPosition(trajectory.getDuration())[1]);
  EXPECT_DOUBLE_EQ(951.0, trajectory.getPosition(trajectory.getDuration())[2]);
  EXPECT_DOUBLE_EQ(90.0, trajectory.getPosition(trajectory.getDuration())[3]);
}

TEST(time_optimal_trajectory_generation, testLargeAccel)
{
  double path_tolerance = 0.1;
  double resample_dt = 0.1;
  Eigen::VectorXd waypoint(6);
  std::list<Eigen::VectorXd> waypoints;
  Eigen::VectorXd max_velocities(6);
  Eigen::VectorXd max_accelerations(6);
waypoint << 1.96936,-0.370082,-1.76986,9.93072e-12,-1.74181,1.17134;
  waypoints.push_back(waypoint);
waypoint << 1.90325,-0.355841,-1.75897,1.94734e-05,-1.73865,1.19883;
  waypoints.push_back(waypoint);
waypoint << 1.83714,-0.3416,-1.74807,3.89467e-05,-1.73549,1.22632;
  waypoints.push_back(waypoint);
waypoint << 1.77103,-0.327359,-1.73718,5.84201e-05,-1.73233,1.25382;
  waypoints.push_back(waypoint);
waypoint << 1.70491,-0.313117,-1.72628,7.78935e-05,-1.72917,1.28131;
  waypoints.push_back(waypoint);
waypoint << 1.6388,-0.298876,-1.71539,9.73668e-05,-1.726,1.3088;
  waypoints.push_back(waypoint);
waypoint << 1.57269,-0.284635,-1.70449,0.00011684,-1.72284,1.3363;
  waypoints.push_back(waypoint);
waypoint << 1.50657,-0.270394,-1.6936,0.000136314,-1.71968,1.36379;
  waypoints.push_back(waypoint);
waypoint << 1.44046,-0.256152,-1.6827,0.000155787,-1.71652,1.39128;
  waypoints.push_back(waypoint);
waypoint << 1.37435,-0.241911,-1.67181,0.00017526,-1.71335,1.41878;
  waypoints.push_back(waypoint);
waypoint << 1.30824,-0.22767,-1.66091,0.000194734,-1.71019,1.44627;
  waypoints.push_back(waypoint);
waypoint << 1.24212,-0.213429,-1.65002,0.000214207,-1.70703,1.47377;
  waypoints.push_back(waypoint);
waypoint << 1.17601,-0.199188,-1.63913,0.00023368,-1.70387,1.50126;
  waypoints.push_back(waypoint);
waypoint << 1.1099,-0.184946,-1.62823,0.000253154,-1.70071,1.52875;
  waypoints.push_back(waypoint);
waypoint << 1.04378,-0.170705,-1.61734,0.000272627,-1.69754,1.55625;
  waypoints.push_back(waypoint);
waypoint << 0.97767,-0.156464,-1.60644,0.0002921,-1.69438,1.58374;
  waypoints.push_back(waypoint);
waypoint << 0.911557,-0.142223,-1.59555,0.000311574,-1.69122,1.61123;
  waypoints.push_back(waypoint);
waypoint << 0.845444,-0.127982,-1.58465,0.000331047,-1.68806,1.63873;
  waypoints.push_back(waypoint);
waypoint << 0.779331,-0.11374,-1.57376,0.000350521,-1.68489,1.66622;
  waypoints.push_back(waypoint);
waypoint << 0.713218,-0.0994991,-1.56286,0.000369994,-1.68173,1.69372;
  waypoints.push_back(waypoint);
waypoint << 0.647105,-0.0852579,-1.55197,0.000389467,-1.67857,1.72121;
  waypoints.push_back(waypoint);
waypoint << 0.613821,-0.072352,-1.57985,0.000440513,-1.63731,1.71628;
  waypoints.push_back(waypoint);
waypoint << 0.580536,-0.0594461,-1.60773,0.000491559,-1.59604,1.71134;
  waypoints.push_back(waypoint);
waypoint << 0.547251,-0.0465402,-1.63561,0.000542605,-1.55478,1.70641;
  waypoints.push_back(waypoint);
waypoint << 0.513967,-0.0336343,-1.66348,0.000593651,-1.51351,1.70148;
  waypoints.push_back(waypoint);
waypoint << 0.480682,-0.0207284,-1.69136,0.000644697,-1.47225,1.69655;
  waypoints.push_back(waypoint);
waypoint << 0.447398,-0.00782247,-1.71924,0.000695743,-1.43099,1.69162;
  waypoints.push_back(waypoint);
waypoint << 0.414113,0.00508344,-1.74712,0.000746789,-1.38972,1.68668;
  waypoints.push_back(waypoint);
waypoint << 0.380828,0.0179893,-1.775,0.000797835,-1.34846,1.68175;
  waypoints.push_back(waypoint);
waypoint << 0.347544,0.0308953,-1.80288,0.000848881,-1.30719,1.67682;
  waypoints.push_back(waypoint);
waypoint << 0.314259,0.0438012,-1.83076,0.000899927,-1.26593,1.67189;
  waypoints.push_back(waypoint);
waypoint << 0.280974,0.0567071,-1.85864,0.000950973,-1.22467,1.66696;
  waypoints.push_back(waypoint);
waypoint << 0.24769,0.069613,-1.88651,0.00100202,-1.1834,1.66202;
  waypoints.push_back(waypoint);
waypoint << 0.214405,0.0825189,-1.91439,0.00105307,-1.14214,1.65709;
  waypoints.push_back(waypoint);
waypoint << 0.18112,0.0954248,-1.94227,0.00110411,-1.10088,1.65216;
  waypoints.push_back(waypoint);
waypoint << 0.147836,0.108331,-1.97015,0.00115516,-1.05961,1.64723;
  waypoints.push_back(waypoint);
waypoint << 0.114551,0.121237,-1.99803,0.0012062,-1.01835,1.6423;
  waypoints.push_back(waypoint);
waypoint << 0.0812662,0.134142,-2.02591,0.00125725,-0.977084,1.63736;
  waypoints.push_back(waypoint);
waypoint << 0.0479816,0.147048,-2.05379,0.0013083,-0.93582,1.63243;
  waypoints.push_back(waypoint);
waypoint << 0.0146969,0.159954,-2.08167,0.00135934,-0.894556,1.6275;
  waypoints.push_back(waypoint);
waypoint << -0.0185878,0.17286,-2.10954,0.00141039,-0.853292,1.62257;
  waypoints.push_back(waypoint);
waypoint << -0.0518724,0.185766,-2.13742,0.00146143,-0.812028,1.61764;
  waypoints.push_back(waypoint);
max_velocities << 1.19381,1.19381,1.06116,1.22697,1.09432,1.85703;
  waypoints.push_back(waypoint);
max_accelerations << 1.10231,1.0472,0.811781,4.27663,1.91986,6.30571;
  waypoints.push_back(waypoint);

  Trajectory parameterized(Path(waypoints, path_tolerance), max_velocities, max_accelerations, 0.001);
  ASSERT_TRUE(parameterized.isValid());
  size_t sample_count = std::ceil(parameterized.getDuration() / resample_dt);
  for (size_t sample = 0; sample <= sample_count; ++sample)
  {
    // always sample the end of the trajectory as well
    double t = std::min(parameterized.getDuration(), sample * resample_dt);
    Eigen::VectorXd acceleration = parameterized.getAcceleration(t);
    for(std::size_t i = 0; i < 6; ++i)
      EXPECT_NEAR(acceleration(i),0.0,100.0);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
