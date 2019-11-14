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

  Trajectory trajectory(Path(waypoints, 100.0), max_velocities, max_accelerations);
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

TEST(time_optimal_trajectory_generation, testLargeAccel1)
{
  double path_tolerance = 0.1;
  double resample_dt = 0.1;
  Eigen::VectorXd waypoint(6);
  std::list<Eigen::VectorXd> waypoints;
  Eigen::VectorXd max_velocities(6);
  Eigen::VectorXd max_accelerations(6);
  waypoint << 1.6113056281076339, -0.21400163389235427, -1.974502599739185, 9.9653618690354051e-12, -1.3810916877429624,
      1.5293902838041467;
  waypoints.push_back(waypoint);
  waypoint << 1.6088016187976597, -0.21792862470933924, -1.9758628799742952, 0.00010424017303217738,
      -1.3835690515335755, 1.5279972853269816;
  waypoints.push_back(waypoint);
  waypoint << 1.6012895908677376, -0.22970959716029427, -1.9799437206796262, 0.00041696066223262368,
      -1.3910011429054152, 1.5238182898954862;
  waypoints.push_back(waypoint);
  waypoint << 1.5887695443178671, -0.24934455124521923, -1.9867451218551782, 0.00093816147756670078,
      -1.4033879618584812, 1.5168532975096607;
  waypoints.push_back(waypoint);
  waypoint << 1.5712414928341785, -0.27683346550041388, -1.9962670760660852, 0.0016678420492903677, -1.42072949485228,
      1.507102315783198;
  waypoints.push_back(waypoint);
  waypoint << 1.5487054312144797, -0.31217634808437744, -2.0085095861383908, 0.002606002593967227, -1.4430257470336456,
      1.494565341822081;
  waypoints.push_back(waypoint);
  waypoint << 1.5211613618678779, -0.35537319521895278, -2.0234726507633698, 0.0037526430113078413, -1.4702767160191073,
      1.4792423769665133;
  waypoints.push_back(waypoint);
  waypoint << 1.4886092858389071, -0.40642400526601669, -2.0411562693735883, 0.0051077632578289834, -1.5024824007752455,
      1.4611334217975775;
  waypoints.push_back(waypoint);
  waypoint << 1.4510492027456556, -0.46532877882451418, -2.0615604421765168, 0.0066713633494293696, -1.5396428016799086,
      1.4402384761028124;
  waypoints.push_back(waypoint);
  waypoint << 1.4084811091471308, -0.53208752129088954, -2.0846851710414422, 0.0084434434293551314, -1.581757922137472,
      1.4165575379679693;
  waypoints.push_back(waypoint);
  waypoint << 1.3618675689933948, -0.60519066168327829, -2.1100075518756838, 0.01038393263096969, -1.6278754409778298,
      1.390626088675514;
  waypoints.push_back(waypoint);
  waypoint << 1.3186826089032833, -0.67291682429386657, -2.1334673878397981, 0.012181692433178149, -1.6706008637628698,
      1.3666019832529253;
  waypoints.push_back(waypoint);
  waypoint << 1.2805056674331203, -0.73278900527048485, -2.1542066633336918, 0.013770971909252978, -1.708371558966683,
      1.3453638747846666;
  waypoints.push_back(waypoint);
  waypoint << 1.2473367445829053, -0.78480720461313291, -2.1722253783573651, 0.015151771059194168, -1.74118752658927,
      1.3269117632707386;
  waypoints.push_back(waypoint);
  waypoint << 1.2191758403526387, -0.82897142232181109, -2.1875235329108174, 0.016324089883001731, -1.7690487666306307,
      1.3112456487111404;
  waypoints.push_back(waypoint);
  waypoint << 1.1960229547423202, -0.8652816583965186, -2.2001011269940487, 0.017287928380675648, -1.7919552790907645,
      1.2983655311058726;
  waypoints.push_back(waypoint);
  waypoint << 1.1778780877519499, -0.89373791283725623, -2.20995816060706, 0.018043286552215931, -1.8099070639696717,
      1.2882714104549351;
  waypoints.push_back(waypoint);
  waypoint << 1.1647412393815282, -0.91434018564402375, -2.2170946337498498, 0.018590164397622583, -1.8229041212673529,
      1.2809632867583278;
  waypoints.push_back(waypoint);
  waypoint << 1.1503050107100292, -0.92787087067084606, -2.2224453508970914, 0.018985805178625942, -1.8308113278134459,
      1.275588347547963;
  waypoints.push_back(waypoint);
  waypoint << 1.1303601517180486, -0.93432437197705887, -2.2264898089115603, 0.019256515051230879, -1.8331691671989989,
      1.2717313378762904;
  waypoints.push_back(waypoint);
  waypoint << 1.1072694611893932, -0.93473125970315851, -2.2292398757395047, 0.019410686141393005, -1.830954651103494,
      1.2693257854740989;
  waypoints.push_back(waypoint);
  waypoint << 1.0763251569513335, -0.93473170757635804, -2.2327762721150433, 0.019605013240311595, -1.8276056211213929,
      1.2662609130255182;
  waypoints.push_back(waypoint);
  waypoint << 1.0371135177484976, -0.93473227510704704, -2.2372574812477164, 0.019851258366119191, -1.8233618367296884,
      1.2623772040323571;
  waypoints.push_back(waypoint);
  waypoint << 0.98963453251569511, -0.93473296229538594, -2.2426835044020845, 0.020149421588304073, -1.8182232967308212,
      1.2576746573986666;
  waypoints.push_back(waypoint);
  waypoint << 0.94195042420873332, -0.934733652452579, -2.2481329695602028, 0.020448872962635339, -1.8130625567393446,
      1.25295179439096;
  waypoints.push_back(waypoint);
  waypoint << 0.90253237635866967, -0.93473422297072895, -2.2526377676152936, 0.020696414313858082, -1.8087964332204516,
      1.2490476416944549;
  waypoints.push_back(waypoint);
  waypoint << 0.87135462553092879, -0.93471424246198187, -2.2561969708645573, 0.020891878882680692, -1.8054055932708675,
      1.2459668617670974;
  waypoints.push_back(waypoint);
  waypoint << 0.8444022545538824, -0.93022143041122873, -2.2584256534645961, 0.020988884519988333, -1.7988502374200697,
      1.2448838614981286;
  waypoints.push_back(waypoint);
  waypoint << 0.82099448708463141, -0.91787651352488608, -2.2587612095653986, 0.020937366179824963, -1.7863203175133031,
      1.2469243949891951;
  waypoints.push_back(waypoint);
  waypoint << 0.79735289822932687, -0.89808454111937552, -2.2577121978788699, 0.02076756446709704, -1.7677348472002912,
      1.251571230499571;
  waypoints.push_back(waypoint);
  waypoint << 0.76547555984944093, -0.87135965673738525, -2.2562905129862432, 0.020537996456364734, -1.742644029592993,
      1.2578503220653712;
  waypoints.push_back(waypoint);
  waypoint << 0.7253308805235652, -0.83770371279530698, -2.2545001162022706, 0.020248890318816479, -1.7110459452465103,
      1.2657578868175752;
  waypoints.push_back(waypoint);
  waypoint << 0.67691886295385151, -0.79711671155853359, -2.2523410076474653, 0.019900246073912102, -1.6729405962877202,
      1.2752939242239221;
  waypoints.push_back(waypoint);
  waypoint << 0.62023951300467106, -0.74959865794355596, -2.2498131875833693, 0.019492063763884494, -1.6283279873324996,
      1.2864584331292677;
  waypoints.push_back(waypoint);
  waypoint << 0.56208427285968199, -0.70084326781153738, -2.247219544872177, 0.019073252675121555, -1.5825536975847947,
      1.2979136579523005;
  waypoints.push_back(waypoint);
  waypoint << 0.51212388107119122, -0.65895816189978329, -2.2449913809838069, 0.018713457650153989, -1.5432296152848393,
      1.3077546889369054;
  waypoints.push_back(waypoint);
  waypoint << 0.47043083837108213, -0.62400412237442371, -2.2431319293499219, 0.018413200810640674, -1.5104128061329873,
      1.3159672451312452;
  waypoints.push_back(waypoint);
  waypoint << 0.43700514475935415, -0.5959811492354582, -2.2416411899705215, 0.018172482156581595, -1.4841032701292385,
      1.3225513265353206;
  waypoints.push_back(waypoint);
  waypoint << 0.41182965468283472, -0.57012383897312591, -2.2395418799140034, 0.017939019520436833, -1.4605861407222787,
      1.3292885920320254;
  waypoints.push_back(waypoint);
  waypoint << 0.39330307870537773, -0.53849278924458122, -2.2354066613232368, 0.017628857263236073, -1.4334615785982261,
      1.338963651388752;
  waypoints.push_back(waypoint);
  waypoint << 0.37205512823505804, -0.49914157023561717, -2.230032220374413, 0.017239388956133892, -1.3999579261831692,
      1.3512104159375973;
  waypoints.push_back(waypoint);
  waypoint << 0.34656637744045482, -0.45193638608459069, -2.2235851145137211, 0.016772188101350737, -1.3597674029461513,
      1.3659014672021286;
  waypoints.push_back(waypoint);
  waypoint << 0.31683682304168193, -0.39687723071714981, -2.2160653429115489, 0.016227254638767317, -1.3128900037154652,
      1.3830368070727876;
  waypoints.push_back(waypoint);
  waypoint << 0.28286646669263649, -0.33396410719631353, -2.2074729059862328, 0.015604588598699055, -1.2593257310989681,
      1.4026164345963106;
  waypoints.push_back(waypoint);
  waypoint << 0.24465530781498634, -0.26319701445101024, -2.1978078035914899, 0.014904189970545295, -1.1990745841847479,
      1.4246403501060334;
  waypoints.push_back(waypoint);
  waypoint << 0.20287107552306982, -0.18581258726414238, -2.1872389306379052, 0.014138298020776825, -1.1331894345984921,
      1.4487236919534348;
  waypoints.push_back(waypoint);
  waypoint << 0.16378465725216679, -0.11342451351544032, -2.1773524407766152, 0.013421856160846825, -1.0715581834466648,
      1.4712520841524879;
  waypoints.push_back(waypoint);
  waypoint << 0.12893904821046057, -0.04889042140070643, -2.1685386180795945, 0.01278314701182208, -1.0166138171580172,
      1.4913361845032955;
  waypoints.push_back(waypoint);
  waypoint << 0.098334248397949936, 0.0077896890800614235, -2.160797462546844, 0.012222170573702569,
      -0.9683563357325472, 1.5089759930058571;
  waypoints.push_back(waypoint);
  waypoint << 0.071970257814636035, 0.05661581792686108, -2.1541289741783634, 0.011738926846488317,
      -0.92678573917025719, 1.5241715096601727;
  waypoints.push_back(waypoint);
  waypoint << 0.049847076460519099, 0.097587965139692132, -2.1485331529741529, 0.011333415830179324,
      -0.89190202747114711, 1.5369227344662422;
  waypoints.push_back(waypoint);
  waypoint << 0.031964704335599101, 0.13070613071855469, -2.1440099989342123, 0.011005637524775591,
      -0.86370520063521716, 1.5472296674240651;
  waypoints.push_back(waypoint);
  waypoint << 0.018323141439876386, 0.15597031466344807, -2.1405595120585419, 0.010755591930277125,
      -0.84219525866246769, 1.5550923085336414;
  waypoints.push_back(waypoint);
  waypoint << 0.0089223877733517337, 0.17338051697437079, -2.1381816923471417, 0.010583279046683939,
      -0.82737220155290014, 1.5605106577949708;
  waypoints.push_back(waypoint);
  waypoint << 0.0037624433360251394, 0.1829367376513229, -2.1368765398000118, 0.010488698873996033,
      -0.81923602930651418, 1.5634847152080531;
  waypoints.push_back(waypoint);
  waypoint << 0.0026731698230432633, 0.18495407295290356, -2.1366010197595817, 0.010468732830133831,
      -0.81751846877215351, 1.5641125440549537;
  waypoints.push_back(waypoint);
  max_velocities << 0.89535390627300004, 0.89535390627300004, 0.79587013890930003, 0.92022484811399996,
      0.82074108075029995, 1.3927727430915;
  max_accelerations << 0.82673490883799994, 0.78539816339699997, 0.60883578557700002, 3.2074759432319997,
      1.4398966328939999, 4.7292792634680003;

  Trajectory parameterized(Path(waypoints, path_tolerance), max_velocities, max_accelerations, 0.001);
  ASSERT_TRUE(parameterized.isValid());
  size_t sample_count = std::ceil(parameterized.getDuration() / resample_dt);
  for (size_t sample = 0; sample <= sample_count; ++sample)
  {
    // always sample the end of the trajectory as well
    double t = std::min(parameterized.getDuration(), sample * resample_dt);
    Eigen::VectorXd acceleration = parameterized.getAcceleration(t);
    ASSERT_EQ(acceleration.size(), 6);
    for (std::size_t i = 0; i < 6; ++i)
      EXPECT_NEAR(acceleration(i), 0.0, 100.0) << "Invalid acceleration at position " << sample_count;
  }
}

TEST(time_optimal_trajectory_generation, testLargeAccel2)
{
  double path_tolerance = 0.1;
  double resample_dt = 0.1;
  Eigen::VectorXd waypoint(6);
  std::list<Eigen::VectorXd> waypoints;
  Eigen::VectorXd max_velocities(6);
  Eigen::VectorXd max_accelerations(6);

  // Waypoints
  waypoint << 1.6113056281076339, -0.21400163389235427, -1.974502599739185, 9.9653618690354051e-12, -1.3810916877429624,
      1.5293902838041467;
  waypoints.push_back(waypoint);
  waypoint << 1.6088016187976597, -0.21792862470933924, -1.9758628799742952, 0.00010424017303217738,
      -1.3835690515335755, 1.5279972853269816;
  waypoints.push_back(waypoint);
  waypoint << 1.5887695443178671, -0.24934455124521923, -1.9867451218551782, 0.00093816147756670078,
      -1.4033879618584812, 1.5168532975096607;
  waypoints.push_back(waypoint);
  waypoint << 1.1647412393815282, -0.91434018564402375, -2.2170946337498498, 0.018590164397622583, -1.8229041212673529,
      1.2809632867583278;
  waypoints.push_back(waypoint);
  // Max velocities
  max_velocities << 0.89535390627300004, 0.89535390627300004, 0.79587013890930003, 0.92022484811399996,
      0.82074108075029995, 1.3927727430915;
  // Max accelerations
  max_accelerations << 0.82673490883799994, 0.78539816339699997, 0.60883578557700002, 3.2074759432319997,
      1.4398966328939999, 4.7292792634680003;

  Trajectory parameterized(Path(waypoints, path_tolerance), max_velocities, max_accelerations, 0.001);

  ASSERT_TRUE(parameterized.isValid());

  size_t sample_count = std::ceil(parameterized.getDuration() / resample_dt);
  for (size_t sample = 0; sample <= sample_count; ++sample)
  {
    // always sample the end of the trajectory as well
    double t = std::min(parameterized.getDuration(), sample * resample_dt);
    Eigen::VectorXd acceleration = parameterized.getAcceleration(t);

    ASSERT_EQ(acceleration.size(), 6);
    for (std::size_t i = 0; i < 6; ++i)
      EXPECT_NEAR(acceleration(i), 0.0, 100.0) << "Invalid acceleration at position " << sample_count << "\n";
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
