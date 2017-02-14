/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Ken Anderson
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ken Anderson */

#include <gtest/gtest.h>
#include <fstream>
#include <boost/filesystem/path.hpp>
#include <urdf_parser/urdf_parser.h>
#include <moveit_resources/config.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// Load pr2.  Take a look at test/ in planning_scene, robot_mode,
// and robot_state for inspiration.
moveit::core::RobotModelConstPtr loadModel()
{
  moveit::core::RobotModelConstPtr robot_model;
  urdf::ModelInterfaceSharedPtr urdf_model;
  srdf::ModelSharedPtr srdf_model;
  boost::filesystem::path res_path(MOVEIT_TEST_RESOURCES_DIR);
  std::string xml_string;
  std::fstream xml_file((res_path / "pr2_description/urdf/robot.xml").string().c_str(), std::fstream::in);
  EXPECT_TRUE(xml_file.is_open());
  while (xml_file.good())
  {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  urdf_model = urdf::parseURDF(xml_string);
  srdf_model.reset(new srdf::Model());
  srdf_model->initFile(*urdf_model, (res_path / "pr2_description/srdf/robot.xml").string());
  robot_model.reset(new moveit::core::RobotModel(urdf_model, srdf_model));
  return robot_model;
}

// Initialize one-joint, straight-line trajectory
// Can specify init/final velocity/acceleration,
// but not all time parameterization methods may accept it.
int initTrajectory(robot_trajectory::RobotTrajectory& trajectory, double vel_i = 0.0, double vel_f = 0.0,
                   double acc_i = 0.0, double acc_f = 0.0)
{
  const int num = 10;
  const double max = 2.0;
  unsigned i;

  const robot_model::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    logError("Need to set the group");
    return -1;
  }
  const std::vector<int>& idx = group->getVariableIndexList();
  moveit::core::RobotState state(trajectory.getRobotModel());

  state.setVariableVelocity(idx[0], vel_i);
  state.setVariableAcceleration(idx[0], acc_i);

  for (i = 0; i < num; i++)
  {
    state.setVariablePosition(idx[0], i * max / num);
    trajectory.addSuffixWayPoint(state, 0.0);
  }

  state.setVariablePosition(idx[0], max);
  state.setVariableVelocity(idx[0], vel_f);
  state.setVariableAcceleration(idx[0], acc_f);
  trajectory.addSuffixWayPoint(state, 0.0);

  return 0;
}

void printTrajectory(robot_trajectory::RobotTrajectory& trajectory)
{
  const robot_model::JointModelGroup* group = trajectory.getGroup();
  const std::vector<int>& idx = group->getVariableIndexList();
  unsigned int count = trajectory.getWayPointCount();

  std::cout << "trajectory length is " << trajectory.getWayPointDurationFromStart(count - 1) << " seconds."
            << std::endl;
  std::cout << "  Trajectory Points" << std::endl;
  for (unsigned i = 0; i < count; i++)
  {
    robot_state::RobotStatePtr point = trajectory.getWayPointPtr(i);
    printf("  waypoint %2d time %6.2f pos %6.2f vel %6.2f acc %6.2f ", i, trajectory.getWayPointDurationFromStart(i),
           point->getVariablePosition(idx[0]), point->getVariableVelocity(idx[0]),
           point->getVariableAcceleration(idx[0]));
    if (i > 0)
    {
      robot_state::RobotStatePtr prev = trajectory.getWayPointPtr(i - 1);
      printf("jrk %6.2f",
             (point->getVariableAcceleration(idx[0]) - prev->getVariableAcceleration(idx[0])) /
                 (trajectory.getWayPointDurationFromStart(i) - trajectory.getWayPointDurationFromStart(i - 1)));
    }
    printf("\n");
  }
}

TEST(TestTimeParameterization, TestIterativeParabolic)
{
  moveit::core::RobotModelConstPtr robot_model = loadModel();
  robot_trajectory::RobotTrajectory trajectory(robot_model, "right_arm");
  trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
  EXPECT_EQ(initTrajectory(trajectory), 0);

  ros::WallTime wt = ros::WallTime::now();
  EXPECT_TRUE(time_parameterization.computeTimeStamps(trajectory));
  std::cout << "IterativeParabolicTimeParameterization  took " << (ros::WallTime::now() - wt).toSec() << std::endl;
  printTrajectory(trajectory);
  ASSERT_LT(trajectory.getWayPointDurationFromStart(trajectory.getWayPointCount() - 1), 3.0);
}

TEST(TestTimeParameterization, TestIterativeSpline)
{
  moveit::core::RobotModelConstPtr robot_model = loadModel();
  robot_trajectory::RobotTrajectory trajectory(robot_model, "right_arm");
  trajectory_processing::IterativeSplineParameterization time_parameterization(false);
  EXPECT_EQ(initTrajectory(trajectory), 0);

  ros::WallTime wt = ros::WallTime::now();
  EXPECT_TRUE(time_parameterization.computeTimeStamps(trajectory));
  std::cout << "IterativeSplineParameterization took " << (ros::WallTime::now() - wt).toSec() << std::endl;
  printTrajectory(trajectory);
  ASSERT_LT(trajectory.getWayPointDurationFromStart(trajectory.getWayPointCount() - 1), 5.0);
}

TEST(TestTimeParameterization, TestIterativeSplineJerk)
{
  moveit::core::RobotModelConstPtr robot_model = loadModel();
  robot_trajectory::RobotTrajectory trajectory(robot_model, "right_arm");
  trajectory_processing::IterativeSplineParameterization time_parameterization(true, 9.0, false);
  EXPECT_EQ(initTrajectory(trajectory), 0);

  ros::WallTime wt = ros::WallTime::now();
  EXPECT_TRUE(time_parameterization.computeTimeStamps(trajectory));
  std::cout << "IterativeSplineParameterization with Jerk took " << (ros::WallTime::now() - wt).toSec() << std::endl;
  printTrajectory(trajectory);
  ASSERT_LT(trajectory.getWayPointDurationFromStart(trajectory.getWayPointCount() - 1), 5.0);
}

TEST(TestTimeParameterization, TestIterativeSplineJerkAddPoints)
{
  moveit::core::RobotModelConstPtr robot_model = loadModel();
  robot_trajectory::RobotTrajectory trajectory(robot_model, "right_arm");
  trajectory_processing::IterativeSplineParameterization time_parameterization(true, 9.0, true);
  EXPECT_EQ(initTrajectory(trajectory), 0);

  ros::WallTime wt = ros::WallTime::now();
  EXPECT_TRUE(time_parameterization.computeTimeStamps(trajectory));
  std::cout << "IterativeSplineParameterization with Jerk and added points took " << (ros::WallTime::now() - wt).toSec()
            << std::endl;
  printTrajectory(trajectory);
  ASSERT_LT(trajectory.getWayPointDurationFromStart(trajectory.getWayPointCount() - 1), 5.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
