/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Southwest Research Institute
 *                2018, Bielefeld University
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

/* Author: Jorge Nicho, Robert Haschke */

#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

const double IK_NEAR = 1e-4;
const double IK_NEAR_TRANSLATE = 1e-5;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const double DEFAULT_SEARCH_DISCRETIZATION = 0.01f;
const double EXPECTED_SUCCESS_RATE = 0.8;

class KinematicsTest : public testing::Test
{
  typedef pluginlib::ClassLoader<kinematics::KinematicsBase> KinematicsLoader;

protected:
  template <typename T>
  inline bool getParam(const std::string& param, T& val) const
  {
    // first look within private namespace
    ros::NodeHandle pnh("~");
    if (pnh.getParam(param, val))
      return true;

    // then in local namespace
    ros::NodeHandle nh;
    if (nh.getParam(param, val))
      return true;

    return false;
  }

  void SetUp() override
  {
    // load robot model
    rdf_loader::RDFLoader rdf_loader(ROBOT_DESCRIPTION_PARAM);
    robot_model_.reset(new robot_model::RobotModel(rdf_loader.getURDF(), rdf_loader.getSRDF()));
    ASSERT_TRUE(bool(robot_model_)) << "Failed to load robot model from " << ROBOT_DESCRIPTION_PARAM;

    // load parameters
    std::string plugin_name;
    ASSERT_TRUE(getParam("ik_plugin_name", plugin_name));
    ASSERT_TRUE(getParam("group", group_name_));
    ASSERT_TRUE(getParam("tip_link", tip_link_));
    ASSERT_TRUE(getParam("root_link", root_link_));
    ASSERT_TRUE(getParam("joint_names", joints_));
    ASSERT_TRUE(getParam("num_fk_tests", num_fk_tests_));
    ASSERT_TRUE(getParam("num_ik_cb_tests", num_ik_cb_tests_));
    ASSERT_TRUE(getParam("num_ik_tests", num_ik_tests_));
    ASSERT_TRUE(getParam("num_ik_multiple_tests", num_ik_multiple_tests_));
    ASSERT_TRUE(getParam("num_nearest_ik_tests", num_nearest_ik_tests_));

    // loading plugin
    kinematics_loader_.reset(new KinematicsLoader("moveit_core", "kinematics::KinematicsBase"));
    ASSERT_TRUE(bool(kinematics_loader_)) << "Failed to initialize ClassLoader";

    ROS_INFO_STREAM("Loading " << plugin_name);
    kinematics_solver_ = kinematics_loader_->createUniqueInstance(plugin_name);
    ASSERT_TRUE(bool(kinematics_loader_)) << "Failed to load plugin: " << plugin_name;

    // initializing plugin
    ASSERT_TRUE(kinematics_solver_->initialize(*robot_model_, group_name_, root_link_, { tip_link_ },
                                               DEFAULT_SEARCH_DISCRETIZATION) ||
                kinematics_solver_->initialize(ROBOT_DESCRIPTION_PARAM, group_name_, root_link_, { tip_link_ },
                                               DEFAULT_SEARCH_DISCRETIZATION))
        << "Solver failed to initialize";

    jmg_ = robot_model_->getJointModelGroup(kinematics_solver_->getGroupName());
    ASSERT_TRUE(jmg_);

    // Validate chain information
    ASSERT_EQ(root_link_, kinematics_solver_->getBaseFrame());
    ASSERT_EQ(tip_link_, kinematics_solver_->getTipFrame());
    ASSERT_EQ(joints_, kinematics_solver_->getJointNames());
  }

public:
  void searchIKCallback(const geometry_msgs::Pose& ik_pose, const std::vector<double>& joint_state,
                        moveit_msgs::MoveItErrorCodes& error_code)
  {
    std::vector<std::string> link_names = { tip_link_ };
    std::vector<geometry_msgs::Pose> solutions;
    if (!kinematics_solver_->getPositionFK(link_names, joint_state, solutions))
    {
      error_code.val = error_code.PLANNING_FAILED;
      return;
    }

    EXPECT_GT(solutions[0].position.z, 0.0f);
    if (solutions[0].position.z > 0.0)
      error_code.val = error_code.SUCCESS;
    else
      error_code.val = error_code.PLANNING_FAILED;
  }

public:
  robot_model::RobotModelPtr robot_model_;
  robot_model::JointModelGroup* jmg_;
  std::unique_ptr<KinematicsLoader> kinematics_loader_;
  kinematics::KinematicsBasePtr kinematics_solver_;
  std::string root_link_;
  std::string tip_link_;
  std::string group_name_;
  std::vector<std::string> joints_;
  int num_fk_tests_;
  int num_ik_cb_tests_;
  int num_ik_tests_;
  int num_ik_multiple_tests_;
  int num_nearest_ik_tests_;
};

TEST_F(KinematicsTest, getFK)
{
  std::vector<double> seed, fk_values, solution;
  seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
  fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
  solution.resize(kinematics_solver_->getJointNames().size(), 0.0);

  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  ASSERT_FALSE(fk_names.empty());
  robot_state::RobotState robot_state(robot_model_);

  int success = 0;
  ros::WallTime start_time = ros::WallTime::now();
  for (unsigned int i = 0; i < num_fk_tests_; ++i)
  {
    robot_state.setToRandomPositions(jmg_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    bool succeeded = kinematics_solver_->getPositionFK(fk_names, fk_values, poses);
    if (succeeded && (poses.size() == fk_names.size()))
      success++;
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / num_fk_tests_);
  EXPECT_GE(success, num_fk_tests_);
  ROS_INFO_STREAM("Elapsed time: " << (ros::WallTime::now() - start_time).toSec());
}

TEST_F(KinematicsTest, searchIK)
{
  // Test inverse kinematics
  std::vector<double> seed, fk_values, solution;
  double timeout = 5.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(kinematics_solver_->getJointNames().size(), 0.0);
  std::vector<std::string> fk_names;
  fk_names.push_back(kinematics_solver_->getTipFrame());
  robot_state::RobotState kinematic_state(robot_model_);

  unsigned int success = 0;
  ros::WallTime start_time = ros::WallTime::now();
  for (unsigned int i = 0; i < num_ik_tests_; ++i)
  {
    seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    kinematic_state.setToRandomPositions(jmg_);
    kinematic_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);

    bool result_fk = kinematics_solver_->getPositionFK(fk_names, fk_values, poses);
    ASSERT_TRUE(result_fk);
    solution.clear();
    kinematics_solver_->searchPositionIK(poses[0], seed, timeout, solution, error_code);
    bool result = error_code.val == error_code.SUCCESS;

    ROS_DEBUG("Pose: %f %f %f", poses[0].position.x, poses[0].position.y, poses[0].position.z);
    ROS_DEBUG("Orient: %f %f %f %f", poses[0].orientation.x, poses[0].orientation.y, poses[0].orientation.z,
              poses[0].orientation.w);

    if (result)
    {
      EXPECT_TRUE(kinematics_solver_->getPositionIK(poses[0], solution, solution, error_code));
      result = error_code.val == error_code.SUCCESS;
      success++;
    }
    else
    {
      ROS_ERROR_STREAM("searchPositionIK failed on test " << i + 1);
      continue;
    }

    std::vector<geometry_msgs::Pose> new_poses;
    new_poses.resize(1);
    result_fk = kinematics_solver_->getPositionFK(fk_names, solution, new_poses);
    EXPECT_NEAR(poses[0].position.x, new_poses[0].position.x, IK_NEAR);
    EXPECT_NEAR(poses[0].position.y, new_poses[0].position.y, IK_NEAR);
    EXPECT_NEAR(poses[0].position.z, new_poses[0].position.z, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.x, new_poses[0].orientation.x, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.y, new_poses[0].orientation.y, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.z, new_poses[0].orientation.z, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.w, new_poses[0].orientation.w, IK_NEAR);
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / num_ik_tests_);
  EXPECT_GT(success, EXPECTED_SUCCESS_RATE * num_ik_tests_);
  ROS_INFO_STREAM("Elapsed time: " << (ros::WallTime::now() - start_time).toSec());
}

TEST_F(KinematicsTest, searchIKWithCallback)
{
  // Test inverse kinematics
  std::vector<double> seed, fk_values, solution;
  double timeout = 5.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(kinematics_solver_->getJointNames().size(), 0.0);
  std::vector<std::string> fk_names;
  fk_names.push_back(kinematics_solver_->getTipFrame());
  robot_state::RobotState kinematic_state(robot_model_);

  unsigned int success = 0;
  ros::WallTime start_time = ros::WallTime::now();
  for (unsigned int i = 0; i < num_ik_cb_tests_; ++i)
  {
    seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    kinematic_state.setToRandomPositions(jmg_);
    kinematic_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);

    bool result_fk = kinematics_solver_->getPositionFK(fk_names, fk_values, poses);
    ASSERT_TRUE(result_fk);

    // check height
    if (poses[0].position.z <= 0.0f)
    {
      continue;
    }

    solution.clear();
    kinematics_solver_->searchPositionIK(poses[0], fk_values, timeout, solution,
                                         boost::bind(&KinematicsTest::searchIKCallback, this, _1, _2, _3), error_code);
    bool result = (error_code.val == error_code.SUCCESS);

    if (result)
    {
      EXPECT_TRUE(kinematics_solver_->getPositionIK(poses[0], solution, solution, error_code));
      success++;
    }
    else
    {
      ROS_ERROR_STREAM("searchPositionIK failed on test " << i + 1);
      continue;
    }

    std::vector<geometry_msgs::Pose> new_poses;
    new_poses.resize(1);
    result_fk = kinematics_solver_->getPositionFK(fk_names, solution, new_poses);
    EXPECT_NEAR(poses[0].position.x, new_poses[0].position.x, IK_NEAR);
    EXPECT_NEAR(poses[0].position.y, new_poses[0].position.y, IK_NEAR);
    EXPECT_NEAR(poses[0].position.z, new_poses[0].position.z, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.x, new_poses[0].orientation.x, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.y, new_poses[0].orientation.y, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.z, new_poses[0].orientation.z, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.w, new_poses[0].orientation.w, IK_NEAR);
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / num_ik_cb_tests_);
  EXPECT_GT(success, EXPECTED_SUCCESS_RATE * num_ik_cb_tests_);
  ROS_INFO_STREAM("Elapsed time: " << (ros::WallTime::now() - start_time).toSec());
}

TEST_F(KinematicsTest, getIK)
{
  // Test inverse kinematics
  std::vector<double> seed, fk_values, solution;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(kinematics_solver_->getJointNames().size(), 0.0);
  std::vector<std::string> fk_names;
  fk_names.push_back(kinematics_solver_->getTipFrame());
  robot_state::RobotState kinematic_state(robot_model_);

  unsigned int success = 0;
  ros::WallTime start_time = ros::WallTime::now();
  for (unsigned int i = 0; i < num_ik_tests_; ++i)
  {
    seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    kinematic_state.setToRandomPositions(jmg_);
    kinematic_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);

    bool result_fk = kinematics_solver_->getPositionFK(fk_names, fk_values, poses);
    ASSERT_TRUE(result_fk);
    solution.clear();

    kinematics_solver_->getPositionIK(poses[0], fk_values, solution, error_code);
    if (error_code.val == error_code.SUCCESS)
    {
      success++;
    }
    else
    {
      ROS_ERROR_STREAM("getPositionIK failed on test " << i + 1 << " for group " << kinematics_solver_->getGroupName());
      continue;
    }

    std::vector<geometry_msgs::Pose> new_poses;
    new_poses.resize(1);
    result_fk = kinematics_solver_->getPositionFK(fk_names, solution, new_poses);
    EXPECT_NEAR(poses[0].position.x, new_poses[0].position.x, IK_NEAR);
    EXPECT_NEAR(poses[0].position.y, new_poses[0].position.y, IK_NEAR);
    EXPECT_NEAR(poses[0].position.z, new_poses[0].position.z, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.x, new_poses[0].orientation.x, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.y, new_poses[0].orientation.y, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.z, new_poses[0].orientation.z, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.w, new_poses[0].orientation.w, IK_NEAR);
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / num_ik_tests_);
  EXPECT_GT(success, EXPECTED_SUCCESS_RATE * num_ik_tests_);
  ROS_INFO_STREAM("Elapsed time: " << (ros::WallTime::now() - start_time).toSec());
}

TEST_F(KinematicsTest, getIKMultipleSolutions)
{
  // Test inverse kinematics
  std::vector<double> seed, fk_values;
  std::vector<std::vector<double>> solutions;
  double timeout = 5.0;
  kinematics::KinematicsQueryOptions options;
  kinematics::KinematicsResult result;

  std::vector<std::string> fk_names;
  fk_names.push_back(kinematics_solver_->getTipFrame());
  robot_state::RobotState kinematic_state(robot_model_);

  unsigned int success = 0;
  unsigned int num_ik_solutions = 0;
  ros::WallTime start_time = ros::WallTime::now();
  for (unsigned int i = 0; i < num_ik_multiple_tests_; ++i)
  {
    seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    kinematic_state.setToRandomPositions(jmg_);
    kinematic_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);

    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));

    solutions.clear();
    kinematics_solver_->getPositionIK(poses, fk_values, solutions, result, options);
    ROS_DEBUG("Pose: %f %f %f", poses[0].position.x, poses[0].position.y, poses[0].position.z);
    ROS_DEBUG("Orient: %f %f %f %f", poses[0].orientation.x, poses[0].orientation.y, poses[0].orientation.z,
              poses[0].orientation.w);

    if (result.kinematic_error == kinematics::KinematicErrors::OK)
    {
      EXPECT_GT(solutions.size(), 0) << "Found " << solutions.size() << " ik solutions.";
      success = solutions.empty() ? success : success + 1;
      num_ik_solutions += solutions.size();
    }
    else
    {
      ROS_ERROR_STREAM("getPositionIK with multiple solutions failed on test " << i + 1 << " for group "
                                                                               << kinematics_solver_->getGroupName());
      continue;
    }

    std::vector<geometry_msgs::Pose> new_poses;
    new_poses.resize(1);

    for (unsigned int i = 0; i < solutions.size(); i++)
    {
      std::vector<double>& solution = solutions[i];
      EXPECT_TRUE(kinematics_solver_->getPositionFK(fk_names, solution, new_poses));
      EXPECT_NEAR(poses[0].position.x, new_poses[0].position.x, IK_NEAR);
      EXPECT_NEAR(poses[0].position.y, new_poses[0].position.y, IK_NEAR);
      EXPECT_NEAR(poses[0].position.z, new_poses[0].position.z, IK_NEAR);
      EXPECT_NEAR(poses[0].orientation.x, new_poses[0].orientation.x, IK_NEAR);
      EXPECT_NEAR(poses[0].orientation.y, new_poses[0].orientation.y, IK_NEAR);
      EXPECT_NEAR(poses[0].orientation.z, new_poses[0].orientation.z, IK_NEAR);
      EXPECT_NEAR(poses[0].orientation.w, new_poses[0].orientation.w, IK_NEAR);
    }
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / num_ik_multiple_tests_);
  EXPECT_GT(success, EXPECTED_SUCCESS_RATE * num_ik_multiple_tests_)
      << "A total of " << num_ik_solutions << " ik solutions were found out of " << num_ik_multiple_tests_ << " tests.";
  ROS_INFO_STREAM("Elapsed time: " << (ros::WallTime::now() - start_time).toSec());
}

TEST_F(KinematicsTest, getNearestIKSolution)
{
  // getmultipleIK intialization
  std::vector<std::vector<double>> solutions;
  double timeout = 5.0;
  kinematics::KinematicsQueryOptions options;
  kinematics::KinematicsResult result;

  // getIK intialization
  moveit_msgs::MoveItErrorCodes error_code;
  std::vector<double> solution_getIK;

  // forward_kinematics and seed
  std::vector<double> seed, fk_values;
  fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
  seed.resize(fk_values.size(), 0.0);

  std::vector<std::string> fk_names;
  fk_names.push_back(kinematics_solver_->getTipFrame());
  robot_state::RobotState kinematic_state(robot_model_);

  // Bounds from robot model for seed
  robot_model::JointBoundsVector joint_bounds = jmg_->getActiveJointModelsBounds();

  std::vector<double> min, max;
  min.resize(seed.size(), -M_PI);
  max.resize(seed.size(), M_PI);
  for (std::size_t i = 0; i < seed.size(); i++)
  {
    const robot_model::JointModel::Bounds& bounds = *joint_bounds[i];
    for (std::size_t j = 0; j < bounds.size(); j++)
    {
      // read from the model if joint is bounded
      if (bounds[j].position_bounded_)
      {
        min[i] = bounds[j].min_position_;
        max[i] = bounds[j].max_position_;
      }
    }
  }

  srand((unsigned)time(0));  // seed random number generator
  ros::WallTime start_time = ros::WallTime::now();

  for (unsigned int i = 0; i < num_nearest_ik_tests_; ++i)
  {
    kinematic_state.setToRandomPositions(jmg_);
    kinematic_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);

    // populating seed vector
    for (std::size_t i = 0; i < seed.size(); i++)
    {
      double rnd = (double)rand() / (double)RAND_MAX;
      seed[i] = min[i] + rnd * (max[i] - min[i]);
    }

    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));

    // uncomment next line to see the behavior change of KDL plugin
    //    seed = fk_values;

    // getPositionIK for single solution
    solution_getIK.clear();
    kinematics_solver_->getPositionIK(poses[0], seed, solution_getIK, error_code);
    ROS_DEBUG("Pose: %f %f %f", poses[0].position.x, poses[0].position.y, poses[0].position.z);
    ROS_DEBUG("Orient: %f %f %f %f", poses[0].orientation.x, poses[0].orientation.y, poses[0].orientation.z,
              poses[0].orientation.w);

    // check if getPositionIK call for single solution returns solution
    if (error_code.val == error_code.SUCCESS)
    {
      double error_getIK = 0.0;
      for (std::size_t j = 0; j < seed.size(); ++j)
      {
        error_getIK += fabs(solution_getIK[j] - seed[j]);
      }

      // getPositionIK for multiple solutions
      solutions.clear();
      kinematics_solver_->getPositionIK(poses, seed, solutions, result, options);
      ROS_DEBUG("Pose: %f %f %f", poses[0].position.x, poses[0].position.y, poses[0].position.z);
      ROS_DEBUG("Orient: %f %f %f %f", poses[0].orientation.x, poses[0].orientation.y, poses[0].orientation.z,
                poses[0].orientation.w);

      // check if getPositionIK call for multiple solutions returns solution
      if (result.kinematic_error == kinematics::KinematicErrors::OK)
      {
        double smallest_error_multipleIK = (std::numeric_limits<double>::max());
        for (unsigned int i = 0; i < solutions.size(); i++)
        {
          std::vector<double>& solution_multipleIK = solutions[i];
          double error_multipleIK = 0.0;
          for (std::size_t j = 0; j < seed.size(); ++j)
          {
            error_multipleIK += fabs(solution_multipleIK[j] - seed[j]);
          }

          if (error_multipleIK <= smallest_error_multipleIK)
          {
            smallest_error_multipleIK = error_multipleIK;
          }
        }
        ASSERT_NEAR(smallest_error_multipleIK, error_getIK, IK_NEAR);
      }
      else
      {
        ROS_ERROR_STREAM("Multiple solution call fails even when single solution do exit");
      }
    }
    else
    {
      ROS_ERROR_STREAM("No Solution Found");
    }
  }

  ROS_INFO_STREAM("Elapsed time: " << (ros::WallTime::now() - start_time).toSec());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kinematics_plugin_test");
  return RUN_ALL_TESTS();
}
