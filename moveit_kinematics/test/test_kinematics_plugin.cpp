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
#include <memory>
#include <boost/thread/mutex.hpp>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

const double IK_NEAR = 1e-5;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const double DEFAULT_SEARCH_DISCRETIZATION = 0.01f;
const double EXPECTED_SUCCESS_RATE = 0.8;

template <typename T>
inline bool getParam(const std::string& param, T& val)
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

// As loading of parameters is quite slow, we share them across all tests
class SharedData
{
  friend class KinematicsTest;
  typedef pluginlib::ClassLoader<kinematics::KinematicsBase> KinematicsLoader;

  robot_model::RobotModelPtr robot_model_;
  std::unique_ptr<KinematicsLoader> kinematics_loader_;
  std::string root_link_;
  std::string tip_link_;
  std::string group_name_;
  std::vector<std::string> joints_;
  int num_fk_tests_;
  int num_ik_cb_tests_;
  int num_ik_tests_;
  int num_ik_multiple_tests_;
  int num_nearest_ik_tests_;

  SharedData(SharedData const&) = delete;  // this is a singleton
  SharedData()
  {
    initialize();
  }

  void initialize()
  {
    ROS_INFO_STREAM("Loading robot model from " << ros::this_node::getNamespace() << "/" << ROBOT_DESCRIPTION_PARAM);
    // load robot model
    rdf_loader::RDFLoader rdf_loader(ROBOT_DESCRIPTION_PARAM);
    robot_model_ = std::make_shared<robot_model::RobotModel>(rdf_loader.getURDF(), rdf_loader.getSRDF());
    ASSERT_TRUE(bool(robot_model_)) << "Failed to load robot model";

    // init ClassLoader
    kinematics_loader_ = std::make_unique<KinematicsLoader>("moveit_core", "kinematics::KinematicsBase");
    ASSERT_TRUE(bool(kinematics_loader_)) << "Failed to instantiate ClassLoader";

    // load parameters
    ASSERT_TRUE(getParam("group", group_name_));
    ASSERT_TRUE(getParam("tip_link", tip_link_));
    ASSERT_TRUE(getParam("root_link", root_link_));
    ASSERT_TRUE(getParam("joint_names", joints_));
    ASSERT_TRUE(getParam("num_fk_tests", num_fk_tests_));
    ASSERT_TRUE(getParam("num_ik_cb_tests", num_ik_cb_tests_));
    ASSERT_TRUE(getParam("num_ik_tests", num_ik_tests_));
    ASSERT_TRUE(getParam("num_ik_multiple_tests", num_ik_multiple_tests_));
    ASSERT_TRUE(getParam("num_nearest_ik_tests", num_nearest_ik_tests_));

    ASSERT_TRUE(robot_model_->hasJointModelGroup(group_name_));
    ASSERT_TRUE(robot_model_->hasLinkModel(root_link_));
    ASSERT_TRUE(robot_model_->hasLinkModel(tip_link_));
  }

public:
  auto createUniqueInstance(const std::string& name) const
  {
    return kinematics_loader_->createUniqueInstance(name);
  }

  static const SharedData& instance()
  {
    static SharedData instance_;
    return instance_;
  }
  static void release()
  {
    SharedData& shared = const_cast<SharedData&>(instance());
    shared.kinematics_loader_.reset();
  }
};

class KinematicsTest : public ::testing::Test
{
protected:
  void operator=(const SharedData& data)
  {
    robot_model_ = data.robot_model_;
    jmg_ = robot_model_->getJointModelGroup(data.group_name_);
    root_link_ = data.root_link_;
    tip_link_ = data.tip_link_;
    group_name_ = data.group_name_;
    joints_ = data.joints_;
    num_fk_tests_ = data.num_fk_tests_;
    num_ik_cb_tests_ = data.num_ik_cb_tests_;
    num_ik_tests_ = data.num_ik_tests_;
    num_ik_multiple_tests_ = data.num_ik_multiple_tests_;
    num_nearest_ik_tests_ = data.num_nearest_ik_tests_;
  }

  void SetUp() override
  {
    *this = SharedData::instance();

    std::string plugin_name;
    ASSERT_TRUE(getParam("ik_plugin_name", plugin_name));
    ROS_INFO_STREAM("Loading " << plugin_name);
    kinematics_solver_ = SharedData::instance().createUniqueInstance(plugin_name);
    ASSERT_TRUE(bool(kinematics_solver_)) << "Failed to load plugin: " << plugin_name;

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
    ASSERT_FALSE(kinematics_solver_->getTipFrames().empty());
    ASSERT_EQ(tip_link_, kinematics_solver_->getTipFrame());
    ASSERT_EQ(joints_, kinematics_solver_->getJointNames());
  }

public:
  void expectNear(const std::vector<geometry_msgs::Pose>& first, const std::vector<geometry_msgs::Pose>& second,
                  double near = IK_NEAR)
  {
    EXPECT_EQ(first.size(), second.size());
    for (size_t i = 0; i < first.size(); ++i)
    {
      EXPECT_NEAR(first[i].position.x, second[i].position.x, near);
      EXPECT_NEAR(first[i].position.y, second[i].position.y, near);
      EXPECT_NEAR(first[i].position.z, second[i].position.z, near);
      EXPECT_NEAR(first[i].orientation.x, second[i].orientation.x, near);
      EXPECT_NEAR(first[i].orientation.y, second[i].orientation.y, near);
      EXPECT_NEAR(first[i].orientation.z, second[i].orientation.z, near);
      EXPECT_NEAR(first[i].orientation.w, second[i].orientation.w, near);
    }
  }

  void searchIKCallback(const geometry_msgs::Pose& ik_pose, const std::vector<double>& joint_state,
                        moveit_msgs::MoveItErrorCodes& error_code)
  {
    std::vector<std::string> link_names = { tip_link_ };
    std::vector<geometry_msgs::Pose> poses;
    if (!kinematics_solver_->getPositionFK(link_names, joint_state, poses))
    {
      error_code.val = error_code.PLANNING_FAILED;
      return;
    }

    EXPECT_GT(poses[0].position.z, 0.0f);
    if (poses[0].position.z > 0.0)
      error_code.val = error_code.SUCCESS;
    else
      error_code.val = error_code.PLANNING_FAILED;
  }

public:
  robot_model::RobotModelPtr robot_model_;
  robot_model::JointModelGroup* jmg_;
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
  std::vector<double> joints(kinematics_solver_->getJointNames().size(), 0.0);
  const std::vector<std::string>& tip_frames = kinematics_solver_->getTipFrames();
  robot_state::RobotState robot_state(robot_model_);

  for (unsigned int i = 0; i < num_fk_tests_; ++i)
  {
    robot_state.setToRandomPositions(jmg_);
    robot_state.copyJointGroupPositions(jmg_, joints);
    std::vector<geometry_msgs::Pose> poses;
    EXPECT_TRUE(kinematics_solver_->getPositionFK(tip_frames, joints, poses));
    EXPECT_EQ(poses.size(), tip_frames.size());
    if (poses.size() != tip_frames.size())
      continue;

    robot_state.updateLinkTransforms();
    for (size_t j = 0; j < poses.size(); ++j)
    {
      const Eigen::Isometry3d model_pose = robot_state.getGlobalLinkTransform(tip_frames[j]);
      Eigen::Isometry3d fk_pose;
      tf2::fromMsg(poses[j], fk_pose);
      EXPECT_TRUE(fk_pose.isApprox(model_pose)) << fk_pose.matrix() << std::endl << model_pose.matrix();
    }
  }
}

// perform random walk in joint-space, reaching poses via IK
TEST_F(KinematicsTest, randomWalkIK)
{
  std::vector<double> seed, goal, solution;
  const std::vector<std::string>& tip_frames = kinematics_solver_->getTipFrames();
  robot_state::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  bool publish_trajectory = false;
  getParam<bool>("publish_trajectory", publish_trajectory);
  moveit_msgs::DisplayTrajectory msg;
  msg.model_id = robot_model_->getName();
  moveit::core::robotStateToRobotStateMsg(robot_state, msg.trajectory_start);
  msg.trajectory.resize(1);
  robot_trajectory::RobotTrajectory traj(robot_model_, jmg_);

  unsigned int failures = 0;
  constexpr double NEAR_JOINT = 0.1;
  const std::vector<double> consistency_limits(jmg_->getVariableCount(), 1.05 * NEAR_JOINT);
  for (unsigned int i = 0; i < num_ik_tests_; ++i)
  {
    // remember previous pose
    robot_state.copyJointGroupPositions(jmg_, seed);
    // sample a new pose nearby
    robot_state.setToRandomPositionsNearBy(jmg_, robot_state, NEAR_JOINT);
    // get joints of new pose
    robot_state.copyJointGroupPositions(jmg_, goal);
    // compute target tip_frames
    std::vector<geometry_msgs::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(tip_frames, goal, poses));

    // compute IK
    moveit_msgs::MoveItErrorCodes error_code;
    kinematics_solver_->searchPositionIK(poses[0], seed, 0.1, consistency_limits, solution, error_code);
    if (error_code.val != error_code.SUCCESS)
    {
      ++failures;
      continue;
    }

    // on success: validate reached poses
    std::vector<geometry_msgs::Pose> reached_poses;
    kinematics_solver_->getPositionFK(tip_frames, solution, reached_poses);
    expectNear(poses, reached_poses);

    // validate closeness of solution pose to goal
    auto diff = Eigen::Map<Eigen::ArrayXd>(solution.data(), solution.size()) -
                Eigen::Map<Eigen::ArrayXd>(goal.data(), goal.size());
    if (!diff.isZero(1.05 * NEAR_JOINT))
    {
      ++failures;
      ROS_WARN_STREAM("jump in [" << i << "]: " << diff.transpose());
    }

    // update robot state to found pose
    robot_state.setJointGroupPositions(jmg_, solution);
    traj.addSuffixWayPoint(robot_state, 0.1);
  }
  EXPECT_LE(failures, (1.0 - EXPECTED_SUCCESS_RATE) * num_ik_tests_);

  if (publish_trajectory)
  {
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Publisher pub = nh.advertise<moveit_msgs::DisplayTrajectory>("display_random_walk", 1, true);
    traj.getRobotTrajectoryMsg(msg.trajectory[0]);
    pub.publish(msg);
    ros::WallDuration(0.1).sleep();
  }
}

TEST_F(KinematicsTest, searchIK)
{
  std::vector<double> seed, fk_values, solution;
  double timeout = 5.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(kinematics_solver_->getJointNames().size(), 0.0);
  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  robot_state::RobotState robot_state(robot_model_);

  unsigned int success = 0;
  for (unsigned int i = 0; i < num_ik_tests_; ++i)
  {
    seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    robot_state.setToRandomPositions(jmg_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));

    kinematics_solver_->searchPositionIK(poses[0], seed, timeout, solution, error_code);
    if (error_code.val == error_code.SUCCESS)
      success++;
    else
      continue;

    std::vector<geometry_msgs::Pose> reached_poses;
    kinematics_solver_->getPositionFK(fk_names, solution, reached_poses);
    expectNear(poses, reached_poses);
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / num_ik_tests_);
  EXPECT_GT(success, EXPECTED_SUCCESS_RATE * num_ik_tests_);
}

TEST_F(KinematicsTest, searchIKWithCallback)
{
  std::vector<double> seed, fk_values, solution;
  double timeout = 5.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(kinematics_solver_->getJointNames().size(), 0.0);
  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  robot_state::RobotState robot_state(robot_model_);

  unsigned int success = 0;
  for (unsigned int i = 0; i < num_ik_cb_tests_; ++i)
  {
    seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    robot_state.setToRandomPositions(jmg_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));
    if (poses[0].position.z <= 0.0f)
      continue;

    kinematics_solver_->searchPositionIK(poses[0], fk_values, timeout, solution,
                                         boost::bind(&KinematicsTest::searchIKCallback, this, _1, _2, _3), error_code);
    if (error_code.val == error_code.SUCCESS)
      success++;
    else
      continue;

    std::vector<geometry_msgs::Pose> reached_poses;
    kinematics_solver_->getPositionFK(fk_names, solution, reached_poses);
    expectNear(poses, reached_poses);
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / num_ik_cb_tests_);
  EXPECT_GT(success, EXPECTED_SUCCESS_RATE * num_ik_cb_tests_);
}

TEST_F(KinematicsTest, getIK)
{
  std::vector<double> seed, fk_values, solution;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(kinematics_solver_->getJointNames().size(), 0.0);
  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  robot_state::RobotState robot_state(robot_model_);

  unsigned int success = 0;
  for (unsigned int i = 0; i < num_ik_tests_; ++i)
  {
    seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    robot_state.setToRandomPositions(jmg_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;

    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));
    kinematics_solver_->getPositionIK(poses[0], fk_values, solution, error_code);
    if (error_code.val == error_code.SUCCESS)
      success++;
    else
      continue;

    std::vector<geometry_msgs::Pose> reached_poses;
    kinematics_solver_->getPositionFK(fk_names, solution, reached_poses);
    expectNear(poses, reached_poses);
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / num_ik_tests_);
  EXPECT_GT(success, EXPECTED_SUCCESS_RATE * num_ik_tests_);
}

TEST_F(KinematicsTest, getIKMultipleSolutions)
{
  std::vector<double> seed, fk_values;
  std::vector<std::vector<double>> solutions;
  kinematics::KinematicsQueryOptions options;
  kinematics::KinematicsResult result;

  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  robot_state::RobotState robot_state(robot_model_);

  unsigned int success = 0;
  for (unsigned int i = 0; i < num_ik_multiple_tests_; ++i)
  {
    seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    robot_state.setToRandomPositions(jmg_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));

    solutions.clear();
    kinematics_solver_->getPositionIK(poses, fk_values, solutions, result, options);

    if (result.kinematic_error == kinematics::KinematicErrors::OK)
      success += solutions.empty() ? 0 : 1;
    else
      continue;

    std::vector<geometry_msgs::Pose> reached_poses;
    for (const auto& s : solutions)
    {
      kinematics_solver_->getPositionFK(fk_names, s, reached_poses);
      expectNear(poses, reached_poses);
    }
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / num_ik_multiple_tests_);
  EXPECT_GT(success, EXPECTED_SUCCESS_RATE * num_ik_multiple_tests_);
}

// validate that getPositionIK() retrieves closest solution to seed
TEST_F(KinematicsTest, getNearestIKSolution)
{
  std::vector<std::vector<double>> solutions;
  kinematics::KinematicsQueryOptions options;
  kinematics::KinematicsResult result;

  std::vector<double> seed, fk_values, solution;
  moveit_msgs::MoveItErrorCodes error_code;
  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  robot_state::RobotState robot_state(robot_model_);

  for (unsigned int i = 0; i < num_nearest_ik_tests_; ++i)
  {
    robot_state.setToRandomPositions(jmg_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));

    // sample seed vector
    robot_state.setToRandomPositions(jmg_);
    robot_state.copyJointGroupPositions(jmg_, seed);

    // getPositionIK for single solution
    kinematics_solver_->getPositionIK(poses[0], seed, solution, error_code);

    // check if getPositionIK call for single solution returns solution
    if (error_code.val != error_code.SUCCESS)
      continue;

    const Eigen::Map<const Eigen::VectorXd> seed_eigen(seed.data(), seed.size());
    double error_getIK =
        (Eigen::Map<const Eigen::VectorXd>(solution.data(), solution.size()) - seed_eigen).array().abs().sum();

    // getPositionIK for multiple solutions
    solutions.clear();
    kinematics_solver_->getPositionIK(poses, seed, solutions, result, options);

    // check if getPositionIK call for multiple solutions returns solution
    EXPECT_EQ(result.kinematic_error, kinematics::KinematicErrors::OK)
        << "Multiple solution call failed, while single solution call succeeded";
    if (result.kinematic_error != kinematics::KinematicErrors::OK)
      continue;

    double smallest_error_multipleIK = std::numeric_limits<double>::max();
    for (const auto& s : solutions)
    {
      double error_multipleIK =
          (Eigen::Map<const Eigen::VectorXd>(s.data(), s.size()) - seed_eigen).array().abs().sum();
      if (error_multipleIK <= smallest_error_multipleIK)
        smallest_error_multipleIK = error_multipleIK;
    }
    EXPECT_NEAR(smallest_error_multipleIK, error_getIK, IK_NEAR);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kinematics_plugin_test");
  ros::NodeHandle nh;
  int result = RUN_ALL_TESTS();
  SharedData::release();  // avoid class_loader::LibraryUnloadException
  return result;
}
