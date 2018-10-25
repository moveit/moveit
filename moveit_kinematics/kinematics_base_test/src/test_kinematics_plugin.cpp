/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Southwest Research Institute
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

/* Author: Jorge Nicho */

#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#define IK_NEAR 1e-4
#define IK_NEAR_TRANSLATE 1e-5

typedef pluginlib::ClassLoader<kinematics::KinematicsBase> KinematicsLoader;

const std::string PLUGIN_NAME_PARAM = "ik_plugin_name";
const std::string GROUP_PARAM = "group";
const std::string TIP_LINK_PARAM = "tip_link";
const std::string ROOT_LINK_PARAM = "root_link";
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string JOINT_NAMES_PARAM = "joint_names";
const std::string NUM_FK_TESTS = "num_fk_tests";
const std::string NUM_IK_CB_TESTS = "num_ik_cb_tests";
const std::string NUM_IK_TESTS = "num_ik_tests";
const std::string NUM_IK_MULTIPLE_TESTS = "num_ik_multiple_tests";
const std::string NUM_NEAREST_IK_TESTS = "num_nearest_ik_tests";
const double DEFAULT_SEARCH_DISCRETIZATION = 0.01f;
const double EXPECTED_SUCCESS_RATE = 0.8;

class KinematicsTest
{
public:
  bool initialize()
  {
    ros::NodeHandle ph("~");
    std::string plugin_name;

    // loading plugin
    kinematics_loader_.reset(new KinematicsLoader("moveit_core", "kinematics::KinematicsBase"));
    if (ph.getParam(PLUGIN_NAME_PARAM, plugin_name))
    {
      try
      {
        ROS_INFO_STREAM("Loading " << plugin_name);
        kinematics_solver_.reset(kinematics_loader_->createUnmanagedInstance(plugin_name));
      }
      catch (pluginlib::PluginlibException& e)
      {
        ROS_ERROR_STREAM("Plugin failed to load: " << e.what());
        EXPECT_TRUE(false);
        return false;
      }
    }
    else
    {
      ROS_ERROR_STREAM("The plugin " << plugin_name << " was not found");
      EXPECT_TRUE(false);
      return false;
    }

    // initializing plugin
    if (ph.getParam(GROUP_PARAM, group_name_) && ph.getParam(TIP_LINK_PARAM, tip_link_) &&
        ph.getParam(ROOT_LINK_PARAM, root_link_) && ph.getParam(JOINT_NAMES_PARAM, joints_))
    {
      if (kinematics_solver_->initialize(ROBOT_DESCRIPTION_PARAM, group_name_, root_link_, tip_link_,
                                         DEFAULT_SEARCH_DISCRETIZATION))
      {
        ROS_INFO_STREAM("Kinematics Solver plugin initialzed");
      }
      else
      {
        ROS_ERROR_STREAM("Kinematics Solver failed to initialize");
        EXPECT_TRUE(false);
        return false;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Kinematics Solver parameters failed to load");
      EXPECT_TRUE(false);
      return false;
    }

    // loading test details parameters
    if (ph.getParam(NUM_FK_TESTS, num_fk_tests_) && ph.getParam(NUM_IK_CB_TESTS, num_ik_cb_tests_) &&
        ph.getParam(NUM_IK_TESTS, num_ik_tests_) && ph.getParam(NUM_IK_MULTIPLE_TESTS, num_ik_multiple_tests_) &&
        ph.getParam(NUM_NEAREST_IK_TESTS, num_nearest_ik_tests_))
    {
      ROS_INFO_STREAM("Loaded test parameters");
    }
    else
    {
      ROS_ERROR_STREAM("Failed to load test parameters");
      EXPECT_TRUE(false);
    }

    return true;
  }

  void searchIKCallback(const geometry_msgs::Pose& ik_pose, const std::vector<double>& joint_state,
                        moveit_msgs::MoveItErrorCodes& error_code)
  {
    std::vector<std::string> link_names;
    link_names.push_back(tip_link_);
    std::vector<geometry_msgs::Pose> solutions;
    solutions.resize(1);
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
  };

public:
  kinematics::KinematicsBasePtr kinematics_solver_;
  boost::shared_ptr<KinematicsLoader> kinematics_loader_;
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

KinematicsTest kinematics_test;

TEST(IKFastPlugin, initialize)
{
  EXPECT_TRUE(kinematics_test.initialize());

  // Test getting chain information
  EXPECT_TRUE(kinematics_test.root_link_ == kinematics_test.kinematics_solver_->getBaseFrame());
  EXPECT_TRUE(kinematics_test.tip_link_ == kinematics_test.kinematics_solver_->getTipFrame());
  std::vector<std::string> joint_names = kinematics_test.kinematics_solver_->getJointNames();
  EXPECT_EQ((int)joint_names.size(), kinematics_test.joints_.size());

  for (unsigned int i = 0; i < joint_names.size(); i++)
  {
    std::vector<std::string>::iterator pos =
        std::find(kinematics_test.joints_.begin(), kinematics_test.joints_.end(), joint_names[i]);
    EXPECT_TRUE(pos != kinematics_test.joints_.end()) << "Joint " << joint_names[i] << " not found in list";
  }
}

TEST(IKFastPlugin, getFK)
{
  // loading robot model
  rdf_loader::RDFLoader rdf_loader(ROBOT_DESCRIPTION_PARAM);
  robot_model::RobotModelPtr kinematic_model;
  const boost::shared_ptr<srdf::Model>& srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf));

  // fk solution setup
  robot_model::JointModelGroup* joint_model_group =
      kinematic_model->getJointModelGroup(kinematics_test.kinematics_solver_->getGroupName());
  std::vector<double> seed, fk_values, solution;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
  std::vector<std::string> fk_names;
  fk_names.push_back(kinematics_test.kinematics_solver_->getTipFrame());
  robot_state::RobotState kinematic_state(kinematic_model);

  bool succeeded;
  int success = 0;
  ros::WallTime start_time = ros::WallTime::now();
  for (unsigned int i = 0; i < kinematics_test.num_fk_tests_; ++i)
  {
    seed.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
    kinematic_state.setToRandomPositions(joint_model_group);
    kinematic_state.copyJointGroupPositions(joint_model_group, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);
    succeeded = kinematics_test.kinematics_solver_->getPositionFK(fk_names, fk_values, poses);
    if (succeeded && (poses.size() == 1))
    {
      success++;
    }
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / kinematics_test.num_fk_tests_);
  EXPECT_GT(success, EXPECTED_SUCCESS_RATE * kinematics_test.num_fk_tests_);
  ROS_INFO_STREAM("Elapsed time: " << (ros::WallTime::now() - start_time).toSec());
}

TEST(IKFastPlugin, searchIK)
{
  rdf_loader::RDFLoader rdf_loader_(ROBOT_DESCRIPTION_PARAM);
  robot_model::RobotModelPtr kinematic_model;
  const boost::shared_ptr<srdf::Model>& srdf_model = rdf_loader_.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader_.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf_model));
  robot_model::JointModelGroup* joint_model_group =
      kinematic_model->getJointModelGroup(kinematics_test.kinematics_solver_->getGroupName());

  // Test inverse kinematics
  std::vector<double> seed, fk_values, solution;
  double timeout = 5.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
  std::vector<std::string> fk_names;
  fk_names.push_back(kinematics_test.kinematics_solver_->getTipFrame());
  robot_state::RobotState kinematic_state(kinematic_model);

  unsigned int success = 0;
  ros::WallTime start_time = ros::WallTime::now();
  for (unsigned int i = 0; i < kinematics_test.num_ik_tests_; ++i)
  {
    seed.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
    kinematic_state.setToRandomPositions(joint_model_group);
    kinematic_state.copyJointGroupPositions(joint_model_group, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);

    bool result_fk = kinematics_test.kinematics_solver_->getPositionFK(fk_names, fk_values, poses);
    ASSERT_TRUE(result_fk);
    solution.clear();
    kinematics_test.kinematics_solver_->searchPositionIK(poses[0], seed, timeout, solution, error_code);
    bool result = error_code.val == error_code.SUCCESS;

    ROS_DEBUG("Pose: %f %f %f", poses[0].position.x, poses[0].position.y, poses[0].position.z);
    ROS_DEBUG("Orient: %f %f %f %f", poses[0].orientation.x, poses[0].orientation.y, poses[0].orientation.z,
              poses[0].orientation.w);

    if (result)
    {
      EXPECT_TRUE(kinematics_test.kinematics_solver_->getPositionIK(poses[0], solution, solution, error_code));
      result = error_code.val == error_code.SUCCESS;
    }

    if (result)
    {
      success++;
    }
    else
    {
      ROS_ERROR_STREAM("searchPositionIK failed on test " << i + 1);
      continue;
    }

    std::vector<geometry_msgs::Pose> new_poses;
    new_poses.resize(1);
    result_fk = kinematics_test.kinematics_solver_->getPositionFK(fk_names, solution, new_poses);
    EXPECT_NEAR(poses[0].position.x, new_poses[0].position.x, IK_NEAR);
    EXPECT_NEAR(poses[0].position.y, new_poses[0].position.y, IK_NEAR);
    EXPECT_NEAR(poses[0].position.z, new_poses[0].position.z, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.x, new_poses[0].orientation.x, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.y, new_poses[0].orientation.y, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.z, new_poses[0].orientation.z, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.w, new_poses[0].orientation.w, IK_NEAR);
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / kinematics_test.num_ik_tests_);
  EXPECT_GT(success, EXPECTED_SUCCESS_RATE * kinematics_test.num_ik_tests_);
  ROS_INFO_STREAM("Elapsed time: " << (ros::WallTime::now() - start_time).toSec());
}

TEST(IKFastPlugin, searchIKWithCallback)
{
  rdf_loader::RDFLoader rdf_loader_(ROBOT_DESCRIPTION_PARAM);
  robot_model::RobotModelPtr kinematic_model;
  const boost::shared_ptr<srdf::Model>& srdf_model = rdf_loader_.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader_.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf_model));
  robot_model::JointModelGroup* joint_model_group =
      kinematic_model->getJointModelGroup(kinematics_test.kinematics_solver_->getGroupName());

  // Test inverse kinematics
  std::vector<double> seed, fk_values, solution;
  double timeout = 5.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
  std::vector<std::string> fk_names;
  fk_names.push_back(kinematics_test.kinematics_solver_->getTipFrame());
  robot_state::RobotState kinematic_state(kinematic_model);

  unsigned int success = 0;
  ros::WallTime start_time = ros::WallTime::now();
  for (unsigned int i = 0; i < kinematics_test.num_ik_cb_tests_; ++i)
  {
    seed.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
    kinematic_state.setToRandomPositions(joint_model_group);
    kinematic_state.copyJointGroupPositions(joint_model_group, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);

    bool result_fk = kinematics_test.kinematics_solver_->getPositionFK(fk_names, fk_values, poses);
    ASSERT_TRUE(result_fk);

    // check height
    if (poses[0].position.z <= 0.0f)
    {
      continue;
    }

    solution.clear();
    kinematics_test.kinematics_solver_->searchPositionIK(
        poses[0], fk_values, timeout, solution,
        boost::bind(&KinematicsTest::searchIKCallback, &kinematics_test, _1, _2, _3), error_code);
    bool result = error_code.val == error_code.SUCCESS;

    ROS_DEBUG("Pose: %f %f %f", poses[0].position.x, poses[0].position.y, poses[0].position.z);
    ROS_DEBUG("Orient: %f %f %f %f", poses[0].orientation.x, poses[0].orientation.y, poses[0].orientation.z,
              poses[0].orientation.w);

    if (result)
    {
      EXPECT_TRUE(kinematics_test.kinematics_solver_->getPositionIK(poses[0], solution, solution, error_code));
      result = error_code.val == error_code.SUCCESS;
    }

    if (result)
    {
      success++;
    }
    else
    {
      ROS_ERROR_STREAM("searchPositionIK failed on test " << i + 1);
      continue;
    }

    std::vector<geometry_msgs::Pose> new_poses;
    new_poses.resize(1);
    result_fk = kinematics_test.kinematics_solver_->getPositionFK(fk_names, solution, new_poses);
    EXPECT_NEAR(poses[0].position.x, new_poses[0].position.x, IK_NEAR);
    EXPECT_NEAR(poses[0].position.y, new_poses[0].position.y, IK_NEAR);
    EXPECT_NEAR(poses[0].position.z, new_poses[0].position.z, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.x, new_poses[0].orientation.x, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.y, new_poses[0].orientation.y, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.z, new_poses[0].orientation.z, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.w, new_poses[0].orientation.w, IK_NEAR);
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / kinematics_test.num_ik_cb_tests_);
  EXPECT_GT(success, EXPECTED_SUCCESS_RATE * kinematics_test.num_ik_cb_tests_);
  ROS_INFO_STREAM("Elapsed time: " << (ros::WallTime::now() - start_time).toSec());
}

TEST(IKFastPlugin, getIK)
{
  rdf_loader::RDFLoader rdf_loader_(ROBOT_DESCRIPTION_PARAM);
  robot_model::RobotModelPtr kinematic_model;
  const boost::shared_ptr<srdf::Model>& srdf_model = rdf_loader_.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader_.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf_model));
  robot_model::JointModelGroup* joint_model_group =
      kinematic_model->getJointModelGroup(kinematics_test.kinematics_solver_->getGroupName());

  // Test inverse kinematics
  std::vector<double> seed, fk_values, solution;
  double timeout = 5.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
  std::vector<std::string> fk_names;
  fk_names.push_back(kinematics_test.kinematics_solver_->getTipFrame());
  robot_state::RobotState kinematic_state(kinematic_model);

  unsigned int success = 0;
  ros::WallTime start_time = ros::WallTime::now();
  for (unsigned int i = 0; i < kinematics_test.num_ik_tests_; ++i)
  {
    seed.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
    kinematic_state.setToRandomPositions(joint_model_group);
    kinematic_state.copyJointGroupPositions(joint_model_group, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);

    bool result_fk = kinematics_test.kinematics_solver_->getPositionFK(fk_names, fk_values, poses);
    ASSERT_TRUE(result_fk);
    solution.clear();

    kinematics_test.kinematics_solver_->getPositionIK(poses[0], fk_values, solution, error_code);
    ROS_DEBUG("Pose: %f %f %f", poses[0].position.x, poses[0].position.y, poses[0].position.z);
    ROS_DEBUG("Orient: %f %f %f %f", poses[0].orientation.x, poses[0].orientation.y, poses[0].orientation.z,
              poses[0].orientation.w);

    if (error_code.val == error_code.SUCCESS)
    {
      success++;
    }
    else
    {
      ROS_ERROR_STREAM("getPositionIK failed on test " << i + 1 << " for group "
                                                       << kinematics_test.kinematics_solver_->getGroupName());
      continue;
    }

    std::vector<geometry_msgs::Pose> new_poses;
    new_poses.resize(1);
    result_fk = kinematics_test.kinematics_solver_->getPositionFK(fk_names, solution, new_poses);
    EXPECT_NEAR(poses[0].position.x, new_poses[0].position.x, IK_NEAR);
    EXPECT_NEAR(poses[0].position.y, new_poses[0].position.y, IK_NEAR);
    EXPECT_NEAR(poses[0].position.z, new_poses[0].position.z, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.x, new_poses[0].orientation.x, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.y, new_poses[0].orientation.y, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.z, new_poses[0].orientation.z, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.w, new_poses[0].orientation.w, IK_NEAR);
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / kinematics_test.num_ik_tests_);
  EXPECT_GT(success, EXPECTED_SUCCESS_RATE * kinematics_test.num_ik_tests_);
  ROS_INFO_STREAM("Elapsed time: " << (ros::WallTime::now() - start_time).toSec());
}

TEST(IKFastPlugin, getIKMultipleSolutions)
{
  rdf_loader::RDFLoader rdf_loader_(ROBOT_DESCRIPTION_PARAM);
  robot_model::RobotModelPtr kinematic_model;
  const boost::shared_ptr<srdf::Model>& srdf_model = rdf_loader_.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader_.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf_model));
  robot_model::JointModelGroup* joint_model_group =
      kinematic_model->getJointModelGroup(kinematics_test.kinematics_solver_->getGroupName());

  // Test inverse kinematics
  std::vector<double> seed, fk_values;
  std::vector<std::vector<double>> solutions;
  double timeout = 5.0;
  kinematics::KinematicsQueryOptions options;
  kinematics::KinematicsResult result;

  std::vector<std::string> fk_names;
  fk_names.push_back(kinematics_test.kinematics_solver_->getTipFrame());
  robot_state::RobotState kinematic_state(kinematic_model);

  unsigned int success = 0;
  unsigned int num_ik_solutions = 0;
  ros::WallTime start_time = ros::WallTime::now();
  for (unsigned int i = 0; i < kinematics_test.num_ik_multiple_tests_; ++i)
  {
    seed.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
    kinematic_state.setToRandomPositions(joint_model_group);
    kinematic_state.copyJointGroupPositions(joint_model_group, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);

    ASSERT_TRUE(kinematics_test.kinematics_solver_->getPositionFK(fk_names, fk_values, poses));

    solutions.clear();
    kinematics_test.kinematics_solver_->getPositionIK(poses, fk_values, solutions, result, options);
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
      ROS_ERROR_STREAM("getPositionIK with multiple solutions failed on test "
                       << i + 1 << " for group " << kinematics_test.kinematics_solver_->getGroupName());
      continue;
    }

    std::vector<geometry_msgs::Pose> new_poses;
    new_poses.resize(1);

    for (unsigned int i = 0; i < solutions.size(); i++)
    {
      std::vector<double>& solution = solutions[i];
      EXPECT_TRUE(kinematics_test.kinematics_solver_->getPositionFK(fk_names, solution, new_poses));
      EXPECT_NEAR(poses[0].position.x, new_poses[0].position.x, IK_NEAR);
      EXPECT_NEAR(poses[0].position.y, new_poses[0].position.y, IK_NEAR);
      EXPECT_NEAR(poses[0].position.z, new_poses[0].position.z, IK_NEAR);
      EXPECT_NEAR(poses[0].orientation.x, new_poses[0].orientation.x, IK_NEAR);
      EXPECT_NEAR(poses[0].orientation.y, new_poses[0].orientation.y, IK_NEAR);
      EXPECT_NEAR(poses[0].orientation.z, new_poses[0].orientation.z, IK_NEAR);
      EXPECT_NEAR(poses[0].orientation.w, new_poses[0].orientation.w, IK_NEAR);
    }
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / kinematics_test.num_ik_multiple_tests_);
  EXPECT_GT(success, EXPECTED_SUCCESS_RATE * kinematics_test.num_ik_multiple_tests_)
      << "A total of " << num_ik_solutions << " ik solutions were found out of "
      << kinematics_test.num_ik_multiple_tests_ << " tests.";
  ROS_INFO_STREAM("Elapsed time: " << (ros::WallTime::now() - start_time).toSec());
}

TEST(IKFastPlugin, getNearestIKSolution)
{
  rdf_loader::RDFLoader rdf_loader_(ROBOT_DESCRIPTION_PARAM);
  robot_model::RobotModelPtr kinematic_model;
  const boost::shared_ptr<srdf::Model>& srdf_model = rdf_loader_.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader_.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf_model));
  robot_model::JointModelGroup* joint_model_group =
      kinematic_model->getJointModelGroup(kinematics_test.kinematics_solver_->getGroupName());

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
  fk_values.resize(kinematics_test.kinematics_solver_->getJointNames().size(), 0.0);
  seed.resize(fk_values.size(), 0.0);

  std::vector<std::string> fk_names;
  fk_names.push_back(kinematics_test.kinematics_solver_->getTipFrame());
  robot_state::RobotState kinematic_state(kinematic_model);

  // Bounds from robot model for seed
  robot_model::JointBoundsVector joint_bounds = joint_model_group->getActiveJointModelsBounds();

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

  for (unsigned int i = 0; i < kinematics_test.num_nearest_ik_tests_; ++i)
  {
    kinematic_state.setToRandomPositions(joint_model_group);
    kinematic_state.copyJointGroupPositions(joint_model_group, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);

    // populating seed vector
    for (std::size_t i = 0; i < seed.size(); i++)
    {
      double rnd = (double)rand() / (double)RAND_MAX;
      seed[i] = min[i] + rnd * (max[i] - min[i]);
    }

    ASSERT_TRUE(kinematics_test.kinematics_solver_->getPositionFK(fk_names, fk_values, poses));

    // uncomment next line to see the behavior change of KDL plugin
    //    seed = fk_values;

    // getPositionIK for single solution
    solution_getIK.clear();
    kinematics_test.kinematics_solver_->getPositionIK(poses[0], seed, solution_getIK, error_code);
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
      kinematics_test.kinematics_solver_->getPositionIK(poses, seed, solutions, result, options);
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
