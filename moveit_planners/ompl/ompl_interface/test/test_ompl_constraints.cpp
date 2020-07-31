/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, KU Leuven
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

/* Author: Jeroen De Maeyer */

/** This file tests the implementation of constriants inheriting from
 * the ompl::base::Constraint class in the file /detail/ompl_constraint.h/cpp.
 * These are used to create an ompl::base::ConstrainedStateSpace to plan with path constraints.
 **/

#include <moveit/ompl_interface/detail/ompl_constraints.h>

#include <memory>
#include <string>
#include <iostream>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <ompl/util/Exception.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>

/** \brief Number of times to run a test that uses randomly generated input. **/
constexpr int NUM_RANDOM_TESTS{ 10 };

/** \brief For failing tests, some extra print statements are useful. **/
constexpr bool VERBOSE{ false };

/** \brief Allowed error when comparing Jacobian matrix error.
 *
 * High tolerance because of high finite difference error.
 * (And it is the L1-norm over the whole matrix difference.)
 **/
constexpr double JAC_ERROR_TOLERANCE{ 1e-4 };

/** \brief Helper function to create a specific position constraint. **/
moveit_msgs::PositionConstraint createPositionConstraint(std::string& base_link, std::string& ee_link_name)
{
  shape_msgs::SolidPrimitive box_constraint;
  box_constraint.type = shape_msgs::SolidPrimitive::BOX;
  box_constraint.dimensions = { 0.05, 0.4, 0.05 }; /* use -1 to indicate no constraints. */

  geometry_msgs::Pose box_pose;
  box_pose.position.x = 0.9;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.2;
  box_pose.orientation.w = 1.0;

  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = base_link;
  position_constraint.link_name = ee_link_name;
  position_constraint.constraint_region.primitives.push_back(box_constraint);
  position_constraint.constraint_region.primitive_poses.push_back(box_pose);

  return position_constraint;
}

moveit_msgs::OrientationConstraint createOrientationConstraint(std::string& base_link, std::string& ee_link_name)
{
  moveit_msgs::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = base_link;
  orientation_constraint.link_name = ee_link_name;
  orientation_constraint.orientation.w = 1.0;
  orientation_constraint.absolute_x_axis_tolerance = 0.1;
  orientation_constraint.absolute_y_axis_tolerance = 0.1;
  orientation_constraint.absolute_z_axis_tolerance = -1.0;
  return orientation_constraint;
}

/** \brief Robot indepentent test class implementing all tests
 *
 * All tests are implemented in a generic test fixture, so it is
 * easy to run them on different robots.
 *
 * based on
 * https://stackoverflow.com/questions/38207346/specify-constructor-arguments-for-a-google-test-fixture/38218657
 * (answer by PiotrNycz)
 *
 * It is implemented this way to avoid the ros specific test framework
 * outside moveit_ros.
 *
 * (This is an (uglier) alternative to using the rostest framework
 * and reading the robot settings from the parameter server.
 * Then we have several rostest launch files that load the parameters
 * for a specific robot and run the same compiled tests for all robots.)
 * */
class ConstraintTestBaseClass : public testing::Test
{
protected:
  ConstraintTestBaseClass(const std::string& robot_name, const std::string& group_name)
    : robot_name_(robot_name), group_name_(group_name)
  {
  }

  void SetUp() override
  {
    // load robot
    robot_model_ = moveit::core::loadTestingRobotModel(robot_name_);
    robot_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
    joint_model_group_ = robot_state_->getJointModelGroup(group_name_);

    // extract useful parameters for tests
    num_dofs_ = joint_model_group_->getVariableCount();
    ee_link_name_ = joint_model_group_->getLinkModelNames().back();
    base_link_name_ = robot_model_->getRootLinkName();
  };

  void TearDown() override
  {
  }

  const Eigen::Isometry3d fk(const Eigen::VectorXd& q) const
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    return robot_state_->getGlobalLinkTransform(ee_link_name_);
  }

  Eigen::VectorXd getRandomState()
  {
    robot_state_->setToRandomPositions(joint_model_group_);
    Eigen::VectorXd joint_positions;
    robot_state_->copyJointGroupPositions(joint_model_group_, joint_positions);
    return joint_positions;
  }

  Eigen::MatrixXd numericalJacobianPosition(const Eigen::VectorXd& q)
  {
    const double h{ 1e-6 }; /* step size for numerical derivation */

    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, num_dofs_);

    // helper matrix for differentiation.
    Eigen::MatrixXd m_helper = h * Eigen::MatrixXd::Identity(num_dofs_, num_dofs_);

    for (std::size_t dim{ 0 }; dim < num_dofs_; ++dim)
    {
      Eigen::Vector3d pos = fk(q).translation();
      Eigen::Vector3d pos_plus_h = fk(q + m_helper.col(dim)).translation();
      Eigen::Vector3d col = (pos_plus_h - pos) / h;
      jacobian.col(dim) = col;
    }
    return jacobian;
  }

  void setPositionConstraints()
  {
    moveit_msgs::Constraints constraint_msgs;
    constraint_msgs.position_constraints.push_back(createPositionConstraint(base_link_name_, ee_link_name_));

    constraint_ = std::make_shared<ompl_interface::PositionConstraint>(robot_model_, group_name_, num_dofs_);
    constraint_->init(constraint_msgs);
  }

  void setOrientationConstraints()
  {
    moveit_msgs::Constraints constraint_msgs;
    constraint_msgs.orientation_constraints.push_back(createOrientationConstraint(base_link_name_, ee_link_name_));

    constraint_ = std::make_shared<ompl_interface::AngleAxisConstraint>(robot_model_, group_name_, num_dofs_);
    constraint_->init(constraint_msgs);
  }

  void setPositionAndOrientationConstraint()
  {
    moveit_msgs::Constraints constraint_msgs;
    constraint_msgs.position_constraints.push_back(createPositionConstraint(base_link_name_, ee_link_name_));
    constraint_msgs.orientation_constraints.push_back(createOrientationConstraint(base_link_name_, ee_link_name_));

    constraint_ = std::make_shared<ompl_interface::PoseConstraint>(robot_model_, group_name_, num_dofs_);
    constraint_->init(constraint_msgs);
  }

  void testJacobian()
  {
    double total_error{ 999.9 };

    for (int i{ 0 }; i < NUM_RANDOM_TESTS; ++i)
    {
      auto q = getRandomState();
      auto jac_exact = constraint_->calcErrorJacobian(q);
      auto jac_approx = numericalJacobianPosition(q);

      if (VERBOSE)
      {
        std::cout << "Analytical jacobian: \n";
        std::cout << jac_exact << std::endl;
        std::cout << "Finite difference jacobian: \n";
        std::cout << jac_approx << std::endl;
      }

      total_error = (jac_exact - jac_approx).lpNorm<1>();
      EXPECT_LT(total_error, JAC_ERROR_TOLERANCE);
    }
  }

  void testOMPLProjectedStateSpaceConstruction()
  {
    auto state_space = std::make_shared<ompl::base::RealVectorStateSpace>(num_dofs_);
    ompl::base::RealVectorBounds bounds(num_dofs_);

    // get joint limits from the joint model group
    auto joint_limits = joint_model_group_->getActiveJointModelsBounds();
    EXPECT_EQ(joint_limits.size(), num_dofs_);
    for (std::size_t i{ 0 }; i < num_dofs_; ++i)
    {
      EXPECT_EQ(joint_limits[i]->size(), (unsigned int)1);
      bounds.setLow(i, joint_limits[i]->at(0).min_position_);
      bounds.setHigh(i, joint_limits[i]->at(0).max_position_);
    }

    state_space->setBounds(bounds);

    auto constrained_state_space = std::make_shared<ompl::base::ProjectedStateSpace>(state_space, constraint_);

    auto constrained_state_space_info =
        std::make_shared<ompl::base::ConstrainedSpaceInformation>(constrained_state_space);

    // TODO(jeroendm) Fix issues with sanity checks.
    // The jacobian test is expected to fail because of the discontinuous constraint derivative.
    // The issue with the state sampler is unresolved.
    // int flags = 1 & ompl::base::ConstrainedStateSpace::CONSTRAINED_STATESPACE_JACOBIAN;
    // flags = flags & ompl::base::ConstrainedStateSpace::CONSTRAINED_STATESPACE_SAMPLERS;
    try
    {
      constrained_state_space->sanityChecks();
    }
    catch (ompl::Exception& ex)
    {
      ROS_ERROR("Sanity checks did not pass: %s", ex.what());
    }
  }

protected:
  const std::string robot_name_;
  const std::string group_name_;

  moveit::core::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;

  std::shared_ptr<ompl_interface::BaseConstraint> constraint_;

  std::size_t num_dofs_;
  std::string base_link_name_;
  std::string ee_link_name_;
};

/***************************************************************************
 * Run all tests on the Panda robot
 * ************************************************************************/
class PandaConstraintTest : public ConstraintTestBaseClass
{
protected:
  PandaConstraintTest() : ConstraintTestBaseClass("panda", "panda_arm")
  {
  }
};

TEST_F(PandaConstraintTest, InitPositionConstraint)
{
  setPositionConstraints();
}

TEST_F(PandaConstraintTest, PositionConstraintJacobian)
{
  setPositionConstraints();
  testJacobian();
}

TEST_F(PandaConstraintTest, PositionConstraintOMPLCheck)
{
  setPositionConstraints();
  testOMPLProjectedStateSpaceConstruction();
}

TEST_F(PandaConstraintTest, InitAngleAxisConstraint)
{
  setOrientationConstraints();
}

TEST_F(PandaConstraintTest, AngleAxisConstraintOMPLCheck)
{
  setOrientationConstraints();
  testOMPLProjectedStateSpaceConstruction();
}

TEST_F(PandaConstraintTest, InitPositionAndOrientationConstraint)
{
  setPositionAndOrientationConstraint();
  testOMPLProjectedStateSpaceConstruction();
}

/***************************************************************************
 * Run all tests on the Fanuc robot
 * ************************************************************************/
class FanucConstraintTest : public ConstraintTestBaseClass
{
protected:
  FanucConstraintTest() : ConstraintTestBaseClass("fanuc", "manipulator")
  {
  }
};

TEST_F(FanucConstraintTest, InitPositionConstraint)
{
  setPositionConstraints();
}

TEST_F(FanucConstraintTest, PositionConstraintJacobian)
{
  setPositionConstraints();
  testJacobian();
}

TEST_F(FanucConstraintTest, PositionConstraintOMPLCheck)
{
  setPositionConstraints();
  testOMPLProjectedStateSpaceConstruction();
}

TEST_F(FanucConstraintTest, InitAngleAxisConstraint)
{
  setOrientationConstraints();
}

TEST_F(FanucConstraintTest, AngleAxisConstraintOMPLCheck)
{
  setOrientationConstraints();
  testOMPLProjectedStateSpaceConstruction();
}

/***************************************************************************
 * Run all tests on the PR2's left arm
 * ************************************************************************/
class PR2LeftArmConstraintTest : public ConstraintTestBaseClass
{
protected:
  PR2LeftArmConstraintTest() : ConstraintTestBaseClass("pr2", "left_arm")
  {
  }
};

TEST_F(PR2LeftArmConstraintTest, InitPositionConstraint)
{
  setPositionConstraints();
}

TEST_F(PR2LeftArmConstraintTest, PositionConstraintJacobian)
{
  setPositionConstraints();
  testJacobian();
}

TEST_F(PR2LeftArmConstraintTest, PositionConstraintOMPLCheck)
{
  setPositionConstraints();
  testOMPLProjectedStateSpaceConstruction();
}

TEST_F(PR2LeftArmConstraintTest, InitAngleAxisConstraint)
{
  setOrientationConstraints();
}

TEST_F(PR2LeftArmConstraintTest, AngleAxisConstraintOMPLCheck)
{
  setOrientationConstraints();
  testOMPLProjectedStateSpaceConstruction();
}

/***************************************************************************
 * MAIN
 * ************************************************************************/
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}