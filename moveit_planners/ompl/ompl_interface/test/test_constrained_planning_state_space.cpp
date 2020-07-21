/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Jeroen De Maeyer
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

#include <moveit/ompl_interface/parameterization/joint_space/constrained_planning_state_space.h>

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

/** dummy constraint for testing, always satisfied. **/
class DummyConstraint : public ompl::base::Constraint
{
public:
  DummyConstraint(const unsigned int num_dofs) : ompl::base::Constraint(num_dofs, 1)
  {
  }
  void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override
  {
    out[0] = 0.0;
  }
};

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
class LoadTestRobotBaseClass : public testing::Test
{
protected:
  LoadTestRobotBaseClass(const std::string& robot_name, const std::string& group_name)
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

    // create a constrained state space for testing
    setupMoveItStateSpace();
    setupOMPLStateSpace();
  };

  void TearDown() override
  {
  }

  Eigen::VectorXd getRandomState()
  {
    robot_state_->setToRandomPositions(joint_model_group_);
    Eigen::VectorXd q;
    robot_state_->copyJointGroupPositions(joint_model_group_, q);
    return q;
  }

  /** \brief Create a joint position vector with values 0.1, 0.2, 0.3, ...
   * where the length depends on the number of joints in the robot.
   *
   * We could also use a random state created in the method above,
   * but do we want this randomness in tests??
   * */
  Eigen::VectorXd getDeterministicState()
  {
    Eigen::VectorXd state(num_dofs_);
    double value = 0.1;
    for (std::size_t i = 0; i < state.size(); ++i)
    {
      state[i] = value;
      value += 0.1;
    }
    return state;
  }

  void setupMoveItStateSpace()
  {
    ompl_interface::ModelBasedStateSpaceSpecification space_spec(robot_model_, group_name_);
    moveit_state_space_ = std::make_shared<ompl_interface::ConstrainedPlanningStateSpace>(space_spec);
  }

  void setupOMPLStateSpace()
  {
    // first call setupMoveItStateSpace()
    assert(moveit_state_space_ != nullptr);
    auto con = std::make_shared<DummyConstraint>(num_dofs_);
    constrained_state_space_ = std::make_shared<ompl::base::ProjectedStateSpace>(moveit_state_space_, con);
  }

  void copyToRobotStateTest()
  {
    // create and OMPL state
    Eigen::VectorXd joint_positions = getDeterministicState();
    ompl::base::ScopedState<> ompl_state(constrained_state_space_);

    // set OMPL state using another moveit state
    moveit::core::RobotState init_moveit_state(robot_model_);
    init_moveit_state.setToDefaultValues();
    init_moveit_state.setJointGroupPositions(joint_model_group_, joint_positions);
    moveit_state_space_->copyToOMPLState(ompl_state.get(), init_moveit_state);

    // ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()->copy(joint_positions);
    // ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>()
    //     ->getState()
    //     ->as<ompl_interface::ConstrainedPlanningStateSpace>()
    //     ->copyFromReals

    // copy into a MoveIt state
    moveit::core::RobotState moveit_state(robot_model_);
    moveit_state.setToDefaultValues();
    moveit_state_space_->copyToRobotState(moveit_state, ompl_state.get());

    // check if copy worked out as expected
    Eigen::VectorXd out_joint_position(num_dofs_);
    moveit_state.copyJointGroupPositions(joint_model_group_, out_joint_position);

    EXPECT_EQ(joint_positions.size(), out_joint_position.size());
    for (std::size_t i = 0; i < joint_positions.size(); ++i)
    {
      EXPECT_EQ(joint_positions[i], out_joint_position[i]);
    }
  }

  void copyToOMPLStateTest()
  {
    // create a MoveIt state
    Eigen::VectorXd joint_positions = getDeterministicState();
    moveit::core::RobotState moveit_state(robot_model_);
    moveit_state.setToDefaultValues();
    moveit_state.setJointGroupPositions(joint_model_group_, joint_positions);

    // copy into an OMPL state
    ompl::base::ScopedState<> ompl_state(constrained_state_space_);
    moveit_state_space_->copyToOMPLState(ompl_state.get(), moveit_state);

    // check if copy worked out as expected
    Eigen::VectorXd out_joint_position(num_dofs_);
    out_joint_position = *ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>();

    EXPECT_EQ(joint_positions.size(), out_joint_position.size());
    for (std::size_t i = 0; i < joint_positions.size(); ++i)
    {
      EXPECT_EQ(joint_positions[i], out_joint_position[i]);
    }
  }

  void copyJointToOMPLStateTest()
  {
    // create a MoveIt state
    Eigen::VectorXd joint_positions = getDeterministicState();
    moveit::core::RobotState moveit_state(robot_model_);
    moveit_state.setToDefaultValues();
    moveit_state.setJointGroupPositions(joint_model_group_, joint_positions);

    // copy into an OMPL state, one index at a time
    ompl::base::ScopedState<> ompl_state(constrained_state_space_);
    auto joint_model_names = joint_model_group_->getActiveJointModelNames();
    for (int joint_index = 0; joint_index < num_dofs_; ++joint_index)
    {
      const moveit::core::JointModel* joint_model = joint_model_group_->getJointModel(joint_model_names[joint_index]);
      EXPECT_TRUE(joint_model != nullptr);
      moveit_state_space_->copyJointToOMPLState(ompl_state.get(), moveit_state, joint_model, joint_index);
    }

    // check if copy worked out as expected
    Eigen::VectorXd out_joint_position(num_dofs_);
    out_joint_position = *ompl_state->as<ompl::base::ConstrainedStateSpace::StateType>();

    EXPECT_EQ(joint_positions.size(), out_joint_position.size());
    for (std::size_t i = 0; i < joint_positions.size(); ++i)
    {
      EXPECT_EQ(joint_positions[i], out_joint_position[i]);
    }
  }

protected:
  const std::string group_name_;
  const std::string robot_name_;

  moveit::core::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;

  std::size_t num_dofs_;
  std::string base_link_name_;
  std::string ee_link_name_;

  ompl::base::ConstrainedStateSpacePtr constrained_state_space_;
  ompl_interface::ConstrainedPlanningStateSpacePtr moveit_state_space_;
};

/***************************************************************************
 * Run all tests on the Panda robot
 * ************************************************************************/
class PandaCopyStateTest : public LoadTestRobotBaseClass
{
protected:
  PandaCopyStateTest() : LoadTestRobotBaseClass("panda", "panda_arm")
  {
  }
};

TEST_F(PandaCopyStateTest, copyToRobotStateTest)
{
  copyToRobotStateTest();
}

TEST_F(PandaCopyStateTest, copyToOMPLStateTest)
{
  copyToOMPLStateTest();
}

TEST_F(PandaCopyStateTest, copyJointToOMPLStateTest)
{
  copyJointToOMPLStateTest();
}

/***************************************************************************
 * Run all tests on the Fanuc robot
 * ************************************************************************/
class FanucCopyStateTest : public LoadTestRobotBaseClass
{
protected:
  FanucCopyStateTest() : LoadTestRobotBaseClass("fanuc", "manipulator")
  {
  }
};

TEST_F(FanucCopyStateTest, copyToRobotStateTest)
{
  copyToRobotStateTest();
}

TEST_F(FanucCopyStateTest, copyToOMPLStateTest)
{
  copyToOMPLStateTest();
}

TEST_F(FanucCopyStateTest, copyJointToOMPLStateTest)
{
  copyJointToOMPLStateTest();
}

/***************************************************************************
 * Run all tests on the PR2 robot its left arm
 * ************************************************************************/
class PR2CopyStateTest : public LoadTestRobotBaseClass
{
protected:
  PR2CopyStateTest() : LoadTestRobotBaseClass("pr2", "left_arm")
  {
  }
};

TEST_F(PR2CopyStateTest, copyToRobotStateTest)
{
  copyToRobotStateTest();
}

TEST_F(PR2CopyStateTest, copyToOMPLStateTest)
{
  copyToOMPLStateTest();
}

TEST_F(PR2CopyStateTest, copyJointToOMPLStateTest)
{
  copyJointToOMPLStateTest();
}

/***************************************************************************
 * MAIN
 * ************************************************************************/
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
