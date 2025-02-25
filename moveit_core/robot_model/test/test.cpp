/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <moveit/robot_model/robot_model.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>
#include <moveit/profiler/profiler.h>
#include <moveit/utils/robot_model_test_utils.h>

constexpr double NEG_INF = -std::numeric_limits<double>::infinity();
constexpr double POS_INF = std::numeric_limits<double>::infinity();

void updateBounds(moveit::core::JointModel::Bounds& bounds,  //
                  std::size_t bound_index,                   //
                  double min_position,                       //
                  double max_position,                       //
                  double min_velocity,                       //
                  double max_velocity,                       //
                  double min_acceleration,                   //
                  double max_acceleration,                   //
                  bool position_bounded,                     //
                  bool velocity_bounded,                     //
                  bool acceleration_bounded)                 //
{
  ASSERT_TRUE(bound_index < bounds.size());

  bounds[bound_index].min_position_ = min_position;
  bounds[bound_index].max_position_ = max_position;
  bounds[bound_index].min_velocity_ = min_velocity;
  bounds[bound_index].max_velocity_ = max_velocity;
  bounds[bound_index].min_acceleration_ = min_acceleration;
  bounds[bound_index].max_acceleration_ = max_acceleration;

  bounds[bound_index].position_bounded_ = position_bounded;
  bounds[bound_index].velocity_bounded_ = velocity_bounded;
  bounds[bound_index].acceleration_bounded_ = acceleration_bounded;
}

struct SatisfiesPositionBounds : testing::Test
{
};

class LoadPlanningModelsPr2 : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("pr2");
  };

  void TearDown() override
  {
  }

protected:
  moveit::core::RobotModelConstPtr robot_model_;
};

TEST_F(LoadPlanningModelsPr2, InitOK)
{
  ASSERT_EQ(robot_model_->getURDF()->getName(), "pr2");
  ASSERT_EQ(robot_model_->getSRDF()->getName(), "pr2");
}

TEST_F(LoadPlanningModelsPr2, Model)
{
  // robot_model_->printModelInfo(std::cout);

  const std::vector<const moveit::core::JointModel*>& joints = robot_model_->getJointModels();
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    ASSERT_EQ(joints[i]->getJointIndex(), static_cast<int>(i));
    ASSERT_EQ(robot_model_->getJointModel(joints[i]->getName()), joints[i]);
  }
  const std::vector<const moveit::core::LinkModel*>& links = robot_model_->getLinkModels();
  for (std::size_t i = 0; i < links.size(); ++i)
  {
    ASSERT_EQ(links[i]->getLinkIndex(), static_cast<int>(i));
  }
  moveit::tools::Profiler::Status();
}

void generateMotionBoundsTests(const moveit::core::JointModel& joint_model,
                               const moveit::core::JointModel::Bounds& bounds)
{
  unsigned int dimensions = joint_model.getStateSpaceDimension();

  static const std::size_t MOTION_TESTS = 6;
  std::array<std::vector<double>, MOTION_TESTS> position_2d;
  std::array<std::vector<double>, MOTION_TESTS> velocity_2d;
  std::array<std::vector<double>, MOTION_TESTS> acceleration_2d;

  for (std::size_t i = 0; i < position_2d.size(); ++i)
  {
    // initialize positions with default
    position_2d[i].resize(dimensions);
    joint_model.getVariableDefaultPositions(&position_2d[i][0], bounds);

    // no need to initialize velocities or acceleration
    velocity_2d[i].resize(dimensions);
    acceleration_2d[i].resize(dimensions);
  }

  // fill motion values
  bool compute_inner = true;
  bool inf_min_bounded = false;
  bool inf_max_bounded = false;
  for (std::size_t i = 0; i < bounds.size(); ++i)
  {
    if (isinf(bounds[i].min_position_))
      inf_min_bounded = true;
    if (isinf(bounds[i].max_position_))
      inf_max_bounded = true;

    if (bounds[i].position_bounded_)
    {
      if (std::abs(bounds[i].max_position_ - bounds[i].min_position_) < 2.0)
        compute_inner = false;

      position_2d[0][i] = bounds[i].min_position_ - 1.0;
      position_2d[1][i] = bounds[i].min_position_;
      position_2d[2][i] = bounds[i].min_position_ + 1.0;
      position_2d[3][i] = bounds[i].max_position_ - 1.0;
      position_2d[4][i] = bounds[i].max_position_;
      position_2d[5][i] = bounds[i].max_position_ + 1.0;
    }
    if (bounds[i].velocity_bounded_)
    {
      velocity_2d[0][i] = bounds[i].min_velocity_ - 1.0;
      velocity_2d[1][i] = bounds[i].min_velocity_;
      velocity_2d[2][i] = bounds[i].min_velocity_ + 1.0;
      velocity_2d[3][i] = bounds[i].max_velocity_ - 1.0;
      velocity_2d[4][i] = bounds[i].max_velocity_;
      velocity_2d[5][i] = bounds[i].max_velocity_ + 1.0;
    }
    if (bounds[i].acceleration_bounded_)
    {
      acceleration_2d[0][i] = bounds[i].min_acceleration_ - 1.0;
      acceleration_2d[1][i] = bounds[i].min_acceleration_;
      acceleration_2d[2][i] = bounds[i].min_acceleration_ + 1.0;
      acceleration_2d[3][i] = bounds[i].max_acceleration_ - 1.0;
      acceleration_2d[4][i] = bounds[i].max_acceleration_;
      acceleration_2d[5][i] = bounds[i].max_acceleration_ + 1.0;
    }
  }

  //      X          |----------|-------------|----------|          |
  //  (min - 1)     min     (min + 1)     (max - 1)     max     (max + 1)
  if (inf_min_bounded)
  {
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[0][0], bounds, 0));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[0][0], bounds, 0.9));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[0][0], bounds, 1));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[0][0], bounds, 1.1));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[0][0], bounds, -0.9));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[0][0], bounds, -1));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[0][0], bounds, -1.1));

  }
  else
  {
    ASSERT_FALSE(joint_model.satisfiesPositionBounds(&position_2d[0][0], bounds, 0));
    ASSERT_FALSE(joint_model.satisfiesPositionBounds(&position_2d[0][0], bounds, 0.9));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[0][0], bounds, 1));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[0][0], bounds, 1.1));

    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[0][0], bounds, -0.9));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[0][0], bounds, -1));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[0][0], bounds, -1.1));
  }

  //      |          X----------|-------------|----------|          |
  //  (min - 1)     min     (min + 1)     (max - 1)     max     (max + 1)
  ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[1][0], bounds, 0));
  ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[1][0], bounds, 0.9));
  ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[1][0], bounds, 1));
  ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[1][0], bounds, 1.1));

  ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[1][0], bounds, -0.9));
  ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[1][0], bounds, -1));
  ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[1][0], bounds, -1.1));

  //      |          |----------X-------------|----------|          |
  //  (min - 1)     min     (min + 1)     (max - 1)     max     (max + 1)
  if (compute_inner)
  {
    std::cout << "compute inner" << std::endl;
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[2][0], bounds, 0));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[2][0], bounds, 0.9));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[2][0], bounds, 1));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[2][0], bounds, 1.1));

    if (inf_min_bounded)
    {
      ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[2][0], bounds, -0.9));
      ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[2][0], bounds, -1));
      ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[2][0], bounds, -1.1));
    }
    else
    {
      ASSERT_FALSE(joint_model.satisfiesPositionBounds(&position_2d[2][0], bounds, -0.9));
      ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[2][0], bounds, -1));
      ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[2][0], bounds, -1.1));
    }
  }

  //      |          |----------|-------------X----------|          |
  //  (min - 1)     min     (min + 1)     (max - 1)     max     (max + 1)
  if (compute_inner)
  {
    std::cout << "compute inner" << std::endl;
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[3][0], bounds, 0));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[3][0], bounds, 0.9));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[3][0], bounds, 1));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[3][0], bounds, 1.1));

    if (inf_max_bounded)
    {
      ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[3][0], bounds, -0.9));
      ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[3][0], bounds, -1));
      ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[3][0], bounds, -1.1));
    }
    else
    {
      ASSERT_FALSE(joint_model.satisfiesPositionBounds(&position_2d[3][0], bounds, -0.9));
      ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[3][0], bounds, -1));
      ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[3][0], bounds, -1.1));
    }
  }

  //      |          |----------|-------------|----------X          |
  //  (min - 1)     min     (min + 1)     (max - 1)     max     (max + 1)
  ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[4][0], bounds, 0));
  ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[4][0], bounds, 0.9));
  ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[4][0], bounds, 1));
  ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[4][0], bounds, 1.1));

  ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[4][0], bounds, -0.9));
  ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[4][0], bounds, -1));
  ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[4][0], bounds, -1.1));

  //      |          |----------|-------------|----------|          X
  //  (min - 1)     min     (min + 1)     (max - 1)     max     (max + 1)
  if (inf_max_bounded)
  {
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[5][0], bounds, 0));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[5][0], bounds, 0.9));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[5][0], bounds, 1));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[5][0], bounds, 1.1));

    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[5][0], bounds, -0.9));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[5][0], bounds, -1));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[5][0], bounds, -1.1));
  }
  else
  {
    ASSERT_FALSE(joint_model.satisfiesPositionBounds(&position_2d[5][0], bounds, 0));
    ASSERT_FALSE(joint_model.satisfiesPositionBounds(&position_2d[5][0], bounds, 0.9));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[5][0], bounds, 1));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[5][0], bounds, 1.1));

    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[5][0], bounds, -0.9));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[5][0], bounds, -1));
    ASSERT_TRUE(joint_model.satisfiesPositionBounds(&position_2d[5][0], bounds, -1.1));
  }
}

TEST(SatisfiesPositionBounds, RevoluteJoint)
{
  moveit::core::RevoluteJointModel joint_model("revolute");
  auto bounds = joint_model.getVariableBounds();
  unsigned int dimensions = joint_model.getStateSpaceDimension();

  std::vector<double> values;
  values.resize(dimensions);
  joint_model.getVariableDefaultPositions(&values[0], bounds);

  {
    updateBounds(bounds, 0, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, NEG_INF, POS_INF, NEG_INF, POS_INF, NEG_INF, POS_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, NEG_INF, 0, NEG_INF, 0, NEG_INF, 0, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, 0, POS_INF, 0, POS_INF, 0, POS_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, -4, -4, -4, -4, -4, -4, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, -4, 0, -4, 0, -4, 0, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, -2, 2, -2, 2, -2, 2, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, 0, 4, 0, 4, 0, 4, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);
  }
}

TEST(SatisfiesPositionBounds, PrismaticJoint)
{
  moveit::core::PrismaticJointModel joint_model("prismatic");
  auto bounds = joint_model.getVariableBounds();
  unsigned int dimensions = joint_model.getStateSpaceDimension();

  std::vector<double> values;
  values.resize(dimensions);
  joint_model.getVariableDefaultPositions(&values[0], bounds);

  {
    updateBounds(bounds, 0, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, NEG_INF, POS_INF, NEG_INF, POS_INF, NEG_INF, POS_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, NEG_INF, 0, NEG_INF, 0, NEG_INF, 0, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, 0, POS_INF, 0, POS_INF, 0, POS_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, -4, -4, -4, -4, -4, -4, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, -4, 0, -4, 0, -4, 0, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, -2, 2, -2, 2, -2, 2, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, 0, 4, 0, 4, 0, 4, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);
  }
}

TEST(SatisfiesPositionBounds, PlanarJoint)
{
  moveit::core::PlanarJointModel joint_model("planar");
  auto bounds = joint_model.getVariableBounds();
  unsigned int dimensions = joint_model.getStateSpaceDimension();

  std::vector<double> values;
  values.resize(dimensions);
  joint_model.getVariableDefaultPositions(&values[0], bounds);

  {
    updateBounds(bounds, 0, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, true, true, true);
    updateBounds(bounds, 1, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, true, true, true);
    updateBounds(bounds, 2, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, NEG_INF, POS_INF, NEG_INF, POS_INF, NEG_INF, POS_INF, true, true, true);
    updateBounds(bounds, 1, NEG_INF, POS_INF, NEG_INF, POS_INF, NEG_INF, POS_INF, true, true, true);
    updateBounds(bounds, 2, NEG_INF, POS_INF, NEG_INF, POS_INF, NEG_INF, POS_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, NEG_INF, 0, NEG_INF, 0, NEG_INF, 0, true, true, true);
    updateBounds(bounds, 1, NEG_INF, 0, NEG_INF, 0, NEG_INF, 0, true, true, true);
    updateBounds(bounds, 2, NEG_INF, 0, NEG_INF, 0, NEG_INF, 0, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, 0, POS_INF, 0, POS_INF, 0, POS_INF, true, true, true);
    updateBounds(bounds, 1, 0, POS_INF, 0, POS_INF, 0, POS_INF, true, true, true);
    updateBounds(bounds, 2, 0, POS_INF, 0, POS_INF, 0, POS_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, true, true, true);
    updateBounds(bounds, 1, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, true, true, true);
    updateBounds(bounds, 2, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, -4, -4, -4, -4, -4, -4, true, true, true);
    updateBounds(bounds, 1, -4, -4, -4, -4, -4, -4, true, true, true);
    updateBounds(bounds, 2, -4, -4, -4, -4, -4, -4, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, -4, 0, -4, 0, -4, 0, true, true, true);
    updateBounds(bounds, 1, -4, 0, -4, 0, -4, 0, true, true, true);
    updateBounds(bounds, 2, -4, 0, -4, 0, -4, 0, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, -2, 2, -2, 2, -2, 2, true, true, true);
    updateBounds(bounds, 1, -2, 2, -2, 2, -2, 2, true, true, true);
    updateBounds(bounds, 2, -2, 2, -2, 2, -2, 2, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, 0, 4, 0, 4, 0, 4, true, true, true);
    updateBounds(bounds, 1, 0, 4, 0, 4, 0, 4, true, true, true);
    updateBounds(bounds, 2, 0, 4, 0, 4, 0, 4, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);
  }
}

TEST(SatisfiesPositionBounds, FloatingJoint)
{
  moveit::core::FloatingJointModel joint_model("floating");
  auto bounds = joint_model.getVariableBounds();
  unsigned int dimensions = joint_model.getStateSpaceDimension();

  std::vector<double> values;
  values.resize(dimensions);
  joint_model.getVariableDefaultPositions(&values[0], bounds);

  {
    // do not update values for: rot_x, rot_y, rot_z, rot_w
    bounds[3].position_bounded_ = false;
    bounds[4].position_bounded_ = false;
    bounds[5].position_bounded_ = false;
    bounds[6].position_bounded_ = false;

    updateBounds(bounds, 0, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, true, true, true);
    updateBounds(bounds, 1, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, true, true, true);
    updateBounds(bounds, 2, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, NEG_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, NEG_INF, POS_INF, NEG_INF, POS_INF, NEG_INF, POS_INF, true, true, true);
    updateBounds(bounds, 1, NEG_INF, POS_INF, NEG_INF, POS_INF, NEG_INF, POS_INF, true, true, true);
    updateBounds(bounds, 2, NEG_INF, POS_INF, NEG_INF, POS_INF, NEG_INF, POS_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, NEG_INF, 0, NEG_INF, 0, NEG_INF, 0, true, true, true);
    updateBounds(bounds, 1, NEG_INF, 0, NEG_INF, 0, NEG_INF, 0, true, true, true);
    updateBounds(bounds, 2, NEG_INF, 0, NEG_INF, 0, NEG_INF, 0, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, 0, POS_INF, 0, POS_INF, 0, POS_INF, true, true, true);
    updateBounds(bounds, 1, 0, POS_INF, 0, POS_INF, 0, POS_INF, true, true, true);
    updateBounds(bounds, 2, 0, POS_INF, 0, POS_INF, 0, POS_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, true, true, true);
    updateBounds(bounds, 1, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, true, true, true);
    updateBounds(bounds, 2, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, POS_INF, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, -4, -4, -4, -4, -4, -4, true, true, true);
    updateBounds(bounds, 1, -4, -4, -4, -4, -4, -4, true, true, true);
    updateBounds(bounds, 2, -4, -4, -4, -4, -4, -4, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, -4, 0, -4, 0, -4, 0, true, true, true);
    updateBounds(bounds, 1, -4, 0, -4, 0, -4, 0, true, true, true);
    updateBounds(bounds, 2, -4, 0, -4, 0, -4, 0, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, -2, 2, -2, 2, -2, 2, true, true, true);
    updateBounds(bounds, 1, -2, 2, -2, 2, -2, 2, true, true, true);
    updateBounds(bounds, 2, -2, 2, -2, 2, -2, 2, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);

    updateBounds(bounds, 0, 0, 4, 0, 4, 0, 4, true, true, true);
    updateBounds(bounds, 1, 0, 4, 0, 4, 0, 4, true, true, true);
    updateBounds(bounds, 2, 0, 4, 0, 4, 0, 4, true, true, true);
    generateMotionBoundsTests(joint_model, bounds);
  }
}

TEST(SiblingAssociateLinks, SimpleYRobot)
{
  /* base_link - a - b - c
                  \
                   - d ~ e          */
  moveit::core::RobotModelBuilder builder("one_robot", "base_link");
  builder.addChain("base_link->a", "continuous");
  builder.addChain("a->b->c", "fixed");
  builder.addChain("a->d", "fixed");
  builder.addChain("d->e", "continuous");
  builder.addVirtualJoint("odom", "base_link", "planar", "base_joint");
  builder.addGroup({}, { "base_joint" }, "base_joint");
  ASSERT_TRUE(builder.isValid());
  moveit::core::RobotModelConstPtr robot_model = builder.build();

  const std::string a = "a", b = "b", c = "c", d = "d";
  auto connected = { a, b, c, d };  // these are rigidly connected with each other
  moveit::core::LinkTransformMap map;

  for (const std::string& root : connected)
  {
    SCOPED_TRACE("root: " + root);
    std::set<std::string> expected_set(connected);
    expected_set.erase(root);
    std::set<std::string> actual_set;
    for (const auto& item : robot_model->getLinkModel(root)->getAssociatedFixedTransforms())
      actual_set.insert(item.first->getName());

    std::ostringstream expected, actual;
    std::copy(expected_set.begin(), expected_set.end(), std::ostream_iterator<std::string>(expected, " "));
    std::copy(actual_set.begin(), actual_set.end(), std::ostream_iterator<std::string>(actual, " "));

    EXPECT_EQ(expected.str(), actual.str());
  }
}

TEST(RobotModel, CycleDetection)
{
  static const std::string URDF = R"(<?xml version="1.0"?>
  <robot name="test">
    <link name="base"/>
    <link name="a"/>
    <link name="b"/>
    <joint name="base_a" type="fixed">
      <parent link="base"/>
      <child link="a"/>
    </joint>
    <joint name="base_b" type="continuous">
      <parent link="base"/>
      <child link="b"/>
    </joint>
    <joint name="a_b" type="continuous">
      <parent link="a"/>
      <child link="b"/>
    </joint>
  </robot>)";

  auto urdf = std::make_shared<urdf::Model>();
  // urdfdom will initialize the model, but mark one joint as "parent_joint" of "b"
  ASSERT_TRUE(urdf->initString(URDF));
  auto srdf = std::make_shared<srdf::Model>();
  moveit::core::RobotModel robot_model(urdf, srdf);

  // MoveIt ignores the second joint with child b
  EXPECT_EQ(robot_model.getActiveJointModels().size(), 1u);  // base_b?
  EXPECT_EQ(robot_model.getLinkModelCount(), 3u);            // base, a, b
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
