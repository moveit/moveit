#include "load_test_robot.h"

#include <gtest/gtest.h>
#include "moveit/ompl_interface/mrx_custom/parameterization/joint_pose_model_space.h"

class TestPlanningContext : public ompl_interface_testing::LoadTestRobot, public testing::Test
{
public:
  TestPlanningContext(const std::string& robot_name, const std::string& group_name)
    : LoadTestRobot(robot_name, group_name)
  {
  }
};

class PandaTestPlanningContext : public TestPlanningContext
{
protected:
  PandaTestPlanningContext() : TestPlanningContext("panda", "panda_arm")
  {
  }
};

TEST_F(PandaTestPlanningContext, testInformedBiTRRT)
{
  loadKDLIkSolver();
  ompl_interface::ModelBasedStateSpaceSpecification spec(robot_model_, joint_model_group_);
  auto space = std::make_shared<ompl_interface::JointPoseModelStateSpace>(spec);

  robot_state::RobotState rstate(robot_model_);

  auto start = space->allocState();
  rstate.setToRandomPositions();
  space->copyToOMPLState(start, rstate);

  auto goal = space->allocState();
  rstate.setToRandomPositions();
  space->copyToOMPLState(goal, rstate);

  std::vector<double> start_vec(space->getNumPositions());
  space->copyPositionsToReals(start_vec, start);
  std::vector<double> goal_vec(space->getNumPositions());
  space->copyPositionsToReals(goal_vec, goal);

  ompl_interface::EllipsoidalSampler sampler(space->getNumPositions(), start_vec, goal_vec, space);

  auto state = space->allocState();
  rstate.setToRandomPositions();
  space->copyToOMPLState(state, rstate);

  const double l1 = sampler.getPathLength(start);
  const double l2 = sampler.getPathLength(goal);
  const double l3 = sampler.getPathLength(state);

  EXPECT_NEAR(l1, l2, 1e-4);

  sampler.setTraverseDiameter(l3);

  for (int i = 0; i < 3; i++)
  {
    sampler.sampleUniform(state);
    const double l4 = sampler.getPathLength(state);

    EXPECT_LE(l4, l3);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_joint_pose_model_space");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
