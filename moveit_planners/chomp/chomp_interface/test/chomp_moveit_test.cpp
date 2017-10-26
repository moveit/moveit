
/// \author Bence Magyar

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class CHOMPMoveitTest : public ::testing::Test
{
public:
  moveit::planning_interface::MoveGroupInterface move_group;

public:
  CHOMPMoveitTest() : move_group(moveit::planning_interface::MoveGroupInterface("arm"))
  {
  }
};

// TEST CASES
TEST_F(CHOMPMoveitTest, jointSpaceGoodGoal)
{
  move_group.setStartState(*(move_group.getCurrentState()));
  move_group.setJointValueTarget(std::vector<double>({ 1.0, 1.0 }));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  moveit::planning_interface::MoveItErrorCode error_code = move_group.plan(my_plan);
  EXPECT_GT(my_plan.trajectory_.joint_trajectory.points.size(), 0);
  EXPECT_EQ(error_code.val, moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

TEST_F(CHOMPMoveitTest, jointSpaceBadGoal)
{
  move_group.setStartState(*(move_group.getCurrentState()));
  // joint2 is limited to [-PI/2, PI/2]
  move_group.setJointValueTarget(std::vector<double>({ 100.0, 2 * M_PI / 3.0 }));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  moveit::planning_interface::MoveItErrorCode error_code = move_group.plan(my_plan);
  EXPECT_EQ(error_code.val, moveit::planning_interface::MoveItErrorCode::INVALID_ROBOT_STATE);
}

TEST_F(CHOMPMoveitTest, cartesianGoal)
{
  move_group.setStartState(*(move_group.getCurrentState()));
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 10000.;
  target_pose1.position.y = 10000.;
  target_pose1.position.z = 10000.;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  moveit::planning_interface::MoveItErrorCode error_code = move_group.plan(my_plan);
  // CHOMP doesn't support Cartesian-space goals at the moment
  EXPECT_EQ(error_code.val, moveit::planning_interface::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS);
}

TEST_F(CHOMPMoveitTest, noStartState)
{
  move_group.setJointValueTarget(std::vector<double>({ 0.2, 0.2 }));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  moveit::planning_interface::MoveItErrorCode error_code = move_group.plan(my_plan);
  EXPECT_EQ(error_code.val, moveit::planning_interface::MoveItErrorCode::INVALID_ROBOT_STATE);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "chomp_moveit_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
