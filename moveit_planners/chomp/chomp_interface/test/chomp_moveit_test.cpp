
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
  CHOMPMoveitTest()
    : move_group(moveit::planning_interface::MoveGroupInterface("arm"))
  {

  }
};

// TEST CASES
TEST_F(CHOMPMoveitTest, jointSpaceGoodPlan)
{
  move_group.setJointValueTarget(std::vector<double>({1.0,1.0}));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = move_group.plan(my_plan);
  EXPECT_TRUE(success);
  EXPECT_GT(my_plan.trajectory_.joint_trajectory.points.size(), 0);
}

TEST_F(CHOMPMoveitTest, cartesianPlan)
{
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 10000.;
  target_pose1.position.y = 10000.;
  target_pose1.position.z = 10000.;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = move_group.plan(my_plan);
  // CHOMP doesn't support Cartesian-space goals at the moment
  EXPECT_FALSE(success);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "chomp_moveit_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
