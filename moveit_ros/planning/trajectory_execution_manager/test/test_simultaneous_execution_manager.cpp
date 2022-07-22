/* Author: Cristian C. Beltran-Hernandez
   Desc: Test the MoveItCpp and Planning Component interfaces
*/

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// Main class
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
// Msgs
#include <geometry_msgs/PointStamped.h>

namespace moveit_cpp
{
class MoveItCppTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    nh_ = ros::NodeHandle("/test_moveit_cpp");

    moveit_cpp_ptr = std::make_shared<MoveItCpp>(nh_);

    robot_model_ptr = moveit_cpp_ptr->getRobotModel();

    panda_1_planning_component_ptr = std::make_shared<PlanningComponent>(PANDA_1_PLANNING_GROUP, moveit_cpp_ptr);
    panda_1_jmg_ptr = robot_model_ptr->getJointModelGroup(PANDA_1_PLANNING_GROUP);
    panda_1_planning_component_ptr->setStartStateToCurrentState();

    panda_2_planning_component_ptr = std::make_shared<PlanningComponent>(PANDA_2_PLANNING_GROUP, moveit_cpp_ptr);
    panda_2_jmg_ptr = robot_model_ptr->getJointModelGroup(PANDA_2_PLANNING_GROUP);

    panda_1_target_pose1.header.frame_id = "base";
    panda_1_target_pose1.pose.position.x = 0.450;
    panda_1_target_pose1.pose.position.y = -0.50;
    panda_1_target_pose1.pose.position.z = 1.50;
    panda_1_target_pose1.pose.orientation.x = 0.993436;
    panda_1_target_pose1.pose.orientation.y = 3.5161e-05;
    panda_1_target_pose1.pose.orientation.z = 0.114386;
    panda_1_target_pose1.pose.orientation.w = 2.77577e-05;

    panda_1_target_pose2.header.frame_id = "base";
    panda_1_target_pose2.pose.position.x = 0.40;
    panda_1_target_pose2.pose.position.y = -0.45;
    panda_1_target_pose2.pose.position.z = 1.40;
    panda_1_target_pose2.pose.orientation.x = 0.993436;
    panda_1_target_pose2.pose.orientation.y = 3.5161e-05;
    panda_1_target_pose2.pose.orientation.z = 0.114386;
    panda_1_target_pose2.pose.orientation.w = 2.77577e-05;

    panda_2_target_pose1.header.frame_id = "base";
    panda_2_target_pose1.pose.position.x = 0.450;
    panda_2_target_pose1.pose.position.y = 0.40;
    panda_2_target_pose1.pose.position.z = 1.600;
    panda_2_target_pose1.pose.orientation.x = 0.993434;
    panda_2_target_pose1.pose.orientation.y = -7.54803e-06;
    panda_2_target_pose1.pose.orientation.z = 0.114403;
    panda_2_target_pose1.pose.orientation.w = 3.67256e-05;

    panda_2_target_pose2.header.frame_id = "base";
    panda_2_target_pose2.pose.position.x = 0.40;
    panda_2_target_pose2.pose.position.y = 0.45;
    panda_2_target_pose2.pose.position.z = 1.500;
    panda_2_target_pose2.pose.orientation.x = 0.993434;
    panda_2_target_pose2.pose.orientation.y = -7.54803e-06;
    panda_2_target_pose2.pose.orientation.z = 0.114403;
    panda_2_target_pose2.pose.orientation.w = 3.67256e-05;
  }

  void plan(const PlanningComponentPtr& planning_component, const geometry_msgs::PoseStamped& target_pose,
            const std::string& end_effector_link)
  {
    planning_component->setStartStateToCurrentState();
    planning_component->setGoal(target_pose, end_effector_link);
    planning_component->plan();
  }

  void plan(const PlanningComponentPtr& planning_component, const geometry_msgs::PoseStamped& target_pose,
            const std::string& end_effector_link, const moveit::core::RobotState& start_state)
  {
    planning_component->setStartState(start_state);
    planning_component->setGoal(target_pose, end_effector_link);
    planning_component->plan();
  }

  void plan(const PlanningComponentPtr& planning_component, const geometry_msgs::PoseStamped& target_pose,
            const std::string& end_effector_link, const geometry_msgs::Pose& start_pose,
            const moveit::core::JointModelGroup* jmg_ptr)
  {
    auto start_state = *(moveit_cpp_ptr->getCurrentState());
    start_state.setFromIK(jmg_ptr, start_pose);
    plan(planning_component, target_pose, end_effector_link, start_state);
  }

protected:
  ros::NodeHandle nh_;
  MoveItCppPtr moveit_cpp_ptr;
  moveit::core::RobotModelConstPtr robot_model_ptr;

  PlanningComponentPtr panda_1_planning_component_ptr;
  const moveit::core::JointModelGroup* panda_1_jmg_ptr;
  const std::string PANDA_1_PLANNING_GROUP = "panda_1";
  geometry_msgs::PoseStamped panda_1_target_pose1;
  geometry_msgs::PoseStamped panda_1_target_pose2;
  geometry_msgs::Pose panda_1_start_pose;

  PlanningComponentPtr panda_2_planning_component_ptr;
  const moveit::core::JointModelGroup* panda_2_jmg_ptr;
  const std::string PANDA_2_PLANNING_GROUP = "panda_2";
  geometry_msgs::PoseStamped panda_2_target_pose1;
  geometry_msgs::PoseStamped panda_2_target_pose2;
  geometry_msgs::Pose panda_2_start_pose;
};

// Test the name of the planning group used by PlanningComponent for the Panda robot
TEST_F(MoveItCppTest, ExecutionWithBlockingTest)
{
  plan(panda_1_planning_component_ptr, panda_1_target_pose1, "panda_1_link8");
  plan(panda_2_planning_component_ptr, panda_2_target_pose1, "panda_2_link8");

  auto panda_1_robot_trajectory = panda_1_planning_component_ptr->getLastPlanSolution()->trajectory;
  moveit_msgs::RobotTrajectory panda_1_robot_trajectory_msg;
  panda_1_robot_trajectory->getRobotTrajectoryMsg(panda_1_robot_trajectory_msg);

  auto panda_2_robot_trajectory = panda_2_planning_component_ptr->getLastPlanSolution()->trajectory;
  moveit_msgs::RobotTrajectory panda_2_robot_trajectory_msg;
  panda_2_robot_trajectory->getRobotTrajectoryMsg(panda_2_robot_trajectory_msg);

  moveit_cpp_ptr->getTrajectoryExecutionManager()->push(panda_1_robot_trajectory_msg);
  moveit_cpp_ptr->getTrajectoryExecutionManager()->execute();

  bool res = moveit_cpp_ptr->getTrajectoryExecutionManager()->push(panda_2_robot_trajectory_msg);

  // Expect FALSE as the second trajectory is rejected by the TEM
  ASSERT_TRUE(!res);
}

TEST_F(MoveItCppTest, SimpleSimulatenousExecutionTest)
{
  plan(panda_1_planning_component_ptr, panda_1_target_pose1, "panda_1_link8");
  plan(panda_2_planning_component_ptr, panda_2_target_pose1, "panda_2_link8");

  auto panda_1_robot_trajectory = panda_1_planning_component_ptr->getLastPlanSolution()->trajectory;
  moveit_msgs::RobotTrajectory panda_1_robot_trajectory_msg;
  panda_1_robot_trajectory->getRobotTrajectoryMsg(panda_1_robot_trajectory_msg);

  auto panda_2_robot_trajectory = panda_2_planning_component_ptr->getLastPlanSolution()->trajectory;
  moveit_msgs::RobotTrajectory panda_2_robot_trajectory_msg;
  panda_2_robot_trajectory->getRobotTrajectoryMsg(panda_2_robot_trajectory_msg);

  bool res1 =
      moveit_cpp_ptr->getTrajectoryExecutionManager()->pushAndExecuteSimultaneously(panda_1_robot_trajectory_msg);
  bool res2 =
      moveit_cpp_ptr->getTrajectoryExecutionManager()->pushAndExecuteSimultaneously(panda_2_robot_trajectory_msg);

  moveit_cpp_ptr->getTrajectoryExecutionManager()->waitForExecution();
  // Expect TRUE as both trajectories should be accepted
  ASSERT_TRUE(res1 && res2);
}

TEST_F(MoveItCppTest, MultipleSimulatenousExecutionTest)
{
  moveit_msgs::RobotTrajectory panda_1_robot_trajectory_msg;
  moveit_msgs::RobotTrajectory panda_2_robot_trajectory_msg;
  moveit_msgs::RobotTrajectory panda_3_robot_trajectory_msg;
  moveit_msgs::RobotTrajectory panda_4_robot_trajectory_msg;

  plan(panda_1_planning_component_ptr, panda_1_target_pose1, "panda_1_link8");
  plan(panda_2_planning_component_ptr, panda_2_target_pose1, "panda_2_link8");

  auto panda_1_robot_trajectory = panda_1_planning_component_ptr->getLastPlanSolution()->trajectory;
  panda_1_robot_trajectory->getRobotTrajectoryMsg(panda_1_robot_trajectory_msg);

  auto panda_2_robot_trajectory = panda_2_planning_component_ptr->getLastPlanSolution()->trajectory;
  panda_2_robot_trajectory->getRobotTrajectoryMsg(panda_2_robot_trajectory_msg);

  moveit::core::RobotState panda_1_start_state2 = *panda_1_robot_trajectory->getLastWayPointPtr();
  moveit::core::RobotState panda_2_start_state2 = *panda_2_robot_trajectory->getLastWayPointPtr();
  plan(panda_1_planning_component_ptr, panda_1_target_pose2, "panda_1_link8", panda_1_start_state2);
  plan(panda_2_planning_component_ptr, panda_2_target_pose2, "panda_2_link8", panda_2_start_state2);

  panda_1_robot_trajectory = panda_1_planning_component_ptr->getLastPlanSolution()->trajectory;
  panda_1_robot_trajectory->getRobotTrajectoryMsg(panda_3_robot_trajectory_msg);

  panda_2_robot_trajectory = panda_2_planning_component_ptr->getLastPlanSolution()->trajectory;
  panda_2_robot_trajectory->getRobotTrajectoryMsg(panda_4_robot_trajectory_msg);

  bool res1 =
      moveit_cpp_ptr->getTrajectoryExecutionManager()->pushAndExecuteSimultaneously(panda_1_robot_trajectory_msg);
  bool res2 =
      moveit_cpp_ptr->getTrajectoryExecutionManager()->pushAndExecuteSimultaneously(panda_2_robot_trajectory_msg);
  bool res3 =
      moveit_cpp_ptr->getTrajectoryExecutionManager()->pushAndExecuteSimultaneously(panda_3_robot_trajectory_msg);
  bool res4 =
      moveit_cpp_ptr->getTrajectoryExecutionManager()->pushAndExecuteSimultaneously(panda_4_robot_trajectory_msg);

  moveit_cpp_ptr->getTrajectoryExecutionManager()->waitForExecution();
  // Expect TRUE as both trajectories should be accepted
  ASSERT_TRUE(res1 && res2 && res3 && res4);
}

// TODO:
// - Multiple trajectories in series A-B-B-B-A-B (Check that order is being respected)
// - Deadlock 1: A-B, B is in collision with A, after A is completed, B is still invalid
// - Deadlock 2:

}  // namespace moveit_cpp

int main(int argc, char** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_moveit_cpp");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  int result = RUN_ALL_TESTS();

  return result;
}
