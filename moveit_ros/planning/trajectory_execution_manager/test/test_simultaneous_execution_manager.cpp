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
//
#include <condition_variable>
namespace moveit_cpp
{
class MoveItCppTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    nh_ = ros::NodeHandle("test_simultaneous_execution_manager");

    moveit_cpp_ptr = std::make_shared<MoveItCpp>(nh_);

    robot_model_ptr = moveit_cpp_ptr->getRobotModel();

    panda_1_planning_component_ptr = std::make_shared<PlanningComponent>(PANDA_1_PLANNING_GROUP, moveit_cpp_ptr);
    panda_1_jmg_ptr = robot_model_ptr->getJointModelGroup(PANDA_1_PLANNING_GROUP);
    panda_1_joint_names = panda_1_jmg_ptr->getJointModelNames();
    panda_1_joint_names.erase(panda_1_joint_names.end() - 1);

    panda_2_planning_component_ptr = std::make_shared<PlanningComponent>(PANDA_2_PLANNING_GROUP, moveit_cpp_ptr);
    panda_2_jmg_ptr = robot_model_ptr->getJointModelGroup(PANDA_2_PLANNING_GROUP);
    panda_2_joint_names = panda_2_jmg_ptr->getJointModelNames();
    panda_2_joint_names.erase(panda_2_joint_names.end() - 1);
  }

  moveit::core::RobotState createGoal(moveit::core::RobotState& start_state, std::vector<std::string>& joint_names,
                                      std::vector<double>& joint_positions)
  {
    moveit::core::RobotState rs = moveit::core::RobotState(start_state);
    trajectory_msgs::JointTrajectory jt;
    jt.joint_names = joint_names;
    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.positions = joint_positions;
    jt.points.push_back(jtp);
    jointTrajPointToRobotState(jt, 0, rs);
    return rs;
  }

  moveit_msgs::RobotTrajectory plan(const PlanningComponentPtr& planning_component,
                                    std::vector<std::string>& joint_names, std::vector<double>& target_joints)
  {
    planning_component->setStartStateToCurrentState();
    auto target = createGoal(*planning_component->getStartState(), joint_names, target_joints);
    planning_component->setGoal(target);
    auto plan = planning_component->plan();

    auto robot_trajectory = planning_component->getLastPlanSolution()->trajectory;
    moveit_msgs::RobotTrajectory robot_trajectory_msg;
    robot_trajectory->getRobotTrajectoryMsg(robot_trajectory_msg);
    return robot_trajectory_msg;
  }

protected:
  ros::NodeHandle nh_;
  MoveItCppPtr moveit_cpp_ptr;
  moveit::core::RobotModelConstPtr robot_model_ptr;
  std::mutex execution_complete_mutex_;
  std::condition_variable execution_complete_condition_;

  PlanningComponentPtr panda_1_planning_component_ptr;
  const moveit::core::JointModelGroup* panda_1_jmg_ptr;
  const std::string PANDA_1_PLANNING_GROUP = "panda_1";
  std::vector<std::string> panda_1_joint_names;

  PlanningComponentPtr panda_2_planning_component_ptr;
  const moveit::core::JointModelGroup* panda_2_jmg_ptr;
  const std::string PANDA_2_PLANNING_GROUP = "panda_2";
  std::vector<std::string> panda_2_joint_names;
};

TEST_F(MoveItCppTest, SimpleSimulatenousExecutionTest)
{
  std::vector<double> p1_target_joints{ 1.057, -0.323, 0.805, -2.857, 0.424, 2.557, 1.468 };
  std::vector<double> p2_target_joints{ 2.049, 0.046, 2.419, -2.660, -0.053, 2.624, -1.666 };

  moveit_msgs::RobotTrajectory panda_1_robot_trajectory_msg =
      plan(panda_1_planning_component_ptr, panda_1_joint_names, p1_target_joints);
  moveit_msgs::RobotTrajectory panda_2_robot_trajectory_msg =
      plan(panda_2_planning_component_ptr, panda_2_joint_names, p2_target_joints);

  // Both trajectories should be pushed and executed successfully
  ASSERT_TRUE(moveit_cpp_ptr->getTrajectoryExecutionManager()->push(panda_1_robot_trajectory_msg));
  ASSERT_TRUE(moveit_cpp_ptr->getTrajectoryExecutionManager()->push(panda_2_robot_trajectory_msg));

  // Wait for every execution to complete
  ASSERT_TRUE(moveit_cpp_ptr->getTrajectoryExecutionManager()->waitForExecution());
}

TEST_F(MoveItCppTest, WaitForSingleTrajectory)
{
  std::vector<double> p1_target_joints{ 0, 0, 0, -1.5707, 0, 1.8, 0 };
  std::vector<double> p2_target_joints{ 0, 0, 0, -1.5707, 0, 1.8, 0 };

  moveit_msgs::RobotTrajectory panda_1_robot_trajectory_msg =
      plan(panda_1_planning_component_ptr, panda_1_joint_names, p1_target_joints);
  moveit_msgs::RobotTrajectory panda_2_robot_trajectory_msg =
      plan(panda_2_planning_component_ptr, panda_2_joint_names, p2_target_joints);

  ASSERT_TRUE(moveit_cpp_ptr->getTrajectoryExecutionManager()->push(panda_1_robot_trajectory_msg));

  moveit_controller_manager::ExecutionStatus last_execution_status;

  auto callback = [this, &last_execution_status](__attribute__((unused))
                                                 const moveit_controller_manager::ExecutionStatus status) {
    execution_complete_condition_.notify_all();
    last_execution_status = status;
  };

  ASSERT_TRUE(moveit_cpp_ptr->getTrajectoryExecutionManager()->push(panda_2_robot_trajectory_msg, "", callback));

  std::unique_lock<std::mutex> ulock(execution_complete_mutex_);
  execution_complete_condition_.wait(ulock);

  // Check that execution status was successful
  ASSERT_TRUE(last_execution_status);

  // Wait for every execution to complete
  ASSERT_TRUE(moveit_cpp_ptr->getTrajectoryExecutionManager()->waitForExecution());
}

TEST_F(MoveItCppTest, RejectTrajectoryInCollision)
{
  std::vector<double> p1_target_joints{ 1.057, -0.323, 0.805, -2.857, 0.424, 2.557, 1.468 };
  std::vector<double> p2_target_joints{ 1.986, -0.416, 2.567, -2.089, 0.344, 2.414, -1.922 };

  moveit_msgs::RobotTrajectory panda_1_robot_trajectory_msg =
      plan(panda_1_planning_component_ptr, panda_1_joint_names, p1_target_joints);
  // Planning will find a collision free trajectory for the current planning scene
  moveit_msgs::RobotTrajectory panda_2_robot_trajectory_msg =
      plan(panda_2_planning_component_ptr, panda_2_joint_names, p2_target_joints);

  ASSERT_TRUE(moveit_cpp_ptr->getTrajectoryExecutionManager()->push(panda_1_robot_trajectory_msg));
  // Rejected because the trajectory is in collision course with panda1's trajectory
  ASSERT_FALSE(moveit_cpp_ptr->getTrajectoryExecutionManager()->push(panda_2_robot_trajectory_msg));

  moveit_cpp_ptr->getTrajectoryExecutionManager()->stopExecution(true);
}

TEST_F(MoveItCppTest, RejectInvalidTrajectory)
{
  std::vector<double> p1_target_joints1{ 1.057, -0.323, 0.805, -2.857, 0.424, 2.557, 1.468 };
  std::vector<double> p2_target_joints1{ 2.049, 0.046, 2.419, -2.660, -0.053, 2.624, -1.666 };
  std::vector<double> p1_target_joints2{ 0, 0, 0, -1.5707, 0, 1.8, 0 };

  moveit_msgs::RobotTrajectory panda_1_robot_trajectory_msg =
      plan(panda_1_planning_component_ptr, panda_1_joint_names, p1_target_joints1);
  moveit_msgs::RobotTrajectory panda_2_robot_trajectory_msg =
      plan(panda_2_planning_component_ptr, panda_2_joint_names, p2_target_joints1);
  moveit_msgs::RobotTrajectory panda_1_robot_trajectory_msg2 =
      plan(panda_1_planning_component_ptr, panda_1_joint_names, p1_target_joints2);

  ASSERT_TRUE(moveit_cpp_ptr->getTrajectoryExecutionManager()->push(panda_1_robot_trajectory_msg));
  ASSERT_TRUE(moveit_cpp_ptr->getTrajectoryExecutionManager()->push(panda_2_robot_trajectory_msg));

  // Rejected because group `panda 1` is still being used or the current state does not match the initial state of this trajectory
  ASSERT_FALSE(moveit_cpp_ptr->getTrajectoryExecutionManager()->push(panda_1_robot_trajectory_msg2));

  moveit_cpp_ptr->getTrajectoryExecutionManager()->stopExecution(true);
}

TEST_F(MoveItCppTest, CancelTrajectory)
{
  std::vector<double> p1_target_joints{ 0, 0, 0, -1.5707, 0, 1.8, 0 };

  moveit_msgs::RobotTrajectory panda_1_robot_trajectory_msg =
      plan(panda_1_planning_component_ptr, panda_1_joint_names, p1_target_joints);

  auto trajectory_id = moveit_cpp_ptr->getTrajectoryExecutionManager()->push(panda_1_robot_trajectory_msg);
  ASSERT_TRUE(trajectory_id);

  moveit_cpp_ptr->getTrajectoryExecutionManager()->stopExecution(trajectory_id);
  ASSERT_EQ(moveit_cpp_ptr->getTrajectoryExecutionManager()->waitForExecution(),
            moveit_controller_manager::ExecutionStatus::ABORTED);
}

}  // namespace moveit_cpp

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_simultaneous_execution_manager");

  int result = RUN_ALL_TESTS();

  return result;
}
