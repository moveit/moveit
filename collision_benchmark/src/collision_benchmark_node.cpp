#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bt.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "compare_collision_checking_speed");
  ros::NodeHandle node_handle;

  ros::Rate loop_rate(1);

  ROS_INFO_STREAM("Starting...");

  robot_model::RobotModelPtr robot_model;
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  robot_model = moveit::core::loadTestingRobotModel("panda");

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_scene_monitor::PlanningSceneMonitor psm(planning_scene, "robot_description");
  psm.startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  psm.startSceneMonitor();
  planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBt::create());

  robot_state::RobotState& current_state{ planning_scene->getCurrentStateNonConst() };
  current_state.setToDefaultValues(current_state.getJointModelGroup("panda_arm"), "home");
  // current_state.setToDefaultValues();
  current_state.update();

  shapes::ShapeConstPtr box(new shapes::Box(0.1, 0.1, 0.1));

  // add with random size and random position
  Eigen::Isometry3d pos{ Eigen::Isometry3d::Identity() };
  pos.translation().x() = 0;
  pos.translation().y() = 0.1;
  pos.translation().z() = 0.65;

  planning_scene->getWorldNonConst()->addToObject("box1", box, pos);

  moveit_msgs::PlanningScene planning_scene_msg;

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  planning_scene->checkCollision(req, res);

  if (res.collision)
  {
    ROS_ERROR_STREAM("collided");
  }

  ROS_ERROR_STREAM(planning_scene->getPlanningFrame());

  while (ros::ok())
  {
    planning_scene->getPlanningSceneMsg(planning_scene_msg);
    planning_scene_diff_publisher.publish(planning_scene_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
