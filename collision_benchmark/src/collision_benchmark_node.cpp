#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bt.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

int main(int argc, char* argv[])
{
  robot_model::RobotModelPtr robot_model;
  ros::init(argc, argv, "compare_collision_checking_speed");
  ros::NodeHandle node_handle;

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  unsigned int trials = 5000;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration sleep_t(2.5);

  robot_model = moveit::core::loadTestingRobotModel("panda");

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_scene_monitor::PlanningSceneMonitor psm(planning_scene, ROBOT_DESCRIPTION);
  psm.startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  psm.startSceneMonitor();
  planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBt::create());

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(
      robot_model->getLinkModelNames(), true) };
  planning_scene->checkCollision(req, res, planning_scene->getCurrentState(), acm);

  ROS_INFO("Starting...");

  if (psm.getPlanningScene())
  {
    ros::Duration(0.5).sleep();

    robot_state::RobotState& current_state{ planning_scene->getCurrentStateNonConst() };
    current_state.setToDefaultValues(current_state.getJointModelGroup("panda_arm"), "home");
    current_state.update();

    shapes::ShapeConstPtr shape;

    Eigen::Isometry3d pos{ Eigen::Isometry3d::Identity() };

    // Box between home and other position
    //pos.translation().x() = 0.43;
    //pos.translation().y() = 0;
    //pos.translation().z() = 0.55;

    shape.reset(new shapes::Box(0.1, 0.1, 0.1));
    std::string name = "box";

    planning_scene->getWorldNonConst()->addToObject(name, shape, pos);

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    planning_scene->checkCollision(req, res);
    ROS_INFO_STREAM((res.collision ? "In collision" : "Not in collision"));
    res.clear();

    // home values
    //    <joint name="panda_joint1" value="0" />
    //    <joint name="panda_joint2" value="-0.785" />
    //    <joint name="panda_joint3" value="0" />
    //    <joint name="panda_joint4" value="-2.356" />
    //    <joint name="panda_joint5" value="0.0" />
    //    <joint name="panda_joint6" value="1.571" />
    //    <joint name="panda_joint7" value="0.785" />

    // joints for in front of box
    //double joint2 = 0.05;
    //double joint4 = -1.6;

    // joints for continous self collision check
    double joint2 = 0.15;
    double joint4 = -3.0;
    double joint5 = 0.8;
    double joint7 = -0.785;
    current_state.setJointPositions("panda_joint2", &joint2);
    current_state.setJointPositions("panda_joint4", &joint4);
    current_state.setJointPositions("panda_joint5", &joint5);
    current_state.setJointPositions("panda_joint7", &joint7);
    current_state.update();

    planning_scene->checkCollision(req, res);
    ROS_INFO_STREAM((res.collision ? "In collision" : "Not in collision"));
    res.clear();

    // load panda link5 as world collision object
    shapes::ShapeConstPtr shape_mesh;
    std::string kinect = "package://moveit_resources/panda_description/meshes/collision/hand.stl";

    Eigen::Isometry3d pos_2{ Eigen::Isometry3d::Identity() };
    pos_2.translation().x() = 1;

    shapes::Mesh* mesh = shapes::createMeshFromResource(kinect);
    mesh->scale(1);
    shape_mesh.reset(mesh);
    planning_scene->getWorldNonConst()->addToObject("mesh", shape_mesh, pos_2);

    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene->getPlanningSceneMsg(planning_scene_msg);
    planning_scene_diff_publisher.publish(planning_scene_msg);
  }
  else
  {
    ROS_ERROR("Planning scene not configured");
  }

  return 0;
}
