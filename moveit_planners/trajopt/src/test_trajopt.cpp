#include <ros/ros.h>
#include "trajopt_planning_context.h"
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "moveit/planning_interface/planning_request.h"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// test

int main(int argc, char** argv)
{
    const std::string node_name = "trajopot_planning_tutorial";
      ros::init(argc, argv, node_name);
      ros::AsyncSpinner spinner(1);
      spinner.start();
      ros::NodeHandle node_handle("~");

      const std::string PLANNING_GROUP = "panda_arm";
      robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
      robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();
      /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
      robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
      const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
      robot_state->setToDefaultValues();

      const std::vector<std::string>& joint_names = joint_model_group->getActiveJointModelNames();
      planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

      // With the planning scene we create a planing scene monitor that
      // monitors planning scene diffs and applys them to the planning scene
      planning_scene_monitor::PlanningSceneMonitorPtr psm(
          new planning_scene_monitor::PlanningSceneMonitor(planning_scene, robot_model_loader));
      psm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
      psm->startStateMonitor();
      psm->startSceneMonitor();

      while (!psm->getStateMonitor()->haveCompleteState() && ros::ok())
       {
         ROS_INFO_STREAM_THROTTLE_NAMED(1, node_name, "Waiting for complete state from topic ");
        }
      // We will now construct a loader to load a planner, by name.
      // Note that we are using the ROS pluginlib library here.
      boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
      planning_interface::PlannerManagerPtr planner_instance;

      std::string planner_plugin_name;

       planner_plugin_name = "trajopt_interface/TrajOptPlanner";

      // planner_plugin_name = "ompl_interface/OMPLPlanner";
      // node_handle.getParam("...", ...);

      node_handle.setParam("planning_plugin", planner_plugin_name);

    // making sure to catch all exceptions.
    if (!node_handle.getParam("planning_plugin", planner_plugin_name))
      ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
      printf("===>>> planner_plugin_name: %s \n", planner_plugin_name.c_str());
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance->initialize(robot_model, node_handle.getNamespace())) // namespace ????. I dont use it
        ROS_FATAL_STREAM("Could not initialize planner instance");
      ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (std::size_t i = 0; i < classes.size(); ++i)
        ss << classes[i] << " ";
      ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                           << "Available plugins: " << ss.str());
     }

    // Visualization
    // ========================================================================================
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, psm);
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();

    /* Remote control is an introspection tool that allows users to step through a high level script
       via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();

    /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

    /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
    visual_tools.trigger();

    /* We can also use visual_tools to wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Pose Goal
    // ========================================================================================
    // We will now create a motion plan request for the arm of the Panda
    // specifying the desired pose of the end-effector as input.
    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    visual_tools.trigger();
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "panda_link0";
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.4;
    pose.pose.position.z = 0.75;
    pose.pose.orientation.w = 1.0;

    // A tolerance of 0.01 m is specified in position
    // and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    // We will create the request as a constraint using a helper function available
    // from the
    // `kinematic_constraints`_
    // package.
    //
    // .. _kinematic_constraints:
    //     http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
    moveit_msgs::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

    //    req.group_name = PLANNING_GROUP;
    // req.goal_constraints.push_back(pose_goal);

    // set the request start joint state
    // ========================================================================================
    // get the joint values of the start state
    std::vector<double> start_joint_values;
    robot_state->copyJointGroupPositions(joint_model_group, start_joint_values);
    int r = 0;
    for (auto x : start_joint_values)
    {
      printf("===>>> joint: %s with value: %f \n", joint_names[r].c_str(), x);
      ++r;
    }

    req.start_state.joint_state.name = joint_names;
    req.start_state.joint_state.position = start_joint_values;

    // set the goal joint state
    // ========================================================================================
    robot_state::RobotState goal_state(robot_model);
    std::vector<double> joint_values = { 0.8, 0.7, 1, -1.3, 1.9, 2.2, -0.1};
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);
    req.group_name = PLANNING_GROUP;

    // retrive joint values at goal
    std::vector<moveit_msgs::Constraints> goal_constraints = req.goal_constraints;
    std::cout << "===>>> number of constraints in goal: " << goal_constraints.size() << std::endl;
    std::vector<moveit_msgs::JointConstraint> goal_joint_constraint = goal_constraints[0].joint_constraints;
    for (auto x : goal_joint_constraint)
    {
      std::cout << "==>> joint position at goal " << x.position << std::endl;
    }

    // planning context
    planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }

    visual_tools.prompt("Press 'next' to visualize the result");

    // Visualize the result
    // ========================================================================================
    ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    /* Visualize the trajectory */
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    for (int timestep_index = 0; timestep_index < response.trajectory.joint_trajectory.points.size(); ++timestep_index)
      {
        for (int joint_index = 0; joint_index < response.trajectory.joint_trajectory.points[timestep_index].positions.size(); ++joint_index)
          {
            std::cout << response.trajectory.joint_trajectory.points[timestep_index].positions[joint_index] << "  ";
          }
        std::cout << std::endl;
      }

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);

    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher.publish(display_trajectory);

  // Set the state in the planning scene to the final state of the last plan
  // robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  // planning_scene->setCurrentState(*robot_state.get());

  // Display the goal state
  // visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  // visual_tools.publishAxisLabeled(pose.pose, "goal_1");
  // visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();


}
