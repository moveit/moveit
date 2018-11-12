/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Raphael Bricout */
/* Author: Shadow Software Team */

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <eigen_conversions/eigen_msg.h>
#include <moveit/kinematic_constraints/utils.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

typedef std::pair<geometry_msgs::Point, geometry_msgs::Quaternion> LinkConstraintPair;
typedef std::map<std::string, LinkConstraintPair> LinkConstraintMap;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_warehouse_as_text", ros::init_options::AnonymousName);

  boost::program_options::options_description desc;
  desc.add_options()("help", "Show help message")("host", boost::program_options::value<std::string>(), "Host for the "
                                                                                                        "DB.")(
      "port", boost::program_options::value<std::size_t>(), "Port for the DB.")("clear", "Clears all the random "
                                                                                         "queries for a given scene")(
      "cartesian", "Generate the cartesian equivalent as "
                   "well.")("limited_joints", "Limit joints from -pi to pi to avoid a lot of impossible queries.")(
      "group_prefix", boost::program_options::value<std::string>(),
      "Specify the group_prefix you'd like to plan with.")("eef", boost::program_options::value<std::string>(),
                                                           "Specify the end effector. Default: last link. Only needed "
                                                           "for cartesian queries");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  std::string eef_name = "";
  if (vm.count("eef"))
  {
    eef_name = vm["eef"].as<std::string>();
  }

  std::string group_prefix = "";
  if (vm.count("group_prefix"))
  {
    group_prefix = vm["group_prefix"].as<std::string>();
  }
  else
  {
    ROS_INFO("If you have a problem with a composite robot, try to use the group_prefix option.");
  }

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  // Set up db
  warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase();
  if (vm.count("host") && vm.count("port"))
    conn->setParams(vm["host"].as<std::string>(), vm["port"].as<std::size_t>());
  if (!conn->connect())
    return 1;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);

  moveit_warehouse::PlanningSceneStorage pss(conn);
  moveit_warehouse::RobotStateStorage rss(conn);
  moveit_warehouse::ConstraintsStorage cs(conn);

  std::string scene_name;
  int tot_queries_number = 1;
  int cur_queries_number = 0;
  int fail_queries_bound = 1000000;
  int fail_queries_cur = 0;

  if (argc >= 2)
  {
    scene_name = argv[1];
  }

  if (argc >= 3)
  {
    tot_queries_number = atoi(argv[2]);
  }

  if (vm.count("clear"))
  {
    std::vector<std::string> query_names;
    std::stringstream pssregex;
    pssregex << ".*";
    pss.getPlanningQueriesNames(pssregex.str(), query_names, scene_name);
    for (int i = 0; i < query_names.size(); ++i)
    {
      if (query_names[i].find("RANDOM") != std::string::npos)
      {
        pss.removePlanningQuery(scene_name, query_names[i]);
        ROS_INFO("Query '%s' removed", query_names[i].c_str());
      }
      if (query_names[i].find("cartesian") != std::string::npos)
      {
        pss.removePlanningQuery(scene_name, query_names[i]);
        ROS_INFO("Query '%s' removed", query_names[i].c_str());
      }
    }
    ROS_INFO("Cleaning done");
    return 0;
  }

  moveit_warehouse::PlanningSceneWithMetadata pswm;

  if (pss.getPlanningScene(pswm, scene_name))
  {
    srand(static_cast<unsigned>(time(0)));

    psm.getPlanningScene()->setPlanningSceneMsg(static_cast<const moveit_msgs::PlanningScene&>(*pswm));
    while (cur_queries_number < tot_queries_number && fail_queries_cur < fail_queries_bound)
    {
      robot_model::RobotModelConstPtr km = psm.getRobotModel();
      planning_scene::PlanningScenePtr planning_scene = psm.getPlanningScene();

      moveit::core::RobotState coll_start_state(km);
      moveit::core::RobotState coll_goal_state(km);

      std::vector<std::string> names = coll_start_state.getVariableNames();
      std::map<std::string, double> var_start;
      std::vector<double> goal_joints = {};

      // generation of random joint values
      for (int i = 0; i < names.size(); ++i)
      {
        if (names[i].compare(0, group_prefix.length(), group_prefix) == 0)
        {
          float bound_up = km->getVariableBounds(names[i]).max_position_;
          float bound_down = km->getVariableBounds(names[i]).min_position_;
          if (vm.count("limited_joints"))
          {
            bound_up = 3.14;
            bound_down = -3.14;
          }
          double j1 =
              static_cast<double>(rand()) / static_cast<double>(RAND_MAX / (bound_up - bound_down)) + bound_down;
          double j2 =
              static_cast<double>(rand()) / static_cast<double>(RAND_MAX / (bound_up - bound_down)) + bound_down;
          coll_start_state.setJointPositions(names[i], { j1 });
          coll_goal_state.setJointPositions(names[i], { j2 });
          var_start[names[i]] = j1;
          goal_joints.push_back(j2);
        }
        else
        {
          coll_start_state.setJointPositions(names[i], { 0.0 });
          coll_goal_state.setJointPositions(names[i], { 0.0 });
          var_start[names[i]] = 0;
          goal_joints.push_back(0);
        }
      }
      collision_detection::CollisionRequest collision_request;
      collision_detection::CollisionResult collision_result_start;
      collision_detection::CollisionResult collision_result_goal;

      moveit_msgs::RobotState msg_start_state;
      moveit_msgs::Constraints msg_goal_state;

      // check start state collision
      planning_scene->checkCollision(collision_request, collision_result_start, coll_start_state);
      if (!collision_result_start.collision)
      {
        robot_state::RobotState st = planning_scene->getCurrentState();
        st.setVariablePositions(var_start);
        robot_state::robotStateToRobotStateMsg(st, msg_start_state);
      }

      // check goal state collision
      planning_scene->checkCollision(collision_request, collision_result_goal, coll_goal_state);
      if (!collision_result_goal.collision)
      {
        std::vector<moveit_msgs::JointConstraint> joint_constraints;
        for (int i = 0; i < names.size(); ++i)
        {
          moveit_msgs::JointConstraint joint_constraint;
          joint_constraint.joint_name = names[i];
          joint_constraint.position = goal_joints[i];
          joint_constraint.tolerance_above = 1.0e-6;
          joint_constraint.tolerance_below = 1.0e-6;
          joint_constraint.weight = 1.0;
          joint_constraints.push_back(joint_constraint);
        }
        msg_goal_state.joint_constraints = joint_constraints;
      }

      if (!collision_result_start.collision && !collision_result_goal.collision)
      {
        moveit_msgs::MotionPlanRequest planning_query;
        planning_query.start_state = msg_start_state;
        planning_query.goal_constraints = { msg_goal_state };

        std::string query_name = "RANDOM_pose" + std::to_string(cur_queries_number);
        pss.addPlanningQuery(planning_query, scene_name, query_name);

        ROS_INFO("Random query '%s' sucessfully added after %d fail(s)", query_name.c_str(), fail_queries_cur);

        if (vm.count("cartesian"))
        {
          const std::vector<std::string>& id_names = km->getLinkModelNames();
          const Eigen::Affine3d& link_pose = coll_goal_state.getGlobalLinkTransform(eef_name);
          geometry_msgs::Transform transform;
          geometry_msgs::PoseStamped pose;
          moveit_msgs::Constraints msg_goal_cart;
          std::string query_name = "cartesian_RANDOM_pose" + std::to_string(cur_queries_number);
          tf::transformEigenToMsg(link_pose, transform);
          pose.pose.orientation = transform.rotation;
          pose.pose.position.x = transform.translation.x;
          pose.pose.position.y = transform.translation.y;
          pose.pose.position.z = transform.translation.z;
          pose.header.frame_id = id_names[0];

          msg_goal_cart = kinematic_constraints::constructGoalConstraints(eef_name, pose);
          planning_query.goal_constraints = { msg_goal_cart };
          pss.addPlanningQuery(planning_query, scene_name, query_name);
          ROS_INFO("Random query '%s' sucessfully added", query_name.c_str());
        }
        cur_queries_number++;
        fail_queries_cur = 0;
      }
      else
      {
        fail_queries_cur++;
      }
    }
    if (fail_queries_cur >= fail_queries_bound)
    {
      ROS_ERROR("FAIL: '%d' consecutive random queries failed, exiting the while loop", fail_queries_bound);
    }
  }
  return 0;
}
