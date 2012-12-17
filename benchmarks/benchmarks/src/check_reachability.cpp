/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mario Prats
 */

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <boost/thread.hpp>
#include <boost/program_options/variables_map.hpp>

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/planning_scene_world_storage.h>
#include <moveit/kinematics_constraint_aware/kinematics_constraint_aware.h>
#include <moveit/kinematics_cache/kinematics_cache.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_state/kinematic_state.h>
#include <moveit/benchmarks/benchmarks_config.h>

int main(int argc, char** argv)
{
  boost::program_options::options_description desc;
  desc.add_options()
          ("help", "Show help message")
          ("host", boost::program_options::value<std::string>(), "Host for the MongoDB.")
          ("port", boost::program_options::value<std::size_t>(), "Port for the MongoDB.");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  ros::init(argc, argv, "arm_workspace_tests");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit_benchmarks::BenchmarkConfig bc(vm.count("host") ? vm["host"].as<std::string>() : "",
      vm.count("port") ? vm["port"].as<std::size_t>() : 0);

  std::string db_host = vm["host"].as<std::string>();
  int db_port = vm["port"].as<std::size_t>();


  planning_models_loader::KinematicModelLoader kinematic_model_loader("robot_description"); /** Used to load the robot model */  
  kinematic_model::KinematicModelPtr kinematic_model = kinematic_model_loader.getModel();

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));


  unsigned int proc = 0;
  for (int i = 1 ; i < argc ; ++i)
  {
    if (bc.readOptions(argv[i]))
    {
      std::stringstream ss;
      bc.printOptions(ss);
      ROS_INFO("Calling kinematic reachability benchmark with options:\n%s\n", ss.str().c_str());
      proc++;

      std::string group_name, frame_id, scene_name;

      scene_name = bc.getOptions().scene;
      group_name = bc.getOptions().group_override;
      frame_id = bc.getOptions().planning_frame;

      moveit_warehouse::PlanningSceneStorage pss(db_host, db_port);
      moveit_warehouse::PlanningSceneWithMetadata pswm;
      pss.getPlanningScene(pswm, scene_name);
      moveit_msgs::PlanningScene ps = static_cast<const moveit_msgs::PlanningScene&>(*pswm);
      planning_scene_monitor->getPlanningScene()->usePlanningSceneMsg(ps);

      ROS_INFO_STREAM("Processing scene " << planning_scene_monitor->getPlanningScene()->getName());

      kinematics_constraint_aware::KinematicsConstraintAwarePtr kinematics_constraint_aware;
      kinematics_constraint_aware.reset(new kinematics_constraint_aware::KinematicsConstraintAware(kinematic_model, group_name));

      moveit_warehouse::ConstraintsStorage cs(db_host, db_port);
      std::vector<std::string> constraints_names;
      cs.getKnownConstraints(bc.getOptions().goal_regex, constraints_names);
      for (int i=0; i<constraints_names.size(); i++)
      {
        moveit_warehouse::ConstraintsWithMetadata cwm;
        cs.getConstraints(cwm, constraints_names[i]);

        geometry_msgs::PoseStamped ik_pose;
        ik_pose.header.frame_id = frame_id;
        ik_pose.pose.position.x = cwm->position_constraints[0].constraint_region.primitive_poses[0].position.x;
        ik_pose.pose.position.y = cwm->position_constraints[0].constraint_region.primitive_poses[0].position.y;
        ik_pose.pose.position.z = cwm->position_constraints[0].constraint_region.primitive_poses[0].position.z;
        ik_pose.pose.orientation.x = cwm->orientation_constraints[0].orientation.x;
        ik_pose.pose.orientation.y = cwm->orientation_constraints[0].orientation.y;
        ik_pose.pose.orientation.z = cwm->orientation_constraints[0].orientation.z;
        ik_pose.pose.orientation.w = cwm->orientation_constraints[0].orientation.w;

        moveit_msgs::MoveItErrorCodes error_code;
        moveit_msgs::RobotState solution;

        //Fill the IK Request msg
        moveit_msgs::GetConstraintAwarePositionIK::Request request;
        moveit_msgs::GetConstraintAwarePositionIK::Response response;

        kinematic_model::KinematicModelConstPtr kinematic_model = kinematics_constraint_aware->getKinematicModel();
        kinematic_state::KinematicState kinematic_state(kinematic_model);
        const kinematic_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_name);
        kinematic_state::JointStateGroup* joint_state_group = kinematic_state.getJointStateGroup(group_name);
        joint_state_group->setToRandomValues();

        request.timeout = ros::Duration(5.0);
        request.ik_request.robot_state.joint_state.name = joint_model_group->getJointModelNames();
        joint_state_group->getVariableValues(request.ik_request.robot_state.joint_state.position);
        request.ik_request.group_name = group_name;
        request.ik_request.pose_stamped = ik_pose;

        //Compute IK
        kinematics_constraint_aware->getIK(planning_scene_monitor->getPlanningScene(),request,response);

        if (response.error_code.val == response.error_code.SUCCESS)
        {
          ROS_INFO_STREAM(" Goal " << constraints_names[i] << " SUCCESS!");
        }
        else
        {
          ROS_INFO_STREAM(" Goal " << constraints_names[i] << " FAIL with code " << (int)response.error_code.val);
        }
      }
    }
  }
  ROS_INFO("Processed %u benchmark configuration files", proc);

  return(0);
}
