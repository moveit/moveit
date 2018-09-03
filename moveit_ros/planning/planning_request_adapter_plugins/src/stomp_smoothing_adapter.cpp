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

/* Author: Raghavender Sahdev */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <class_loader/class_loader.hpp>
#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>
#include <stomp_moveit/stomp_planner_manager.h>
#include <ros/node_handle.h>
#include <stomp_moveit/stomp_planner.h>
#include <stomp_core/stomp.h>

#include <stdio.h>
#include <iostream>

#include <XmlRpc.h>
#include <string.h>

// #include <stomp_core/utils.h>

using namespace stomp_moveit;
using namespace std;

namespace default_planner_request_adapters
{
class StompSmoothingAdapter : public planning_request_adapter::PlanningRequestAdapter
{
private:
  ros::NodeHandle nh_;

public:
  std::map<std::string, XmlRpc::XmlRpcValue>* group_config;

public:
  std::map<std::string, planning_interface::PlanningContextPtr> planners_; /**< The planners for each planning group */
  planning_interface::PlannerConfigurationMap pcs;
  moveit_msgs::MoveItErrorCodes error_code;

  StompSmoothingAdapter() : planning_request_adapter::PlanningRequestAdapter(), nh_("~")
  {
    const string ns = std::string("/move_group");
    if (!ns.empty())
    {
      nh_ = ros::NodeHandle(ns);
    }

    group_config = new std::map<string, XmlRpc::XmlRpcValue>();

    std::string param = "stomp";
    // Create a stomp planner for each group
    XmlRpc::XmlRpcValue stomp_config;
    std::cout << " I AM JUST BEFORE NH.GETPARAM() " << std::endl;
    std::cout << nh_.getParam(param, stomp_config) << "decision " << std::endl;

    if (!nh_.getParam(param, stomp_config))
    {
      ROS_ERROR("The 'stomp' configuration parameter was not found");
      // return false;
    }

    std::cout << "JUST AFTER GETPARAM() " << std::endl;

    // each element under 'stomp' should be a group name
    std::string group_name;
    try
    {
      // std::cout << stomp_config.begin()->first << " stomp_config begin" << std::endl;
      // std::cout << stomp_config.begin()->second << " stomp_config begin sec" << std::endl;

      for (XmlRpc::XmlRpcValue::iterator v = stomp_config.begin(); v != stomp_config.end(); v++)
      {
        // std::cout << v->first << " stomp_config begin" << std::endl;
        // std::cout << v->second << " stomp_config begin sec" << std::endl;

        group_name = static_cast<std::string>(v->second["group_name"]);
        group_config->insert(std::make_pair(group_name, v->second));

        // std::cout << group_name << std::endl;
        // std::cout << v->second << " #$@$#@ " << std::endl;
      }
      // return true;
    }
    catch (XmlRpc::XmlRpcException& e)
    {
      ROS_ERROR("Unable to parse ROS parameter:\n ");
      // ROS_ERROR("Unable to parse ROS parameter:\n %s", stomp_config.toXml().c_str());
      // return false;
    }
  }

  virtual std::string getDescription() const
  {
    return "Fix Start State In Collision";
  }

  robot_model::RobotModelConstPtr model2;
  moveit::core::RobotModelConstPtr robot_model_;
  virtual bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const planning_interface::MotionPlanRequest& req,
                            planning_interface::MotionPlanResponse& res,
                            std::vector<std::size_t>& added_path_index) const
  {
    bool solved = planner(planning_scene, req, res);

    const robot_model::RobotModelConstPtr model = planning_scene->getRobotModel();
    moveit::core::RobotModelConstPtr robot_model_ = model;

    StompPlanner* planners;

    for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator v = group_config->begin(); v != group_config->end(); v++)
    {
      if (!model->hasJointModelGroup(v->first))
      {
        ROS_WARN("The robot model does not support the planning group '%s' in the STOMP configuration, skipping STOMP "
                 "setup for this group",
                 v->first.c_str());
        continue;
      }

      std::cout << v->first << " v->first " << std::endl;
      std::cout << v->second << " v->second " << std::endl;

      std::cout << robot_model_.get()->getName() << " model.get()->getName()" << std::endl;
      std::cout << model.get()->getModelFrame() << " model.get()->printModelINfo()" << std::endl;
      std::cout << model.get()->getVariableNames().at(1) << " model.get()->getVariableNames()" << std::endl;
      std::cout << model.get()->getJointModelNames().at(1) << " model.get()->getJointModelNames()" << std::endl;

      /// I keep getting a symbol lookup error here for some weird reason for either of the following 2 lines....
      planners = new StompPlanner(v->first, v->second, robot_model_);
      // std::shared_ptr<StompPlanner> planner(new StompPlanner(v->first, v->second, robot_model_));

      // planners_.insert(std::make_pair(v->first, planner));
    }

    std::cout << "I am in STOMP PLanning adapter" << std::endl;

    planning_scene::PlanningSceneConstPtr planning_scene1;
    planning_interface::MotionPlanRequest req1;
    planning_interface::MotionPlanResponse res1;

    /// Need to send the MotionPlanRequest object, 'req' to the STOMP solver here, Also initialize the MotionPlanRequest
    /// in the STOMP constructor in the stomp_core package
    /// this part is not as intuitive as CHOMP

    ros::WallTime start_time = ros::WallTime::now();

    planning_interface::MotionPlanDetailedResponse detailed_res;
    bool success = planners->solve(detailed_res);
    if (success)
    {
      res1.trajectory_ = detailed_res.trajectory_.back();
    }
    ros::WallDuration wd = ros::WallTime::now() - start_time;
    res1.planning_time_ = ros::Duration(wd.sec, wd.nsec).toSec();
    res1.error_code_ = detailed_res.error_code_;

    /// Once motion plan is found for STOMP copy it into the original MotionPlanResponse res object

    // return success; // stomp planner solver result status

    return solved;
  }
};
}

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::StompSmoothingAdapter,
                            planning_request_adapter::PlanningRequestAdapter);