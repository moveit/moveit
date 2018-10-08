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
  // std::map<std::string, planning_interface::PlanningContextPtr> planners_; /**< The planners for each planning group
  // */

  StompSmoothingAdapter() : planning_request_adapter::PlanningRequestAdapter(), nh_("~")
  {
    const string ns = std::string("/move_group");

    if (!ns.empty())
    {
      nh_ = ros::NodeHandle(ns);
    }

    group_config = new std::map<string, XmlRpc::XmlRpcValue>();

    // below is the code from the function call to StompPlanner::getConfigData() function in the
    // StompPlannerManager::initialize()'s function body
    std::string param = "stomp";

    XmlRpc::XmlRpcValue stomp_config;

    if (!nh_.getParam(param, stomp_config))
    {
      ROS_ERROR("The 'stomp' configuration parameter was not found");
      // return false;
    }

    // each element under 'stomp' should be a group name
    std::string group_name;
    try
    {
      for (XmlRpc::XmlRpcValue::iterator v = stomp_config.begin(); v != stomp_config.end(); v++)
      {
        group_name = static_cast<std::string>(v->second["group_name"]);
        group_config->insert(std::make_pair(group_name, v->second));
      }
    }
    catch (XmlRpc::XmlRpcException& e)
    {
      ROS_ERROR("Unable to parse ROS parameter:\n ");
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

    // below is the implementation of code similar to what is done in the StompPlannerManager::initialize() function
    // body
    for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator v = group_config->begin(); v != group_config->end(); v++)
    {
      if (!model->hasJointModelGroup(v->first))
      {
        ROS_WARN("The robot model does not support the planning group '%s' in the STOMP configuration, skipping STOMP "
                 "setup for this group",
                 v->first.c_str());
        continue;
      }

      planners = new StompPlanner(v->first, v->second, robot_model_);
      break;

      // std::shared_ptr<StompPlanner> planner22(new StompPlanner(v->first, v->second, robot_model_));
      // planners_.insert(std::make_pair(v->first, planner22));
    }

    // create a temporary response object to populate the trajectory obtained from a given planner's initial response
    planning_interface::MotionPlanResponse res1 = res;

    // set the motion planning request
    planners->setMotionPlanRequest(req);

    // the res1 object at this stage has the trajectory obtained from the first planner and this trajectory is then used
    // as an initialization of the stomp planner in the stomp motion planner code
    bool stomp_planning_success = planners->solve(res1);

    // if stomp planner is successful in finding the solution, then store that into the res object as the updated
    // solution obtained from 2 motion planners=> initial planner+STOMP
    if (stomp_planning_success)
    {
      std::cout << "Stomp Planning adapter applied successfully " << std::endl;
      res = res1;
    }
    else
    {
      std::cout << "STOMP planning adapter DID NOT succeed!!" << std::endl;
    }
    return solved;
  }
};
}

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::StompSmoothingAdapter,
                            planning_request_adapter::PlanningRequestAdapter);