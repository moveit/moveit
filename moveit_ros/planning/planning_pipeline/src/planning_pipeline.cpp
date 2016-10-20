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

/* Author: Ioan Sucan */

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string/join.hpp>
#include <sstream>

const std::string planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC = "display_planned_path";
const std::string planning_pipeline::PlanningPipeline::MOTION_PLAN_REQUEST_TOPIC = "motion_plan_request";
const std::string planning_pipeline::PlanningPipeline::MOTION_CONTACTS_TOPIC = "display_contacts";

planning_pipeline::PlanningPipeline::PlanningPipeline(const robot_model::RobotModelConstPtr& model,
                                                      const ros::NodeHandle& nh,
                                                      const std::string& planner_plugin_param_name,
                                                      const std::string& adapter_plugins_param_name)
  : nh_(nh), kmodel_(model)
{
  std::string planner;
  if (nh_.getParam(planner_plugin_param_name, planner))
    planner_plugin_name_ = planner;

  std::string adapters;
  if (nh_.getParam(adapter_plugins_param_name, adapters))
  {
    boost::char_separator<char> sep(" ");
    boost::tokenizer<boost::char_separator<char> > tok(adapters, sep);
    for (boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin(); beg != tok.end(); ++beg)
      adapter_plugin_names_.push_back(*beg);
  }

  configure();
}

planning_pipeline::PlanningPipeline::PlanningPipeline(const robot_model::RobotModelConstPtr& model,
                                                      const ros::NodeHandle& nh, const std::string& planner_plugin_name,
                                                      const std::vector<std::string>& adapter_plugin_names)
  : nh_(nh), planner_plugin_name_(planner_plugin_name), adapter_plugin_names_(adapter_plugin_names), kmodel_(model)
{
  configure();
}

void planning_pipeline::PlanningPipeline::configure()
{
  check_solution_paths_ = false;  // this is set to true below
  publish_received_requests_ = false;
  display_computed_motion_plans_ = false;  // this is set to true below

  // load the planning plugin
  try
  {
    planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }

  std::vector<std::string> classes;
  if (planner_plugin_loader_)
    classes = planner_plugin_loader_->getDeclaredClasses();
  if (planner_plugin_name_.empty() && classes.size() == 1)
  {
    planner_plugin_name_ = classes[0];
    ROS_INFO("No '~planning_plugin' parameter specified, but only '%s' planning plugin is available. Using that one.",
             planner_plugin_name_.c_str());
  }
  if (planner_plugin_name_.empty() && classes.size() > 1)
  {
    planner_plugin_name_ = classes[0];
    ROS_INFO("Multiple planning plugins available. You should specify the '~planning_plugin' parameter. Using '%s' for "
             "now.",
             planner_plugin_name_.c_str());
  }
  try
  {
    planner_instance_.reset(planner_plugin_loader_->createUnmanagedInstance(planner_plugin_name_));
    if (!planner_instance_->initialize(kmodel_, nh_.getNamespace()))
      throw std::runtime_error("Unable to initialize planning plugin");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance_->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_STREAM("Exception while loading planner '"
                     << planner_plugin_name_ << "': " << ex.what() << std::endl
                     << "Available plugins: " << boost::algorithm::join(classes, ", "));
  }

  // load the planner request adapters
  if (!adapter_plugin_names_.empty())
  {
    std::vector<planning_request_adapter::PlanningRequestAdapterConstPtr> ads;
    try
    {
      adapter_plugin_loader_.reset(new pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter>(
          "moveit_core", "planning_request_adapter::PlanningRequestAdapter"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Exception while creating planning plugin loader " << ex.what());
    }

    if (adapter_plugin_loader_)
      for (std::size_t i = 0; i < adapter_plugin_names_.size(); ++i)
      {
        planning_request_adapter::PlanningRequestAdapterConstPtr ad;
        try
        {
          ad.reset(adapter_plugin_loader_->createUnmanagedInstance(adapter_plugin_names_[i]));
        }
        catch (pluginlib::PluginlibException& ex)
        {
          ROS_ERROR_STREAM("Exception while loading planning adapter plugin '" << adapter_plugin_names_[i]
                                                                               << "': " << ex.what());
        }
        if (ad)
          ads.push_back(ad);
      }
    if (!ads.empty())
    {
      adapter_chain_.reset(new planning_request_adapter::PlanningRequestAdapterChain());
      for (std::size_t i = 0; i < ads.size(); ++i)
      {
        ROS_INFO_STREAM("Using planning request adapter '" << ads[i]->getDescription() << "'");
        adapter_chain_->addAdapter(ads[i]);
      }
    }
  }
  displayComputedMotionPlans(true);
  checkSolutionPaths(true);
}

void planning_pipeline::PlanningPipeline::displayComputedMotionPlans(bool flag)
{
  if (display_computed_motion_plans_ && !flag)
    display_path_publisher_.shutdown();
  else if (!display_computed_motion_plans_ && flag)
    display_path_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>(DISPLAY_PATH_TOPIC, 10, true);
  display_computed_motion_plans_ = flag;
}

void planning_pipeline::PlanningPipeline::publishReceivedRequests(bool flag)
{
  if (publish_received_requests_ && !flag)
    received_request_publisher_.shutdown();
  else if (!publish_received_requests_ && flag)
    received_request_publisher_ = nh_.advertise<moveit_msgs::MotionPlanRequest>(MOTION_PLAN_REQUEST_TOPIC, 10, true);
  publish_received_requests_ = flag;
}

void planning_pipeline::PlanningPipeline::checkSolutionPaths(bool flag)
{
  if (check_solution_paths_ && !flag)
    contacts_publisher_.shutdown();
  else if (!check_solution_paths_ && flag)
    contacts_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(MOTION_CONTACTS_TOPIC, 100, true);
  check_solution_paths_ = flag;
}

bool planning_pipeline::PlanningPipeline::generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                       const planning_interface::MotionPlanRequest& req,
                                                       planning_interface::MotionPlanResponse& res) const
{
  std::vector<std::size_t> dummy;
  return generatePlan(planning_scene, req, res, dummy);
}

bool planning_pipeline::PlanningPipeline::generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                       const planning_interface::MotionPlanRequest& req,
                                                       planning_interface::MotionPlanResponse& res,
                                                       std::vector<std::size_t>& adapter_added_state_index) const
{
  // broadcast the request we are about to work on, if needed
  if (publish_received_requests_)
    received_request_publisher_.publish(req);
  adapter_added_state_index.clear();

  if (!planner_instance_)
  {
    ROS_ERROR("No planning plugin loaded. Cannot plan.");
    return false;
  }

  bool solved = false;
  try
  {
    if (adapter_chain_)
    {
      solved = adapter_chain_->adaptAndPlan(planner_instance_, planning_scene, req, res, adapter_added_state_index);
      if (!adapter_added_state_index.empty())
      {
        std::stringstream ss;
        for (std::size_t i = 0; i < adapter_added_state_index.size(); ++i)
          ss << adapter_added_state_index[i] << " ";
        ROS_INFO("Planning adapters have added states at index positions: [ %s]", ss.str().c_str());
      }
    }
    else
    {
      planning_interface::PlanningContextPtr context =
          planner_instance_->getPlanningContext(planning_scene, req, res.error_code_);
      solved = context ? context->solve(res) : false;
    }
  }
  catch (std::runtime_error& ex)
  {
    ROS_ERROR("Exception caught: '%s'", ex.what());
    return false;
  }
  catch (...)
  {
    ROS_ERROR("Unknown exception thrown by planner");
    return false;
  }
  bool valid = true;

  if (solved && res.trajectory_)
  {
    std::size_t state_count = res.trajectory_->getWayPointCount();
    ROS_DEBUG_STREAM("Motion planner reported a solution path with " << state_count << " states");
    if (check_solution_paths_)
    {
      std::vector<std::size_t> index;
      if (!planning_scene->isPathValid(*res.trajectory_, req.path_constraints, req.group_name, false, &index))
      {
        // check to see if there is any problem with the states that are found to be invalid
        // they are considered ok if they were added by a planning request adapter
        bool problem = false;
        for (std::size_t i = 0; i < index.size() && !problem; ++i)
        {
          bool found = false;
          for (std::size_t j = 0; j < adapter_added_state_index.size(); ++j)
            if (index[i] == adapter_added_state_index[j])
            {
              found = true;
              break;
            }
          if (!found)
            problem = true;
        }
        if (problem)
        {
          if (index.size() == 1 && index[0] == 0)  // ignore cases when the robot starts at invalid location
            ROS_DEBUG("It appears the robot is starting at an invalid state, but that is ok.");
          else
          {
            valid = false;
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;

            // display error messages
            std::stringstream ss;
            for (std::size_t i = 0; i < index.size(); ++i)
              ss << index[i] << " ";
            ROS_ERROR_STREAM("Computed path is not valid. Invalid states at index locations: [ "
                             << ss.str() << "] out of " << state_count
                             << ". Explanations follow in command line. Contacts are published on "
                             << nh_.resolveName(MOTION_CONTACTS_TOPIC));

            // call validity checks in verbose mode for the problematic states
            visualization_msgs::MarkerArray arr;
            for (std::size_t i = 0; i < index.size(); ++i)
            {
              // check validity with verbose on
              const robot_state::RobotState& kstate = res.trajectory_->getWayPoint(index[i]);
              planning_scene->isStateValid(kstate, req.path_constraints, req.group_name, true);

              // compute the contacts if any
              collision_detection::CollisionRequest c_req;
              collision_detection::CollisionResult c_res;
              c_req.contacts = true;
              c_req.max_contacts = 10;
              c_req.max_contacts_per_pair = 3;
              c_req.verbose = false;
              planning_scene->checkCollision(c_req, c_res, kstate);
              if (c_res.contact_count > 0)
              {
                visualization_msgs::MarkerArray arr_i;
                collision_detection::getCollisionMarkersFromContacts(arr_i, planning_scene->getPlanningFrame(),
                                                                     c_res.contacts);
                arr.markers.insert(arr.markers.end(), arr_i.markers.begin(), arr_i.markers.end());
              }
            }
            ROS_ERROR_STREAM("Completed listing of explanations for invalid states.");
            if (!arr.markers.empty())
              contacts_publisher_.publish(arr);
          }
        }
        else
          ROS_DEBUG("Planned path was found to be valid, except for states that were added by planning request "
                    "adapters, but that is ok.");
      }
      else
        ROS_DEBUG("Planned path was found to be valid when rechecked");
    }
  }

  // display solution path if needed
  if (display_computed_motion_plans_ && solved)
  {
    moveit_msgs::DisplayTrajectory disp;
    disp.model_id = kmodel_->getName();
    disp.trajectory.resize(1);
    res.trajectory_->getRobotTrajectoryMsg(disp.trajectory[0]);
    robot_state::robotStateToRobotStateMsg(res.trajectory_->getFirstWayPoint(), disp.trajectory_start);
    display_path_publisher_.publish(disp);
  }

  return solved && valid;
}

void planning_pipeline::PlanningPipeline::terminate() const
{
  if (planner_instance_)
    planner_instance_->terminate();
}
