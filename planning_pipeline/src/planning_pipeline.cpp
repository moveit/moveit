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
*   * Neither the name of the Willow Garage nor the names of its
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

#include "planning_pipeline/planning_pipeline.h"
#include <boost/tokenizer.hpp>
#include <sstream>
#include <ros/ros.h>

planning_pipeline::PlanningPipeline::PlanningPipeline(const planning_models::KinematicModelConstPtr& model, 
                                                      const std::string &planner_plugin_param_name,
                                                      const std::string &adapter_plugins_param_name)
{
  ros::NodeHandle nh("~");
  std::string planner;
  if (nh.getParam(planner_plugin_param_name, planner))
    planner_plugin_name_ = planner;
  
  std::string adapters;
  if (nh.getParam("request_adapters", adapters))
  { 
    boost::char_separator<char> sep(" ");
    boost::tokenizer<boost::char_separator<char> > tok(adapters, sep);
    for(boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin() ; beg != tok.end(); ++beg)
      adapter_plugin_names_.push_back(*beg);
  }
  
  configure(model);
}

planning_pipeline::PlanningPipeline::PlanningPipeline(const planning_models::KinematicModelConstPtr& model, 
                                                      const std::string &planner_plugin_name,
                                                      const std::vector<std::string> &adapter_plugin_names) :
  planner_plugin_name_(planner_plugin_name), adapter_plugin_names_(adapter_plugin_names)
{
  configure(model);
}

void planning_pipeline::PlanningPipeline::configure(const planning_models::KinematicModelConstPtr& model)
{
  // load the planning plugin
  try
  {
    planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::Planner>("planning_interface", "planning_interface::Planner"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }

  const std::vector<std::string> &classes = planner_plugin_loader_->getDeclaredClasses();
  if (planner_plugin_name_.empty() && classes.size() == 1)
  {
    planner_plugin_name_ = classes[0];
    ROS_INFO("No 'planning_plugin' parameter specified, but only '%s' planning plugin is available. Using that one.", planner_plugin_name_.c_str());
  }
  if (planner_plugin_name_.empty() && classes.size() > 1)
  {      
    planner_plugin_name_ = classes[0];   
    ROS_INFO("Multiple planning plugins available. You shuold specify the 'planning_plugin' parameter. Using '%s' for now.", planner_plugin_name_.c_str());
  }
  try
  {
    planner_instance_.reset(planner_plugin_loader_->createUnmanagedInstance(planner_plugin_name_));
    planner_instance_->init(model);
    ROS_INFO_STREAM("Using planning interface '" << planner_instance_->getDescription() << "'");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    std::stringstream ss;
    for (std::size_t i = 0 ; i < classes.size() ; ++i)
      ss << classes[i] << " ";
    ROS_FATAL_STREAM("Exception while loading planner '" << planner_plugin_name_ << "': " << ex.what() << std::endl
                     << "Available plugins: " << ss.str());
  }
  
  // load the planner request adapters
  if (!adapter_plugin_names_.empty())
  {   
    std::vector<planning_request_adapter::PlanningRequestAdapterConstPtr> ads;
    try
    {
      adapter_plugin_loader_.reset(new pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter>("planning_request_adapter", "planning_request_adapter::PlanningRequestAdapter"));
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    
    for (std::size_t i = 0 ; i < adapter_plugin_names_.size() ; ++i)
    {
      planning_request_adapter::PlanningRequestAdapterConstPtr ad;
      try
      {
        ad.reset(adapter_plugin_loader_->createUnmanagedInstance(adapter_plugin_names_[i]));
      }
      catch (pluginlib::PluginlibException& ex)
      {
        ROS_ERROR_STREAM("Exception while planning adapter plugin '" << adapter_plugin_names_[i] << "': " << ex.what());
      }
      if (ad)
        ads.push_back(ad);
    }
    if (!ads.empty())
    {
      adapter_chain_.reset(new planning_request_adapter::PlanningRequestAdapterChain());
      for (std::size_t i = 0 ; i < ads.size() ; ++i)
      {
        ROS_INFO_STREAM("Using planning request adapter '" << ads[i]->getDescription() << "'");
        adapter_chain_->addAdapter(ads[i]);
      }
    }
  }
}

bool planning_pipeline::PlanningPipeline::generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                       const moveit_msgs::GetMotionPlan::Request& req,
                                                       moveit_msgs::GetMotionPlan::Response& res) const
{
  bool solved = false;
  try
  {
    if (adapter_chain_)
      solved = adapter_chain_->adaptAndPlan(planner_instance_, planning_scene, req, res);
    else
      solved = planner_instance_->solve(planning_scene, req, res);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Exception caught: '%s'", ex.what());
    return false;
  }
  catch(...)
  {
    ROS_ERROR("Unknown exception thrown by planner");
    return false;
  }
  
  if (solved)
  {
    trajectory_msgs::JointTrajectory trajectory_out;
    const std::vector<moveit_msgs::JointLimits> &jlim = planning_scene->getKinematicModel()->getJointModelGroup(req.motion_plan_request.group_name)->getAdditionalLimits();
    smoother_.smooth(res.trajectory.joint_trajectory, trajectory_out, jlim);
    res.trajectory.joint_trajectory = trajectory_out; /// \todo apply this for the RobotTrajectory; is this the right place for this operation?
  }
  
  return solved;
}
