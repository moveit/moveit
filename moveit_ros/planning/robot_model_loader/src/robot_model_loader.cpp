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

/* Author: Ioan Sucan, E. Gil Jones */

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/profiler/profiler.h>
#include <ros/ros.h>
#include <typeinfo>

robot_model_loader::RobotModelLoader::RobotModelLoader(const std::string& robot_description,
                                                       bool load_kinematics_solvers)
{
  Options opt(robot_description);
  opt.load_kinematics_solvers_ = load_kinematics_solvers;
  configure(opt);
}

robot_model_loader::RobotModelLoader::RobotModelLoader(const Options& opt)
{
  configure(opt);
}

robot_model_loader::RobotModelLoader::~RobotModelLoader()
{
  // Make sure we destroy the robot model first. It contains the loaded
  // kinematics plugins, and those must be destroyed before the pluginlib class
  // that implements them is destroyed (that happens when kinematics_loader_ is
  // destroyed below). This is a workaround - ideally pluginlib would handle
  // this better.
  model_.reset();
  rdf_loader_.reset();
  kinematics_loader_.reset();
}

namespace
{
bool canSpecifyPosition(const robot_model::JointModel* jmodel, const unsigned int index)
{
  bool ok = false;
  if (jmodel->getType() == robot_model::JointModel::PLANAR && index == 2)
    ROS_ERROR("Cannot specify position limits for orientation of planar joint '%s'", jmodel->getName().c_str());
  else if (jmodel->getType() == robot_model::JointModel::FLOATING && index > 2)
    ROS_ERROR("Cannot specify position limits for orientation of floating joint '%s'", jmodel->getName().c_str());
  else if (jmodel->getType() == robot_model::JointModel::REVOLUTE &&
           static_cast<const robot_model::RevoluteJointModel*>(jmodel)->isContinuous())
    ROS_ERROR("Cannot specify position limits for continuous joint '%s'", jmodel->getName().c_str());
  else
    ok = true;
  return ok;
}
}

void robot_model_loader::RobotModelLoader::configure(const Options& opt)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RobotModelLoader::configure");

  ros::WallTime start = ros::WallTime::now();
  if (opt.urdf_doc_ && opt.srdf_doc_)
    rdf_loader_.reset(new rdf_loader::RDFLoader(opt.urdf_doc_, opt.srdf_doc_));
  else if (!opt.urdf_string_.empty() && !opt.srdf_string_.empty())
    rdf_loader_.reset(new rdf_loader::RDFLoader(opt.urdf_string_, opt.srdf_string_));
  else
    rdf_loader_.reset(new rdf_loader::RDFLoader(opt.robot_description_));
  if (rdf_loader_->getURDF())
  {
    const srdf::ModelSharedPtr& srdf =
        rdf_loader_->getSRDF() ? rdf_loader_->getSRDF() : srdf::ModelSharedPtr(new srdf::Model());
    model_.reset(new robot_model::RobotModel(rdf_loader_->getURDF(), srdf));
  }

  if (model_ && !rdf_loader_->getRobotDescription().empty())
  {
    moveit::tools::Profiler::ScopedBlock prof_block2("RobotModelLoader::configure joint limits");

    // if there are additional joint limits specified in some .yaml file, read those in
    ros::NodeHandle nh("~");

    for (std::size_t i = 0; i < model_->getJointModels().size(); ++i)
    {
      robot_model::JointModel* jmodel = model_->getJointModels()[i];
      std::vector<moveit_msgs::JointLimits> jlim = jmodel->getVariableBoundsMsg();
      for (std::size_t j = 0; j < jlim.size(); ++j)
      {
        std::string prefix = rdf_loader_->getRobotDescription() + "_planning/joint_limits/" + jlim[j].joint_name + "/";

        double max_position;
        if (nh.getParam(prefix + "max_position", max_position))
        {
          if (canSpecifyPosition(jmodel, j))
          {
            jlim[j].has_position_limits = true;
            jlim[j].max_position = max_position;
          }
        }
        double min_position;
        if (nh.getParam(prefix + "min_position", min_position))
        {
          if (canSpecifyPosition(jmodel, j))
          {
            jlim[j].has_position_limits = true;
            jlim[j].min_position = min_position;
          }
        }
        double max_velocity;
        if (nh.getParam(prefix + "max_velocity", max_velocity))
        {
          jlim[j].has_velocity_limits = true;
          jlim[j].max_velocity = max_velocity;
        }
        bool has_vel_limits;
        if (nh.getParam(prefix + "has_velocity_limits", has_vel_limits))
          jlim[j].has_velocity_limits = has_vel_limits;

        double max_acc;
        if (nh.getParam(prefix + "max_acceleration", max_acc))
        {
          jlim[j].has_acceleration_limits = true;
          jlim[j].max_acceleration = max_acc;
        }
        bool has_acc_limits;
        if (nh.getParam(prefix + "has_acceleration_limits", has_acc_limits))
          jlim[j].has_acceleration_limits = has_acc_limits;
      }
      jmodel->setVariableBounds(jlim);
    }
  }

  if (model_ && opt.load_kinematics_solvers_)
    loadKinematicsSolvers();

  ROS_DEBUG_STREAM_NAMED("robot_model_loader", "Loaded kinematic model in " << (ros::WallTime::now() - start).toSec()
                                                                            << " seconds");
}

void robot_model_loader::RobotModelLoader::loadKinematicsSolvers(
    const kinematics_plugin_loader::KinematicsPluginLoaderPtr& kloader)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RobotModelLoader::loadKinematicsSolvers");

  if (rdf_loader_ && model_)
  {
    // load the kinematics solvers
    if (kloader)
      kinematics_loader_ = kloader;
    else
      kinematics_loader_.reset(
          new kinematics_plugin_loader::KinematicsPluginLoader(rdf_loader_->getRobotDescription()));
    robot_model::SolverAllocatorFn kinematics_allocator = kinematics_loader_->getLoaderFunction(rdf_loader_->getSRDF());
    const std::vector<std::string>& groups = kinematics_loader_->getKnownGroups();
    std::stringstream ss;
    std::copy(groups.begin(), groups.end(), std::ostream_iterator<std::string>(ss, " "));
    ROS_DEBUG_STREAM("Loaded information about the following groups: '" << ss.str() << "'");
    if (groups.empty() && !model_->getJointModelGroups().empty())
      ROS_WARN("No kinematics plugins defined. Fill and load kinematics.yaml!");

    std::map<std::string, robot_model::SolverAllocatorFn> imap;
    for (std::size_t i = 0; i < groups.size(); ++i)
    {
      // Check if a group in kinematics.yaml exists in the srdf
      if (!model_->hasJointModelGroup(groups[i]))
        continue;

      const robot_model::JointModelGroup* jmg = model_->getJointModelGroup(groups[i]);

      kinematics::KinematicsBasePtr solver = kinematics_allocator(jmg);
      if (solver)
      {
        std::string error_msg;
        if (solver->supportsGroup(jmg, &error_msg))
        {
          imap[groups[i]] = kinematics_allocator;
        }
        else
        {
          ROS_ERROR("Kinematics solver %s does not support joint group %s.  Error: %s", typeid(*solver).name(),
                    groups[i].c_str(), error_msg.c_str());
        }
      }
      else
      {
        ROS_ERROR("Kinematics solver could not be instantiated for joint group %s.", groups[i].c_str());
      }
    }
    model_->setKinematicsAllocators(imap);

    // set the default IK timeouts
    const std::map<std::string, double>& timeout = kinematics_loader_->getIKTimeout();
    for (std::map<std::string, double>::const_iterator it = timeout.begin(); it != timeout.end(); ++it)
    {
      if (!model_->hasJointModelGroup(it->first))
        continue;
      robot_model::JointModelGroup* jmg = model_->getJointModelGroup(it->first);
      jmg->setDefaultIKTimeout(it->second);
    }

    // set the default IK attempts
    const std::map<std::string, unsigned int>& attempts = kinematics_loader_->getIKAttempts();
    for (std::map<std::string, unsigned int>::const_iterator it = attempts.begin(); it != attempts.end(); ++it)
    {
      if (!model_->hasJointModelGroup(it->first))
        continue;
      robot_model::JointModelGroup* jmg = model_->getJointModelGroup(it->first);
      jmg->setDefaultIKAttempts(it->second);
    }
  }
}
