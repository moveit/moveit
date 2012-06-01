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

/* Author: Ioan Sucan, E. Gil Jones */

#include "planning_models_loader/kinematic_model_loader.h"
#include <ros/ros.h>

planning_models_loader::KinematicModelLoader::KinematicModelLoader(const std::string &robot_description, const std::string &root_link)
{
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(robot_description));
  if (robot_model_loader_->getURDF())
  {
    const boost::shared_ptr<srdf::Model> &srdf = robot_model_loader_->getSRDF() ? robot_model_loader_->getSRDF() : boost::shared_ptr<srdf::Model>(new srdf::Model());
    if (root_link.empty())
      model_.reset(new planning_models::KinematicModel(robot_model_loader_->getURDF(), srdf));
    else
      model_.reset(new planning_models::KinematicModel(robot_model_loader_->getURDF(), srdf, root_link));
  }
  
  if (model_)
  {
    // if there are additional joint limits specified in some .yaml file, read those in
    ros::NodeHandle nh("~");
    std::map<std::string, std::vector<moveit_msgs::JointLimits> > individual_joint_limits_map;

    for (unsigned int i = 0; i < model_->getJointModels().size() ; ++i)
    {
      std::vector<moveit_msgs::JointLimits> jlim = model_->getJointModels()[i]->getVariableLimits();
      for(unsigned int j = 0; j < jlim.size(); ++j)
      {
        std::string prefix = robot_model_loader_->getRobotDescription() + "_planning/joint_limits/" + jlim[j].joint_name + "/";
        ROS_DEBUG_STREAM("Joint " << jlim[j].joint_name << " before vel " << jlim[j].max_velocity);
        bool has_vel = jlim[j].has_velocity_limits;
        nh.param(prefix + "has_velocity_limits", has_vel, has_vel);
        jlim[j].has_velocity_limits = has_vel;
        nh.param(prefix + "max_velocity", jlim[j].max_velocity, jlim[j].max_velocity);
        bool has_accel = jlim[j].has_acceleration_limits;
        nh.param(prefix + "has_acceleration_limits", has_accel, has_accel); 
        jlim[j].has_acceleration_limits = has_accel;
        nh.param(prefix + "max_acceleration", jlim[j].max_acceleration, jlim[j].max_acceleration);
        ROS_DEBUG_STREAM("Joint " << jlim[j].joint_name << " after vel " << jlim[j].max_velocity);
        ROS_DEBUG_STREAM("Joint " << jlim[j].joint_name << " accel " << jlim[j].max_acceleration);
      }
      model_->getJointModels()[i]->setLimits(jlim);
      individual_joint_limits_map[model_->getJointModels()[i]->getName()] = jlim;
    }
    const std::map<std::string, planning_models::KinematicModel::JointModelGroup*> &jmgm = model_->getJointModelGroupMap();
    for (std::map<std::string, planning_models::KinematicModel::JointModelGroup*>::const_iterator it = jmgm.begin() ; it != jmgm.end() ; ++it) 
    {
      std::vector<moveit_msgs::JointLimits> group_joint_limits;
      for(unsigned int i = 0; i < it->second->getJointModelNames().size(); i++)
      {
        group_joint_limits.insert(group_joint_limits.end(),
                                  individual_joint_limits_map[it->second->getJointModelNames()[i]].begin(),
                                  individual_joint_limits_map[it->second->getJointModelNames()[i]].end());
      }
      it->second->setJointLimits(group_joint_limits);
    }

    // load the kinematics solvers     
    kinematics_loader_.reset(new kinematics_plugin_loader::KinematicsPluginLoader(robot_model_loader_->getRobotDescription()));
    kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_loader_->getLoaderFunction(robot_model_loader_->getSRDF());
    const std::vector<std::string> &groups = kinematics_loader_->getKnownGroups();
    std::map<std::string, planning_models::KinematicModel::SolverAllocatorFn> imap;
    for (std::size_t i = 0 ; i < groups.size() ; ++i)
      imap[groups[i]] = kinematics_allocator;
    model_->setKinematicsAllocators(imap);
  }
}

std::map<std::string, kinematics::KinematicsBasePtr> planning_models_loader::KinematicModelLoader::generateKinematicsSolversMap(void) const
{
  std::map<std::string, kinematics::KinematicsBasePtr> result;
  const std::vector<std::string> &groups = kinematics_loader_->getKnownGroups();
  for (std::size_t i = 0 ; i < groups.size() ; ++i)
  {
    const planning_models::KinematicModel::JointModelGroup *jmg = model_->getJointModelGroup(groups[i]);
    planning_models::KinematicModel::SolverAllocatorFn a = jmg->getSolverAllocators().first;
    if (a)
      result[jmg->getName()] = a(jmg);
  }
  return result;
}
