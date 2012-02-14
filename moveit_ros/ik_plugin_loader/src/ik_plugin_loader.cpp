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

/* Author: Ioan Sucan */

#include "ik_plugin_loader/ik_plugin_loader.h"
#include <pluginlib/class_loader.h>
#include <boost/thread/mutex.hpp>
#include <sstream>
#include <vector>
#include <map>
#include <ros/ros.h>

namespace ik_plugin_loader
{
class IKPluginLoader::IKLoaderImpl
{
public:
  IKLoaderImpl(const std::map<std::string, std::vector<std::string> > &possible_ik_solvers, 
               const std::map<std::string, std::vector<double> > &search_res) :
    possible_ik_solvers_(possible_ik_solvers), search_res_(search_res)
  {
    try
    {
      kinematics_loader_.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("kinematics_base", "kinematics::KinematicsBase"));
    }
    catch(pluginlib::PluginlibException& e)
    {
      ROS_ERROR("Unable to construct IK loader. Error: %s", e.what());
    }
  }
  
  boost::shared_ptr<kinematics::KinematicsBase> allocIKSolver(const planning_models::KinematicModel::JointModelGroup *jmg)
  {
    boost::shared_ptr<kinematics::KinematicsBase> result;
    ROS_DEBUG("Received request to allocate IK solver for group '%s'", jmg->getName().c_str());
    if (kinematics_loader_ && jmg)
    {
      std::map<std::string, std::vector<std::string> >::const_iterator it = possible_ik_solvers_.find(jmg->getName());
      if (it != possible_ik_solvers_.end())
      {
        // just to be sure, do not call the same pluginlib instance allocation function in parallel
        boost::mutex::scoped_lock slock(lock_);
        for (std::size_t i = 0 ; !result && i < it->second.size() ; ++i)
        {
          try
          {
            result.reset(kinematics_loader_->createClassInstance(it->second[i]));
            if (result)
            {
              /// \todo What is the search discretization? any reasonable way to determine this?
              const std::vector<const planning_models::KinematicModel::JointModel*> &jnts = jmg->getJointModels();
              const std::string &base = jnts.front()->getParentLinkModel() ? jnts.front()->getParentLinkModel()->getName() :
                jmg->getParentModel()->getModelFrame();
              const std::string &tip = jnts.back()->getChildLinkModel()->getName();
              double search_res = search_res_.find(jmg->getName())->second[i]; // we know this exists, by construction
              if (!result->initialize(jmg->getName(), base, tip, search_res))
              {
                ROS_ERROR("IK solver of type '%s' could not initialize for group '%s'", it->second[i].c_str(), jmg->getName().c_str());
                result.reset();
              }
              else
                ROS_DEBUG("Successfully allocated and initialized an IK solver of type '%s' with search resolution %lf for group '%s' at address %p",
                          it->second[i].c_str(), search_res, jmg->getName().c_str(), result.get());
            }
          }
          catch (pluginlib::PluginlibException& e)
          {
            ROS_ERROR("The IK plugin (%s) failed to load. Error: %s", it->first.c_str(), e.what());
          }
        }
      }
      else
        ROS_DEBUG("No IK solver available for this group");
    }
    return result;
  }
  
private:
  
  const std::map<std::string, std::vector<std::string> >                 possible_ik_solvers_;
  const std::map<std::string, std::vector<double> >                      search_res_;
  boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematics_loader_;
  boost::mutex                                                           lock_;
};

}

ik_plugin_loader::IKLoaderFn ik_plugin_loader::IKPluginLoader::getLoaderFunction(void)
{
  if (!loader_)
  {
    ROS_INFO("Configuring IK solvers");
    groups_.clear();
    
    ros::NodeHandle nh("~");
    XmlRpc::XmlRpcValue group_list;
    std::vector<std::string> group_names;
    
    // read the list of group names that have additional configurations
    if (nh.getParam("groups", group_list))
      if (group_list.getType() == XmlRpc::XmlRpcValue::TypeArray)      
        for (int32_t i = 0; i < group_list.size(); ++i)
          if (group_list[i].getType() == XmlRpc::XmlRpcValue::TypeString)
          {
            std::string gnm = static_cast<std::string>(group_list[i]);
            group_names.push_back(gnm);
          }
    std::map<std::string, std::vector<std::string> > possible_ik_solvers;
    std::map<std::string, std::vector<double> > search_res;
    
    // read the list of plugin names for possible IK solvers
    for (std::size_t i = 0 ; i < group_names.size() ; ++i)
    {
      std::string ksolver;
      if (nh.getParam(group_names[i] + "/kinematics_solver", ksolver))
      {
        std::stringstream ss(ksolver);
        bool first = true;
        while (ss.good() && !ss.eof())
        {
          if (first)
          {
            first = false;
            groups_.push_back(group_names[i]);
          }
          std::string solver; ss >> solver >> std::ws;          
          possible_ik_solvers[group_names[i]].push_back(solver);
          ROS_INFO("Using IK solver '%s' for group '%s'.", solver.c_str(), group_names[i].c_str());
        }
      }
      
      std::string ksolver_res;
      if (nh.getParam(group_names[i] + "/kinematics_solver_search_resolution", ksolver_res))
      {
        std::stringstream ss(ksolver_res);
        while (ss.good() && !ss.eof())
        {
          double res; ss >> res >> std::ws;
          search_res[group_names[i]].push_back(res);
        }
      }

      // make sure there is a default resolution at least specified for every solver (in case it was not specified on the param server)
      while (search_res[group_names[i]].size() < possible_ik_solvers[group_names[i]].size())
        search_res[group_names[i]].push_back(0.1);
    }
    loader_.reset(new IKLoaderImpl(possible_ik_solvers, search_res));
  }
  
  return boost::bind(&IKPluginLoader::IKLoaderImpl::allocIKSolver, loader_.get(), _1);
}
