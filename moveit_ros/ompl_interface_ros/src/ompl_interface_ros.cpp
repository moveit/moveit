/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Sachin Chitta */

#include "ompl_interface_ros/ompl_interface_ros.h"
#include <pluginlib/class_loader.h>
#include <boost/thread/mutex.hpp>
#include <sstream>

namespace ompl_interface_ros
{
    class OMPLInterfaceROS::IKLoader
    {
    public:
        IKLoader(const std::map<std::string, std::vector<std::string> > &possible_ik_solvers) : possible_ik_solvers_(possible_ik_solvers)
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
                                if (!result->initialize(jmg->getName(), base, tip, 0.1))
                                {
                                    ROS_ERROR("IK solver of type '%s' could not initialize for group '%s'", it->second[i].c_str(), jmg->getName().c_str());
                                    result.reset();
                                }
                                else
                                    ROS_DEBUG("Successfully allocated and initialized an IK solver of type '%s' for group '%s' at address %p",
                                              it->second[i].c_str(), jmg->getName().c_str(), result.get());
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
        boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematics_loader_;
        boost::mutex                                                           lock_;
    };
}

ompl_interface_ros::OMPLInterfaceROS::OMPLInterfaceROS(const planning_scene::PlanningSceneConstPtr &scene) : ompl_interface::OMPLInterface(), nh_("~")
{
    ROS_INFO("Initializing OMPL interface using ROS parameters");
    if (scene->isConfigured())
    {
        scene_ = scene;

        std::vector<ompl_interface::PlannerConfigs> pconfig;
        configurePlanners(pconfig);

        // this call will configure planning for all groups known to the kinematic model
        // and it will use additional configuration options, if available
        if (configure(scene, pconfig))
        {
            configureIKSolvers();
            plan_service_ = nh_.advertiseService("plan_kinematic_path", &OMPLInterfaceROS::computePlan, this);
        }
    }
    else
        ROS_ERROR("Planning scene is not configured. Cannot configure OMPL interface.");
}

std::vector<std::string> ompl_interface_ros::OMPLInterfaceROS::getAdditionalConfigGroupNames(void)
{
    XmlRpc::XmlRpcValue group_list;
    std::vector<std::string> group_names;

    // read the list of group names that have additional configurations
    if (nh_.getParam("groups", group_list))
    {
        ROS_INFO("Using additional configurations for planning groups on param server:");
        if (group_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for (int32_t i = 0; i < group_list.size(); ++i)
            {
                if (group_list[i].getType() == XmlRpc::XmlRpcValue::TypeString)
                {
                    std::string gnm = static_cast<std::string>(group_list[i]);
                    if (scene_->getKinematicModel()->hasJointModelGroup(gnm))
                    {
                        ROS_INFO("  - group '%s'", gnm.c_str());
                        group_names.push_back(gnm);
                    }
                    else
                        ROS_ERROR("Additional configuration specified for group '%s', but that group is not known to the kinematic model.", gnm.c_str());
                }
                else
                    ROS_ERROR("Group names should be strings");
            }
        }
        else
            ROS_ERROR("Group list should be of XmlRpc Array type");
    }
    else
        ROS_INFO("No additional configurations for planning groups found on param server");

    return group_names;
}

void ompl_interface_ros::OMPLInterfaceROS::configureIKSolvers(void)
{
    ROS_INFO("Configuring IK solvers");

    std::vector<std::string> group_names = getAdditionalConfigGroupNames();
    std::map<std::string, std::vector<std::string> > possible_ik_solvers;

    // read the list of plugin names for possible IK solvers
    for (std::size_t i = 0 ; i < group_names.size() ; ++i)
    {
        std::string ksolver;
        if (nh_.getParam(group_names[i] + "/kinematics_solver", ksolver))
        {
            std::stringstream ss(ksolver);
            while (ss.good() && !ss.eof())
            {
                std::string solver; ss >> solver >> std::ws;
                possible_ik_solvers[group_names[i]].push_back(solver);
                ROS_INFO("Using IK solver '%s' for group '%s'.", solver.c_str(), group_names[i].c_str());
            }
        }
    }

    std::map<std::string, kinematic_constraints::IKAllocator> imap;
    ik_loader_.reset(new OMPLInterfaceROS::IKLoader(possible_ik_solvers));
    kinematic_constraints::IKAllocator ik_allocator = boost::bind(&OMPLInterfaceROS::IKLoader::allocIKSolver, ik_loader_.get(), _1);
    for (std::map<std::string, std::vector<std::string> >::iterator it = possible_ik_solvers.begin() ; it != possible_ik_solvers.end() ; ++it)
        imap[it->first] = ik_allocator;
    OMPLInterface::configureIKSolvers(imap);
}

void ompl_interface_ros::OMPLInterfaceROS::configurePlanners(std::vector<ompl_interface::PlannerConfigs> &pconfig)
{
    std::vector<std::string> group_names = getAdditionalConfigGroupNames();

    // read the planning configuration for each group
    pconfig.clear();
    for (std::size_t i = 0 ; i < group_names.size() ; ++i)
    {
        // get parameters specific for the group
        std::string projection_evaluator, lvsf;
        nh_.getParam(group_names[i] + "/projection_evaluator", projection_evaluator);
        nh_.getParam(group_names[i] + "/longest_valid_segment_fraction", lvsf);

        // set the parameters (if any) for the default group configuration;
        if (!projection_evaluator.empty() || !lvsf.empty())
        {
            ompl_interface::PlannerConfigs pc;
            pc.name = group_names[i];
            pc.group = group_names[i];
            if (!projection_evaluator.empty())
                pc.config["projection_evaluator"] = projection_evaluator;
            if (!lvsf.empty())
                pc.config["longest_valid_segment_fraction"] = lvsf;
            pconfig.push_back(pc);
        }

        XmlRpc::XmlRpcValue config_names;
        if (nh_.getParam(group_names[i] + "/planner_configs", config_names))
        {
            if (config_names.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                for (int32_t j = 0; j < config_names.size() ; ++j)
                    if (config_names[j].getType() == XmlRpc::XmlRpcValue::TypeString)
                    {
                        std::string planner_config = static_cast<std::string>(config_names[j]);
                        XmlRpc::XmlRpcValue xml_config;
                        if (nh_.getParam("planner_configs/" + planner_config, xml_config))
                        {
                            if (xml_config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                            {
                                ompl_interface::PlannerConfigs pc;
                                pc.name = group_names[i] + "[" + planner_config + "]";
                                pc.group = group_names[i];

                                // inherit parameters from the group (which can be overriden)
                                if (!projection_evaluator.empty())
                                    pc.config["projection_evaluator"] = projection_evaluator;
                                if (!lvsf.empty())
                                    pc.config["longest_valid_segment_fraction"] = lvsf;

                                // read parameters specific for this configuration
                                for (XmlRpc::XmlRpcValue::iterator it = xml_config.begin() ; it != xml_config.end() ; ++it)
                                    pc.config[it->first] = static_cast<std::string>(it->second);
                                pconfig.push_back(pc);
                            }
                            else
                                ROS_ERROR("A planning configuration should be of type XmlRpc Struct type (for configuration '%s')", planner_config.c_str());
                        }
                        else
                            ROS_ERROR("Could not find the planner configuration '%s' on the param server", planner_config.c_str());
                    }
                    else
                        ROS_ERROR("Planner configuration names must be of type string (for group '%s')", group_names[i].c_str());
            }
            else
                ROS_ERROR("The planner_configs argument of a group configuration should be an array of strings (for group '%s')", group_names[i].c_str());
        }
        else
            ROS_INFO("Group '%s' mentioned for additional configuration but no planner_configs specified. Using default settings.", group_names[i].c_str());
    }

    for (std::size_t i = 0 ; i < pconfig.size() ; ++i)
    {
        ROS_DEBUG_STREAM("Parameters for configuration '"<< pconfig[i].name << "'");
        for (std::map<std::string, std::string>::const_iterator it = pconfig[i].config.begin() ; it != pconfig[i].config.end() ; ++it)
            ROS_DEBUG_STREAM(it->first << " = " << it->second);
    }
}

bool ompl_interface_ros::OMPLInterfaceROS::computePlan(moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res)
{
    ROS_INFO("Received new planning request...");
    return solve(req, res);
}

void ompl_interface_ros::OMPLInterfaceROS::status(void)
{
    if (isConfigured())
    {
        //        std::stringstream ss;
        //        scene_->getKinematicModel()->printModelInfo(ss);
        //        ROS_INFO("%s", ss.str().c_str());
        ROS_INFO("OMPL planning node started.");
    }
    else
    {
        ROS_ERROR("Cannot start OMPL planning node.");
    }
}
