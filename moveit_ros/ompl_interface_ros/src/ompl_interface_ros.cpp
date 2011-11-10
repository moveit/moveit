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
    class IKLoader
    {
    public:
	IKLoader(const std::map<std::string, std::vector<std::string> > &possible_ik_solvers) : possible_ik_solvers_(possible_ik_solvers)
	{
	    kinematics_loader_.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("kinematics_base", "kinematics::KinematicsBase"));
	}
	
	boost::shared_ptr<kinematics::KinematicsBase> allocIKSolver(const planning_models::KinematicModel::JointModelGroup *jmg)
	{
	    boost::shared_ptr<kinematics::KinematicsBase> result;
	    if (jmg)
	    {
		std::map<std::string, std::vector<std::string> >::const_iterator it = possible_ik_solvers_.find(jmg->getName());
		if (it != possible_ik_solvers_.end())
		{
		    // just to be sure, do not call the same pluginlib instance allocation function in parallel
		    boost::mutex::scoped_lock slock(lock_);
		    if (kinematics_loader_)
		    {
			for (std::size_t i = 0 ; !result && i < it->second.size() ; ++i)
			{
			    try
			    {
				result.reset(kinematics_loader_->createClassInstance(it->second[i]));
				if (result)
				    if (!result->initialize(jmg->getName()))
				    {
					ROS_ERROR("IK solver (%s) could not initialize", it->first.c_str());
					result.reset();
				    }
			    }
			    catch (pluginlib::PluginlibException& e)
			    {
				ROS_ERROR("The IK plugin (%s) failed to load. Error: %s", it->first.c_str(), e.what());
			    }
			}
		    }
		}
	    }
	    return result;
	}

    private:
	
	const std::map<std::string, std::vector<std::string> >                 possible_ik_solvers_;
	boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematics_loader_;
	boost::mutex                                                           lock_;
    };
}

ompl_interface_ros::OMPLInterfaceROS::OMPLInterfaceROS(const std::string &robot_description) : ompl_interface::OMPLInterface(), nh_("~")
{
    planning_scene_ = new planning_scene_ros::PlanningSceneROS(robot_description);
    planning_scene_ptr_.reset(planning_scene_);
    if (planning_scene_->isConfigured())
    {
	if (configurePlanners())
	{
	    configureIKSolvers();
	    plan_service_ = nh_.advertiseService("plan_kinematic_path", &OMPLInterfaceROS::computePlan, this);
	}
    }
}

void ompl_interface_ros::OMPLInterfaceROS::configureIKSolvers(void)
{
    const std::string &rd = planning_scene_->getRobotDescription(); // this one is resolved & remapped
    std::map<std::string, std::vector<std::string> > possible_ik_solvers;  

    // read from param server

    ik_loader_.reset(new IKLoader(possible_ik_solvers));
    kinematic_constraints::IKAllocator ik_allocator = boost::bind(&IKLoader::allocIKSolver, ik_loader_.get(), _1);
    for (std::map<std::string, ompl_interface::PlanningGroupPtr>::iterator it = planning_groups_.begin() ; it != planning_groups_.end() ; ++it)
	it->second->setIKAllocator(ik_allocator);
}
 
bool ompl_interface_ros::OMPLInterfaceROS::configurePlanners(void)
{
    const std::string &rd = planning_scene_->getRobotDescription(); // this one is resolved & remapped
    std::vector<ompl_interface::PlannerConfigs> pconfig;
    // read from param server
    return configure(planning_scene_ptr_, pconfig);
}

bool ompl_interface_ros::OMPLInterfaceROS::computePlan(moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res)
{
    return solve(req, res);
}

void ompl_interface_ros::OMPLInterfaceROS::run(void)
{
    if (isConfigured())
    {
        std::stringstream ss;
        planning_scene_->getKinematicModel()->printModelInfo(ss);
        ROS_INFO("%s", ss.str().c_str());
        ROS_INFO("OMPL planning node started.");
    }
    else
    {
        ROS_ERROR("Cannot start OMPL planning node.");
    }
}
