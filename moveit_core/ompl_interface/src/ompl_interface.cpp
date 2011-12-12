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

#include "ompl_interface/ompl_interface.h"
#include <planning_models/conversions.h>
#include <algorithm>
#include <fstream>
#include <set>

bool ompl_interface::OMPLInterface::configure(const planning_scene::PlanningSceneConstPtr &scene, const std::vector<PlannerConfigs> &pconfig)
{
    scene_ = scene;
    if (!scene_ || !scene_->isConfigured())
    {
        ROS_ERROR("Cannot configure OMPL interface without configured planning scene");
        return false;
    }

    // this will be the set of joints not covered by any groups
    std::set<const planning_models::KinematicModel::JointModel*> rest;
    rest.insert(scene_->getKinematicModel()->getJointModels().begin(), scene_->getKinematicModel()->getJointModels().end());
    fullSpace_.reset();

    // construct specified configurations
    for (std::size_t i = 0 ; i < pconfig.size() ; ++i)
    {
        const planning_models::KinematicModel::JointModelGroup *jmg = scene_->getKinematicModel()->getJointModelGroup(pconfig[i].group);
        if (jmg)
            planning_groups_[pconfig[i].name].reset(new PlanningGroup(pconfig[i].name, jmg, pconfig[i].config, scene_));
    }
    // construct default configurations
    const std::map<std::string, planning_models::KinematicModel::JointModelGroup*>& groups = scene_->getKinematicModel()->getJointModelGroupMap();
    for (std::map<std::string, planning_models::KinematicModel::JointModelGroup*>::const_iterator it = groups.begin() ; it != groups.end() ; ++it)
    {
        if (planning_groups_.find(it->first) == planning_groups_.end())
        {
            static const std::map<std::string, std::string> empty;
            planning_groups_[it->first].reset(new PlanningGroup(it->first, it->second, empty, scene_));
        }

        fullSpace_ = fullSpace_ + planning_groups_[it->first]->getKMStateSpace().getOMPLSpace();
        const std::vector<const planning_models::KinematicModel::JointModel*> &js = it->second->getJointModels();
        for (std::size_t k = 0 ; k < js.size() ; ++k)
            rest.erase(js[k]);
    }

    // finish the construction of the full state space for the robot

    if (!rest.empty())
    {
        std::vector<const planning_models::KinematicModel::JointModel*> js;
        js.insert(js.end(), rest.begin(), rest.end());
        KMStateSpace dummy(js);
        dummy.getOMPLSpace()->setName(scene_->getKinematicModel()->getName() + "_ungrouped");
        fullSpace_ = fullSpace_ + dummy.getOMPLSpace();
    }
    if (fullSpace_)
	fullSpace_->setName(scene_->getKinematicModel()->getName());

    configured_ = true;
    return true;
}

void ompl_interface::OMPLInterface::configureIKSolvers(const std::map<std::string, kinematic_constraints::IKAllocator> &ik_allocators)
{
    for (std::map<std::string, PlanningGroupPtr>::iterator it = planning_groups_.begin() ; it != planning_groups_.end() ; ++it)
    {
	std::map<std::string, kinematic_constraints::IKAllocator>::const_iterator jt = ik_allocators.find(it->second->getJointModelGroup()->getName());
	if (jt == ik_allocators.end())
	{
	    // if an IK allocator is NOT available for this group, we try to see if we can use subgroups for IK
	    const planning_models::KinematicModel::JointModelGroup *jmg = it->second->getJointModelGroup();
	    const planning_models::KinematicModel *km = jmg->getKinematicModel();
	    std::set<const planning_models::KinematicModel::JointModel*> joints;
	    joints.insert(jmg->getJointModels().begin(), jmg->getJointModels().end());
	    
	    std::vector<const planning_models::KinematicModel::JointModelGroup*> subs;
	    
	    // go through the groups that we know have IK allocators and see if they are included in the group that does not; fi so, put that group in sub
	    for (std::map<std::string, kinematic_constraints::IKAllocator>::const_iterator kt = ik_allocators.begin() ; kt != ik_allocators.end() ; ++kt)
	    {
		const planning_models::KinematicModel::JointModelGroup *sub = km->getJointModelGroup(kt->first);
		const std::vector<const planning_models::KinematicModel::JointModel*> &sub_joints = sub->getJointModels();
		if (std::includes(joints.begin(), joints.end(), sub_joints.begin(), sub_joints.end()))
		{
		    subs.push_back(sub);
		    for (std::size_t i = 0 ; i < sub_joints.size() ; ++i)
			joints.erase(sub_joints[i]);
		}
	    }
	    
	    // if we found subgroups, pass that information to the planning group
	    if (!subs.empty())
	    {
		PlanningGroup::SubgroupAllocators sa;
		for (std::size_t i = 0 ; i < subs.size() ; ++i)
		    sa.ik_allocators_[i].push_back(std::make_pair(sub[i], *ik_allocators.find(sub[i]->getName())));
		for (std::set<const planning_models::KinematicModel::JointModel*>::iterator kt = joints.begin() ; kt != joints.end() ; ++kt)
		    sa.remainder_joints_.push_back(*kt);
		ik_subgroup_allocators_.push_back(sa);
	    }
	}
	else
	    // if the IK allocator is for this group, we use it
	    it->second->ik_allocator_ = jt->second;
    }
}

void ompl_interface::OMPLInterface::setMaximumSamplingAttempts(unsigned int max_sampling_attempts)
{
    for (std::map<std::string, PlanningGroupPtr>::iterator it = planning_groups_.begin() ; it != planning_groups_.end() ; ++it)
        it->second->setMaximumSamplingAttempts(max_sampling_attempts);
}

void ompl_interface::OMPLInterface::setMaximumGoalSamples(unsigned int max_goal_samples)
{
    for (std::map<std::string, PlanningGroupPtr>::iterator it = planning_groups_.begin() ; it != planning_groups_.end() ; ++it)
        it->second->setMaximumGoalSamples(max_goal_samples);
}

void ompl_interface::OMPLInterface::setMaximumPlanningThreads(unsigned int max_planning_threads)
{
    for (std::map<std::string, PlanningGroupPtr>::iterator it = planning_groups_.begin() ; it != planning_groups_.end() ; ++it)
        it->second->setMaximumPlanningThreads(max_planning_threads);
}

bool ompl_interface::OMPLInterface::solve(const moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res) const
{
    if (req.motion_plan_request.group_name.empty())
    {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
        ROS_ERROR("No group specified to plan for");
        return false;
    }

    // identify the correct planning group
    std::map<std::string, PlanningGroupPtr>::const_iterator pg = planning_groups_.end();
    if (!req.motion_plan_request.planner_id.empty())
    {
        pg = planning_groups_.find(req.motion_plan_request.group_name + "." + req.motion_plan_request.planner_id);
        if (pg == planning_groups_.end())
            ROS_WARN_STREAM("Cannot find planning configuration for group '" << req.motion_plan_request.group_name
                            << "' using planner '" << req.motion_plan_request.planner_id << "'. Will use defaults instead.");
    }
    if (pg == planning_groups_.end())
    {
        pg = planning_groups_.find(req.motion_plan_request.group_name);
        if (pg == planning_groups_.end())
        {
            res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
            ROS_ERROR_STREAM("Cannot find planning configuration for group '" << req.motion_plan_request.group_name << "'");
            return false;
        }
    }

    // configure the planning group

    // get the starting state
    planning_models::KinematicState ks = scene_->getCurrentState();
    planning_models::robotStateToKinematicState(*scene_->getTransforms(), req.motion_plan_request.start_state, ks);

    if (!pg->second->setupPlanningContext(ks, req.motion_plan_request.goal_constraints, req.motion_plan_request.path_constraints, &res.error_code))
        return false;
    pg->second->setPlanningVolume(req.motion_plan_request.workspace_parameters);

    // solve the planning problem
    double timeout = req.motion_plan_request.allowed_planning_time.toSec();
    if (timeout <= 0.0)
    {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ALLOWED_PLANNING_TIME;
        ROS_ERROR("The timeout for planning must be positive (%lf specified)", timeout);
    }
    else
    {
        unsigned int att = 1;
        if (req.motion_plan_request.num_planning_attempts > 0)
            att = req.motion_plan_request.num_planning_attempts;
        else
            if (req.motion_plan_request.num_planning_attempts < 0)
                ROS_ERROR("The number of desired planning attempts should be positive");
        if (pg->second->solve(timeout, att))
        {
            double ptime = pg->second->getLastPlanTime();
            if (ptime < timeout)
                pg->second->simplifySolution(timeout - ptime);
	    pg->second->interpolateSolution();
            pg->second->fillResponse(res);
            return true;
        }
        else
            ROS_INFO("Unable to solve the planning problem");
    }
    return false;
}

bool ompl_interface::OMPLInterface::solve(const std::string &config, const planning_models::KinematicState &start_state, const moveit_msgs::Constraints &goal_constraints, double timeout)
{
    moveit_msgs::Constraints empty;
    return solve(config, start_state, goal_constraints, empty, timeout);
}

bool ompl_interface::OMPLInterface::solve(const std::string &config, const planning_models::KinematicState &start_state, const moveit_msgs::Constraints &goal_constraints,
                                          const moveit_msgs::Constraints &path_constraints, double timeout)
{
    std::map<std::string, PlanningGroupPtr>::const_iterator pg = planning_groups_.find(config);
    if (pg == planning_groups_.end())
    {
        ROS_ERROR("Planner configuration '%s' not found", config.c_str());
        return false;
    }

    // configure the planning group
    std::vector<moveit_msgs::Constraints> goal_constraints_v(1, goal_constraints);
    if (!pg->second->setupPlanningContext(start_state, goal_constraints_v, path_constraints))
        return false;

    // solve the planning problem
    if (pg->second->solve(timeout, 1))
    {
        double ptime = pg->second->getLastPlanTime();
        if (ptime < timeout)
            pg->second->simplifySolution(timeout - ptime);
	pg->second->interpolateSolution();
	return true;
    }

    return false;
}

const ompl_interface::PlanningGroupPtr& ompl_interface::OMPLInterface::getPlanningConfiguration(const std::string &config) const
{
    std::map<std::string, PlanningGroupPtr>::const_iterator pg = planning_groups_.find(config);
    if (pg == planning_groups_.end())
    {
        ROS_ERROR("Planner configuration '%s' not found", config.c_str());
        static PlanningGroupPtr empty;
        return empty;
    }
    else
        return pg->second;
}
