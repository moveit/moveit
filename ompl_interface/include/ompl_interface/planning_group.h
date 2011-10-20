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

#ifndef OMPL_INTERFACE_PLANNING_GROUP_
#define OMPL_INTERFACE_PLANNING_GROUP_

#include <ompl/tools/spaces/StateSpaceCollection.h>
#include <ompl/geometric/SimpleSetup.h>
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <ompl_interface/state_space.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <boost/shared_ptr.hpp>

namespace ompl_interface
{
    
    class PlanningGroup
    {
    public:
	
	PlanningGroup(ompl::StateSpaceCollection &ssc, const planning_models::KinematicModel::JointModelGroup *jmg) :
	    jmg_(jmg), state_space_(ssc, jmg->getJointModels()), ssetup_(state_space_.getOMPLSpace()), state_sampler_(ssetup_.getStateSpace()->allocStateSampler())
	{
	}
	
	virtual ~PlanningGroup(void)
	{
	}

	/* @brief Return the name of the group this planner is operating on */
	std::string getName()
	{
	    return jmg_->getName();
	};

	bool computePlan(arm_navigation_msgs::GetMotionPlan::Request &request, 
			 arm_navigation_msgs::GetMotionPlan::Response &response);

    protected:

	/*
	  @brief Check whether the request is valid. This function must be implemented by every derived class.
	  @param The motion planning request
	  @param The motion planner response
	*/
	virtual bool isRequestValid(arm_navigation_msgs::GetMotionPlan::Request &request,
				    arm_navigation_msgs::GetMotionPlan::Response &response);
	
	/*
	  @brief Set the start. This function must be implemented by every derived class.
	  @param The motion planning request
	  @param The motion planner response
	*/
	virtual bool setStart(arm_navigation_msgs::GetMotionPlan::Request &request,
			      arm_navigation_msgs::GetMotionPlan::Response &response);
	
	/*
	  @brief Set the start. This function must be implemented by every derived class.
	  @param The motion planning request
	  @param The motion planner response
	*/
	virtual bool setGoal(arm_navigation_msgs::GetMotionPlan::Request &request,
			     arm_navigation_msgs::GetMotionPlan::Response &response);
	
	
	const planning_models::KinematicModel::JointModelGroup *jmg_;
	KMStateSpace                                            state_space_;
	ompl::geometric::SimpleSetup                            ssetup_;	
	ompl::base::StateSamplerPtr                             state_sampler_;
	ompl::RNG                                               rng_;
    };
    
    typedef boost::shared_ptr<PlanningGroup> PlanningGroupPtr;
}

#endif
