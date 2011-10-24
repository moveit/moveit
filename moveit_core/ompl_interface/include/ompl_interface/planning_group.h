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
#include <ompl/base/GoalLazySamples.h>
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_models/transforms.h>
#include <ompl_interface/state_space.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <boost/shared_ptr.hpp>

namespace ompl_interface
{
    
    class PlanningGroup
    {
    public:
	
	PlanningGroup(ompl::StateSpaceCollection &ssc, const planning_models::KinematicModel::JointModelGroup *jmg, const planning_models::Transforms &tf);
	virtual ~PlanningGroup(void);

	/* @brief Return the name of the group this planner is operating on */
	const std::string& getName(void)
	{
	    return jmg_->getName();
	}

	bool setupPlanningContext(const planning_models::KinematicState &current_state,
				  const moveit_msgs::RobotState &start_state,
				  const moveit_msgs::Constraints &goal_constraints, 
				  const moveit_msgs::Constraints &path_constraints);
	
	ompl::geometric::SimpleSetup& getPlanningContext(void)
	{
	    return ssetup_;
	}
	
    protected:
	
	struct J_Data;
	struct IK_Data;
	
	bool samplingFuncJ(J_Data *data, const ompl::base::GoalLazySamples *gls, ompl::base::State *newGoal);
	bool samplingFuncIK(IK_Data *data, const ompl::base::GoalLazySamples *gls, ompl::base::State *newGoal);
	
	const planning_models::KinematicModel::JointModelGroup *jmg_;
	KMStateSpace                                            state_space_;
	ompl::geometric::SimpleSetup                            ssetup_;	
	ompl::base::StateSamplerPtr                             state_sampler_;
	planning_models::Transforms                             tf_;
	
	unsigned int                                            max_goal_samples_;
	unsigned int                                            max_goal_sampling_attempts_;
	
	ompl::RNG                                               rng_;
    };
    
    typedef boost::shared_ptr<PlanningGroup> PlanningGroupPtr;
}

#endif
