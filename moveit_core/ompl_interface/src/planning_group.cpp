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

#include <ompl_interface/planning_group.h>

bool ompl_interface::PlanningGroup::computePlan(arm_navigation_msgs::GetMotionPlan::Request &request, 
						arm_navigation_msgs::GetMotionPlan::Response &response)
{
    // first we need to identify what kind of planning we will perform
    if (request.motion_plan_request.goal_constraints.joint_constraints.empty() &&
	request.motion_plan_request.goal_constraints.position_constraints.empty() &&
	request.motion_plan_request.goal_constraints.orientation_constraints.empty() &&
	request.motion_plan_request.goal_constraints.visibility_constraints.empty())
    {
	ROS_WARN("No goal constraints specified");
	return false;
    }
    
    // based on the goal constraints, decide on a goal sampling function
    ompl::base::GoalSamplingFn gsf;
    
    for (std::size_t p = 0 ; p < request.motion_plan_request.goal_constraints.position_constraints.size() ; ++p)
	if (jpmg_->supportsIK(request.motion_plan_request.goal_constraints.position_constraints[p].link_name))
	    for (std::size_t o = 0 ; o < request.motion_plan_request.goal_constraints.orientation_constraints.size() ; ++o)
		if (request.motion_plan_request.goal_constraints.position_constraints[p].link_name == 
		    request.motion_plan_request.goal_constraints.orientation_constraints[o].link_name)
		    gsf = boost::bind(&PlanningGroup::samplingFuncIKPO, this,
				      boost::cref(request.motion_plan_request.goal_constraints.position_constraints[p]),
				      boost::cref(request.motion_plan_request.goal_constraints.orientation_constraints[o]), _1, _2);
    if (!gsf)
	for (std::size_t p = 0 ; p < request.motion_plan_request.goal_constraints.position_constraints.size() ; ++p)
	    if (jpmg_->supportsIK(request.motion_plan_request.goal_constraints.position_constraints[p].link_name))
		gsf = boost::bind(&PlanningGroup::samplingFuncIKP, this,
				  boost::cref(request.motion_plan_request.goal_constraints.position_constraints[p]), _1, _2);	
    if (!gsf)
	for (std::size_t o = 0 ; o < request.motion_plan_request.goal_constraints.orientation_constraints.size() ; ++o)
	    if (jpmg_->supportsIK(request.motion_plan_request.goal_constraints.orientation_constraints[o].link_name))
		gsf = boost::bind(&PlanningGroup::samplingFuncIKO, this,
				  boost::cref(request.motion_plan_request.goal_constraints.orientation_constraints[o]), _1, _2);	
    if (!gsf)
	gsf = boost::bind(&PlanningGroup::samplingFunc, this,
			  boost::cref(request.motion_plan_request.goal_constraints.joint_constraints), _1, _2);
    
    if (!request.motion_plan_request.path_constraints.joint_constraints.empty() ||
	!request.motion_plan_request.path_constraints.position_constraints.empty() ||
	!request.motion_plan_request.path_constraints.orientation_constraints.empty() ||
	!request.motion_plan_request.path_constraints.visibility_constraints.empty())
    {
	// we need a new sampler for the state space
    }
    else
	sspace_->getStateSpace()->clearStateSamplerAllocator();
    
    // set up the collision checker 


    // We construct a GoalLazySamples that attempts to generate samples
    // that satisfy the rest of the constraints
    ompl::base::GoalLazySamples *goal = new ompl::base::GoalLazySamples(ssetup_.getSpaceInformation(), gsf);
    
									
    
}

bool ompl_interface::PlanningGroup::samplingFuncIKP(const arm_navigation_msgs::PositionConstraint &pc,
						    const ompl::base::GoalLazySamples *gls, ompl::base::State *newGoal)
{
    // sample a quaternion
    //    run IK


    ssetup_.getStateSpace()->enforceBounds(newGoal);
}

bool ompl_interface::PlanningGroup::samplingFuncIKO(const arm_navigation_msgs::OrientationConstraint &oc,
						    const ompl::base::GoalLazySamples *gls, ompl::base::State *newGoal)
{
    // sample a state
    // find the orientation

    ssetup_.getStateSpace()->enforceBounds(newGoal);
}

bool ompl_interface::PlanningGroup::samplingFuncIKPO(const arm_navigation_msgs::PositionConstraint &pc,
						     const arm_navigation_msgs::PositionConstraint &oc,
						     const ompl::base::GoalLazySamples *gls, ompl::base::State *newGoal)
{
    //    run IK

    ssetup_.getStateSpace()->enforceBounds(newGoal);
}

bool ompl_interface::PlanningGroup::samplingFunc(const std::vector<arm_navigation_msgs::JointConstraint> &joint_constraints,
						 const ompl::base::GoalLazySamples *gls, ompl::base::State *newGoal)
{
    if (gls->maxSampleCount() > 10)
	return false;
    
    // sample a random state
    state_sampler_.sampleUniform(newGoal);

    // enforce the constraints for the constrained components (could be all of them)
    for (std::size_t i = 0 ; i < joint_constraints.size() ; ++i)
    {
	const JointModel *joint = jmg_->getJointModel(joint_constraints[i].joint_name);
	if (joint == NULL)
	{
	    ROS_ERROR_STREAM("Constraint specified for joint '" << joint_constraints[i].joint_name << "' but no such joint exists in group '" << jmg_->getName() << "'");
	    continue;
	}
	std::pair<double, double> bounds;
	joint->getVariableBounds(joint->getName(), bounds);
	double *value = state_space_.getOMPLStateValueAddress(joint_constraints[i].joint_name, newGoal);
	*value = rng_.uniformReal(std::max(bounds.first, joint_constraints[i].position - joint_constraints[i].tolerance_below),
				  std::min(bounds.second, joint_constraints[i].position + joint_constraints[i].tolerance_above));
    }
    ssetup_.getStateSpace()->enforceBounds(newGoal);
    return true;
}

bool ompl_interface::PlanningGroup::setStart(arm_navigation_msgs::GetMotionPlan::Request &request,
					     arm_navigation_msgs::GetMotionPlan::Response &response)
{
    //
}
