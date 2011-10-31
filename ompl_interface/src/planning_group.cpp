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
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>
#include <planning_models/conversions.h>

ompl_interface::PlanningGroup::PlanningGroup(ompl::StateSpaceCollection &ssc, const planning_models::KinematicModelPtr &kmodel,
					     const planning_models::KinematicModel::JointModelGroup *jmg, const planning_models::Transforms &tf) : 
    kmodel_(kmodel), jmg_(jmg), state_space_(ssc, jmg), ssetup_(state_space_.getOMPLSpace()), tf_(tf),
    max_goal_samples_(10), max_sampling_attempts_(10000), start_state_(kmodel)
{
}

ompl_interface::PlanningGroup::~PlanningGroup(void)
{
}

namespace ompl_interface
{

    // we could do this without defining a new class, but this way we have a nice container for the constraint sampler as well
    class ConstrainedGoalSampler : public ompl::base::GoalLazySamples
    {
    public:
	ConstrainedGoalSampler(const ompl::base::SpaceInformationPtr &si, const KMStateSpace &kspace, const planning_models::KinematicState &start_state,
			       unsigned int max_sampling_attempts,  unsigned int max_goal_samples, const kinematic_constraints::ConstraintSamplerPtr &cs) : 
	    ompl::base::GoalLazySamples(si, boost::bind(&ConstrainedGoalSampler::sampleC, this, _1, _2), false),
	    kss_(kspace), start_state_(start_state), max_sampling_attempts_(max_sampling_attempts), max_goal_samples_(max_goal_samples_), cs_(cs)
	{
	    startSampling();
	}
	
    private:
	
	bool sampleC(const ompl::base::GoalLazySamples *gls, ompl::base::State *newGoal)
	{
	    if (gls->samplingAttemptsCount() >= max_sampling_attempts_)
		return false;
	    if (gls->getStateCount() >= max_goal_samples_)
		return false;
	    std::vector<double> values;
	    if (cs_->sample(values, max_sampling_attempts_, &start_state_))
	    {
		kss_.copyToOMPLState(newGoal, values);
		return true;
	    }
	    return false;
	}
	
	const KMStateSpace                         &kss_;
	planning_models::KinematicState             start_state_;
	unsigned int                                max_sampling_attempts_;
	unsigned int                                max_goal_samples_;
	kinematic_constraints::ConstraintSamplerPtr cs_;
    };
    
    class ConstrainedGoalRegion : public ompl::base::GoalRegion
    {
    public:
	ConstrainedGoalRegion(const ompl::base::SpaceInformationPtr &si, const KMStateSpace &kspace, const planning_models::KinematicState &start_state,
			      const kinematic_constraints::KinematicConstraintSetPtr &ks) : ompl::base::GoalRegion(si), kspace_(kspace), start_state_(start_state), ks_(ks)
	{
	}
	
	virtual ~ConstrainedGoalRegion(void)
	{
	    for (std::map<boost::thread::id, planning_models::KinematicState*>::iterator it = thread_states_.begin() ; it != thread_states_.end() ; ++it)
		delete it->second;
	}
	
	planning_models::KinematicState* getStateStorage(void) const
	{
	    planning_models::KinematicState *st = NULL;
	    lock_.lock();
	    std::map<boost::thread::id, planning_models::KinematicState*>::const_iterator it = thread_states_.find(boost::this_thread::get_id());
	    if (it == thread_states_.end())
	    {
		st = new planning_models::KinematicState(start_state_);
		thread_states_[boost::this_thread::get_id()] = st;
	    }
	    else
		st = it->second;
	    lock_.unlock();
	    return st;
	}
	
	virtual double distanceGoal(const ompl::base::State *st) const
	{
	    planning_models::KinematicState *s = getStateStorage();
	    kspace_.copyToKinematicState(*s, st);
	    const std::pair<bool, double> &r = ks_->decide(*s);
	    return r.second;
	}
	
	virtual bool isSatisfied(const ompl::base::State *st, double *distance) const
	{
	    planning_models::KinematicState *s = getStateStorage();
	    kspace_.copyToKinematicState(*s, st);
	    const std::pair<bool, double> &r = ks_->decide(*s);
	    if (distance)
		*distance = r.second;
	    return r.first;	    
	}
	
    protected:
	
	const KMStateSpace                                                   &kspace_;
	planning_models::KinematicState                                       start_state_;
	kinematic_constraints::KinematicConstraintSetPtr                      ks_;
	mutable std::map<boost::thread::id, planning_models::KinematicState*> thread_states_;
	mutable boost::mutex                                                  lock_;
    };
    
    class ConstrainedSampler : public ompl::base::StateSampler
    {
    public:
	ConstrainedSampler(const KMStateSpace &kspace, const planning_models::KinematicState &start_state,
			   unsigned int max_attempts, const kinematic_constraints::ConstraintSamplerPtr &cs) : 
	    ompl::base::StateSampler(kspace.getOMPLSpace().get()), kspace_(kspace), default_(kspace.getOMPLSpace()->allocDefaultStateSampler()),
	    start_state_(start_state), max_attempts_(max_attempts), cs_(cs)
	{
	}	
	
	bool sampleC(ompl::base::State *state)
	{
	    std::vector<double> values;
	    if (cs_->sample(values, max_attempts_, &start_state_))
	    {
		kspace_.copyToOMPLState(state, values);
		return true;
	    }
	    return false;
	}
	
	virtual void sampleUniform(ompl::base::State *state)
	{
	    if (!sampleC(state))
		default_->sampleUniform(state);
	}
	
	virtual void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
	{
	    if (!sampleC(state))
		default_->sampleUniformNear(state, near, distance);
	}
	
	virtual void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev)
	{
	    if (!sampleC(state))
		default_->sampleGaussian(state, mean, stdDev);
	}
	
    private:
	
	const KMStateSpace                         &kspace_;
	ompl::base::StateSamplerPtr                 default_;
	planning_models::KinematicState             start_state_;
	unsigned int                                max_attempts_;
	kinematic_constraints::ConstraintSamplerPtr cs_;	
    };   
}

ompl::base::StateSamplerPtr ompl_interface::PlanningGroup::allocConstrainedSampler(const ompl::base::StateSpace *ss, const moveit_msgs::Constraints *constraints) const
{
    if (state_space_.getOMPLSpace().get() != ss)
	ROS_FATAL("Attempted to allocate a state sampler for an unknown state space");	
    const boost::shared_ptr<kinematic_constraints::ConstraintSampler> &cs = getConstraintsSampler(*constraints);
    if (!cs)
	ROS_FATAL("Unable to allocate constrained sampler");
    return ompl::base::StateSamplerPtr(new ConstrainedSampler(state_space_, start_state_, max_sampling_attempts_, cs));
}

kinematic_constraints::ConstraintSamplerPtr ompl_interface::PlanningGroup::getConstraintsSampler(const moveit_msgs::Constraints &constr) const
{
    // based on the goal constraints, decide on a goal representation
    kinematic_constraints::ConstraintSamplerPtr sampler;
    
    // if we have position and/or orientation constraints on links that we can perform IK for,
    // we will use a sampleable goal region that employs IK to sample goals
    for (std::size_t p = 0 ; p < constr.position_constraints.size() ; ++p)
	if (jmg_->supportsIK(constr.position_constraints[p].link_name))
	    for (std::size_t o = 0 ; o < constr.orientation_constraints.size() ; ++o)
		if (constr.position_constraints[p].link_name == constr.orientation_constraints[o].link_name)
		{
		    boost::scoped_ptr<kinematic_constraints::PositionConstraint> pc(new kinematic_constraints::PositionConstraint(*kmodel_, tf_));
		    boost::scoped_ptr<kinematic_constraints::OrientationConstraint> oc(new kinematic_constraints::OrientationConstraint(*kmodel_, tf_));
		    if (pc->use(constr.position_constraints[p]) && oc->use(constr.orientation_constraints[o]))
			sampler.reset(new kinematic_constraints::IKConstraintSampler(ik_allocator_, jmg_, *pc, *oc));
		}
    
    if (!sampler)
	for (std::size_t p = 0 ; p < constr.position_constraints.size() ; ++p)
	    if (jmg_->supportsIK(constr.position_constraints[p].link_name))
	    {
		boost::scoped_ptr<kinematic_constraints::PositionConstraint> pc(new kinematic_constraints::PositionConstraint(*kmodel_, tf_));
		if (pc->use(constr.position_constraints[p]))
		    sampler.reset(new kinematic_constraints::IKConstraintSampler(ik_allocator_, jmg_, *pc));
	    }
    
    if (!sampler)
	for (std::size_t o = 0 ; o < constr.orientation_constraints.size() ; ++o)
	    if (jmg_->supportsIK(constr.orientation_constraints[o].link_name))
	    {
		boost::scoped_ptr<kinematic_constraints::OrientationConstraint> oc(new kinematic_constraints::OrientationConstraint(*kmodel_, tf_));
		if (oc->use(constr.orientation_constraints[o]))
		    sampler.reset(new kinematic_constraints::IKConstraintSampler(ik_allocator_, jmg_, *oc));
	    }
    
    // if we cannot perform IK but there are joint constraints that we can use to construct goal samples,
    // we again use a sampleable goal region
    if (!sampler && !constr.joint_constraints.empty())
    {
	std::vector<kinematic_constraints::JointConstraint> jc;
	for (std::size_t i = 0 ; i < constr.joint_constraints.size() ; ++i)
	{
	    kinematic_constraints::JointConstraint j(*kmodel_, tf_);
	    if (j.use(constr.joint_constraints[i]))
		jc.push_back(j);
	}
	if (!jc.empty()) 
	    sampler.reset(new kinematic_constraints::JointConstraintSampler(jmg_, jc));
    }
    return sampler;
}

bool ompl_interface::PlanningGroup::setupPlanningContext(const planning_models::KinematicState &current_state,
							 const moveit_msgs::RobotState &start_state,
							 const moveit_msgs::Constraints &goal_constraints, 
							 const moveit_msgs::Constraints &path_constraints)
{    
    // ******************* check if the input is correct 
    
    // first we need to identify what kind of planning we will perform
    if (goal_constraints.joint_constraints.empty() &&
	goal_constraints.position_constraints.empty() &&
	goal_constraints.orientation_constraints.empty())
    {
	ROS_WARN("No goal constraints specified");
	return false;
    }


    // ******************* set up the starting state for the plannig context 
    // get the starting state
    start_state_ = current_state;
    planning_models::robotStateToKinematicState(tf_, start_state, start_state_);
    
    // \TODO set the collision world to this full robot state


    // convert the input state to the corresponding OMPL state    
    ompl::base::ScopedState<> ompl_start_state(state_space_.getOMPLSpace());
    state_space_.copyToOMPLState(ompl_start_state.get(), start_state_);
    ssetup_.setStartState(ompl_start_state);
    
    // ******************* set up the sampler (based on path constraints)    
    path_constraints_ = path_constraints;
    if (getConstraintsSampler(path_constraints))
	// we need a new sampler for the state space
	ssetup_.getStateSpace()->setStateSamplerAllocator(boost::bind(&PlanningGroup::allocConstrainedSampler, this, _1, &path_constraints_));
    else
    	ssetup_.getStateSpace()->clearStateSampleAllocator();
    

    // ******************* set up the goal representation, based on goal constraints

    // first, we add path constraints to the goal ones
    goal_constraints_ = kinematic_constraints::mergeConstraints(goal_constraints, path_constraints);
    const kinematic_constraints::ConstraintSamplerPtr &gsampler = getConstraintsSampler(goal_constraints_);
    if (gsampler)
	ssetup_.setGoal(ompl::base::GoalPtr(new ConstrainedGoalSampler(ssetup_.getSpaceInformation(), state_space_, start_state_, max_sampling_attempts_, max_goal_samples_, gsampler)));
    else
    {
	// the constraints are such that we cannot easily sample the goal region,
	// so we use a simpler goal region specification
	kinematic_constraints::KinematicConstraintSetPtr ks(new kinematic_constraints::KinematicConstraintSet(*kmodel_, tf_));
	ks->add(goal_constraints_);
	ssetup_.setGoal(ompl::base::GoalPtr(new ConstrainedGoalRegion(ssetup_.getSpaceInformation(), state_space_, start_state_, ks)));
    }
    
    return true;    
}


