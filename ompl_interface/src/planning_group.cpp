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
#include <kinematic_constraints/kinematic_constraint.h>

ompl_interface::PlanningGroup::PlanningGroup(ompl::StateSpaceCollection &ssc, const planning_models::KinematicModel::JointModelGroup *jmg,
					     const planning_models::Transforms &tf) : 
    jmg_(jmg), state_space_(ssc, jmg), ssetup_(state_space_.getOMPLSpace()), state_sampler_(ssetup_.getStateSpace()->allocStateSampler()),
    tf_(tf), max_goal_samples_(10), max_goal_sampling_attempts_(10000)
{
}

ompl_interface::PlanningGroup::~PlanningGroup(void)
{
}

namespace ompl_interface
{
    
    struct PlanningGroup::IK_Data
    {
	IK_Data(const planning_models::Transforms &tf, const moveit_msgs::PositionConstraint &position, const moveit_msgs::OrientationConstraint &orientation) : tf_(tf)
	{
	    usePosition(position);
	    useOrientation(orientation);
	}

	IK_Data(const planning_models::Transforms &tf, const moveit_msgs::PositionConstraint &position) : tf_(tf)
	{
	    usePosition(position);
	    have_orientation_ = false;
	}

	IK_Data(const planning_models::Transforms &tf, const moveit_msgs::OrientationConstraint &orientation) : tf_(tf)
	{
	    useOrientation(orientation);
	    have_position_ = false;
	}
	
	void usePosition(const moveit_msgs::PositionConstraint &pc)
	{
	    shapes::Shape *shape = shapes::constructShapeFromMsg(pc.constraint_region_shape);
	    if (shape)
	    {
		region_body_.reset(bodies::createBodyFromShape(shape));
		if (region_body_)
		{
		    link_name_ = pc.link_name;
		    offset_.setX(pc.target_point_offset.x);
		    offset_.setY(pc.target_point_offset.y);
		    offset_.setZ(pc.target_point_offset.z);
		    btQuaternion q;
		    if (!planning_models::quatFromMsg(pc.constraint_region_pose.pose.orientation, q))
			ROS_WARN("Incorrect specification of orientation for constraint region on link '%s'", link_name_.c_str());
		    
		    region_pose_ = btTransform(q, btVector3(pc.constraint_region_pose.pose.position.x,
							    pc.constraint_region_pose.pose.position.y,
							    pc.constraint_region_pose.pose.position.z));
		    if (!tf_.isFixedFrame(pc.constraint_region_pose.header.frame_id))
			ROS_WARN("The goal constraint is specified with respect to a mobile frame (with respect to the robot). This is not supported.");
		    
		    tf_.transformTransform(region_pose_, region_pose_, pc.constraint_region_pose.header.frame_id);
		    have_position_ = true;
		}
	    }
	}
	
	void useOrientation(const moveit_msgs::OrientationConstraint &oc)
	{
	    have_orientation_ = true;
	    link_name_ = oc.link_name;
	    btQuaternion q;
	    if (!planning_models::quatFromMsg(oc.orientation.quaternion, q))
		ROS_WARN("Incorrect specification of orientation for link '%s'", link_name_.c_str());
	    if (!tf_.isFixedFrame(oc.orientation.header.frame_id))
		ROS_WARN("The goal constraint is specified with respect to a mobile frame (with respect to the robot). This is not supported.");
	    tf_.transformQuaternion(q, q, oc.orientation.header.frame_id);
	    link_orientation_ = btMatrix3x3(q);
	    roll_tol = oc.absolute_roll_tolerance;
	    pitch_tol = oc.absolute_pitch_tolerance;
	    yaw_tol = oc.absolute_yaw_tolerance;
	}
	
	const planning_models::Transforms &tf_;
	std::string                        link_name_;
	btVector3                          offset_;
	boost::shared_ptr<bodies::Body>    region_body_;
	btTransform                        region_pose_;	
	btMatrix3x3                        link_orientation_;
	double                             roll_tol, pitch_tol, yaw_tol;
	bool                               have_position_;
	bool                               have_orientation_;
    };
    
    struct PlanningGroup::J_Data
    {
	J_Data(const planning_models::KinematicModel::JointModelGroup *model, const std::vector<moveit_msgs::JointConstraint> &jc) : jc_(jc)
	{
	    constrained_joints_count_ = 0;
	    for (std::size_t i = 0 ; i < jc.size() ; ++i)
	    {
		const planning_models::KinematicModel::JointModel *jm = model->getJointModel(jc[i].joint_name);
		if (jm && jm->getVariableCount() != 1)
		{
		    ROS_WARN("Only joints that have 1 DOF are supported in the specification of joint constraints. Joint '%s' has %u DOF and will be ignored",
			     jm->getName().c_str(), jm->getVariableCount());
		    jm = NULL;
		}
		std::pair<double, double> bounds(0.0, 0.0);
		if (jm)
		{
		    constrained_joints_count_++;
		    jm->getVariableBounds(jm->getName(), bounds);
		}
		joints_.push_back(jm);
		bounds_.push_back(bounds);
	    }
	}

	const std::vector<moveit_msgs::JointConstraint>                &jc_;
	std::vector<const planning_models::KinematicModel::JointModel*> joints_;
	std::vector<std::pair<double, double> >                         bounds_;
	unsigned int                                                    constrained_joints_count_;
    };

    class ConstrainedGoalRegion : public ompl::base::GoalRegion
    {
    public:
	ConstrainedGoalRegion(const ompl::base::SpaceInformationPtr &si, const KMStateSpace &kspace, const planning_models::KinematicState &start_state,
			      const kinematic_constraints::KinematicConstraintSetPtr &ks) : ompl::base::GoalRegion(si), ks_(ks), kspace_(kspace), start_state_(start_state)
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
	
	kinematic_constraints::KinematicConstraintSetPtr                      ks_;
	const KMStateSpace                                                   &kspace_;
	planning_models::KinematicState                                       start_state_;
	mutable std::map<boost::thread::id, planning_models::KinematicState*> thread_states_;
	mutable boost::mutex                                                  lock_;
    };
    
}

bool ompl_interface::PlanningGroup::samplingFuncIK(IK_Data *data, const ompl::base::GoalLazySamples *gls, ompl::base::State *newGoal)
{
    //    if (gls->getStatesCount() >= max_goal_samples_)   ENABLE
    //	return false;
    if (gls->samplingAttemptsCount() >= max_goal_sampling_attempts_)
	return false;
    if (data->have_position_ && data->have_orientation_)
    {
    }
    else
	if (data->have_position_ && !data->have_orientation_)
	{
	    // will sample random orientations
	    


	}
	else
	{
	    // will sample random positions
	    
	}
    
    ssetup_.getStateSpace()->enforceBounds(newGoal);
}

bool ompl_interface::PlanningGroup::samplingFuncJ(J_Data *data, const ompl::base::GoalLazySamples *gls, ompl::base::State *newGoal)
{
    //    if (gls->getStatesCount() >= max_goal_samples_) ENABLE
    //	return false;
    if (gls->samplingAttemptsCount() >= max_goal_sampling_attempts_)
	return false;
    
    // sample a random state
    state_sampler_->sampleUniform(newGoal);
    
    // enforce the constraints for the constrained components (could be all of them)
    for (std::size_t i = 0 ; i < data->joints_.size() ; ++i)
    {
	if (data->joints_[i] == NULL)
	    continue;
	double *value = state_space_.getOMPLStateValueAddress(data->jc_[i].joint_name, newGoal);
	*value = rng_.uniformReal(std::max(data->bounds_[i].first, data->jc_[i].position - data->jc_[i].tolerance_below),
				  std::min(data->bounds_[i].second, data->jc_[i].position + data->jc_[i].tolerance_above));
    }
    return true;
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
    planning_models::KinematicState start(current_state);
    planning_models::robotStateToKinematicState(tf_, start_state, start);
    
    // \TODO set the collision world to this full robot state


    // convert the input state to the corresponding OMPL state    
    ompl::base::ScopedState<> ompl_start_state(state_space_.getOMPLSpace());
    state_space_.copyToOMPLState(ompl_start_state.get(), start);
    ssetup_.setStartState(ompl_start_state);
    
    // ******************* set up the sampler (based on path constraints)    

    if (!path_constraints.joint_constraints.empty() ||
	!path_constraints.position_constraints.empty() ||
	!path_constraints.orientation_constraints.empty())
    {
	// we need a new sampler for the state space
    }
    //    else ENABLE
    //	ssetup_.getStateSpace()->clearStateSamplerAllocator();
    




    // ******************* set up the goal representation, based on goal constraints

    // based on the goal constraints, decide on a goal representation
    boost::scoped_ptr<IK_Data> ik_data;
    boost::scoped_ptr<J_Data>  j_data;
    
    // if we have position and/or orientation constraints on links that we can perform IK for,
    // we will use a sampleable goal region that employs IK to sample goals
    for (std::size_t p = 0 ; p < goal_constraints.position_constraints.size() ; ++p)
	if (jmg_->supportsIK(goal_constraints.position_constraints[p].link_name))
	    for (std::size_t o = 0 ; o < goal_constraints.orientation_constraints.size() ; ++o)
		if (goal_constraints.position_constraints[p].link_name == goal_constraints.orientation_constraints[o].link_name)
		    ik_data.reset(new IK_Data(tf_, goal_constraints.position_constraints[p], goal_constraints.orientation_constraints[o]));
    if (!ik_data)
	for (std::size_t p = 0 ; p < goal_constraints.position_constraints.size() ; ++p)
	    if (jmg_->supportsIK(goal_constraints.position_constraints[p].link_name))
		ik_data.reset(new IK_Data(tf_, goal_constraints.position_constraints[p]));
    if (!ik_data)
	for (std::size_t o = 0 ; o < goal_constraints.orientation_constraints.size() ; ++o)
	    if (jmg_->supportsIK(goal_constraints.orientation_constraints[o].link_name))
		ik_data.reset(new IK_Data(tf_, goal_constraints.orientation_constraints[o]));

    // if we cannot perform IK but there are joint constraints that we can use to construct goal samples,
    // we again use a sampleable goal region
    if (!ik_data && !goal_constraints.joint_constraints.empty())
    {
	j_data.reset(new J_Data(jmg_, goal_constraints.joint_constraints));
	// if there are no valid constraints, just ignore the constraints
	if (j_data->constrained_joints_count_ == 0)
	    j_data.reset();
    }
    
    if (ik_data || j_data)
    {
	ompl::base::GoalSamplingFn gsf;
	if (ik_data)
	    gsf = boost::bind(&PlanningGroup::samplingFuncIK, this, ik_data.get(), _1, _2);
	else
	    gsf = boost::bind(&PlanningGroup::samplingFuncJ, this, j_data.get(), _1, _2);
	ssetup_.setGoal(ompl::base::GoalPtr(new ompl::base::GoalLazySamples(ssetup_.getSpaceInformation(), gsf)));
    }
    else
    {
	// the constraints are such that we cannot easily sample the goal region,
	// so we use a simpler goal region specification
	kinematic_constraints::KinematicConstraintSetPtr ks(new kinematic_constraints::KinematicConstraintSet(*start.getKinematicModel(), tf_));
	ks->add(goal_constraints);
	ssetup_.setGoal(ompl::base::GoalPtr(new ConstrainedGoalRegion(ssetup_.getSpaceInformation(), state_space_, start, ks)));
    }
    
    return true;    
}


