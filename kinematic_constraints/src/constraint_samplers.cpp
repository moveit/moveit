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

/** \author Ioan Sucan */

#include "kinematic_constraints/constraint_samplers.h"
#include <boost/math/constants/constants.hpp>
#include <ros/console.h>
#include <algorithm>

kinematic_constraints::JointConstraintSampler::JointConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg,
								      const std::vector<JointConstraint> &jc) : ConstraintSampler(jmg)
{
    // find and keep the constraints that operate on the group we sample
    // also keep bounds for joints as convenient
    const std::map<std::string, unsigned int> &vim = jmg_->getJointVariablesIndexMap();
    std::set<const planning_models::KinematicModel::JointModel*> bounded;
    for (std::size_t i = 0 ; i < jc.size() ; ++i)
    {
	if (!jc[i].enabled())
	    continue;
	const planning_models::KinematicModel::JointModel *jm = jc[i].getJointModel();
	if (!jmg_->hasJointModel(jm->getName()))
	    continue;
	bounded.insert(jm);
	std::pair<double, double> bounds;
	jm->getVariableBounds(jm->getName(), bounds);
	bounds_.push_back(bounds);
	index_.push_back(vim.find(jm->getName())->second);
	jc_.push_back(jc[i]);
    }

    // get a separate list of joints that are not bounded; we will sample these randomly
    const std::vector<const planning_models::KinematicModel::JointModel*> &joints = jmg_->getJointModels();
    for (std::size_t i = 0 ; i < joints.size() ; ++i)
	if (bounded.find(joints[i]) == bounded.end())
	{
	    unbounded_.push_back(joints[i]);
	    uindex_.push_back(vim.find(joints[i]->getName())->second);
	}
}

bool kinematic_constraints::JointConstraintSampler::sample(std::vector<double> &values, unsigned int /* max_attempts */, 
							   const planning_models::KinematicState::JointStateGroup * /* jsg */)
{
    values.resize(jmg_->getVariableCount());    
    // enforce the constraints for the constrained components (could be all of them)
    for (std::size_t i = 0 ; i < jc_.size() ; ++i)
	values[index_[i]] = rng_.uniformReal(std::max(bounds_[i].first, jc_[i].getDesiredJointPosition() - jc_[i].getJointToleranceBelow()),
					     std::min(bounds_[i].second, jc_[i].getDesiredJointPosition() + jc_[i].getJointToleranceAbove()));
    
    // sample the rest of the components
    for (std::size_t i = 0 ; i < unbounded_.size() ; ++i)
    {
	std::vector<double> v;
	unbounded_[i]->getRandomValues(rng_, v);
	for (std::size_t j = 0 ; j < v.size() ; ++j)
	    values[uindex_[i] + j] = v[j];
    }
    
    // we are always successful
    return true;
}



kinematic_constraints::IKConstraintSampler::IKConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg,
								const PositionConstraint &pc, const OrientationConstraint &oc) :
    ConstraintSampler(jmg), pc_(new PositionConstraint(pc)), oc_(new OrientationConstraint(oc))
{
    if (pc_->getLinkModel()->getName() != oc_->getLinkModel()->getName())
	ROS_FATAL("Position and orientation constraints need to be specified for the same link in order to use IK-based sampling");
}

	
kinematic_constraints::IKConstraintSampler::IKConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg,
								const PositionConstraint &pc) :
    ConstraintSampler(jmg), pc_(new PositionConstraint(pc))
{

}


kinematic_constraints::IKConstraintSampler::IKConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg,
								const OrientationConstraint &oc) :
    ConstraintSampler(jmg), oc_(new OrientationConstraint(oc))
{

}

bool kinematic_constraints::IKConstraintSampler::sample(std::vector<double> &values, unsigned int max_attempts, 
							const planning_models::KinematicState::JointStateGroup *jsg)
{
    // make sure we at least have a chance of sampling using IK
    if (!pc_ && !oc_)
	return false;
    if (!kb_)
	return false;
    if (pc_)
    {
	if (!jmg_->supportsIK(pc_->getLinkModel()->getName()))
	    return false;	
    }
    else
	if (!jmg_->supportsIK(oc_->getLinkModel()->getName()))
	    return false;	

    for (unsigned int a = 0 ; a < max_attempts ; ++a)
    {
	// sample a point in the constraint region
	btVector3 point;
	if (pc_)
	{
	    if (!pc_->getConstraintRegion()->samplePointInside(rng_, max_attempts, point))
		return false;
	}
	else
	{
	    // do FK for rand state
	    if (!jsg)
	    {
		ROS_WARN("An initial JointStateGroup is needed to sample possible link positions");
		return false;
	    }
	    planning_models::KinematicState ks(*(jsg->getKinematicState()));
	    planning_models::KinematicState::JointStateGroup *tmp = ks.getJointStateGroup(jmg_->getName());
	    if (tmp)
	    {
		tmp->setRandomValues();
		point = ks.getLinkState(pc_->getLinkModel()->getName())->getGlobalLinkTransform().getOrigin();
	    }
	    else
	    {
		ROS_ERROR("Passed a JointStateGroup for a mismatching JointModelGroup");
		return false;
	    }
	}
	
	btQuaternion quat;
	if (oc_)
	{
	    // sample a rotation matrix within the allowed bounds
	    double rpy[3];
	    rng_.eulerRPY(rpy);
	    btMatrix3x3 diff;
	    diff.setEulerYPR(rpy[0] * oc_->getRollTolerance() / boost::math::constants::pi<double>(),
			     rpy[1] * oc_->getPitchTolerance() / boost::math::constants::pi<double>(),
			     rpy[2] * oc_->getYawTolerance() / boost::math::constants::pi<double>());
	    (oc_->getDesiredRotationMatrix() * diff).getRotation(quat);
	}
	else
	{
	    // sample a random orientation
	    double q[4];
	    rng_.quaternion(q);
	    quat = btQuaternion(q[0], q[1], q[2], q[3]);
	}
	
	// we now have the transform we wish to perform IK for
	geometry_msgs::Pose ik_query;
	ik_query.position.x = point.x();
	ik_query.position.y = point.y();
	ik_query.position.z = point.z();
	ik_query.orientation.x = quat.x();
	ik_query.orientation.y = quat.y();
	ik_query.orientation.z = quat.z();
	ik_query.orientation.w = quat.w();
	
	const std::vector<std::string> &ik_jnames = kb_->getJointNames();
	
	// sample a seed value
	std::vector<double> seed;
	if (jsg)
	{
	    planning_models::KinematicState ks(*(jsg->getKinematicState()));
	    planning_models::KinematicState::JointStateGroup *tmp = ks.getJointStateGroup(jmg_->getName());
	    if (tmp)
	    {
		tmp->setRandomValues();	
		std::map<std::string, double> v;
		tmp->getGroupStateValues(v);
		for (std::size_t i = 0 ; i < ik_jnames.size() ; ++i)
		    seed.push_back(v[ik_jnames[i]]);
	    }
	    else
		seed.resize(ik_jnames.size(), 0.0);
	}
	else
	    seed.resize(ik_jnames.size(), 0.0);
	
	std::vector<double> sol;
	int error;
	
	if (kb_->searchPositionIK(ik_query, seed, 0.5, sol, error))
	{
	    std::map<std::string, double> s;
	    for (std::size_t i = 0 ; i < ik_jnames.size() ; ++i)
		s[ik_jnames[i]] = sol[i];
	    const std::vector<std::string> &nm = jmg_->getJointModelNames();
	    values.resize(jmg_->getVariableCount());    
	    for (std::size_t i = 0 ; i < nm.size() ; ++i)
		values[i] = s[nm[i]];
	    return true;
	}
	else
	    ROS_DEBUG("IK solved failed with error %d", error);
    }
    return false;
}
