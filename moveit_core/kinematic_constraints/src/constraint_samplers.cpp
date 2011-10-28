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

bool kinematic_constraints::JointConstraintSampler::sample(std::vector<double> &values)
{
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
    ConstraintSampler(jmg), have_position_(true), have_orientation_(true), pc_(new PositionConstraint(pc)), oc_(new OrientationConstraint(oc))
{
    
}

	
kinematic_constraints::IKConstraintSampler::IKConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg,
								const PositionConstraint &pc) :
    ConstraintSampler(jmg), have_position_(true), have_orientation_(false), pc_(new PositionConstraint(pc))
{

}


kinematic_constraints::IKConstraintSampler::IKConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg,
								const OrientationConstraint &oc) :
    ConstraintSampler(jmg), have_position_(false), have_orientation_(true), oc_(new OrientationConstraint(oc))
{

}

bool kinematic_constraints::IKConstraintSampler::sample(std::vector<double> &values)
{
    
    return false;
}
