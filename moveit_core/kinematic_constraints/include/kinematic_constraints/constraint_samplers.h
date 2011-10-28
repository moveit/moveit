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

#ifndef KINEMATIC_CONSTRAINTS_CONSTRAINT_SAMPLERS_
#define KINEMATIC_CONSTRAINTS_CONSTRAINT_SAMPLERS_

#include "kinematic_constraints/kinematic_constraint.h"

namespace kinematic_constraints
{
    
    class ConstraintSampler
    {
    public:
	ConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg) : jmg_(jmg)
	{
	}

	virtual ~ConstraintSampler(void)
	{
	}
	
	virtual bool sample(std::vector<double> &values) = 0;
	
    protected:
	
	const planning_models::KinematicModel::JointModelGroup *jmg_;
	planning_models::RNG                                    rng_;
    };
    
	
    class JointConstraintSampler : public ConstraintSampler
    {
    public:
	
	JointConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg, const std::vector<JointConstraint> &jc);
	virtual bool sample(std::vector<double> &values);

    protected:
	
	std::vector<JointConstraint>            jc_;	
	std::vector<std::pair<double, double> > bounds_;
	std::vector<unsigned int>               index_;
	
	std::vector<const planning_models::KinematicModel::JointModel*> unbounded_;
	std::vector<unsigned int>                                       uindex_;
    };
	
    class IKConstraintSampler : public ConstraintSampler
    {
    public:
	IKConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg,
			    const PositionConstraint &pc, const OrientationConstraint &oc);
	
	IKConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg,
			    const PositionConstraint &pc);

	IKConstraintSampler(const planning_models::KinematicModel::JointModelGroup *jmg,
			    const OrientationConstraint &oc);

	virtual bool sample(std::vector<double> &values);

    protected:
	
	bool                  have_position_;
	bool                  have_orientation_;
	boost::shared_ptr<PositionConstraint>    pc_;
	boost::shared_ptr<OrientationConstraint> oc_;
	
    };
    
}


#endif
