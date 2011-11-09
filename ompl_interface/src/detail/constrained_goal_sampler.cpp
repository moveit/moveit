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

/* Author: Ioan Sucan */

#include "ompl_interface/detail/constrained_goal_sampler.h"

ompl_interface::ConstrainedGoalSampler::ConstrainedGoalSampler(const PlanningGroup *pg, const kinematic_constraints::KinematicConstraintSetPtr &ks,
                                                               const kinematic_constraints::ConstraintSamplerPtr &cs) :
    ompl::base::GoalLazySamples(pg->getPlanningContext().ssetup_.getSpaceInformation(), boost::bind(&ConstrainedGoalSampler::sampleC, this, _1, _2), false),
    pg_(pg), ks_(ks), cs_(cs), tss_(*pg->getPlanningContext().start_state_)
{
    startSampling();
}

bool ompl_interface::ConstrainedGoalSampler::sampleC(const ompl::base::GoalLazySamples *gls, ompl::base::State *newGoal)
{
    unsigned int ma = pg_->getMaximumSamplingAttempts();

    // terminate after too many attempts
    if (gls->samplingAttemptsCount() >= ma)
        return false;
    // terminate after a maximum number of samples
    if (gls->getStateCount() >= pg_->getMaximumGoalSamples())
        return false;
    // terminate the sampling thread when a solution has been found
    if (gls->isAchieved())
        return false;

    planning_models::KinematicState *s = tss_.getStateStorage();
    std::vector<double> values;
    for (unsigned int a = 0 ; a < ma && gls->isSampling() ; ++a)
        if (cs_->sample(values, ma, pg_->getPlanningContext().start_state_.get()))
        {
            s->getJointStateGroup(pg_->getJointModelGroup()->getName())->setStateValues(values);
            if (ks_->decide(*s).first)
            {
                pg_->getKMStateSpace().copyToOMPLState(newGoal, values);
                return true;
            }
        }
    return false;
}
