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

#include "ompl_interface/constrained_goal_region.h"

ompl_interface::ConstrainedGoalRegion::ConstrainedGoalRegion(const PlanningGroup *pg, const kinematic_constraints::KinematicConstraintSetPtr &ks) :
    ompl::base::GoalRegion(pg->getPlanningContext().ssetup_.getSpaceInformation()), pg_(pg), ks_(ks),
    tss_(*pg->getPlanningContext().start_state_)
{
}

double ompl_interface::ConstrainedGoalRegion::distanceGoal(const ompl::base::State *st) const
{
    planning_models::KinematicState *s = tss_.getStateStorage();
    pg_->getKMStateSpace().copyToKinematicState(*s, st);
    return ks_->decide(*s).second;
}

bool ompl_interface::ConstrainedGoalRegion::isSatisfied(const ompl::base::State *st, double *distance) const
{
    planning_models::KinematicState *s = tss_.getStateStorage();
    pg_->getKMStateSpace().copyToKinematicState(*s, st);
    const std::pair<bool, double> &r = ks_->decide(*s);
    if (distance)
        *distance = r.second;
    return r.first;
}
