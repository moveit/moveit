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

#ifndef OMPL_INTERFACE_STATE_VALIDITY_CHECKER_
#define OMPL_INTERFACE_STATE_VALIDITY_CHECKER_

#include "ompl_interface/planning_group.h"
#include "ompl_interface/detail/threadsafe_state_storage.h"

namespace ompl_interface
{

    class StateValidityChecker : public ompl::base::StateValidityChecker
    {
    public:

        StateValidityChecker(const PlanningGroup *pg) :
            ompl::base::StateValidityChecker(pg->getPlanningContext().ssetup_.getSpaceInformation()), pg_(pg)
        {
            cr_with_distance_.distance = true;
        }

        void updatePlanningContext(void)
        {
            tss_.reset(new TSStorage(pg->getPlanningContext().start_state_));
        }

        virtual bool isValid(const ompl::base::State *state) const
        {
            planning_models::KinematicState *kstate = tss_->getStateStorage();
            pg_->getKMStateSpace().copyToKinematicState(*kstate, state);
            if (!pg_->getPlanningContext()->kcs_->decide(*kstate).first)
                return false;

            collision_detection::CollisionResult res;
            pg_->getPlanningScene()->checkCollision(cr_simple_, res, *kstate);
            return res.collision;
        }

        virtual bool isValid(const ompl::base::State *state, double &dist) const
        {
            planning_models::KinematicState *kstate = tss_->getStateStorage();
            pg_->getKMStateSpace().copyToKinematicState(*kstate, state);
            std::pair<bool, double> &r = pg_->getPlanningContext()->kcs_->decide(*kstate);
            if (!r.first)
            {
                dist = r.second;
                return false;
            }
            collision_detection::CollisionResult res;
            pg_->getPlanningScene()->checkCollision(cr_with_distance_, res, *kstate);
            dist = res.distance;
            return res.collision;
        }

    protected:

        const PlanningGroup                  *pg_;
        boost::scoped_ptr<TSStateStorage>     tss_;
        collision_detection::CollisionRequest cr_simple_;
        collision_detection::CollisionRequest cr_with_distance_;
    };

}

#endif
