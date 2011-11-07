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
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/GoalLazySamples.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <kinematic_constraints/constraint_samplers.h>
#include <planning_scene/planning_scene.h>

#include "ompl_interface/detail/km_state_space.h"

namespace ompl_interface
{
    struct PlanningContext
    {
        PlanningContext(const KMStateSpace &ks) : ssetup_(ks.getOMPLSpace()), pplan_(ssetup_.getProblemDefinition())
        {
        }

        ompl::geometric::SimpleSetup                     ssetup_;
        ompl::ParallelPlan                               pplan_;
        planning_models::KinematicStatePtr               start_state_;
        moveit_msgs::Constraints                         path_constraints_;
        moveit_msgs::Constraints                         goal_constraints_;
        kinematic_constraints::KinematicConstraintSetPtr kset_;
    };

    class PlanningGroup
    {
    public:

        PlanningGroup(const std::string &name, const planning_models::KinematicModel::JointModelGroup *jmg,
                      const std::map<std::string, std::string> &config, const planning_scene::PlanningScenePtr &scene, ompl::StateSpaceCollection &ssc);
        virtual ~PlanningGroup(void);

        /* \brief Return the name of this planning group. This is not always the same as the name of the joint group the planner is operating on */
        const std::string& getName(void) const
        {
            return name_;
        }

        const planning_models::KinematicModel::JointModelGroup* getJointModelGroup(void) const
        {
            return jmg_;
        }

        const planning_scene::PlanningScenePtr& getPlanningScene(void) const
        {
            return planning_scene_;
        }

        const KMStateSpace& getKMStateSpace(void) const
        {
            return km_state_space_;
        }

        const PlanningContext& getPlanningContext(void) const
        {
            return planning_context_;
        }

        unsigned int getMaximumSamplingAttempts(void) const
        {
            return max_sampling_attempts_;
        }

        unsigned int getMaximumGoalSamples(void) const
        {
            return max_goal_samples_;
        }

        bool setupPlanningContext(const planning_models::KinematicState &start_state,
                                  const moveit_msgs::Constraints &goal_constraints,
                                  const moveit_msgs::Constraints &path_constraints);

        bool solve(double timeout, unsigned int count);

        bool getSolutionPath(moveit_msgs::RobotTrajectory &traj) const;
        void fillResponse(moveit_msgs::GetMotionPlan::Response &res) const;

    protected:

        kinematic_constraints::ConstraintSamplerPtr getConstraintsSampler(const moveit_msgs::Constraints &constr) const;
        ompl::base::StateSamplerPtr allocPathConstrainedSampler(const ompl::base::StateSpace *ss) const;
        ompl::base::PlannerPtr plannerAllocator(const ompl::base::SpaceInformationPtr &si, const std::string &planner,
                                                const std::map<std::string, std::string> &config) const;

        std::string                                             name_;
        const planning_models::KinematicModel::JointModelGroup *jmg_;
        planning_scene::PlanningScenePtr                        planning_scene_;
        KMStateSpace                                            km_state_space_;
        PlanningContext                                         planning_context_;

        unsigned int                                            max_goal_samples_;
        unsigned int                                            max_sampling_attempts_;

        kinematic_constraints::IKAllocator                      ik_allocator_;
        ompl::RNG                                               rng_;
    };

    typedef boost::shared_ptr<PlanningGroup> PlanningGroupPtr;
}

#endif
