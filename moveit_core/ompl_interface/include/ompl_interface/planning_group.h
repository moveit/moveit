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

#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/GoalLazySamples.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <kinematic_constraints/constraint_samplers.h>
#include <planning_scene/planning_scene.h>

#include "ompl_interface/detail/km_state_space.h"

namespace ompl_interface
{

    class PlanningGroup
    {
    public:

        PlanningGroup(const std::string &name, const planning_models::KinematicModel::JointModelGroup *jmg,
                      const std::map<std::string, std::string> &config, const planning_scene::PlanningSceneConstPtr &scene);
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

        const planning_scene::PlanningSceneConstPtr& getPlanningScene(void) const
        {
            return planning_scene_;
        }

        const KMStateSpace& getKMStateSpace(void) const
        {
            return km_state_space_;
        }

        unsigned int getMaximumSamplingAttempts(void) const
        {
            return max_sampling_attempts_;
        }

        unsigned int getMaximumGoalSamples(void) const
        {
            return max_goal_samples_;
        }

        unsigned int getMaximumPlanningThreads(void) const
        {
            return max_planning_threads_;
        }

        void setMaximumSamplingAttempts(unsigned int max_sampling_attempts)
        {
            max_sampling_attempts_ = max_sampling_attempts;
        }

        void setMaximumGoalSamples(unsigned int max_goal_samples)
        {
            max_goal_samples_ = max_goal_samples;
        }

        void setMaximumPlanningThreads(unsigned int max_planning_threads)
        {
            max_planning_threads_ = max_planning_threads;
        }

        const planning_models::KinematicState& getStartState(void) const
        {
            return start_state_;
        }

        const moveit_msgs::Constraints& getPathConstraintsMsg(void) const
        {
            return path_constraints_;
        }

        const moveit_msgs::Constraints& getGoalConstraintsMsg(void) const
        {
            return goal_constraints_;
        }

        const kinematic_constraints::KinematicConstraintSetPtr& getPathConstraints(void) const
        {
            return path_kset_;
        }

        const kinematic_constraints::KinematicConstraintSetPtr& getGoalConstraints(void) const
        {
            return goal_kset_;
        }

        const ompl::geometric::SimpleSetup& getOMPLContext(void) const
        {
            return ssetup_;
        }

        bool setupPlanningContext(const planning_models::KinematicState &start_state,
                                  const moveit_msgs::Constraints &goal_constraints,
                                  const moveit_msgs::Constraints &path_constraints,
                                  moveit_msgs::MoveItErrorCodes *error = NULL);
        void setPlanningVolume(const moveit_msgs::WorkspaceParameters &wparams);

        void setIKAllocator(const kinematic_constraints::IKAllocator &ik_alloc)
        {
            ik_allocator_ = ik_alloc;
        }

        bool solve(double timeout, unsigned int count);

        double getLastPlanTime(void) const
        {
            return last_plan_time_;
        }

        void simplifySolution(double timeout);

        bool getSolutionPath(moveit_msgs::RobotTrajectory &traj) const;
        void fillResponse(moveit_msgs::GetMotionPlan::Response &res) const;

    protected:

        void useConfig(const std::map<std::string, std::string> &config);
        void setProjectionEvaluator(const std::string &peval);
        kinematic_constraints::ConstraintSamplerPtr getConstraintsSampler(const moveit_msgs::Constraints &constr) const;
        ompl::base::StateSamplerPtr allocPathConstrainedSampler(const ompl::base::StateSpace *ss) const;
        ompl::base::PlannerPtr plannerAllocator(const ompl::base::SpaceInformationPtr &si, const std::string &planner,
                                                const std::map<std::string, std::string> &config) const;

        /// name of this planning group (or configuration)
        std::string                                             name_;

        /// the group planning is performed for. there may be multiple (perhaps differently configured and differently named) configurations for the same group
        const planning_models::KinematicModel::JointModelGroup *jmg_;

        /// pointer to the planning scene used for collision avoidance
        planning_scene::PlanningSceneConstPtr                   planning_scene_;

        /// wrapper around an OMPL space, which includes conversions to and from planning_models::KinematicState
        KMStateSpace                                            km_state_space_;

        /// the OMPL planning context; this contains the problem definition and the planner used
        ompl::geometric::SimpleSetup                            ssetup_;

        /// utility to compute multiple plans in parallel; this uses the problem definition maintained by ssetup_
        ompl::ParallelPlan                                      pplan_;

        /// the starting state considered for planning
        planning_models::KinematicState                         start_state_;

        /// the path constraints currently being considered
        moveit_msgs::Constraints                                path_constraints_;

        /// the goal constraints currently being considered (these include the path constraints as well)
        moveit_msgs::Constraints                                goal_constraints_;

        /// the set of kinematic constraints to be respected by any state on the path
        kinematic_constraints::KinematicConstraintSetPtr        path_kset_;

        /// the set of kinematic constraints to be respected by the goal state
        kinematic_constraints::KinematicConstraintSetPtr        goal_kset_;

        /// the time spend computing the last plan
        double                                                  last_plan_time_;

        /// maximum number of states to sample in the goal region for any planning request (when such sampling is possible)
        unsigned int                                            max_goal_samples_;

        /// maximum number of attempts to be made at sampling a state when attempting to find valid states that satisfy some set of constraints
        unsigned int                                            max_sampling_attempts_;

        /// when planning in parallel, this is the maximum number of threads to use at one time
        unsigned int                                            max_planning_threads_;

        /// a function pointer that returns an IK solver; this is useful for sampling states using IK
        kinematic_constraints::IKAllocator                      ik_allocator_;

    };

    typedef boost::shared_ptr<PlanningGroup> PlanningGroupPtr;
    typedef boost::shared_ptr<const PlanningGroup> PlanningGroupConstPtr;

}

#endif
