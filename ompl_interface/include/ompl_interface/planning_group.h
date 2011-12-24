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

#include "ompl_interface/detail/kinematic_model_state_space.h"

namespace ompl_interface
{
    /** @class PlanningGroup
     *  This class defines a planning group, i.e. a set of joints configured with a set of planners and a planning scene*/
    class PlanningGroup
    {
        friend class OMPLInterface;
    public:

      /**
         @brief Constructor
         @param name The name of the planning group
         @param jmg The joint model group to plan with
         @param config A map of configuration parameters for the planners
         @param scene A pointer to a planning scene
       */
        PlanningGroup(const std::string &name, const planning_models::KinematicModel::JointModelGroup *jmg,
                      const std::map<std::string, std::string> &config, const planning_scene::PlanningSceneConstPtr &scene);
        virtual ~PlanningGroup(void);

        /* \brief Return the name of this planning group. This is not always the same as the name of the joint group the planner is operating on */
        const std::string& getName(void) const
        {
            return name_;
        }

        /* \brief Return the joint model group this planning group is working with. */
        const planning_models::KinematicModel::JointModelGroup* getJointModelGroup(void) const
        {
            return joint_model_group_;
        }

        /* \brief Return the planning scene this planning group is working with. */
        const planning_scene::PlanningSceneConstPtr& getPlanningScene(void) const
        {
            return planning_scene_;
        }

        /* \brief Return the kinematic model state space this planning group is working with. */
        const KMStateSpace& getKMStateSpace(void) const
        {
            return kinematic_model_state_space_;
        }

        /* \brief Get the maximum number of sampling attempts allowed */
        unsigned int getMaximumSamplingAttempts(void) const
        {
            return max_sampling_attempts_;
        }

        /* \brief Get the maximum number of goal samples */
        unsigned int getMaximumGoalSamples(void) const
        {
            return max_goal_samples_;
        }

        /* \brief Get the maximum number of planning threads allowed */
        unsigned int getMaximumPlanningThreads(void) const
        {
            return max_planning_threads_;
        }

        /* \brief Set the maximum number of sampling attempts allowed */
        void setMaximumSamplingAttempts(unsigned int max_sampling_attempts)
        {
            max_sampling_attempts_ = max_sampling_attempts;
        }

        /* \brief Set the maximum number of goal samples */
        void setMaximumGoalSamples(unsigned int max_goal_samples)
        {
            max_goal_samples_ = max_goal_samples;
        }

        /* \brief Set the maximum number of planning threads */
        void setMaximumPlanningThreads(unsigned int max_planning_threads)
        {
            max_planning_threads_ = max_planning_threads;
        }

        /* \brief Get the maximum solution segment length */
        double getMaximumSolutionSegmentLength(void) const
        {
            return max_solution_segment_length_;
        }

        /* \brief Set the maximum solution segment length */
        void setMaximumSolutionSegmentLength(double mssl)
        {
            max_solution_segment_length_ = mssl;
        }

        /* \brief Get the start state */
        const planning_models::KinematicState& getStartState(void) const
        {
            return start_state_;
        }

        /* \brief Get the path constraints */
        const kinematic_constraints::KinematicConstraintSetPtr& getPathConstraints(void) const
        {
            return path_kinematic_constraints_set_;
        }

        /*
        const moveit_msgs::Constraints& getPathConstraintsMsg(void) const
        {
            return path_constraints_;
        }

        const moveit_msgs::Constraints& getGoalConstraintsMsg(void) const
        {
            return goal_constraints_;
        }


        const kinematic_constraints::KinematicConstraintSetPtr& getGoalConstraints(void) const
        {
            return goal_kset_;
        }
        */

        /* \brief Get the OMPL SimpleSetup object being used */
        const ompl::geometric::SimpleSetup& getOMPLSimpleSetup(void) const
        {
            return ompl_simple_setup_;
        }

        /* @brief Set all the information needed for a planner 
           @param start_state The start state that the planner will use
           @param goal_constraints The goal constraints 
           @param path_constraints Path constraints
           @param status The return status code
           @return False if something goes wrong (status code provides more information on what went wrong)
        */     
        bool setupPlanningContext(const planning_models::KinematicState &start_state,
                                  const std::vector<moveit_msgs::Constraints> &goal_constraints,
                                  const moveit_msgs::Constraints &path_constraints,
                                  moveit_msgs::MoveItErrorCodes *status = NULL);

        /* @brief Set the volume of space that the planner works in*/     
        void setPlanningVolume(const moveit_msgs::WorkspaceParameters &wparams);

        /* @brief solve the planning problem
           @param timeout The time to spend on solving
           @param count 
        */     
        bool solve(double timeout, unsigned int count);

        /* @brief Get the amount of time spent on the last plan*/     
        double getLastPlanTime(void) const
        {
            return last_plan_time_;
        }

        /* @brief Apply smoothing and try to simplify the plan
           @param timeout The amount of time allowed to be spent on simplifying the plan*/     
        void simplifySolution(double timeout);

        /* @brief Interpolate the solution*/     
        void interpolateSolution();

        /* @brief Get the solution as a RobotTrajectory object*/     
        bool getSolutionPath(moveit_msgs::RobotTrajectory &traj) const;

        /* @brief Fill in the response to the motion plan request. This includes the status code of the motion plan*/     
        void fillResponse(moveit_msgs::GetMotionPlan::Response &res) const;

    protected:

        void useConfig(const std::map<std::string, std::string> &config);
        void setProjectionEvaluator(const std::string &peval);
        kinematic_constraints::ConstraintSamplerPtr getConstraintsSampler(const moveit_msgs::Constraints &constr) const;
        ompl::base::GoalPtr getGoalRepresentation(const kinematic_constraints::KinematicConstraintSetPtr &kset) const;

        ompl::base::StateSamplerPtr allocPathConstrainedSampler(const ompl::base::StateSpace *ss) const;
        ompl::base::PlannerPtr plannerAllocator(const ompl::base::SpaceInformationPtr &si, const std::string &planner,
                                                const std::map<std::string, std::string> &config) const;

        /// name of this planning group (or externally specified configuration name, if such a configuration is provided;
        /// there may be multiple (perhaps differently configured and differently named) configurations for the same group)
        std::string                                             name_;

        /// the group planning is performed for.
        const planning_models::KinematicModel::JointModelGroup *joint_model_group_;

        /// pointer to the planning scene used for collision avoidance
        planning_scene::PlanningSceneConstPtr                   planning_scene_;

        /// wrapper around an OMPL space, which includes conversions to and from planning_models::KinematicState
        KMStateSpace                                            kinematic_model_state_space_;

        /// the OMPL planning context; this contains the problem definition and the planner used
        ompl::geometric::SimpleSetup                            ompl_simple_setup_;

        /// tool used to compute multiple plans in parallel; this uses the problem definition maintained by ompl_simple_setup_
        ompl::ParallelPlan                                      pplan_;

        /// the starting state considered for planning
        planning_models::KinematicState                         start_state_;

        /// the set of kinematic constraints to be respected by any state on the path
        kinematic_constraints::KinematicConstraintSetPtr        path_kinematic_constraints_set_;

        /// the set of kinematic constraints to be respected by the goal state
        std::vector<kinematic_constraints::KinematicConstraintSetPtr> goal_constraints_;

        /// the time spent computing the last plan
        double                                                  last_plan_time_;

        /// the maximum length that is allowed for segments that make up the motion plan; by default this is 1% from the extent of the space
        double                                                  max_solution_segment_length_;

        /// maximum number of states to sample in the goal region for any planning request (when such sampling is possible)
        unsigned int                                            max_goal_samples_;

        /// maximum number of attempts to be made at sampling a state when attempting to find valid states that satisfy some set of constraints
        unsigned int                                            max_sampling_attempts_;

        /// when planning in parallel, this is the maximum number of threads to use at one time
        unsigned int                                            max_planning_threads_;

        /// a function pointer that returns an IK solver for this group; this is useful for sampling states using IK
        kinematic_constraints::IKAllocator                      ik_allocator_;

        kinematic_constraints::IKSubgroupAllocator              ik_subgroup_allocators_;

    };

    typedef boost::shared_ptr<PlanningGroup> PlanningGroupPtr;
    typedef boost::shared_ptr<const PlanningGroup> PlanningGroupConstPtr;

}

#endif
