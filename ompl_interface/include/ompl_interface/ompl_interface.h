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

#ifndef OMPL_INTERFACE_OMPL_INTERFACE_
#define OMPL_INTERFACE_OMPL_INTERFACE_

#include "ompl_interface/planning_group.h"
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/ComputePlanningBenchmark.h>
#include <string>
#include <map>

namespace ompl_interface
{

    struct PlannerConfigs
    {
        std::string                        name;
        std::string                        group;
        std::map<std::string, std::string> config;
    };

    /** @class OMPLInterface
     *  This class defines the interface to the motion planners in OMPL*/
    class OMPLInterface
    {
    public: 
	
        OMPLInterface(void);
        virtual ~OMPLInterface(void);

        /** @brief Configure with a planning scene and configuration for the planners.
            @param scene The planning scene
            @param pconfig The configuration for the planning scene */
        bool configure(const planning_scene::PlanningSceneConstPtr &scene, const std::vector<PlannerConfigs> &pconfig);

        /** @brief Configure the inverse kinematics solvers
            @param ik_allocators Allocate the inverse kinematics solvers*/
        void configureIKSolvers(const std::map<std::string, kinematic_constraints::IKAllocator> &ik_allocators);

        /** @brief Set the maximum number of sampling attempts*/
        void setMaximumSamplingAttempts(unsigned int max_sampling_attempts);

        /** @brief Set the maximum number of goal samples*/
        void setMaximumGoalSamples(unsigned int max_goal_samples);

        /** @brief Set the maximum number of planning threads*/
        void setMaximumPlanningThreads(unsigned int max_planning_threads);

        /** @brief Get the planning group*/
        const PlanningGroupPtr& getPlanningConfiguration(const std::string &config) const;

        /** @brief Solve the planning problem*/
        bool solve(const moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res) const;

        /** @brief Benchmark the planning problem*/
        bool benchmark(const moveit_msgs::ComputePlanningBenchmark::Request &req, moveit_msgs::ComputePlanningBenchmark::Response &res) const;

        /** @brief Solve the planning problem
         *  @param config
         *  @param start_state The start state specified for the planning problem
         *  @param goal_constraints The goal constraints
         *  @param timeout The amount of time to spend on planning
         */
        bool solve(const std::string &config, const planning_models::KinematicState &start_state, const moveit_msgs::Constraints &goal_constraints, double timeout);

        /** @brief Solve the planning problem
         *  @param config
         *  @param start_state The start state specified for the planning problem
         *  @param goal_constraints The goal constraints
         *  @param path_constraints The path constraints
         *  @param timeout The amount of time to spend on planning
         */
        bool solve(const std::string &config,
                   const planning_models::KinematicState &start_state,
                   const moveit_msgs::Constraints &goal_constraints,
                   const moveit_msgs::Constraints &path_constraints,
                   double timeout);

        /** @brief Return if this class has been configured*/
        bool isConfigured(void) const
        {
            return configured_;
        }

        void addConstraintApproximation(const moveit_msgs::Constraints &msg, const std::string &group, unsigned int samples);
        void loadConstraintApproximations(const std::string &path);
        void saveConstraintApproximations(const std::string &path);
	void printConstraintApproximations(std::ostream &out = std::cout) const;
	void clearConstraintApproximations();
	
	const ConstraintApproximationsPtr& getConstraintApproximations(void) const
	{
	    return constraints_;
	}
	
    protected:


        /** \brief Configure the OMPL planning context for a new planning request */
        bool prepareForSolve(const moveit_msgs::MotionPlanRequest &req, moveit_msgs::MoveItErrorCodes &error_code,
                             PlanningGroup* &pg_to_use, unsigned int &attempts, double &timeout) const;

        /** \brief The planning scene to consider as context when computing motion plans */
        planning_scene::PlanningSceneConstPtr   scene_;

        /** \brief All the existing planning configurations. The name
            of the configuration is the key of the map. This name can
            be of the form "group_name[config_name]" if there are
            particular configurations specified for a group, or of the
            form "group_name" if default settings are to be used. */
        std::map<std::string, PlanningGroupPtr> planning_groups_;

	ConstraintApproximationsPtr             constraints_;

        /** \brief Flag indicating whether the OMPL interface has been configured (the configure() function has been called) */
        bool                                    configured_;
    };
}

#endif
