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

    class OMPLInterface
    {
    public:

        OMPLInterface(void) : configured_(false)
        {
        }

        virtual ~OMPLInterface(void)
        {
        }

        bool configure(const planning_scene::PlanningSceneConstPtr &scene, const std::vector<PlannerConfigs> &pconfig);
	void configureIKSolvers(const std::map<std::string, kinematic_constraints::IKAllocator> &ik_allocators);
	
        void setMaximumSamplingAttempts(unsigned int max_sampling_attempts);
        void setMaximumGoalSamples(unsigned int max_goal_samples);
        void setMaximumPlanningThreads(unsigned int max_planning_threads);

        const PlanningGroupPtr& getPlanningConfiguration(const std::string &config) const;

        bool solve(const moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res) const;
        bool solve(const std::string &config, const planning_models::KinematicState &start_state, const moveit_msgs::Constraints &goal_constraints, double timeout);
        bool solve(const std::string &config, const planning_models::KinematicState &start_state, const moveit_msgs::Constraints &goal_constraints,
                   const moveit_msgs::Constraints &path_constraints, double timeout);

        bool isConfigured(void) const
        {
            return configured_;
        }

    protected:

        /** \brief The planning scene to consider as context when computing motion plans */
        planning_scene::PlanningSceneConstPtr   scene_;

        /** \brief All the existing planning configurations. The name
            of the configuration is the key of the map. This name can
            be of the form "group_name[config_name]" if there are
            particular configurations specified for a group, or of the
            form "group_name" if default settings are to be used. */
        std::map<std::string, PlanningGroupPtr> planning_groups_;

        /** \brief This is the full state space of the kinematic model
            specified by the planning scene. */
        ompl::base::StateSpacePtr               fullSpace_;

        /** \brief Flag indicating whether the OMPL interface has been configured (the configure() function has been called) */
        bool                                    configured_;
    };
}

#endif
