/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef MOVEIT_OMPL_INTERFACE_PLANNING_CONTEXT_MANAGER_
#define MOVEIT_OMPL_INTERFACE_PLANNING_CONTEXT_MANAGER_

#include "ompl_interface/model_based_planning_context.h"
#include "ompl_interface/parameterization/model_based_state_space_factory.h"
#include <moveit_msgs/MotionPlanRequest.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>
#include <map>

namespace ompl_interface
{

struct PlanningConfigurationSettings
{
  std::string                        name;
  std::string                        group;
  std::map<std::string, std::string> config;
};

class PlanningContextManager
{
public: 
  
  PlanningContextManager(const planning_models::KinematicModelConstPtr &kmodel);
  ~PlanningContextManager(void);
  
  /** @brief Specify configurations for the planners.
      @param pconfig Configurations for the different planners */
  void setPlanningConfigurations(const std::vector<PlanningConfigurationSettings> &pconfig);

  /** @brief Specify the available inverse kinematics solvers
      @param kinematics_allocators Allocate the inverse kinematics solvers*/
  void specifyIKSolvers(const std::map<std::string, kc::KinematicsAllocator> &kinematics_allocators);

  /* \brief Get the maximum number of sampling attempts allowed */
  unsigned int getMaximumSamplingAttempts(void) const
  {
    return max_sampling_attempts_;
  }
  
  /* \brief Set the maximum number of sampling attempts allowed */
  void setMaximumSamplingAttempts(unsigned int max_sampling_attempts)
  {
    max_sampling_attempts_ = max_sampling_attempts;
  }
  
  /* \brief Get the maximum number of goal samples */
  unsigned int getMaximumGoalSamples(void) const
  {
    return max_goal_samples_;
  }
  
  /* \brief Set the maximum number of goal samples */
  void setMaximumGoalSamples(unsigned int max_goal_samples)
  {
    max_goal_samples_ = max_goal_samples;
  }
  
  /* \brief Get the maximum number of planning threads allowed */
  unsigned int getMaximumPlanningThreads(void) const
  {
    return max_planning_threads_;
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
  
  double getMaximumVelocity(void) const
  {
    return max_velocity_;    
  }
  
  void setMaximumVelocity(double mv)
  {
    max_velocity_ = mv;
  }
  
  double getMaximumAcceleration(void) const
  {
    return max_acceleration_;
  }
  
  void setMaximumAcceleration(double ma)
  {
    max_acceleration_ = ma;
  }
  
  const planning_models::KinematicModelConstPtr& getKinematicModel(void) const
  {
    return kmodel_;
  }
  
  ModelBasedPlanningContextPtr getLastPlanningContext(void) const;
    
  ModelBasedPlanningContextPtr getPlanningContext(const std::string &config, const std::string &factory_type = "") const;

  ModelBasedPlanningContextPtr getPlanningContext(const moveit_msgs::MotionPlanRequest &req) const;
  
  void registerPlannerAllocator(const std::string &planner_id, const ob::PlannerAllocator &pa)
  {
    known_planners_[planner_id] = pa;
  }
  
  void registerStateSpaceFactory(const ModelBasedStateSpaceFactoryPtr &factory)
  {
    state_space_factories_[factory->getType()] = factory;
  }

  ConfiguredPlannerAllocator getPlannerAllocator(void) const;

  const std::map<std::string, PlanningConfigurationSettings>& getPlanningConfigurations(void) const
  {
    return planner_configs_;
  }
  
protected:
  
  ob::PlannerPtr plannerAllocator(const ompl::base::SpaceInformationPtr &si, const std::string &planner,
                                  const std::string &name, const std::map<std::string, std::string> &config) const;
  
  void registerDefaultPlanners(void);
  void registerDefaultStateSpaces(void);
  
  ModelBasedPlanningContextPtr getPlanningContext(const PlanningConfigurationSettings &config, const ModelBasedStateSpaceFactory *factory) const;
  
  /** \brief The kinematic model for which motion plans are computed */
  planning_models::KinematicModelConstPtr               kmodel_;
  
  /** \brief A map from group names to IK allocators; these are the available IK solvers */
  AvailableKinematicsSolvers                            kinematics_allocators_;
  std::map<std::string, ob::PlannerAllocator>           known_planners_;
  std::map<std::string, ModelBasedStateSpaceFactoryPtr> state_space_factories_; 

  /** \brief All the existing planning configurations. The name
      of the configuration is the key of the map. This name can
      be of the form "group_name[config_name]" if there are
      particular configurations specified for a group, or of the
      form "group_name" if default settings are to be used. */
  std::map<std::string, PlanningConfigurationSettings>  planner_configs_;

  /// maximum number of states to sample in the goal region for any planning request (when such sampling is possible)
  unsigned int                                          max_goal_samples_;
  
  /// maximum number of attempts to be made at sampling a state when attempting to find valid states that satisfy some set of constraints
  unsigned int                                          max_sampling_attempts_;
  
  /// when planning in parallel, this is the maximum number of threads to use at one time
  unsigned int                                          max_planning_threads_;
  
  /// the maximum velocity to move with
  double                                                max_velocity_;
  
  /// the maximum acceleration to move at
  double                                                max_acceleration_;
  
  /// the maximum length that is allowed for segments that make up the motion plan; by default this is 1% from the extent of the space
  double                                                max_solution_segment_length_;

private:
  
  class LastPlanningContext;
  boost::shared_ptr<LastPlanningContext>                last_planning_context_;

  struct CachedContexts;
  boost::shared_ptr<CachedContexts>                     cached_contexts_;
};

}


#endif
