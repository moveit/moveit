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

#ifndef MOVEIT_OMPL_INTERFACE_OMPL_INTERFACE_
#define MOVEIT_OMPL_INTERFACE_OMPL_INTERFACE_

#include "ompl_interface/parameterization/model_based_planning_context_factory.h"
#include <moveit_msgs/ComputePlanningBenchmark.h>
#include <moveit_msgs/GetMotionPlan.h>
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

/** @class OMPLInterface
 *  This class defines the interface to the motion planners in OMPL*/
class OMPLInterface
{
public: 
  
  OMPLInterface(const planning_models::KinematicModelConstPtr &kmodel);
  virtual ~OMPLInterface(void);
  
  /** @brief Specify configurations for the planners.
      @param pconfig Configurations for the different planners */
  void setPlanningConfigurations(const std::vector<PlanningConfigurationSettings> &pconfig);
    
  /** @brief Specify the available inverse kinematics solvers
      @param ik_allocators Allocate the inverse kinematics solvers*/
  void specifyIKSolvers(const std::map<std::string, kinematic_constraints::IKAllocator> &ik_allocators)
  {
    ik_allocators_ = ik_allocators;
  }  
  
  /** @brief Set the maximum number of sampling attempts*/
  void setMaximumSamplingAttempts(unsigned int max_sampling_attempts)
  {
    max_sampling_attempts_ = max_sampling_attempts;
  }  
  
  /** @brief Set the maximum number of goal samples*/
  void setMaximumGoalSamples(unsigned int max_goal_samples)
  {
    max_goal_samples_ = max_goal_samples;
  }  
  
  /** @brief Set the maximum number of planning threads*/
  void setMaximumPlanningThreads(unsigned int max_planning_threads)
  {
    max_planning_threads_ = max_planning_threads;
  }
  
  /** @brief Solve the planning problem*/
  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res) const;
  
  /** @brief Benchmark the planning problem*/
  bool benchmark(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::ComputePlanningBenchmark::Request &req,                  
                 moveit_msgs::ComputePlanningBenchmark::Response &res) const;
  
  /** @brief Solve the planning problem
   *  @param config
   *  @param start_state The start state specified for the planning problem
   *  @param goal_constraints The goal constraints
   *  @param timeout The amount of time to spend on planning
   */
  //  ompl::base::PathPtr solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
  //                            const std::string &config, const planning_models::KinematicState &start_state,
  //                            const moveit_msgs::Constraints &goal_constraints, double timeout) const;
  
  /** @brief Solve the planning problem
   *  @param config
   *  @param start_state The start state specified for the planning problem
   *  @param goal_constraints The goal constraints
   *  @param path_constraints The path constraints
   *  @param timeout The amount of time to spend on planning
   */
  //  ompl::base::PathPtr solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
  //                            const std::string &config, const planning_models::KinematicState &start_state,
  //                            const moveit_msgs::Constraints &goal_constraints,
  //                            const moveit_msgs::Constraints &path_constraints, double timeout) const;
  
  void registerPlannerAllocator(const std::string &planner_id, const ob::PlannerAllocator &pa)
  {
    known_planners_[planner_id] = pa;
  }
  
  void registerStateSpaceFactory(const ModelBasedPlanningContextFactoryPtr &factory)
  {
    planning_context_factories_.push_back(factory);
  }  


  /*

  const PlanningConfigurationPtr& getPlanningConfiguration(const std::string &config) const;
  

  const PlanningConfigurationPtr& getLastPlanningConfiguration(void) const
  {
    return last_planning_configuration_solve_;
  }

  void addConstraintApproximation(const moveit_msgs::Constraints &constr_sampling, const moveit_msgs::Constraints &constr_hard, const std::string &group, unsigned int samples);
  void addConstraintApproximation(const moveit_msgs::Constraints &constr, const std::string &group, unsigned int samples);
  void loadConstraintApproximations(const std::string &path);
  void saveConstraintApproximations(const std::string &path);
  void printConstraintApproximations(std::ostream &out = std::cout) const;
  void clearConstraintApproximations();
  
  const ConstraintApproximationsPtr& getConstraintApproximations(void) const
  {
    return constraints_;
  }
  */

  ConfiguredPlannerAllocator getPlannerAllocator(void) const;
  
protected:
  
  ob::PlannerPtr plannerAllocator(const ompl::base::SpaceInformationPtr &si, const std::string &planner,
                                  const std::map<std::string, std::string> &config) const;
  
  void registerDefaultPlanners(void);
  
  /** \brief Configure the OMPL planning context for a new planning request */
  ModelBasedPlanningContextPtr prepareForSolve(const moveit_msgs::MotionPlanRequest &req,
                                               const planning_scene::PlanningSceneConstPtr& planning_scene, 
                                               moveit_msgs::MoveItErrorCodes &error_code,
                                               unsigned int &attempts, double &timeout) const;
  
  
  /** \brief The kinematic model for which motion plans are computed */
  const planning_models::KinematicModelConstPtr   kmodel_;
  
  /** \brief All the existing planning configurations. The name
      of the configuration is the key of the map. This name can
      be of the form "group_name[config_name]" if there are
      particular configurations specified for a group, or of the
      form "group_name" if default settings are to be used. */
  //  std::map<std::string, PlanningConfigurationPtr> planning_configurations_;
  
  /** \brief A map from group names to IK allocators; these are the available IK solvers */
  std::map<std::string, kinematic_constraints::IKAllocator> ik_allocators_;
  
  std::map<std::string, ob::PlannerAllocator>               known_planners_;
  std::vector<ModelBasedPlanningContextFactoryPtr>      planning_context_factories_;
  std::map<std::string, PlanningConfigurationSettings>    planner_configs_;

    /// maximum number of attempts to be made at sampling a state when attempting to find valid states that satisfy some set of constraints
  unsigned int                                            max_sampling_attempts_;
  
  /// when planning in parallel, this is the maximum number of threads to use at one time
  unsigned int                                            max_planning_threads_;

  /// maximum number of states to sample in the goal region for any planning request (when such sampling is possible)
  unsigned int                                            max_goal_samples_;

  /// the maximum length that is allowed for segments that make up the motion plan; by default this is 1% from the extent of the space
  double                                                  max_solution_segment_length_;

  /** \brief The planning group for which solve() was called last */
  //  mutable PlanningConfigurationPtr                last_planning_configuration_solve_;
  
  //  ConstraintApproximationsPtr                     constraints_;
  
};

}




#endif
