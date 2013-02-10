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

/* Author: Ioan Sucan, Sachin Chitta */

#ifndef MOVEIT_OMPL_INTERFACE_MODEL_BASED_PLANNING_CONTEXT_
#define MOVEIT_OMPL_INTERFACE_MODEL_BASED_PLANNING_CONTEXT_

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/ompl_interface/detail/constrained_valid_state_sampler.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/base/StateStorage.h>
#include <boost/thread/mutex.hpp>

namespace ompl_interface
{
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

class ModelBasedPlanningContext;
typedef boost::shared_ptr<ModelBasedPlanningContext> ModelBasedPlanningContextPtr;

class ConstraintsLibrary;
typedef boost::shared_ptr<const ConstraintsLibrary> ConstraintsLibraryConstPtr;

struct ModelBasedPlanningContextSpecification;
typedef boost::function<ob::PlannerPtr(const ompl::base::SpaceInformationPtr &si, const std::string &name,
                                       const ModelBasedPlanningContextSpecification &spec)> ConfiguredPlannerAllocator;
typedef boost::function<ConfiguredPlannerAllocator(const std::string &planner_type)> ConfiguredPlannerSelector;

struct ModelBasedPlanningContextSpecification
{
  std::map<std::string, std::string> config_;
  ConfiguredPlannerSelector planner_selector_; 
  ConstraintsLibraryConstPtr constraints_library_;
  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager_;
  bool use_state_validity_cache_;
  
  ModelBasedStateSpacePtr state_space_;
  std::vector<ModelBasedStateSpacePtr> subspaces_;
};
  
class ModelBasedPlanningContext
{  
public:
  
  ModelBasedPlanningContext(const std::string &name, const ModelBasedPlanningContextSpecification &spec);
  
  virtual ~ModelBasedPlanningContext()
  {
  }
  
  const std::string& getName() const
  {
    return name_;
  }

  const ModelBasedPlanningContextSpecification& getSpecification() const
  {
    return spec_;
  }

  const robot_model::RobotModelConstPtr& getRobotModel() const
  {
    return spec_.state_space_->getRobotModel();
  }
  
  const robot_model::JointModelGroup* getJointModelGroup() const
  {
    return spec_.state_space_->getJointModelGroup();
  }  
  
  const std::string& getJointModelGroupName() const
  {
    return spec_.state_space_->getJointModelGroupName();
  }
  
  const planning_scene::PlanningSceneConstPtr& getPlanningScene() const
  {
    return planning_scene_;
  }
  
  const robot_state::RobotState& getCompleteInitialRobotState() const
  {
    return complete_initial_robot_state_;
  }
  
  const ModelBasedStateSpacePtr& getOMPLStateSpace() const
  {
    return spec_.state_space_;
  }
  
  const og::SimpleSetup& getOMPLSimpleSetup() const
  {
    return ompl_simple_setup_;
  }

  og::SimpleSetup& getOMPLSimpleSetup()
  {
    return ompl_simple_setup_;
  }

  const ot::Benchmark& getOMPLBenchmark() const
  {
    return ompl_benchmark_;
  }

  ot::Benchmark& getOMPLBenchmark()
  {
    return ompl_benchmark_;
  }

  const kinematic_constraints::KinematicConstraintSetPtr& getPathConstraints() const
  {
    return path_constraints_;
  }
  
  /* \brief Get the maximum number of sampling attempts allowed when sampling states is needed */
  unsigned int getMaximumStateSamplingAttempts() const
  {
    return max_state_sampling_attempts_;
  }
  
  /* \brief Set the maximum number of sampling attempts allowed when sampling states is needed */
  void setMaximumStateSamplingAttempts(unsigned int max_state_sampling_attempts)
  {
    max_state_sampling_attempts_ = max_state_sampling_attempts;
  }

  /* \brief Get the maximum number of sampling attempts allowed when sampling goals is needed */
  unsigned int getMaximumGoalSamplingAttempts() const
  {
    return max_goal_sampling_attempts_;
  }
  
  /* \brief Set the maximum number of sampling attempts allowed when sampling goals is needed */
  void setMaximumGoalSamplingAttempts(unsigned int max_goal_sampling_attempts)
  {
    max_goal_sampling_attempts_ = max_goal_sampling_attempts;
  }
  
  /* \brief Get the maximum number of valid goal samples to store */
  unsigned int getMaximumGoalSamples() const
  {
    return max_goal_samples_;
  }
  
  /* \brief Set the maximum number of valid goal samples to store */
  void setMaximumGoalSamples(unsigned int max_goal_samples)
  {
    max_goal_samples_ = max_goal_samples;
  }

  /* \brief Get the maximum number of planning threads allowed */
  unsigned int getMaximumPlanningThreads() const
  {
    return max_planning_threads_;
  }
  
  /* \brief Set the maximum number of planning threads */
  void setMaximumPlanningThreads(unsigned int max_planning_threads)
  {
    max_planning_threads_ = max_planning_threads;
  }
  
  /* \brief Get the maximum solution segment length */
  double getMaximumSolutionSegmentLength() const
  {
    return max_solution_segment_length_;
  }
  
  /* \brief Set the maximum solution segment length */
  void setMaximumSolutionSegmentLength(double mssl)
  {
    max_solution_segment_length_ = mssl;
  }

  const constraint_samplers::ConstraintSamplerManagerPtr& getConstraintSamplerManager()
  {
    return spec_.constraint_sampler_manager_;
  }
  
  void setConstraintSamplerManager(const constraint_samplers::ConstraintSamplerManagerPtr &csm)
  {
    spec_.constraint_sampler_manager_ = csm;
  }
  
  void setVerboseStateValidityChecks(bool flag);
  
  void setProjectionEvaluator(const std::string &peval);
  
  void setPlanningScene(const planning_scene::PlanningSceneConstPtr &planning_scene);
  void setPlanningVolume(const moveit_msgs::WorkspaceParameters &wparams);

  void setCompleteInitialState(const robot_state::RobotState &complete_initial_robot_state);
  
  bool setGoalConstraints(const std::vector<moveit_msgs::Constraints> &goal_constraints,
			  const moveit_msgs::Constraints &path_constraints,
			  moveit_msgs::MoveItErrorCodes *error);
  bool setPathConstraints(const moveit_msgs::Constraints &path_constraints,
			  moveit_msgs::MoveItErrorCodes *error);

  void setConstraintsApproximations(const ConstraintsLibraryConstPtr &constraints_library)
  {
    spec_.constraints_library_ = constraints_library;
  }
  
  void setFollowSamplers(const std::vector<ValidConstrainedSamplerPtr> &samplers)
  {
    follow_samplers_ = samplers;
  }
  
  void clear();
  
  bool useStateValidityCache() const
  {
    return spec_.use_state_validity_cache_;
  }

  /* @brief Solve the planning problem. Return true if the problem is solved
     @param timeout The time to spend on solving
     @param count The number of runs to combine the paths of, in an attempt to generate better quality paths
  */
  bool solve(double timeout, unsigned int count);
  
  /* @brief Solve the planning problem in the case that requires following a specified set of trajectory constaints. Return true if the problem is solved.
     @param timeout The time to spend on solving
     @param count The number of runs to combine the paths of, in an attempt to generate better quality paths
  */
  bool follow(double timeout, unsigned int count);
  
  /* @brief Benchmark the planning problem. Return true on succesful saving of benchmark results
     @param timeout The time to spend on solving
     @param count The number of runs to average in the computation of the benchmark
     @param filename The name of the file to which the benchmark results are to be saved (automatic names can be provided if a name is not specified)
  */
  bool benchmark(double timeout, unsigned int count, const std::string &filename = "");
  
  void terminateSolve();
  
  /* @brief Get the amount of time spent computing the last plan */
  double getLastPlanTime() const
  {
    return last_plan_time_;
  }

  /* @brief Get the amount of time spent simplifying the last plan */
  double getLastSimplifyTime() const
  {
    return last_simplify_time_;
  }
  
  /* @brief Apply smoothing and try to simplify the plan
     @param timeout The amount of time allowed to be spent on simplifying the plan*/
  void simplifySolution(double timeout);
  
  /* @brief Interpolate the solution*/
  void interpolateSolution();
  
  /* @brief Get the solution as a RobotTrajectory object*/
  bool getSolutionPath(robot_trajectory::RobotTrajectory &traj) const;
  
  void convertPath(const og::PathGeometric &pg, robot_trajectory::RobotTrajectory &traj) const;  

  virtual void configure();

protected:
  
  void preSolve();
  void postSolve();
  
  virtual ob::ProjectionEvaluatorPtr getProjectionEvaluator(const std::string &peval) const;
  virtual ob::StateSamplerPtr allocPathConstrainedSampler(const ompl::base::StateSpace *ss) const;
  virtual void useConfig();
  virtual ob::GoalPtr constructGoal();

  void registerTerminationCondition(const ob::PlannerTerminationCondition &ptc);
  void unregisterTerminationCondition();
    
  ModelBasedPlanningContextSpecification spec_;
  
  std::string name_;
  
  robot_state::RobotState complete_initial_robot_state_;
  planning_scene::PlanningSceneConstPtr planning_scene_;

  /// the OMPL planning context; this contains the problem definition and the planner used
  og::SimpleSetup ompl_simple_setup_;
  
  /// the OMPL tool for benchmarking planners
  ot::Benchmark ompl_benchmark_;
  
  /// tool used to compute multiple plans in parallel; this uses the problem definition maintained by ompl_simple_setup_
  ot::ParallelPlan ompl_parallel_plan_;

  std::vector<int> space_signature_;

  kinematic_constraints::KinematicConstraintSetPtr              path_constraints_;
  moveit_msgs::Constraints                                      path_constraints_msg_;
  std::vector<kinematic_constraints::KinematicConstraintSetPtr> goal_constraints_;
  std::vector<ValidConstrainedSamplerPtr>                       follow_samplers_;
  
  const ob::PlannerTerminationCondition *ptc_;
  boost::mutex ptc_lock_;
    
  /// the time spent computing the last plan
  double                                                  last_plan_time_;  

  /// the time spent simplifying the last plan
  double                                                  last_simplify_time_;  

  /// maximum number of valid states to store in the goal region for any planning request (when such sampling is possible)
  unsigned int                                            max_goal_samples_;

  /// maximum number of attempts to be made at sampling a state when attempting to find valid states that satisfy some set of constraints
  unsigned int                                            max_state_sampling_attempts_;

  /// maximum number of attempts to be made at sampling a goal states
  unsigned int                                            max_goal_sampling_attempts_;
  
  /// when planning in parallel, this is the maximum number of threads to use at one time
  unsigned int                                            max_planning_threads_;

  /// the maximum length that is allowed for segments that make up the motion plan; by default this is 1% from the extent of the space
  double                                                  max_solution_segment_length_;
};

}

#endif

