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
*   * Neither the name of Willow Garage nor the names of its
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

#ifndef MOVEIT_OMPL_INTERFACE_MODEL_BASED_PLANNING_CONTEXT_
#define MOVEIT_OMPL_INTERFACE_MODEL_BASED_PLANNING_CONTEXT_

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/ompl_interface/detail/constrained_valid_state_sampler.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/ompl_interface/ompl_planning_context.h>

#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space_factory.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space_factory.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/base/StateStorage.h>

#include <boost/thread/mutex.hpp>

namespace ompl_interface
{
using namespace moveit_planners_ompl;

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

MOVEIT_CLASS_FORWARD(ModelBasedPlanningContext);
MOVEIT_CLASS_FORWARD(ConstraintsLibrary);

struct ModelBasedPlanningContextSpecification;
typedef boost::function<ob::PlannerPtr(const ob::SpaceInformationPtr& si, const std::string& name,
                                       const OMPLPlanningContextSpecification& spec)>
    ConfiguredPlannerAllocator;
typedef boost::function<ConfiguredPlannerAllocator(const std::string& planner_type)> ConfiguredPlannerSelector;

class ModelBasedPlanningContext : public OMPLPlanningContext
{
public:
  ModelBasedPlanningContext();

  virtual void initialize(const OMPLPlanningContextSpecification& spec);
  virtual void configure(const ros::NodeHandle& nh, const OMPLDynamicReconfigureConfig& config);

  virtual bool solve(planning_interface::MotionPlanResponse& res);
  virtual bool solve(planning_interface::MotionPlanDetailedResponse& res);

  virtual void clear();
  virtual bool terminate();

  const robot_model::RobotModelConstPtr& getRobotModel() const
  {
    return ompl_state_space_->getRobotModel();
  }

  const robot_model::JointModelGroup* getJointModelGroup() const
  {
    return ompl_state_space_->getJointModelGroup();
  }

  const ModelBasedStateSpacePtr& getOMPLStateSpace() const
  {
    return ompl_state_space_;
  }

  const og::SimpleSetupPtr& getOMPLSimpleSetup() const
  {
    return ompl_simple_setup_;
  }

  og::SimpleSetupPtr& getOMPLSimpleSetup()
  {
    return ompl_simple_setup_;
  }

  const ot::Benchmark& getOMPLBenchmark() const
  {
    return *ompl_benchmark_;
  }

  ot::Benchmark& getOMPLBenchmark()
  {
    return *ompl_benchmark_;
  }

  const kinematic_constraints::KinematicConstraintSetPtr& getPathConstraints() const
  {
    return path_constraints_;
  }

  void setVerboseStateValidityChecks(bool flag);

  void setProjectionEvaluator(const std::string& peval);

  void setPlanningVolume(const moveit_msgs::WorkspaceParameters& wparams);

  bool setGoalConstraints(const std::vector<moveit_msgs::Constraints>& goal_constraints,
                          const moveit_msgs::Constraints& path_constraints, moveit_msgs::MoveItErrorCodes* error);
  bool setPathConstraints(const moveit_msgs::Constraints& path_constraints, moveit_msgs::MoveItErrorCodes* error);

  bool useStateValidityCache() const
  {
    return use_state_validity_cache_;
  }

  void useStateValidityCache(bool flag)
  {
    use_state_validity_cache_ = flag;
  }

  /* @brief Solve the planning problem. Return true if the problem is solved
     @param timeout The time to spend on solving
     @param count The number of runs to combine the paths of, in an attempt to generate better quality paths
  */
  bool solve(double timeout, unsigned int count);

  /* @brief Benchmark the planning problem. Return true on succesful saving of benchmark results
     @param timeout The time to spend on solving
     @param count The number of runs to average in the computation of the benchmark
     @param filename The name of the file to which the benchmark results are to be saved (automatic names can be
     provided if a name is not specified)
  */
  bool benchmark(double timeout, unsigned int count, const std::string& filename = "");

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
  bool getSolutionPath(robot_trajectory::RobotTrajectory& traj) const;

  void convertPath(const og::PathGeometric& pg, robot_trajectory::RobotTrajectory& traj) const;

  void registerPlannerAllocator(const std::string& planner_id, const ConfiguredPlannerAllocator& pa)
  {
    known_planners_[planner_id] = pa;
  }

  const std::map<std::string, ConfiguredPlannerAllocator>& getRegisteredPlannerAllocators() const
  {
    return known_planners_;
  }

  bool saveConstraintApproximations(const ros::NodeHandle& nh);
  bool loadConstraintApproximations(const ros::NodeHandle& nh);

  void setConstraintsApproximations(const ConstraintsLibraryPtr& constraints_library)
  {
    constraints_library_ = constraints_library;
  }

  ConstraintsLibraryPtr getConstraintsLibrary()
  {
    return constraints_library_;
  }

  const ConstraintsLibraryPtr& getConstraintsLibrary() const
  {
    return constraints_library_;
  }

protected:
  void preSolve();
  void postSolve();

  void startSampling();
  void stopSampling();

  virtual ob::ProjectionEvaluatorPtr getProjectionEvaluator(const std::string& peval) const;
  virtual ob::StateSamplerPtr allocPathConstrainedSampler(const ob::StateSpace* ss) const;
  virtual void useConfig();
  virtual ob::GoalPtr constructGoal();

  void registerTerminationCondition(const ob::PlannerTerminationCondition& ptc);
  void unregisterTerminationCondition();

  ConfiguredPlannerAllocator plannerSelector(const std::string& planner) const;

  void registerDefaultPlanners();

  /// State space factories
  typedef boost::function<const ModelBasedStateSpaceFactoryPtr&(const std::string&)> StateSpaceFactoryTypeSelector;

  void registerDefaultStateSpaces()
  {
    registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new JointModelStateSpaceFactory()));
    registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new PoseModelStateSpaceFactory()));
  }

  void registerStateSpaceFactory(const ModelBasedStateSpaceFactoryPtr& factory)
  {
    state_space_factories_[factory->getType()] = factory;
  }

  const std::map<std::string, ModelBasedStateSpaceFactoryPtr>& getRegisteredStateSpaceFactories() const
  {
    return state_space_factories_;
  }

  const ModelBasedStateSpaceFactoryPtr& getStateSpaceFactory1(const std::string& group_name,
                                                              const std::string& factory_type) const;
  const ModelBasedStateSpaceFactoryPtr& getStateSpaceFactory2(const std::string& group_name,
                                                              const moveit_msgs::MotionPlanRequest& req) const;

  virtual ModelBasedStateSpacePtr getStateSpace();

  std::map<std::string, ConfiguredPlannerAllocator> known_planners_;
  std::map<std::string, ModelBasedStateSpaceFactoryPtr> state_space_factories_;

  ModelBasedStateSpacePtr ompl_state_space_;

  /// the OMPL planning context; this contains the problem definition and the planner used
  og::SimpleSetupPtr ompl_simple_setup_;

  /// the OMPL tool for benchmarking planners
  std::shared_ptr<ot::Benchmark> ompl_benchmark_;

  /// tool used to compute multiple plans in parallel; this uses the problem definition maintained by ompl_simple_setup_
  std::shared_ptr<ot::ParallelPlan> ompl_parallel_plan_;

  std::vector<int> space_signature_;

  kinematic_constraints::KinematicConstraintSetPtr path_constraints_;
  moveit_msgs::Constraints path_constraints_msg_;
  std::vector<kinematic_constraints::KinematicConstraintSetPtr> goal_constraints_;

  const ob::PlannerTerminationCondition* ptc_;
  boost::mutex ptc_lock_;

  /// the time spent computing the last plan
  double last_plan_time_;

  /// the time spent simplifying the last plan
  double last_simplify_time_;

  /// use the state validity cache for collision checking.
  bool use_state_validity_cache_;

  ConstraintsLibraryPtr constraints_library_;
};
}

#endif
