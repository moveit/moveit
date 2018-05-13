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

/* Author: Zachary Kingston */

#ifndef MOVEIT_OMPL_INTERFACE_OMPL_PLANNING_CONTEXT_
#define MOVEIT_OMPL_INTERFACE_OMPL_PLANNING_CONTEXT_

#include "moveit_planners_ompl/OMPLDynamicReconfigureConfig.h"

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>

namespace ompl_interface
{
using namespace moveit_planners_ompl;

struct OMPLPlanningContextSpecification
{
  planning_interface::PlannerConfigurationSettings config_;
  constraint_samplers::ConstraintSamplerManagerPtr csm_;
  moveit_msgs::MotionPlanRequest req_;

  robot_model::RobotModelConstPtr robot_model_;
  const robot_model::JointModelGroup* jmg_;
  robot_model::JointBoundsVector joint_bounds_;
};

MOVEIT_CLASS_FORWARD(OMPLPlanningContext);

class OMPLPlanningContext : public planning_interface::PlanningContext
{
public:
  OMPLPlanningContext()
    : planning_interface::PlanningContext("UNINITIALIZED", "NO_GROUP")
    , max_goal_samples_(0)
    , max_state_sampling_attempts_(0)
    , max_goal_sampling_attempts_(0)
    , max_planning_threads_(0)
    , max_solution_segment_length_(0.0)
    , minimum_waypoint_count_(0)
    , simplify_solutions_(true)
  {
  }

  virtual void initialize(const OMPLPlanningContextSpecification& spec)
  {
    name_ = spec.config_.name;
    group_ = spec.config_.group;
    spec_ = spec;
  }

  virtual void configure(const ros::NodeHandle& nh, const OMPLDynamicReconfigureConfig& config)
  {
    simplifySolutions(config.simplify_solutions);
    setMaximumPlanningThreads(config.max_planning_threads);
    setMaximumGoalSamples(config.max_goal_samples);
    setMaximumStateSamplingAttempts(config.max_state_sampling_attempts);
    setMaximumGoalSamplingAttempts(config.max_goal_sampling_attempts);
    if (config.max_solution_segment_length > std::numeric_limits<double>::epsilon())
      setMaximumSolutionSegmentLength(config.max_solution_segment_length);
    setMinimumWaypointCount(config.minimum_waypoint_count);
  }

  void setCompleteInitialState(const robot_state::RobotState& complete_initial_robot_state)
  {
    *complete_initial_robot_state_ = complete_initial_robot_state;
    complete_initial_robot_state_->update();
  }

  const robot_state::RobotState& getCompleteInitialRobotState() const
  {
    return *complete_initial_robot_state_;
  }

  virtual void setPlanningVolume(const moveit_msgs::WorkspaceParameters& wparams) = 0;
  virtual bool setGoalConstraints(const std::vector<moveit_msgs::Constraints>& goal_constraints,
                                  const moveit_msgs::Constraints& path_constraints,
                                  moveit_msgs::MoveItErrorCodes* error) = 0;
  virtual bool setPathConstraints(const moveit_msgs::Constraints& path_constraints,
                                  moveit_msgs::MoveItErrorCodes* error) = 0;

  const OMPLPlanningContextSpecification& getSpecification() const
  {
    return spec_;
  }

  const std::map<std::string, std::string>& getSpecificationConfig() const
  {
    return spec_.config_.config;
  }

  void setSpecificationConfig(const std::map<std::string, std::string>& config)
  {
    spec_.config_.config = config;
  }

  const constraint_samplers::ConstraintSamplerManagerPtr& getConstraintSamplerManager()
  {
    return spec_.csm_;
  }

  void setConstraintSamplerManager(const constraint_samplers::ConstraintSamplerManagerPtr& csm)
  {
    spec_.csm_ = csm;
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

  unsigned int getMinimumWaypointCount() const
  {
    return minimum_waypoint_count_;
  }

  /** \brief Get the minimum number of waypoints along the solution path */
  void setMinimumWaypointCount(unsigned int mwc)
  {
    minimum_waypoint_count_ = mwc;
  }

  bool simplifySolutions() const
  {
    return simplify_solutions_;
  }

  void simplifySolutions(bool flag)
  {
    simplify_solutions_ = flag;
  }

protected:
  /// specification for the plannning context
  OMPLPlanningContextSpecification spec_;

  /// the complete initial state of the robot
  robot_state::RobotStatePtr complete_initial_robot_state_;

  /// maximum number of valid states to store in the goal region for any planning request (when such sampling is
  /// possible)
  unsigned int max_goal_samples_;

  /// maximum number of attempts to be made at sampling a state when attempting to find valid states that satisfy some
  /// set of constraints
  unsigned int max_state_sampling_attempts_;

  /// maximum number of attempts to be made at sampling a goal states
  unsigned int max_goal_sampling_attempts_;

  /// when planning in parallel, this is the maximum number of threads to use at one time
  unsigned int max_planning_threads_;

  /// the maximum length that is allowed for segments that make up the motion plan; by default this is 1% from the
  /// extent of the space
  double max_solution_segment_length_;

  /// the minimum number of points to include on the solution path (interpolation is used to reach this number, if
  /// needed)
  unsigned int minimum_waypoint_count_;

  /// should solutions be simplified?
  bool simplify_solutions_;
};
}

#endif
