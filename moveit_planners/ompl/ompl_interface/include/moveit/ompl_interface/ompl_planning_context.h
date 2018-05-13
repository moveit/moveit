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

/**
 * \brief OMPLPlanningContextSpecification contains all parameters necessary for a derived class of OMPLPLanningContext
 * to setup.
 */
struct OMPLPlanningContextSpecification
{
  planning_interface::PlannerConfigurationSettings config_; /**< \brief Planner configuration for context */
  constraint_samplers::ConstraintSamplerManagerPtr csm_;    /**< \brief Constraint sampler manager */
  moveit_msgs::MotionPlanRequest req_;                      /**< \brief Motion planning request */
  robot_model::RobotModelConstPtr robot_model_;             /**< \brief Robot model */
  const robot_model::JointModelGroup* jmg_;                 /**< \brief Joint model group to plan for */
  robot_model::JointBoundsVector joint_bounds_;             /**< \brief Additional bounds on joints */
};

MOVEIT_CLASS_FORWARD(OMPLPlanningContext);

/**
 * \brief OMPLPlanningContext is an abstract base class that all OMPL planning context plugins must implement to be
 * loaded.
 */
class OMPLPlanningContext : public planning_interface::PlanningContext
{
public:
  /**
   * \brief Constructor.
   */
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

  /** \brief Initializes the planning context. Derived classes should call this method.
   *
   * @param [in] spec The planning context specification to initialize with.
   */
  virtual void initialize(const OMPLPlanningContextSpecification& spec)
  {
    name_ = spec.config_.name;
    group_ = spec.config_.group;
    spec_ = spec;
  }

  /** \brief Configures the planning context (usually from a dynamic reconfigure callback). Derived classes should call
   * this method.
   *
   * @param [in] nh ROS node handle to use for parameter initialization.
   * @param [in] config The dynamic reconfigure config.
   */
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

  /** \brief Sets the initial state of the robot.
   *
   * @param [in] complete_initial_robot_state The initial state of the robot.
   */
  void setCompleteInitialState(const robot_state::RobotState& complete_initial_robot_state)
  {
    *complete_initial_robot_state_ = complete_initial_robot_state;
    complete_initial_robot_state_->update();
  }

  /** \brief Gets the initial state of the robot.
   *
   * @return The initial state of the robot.
   */
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

  /** \brief Gets the planning context specification.
   *
   * @return The planning context specification
   */
  const OMPLPlanningContextSpecification& getSpecification() const
  {
    return spec_;
  }

  /** \brief Gets the planner specification configuration in the planning context specification.
   *
   * @return The planner specification configuration.
   */
  const std::map<std::string, std::string>& getSpecificationConfig() const
  {
    return spec_.config_.config;
  }

  /** \brief Sets the planner specification configuration in the planning context specification.
   *
   * @param [in] config The planner specification configuration to set.
   */
  void setSpecificationConfig(const std::map<std::string, std::string>& config)
  {
    spec_.config_.config = config;
  }

  /** \brief Gets the constraint sampler manager from the planning context specification.
   *
   * @return The constraint sampler manager.
   */
  const constraint_samplers::ConstraintSamplerManagerPtr& getConstraintSamplerManager()
  {
    return spec_.csm_;
  }

  /** \brief Sets the constraint sampler manager from the planning context specification.
   *
   * @param [in] csm The constraint sampler manager to set.
   */
  void setConstraintSamplerManager(const constraint_samplers::ConstraintSamplerManagerPtr& csm)
  {
    spec_.csm_ = csm;
  }

  /** \brief Get the maximum number of sampling attempts allowed when sampling states is needed.
   *
   * @return The maximum number of sampling attempts allowed.
   */
  unsigned int getMaximumStateSamplingAttempts() const
  {
    return max_state_sampling_attempts_;
  }

  /** \brief Set the maximum number of sampling attempts allowed when sampling states is needed.
   *
   * @param [in] max_state_sampling_attempts The maximum number of sampling attempts allowed.
   */
  void setMaximumStateSamplingAttempts(unsigned int max_state_sampling_attempts)
  {
    max_state_sampling_attempts_ = max_state_sampling_attempts;
  }

  /** \brief Get the maximum number of sampling attempts allowed when sampling goals is needed.
   *
   * @return The maximum number of sampling attempts allowed when sampling goals is needed.
   */
  unsigned int getMaximumGoalSamplingAttempts() const
  {
    return max_goal_sampling_attempts_;
  }

  /** \brief Set the maximum number of sampling attempts allowed when sampling goals is needed.
   *
   * @param [in] The maximum number of sampling attempts allowed when sampling goals is needed.
   */
  void setMaximumGoalSamplingAttempts(unsigned int max_goal_sampling_attempts)
  {
    max_goal_sampling_attempts_ = max_goal_sampling_attempts;
  }

  /** \brief Get the maximum number of valid goal samples to store.
   *
   * @return The maximum number of valid goal samples stored.
   */
  unsigned int getMaximumGoalSamples() const
  {
    return max_goal_samples_;
  }

  /** \brief Set the maximum number of valid goal samples to store.
   *
   * @param [in] max_goal_samples The maximum number of goal samples to store.
   */
  void setMaximumGoalSamples(unsigned int max_goal_samples)
  {
    max_goal_samples_ = max_goal_samples;
  }

  /** \brief Get the maximum number of planning threads allowed.
   *
   * @return The maximum number of planning threads.
   */
  unsigned int getMaximumPlanningThreads() const
  {
    return max_planning_threads_;
  }

  /** \brief Set the maximum number of planning threads.
   *
   * @param [in] max_planning_threads The maximum number of planning threads.
   */
  void setMaximumPlanningThreads(unsigned int max_planning_threads)
  {
    max_planning_threads_ = max_planning_threads;
  }

  /** \brief Get the maximum solution segment length.
   *
   * @return The maximum solution segment length.
   */
  double getMaximumSolutionSegmentLength() const
  {
    return max_solution_segment_length_;
  }

  /** \brief Set the maximum solution segment length.
   *
   * @param [in] mssl The maximum solution segment length.
   */
  void setMaximumSolutionSegmentLength(double mssl)
  {
    max_solution_segment_length_ = mssl;
  }

  /** \brief Get the minimum number of waypoints along the solution path.
   *
   * @return The minimum waypoint count.
   */
  unsigned int getMinimumWaypointCount() const
  {
    return minimum_waypoint_count_;
  }

  /** \brief Set the minimum number of waypoints along the solution path.
   *
   * @param [in] mwc The minimum waypoint count.
   */
  void setMinimumWaypointCount(unsigned int mwc)
  {
    minimum_waypoint_count_ = mwc;
  }

  /**
   * \brief Returns whether or not the context is set to simplify solutions.
   *
   * @return Whether or not the context is set to simplify solutions.
   */
  bool simplifySolutions() const
  {
    return simplify_solutions_;
  }

  /**
   * \brief Sets whether or not the context is set to simplify solutions.
   *
   * @param [in] flag Whether or not the context is set to simplify solutions.
   */
  void simplifySolutions(bool flag)
  {
    simplify_solutions_ = flag;
  }

protected:
  OMPLPlanningContextSpecification spec_;                   /**< \brief specification for the plannning context */
  robot_state::RobotStatePtr complete_initial_robot_state_; /**< \brief the complete initial state of the robot */

  unsigned int max_goal_samples_; /**< \brief maximum number of valid states to store in the goal region for any
                                     planning request (when such sampling is possible) */
  unsigned int max_state_sampling_attempts_; /**< \brief maximum number of attempts to be made at sampling a state when
                                                attempting to find valid states that satisfy some set of constraints */
  unsigned int
      max_goal_sampling_attempts_;      /**< \brief maximum number of attempts to be made at sampling a goal states */
  unsigned int max_planning_threads_;   /**< \brief when planning in parallel, this is the maximum number of threads to
                                           use at one time */
  double max_solution_segment_length_;  /**< \brief the maximum length that is allowed for segments that make up the
                                           motion plan; by default this is 1% from the extent of the space */
  unsigned int minimum_waypoint_count_; /**< \brief the minimum number of points to include on the solution path
                                           (interpolation is used to reach this number, if needed) */
  bool simplify_solutions_;             /**< \brief Flag to indicate if solutions should be simplified */
};
}

#endif
