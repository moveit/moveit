/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Raghavender Sahdev.
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
 *   * Neither the name of Raghavender Sahdev nor the names of its
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

/* Author: Raghavender Sahdev */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <class_loader/class_loader.hpp>
#include <ros/ros.h>

#include <chomp_motion_planner/chomp_planner.h>
#include <chomp_motion_planner/chomp_parameters.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>

#include <tf/transform_listener.h>

#include <moveit/robot_state/conversions.h>

#include <vector>
#include <eigen3/Eigen/Core>

namespace chomp
{
class OptimizerAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:
  OptimizerAdapter() : planning_request_adapter::PlanningRequestAdapter(), nh_("~")
  {
    if (!nh_.getParam("planning_time_limit", params_.planning_time_limit_))
    {
      params_.planning_time_limit_ = 10.0;
      ROS_INFO_STREAM("Param planning_time_limit was not set. Using default value: " << params_.planning_time_limit_);
    }
    if (!nh_.getParam("max_iterations", params_.max_iterations_))
    {
      params_.max_iterations_ = 200;
      ROS_INFO_STREAM("Param max_iterations was not set. Using default value: " << params_.max_iterations_);
    }
    if (!nh_.getParam("max_iterations_after_collision_free", params_.max_iterations_after_collision_free_))
    {
      params_.max_iterations_after_collision_free_ = 5;
      ROS_INFO_STREAM("Param max_iterations_after_collision_free was not set. Using default value: "
                      << params_.max_iterations_after_collision_free_);
    }
    if (!nh_.getParam("smoothness_cost_weight", params_.smoothness_cost_weight_))
    {
      params_.smoothness_cost_weight_ = 0.1;
      ROS_INFO_STREAM(
          "Param smoothness_cost_weight was not set. Using default value: " << params_.smoothness_cost_weight_);
    }
    if (!nh_.getParam("obstacle_cost_weight", params_.obstacle_cost_weight_))
    {
      params_.obstacle_cost_weight_ = 1.0;
      ROS_INFO_STREAM("Param obstacle_cost_weight was not set. Using default value: " << params_.obstacle_cost_weight_);
    }
    if (!nh_.getParam("learning_rate", params_.learning_rate_))
    {
      params_.learning_rate_ = 0.01;
      ROS_INFO_STREAM("Param learning_rate was not set. Using default value: " << params_.learning_rate_);
    }
    if (!nh_.getParam("smoothness_cost_velocity", params_.smoothness_cost_velocity_))
    {
      params_.smoothness_cost_velocity_ = 0.0;
      ROS_INFO_STREAM(
          "Param smoothness_cost_velocity was not set. Using default value: " << params_.smoothness_cost_velocity_);
    }
    if (!nh_.getParam("smoothness_cost_acceleration", params_.smoothness_cost_acceleration_))
    {
      params_.smoothness_cost_acceleration_ = 1.0;
      ROS_INFO_STREAM("Param smoothness_cost_acceleration was not set. Using default value: "
                      << params_.smoothness_cost_acceleration_);
    }
    if (!nh_.getParam("smoothness_cost_jerk", params_.smoothness_cost_jerk_))
    {
      params_.smoothness_cost_jerk_ = 0.0;
      ROS_INFO_STREAM(
          "Param smoothness_cost_jerk_ was not set. Using default value: " << params_.smoothness_cost_jerk_);
    }
    if (!nh_.getParam("ridge_factor", params_.ridge_factor_))
    {
      params_.ridge_factor_ = 0.0;
      ROS_INFO_STREAM("Param ridge_factor_ was not set. Using default value: " << params_.ridge_factor_);
    }
    if (!nh_.getParam("use_pseudo_inverse", params_.use_pseudo_inverse_))
    {
      params_.use_pseudo_inverse_ = 0.0;
      ROS_INFO_STREAM("Param use_pseudo_inverse_ was not set. Using default value: " << params_.use_pseudo_inverse_);
    }
    if (!nh_.getParam("pseudo_inverse_ridge_factor", params_.pseudo_inverse_ridge_factor_))
    {
      params_.pseudo_inverse_ridge_factor_ = 1e-4;
      ROS_INFO_STREAM("Param pseudo_inverse_ridge_factor was not set. Using default value: "
                      << params_.pseudo_inverse_ridge_factor_);
    }
    if (!nh_.getParam("joint_update_limit", params_.joint_update_limit_))
    {
      params_.joint_update_limit_ = 0.1;
      ROS_INFO_STREAM("Param joint_update_limit was not set. Using default value: " << params_.joint_update_limit_);
    }
    if (!nh_.getParam("min_clearence", params_.min_clearence_))
    {
      params_.min_clearence_ = 0.2;
      ROS_INFO_STREAM("Param min_clearence was not set. Using default value: " << params_.min_clearence_);
    }
    if (!nh_.getParam("collision_threshold", params_.collision_threshold_))
    {
      params_.collision_threshold_ = 0.07;
      ROS_INFO_STREAM("Param collision_threshold_ was not set. Using default value: " << params_.collision_threshold_);
    }
    if (!nh_.getParam("use_stochastic_descent", params_.use_stochastic_descent_))
    {
      params_.use_stochastic_descent_ = true;
      ROS_INFO_STREAM(
          "Param use_stochastic_descent was not set. Using default value: " << params_.use_stochastic_descent_);
    }
    if (!nh_.getParam("trajectory_initialization_method", params_.trajectory_initialization_method_))
    {
      params_.trajectory_initialization_method_ = std::string("fillTrajectory");
      ROS_INFO_STREAM("Param trajectory_initialization_method was not set. Using New value as: "
                      << params_.trajectory_initialization_method_);
    }
  }

  virtual std::string getDescription() const
  {
    return "CHOMP Optimizer";
  }

  virtual bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& ps,
                            const planning_interface::MotionPlanRequest& req,
                            planning_interface::MotionPlanResponse& res,
                            std::vector<std::size_t>& added_path_index) const
  {
    // following call to planner() calls the OMPL planner and stores the trajectory inside the MotionPlanResponse res
    // variable which is then used by CHOMP for optimization of the computed trajectory
    bool solved = planner(ps, req, res);

    // create a hybrid collision detector to set the collision checker as hybrid
    collision_detection::CollisionDetectorAllocatorPtr hybrid_cd(
        collision_detection::CollisionDetectorAllocatorHybrid::create());

    // create a writable planning scene
    planning_scene::PlanningScenePtr planning_scene = ps->diff();
    ROS_INFO_STREAM("Configuring Planning Scene for CHOMP ....");
    planning_scene->setActiveCollisionDetector(hybrid_cd, true);

    chomp::ChompPlanner chompPlanner;
    planning_interface::MotionPlanDetailedResponse res_detailed;
    moveit_msgs::MotionPlanDetailedResponse res_detailed_moveit_msgs;

    // populate the trajectory to pass to CHOMPPlanner::solve() method. Obtain trajectory from OMPL's
    // planning_interface::MotionPlanResponse object and put / populate it in the
    // moveit_msgs::MotionPlanDetailedResponse object
    moveit_msgs::RobotTrajectory trajectory_msgs_from_response;
    res.trajectory_->getRobotTrajectoryMsg(trajectory_msgs_from_response);
    res_detailed_moveit_msgs.trajectory.resize(1);
    res_detailed_moveit_msgs.trajectory[0] = trajectory_msgs_from_response;

    bool planning_success = chompPlanner.solve(planning_scene, req, params_, res_detailed_moveit_msgs);

    if (planning_success)
    {
      res_detailed.trajectory_.resize(1);
      res_detailed.trajectory_[0] = robot_trajectory::RobotTrajectoryPtr(
          new robot_trajectory::RobotTrajectory(res.trajectory_->getRobotModel(), res.trajectory_->getGroup()));

      moveit::core::RobotState start_state(planning_scene->getRobotModel());
      robot_state::robotStateMsgToRobotState(res_detailed_moveit_msgs.trajectory_start, start_state);
      res_detailed.trajectory_[0]->setRobotTrajectoryMsg(start_state, res_detailed_moveit_msgs.trajectory[0]);
      res_detailed.description_.push_back("plan");
      res_detailed.processing_time_ = res_detailed_moveit_msgs.processing_time;
      res_detailed.error_code_ = res_detailed_moveit_msgs.error_code;
    }
    else
      res_detailed.error_code_ = res_detailed_moveit_msgs.error_code;

    res.error_code_ = res_detailed.error_code_;

    // populate the original response object 'res' with the CHOMP's optimized trajectory.
    if (planning_success)
    {
      res.trajectory_ = res_detailed.trajectory_[0];
      res.planning_time_ = res_detailed.processing_time_[0];
    }

    return solved;
  }

private:
  ros::NodeHandle nh_;
  chomp::ChompParameters params_;
};
}

CLASS_LOADER_REGISTER_CLASS(chomp::OptimizerAdapter, planning_request_adapter::PlanningRequestAdapter);
