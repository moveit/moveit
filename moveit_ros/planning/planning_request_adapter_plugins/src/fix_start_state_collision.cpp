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

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <class_loader/class_loader.hpp>
#include <ros/ros.h>

namespace default_planner_request_adapters
{
class FixStartStateCollision : public planning_request_adapter::PlanningRequestAdapter
{
public:
  static const std::string DT_PARAM_NAME;
  static const std::string JIGGLE_PARAM_NAME;
  static const std::string DISTANCE_PARAM_NAME;
  static const std::string ATTEMPTS_PARAM_NAME;

  FixStartStateCollision() : planning_request_adapter::PlanningRequestAdapter()
  {
  }

  void initialize(const ros::NodeHandle& nh) override
  {
    if (!nh.getParam(DT_PARAM_NAME, max_dt_offset_))
    {
      max_dt_offset_ = 0.5;
      ROS_INFO_STREAM("Param '" << DT_PARAM_NAME << "' was not set. Using default value: " << max_dt_offset_);
    }
    else
      ROS_INFO_STREAM("Param '" << DT_PARAM_NAME << "' was set to " << max_dt_offset_);

    if (!nh.getParam(JIGGLE_PARAM_NAME, jiggle_fraction_))
    {
      jiggle_fraction_ = 0.02;
      ROS_INFO_STREAM("Param '" << JIGGLE_PARAM_NAME << "' was not set. Using default value: " << jiggle_fraction_);
    }
    else
      ROS_INFO_STREAM("Param '" << JIGGLE_PARAM_NAME << "' was set to " << jiggle_fraction_);

    if (!nh.getParam(DISTANCE_PARAM_NAME, max_jiggle_distance_))
    {
      max_jiggle_distance_ = 0.02;
      ROS_INFO_STREAM("Param '" << DISTANCE_PARAM_NAME
                                << "' was not set. Using default value: " << max_jiggle_distance_);
    }
    else
      ROS_INFO_STREAM("Param '" << DISTANCE_PARAM_NAME << "' was set to " << max_jiggle_distance_);
    max_jiggle_distance_squared_ = max_jiggle_distance_ * max_jiggle_distance_;  // Square the rosparam

    if (!nh.getParam(ATTEMPTS_PARAM_NAME, max_sampling_attempts_))
    {
      max_sampling_attempts_ = 100;
      ROS_INFO_STREAM("Param '" << ATTEMPTS_PARAM_NAME
                                << "' was not set. Using default value: " << max_sampling_attempts_);
    }
    else
    {
      if (max_sampling_attempts_ < 1)
      {
        max_sampling_attempts_ = 1;
        ROS_WARN_STREAM("Param '" << ATTEMPTS_PARAM_NAME << "' needs to be at least 1.");
      }
      ROS_INFO_STREAM("Param '" << ATTEMPTS_PARAM_NAME << "' was set to " << max_sampling_attempts_);
    }
  }

  std::string getDescription() const override
  {
    return "Fix Start State In Collision";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& added_path_index) const override
  {
    ROS_DEBUG("Running '%s'", getDescription().c_str());

    // get the specified start state
    moveit::core::RobotState start_state = planning_scene->getCurrentState();
    moveit::core::robotStateMsgToRobotState(planning_scene->getTransforms(), req.start_state, start_state);

    collision_detection::CollisionRequest creq;
    creq.group_name = req.group_name;
    collision_detection::CollisionResult cres;
    planning_scene->checkCollision(creq, cres, start_state);
    if (cres.collision)
    {
      // Rerun in verbose mode
      collision_detection::CollisionRequest vcreq = creq;
      collision_detection::CollisionResult vcres;
      vcreq.verbose = true;
      planning_scene->checkCollision(vcreq, vcres, start_state);

      if (creq.group_name.empty())
        ROS_INFO("Start state appears to be in collision. Attempting to fix.");
      else
        ROS_INFO_STREAM("Start state appears to be in collision with respect to group " << creq.group_name
                                                                                        << ". Attempting to fix.");

      moveit::core::RobotStatePtr original_state(new moveit::core::RobotState(start_state));
      random_numbers::RandomNumberGenerator& rng = original_state->getRandomNumberGenerator();

      const std::vector<const moveit::core::JointModel*>& jmodels =
          planning_scene->getRobotModel()->hasJointModelGroup(req.group_name) ?
              planning_scene->getRobotModel()->getJointModelGroup(req.group_name)->getJointModels() :
              planning_scene->getRobotModel()->getJointModels();

      // Record the cartesian pose of every link to ensure no sudden movements will occur.
      // The "furthest" links are checked first, as their movement is the largest.

      // Check if an end effector link is set that is not part of the robot
      const moveit::core::LinkModel* ee_link_model;
      if (!req.goal_constraints[0].position_constraints.empty())
        ee_link_model = start_state.getLinkModel(req.goal_constraints[0].position_constraints[0].link_name);
      else if (!req.goal_constraints[0].orientation_constraints.empty())
        ee_link_model = start_state.getLinkModel(req.goal_constraints[0].orientation_constraints[0].link_name);

      // Collect the links that will be checked
      std::vector<const moveit::core::LinkModel*> link_models;
      link_models.resize(jmodels.size());
      for (std::size_t i = jmodels.size() - 1; i > 0; --i)
        link_models[i] = jmodels[i]->getChildLinkModel();

      if (std::find(link_models.begin(), link_models.end(), ee_link_model) == link_models.end())
      {  // If the end effector is a separate link, add it to the front of the vector
        link_models.resize(jmodels.size() + 1);
        link_models[0] = ee_link_model;
        for (std::size_t i = jmodels.size() - 1; i > 0; --i)
          link_models[i + 1] = jmodels[i]->getChildLinkModel();
      }

      // Store the unchanged link positions
      EigenSTL::vector_Isometry3d original_link_poses;
      original_link_poses.resize(link_models.size());
      Eigen::Isometry3d new_link_pose;

      for (std::size_t i = 0; i < link_models.size(); ++i)
        original_link_poses[i] = start_state.getGlobalLinkTransform(link_models[i]);

      double dist, max_dist;
      double local_jiggle_fraction = jiggle_fraction_;
      bool found = false;
      for (int c = 0; !found && c < max_sampling_attempts_; ++c)
      {
        // Change each joint slightly ("jiggle")
        for (std::size_t i = 0; !found && i < jmodels.size(); ++i)
        {
          std::vector<double> sampled_variable_values(jmodels[i]->getVariableCount());
          const double* original_values = original_state->getJointPositions(jmodels[i]);
          jmodels[i]->getVariableRandomPositionsNearBy(rng, &sampled_variable_values[0], original_values,
                                                       jmodels[i]->getMaximumExtent() * jiggle_fraction_);
          start_state.setJointPositions(jmodels[i], sampled_variable_values);
        }

        // Confirm that no links move far after jiggling
        max_dist = -1;
        bool euclidean_distance_ok = true;
        for (std::size_t i = 0; !found && i < link_models.size(); ++i)
        {
          new_link_pose = start_state.getGlobalLinkTransform(link_models[i]);
          dist = (new_link_pose.translation() - original_link_poses[i].translation()).squaredNorm();

          if (dist > max_dist)
            max_dist = dist;
          if (dist > max_jiggle_distance_squared_)
          {
            // Reduce jiggle fraction if it caused excessive movement
            local_jiggle_fraction = local_jiggle_fraction * ((max_sampling_attempts_ - 2) / max_sampling_attempts_);
            euclidean_distance_ok = false;
            break;
          }
        }
        if (!euclidean_distance_ok)
          continue;

        // Check changed state for collision
        collision_detection::CollisionResult cres;
        planning_scene->checkCollision(creq, cres, start_state);
        if (!cres.collision)
        {
          found = true;
          ROS_INFO_STREAM("Start state fixed successfully. Found valid state at cumulative joint distance "
                          << original_state->distance(start_state) << " and max euclidean distance " << sqrt(max_dist)
                          << " after " << c << " attempts");
        }
      }

      if (found)
      {
        planning_interface::MotionPlanRequest req2 = req;
        moveit::core::robotStateToRobotStateMsg(start_state, req2.start_state);
        bool solved = planner(planning_scene, req2, res);
        if (solved && !res.trajectory_->empty())
        {
          // heuristically decide a duration offset for the trajectory (induced by the additional point added as a
          // prefix to the computed trajectory)
          res.trajectory_->setWayPointDurationFromPrevious(0, std::min(max_dt_offset_,
                                                                       res.trajectory_->getAverageSegmentDuration()));
          res.trajectory_->addPrefixWayPoint(original_state, 0.0);
          // we add a prefix point, so we need to bump any previously added index positions
          for (std::size_t& added_index : added_path_index)
            added_index++;
          added_path_index.push_back(0);
        }
        return solved;
      }
      else
      {
        ROS_WARN_STREAM("Unable to find a valid state nearby the start state (using jiggle fraction of "
                        << jiggle_fraction_ << ", max euclidean distance of " << max_jiggle_distance_ << " and "
                        << max_sampling_attempts_
                        << " max sampling attempts). Passing the unchanged planning request to the planner.");
        return planner(planning_scene, req, res);
      }
    }
    else
    {
      if (creq.group_name.empty())
        ROS_DEBUG("Start state is valid");
      else
        ROS_DEBUG_STREAM("Start state is valid with respect to group " << creq.group_name);
      return planner(planning_scene, req, res);
    }
  }

private:
  double max_dt_offset_;
  double jiggle_fraction_;
  double max_jiggle_distance_;
  double max_jiggle_distance_squared_;
  int max_sampling_attempts_;
};

const std::string FixStartStateCollision::DT_PARAM_NAME = "start_state_max_dt";
const std::string FixStartStateCollision::JIGGLE_PARAM_NAME = "jiggle_fraction";
const std::string FixStartStateCollision::DISTANCE_PARAM_NAME = "max_jiggle_distance";
const std::string FixStartStateCollision::ATTEMPTS_PARAM_NAME = "max_sampling_attempts";
}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::FixStartStateCollision,
                            planning_request_adapter::PlanningRequestAdapter);
