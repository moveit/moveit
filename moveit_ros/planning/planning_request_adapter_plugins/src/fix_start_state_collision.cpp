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
  static const std::string ATTEMPTS_PARAM_NAME;

  FixStartStateCollision() : planning_request_adapter::PlanningRequestAdapter(), nh_("~")
  {
    if (!nh_.getParam(DT_PARAM_NAME, max_dt_offset_))
    {
      max_dt_offset_ = 0.5;
      ROS_INFO_STREAM("Param '" << DT_PARAM_NAME << "' was not set. Using default value: " << max_dt_offset_);
    }
    else
      ROS_INFO_STREAM("Param '" << DT_PARAM_NAME << "' was set to " << max_dt_offset_);

    if (!nh_.getParam(JIGGLE_PARAM_NAME, jiggle_fraction_))
    {
      jiggle_fraction_ = 0.02;
      ROS_INFO_STREAM("Param '" << JIGGLE_PARAM_NAME << "' was not set. Using default value: " << jiggle_fraction_);
    }
    else
      ROS_INFO_STREAM("Param '" << JIGGLE_PARAM_NAME << "' was set to " << jiggle_fraction_);

    if (!nh_.getParam(ATTEMPTS_PARAM_NAME, sampling_attempts_))
    {
      sampling_attempts_ = 100;
      ROS_INFO_STREAM("Param '" << ATTEMPTS_PARAM_NAME << "' was not set. Using default value: " << sampling_attempts_);
    }
    else
    {
      if (sampling_attempts_ < 1)
      {
        sampling_attempts_ = 1;
        ROS_WARN_STREAM("Param '" << ATTEMPTS_PARAM_NAME << "' needs to be at least 1.");
      }
      ROS_INFO_STREAM("Param '" << ATTEMPTS_PARAM_NAME << "' was set to " << sampling_attempts_);
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
    robot_state::RobotState start_state = planning_scene->getCurrentState();
    robot_state::robotStateMsgToRobotState(planning_scene->getTransforms(), req.start_state, start_state);

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
        ROS_INFO("Start state appears to be in collision");
      else
        ROS_INFO_STREAM("Start state appears to be in collision with respect to group " << creq.group_name);

      robot_state::RobotStatePtr prefix_state(new robot_state::RobotState(start_state));
      random_numbers::RandomNumberGenerator& rng = prefix_state->getRandomNumberGenerator();

      const std::vector<const robot_model::JointModel*>& jmodels =
          planning_scene->getRobotModel()->hasJointModelGroup(req.group_name) ?
              planning_scene->getRobotModel()->getJointModelGroup(req.group_name)->getJointModels() :
              planning_scene->getRobotModel()->getJointModels();

      bool found = false;
      for (int c = 0; !found && c < sampling_attempts_; ++c)
      {
        for (std::size_t i = 0; !found && i < jmodels.size(); ++i)
        {
          std::vector<double> sampled_variable_values(jmodels[i]->getVariableCount());
          const double* original_values = prefix_state->getJointPositions(jmodels[i]);
          jmodels[i]->getVariableRandomPositionsNearBy(rng, &sampled_variable_values[0], original_values,
                                                       jmodels[i]->getMaximumExtent() * jiggle_fraction_);
          start_state.setJointPositions(jmodels[i], sampled_variable_values);
          collision_detection::CollisionResult cres;
          planning_scene->checkCollision(creq, cres, start_state);
          if (!cres.collision)
          {
            found = true;
            ROS_INFO("Found a valid state near the start state at distance %lf after %d attempts",
                     prefix_state->distance(start_state), c);
          }
        }
      }

      if (found)
      {
        planning_interface::MotionPlanRequest req2 = req;
        robot_state::robotStateToRobotStateMsg(start_state, req2.start_state);
        bool solved = planner(planning_scene, req2, res);
        if (solved && !res.trajectory_->empty())
        {
          // heuristically decide a duration offset for the trajectory (induced by the additional point added as a
          // prefix to the computed trajectory)
          res.trajectory_->setWayPointDurationFromPrevious(
              0, std::min(max_dt_offset_, res.trajectory_->getAverageSegmentDuration()));
          res.trajectory_->addPrefixWayPoint(prefix_state, 0.0);
          // we add a prefix point, so we need to bump any previously added index positions
          for (std::size_t i = 0; i < added_path_index.size(); ++i)
            added_path_index[i]++;
          added_path_index.push_back(0);
        }
        return solved;
      }
      else
      {
        ROS_WARN("Unable to find a valid state nearby the start state (using jiggle fraction of %lf and %u sampling "
                 "attempts). Passing the original planning request to the planner.",
                 jiggle_fraction_, sampling_attempts_);
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
  ros::NodeHandle nh_;
  double max_dt_offset_;
  double jiggle_fraction_;
  int sampling_attempts_;
};

const std::string FixStartStateCollision::DT_PARAM_NAME = "start_state_max_dt";
const std::string FixStartStateCollision::JIGGLE_PARAM_NAME = "jiggle_fraction";
const std::string FixStartStateCollision::ATTEMPTS_PARAM_NAME = "max_sampling_attempts";
}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::FixStartStateCollision,
                            planning_request_adapter::PlanningRequestAdapter);
