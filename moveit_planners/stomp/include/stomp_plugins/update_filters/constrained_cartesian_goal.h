/**
 * @file constrained_cartesian_goal.h
 * @brief This defines a underconstrained goal update filter. It forces
 * the goal cartesian tool pose into the task space.
 *
 * @author Jorge Nicho
 * @date June 3, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_CONSTRAINED_CARTESIAN_GOAL_H_
#define MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_CONSTRAINED_CARTESIAN_GOAL_H_

#include <stomp_moveit/update_filters/stomp_update_filter.h>
#include <stomp_moveit/utils/kinematics.h>

namespace stomp_moveit
{
namespace update_filters
{

/**
 * @class stomp_moveit::update_filters::ConstrainedCartesianGoal
 * @brief Forces the goal cartesian tool pose into the task space.
 *
 * @par Examples:
 * All examples are located here @ref stomp_plugins_examples
 *
 */
class ConstrainedCartesianGoal : public StompUpdateFilter
{
public:
  ConstrainedCartesianGoal();
  virtual ~ConstrainedCartesianGoal();

  /**
   * @brief Initializes and configures.
   * @param robot_model_ptr A pointer to the robot model.
   * @param group_name      The designated planning group.
   * @param config          The configuration data.  Usually loaded from the ros parameter server
   * @return true if succeeded, false otherwise.
   */
  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) override;

  /**
   * @brief Sets internal members of the plugin from the configuration data.
   * @param config  The configuration data.  Usually loaded from the ros parameter server
   * @return  true if succeeded, false otherwise.
   */
  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

  /**
   * @brief Stores the planning details.
   * @param planning_scene      A smart pointer to the planning scene
   * @param req                 The motion planning request
   * @param config              The  Stomp configuration.
   * @param error_code          Moveit error code.
   * @return  true if succeeded, false otherwise.
   */
  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code) override;

  /**
   * @brief Forces the goal to be within the tool's task manifold.
   *
   * @param start_timestep    The start index into the 'parameters' array, usually 0.
   * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'.
   * @param iteration_number  The current iteration count in the optimization loop.
   * @param parameters        The current parameter values [num_dimensions x num_timesteps]
   * @param updates           The output updates to be applied to the parameters [num_dimensions x num_timesteps]
   * @param filtered          Set to 'true' if the updates were modified, false otherwise
   * @return false if there was an irrecoverable failure, true otherwise.
   */
  virtual bool filter(std::size_t start_timestep,
                      std::size_t num_timesteps,
                      int iteration_number,
                      const Eigen::MatrixXd& parameters,
                      Eigen::MatrixXd& updates,
                      bool& filtered) override;

  virtual std::string getGroupName() const
  {
    return group_name_;
  }

  virtual std::string getName() const
  {
    return name_ + "/" + group_name_;
  }

protected:

  std::string name_;
  std::string group_name_;

  // kinematics
  utils::kinematics::KinematicConfig kc_;

  // robot
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr state_;
  std::string tool_link_;
};

} /* namespace update_filters */
} /* namespace stomp_moveit */

#endif /* MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_CONSTRAINED_CARTESIAN_GOAL_H_ */
