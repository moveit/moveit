/**
 * @file joint_limits.h
 * @brief This defines a joint limit filter.
 *
 * @author Jorge Nicho
 * @date April 1, 2016
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
#ifndef MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISY_FILTERS_JOINT_LIMITS_H_
#define MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISY_FILTERS_JOINT_LIMITS_H_

#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <stomp_moveit/noisy_filters/stomp_noisy_filter.h>

namespace stomp_moveit
{
namespace noisy_filters
{
/**
 * @class stomp_moveit::noisy_filters::JointLimits
 * @brief Checks that the joint values are within the limits as defined in the urdf file.  It modifies
 *  the values of those joints that exceed the limits.
 *
 * @par Examples:
 * All examples are located here @ref stomp_moveit_examples
 *
 */
class JointLimits : public StompNoisyFilter
{
public:
  JointLimits();
  virtual ~JointLimits();

  /** @brief see base class for documentation*/
  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr, const std::string& group_name,
                          const XmlRpc::XmlRpcValue& config) override;

  /** @brief see base class for documentation*/
  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

  /** @brief see base class for documentation*/
  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                    const moveit_msgs::MotionPlanRequest& req,
                                    const stomp_core::StompConfiguration& config,
                                    moveit_msgs::MoveItErrorCodes& error_code) override;

  virtual std::string getGroupName() const override
  {
    return group_name_;
  }

  virtual std::string getName() const override
  {
    return "JointLimits/" + group_name_;
  }

  /**
   * @brief Sets the joint values to the closest joint limit whenever it's found outside the allowed range.
   *
   * @param start_timestep    Start index into the 'parameters' array, usually 0.
   * @param num_timesteps     Number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop.
   * @param rollout_number    Index of the noisy trajectory whose cost is being evaluated.
   * @param parameters        Output argument containing the parameters to be filtered [num_dimensions x num_timesteps].
   * @param filtered          Output argument that's set to 'true' if the parameters were changed according to the
   * filtering method.
   * @return false if there was an irrecoverable failure, true otherwise.
   */
  virtual bool filter(std::size_t start_timestep, std::size_t num_timesteps, int iteration_number, int rollout_number,
                      Eigen::MatrixXd& parameters, bool& filtered) override;

protected:
  moveit::core::RobotModelConstPtr robot_model_;
  std::string group_name_;

  // options
  bool lock_start_;
  bool lock_goal_;

  // start and goal
  moveit::core::RobotStatePtr start_state_;
  moveit::core::RobotStatePtr goal_state_;
};

} /* namespace filters */
} /* namespace stomp_moveit */

#endif /* MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISY_FILTERS_JOINT_LIMITS_H_ */
