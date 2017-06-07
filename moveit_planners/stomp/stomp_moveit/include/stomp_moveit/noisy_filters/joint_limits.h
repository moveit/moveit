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
 * @license Software License Agreement (Apache License)\n
 * \n
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at\n
 * \n
 * http://www.apache.org/licenses/LICENSE-2.0\n
 * \n
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISY_FILTERS_JOINT_LIMITS_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISY_FILTERS_JOINT_LIMITS_H_

#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <stomp_moveit/noisy_filters/stomp_noisy_filter.h>

namespace stomp_moveit
{
namespace noisy_filters
{

class JointLimits : public StompNoisyFilter
{
public:
  JointLimits();
  virtual ~JointLimits();

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) override;

  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
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
   * @brief filters the parameters and modifies the original values
   *
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    index of the noisy trajectory whose cost is being evaluated.
   * @param parameters [num_dimensions] x [num_timesteps]
   * @return false if no filtering was applied
   */
  virtual bool filter(std::size_t start_timestep,
                      std::size_t num_timesteps,
                      int iteration_number,
                      int rollout_number,
                      Eigen::MatrixXd& parameters,
                      bool& filtered) override;

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

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISY_FILTERS_JOINT_LIMITS_H_ */
