/**
 * @file collision_check.h
 * @brief This defines a cost function for collision checking.
 *
 * @author Jorge Nicho
 * @date March 30, 2016
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
#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_COST_FUNCTIONS_COLLISION_CHECK_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_COST_FUNCTIONS_COLLISION_CHECK_H_

#include <Eigen/Sparse>
#include <moveit/robot_model/robot_model.h>
#include <industrial_collision_detection/collision_robot_industrial.h>
#include <industrial_collision_detection/collision_world_industrial.h>
#include "stomp_moveit/cost_functions/stomp_cost_function.h"

namespace stomp_moveit
{
namespace cost_functions
{

class CollisionCheck : public StompCostFunction
{
public:
  CollisionCheck();
  virtual ~CollisionCheck();

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,XmlRpc::XmlRpcValue& config) override;

  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code) override;


  /**
   * @brief computes the state costs as a function of the parameters for each time step.
   * @param parameters [num_dimensions] num_parameters - policy parameters to execute
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'   *
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    index of the noisy trajectory whose cost is being evaluated.   *
   * @param costs             vector containing the state costs per timestep.
   * @param validity          whether or not the trajectory is valid
   * @return true if cost were properly computed
   */
  virtual bool computeCosts(const Eigen::MatrixXd& parameters,
                            std::size_t start_timestep,
                            std::size_t num_timesteps,
                            int iteration_number,
                            int rollout_number,
                            Eigen::VectorXd& costs,
                            bool& validity) override;

  virtual std::string getGroupName() const override
  {
    return group_name_;
  }


  virtual std::string getName() const override
  {
    return "CollisionCheck/" + group_name_;
  }

  virtual void done(bool success,int total_iterations,double final_cost) override;

protected:

  std::string name_;

  // robot details
  std::string group_name_;
  moveit::core::RobotModelConstPtr robot_model_ptr_;
  moveit::core::RobotStatePtr robot_state_;

  // planning context information
  planning_scene::PlanningSceneConstPtr planning_scene_;
  moveit_msgs::MotionPlanRequest plan_request_;

  // parameters
  double collision_penalty_;
  double kernel_window_percentage_;

  // cost calculation
  Eigen::VectorXd raw_costs_;
  Eigen::ArrayXd intermediate_costs_slots_;

  // collision
  collision_detection::CollisionRequest collision_request_;
  collision_detection::CollisionRobotIndustrialConstPtr collision_robot_;
  collision_detection::CollisionWorldIndustrialConstPtr collision_world_;




};

} /* namespace cost_functions */
} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_COST_FUNCTIONS_COLLISION_CHECK_H_ */
