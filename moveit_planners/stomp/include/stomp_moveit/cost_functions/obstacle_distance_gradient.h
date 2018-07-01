/**
 * @file obstacle_distance_gradient.h
 * @brief This defines a Robot Model for the Stomp Planner.
 *
 * @author Jorge Nicho
 * @date Jul 22, 2016
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

#ifndef MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_COST_FUNCTIONS_OBSTACLE_DISTANCE_GRADIENT_H_
#define MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_COST_FUNCTIONS_OBSTACLE_DISTANCE_GRADIENT_H_

#include <stomp_moveit/cost_functions/stomp_cost_function.h>
#include <array>

namespace stomp_moveit
{
namespace cost_functions
{
/**
 * @class stomp_moveit::cost_functions::StompCostFunction
 * @brief Assigns a cost value to  each robot state by evaluating the minimum distance between the robot and the nearest obstacle.
 *
 * @par Examples:
 * All examples are located here @ref stomp_moveit_examples
 */
class ObstacleDistanceGradient : public StompCostFunction
{
public:
  ObstacleDistanceGradient();
  virtual ~ObstacleDistanceGradient();

  /**
   * @brief Initializes and configures the Cost Function.  Calls the configure method and passes the 'config' value.
   * @param robot_model_ptr A pointer to the robot model.
   * @param group_name      The designated planning group.
   * @param config          The configuration data.  Usually loaded from the ros parameter server
   * @return true if succeeded, false otherwise.
   */
  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr, const std::string& group_name,
                          XmlRpc::XmlRpcValue& config) override;

  /**
   * @brief Sets internal members of the plugin from the configuration data.
   * @param config  The configuration data .  Usually loaded from the ros parameter server
   * @return  true if succeeded, false otherwise.
   */
  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;


  /**
   * @brief Stores the planning details which will be used during the costs calculations.
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
   * @brief computes the state costs by calculating the minimum distance between the robot and an obstacle.
   * @param parameters        The parameter values to evaluate for state costs [num_dimensions x num_parameters]
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'   *
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    index of the noisy trajectory whose cost is being evaluated.   *
   * @param costs             vector containing the state costs per timestep.
   * @param validity          whether or not the trajectory is valid
   * @return false if there was an irrecoverable failure, true otherwise.
   */
  virtual bool computeCosts(const Eigen::MatrixXd& parameters, std::size_t start_timestep, std::size_t num_timesteps,
                            int iteration_number, int rollout_number, Eigen::VectorXd& costs, bool& validity) override;

  virtual std::string getGroupName() const override
  {
    return group_name_;
  }

  virtual std::string getName() const override
  {
    return name_ + "/" + group_name_  ;
  }

  virtual void done(bool success,int total_iterations,double final_cost,const Eigen::MatrixXd& parameters) override;


protected:

  /**
   * @brief Checks for collision between consecutive points by dividing the joint move into sub-moves where the maximum joint motion
   *        can not exceed the @e longest_valid_joint_move value.
   * @param start                     The start joint pose
   * @param end                       The end joint pose
   * @param longest_valid_joint_move  The maximum distance that the joints are allowed to move before checking for collisions.
   * @return  True if the interval is collision free, false otherwise.
   */
  bool checkIntermediateCollisions(const Eigen::VectorXd& start, const Eigen::VectorXd& end,double longest_valid_joint_move);


  std::string name_;

  // robot details
  std::string group_name_;
  moveit::core::RobotModelConstPtr robot_model_ptr_;
  moveit::core::RobotStatePtr robot_state_;

  // intermediate collision check support
  std::array<moveit::core::RobotStatePtr,3 > intermediate_coll_states_;   /**< @brief Used in checking collisions between to consecutive poses*/


  // planning context information
  planning_scene::PlanningSceneConstPtr planning_scene_;
  moveit_msgs::MotionPlanRequest plan_request_;

  // distance and collision check
  collision_detection::CollisionRequest collision_request_;
  collision_detection::CollisionResult collision_result_;

  // parameters
  double max_distance_;               /**< @brief maximum distance from at which the trajectory will be penalized */
  double longest_valid_joint_move_;   /**< @brief how far can a joint move in between consecutive trajectory points */

};

} /* namespace cost_functions */
} /* namespace stomp_moveit */

#endif /* MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_COST_FUNCTIONS_OBSTACLE_DISTANCE_GRADIENT_H_ */
