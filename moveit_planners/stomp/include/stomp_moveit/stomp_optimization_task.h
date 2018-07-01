/**
 * @file stomp_optimization_task.h
 * @brief This defines stomp's optimization task
 *
 * @author Jorge Nicho
 * @date March 23, 2016
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
#ifndef MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_STOMP_OPTIMIZATION_TASK_H_
#define MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_STOMP_OPTIMIZATION_TASK_H_

#include <memory>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/robot_model/robot_model.h>
#include <stomp_core/task.h>
#include <stomp_moveit/cost_functions/stomp_cost_function.h>
#include <XmlRpcValue.h>
#include <pluginlib/class_loader.h>
#include <stomp_moveit/noise_generators/stomp_noise_generator.h>
#include <stomp_moveit/noisy_filters/stomp_noisy_filter.h>
#include <stomp_moveit/update_filters/stomp_update_filter.h>


namespace stomp_moveit
{

typedef pluginlib::ClassLoader<stomp_moveit::cost_functions::StompCostFunction> CostFunctionLoader;
typedef std::shared_ptr<CostFunctionLoader> CostFuctionLoaderPtr;
typedef pluginlib::ClassLoader<stomp_moveit::noisy_filters::StompNoisyFilter> NoisyFilterLoader;
typedef std::shared_ptr<NoisyFilterLoader> NoisyFilterLoaderPtr;
typedef pluginlib::ClassLoader<stomp_moveit::update_filters::StompUpdateFilter> UpdateFilterLoader;
typedef std::shared_ptr<UpdateFilterLoader> UpdateFilterLoaderPtr;
typedef pluginlib::ClassLoader<stomp_moveit::noise_generators::StompNoiseGenerator> NoiseGeneratorLoader;
typedef std::shared_ptr<NoiseGeneratorLoader> NoiseGeneratorLoaderPtr;


/**
 * @class stomp_moveit::StompOptimizationTask
 * @brief Loads and manages the STOMP plugins during the planning process.
 *
 * @par Examples:
 * All examples are located here @ref stomp_moveit_examples
 *
 */
class StompOptimizationTask: public stomp_core::Task
{
public:
  /**
   * @brief Constructor
   * @param robot_model_ptr A pointer to the robot model
   * @param group_name      The planning group name
   * @param config          The configuration parameter data
   */
  StompOptimizationTask(moveit::core::RobotModelConstPtr robot_model_ptr, std::string group_name,
                        const XmlRpc::XmlRpcValue& config);
  virtual ~StompOptimizationTask();

  /**
   * @brief Passes the planning details down to each loaded plugin
   * @param planning_scene  A smart pointer to the planning scene
   * @param req                 The motion planning request
   * @param config              The  Stomp configuration, usually loaded from the ros parameter server
   * @param error_code          Moveit error code
   * @return  true if succeeded,false otherwise.
   */
  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code);

  /**
   * @brief Generates a noisy trajectory from the parameters by calling the active Noise Generator plugin.
   * @param parameters        [num_dimensions] x [num_parameters] the current value of the optimized parameters
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    index of the noisy trajectory.
   * @param parameters_noise  the parameters + noise
   * @param noise             the noise applied to the parameters
   * @return  false if there was an irrecoverable failure, true otherwise.
   */
  virtual bool generateNoisyParameters(const Eigen::MatrixXd& parameters,
                                       std::size_t start_timestep,
                                       std::size_t num_timesteps,
                                       int iteration_number,
                                       int rollout_number,
                                       Eigen::MatrixXd& parameters_noise,
                                       Eigen::MatrixXd& noise) override;

  /**
   * @brief computes the state costs as a function of the noisy parameters for each time step. It does this by calling the loaded Cost Function plugins
   * @param parameters [num_dimensions] num_parameters - policy parameters to execute
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number index of the noisy trajectory whose cost is being evaluated.
   * @param costs vector containing the state costs per timestep.
   * @param validity whether or not the trajectory is valid
   * @return  false if there was an irrecoverable failure, true otherwise.
   */
  virtual bool computeNoisyCosts(const Eigen::MatrixXd& parameters,
                       std::size_t start_timestep,
                       std::size_t num_timesteps,
                       int iteration_number,
                       int rollout_number,
                       Eigen::VectorXd& costs,
                       bool& validity) override;

  /**
   * @brief computes the state costs as a function of the optimized parameters for each time step. It does this by calling the loaded Cost Function plugins
   * @param parameters        [num_dimensions] num_parameters - policy parameters to execute
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param costs             vector containing the state costs per timestep.
   * @param validity          whether or not the trajectory is valid
   * @return  false if there was an irrecoverable failure, true otherwise.
   */
  virtual bool computeCosts(const Eigen::MatrixXd& parameters,
                       std::size_t start_timestep,
                       std::size_t num_timesteps,
                       int iteration_number,
                       Eigen::VectorXd& costs,
                       bool& validity) override;

  /**
   * @brief Filters the given noisy parameters which is applied after noisy trajectory generation. It could be used for clipping
   * of joint limits or projecting into the null space of the Jacobian.  It accomplishes this by calling the loaded Noisy Filter plugins.
   *
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    The rollout index for this noisy parameter set
   * @param parameters        The noisy parameters     *
   * @param filtered          False if no filtering was done
   * @return  false if there was an irrecoverable failure, true otherwise.
   */
  virtual bool filterNoisyParameters(std::size_t start_timestep,
                                     std::size_t num_timesteps,
                                     int iteration_number,
                                     int rollout_number,
                                     Eigen::MatrixXd& parameters,
                                     bool& filtered) override;

  /**
   * @brief Filters the given parameters which is applied after calculating the update. It could be used for clipping of joint limits
   * or projecting the goal pose into the null space of the Jacobian. It accomplishes this by calling the loaded Update Filter plugins.
   *
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param parameters        The optimized parameters
   * @param updates           The updates to the parameters
   * @return  false if there was an irrecoverable failure, true otherwise.
   */
  virtual bool filterParameterUpdates(std::size_t start_timestep,
                                      std::size_t num_timesteps,
                                      int iteration_number,
                                      const Eigen::MatrixXd& parameters,
                                      Eigen::MatrixXd& updates) override;

  /**
   * @brief Called by STOMP at the end of each iteration.
   * @param start_timestep    The start index into the 'parameters' array, usually 0.
   * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param cost              The cost value for the current parameters.
   * @param parameters        The value of the parameters at the end of the current iteration [num_dimensions x num_timesteps].
   */
  virtual void postIteration(std::size_t start_timestep,
                                std::size_t num_timesteps,int iteration_number,double cost,const Eigen::MatrixXd& parameters);

  /**
   * @brief Called by Stomp at the end of the optimization process
   *
   * @param success           Whether the optimization succeeded
   * @param total_iterations  Number of iterations used
   * @param final_cost        The cost value after optimizing.
   * @param parameters        The parameters generated at the end of current iteration[num_dimensions x num_timesteps]
   */
  virtual void done(bool success,int total_iterations,double final_cost,const Eigen::MatrixXd& parameters) override;

protected:

  // robot environment
  std::string group_name_;
  moveit::core::RobotModelConstPtr robot_model_ptr_;
  planning_scene::PlanningSceneConstPtr planning_scene_ptr_;

  /**< The plugin loaders for each type of plugin supported>*/
  CostFuctionLoaderPtr cost_function_loader_;
  NoisyFilterLoaderPtr noisy_filter_loader_;
  UpdateFilterLoaderPtr update_filter_loader_;
  NoiseGeneratorLoaderPtr noise_generator_loader_;

  /**< Arrays containing the loaded plugins >*/
  std::vector<cost_functions::StompCostFunctionPtr> cost_functions_;
  std::vector<noisy_filters::StompNoisyFilterPtr> noisy_filters_;
  std::vector<update_filters::StompUpdateFilterPtr> update_filters_;
  std::vector<noise_generators::StompNoiseGeneratorPtr> noise_generators_;
};


} /* namespace stomp_moveit */

#endif /* MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_STOMP_OPTIMIZATION_TASK_H_ */
