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
#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_STOMP_OPTIMIZATION_TASK_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_STOMP_OPTIMIZATION_TASK_H_

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

class StompOptimizationTask: public stomp_core::Task
{
public:
  StompOptimizationTask(moveit::core::RobotModelConstPtr robot_model_ptr, std::string group_name,
                        const XmlRpc::XmlRpcValue& config);
  virtual ~StompOptimizationTask();

  /**
   * @brief calls the updatePlanningScene method of each cost function and filter class it contains
   * @param planning_scene_ptr  A smart pointer to the planning scene
   * @param req                 The motion planning request
   * @param num_timesteps       The number of time steps in the trajectory
   * @param dt                  Time interval between consecutive points
   * @param error_code          Moveit error code
   * @return                    True if succeeded
   */
  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code);

  /**
   * @brief Generates a noisy trajectory from the parameters.
   * @param parameters        [num_dimensions] x [num_parameters] the current value of the optimized parameters
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    index of the noisy trajectory.
   * @param parameters_noise  the parameters + noise
   * @param noise             the noise applied to the parameters
   * @return true if cost were properly computed
   */
  virtual bool generateNoisyParameters(const Eigen::MatrixXd& parameters,
                                       std::size_t start_timestep,
                                       std::size_t num_timesteps,
                                       int iteration_number,
                                       int rollout_number,
                                       Eigen::MatrixXd& parameters_noise,
                                       Eigen::MatrixXd& noise) override;

  /**
   * @brief computes the state costs as a function of the noisy parameters for each time step.
   * @param parameters [num_dimensions] num_parameters - policy parameters to execute
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number index of the noisy trajectory whose cost is being evaluated.
   * @param costs vector containing the state costs per timestep.
   * @param validity whether or not the trajectory is valid
   * @return true if cost were properly computed
   */
  virtual bool computeNoisyCosts(const Eigen::MatrixXd& parameters,
                       std::size_t start_timestep,
                       std::size_t num_timesteps,
                       int iteration_number,
                       int rollout_number,
                       Eigen::VectorXd& costs,
                       bool& validity) override;

  /**
   * @brief computes the state costs as a function of the optimized parameters for each time step.
   * @param parameters        [num_dimensions] num_parameters - policy parameters to execute
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param costs             vector containing the state costs per timestep.
   * @param validity          whether or not the trajectory is valid
   * @return true if cost were properly computed
   */
  virtual bool computeCosts(const Eigen::MatrixXd& parameters,
                       std::size_t start_timestep,
                       std::size_t num_timesteps,
                       int iteration_number,
                       Eigen::VectorXd& costs,
                       bool& validity) override;

  /**
   * @brief Filters the given noisy parameters which is applied after noisy trajectory generation. It could be used for clipping
   * of joint limits or projecting into the null space of the Jacobian.
   *
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    The rollout index for this noisy parameter set
   * @param parameters        The noisy parameters     *
   * @param filtered          False if no filtering was done
   * @return false if no filtering was done
   */
  virtual bool filterNoisyParameters(std::size_t start_timestep,
                                     std::size_t num_timesteps,
                                     int iteration_number,
                                     int rollout_number,
                                     Eigen::MatrixXd& parameters,
                                     bool& filtered) override;

  /**
   * @brief Filters the given parameters which is applied after the update. It could be used for clipping of joint limits
   * or projecting into the null space of the Jacobian.
   *
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param parameters        The optimized parameters
   * @param updates           The updates to the parameters
   * @return                  False if there was a failure
   */
  virtual bool filterParameterUpdates(std::size_t start_timestep,
                                      std::size_t num_timesteps,
                                      int iteration_number,
                                      const Eigen::MatrixXd& parameters,
                                      Eigen::MatrixXd& updates) override;

  /**
   * @brief Called by Stomp at the end of the optimization process
   *
   * @param success           Whether the optimization succeeded
   * @param total_iterations  Number of iterations used
   * @param final_cost        The cost value after optimizing.
   */
  virtual void done(bool success,int total_iterations,double final_cost) override;

protected:

  bool initializeCostFunctionPlugins(const XmlRpc::XmlRpcValue& config);
  bool initializeNoisyFilterPlugins(const XmlRpc::XmlRpcValue& config);
  bool initializeUpdateFilterPlugins(const XmlRpc::XmlRpcValue& config);
  bool initializeNoiseGeneratorPlugins(const XmlRpc::XmlRpcValue& config);
protected:

  // robot environment
  std::string group_name_;
  moveit::core::RobotModelConstPtr robot_model_ptr_;
  planning_scene::PlanningSceneConstPtr planning_scene_ptr_;

  // plugins
  CostFuctionLoaderPtr cost_function_loader_;
  NoisyFilterLoaderPtr noisy_filter_loader_;
  UpdateFilterLoaderPtr update_filter_loader_;
  NoiseGeneratorLoaderPtr noise_generator_loader_;

  std::vector<cost_functions::StompCostFunctionPtr> cost_functions_;
  std::vector<noisy_filters::StompNoisyFilterPtr> noisy_filters_;
  std::vector<update_filters::StompUpdateFilterPtr> update_filters_;
  std::vector<noise_generators::StompNoiseGeneratorPtr> noise_generators_;
};


} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_STOMP_OPTIMIZATION_TASK_H_ */
