/**
 * @file stomp_noise_generator.h
 * @brief This is the base class for all stomp noisy generators.
 *
 * @author Jorge Nicho
 * @date May 31, 2016
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
#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISE_GENERATORS_STOMP_NOISE_GENERATOR_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISE_GENERATORS_STOMP_NOISE_GENERATOR_H_

#include <Eigen/Core>
#include <XmlRpc.h>
#include <stomp_core/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/MotionPlanRequest.h>

namespace stomp_moveit
{

namespace noise_generators
{

class StompNoiseGenerator;
typedef std::shared_ptr<StompNoiseGenerator> StompNoiseGeneratorPtr;

/**
 * @class stomp_moveit::StompNoiseGenerator
 * @brief Interface class for plugins that apply random noise to the trajectory in order to explore the workspace.
 *
 * @par Examples:
 * All examples are located here @ref stomp_moveit_examples
 */
class StompNoiseGenerator
{
public:
  StompNoiseGenerator(){}
  virtual ~StompNoiseGenerator(){}

  /**
   * @brief Initializes and configures.
   * @param robot_model_ptr A pointer to the robot model.
   * @param group_name      The designated planning group.
   * @param config          The configuration data.  Usually loaded from the ros parameter server
   * @return true if succeeded, false otherwise.
   */
  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) = 0;

  /**
   * @brief Sets internal members of the plugin from the configuration data.
   * @param config  The configuration data.  Usually loaded from the ros parameter server
   * @return  true if succeeded, false otherwise.
   */
  virtual bool configure(const XmlRpc::XmlRpcValue& config) = 0;

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
                   moveit_msgs::MoveItErrorCodes& error_code) = 0;

  /**
   * @brief Generates a noisy trajectory from the parameters.
   * @param parameters        The current value of the optimized parameters to add noise to [num_dimensions x num_parameters]
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    index of the noisy trajectory.
   * @param parameters_noise  the parameters + noise
   * @param noise             the noise applied to the parameters
   * @return false if there was an irrecoverable failure, true otherwise.
   */
  virtual bool generateNoise(const Eigen::MatrixXd& parameters,
                                       std::size_t start_timestep,
                                       std::size_t num_timesteps,
                                       int iteration_number,
                                       int rollout_number,
                                       Eigen::MatrixXd& parameters_noise,
                                       Eigen::MatrixXd& noise) = 0;

  /**
   * @brief Called by STOMP at the end of each iteration.
   * @param start_timestep    The start index into the 'parameters' array, usually 0.
   * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param cost              The cost value for the current parameters.
   * @param parameters        The value of the parameters at the end of the current iteration [num_dimensions x num_timesteps].
   */
  virtual void postIteration(std::size_t start_timestep,
                                std::size_t num_timesteps,int iteration_number,double cost,const Eigen::MatrixXd& parameters){}

  /**
   * @brief Called by the STOMP instance at the end of the optimization process
   *
   * @param success           Whether the optimization succeeded
   * @param total_iterations  Number of iterations used
   * @param final_cost        The cost value after optimizing.
   * @param parameters        The parameters generated at the end of current iteration [num_dimensions x num_timesteps]
   */
  virtual void done(bool success,int total_iterations,double final_cost,const Eigen::MatrixXd& parameters){}


  virtual std::string getName() const
  {
    return "Not implemented";
  }


  virtual std::string getGroupName() const
  {
    return "Not implemented";
  }
};

} /* namespace noise_generators */
} /* namespace stomp_moveit */



#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISE_GENERATORS_STOMP_NOISE_GENERATOR_H_ */
