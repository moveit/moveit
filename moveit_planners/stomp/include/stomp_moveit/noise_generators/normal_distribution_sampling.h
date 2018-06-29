/**
 * @file normal_distribution_sampling.h
 * @brief This a normial distribution noisy trajectory update generator.
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
#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISE_GENERATORS_NORMAL_DISTRIBUTION_SAMPLING_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISE_GENERATORS_NORMAL_DISTRIBUTION_SAMPLING_H_

#include <stomp_moveit/noise_generators/stomp_noise_generator.h>
#include <stomp_moveit/utils/multivariate_gaussian.h>

namespace stomp_moveit
{

namespace noise_generators
{

/**
 * @class stomp_moveit::noise_generators::NormalDistributionSampling
 * @brief Uses a normal distribution to apply noise onto the trajectory.
 *
 * @par Examples:
 * All examples are located here @ref stomp_moveit_examples
 */
class NormalDistributionSampling: public StompNoiseGenerator
{
public:
  NormalDistributionSampling();
  virtual ~NormalDistributionSampling();

  /** @brief see base class for documentation*/
  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) override;

  /** @brief see base class for documentation*/
  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

  /** @brief see base class for documentation*/
  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code) override;

  /**
   * @brief Generates a noisy trajectory from the parameters.
   * @param parameters        The current value of the optimized parameters to add noise to [num_dimensions x num_parameters]
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    index of the noisy trajectory.
   * @param parameters_noise  the parameters + noise
   * @param noise             the noise applied to the parameters
   * @return true if cost were properly computed
   */
  virtual bool generateNoise(const Eigen::MatrixXd& parameters,
                                       std::size_t start_timestep,
                                       std::size_t num_timesteps,
                                       int iteration_number,
                                       int rollout_number,
                                       Eigen::MatrixXd& parameters_noise,
                                       Eigen::MatrixXd& noise) override;

  /**
   * @brief Called by the Stomp at the end of the optimization process
   *
   * @param success           Whether the optimization succeeded
   * @param total_iterations  Number of iterations used
   * @param final_cost        The cost value after optimizing.
   * @param parameters        The parameters generated at the end of current iteration [num_dimensions x num_timesteps]
   */
  virtual void done(bool success,int total_iterations,double final_cost,const Eigen::MatrixXd& parameters){}


  virtual std::string getName() const
  {
    return name_ + "/" + group_;
  }


  virtual std::string getGroupName() const
  {
    return group_;
  }

protected:

  // names
  std::string name_;
  std::string group_;

  // random noise generation
  std::vector<utils::MultivariateGaussianPtr> rand_generators_;
  Eigen::VectorXd raw_noise_;
  std::vector<double> stddev_;

};

} /* namespace noise_generators */
} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISE_GENERATORS_NORMAL_DISTRIBUTION_SAMPLING_H_ */
