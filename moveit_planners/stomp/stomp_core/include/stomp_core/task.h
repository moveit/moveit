/**
 * @file task.h
 * @brief This defines the stomp task
 *
 * @author Jorge Nicho
 * @date March 7, 2016
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
#ifndef STOMP_TASK_H_
#define STOMP_TASK_H_

#include <XmlRpcValue.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include "stomp_core/utils.h"

namespace stomp_core
{

class Task;
typedef boost::shared_ptr<Task> TaskPtr;

class Task
{

public:

    Task(){}

    virtual ~Task(){};

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
                                         Eigen::MatrixXd& noise) = 0;

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
                         bool& validity) = 0 ;

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
                         bool& validity) = 0 ;

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
                                       bool& filtered)
    {
      filtered = false;
      return true;
    };

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
                                  Eigen::MatrixXd& updates)
    {
      return true;
    };


    /**
     * @brief Called by Stomp at the end of the optimization process
     *
     * @param success           Whether the optimization succeeded
     * @param total_iterations  Number of iterations used
     * @param final_cost        The cost value after optimizing.
     */
    virtual void done(bool success,int total_iterations,double final_cost){}

};

}
#endif /* TASK_H_ */
