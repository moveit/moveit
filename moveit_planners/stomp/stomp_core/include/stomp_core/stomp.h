/**
 * @file stomp.h
 * @brief This contains the stomp core algorithm
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
#ifndef INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_H_
#define INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_H_

#include <atomic>
#include <stomp_core/utils.h>
#include <XmlRpc.h>
#include "stomp_core/task.h"

namespace stomp_core
{


class Stomp
{
public:
  Stomp(const StompConfiguration& config,TaskPtr task);
  virtual ~Stomp();

  bool solve(const std::vector<double>& first,const std::vector<double>& last,
             Eigen::MatrixXd& parameters_optimized);
  bool solve(const Eigen::MatrixXd& initial_parameters,
             Eigen::MatrixXd& parameters_optimized);
  bool cancel();

  bool clear();

  static bool parseConfig(XmlRpc::XmlRpcValue config,StompConfiguration& stomp_config);

protected:

  // initialization methods
  bool resetVariables();
  bool computeInitialTrajectory(const std::vector<double>& first,const std::vector<double>& last);

  // optimization methods
  bool runSingleIteration();
  bool generateNoisyRollouts();
  bool filterNoisyRollouts();
  bool computeNoisyRolloutsCosts();
  bool computeRolloutsStateCosts();
  bool computeRolloutsControlCosts();
  bool computeProbabilities();
  bool updateParameters();
  bool computeOptimizedCost();

protected:

  // process control
  std::atomic<bool> proceed_;
  TaskPtr task_;
  StompConfiguration config_;
  unsigned int current_iteration_;

  // optimized parameters
  bool parameters_valid_;         /**< whether or not the optimized parameters are valid */
  double parameters_total_cost_;  /**< Total cost of the optimized parameters */
  double current_lowest_cost_;
  Eigen::MatrixXd parameters_optimized_;               /**< [Dimensions][timesteps]*/
  Eigen::MatrixXd parameters_updates_;                 /**< [Dimensions][timesteps]*/
  Eigen::VectorXd parameters_state_costs_;                          /**< [timesteps]*/
  Eigen::MatrixXd parameters_control_costs_;           /**< [Dimensions][timesteps]*/

  // rollouts
  std::vector<Rollout> noisy_rollouts_;
  std::vector<Rollout> reused_rollouts_;     /**< Used for reordering arrays based on cost */
  int num_active_rollouts_;

  // finite difference and optimization matrices
  int num_timesteps_padded_;                        /**< timesteps + 2*(FINITE_DIFF_RULE_LENGTH - 1) */
  int start_index_padded_;                          /** index corresponding to the start of the non-paded section in the padded arrays */
  Eigen::MatrixXd finite_diff_matrix_A_padded_;
  Eigen::MatrixXd control_cost_matrix_R_padded_;
  Eigen::MatrixXd control_cost_matrix_R_;           /**< [timesteps x timesteps], Referred to as 'R = A x A_transpose' in the literature */
  Eigen::MatrixXd inv_control_cost_matrix_R_;       /**< [timesteps x timesteps], R^-1 ' matrix */


};

} /* namespace stomp */

#endif /* INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_H_ */
