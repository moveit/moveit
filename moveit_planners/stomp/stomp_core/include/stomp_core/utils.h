/**
 * @file utils.h
 * @brief This is a utility class for stomp
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
#ifndef INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_UTILS_H_
#define INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_UTILS_H_

#include <string>
#include <vector>
#include <Eigen/Core>

namespace stomp_core
{

struct Rollout
{
  Eigen::MatrixXd noise;                       /**< [num_dimensions] x num_time_steps, random noise applied to the parameters*/
  Eigen::MatrixXd parameters_noise;            /**< [num_dimensions] x num_time_steps, the sum of parameters + noise */

  Eigen::VectorXd state_costs; /**< num_time_steps */
  Eigen::MatrixXd control_costs; /**< [num_dimensions] x num_time_steps */
  Eigen::MatrixXd total_costs; /**< [num_dimensions] x num_time_steps total_cost[d] = state_costs_ + control_costs_[d]*/
  Eigen::MatrixXd probabilities; /**< [num_dimensions] x num_time_steps */

  std::vector<double> full_probabilities; /**< [num_dimensions] probabilities of full trajectory */
  std::vector<double> full_costs; /**< [num_dimensions] state_cost + control_cost for each joint over the entire trajectory
                                       full_costs_[d] = state_cost.sum() + control_cost[d].sum() */

  double importance_weight; /**< importance sampling weight */
  double total_cost; /**< combined state + control cost over the entire trajectory for all joints */

};

namespace DerivativeOrders
{
  enum DerivativeOrder
  {
    STOMP_POSITION = 0, STOMP_VELOCITY = 1, STOMP_ACCELERATION = 2, STOMP_JERK = 3
  };
}
;

namespace TrajectoryInitializations
{
enum TrajectoryInitialization
{
  LINEAR_INTERPOLATION = 1,
  CUBIC_POLYNOMIAL_INTERPOLATION,
  MININUM_CONTROL_COST
};
}

struct StompConfiguration
{
  // General settings
  int num_iterations;
  int num_iterations_after_valid;   /**< Stomp will stop optimizing this many iterations after finding a valid solution */
  int num_timesteps;
  int num_dimensions;               /** parameter dimensionality */
  double delta_t;               /** time change between consecutive points */
  int initialization_method; /** TrajectoryInitializations::TrajectoryInitialization */

  // Noisy trajectory generation
  int num_rollouts; /**< Number of noisy trajectories*/
  int max_rollouts; /**< The combined number of new and old rollouts during each iteration shouldn't exceed this value */

  // Cost calculation
  double control_cost_weight;  /**< Percentage of the trajectory accelerations cost to be applied in the total cost calculation >*/
};

static const int FINITE_DIFF_RULE_LENGTH = 7;
static const double FINITE_DIFF_COEFFS[FINITE_DIFF_RULE_LENGTH][FINITE_DIFF_RULE_LENGTH] = { {0, 0, 0, 1, 0, 0, 0}, // position
    {0, 0, -1, 1, 0, 0, 0}, // velocity (backward difference)
    {0, -1 / 12.0, 16 / 12.0, -30 / 12.0, 16 / 12.0, -1 / 12.0, 0}, // acceleration (five point stencil)
    {0, 1 / 12.0, -17 / 12.0, 46 / 12.0, -46 / 12.0, 17 / 12.0, -1 / 12.0} // jerk
};

void generateFiniteDifferenceMatrix(int num_time_steps, DerivativeOrders::DerivativeOrder order, double dt,
                                    Eigen::MatrixXd& diff_matrix);

void differentiate(const Eigen::VectorXd& parameters, DerivativeOrders::DerivativeOrder order,
                          double dt, Eigen::VectorXd& derivatives );

void generateSmoothingMatrix(int num_time_steps,double dt, Eigen::MatrixXd& projection_matrix_M);

void toVector(const Eigen::MatrixXd& m,std::vector<Eigen::VectorXd>& v);
std::string toString(const std::vector<Eigen::VectorXd>& data);
std::string toString(const Eigen::VectorXd& data);
std::string toString(const Eigen::MatrixXd& data);



} /* namespace stomp */

#endif /* INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_UTILS_H_ */
