/**
 * @file stomp_3dof.cpp
 * @brief This contains gtest code for stomp
 *
 * @author Jorge Nicho
 * @date March 25, 2016
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
#include <iostream>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "stomp_core/stomp.h"
#include "stomp_core/task.h"

using Trajectory = Eigen::MatrixXd; /**< Assign Type Trajectory to Eigen::MatrixXd Type */

const std::size_t NUM_DIMENSIONS = 3;                      /**< Number of parameters to optimize */
const std::size_t NUM_TIMESTEPS = 20;                      /**< Number of timesteps */
const double DELTA_T = 0.1;                                /**< Timestep in seconds */
const std::vector<double> START_POS = { 1.4, 1.4, 0.5 };   /**< Trajectory starting position */
const std::vector<double> END_POS = { -1.25, 1.0, -0.26 }; /**< Trajectory ending posiiton */
const std::vector<double> BIAS_THRESHOLD = { 0.050, 0.050,
                                             0.050 };  /**< Threshold to determine whether two trajectories are equal */
const std::vector<double> STD_DEV = { 1.0, 1.0, 1.0 }; /**< Standard deviation used for generating noisy parameters */

using namespace stomp_core;

/** @brief A dummy task for testing STOMP */
class DummyTask : public Task
{
public:
  /**
   * @brief A dummy task for testing Stomp
   * @param parameters_bias default parameter bias used for computing cost for the test
   * @param bias_thresholds threshold to determine whether two trajectories are equal
   * @param std_dev standard deviation used for generating noisy parameters
   */
  DummyTask(const Trajectory& parameters_bias, const std::vector<double>& bias_thresholds,
            const std::vector<double>& std_dev)
    : parameters_bias_(parameters_bias), bias_thresholds_(bias_thresholds), std_dev_(std_dev)
  {
    // generate smoothing matrix
    int num_timesteps = parameters_bias.cols();
    generateSmoothingMatrix(num_timesteps, 1.0, smoothing_M_);
    srand(1);
  }

  /** @brief See base clase for documentation */
  bool generateNoisyParameters(const Eigen::MatrixXd& parameters, std::size_t start_timestep, std::size_t num_timesteps,
                               int iteration_number, int rollout_number, Eigen::MatrixXd& parameters_noise,
                               Eigen::MatrixXd& noise) override
  {
    double rand_noise;
    for (std::size_t d = 0; d < parameters.rows(); d++)
    {
      for (std::size_t t = 0; t < parameters.cols(); t++)
      {
        rand_noise = static_cast<double>(rand() % RAND_MAX) / static_cast<double>(RAND_MAX - 1);  // 0 to 1
        rand_noise = 2 * (0.5 - rand_noise);
        noise(d, t) = rand_noise * std_dev_[d];
      }
    }

    parameters_noise = parameters + noise;

    return true;
  }

  bool computeCosts(const Trajectory& parameters, std::size_t start_timestep, std::size_t num_timesteps,
                    int iteration_number, Eigen::VectorXd& costs, bool& validity) override
  {
    return computeNoisyCosts(parameters, start_timestep, num_timesteps, iteration_number, -1, costs, validity);
  }

  bool computeNoisyCosts(const Trajectory& parameters, std::size_t start_timestep, std::size_t num_timesteps,
                         int iteration_number, int rollout_number, Eigen::VectorXd& costs, bool& validity) override
  {
    costs.setZero(num_timesteps);
    double diff;
    double cost = 0.0;
    validity = true;

    for (std::size_t t = 0u; t < num_timesteps; t++)
    {
      cost = 0;
      for (std::size_t d = 0u; d < parameters.rows(); d++)
      {
        diff = std::abs(parameters(d, t) - parameters_bias_(d, t));
        if (diff > std::abs(bias_thresholds_[d]))
        {
          cost += diff;
          validity = false;
        }
      }

      costs(t) = cost;
    }

    return true;
  }

  bool filterParameterUpdates(std::size_t start_timestep, std::size_t num_timesteps, int iteration_number,
                              const Eigen::MatrixXd& parameters, Eigen::MatrixXd& updates) override
  {
    return smoothParameterUpdates(start_timestep, num_timesteps, iteration_number, updates);
  }

protected:
  /**
   * @brief Perform a smooth update given a noisy update
   * @param start_timestep starting timestep
   * @param num_timesteps number of timesteps
   * @param iteration_number number of interations allowed
   * @param updates returned smooth update
   * @return True if successful, otherwise false
   */
  bool smoothParameterUpdates(std::size_t start_timestep, std::size_t num_timesteps, int iteration_number,
                              Eigen::MatrixXd& updates)
  {
    for (auto d = 0u; d < updates.rows(); d++)
    {
      updates.row(d).transpose() = smoothing_M_ * (updates.row(d).transpose());
    }

    return true;
  }

protected:
  Trajectory parameters_bias_;          /**< Parameter bias used for computing cost for the test */
  std::vector<double> bias_thresholds_; /**< Threshold to determine whether two trajectories are equal */
  std::vector<double> std_dev_;         /**< Standard deviation used for generating noisy parameters */
  Eigen::MatrixXd smoothing_M_;         /**< Matrix used for smoothing the trajectory */
};

/**
 * @brief Compares whether two trajectories are close to each other within a threshold.
 * @param optimized optimized trajectory
 * @param desired desired trajectory
 * @param thresholds used to determine if two values are equal
 * @return True if the difference between the two is less than the threshold, otherwise false
 */
bool compareDiff(const Trajectory& optimized, const Trajectory& desired, const std::vector<double>& thresholds)
{
  auto num_dimensions = optimized.rows();
  Trajectory diff = Trajectory::Zero(num_dimensions, optimized.cols());
  for (auto d = 0u; d < num_dimensions; d++)
  {
    diff.row(d) = optimized.row(d) - desired.row(d);
    diff.row(d).cwiseAbs();
    if ((diff.row(d).array() > thresholds[d]).any())
    {
      return false;
    }
  }

  return true;
}

/**
 * @brief Create the STOMP configuration for the 3 DOF problem.
 * @return  StompConfiguration
 */
StompConfiguration create3DOFConfiguration()
{
  StompConfiguration c;
  c.num_timesteps = NUM_TIMESTEPS;
  c.num_iterations = 40;
  c.num_dimensions = NUM_DIMENSIONS;
  c.delta_t = DELTA_T;
  c.control_cost_weight = 0.0;
  c.initialization_method = TrajectoryInitializations::LINEAR_INTERPOLATION;
  c.num_iterations_after_valid = 0;
  c.num_rollouts = 20;
  c.max_rollouts = 20;

  return c;
}

/**
 * @brief Compute a linear interpolated trajectory given a start and end state
 * @param start start position
 * @param end last position
 * @param num_timesteps number of timesteps
 * @param traj returned linear interpolated trajectory
 */
void interpolate(const std::vector<double>& start, const std::vector<double>& end, std::size_t num_timesteps,
                 Trajectory& traj)
{
  auto dimensions = start.size();
  traj = Eigen::MatrixXd::Zero(dimensions, num_timesteps);
  for (auto d = 0u; d < dimensions; d++)
  {
    double delta = (end[d] - start[d]) / (num_timesteps - 1);
    for (auto t = 0u; t < num_timesteps; t++)
    {
      traj(d, t) = start[d] + t * delta;
    }
  }
}

/** @brief This tests the Stomp constructor */
TEST(Stomp3DOF, construction)
{
  Trajectory trajectory_bias;
  interpolate(START_POS, END_POS, NUM_TIMESTEPS, trajectory_bias);
  TaskPtr task(new DummyTask(trajectory_bias, BIAS_THRESHOLD, STD_DEV));

  Stomp stomp(create3DOFConfiguration(), task);
}

/** @brief This tests the Stomp solve method for a default case */
TEST(Stomp3DOF, solve_default)
{
  Trajectory trajectory_bias;
  interpolate(START_POS, END_POS, NUM_TIMESTEPS, trajectory_bias);
  TaskPtr task(new DummyTask(trajectory_bias, BIAS_THRESHOLD, STD_DEV));

  StompConfiguration config = create3DOFConfiguration();
  Stomp stomp(config, task);

  Trajectory optimized;
  stomp.solve(START_POS, END_POS, optimized);

  EXPECT_EQ(optimized.rows(), NUM_DIMENSIONS);
  EXPECT_EQ(optimized.cols(), NUM_TIMESTEPS);
  EXPECT_TRUE(compareDiff(optimized, trajectory_bias, BIAS_THRESHOLD));

  // calculate difference
  Trajectory diff;
  diff = trajectory_bias - optimized;

  std::string line_separator = "\n------------------------------------------------------\n";
  std::cout << line_separator;
  std::cout << stomp_core::toString(trajectory_bias);
  std::cout << line_separator;
  std::cout << toString(optimized) << "\n";
  std::cout << "Differences"
            << "\n"
            << toString(diff) << line_separator;
}

/** @brief This tests the Stomp solve method using a linear interpolated inital trajectory */
TEST(Stomp3DOF, solve_interpolated_initial)
{
  Trajectory trajectory_bias;
  interpolate(START_POS, END_POS, NUM_TIMESTEPS, trajectory_bias);
  TaskPtr task(new DummyTask(trajectory_bias, BIAS_THRESHOLD, STD_DEV));

  StompConfiguration config = create3DOFConfiguration();
  config.initialization_method = TrajectoryInitializations::LINEAR_INTERPOLATION;
  Stomp stomp(config, task);

  Trajectory optimized;
  stomp.solve(trajectory_bias, optimized);

  EXPECT_EQ(optimized.rows(), NUM_DIMENSIONS);
  EXPECT_EQ(optimized.cols(), NUM_TIMESTEPS);
  EXPECT_TRUE(compareDiff(optimized, trajectory_bias, BIAS_THRESHOLD));

  // calculate difference
  Trajectory diff;
  diff = trajectory_bias - optimized;

  std::string line_separator = "\n------------------------------------------------------\n";
  std::cout << line_separator;
  std::cout << stomp_core::toString(trajectory_bias);
  std::cout << line_separator;
  std::cout << toString(optimized) << "\n";
  std::cout << "Differences"
            << "\n"
            << toString(diff) << line_separator;
}

/** @brief This tests the Stomp solve method using a cubic polynomial interpolated inital trajectory */
TEST(Stomp3DOF, solve_cubic_polynomial_initial)
{
  Trajectory trajectory_bias;
  interpolate(START_POS, END_POS, NUM_TIMESTEPS, trajectory_bias);
  TaskPtr task(new DummyTask(trajectory_bias, BIAS_THRESHOLD, STD_DEV));

  StompConfiguration config = create3DOFConfiguration();
  config.initialization_method = TrajectoryInitializations::CUBIC_POLYNOMIAL_INTERPOLATION;
  Stomp stomp(config, task);

  Trajectory optimized;
  stomp.solve(trajectory_bias, optimized);

  EXPECT_EQ(optimized.rows(), NUM_DIMENSIONS);
  EXPECT_EQ(optimized.cols(), NUM_TIMESTEPS);
  EXPECT_TRUE(compareDiff(optimized, trajectory_bias, BIAS_THRESHOLD));

  // calculate difference
  Trajectory diff;
  diff = trajectory_bias - optimized;

  std::string line_separator = "\n------------------------------------------------------\n";
  std::cout << line_separator;
  std::cout << stomp_core::toString(trajectory_bias);
  std::cout << line_separator;
  std::cout << toString(optimized) << "\n";
  std::cout << "Differences"
            << "\n"
            << toString(diff) << line_separator;
}

/** @brief This tests the Stomp solve method using a minimum cost inital trajectory */
TEST(Stomp3DOF, solve_min_control_cost_initial)
{
  Trajectory trajectory_bias;
  interpolate(START_POS, END_POS, NUM_TIMESTEPS, trajectory_bias);
  TaskPtr task(new DummyTask(trajectory_bias, BIAS_THRESHOLD, STD_DEV));

  StompConfiguration config = create3DOFConfiguration();
  config.initialization_method = TrajectoryInitializations::MININUM_CONTROL_COST;
  Stomp stomp(config, task);

  Trajectory optimized;
  stomp.solve(trajectory_bias, optimized);

  EXPECT_EQ(optimized.rows(), NUM_DIMENSIONS);
  EXPECT_EQ(optimized.cols(), NUM_TIMESTEPS);
  EXPECT_TRUE(compareDiff(optimized, trajectory_bias, BIAS_THRESHOLD));

  // calculate difference
  Trajectory diff;
  diff = trajectory_bias - optimized;

  std::string line_separator = "\n------------------------------------------------------\n";
  std::cout << line_separator;
  std::cout << stomp_core::toString(trajectory_bias);
  std::cout << line_separator;
  std::cout << toString(optimized) << "\n";
  std::cout << "Differences"
            << "\n"
            << toString(diff) << line_separator;
}

/** @brief This tests Stomp solve method given 40 timesteps */
TEST(Stomp3DOF, solve_40_timesteps)
{
  Trajectory trajectory_bias;
  int num_timesteps = 40;
  interpolate(START_POS, END_POS, num_timesteps, trajectory_bias);
  TaskPtr task(new DummyTask(trajectory_bias, BIAS_THRESHOLD, STD_DEV));

  StompConfiguration config = create3DOFConfiguration();
  config.initialization_method = TrajectoryInitializations::LINEAR_INTERPOLATION;
  config.num_iterations = 100;
  config.num_timesteps = num_timesteps;
  Stomp stomp(config, task);

  Trajectory optimized;
  stomp.solve(START_POS, END_POS, optimized);

  EXPECT_EQ(optimized.rows(), NUM_DIMENSIONS);
  EXPECT_EQ(optimized.cols(), num_timesteps);
  EXPECT_TRUE(compareDiff(optimized, trajectory_bias, BIAS_THRESHOLD));

  // calculate difference
  Trajectory diff;
  diff = trajectory_bias - optimized;

  std::string line_separator = "\n------------------------------------------------------\n";
  std::cout << line_separator;
  std::cout << stomp_core::toString(trajectory_bias);
  std::cout << line_separator;
  std::cout << toString(optimized) << "\n";
  std::cout << "Differences"
            << "\n"
            << toString(diff) << line_separator;
}

/** @brief This tests Stomp solve method given 60 timesteps */
TEST(Stomp3DOF, solve_60_timesteps)
{
  Trajectory trajectory_bias;
  int num_timesteps = 60;
  interpolate(START_POS, END_POS, num_timesteps, trajectory_bias);
  TaskPtr task(new DummyTask(trajectory_bias, BIAS_THRESHOLD, STD_DEV));

  StompConfiguration config = create3DOFConfiguration();
  // config.initialization_method = TrajectoryInitializations::MININUM_CONTROL_COST; // this initialization method is
  // currently behaving oddly
  config.num_iterations = 100;
  config.num_timesteps = num_timesteps;
  Stomp stomp(config, task);

  Trajectory optimized;
  stomp.solve(START_POS, END_POS, optimized);

  EXPECT_EQ(optimized.rows(), NUM_DIMENSIONS);
  EXPECT_EQ(optimized.cols(), num_timesteps);
  EXPECT_TRUE(compareDiff(optimized, trajectory_bias, BIAS_THRESHOLD));

  // calculate difference
  Trajectory diff;
  diff = trajectory_bias - optimized;

  std::string line_separator = "\n------------------------------------------------------\n";
  std::cout << line_separator;
  std::cout << stomp_core::toString(trajectory_bias);
  std::cout << line_separator;
  std::cout << toString(optimized) << "\n";
  std::cout << "Differences"
            << "\n"
            << toString(diff) << line_separator;
}
