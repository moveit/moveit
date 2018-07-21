/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Mrinal Kalakrishnan */

#ifndef CHOMP_PARAMETERS_H_
#define CHOMP_PARAMETERS_H_

#include <ros/ros.h>

namespace chomp
{
class ChompParameters
{
public:
  ChompParameters();
  virtual ~ChompParameters();

  double getPlanningTimeLimit() const;
  void setPlanningTimeLimit(double planning_time_limit);
  int getMaxIterations() const;
  int getMaxIterationsAfterCollisionFree() const;
  double getSmoothnessCostWeight() const;
  double getObstacleCostWeight() const;

  double getLearningRate() const;
  double getSmoothnessCostVelocity() const;
  double getSmoothnessCostAcceleration() const;
  double getSmoothnessCostJerk() const;
  // bool getAddRandomness() const;
  // bool getUseHamiltonianMonteCarlo() const;
  // double getHmcDiscretization() const;
  // double getHmcStochasticity() const;
  // double getHmcAnnealingFactor() const;
  double getRidgeFactor() const;
  bool getUsePseudoInverse() const;
  double getPseudoInverseRidgeFactor() const;

  double getJointUpdateLimit() const;
  double getMinClearence() const;
  double getCollisionThreshold() const;
  bool getFilterMode() const;
  void setFilterMode(bool mode);
  // double getRandomJumpAmount() const;
  // void setRandomJumpAmount(double amount);
  bool getUseStochasticDescent() const;
  std::string getTrajectoryInitializationMethod() const;
  bool getEnableFailureRecovery() const;
  int getMaxRecoveryAttempts() const;

  /**
   * function which returns the non constant version of the ChompParameters
   * @param params equates the returned chomp parameters to this argument
   * @return returns the non constant version of ChompParameters
   */
  ChompParameters getNonConstantParams(ChompParameters params);

public:
  double planning_time_limit_;  /// maximum time the optimizer can take to find a solution before terminating
  int max_iterations_;          /// maximum number of iterations that the planner can take to find a good solution while
                                /// optimization
  int max_iterations_after_collision_free_;  /// maximum iterations to be performed after a collision free path is
                                             /// found.
  double smoothness_cost_weight_;  /// smoothness_cost_weight parameters controls its weight in the final cost that
                                   /// CHOMP is actually optimizing over
  double obstacle_cost_weight_;  /// controls the weight to be given to obstacles towards the final cost CHOMP optimizes
                                 /// over
  double learning_rate_;  /// learning rate used by the optimizer to find the local / global minima while reducing the
                          /// total cost

  double smoothness_cost_velocity_;      /// variables associated with the cost in velocity
  double smoothness_cost_acceleration_;  /// variables associated with the cost in acceleration
  double smoothness_cost_jerk_;          /// variables associated with the cost in jerk
  // bool add_randomness_;
  // bool use_hamiltonian_monte_carlo_;
  bool use_stochastic_descent_;  /// set this to true/false if you want to use stochastic descent while optimizing the
                                 /// cost.

  // double hmc_stochasticity_;
  // double hmc_discretization_;
  // double hmc_annealing_factor_;
  double ridge_factor_;  /// the noise added to the diagnal of the total quadratic cost matrix in the objective function
  bool use_pseudo_inverse_;             /// enable pseudo inverse calculations or not.
  double pseudo_inverse_ridge_factor_;  /// set the ridge factor if pseudo inverse is enabled

  double joint_update_limit_;   /// set the update limit for the robot joints
  double min_clearence_;        /// the minimum distance that needs to be maintained to avoid obstacles
  double collision_threshold_;  /// the collision threshold cost that needs to be mainted to avoid collisions
  bool filter_mode_;
  // double random_jump_amount_;
  std::string trajectory_initialization_method_;  /// trajectory initialization method to be specified
  bool enable_failure_recovery_;  /// if set to true, CHOMP tries to vary certain parameters to try and find a path if
                                  /// an initial path is not found with the specified chomp parameters
  int max_recovery_attempts_;     /// this the maximum recovery attempts to find a collision free path after an initial
                                  /// failure to find a solution
};

}  // namespace chomp

#endif /* CHOMP_PARAMETERS_H_ */
