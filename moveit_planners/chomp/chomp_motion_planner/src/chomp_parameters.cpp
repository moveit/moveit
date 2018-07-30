/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <chomp_motion_planner/chomp_parameters.h>

namespace chomp
{
ChompParameters::ChompParameters()
{
  planning_time_limit_ = 6.0;
  max_iterations_ = 50;
  max_iterations_after_collision_free_ = 5;
  smoothness_cost_weight_ = 0.1;
  obstacle_cost_weight_ = 1.0;
  learning_rate_ = 0.01;

  // add_randomness_ = false;
  smoothness_cost_velocity_ = 0.0;
  smoothness_cost_acceleration_ = 1.0;
  smoothness_cost_jerk_ = 0.0;
  // hmc_discretization_ = 0.01;
  // hmc_stochasticity_ = 0.01;
  // hmc_annealing_factor_ = 0.99;
  // use_hamiltonian_monte_carlo_ = false;
  ridge_factor_ = 0.0;
  use_pseudo_inverse_ = false;
  pseudo_inverse_ridge_factor_ = 1e-4;

  joint_update_limit_ = 0.1;
  min_clearence_ = 0.2;
  collision_threshold_ = 0.07;
  // random_jump_amount_ = 1.0;
  use_stochastic_descent_ = true;
  filter_mode_ = false;
  trajectory_initialization_method_ = std::string("quintic-spline");
}

ChompParameters::~ChompParameters()
{
}

}  // namespace chomp
