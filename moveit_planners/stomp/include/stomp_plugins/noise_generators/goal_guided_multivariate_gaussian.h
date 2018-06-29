/**
 * @file goal_guided_multivariate_gaussian.h
 * @brief This class generates noisy trajectories to an under-constrained cartesian goal pose
 *
 * @author Jorge Nicho
 * @date Jun 14, 2016
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

#ifndef STOMP_PLUGINS_INCLUDE_STOMP_PLUGINS_NOISE_GENERATORS_GOAL_GUIDED_MULTIVARIATE_GAUSSIAN_H_
#define STOMP_PLUGINS_INCLUDE_STOMP_PLUGINS_NOISE_GENERATORS_GOAL_GUIDED_MULTIVARIATE_GAUSSIAN_H_

#include <stomp_moveit/noise_generators/stomp_noise_generator.h>
#include <stomp_moveit/utils/multivariate_gaussian.h>
#include "stomp_moveit/utils/kinematics.h"


namespace stomp_moveit
{
namespace noise_generators
{

typedef boost::mt19937 RGNType;
typedef boost::variate_generator< RGNType, boost::uniform_real<> > RandomGenerator;

/**
 * @class stomp_moveit::noise_generators::GoalGuidedMultivariateGaussian
 * @brief This class generates noisy trajectories to an under-constrained cartesian goal pose.
 *
 * @par Examples:
 * All examples are located here @ref stomp_plugins_examples
 *
 */
class GoalGuidedMultivariateGaussian: public StompNoiseGenerator
{
public:
  GoalGuidedMultivariateGaussian();
  virtual ~GoalGuidedMultivariateGaussian();

  /**
   * @brief Initializes and configures.
   * @param robot_model_ptr A pointer to the robot model.
   * @param group_name      The designated planning group.
   * @param config          The configuration data.  Usually loaded from the ros parameter server
   * @return true if succeeded, false otherwise.
   */
  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) override;

  /**
   * @brief Sets internal members of the plugin from the configuration data.
   * @param config  The configuration data.  Usually loaded from the ros parameter server
   * @return  true if succeeded, false otherwise.
   */
  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

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
                   moveit_msgs::MoveItErrorCodes& error_code) override;

  /**
   * @brief Generates a noisy trajectory from the parameters.
   * @param parameters        The current value of the optimized parameters [num_dimensions x num_parameters]
   * @param start_timestep    Start index into the 'parameters' array, usually 0.
   * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    The index of the noisy trajectory.
   * @param parameters_noise  The parameters + noise
   * @param noise             The noise applied to the parameters
   * @return true if cost were properly computed, false otherwise.
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
   * @param parameters        The parameters generated at the end of current iteration[num_dimensions x num_timesteps]
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

  virtual bool setNoiseGeneration(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code);

  virtual bool setGoalConstraints(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code);

  virtual bool generateRandomGoal(const Eigen::VectorXd& seed,Eigen::VectorXd& goal_joint_pose);

protected:

  // names
  std::string name_;
  std::string group_;

  // goal tool constraints
  std::string tool_link_;

  // ros parameters
  utils::kinematics::KinematicConfig kc_;                             /**< @brief The kinematic configuration to find valid goal poses **/

  // noisy trajectory generation
  std::vector<utils::MultivariateGaussianPtr> traj_noise_generators_; /**< @brief Randomized numerical distribution generators, [6 x 1] **/
  Eigen::VectorXd raw_noise_;                                         /**< @brief The noise vector **/
  std::vector<double> stddev_;                                        /**< @brief The standard deviations applied to each joint, [num_dimensions x 1 **/
  std::vector<double> goal_stddev_;                                   /**< @brief The standard deviations applied to each cartesian dimension at the goal, [6 x 1] **/

  // random goal generation
  boost::shared_ptr<RandomGenerator> goal_rand_generator_;            /**< @brief Random generator for the tool goal pose **/

  // robot
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr state_;

};

} /* namespace noise_generators */
} /* namespace stomp_moveit */

#endif /* STOMP_PLUGINS_INCLUDE_STOMP_PLUGINS_NOISE_GENERATORS_GOAL_GUIDED_MULTIVARIATE_GAUSSIAN_H_ */
