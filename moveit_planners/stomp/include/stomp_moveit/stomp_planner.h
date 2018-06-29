/**
 * @file stomp_planner.h
 * @brief This defines the stomp planner for MoveIt
 *
 * @author Jorge Nicho
 * @date April 4, 2016
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
#ifndef STOMP_MOVEIT_STOMP_PLANNER_H_
#define STOMP_MOVEIT_STOMP_PLANNER_H_

#include <moveit/planning_interface/planning_interface.h>
#include <stomp_core/stomp.h>
#include <stomp_moveit/stomp_optimization_task.h>
#include <boost/thread.hpp>
#include <ros/ros.h>

namespace stomp_moveit
{

using StompOptimizationTaskPtr = std::shared_ptr<StompOptimizationTask>;

/**
 * @brief The PlanningContext specialization that wraps the STOMP algorithm.
 *
 * @par Examples:
 * All examples are located here @ref stomp_moveit_examples
 *
 */
class StompPlanner: public planning_interface::PlanningContext
{
public:
  /**
   * @brief StompPlanner constructor.
   * @param group   The planning group for which this instance will plan.
   * @param config  The parameter containing the configuration data for this planning group, includes plugins specifications.
   * @param model   A pointer to the robot model.
   */
  StompPlanner(const std::string& group,const XmlRpc::XmlRpcValue& config,const moveit::core::RobotModelConstPtr& model);
  virtual ~StompPlanner();

  /**
   * @brief Solve the motion planning problem as defined in the motion request passed before hand.
   * @param res Contains the solved planned path.
   * @return true if succeeded, false otherwise.
   */
  virtual bool solve(planning_interface::MotionPlanResponse &res) override;

  /**
   * @brief Solve the motion planning problem as defined in the motion request passed before hand.
   * @param res Contains the solved planned path.
   * @return true if succeeded, false otherwise.
   */
  virtual bool solve(planning_interface::MotionPlanDetailedResponse &res) override;

  /**
   * @brief Thread-safe method that request early termination, if a solve() function is currently computing plans.
   * @return true if succeeded, false otherwise.
   */
  virtual bool terminate() override;

  /**
   * @brief Clears results from previous plan.
   */
  virtual void clear() override;

  /**
   * @brief Convenience method to load extract the parameters for each supported planning group.
   * @param nh      A ros node handle.
   * @param config  A map containing the configuration data for each planning group found.
   * @param param   The parameter name containing the confuration data for all planning groups.
   * @return  true if succeeded, false otherwise.
   */
  static bool getConfigData(ros::NodeHandle &nh, std::map<std::string, XmlRpc::XmlRpcValue> &config, std::string param = std::string("stomp"));

  /**
   * @brief Checks some conditions to determine whether it is able to plan given for this planning request.
   * @return  true if succeeded, false otherwise.
   */
  bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req)  const;

  /**
   * @brief From a trajectory_msgs::JointTrajectory create a set of trajectory constraints that stomp can use later as
   * a 'seed' for the optimization planning.
   * @param seed The trajectory to encode into 'seed' trajectory constraints
   * @return The encoded trajectory constraints which can be added directly to a moveit planning request. In the case
   * of failure, may throw a std::runtime_error.
   */
  static moveit_msgs::TrajectoryConstraints encodeSeedTrajectory(const trajectory_msgs::JointTrajectory& seed);

protected:

  /**
   * @brief planner setup
   */
  void setup();

  /**
   * @brief Gets the start and goal joint values from the motion plan request passed.
   * @param start The start joint values
   * @param goal  The goal joint values
   * @return  true if succeeded, false otherwise.
   */
  bool getStartAndGoal(Eigen::VectorXd& start, Eigen::VectorXd& goal);

  /**
   * @brief This function 1) gets the seed trajectory from the active motion plan request, 2) checks to see if
   * the given seed trajectory makes sense in the context of the user provided goal constraints, 3) modifies
   * the seed's first and last point to 'fix' it for small deviations in the goal constraints and 4) applies
   * a smoothing method to the seed.
   * @param parameters  Output argument containing the seed parameters
   * @return True if the seed state is deemed to be valid. False otherwise.
   */
  bool getSeedParameters(Eigen::MatrixXd& parameters) const;

  /**
   * @brief Converts from an Eigen Matrix to to a joint trajectory
   * @param parameters  The input matrix of size [num joints][num_timesteps] containing the trajectory joint values.
   * @param traj        A trajectory in joint space.
   * @return  true if succeeded, false otherwise.
   */
  bool parametersToJointTrajectory(const Eigen::MatrixXd& parameters, trajectory_msgs::JointTrajectory& traj);

  /**
   * @brief Converts from a joint trajectory to an Eigen Matrix.
   * @param traj        The input trajectory in joint space.
   * @param parameters  The matrix of size [num joints][num_timesteps] containing the trajectory joint values.
   * @return  true if succeeded, false otherwise.
   */
  bool jointTrajectorytoParameters(const trajectory_msgs::JointTrajectory& traj, Eigen::MatrixXd& parameters) const;

  /**
   * @brief Populates a seed joint trajectory from the 'trajectory_constraints' moveit_msgs::Constraints[] array.
   *  each entry in the array is considered to be joint values for that time step.
   * @param req   The motion plan request containing the seed trajectory in the 'trajectory_constraints' field.
   * @param seed  The output seed trajectory which is used to initialize the STOMP optimization
   * @return true if succeeded, false otherwise.
   */
  bool extractSeedTrajectory(const moveit_msgs::MotionPlanRequest& req, trajectory_msgs::JointTrajectory& seed) const;

protected:

  // stomp optimization
  std::shared_ptr< stomp_core::Stomp> stomp_;
  StompOptimizationTaskPtr task_;
  XmlRpc::XmlRpcValue config_;
  stomp_core::StompConfiguration stomp_config_;

  // robot environment
  moveit::core::RobotModelConstPtr robot_model_;

  // ros tasks
  ros::NodeHandlePtr ph_;
};


} /* namespace stomp_moveit */
#endif /* STOMP_MOVEIT_STOMP_PLANNER_H_ */
