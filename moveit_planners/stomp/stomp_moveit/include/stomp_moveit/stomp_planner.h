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
#ifndef STOMP_MOVEIT_STOMP_PLANNER_H_
#define STOMP_MOVEIT_STOMP_PLANNER_H_

#include <moveit/planning_interface/planning_interface.h>
#include <stomp_core/stomp.h>
#include <stomp_moveit/stomp_optimization_task.h>
#include <boost/thread.hpp>
#include <ros/ros.h>

namespace stomp_moveit
{

using StompOptimizationTaskPtr = boost::shared_ptr<StompOptimizationTask>;

class StompPlanner: public planning_interface::PlanningContext
{
public:
  StompPlanner(const std::string& group,const XmlRpc::XmlRpcValue& config,const moveit::core::RobotModelConstPtr& model);
  virtual ~StompPlanner();

  /**
   * @brief Solve the motion planning problem and store the result in \e res
   */
  virtual bool solve(planning_interface::MotionPlanResponse &res) override;

  /**
   * @brief Solve the motion planning problem and store the detailed result in \e res
   */
  virtual bool solve(planning_interface::MotionPlanDetailedResponse &res) override;

  /**
   * @brief Request termination, if a solve() function is currently computing plans
   */
  virtual bool terminate() override;

  /**
   * @brief Clears results from previous plan
   */
  virtual void clear() override;

  static bool getConfigData(ros::NodeHandle &nh, std::map<std::string, XmlRpc::XmlRpcValue> &config, std::string param = std::string("stomp"));

  /**
   * @brief Determine whether this plugin instance is able to represent this planning request
   */
  bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req)  const;

protected:

  void setup();
  bool getStartAndGoal(std::vector<double>& start, std::vector<double>& goal);

  // Converts from STOMP optimization format to a joint trajectory
  bool parametersToJointTrajectory(Eigen::MatrixXd& parameters, trajectory_msgs::JointTrajectory& traj);

  // Converts from a joint trajectory to STOMP optimization format
  bool jointTrajectorytoParameters(const trajectory_msgs::JointTrajectory& traj, Eigen::MatrixXd& parameters) const;

  // Attempts to parse a seed trajectory out of the 'trajectory_constraints' field of the motion planning
  // request. If successful, returns true and sets 'seed' to the seed trajectory.
  bool extractSeedTrajectory(const moveit_msgs::MotionPlanRequest& req, trajectory_msgs::JointTrajectory& seed) const;

protected:

  // stomp optimization
  boost::shared_ptr< stomp_core::Stomp> stomp_;
  StompOptimizationTaskPtr task_;
  XmlRpc::XmlRpcValue config_;
  stomp_core::StompConfiguration stomp_config_;

  // robot environment
  moveit::core::RobotModelConstPtr robot_model_;
};

/**
 * @brief A hack to encode seed trajectories directly into a set of constraints. Only position is currently
 * encoded.
 *
 * Assumes that the seed's joint_names & values match the corresponding moveit planning group.
 */
moveit_msgs::TrajectoryConstraints encodeSeedTrajectory(const trajectory_msgs::JointTrajectory& seed);

} /* namespace stomp_moveit */
#endif /* STOMP_MOVEIT_STOMP_PLANNER_H_ */
