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

/* Author: Sachin Chitta, David Lu!!, Ugo Cupcic */

#ifndef MOVEIT_ROS_PLANNING_KDL_KINEMATICS_PLUGIN_
#define MOVEIT_ROS_PLANNING_KDL_KINEMATICS_PLUGIN_

// ROS
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// KDL
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <moveit/kdl_kinematics_plugin/chainiksolver_pos_nr_jl_mimic.hpp>
#include <moveit/kdl_kinematics_plugin/chainiksolver_vel_pinv_mimic.hpp>
#include <moveit/kdl_kinematics_plugin/joint_mimic.hpp>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace kdl_kinematics_plugin
{
/**
 * @brief Specific implementation of kinematics using KDL. This version can be used with any robot.
 */
class KDLKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  /**
   *  @brief Default constructor
   */
  KDLKinematicsPlugin();

  virtual bool
  getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   std::vector<double>& solution, const IKCallbackFn& solution_callback,
                   moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                             std::vector<geometry_msgs::Pose>& poses) const;

  virtual bool initialize(const std::string& robot_description, const std::string& group_name,
                          const std::string& base_name, const std::string& tip_name, double search_discretization);

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  const std::vector<std::string>& getJointNames() const;

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  const std::vector<std::string>& getLinkNames() const;

protected:
  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param solution the solution vector
   * @param solution_callback A callback solution for the IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @param check_consistency Set to true if consistency check needs to be performed
   * @param redundancy The index of the redundant joint
   * @param consistency_limit The returned solutuion will contain a value for the redundant joint in the range
   * [seed_state(redundancy_limit)-consistency_limit,seed_state(redundancy_limit)+consistency_limit]
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                        std::vector<double>& solution, const IKCallbackFn& solution_callback,
                        moveit_msgs::MoveItErrorCodes& error_code, const std::vector<double>& consistency_limits,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool setRedundantJoints(const std::vector<unsigned int>& redundant_joint_indices);

private:
  bool timedOut(const ros::WallTime& start_time, double duration) const;

  /** @brief Check whether the solution lies within the consistency limit of the seed state
   *  @param seed_state Seed state
   *  @param redundancy Index of the redundant joint within the chain
   *  @param consistency_limit The returned state for redundant joint should be in the range
   * [seed_state(redundancy_limit)-consistency_limit,seed_state(redundancy_limit)+consistency_limit]
   *  @param solution solution configuration
   *  @return true if check succeeds
   */
  bool checkConsistency(const KDL::JntArray& seed_state, const std::vector<double>& consistency_limit,
                        const KDL::JntArray& solution) const;

  int getJointIndex(const std::string& name) const;

  int getKDLSegmentIndex(const std::string& name) const;

  void getRandomConfiguration(KDL::JntArray& jnt_array, bool lock_redundancy) const;

  /** @brief Get a random configuration within joint limits close to the seed state
   *  @param seed_state Seed state
   *  @param redundancy Index of the redundant joint within the chain
   *  @param consistency_limit The returned state will contain a value for the redundant joint in the range
   * [seed_state(redundancy_limit)-consistency_limit,seed_state(redundancy_limit)+consistency_limit]
   *  @param jnt_array Returned random configuration
   */
  void getRandomConfiguration(const KDL::JntArray& seed_state, const std::vector<double>& consistency_limits,
                              KDL::JntArray& jnt_array, bool lock_redundancy) const;

  bool isRedundantJoint(unsigned int index) const;

  bool active_; /** Internal variable that indicates whether solvers are configured and ready */

  moveit_msgs::KinematicSolverInfo ik_chain_info_; /** Stores information for the inverse kinematics solver */

  moveit_msgs::KinematicSolverInfo fk_chain_info_; /** Store information for the forward kinematics solver */

  KDL::Chain kdl_chain_;

  unsigned int dimension_; /** Dimension of the group */

  KDL::JntArray joint_min_, joint_max_; /** Joint limits */

  mutable random_numbers::RandomNumberGenerator random_number_generator_;

  robot_model::RobotModelPtr robot_model_;

  robot_state::RobotStatePtr state_, state_2_;

  int num_possible_redundant_joints_;
  std::vector<unsigned int> redundant_joints_map_index_;

  // Storage required for when the set of redundant joints is reset
  bool position_ik_;  // whether this solver is only being used for position ik
  robot_model::JointModelGroup* joint_model_group_;
  double max_solver_iterations_;
  double epsilon_;
  std::vector<JointMimic> mimic_joints_;
};
}

#endif
