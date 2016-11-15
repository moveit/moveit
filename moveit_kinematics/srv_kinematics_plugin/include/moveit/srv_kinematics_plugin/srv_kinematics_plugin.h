/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
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
 *   * Neither the name of JSK, The University of Tokyo nor the names of its
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

/* Author: Dave Coleman, Masaki Murooka
   Desc:   Connects MoveIt to any inverse kinematics solver via a ROS service call
           Supports planning groups with multiple tip frames
           \todo: better support for mimic joints
           \todo: better support for redundant joints
*/

#ifndef MOVEIT_ROS_PLANNING_SRV_KINEMATICS_PLUGIN_
#define MOVEIT_ROS_PLANNING_SRV_KINEMATICS_PLUGIN_

// ROS
#include <ros/ros.h>

// System
#include <memory>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace srv_kinematics_plugin
{
/**
 * @brief Specific implementation of kinematics using ROS service calls to communicate with
   external IK solvers. This version can be used with any robot. Supports non-chain kinematic groups
 */
class SrvKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  /**
   *  @brief Default constructor
   */
  SrvKinematicsPlugin();

  virtual bool
  getPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code,
                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
                   std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
                   const std::vector<double> &consistency_limits, std::vector<double> &solution,
                   moveit_msgs::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool searchPositionIK(
      const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
      std::vector<double> &solution, const IKCallbackFn &solution_callback, moveit_msgs::MoveItErrorCodes &error_code,
      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
                   const std::vector<double> &consistency_limits, std::vector<double> &solution,
                   const IKCallbackFn &solution_callback, moveit_msgs::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool getPositionFK(const std::vector<std::string> &link_names, const std::vector<double> &joint_angles,
                             std::vector<geometry_msgs::Pose> &poses) const;

  virtual bool initialize(const std::string &robot_description, const std::string &group_name,
                          const std::string &base_name, const std::string &tip_frame, double search_discretization)
  {
    std::vector<std::string> tip_frames;
    tip_frames.push_back(tip_frame);
    return initialize(robot_description, group_name, base_name, tip_frames, search_discretization);
  }

  virtual bool initialize(const std::string &robot_description, const std::string &group_name,
                          const std::string &base_name, const std::vector<std::string> &tip_frames,
                          double search_discretization);

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  const std::vector<std::string> &getJointNames() const;

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  const std::vector<std::string> &getLinkNames() const;

  /**
   * @brief  Return all the variable names in the order they are represented internally
   */
  const std::vector<std::string> &getVariableNames() const;

protected:
  virtual bool
  searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
                   std::vector<double> &solution, const IKCallbackFn &solution_callback,
                   moveit_msgs::MoveItErrorCodes &error_code, const std::vector<double> &consistency_limits,
                   const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool
  searchPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses, const std::vector<double> &ik_seed_state,
                   double timeout, const std::vector<double> &consistency_limits, std::vector<double> &solution,
                   const IKCallbackFn &solution_callback, moveit_msgs::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  virtual bool setRedundantJoints(const std::vector<unsigned int> &redundant_joint_indices);

private:
  bool timedOut(const ros::WallTime &start_time, double duration) const;

  int getJointIndex(const std::string &name) const;

  bool isRedundantJoint(unsigned int index) const;

  bool active_; /** Internal variable that indicates whether solvers are configured and ready */

  moveit_msgs::KinematicSolverInfo ik_group_info_; /** Stores information for the inverse kinematics solver */

  unsigned int dimension_; /** Dimension of the group */

  robot_model::RobotModelPtr robot_model_;
  robot_model::JointModelGroup *joint_model_group_;

  robot_state::RobotStatePtr robot_state_;

  int num_possible_redundant_joints_;

  std::shared_ptr<ros::ServiceClient> ik_service_client_;
};
}

#endif
