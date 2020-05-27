/*******************************************************************************
 *      Title     : jog_interface_base.cpp
 *      Project   : moveit_jog_arm
 *      Created   : 3/9/2017
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <memory>
#include <mutex>

#include <Eigen/Eigenvalues>

#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/ChangeDriftDimensions.h>
#include <moveit_msgs/ChangeControlDimensions.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>

#include "collision_check_thread.h"
#include "jog_arm_data.h"
#include "jog_calcs.h"
#include "low_pass_filter.h"

namespace moveit_jog_arm
{
/**
 * Class JogArm - Base class for C++ interface and ROS node.
 * Handles ROS subs & pubs and creates the worker threads.
 */
class JogArm
{
public:
  JogArm(ros::NodeHandle& nh, const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  ~JogArm();

  /** \brief start jog arm threads */
  void start();

  /** \brief stop jog arm threads */
  void stop();

  /** \brief Pause or unpause processing jog commands while keeping the threads alive */
  void setPaused(bool paused);

  /**
   * Allow drift in certain dimensions. For example, may allow the wrist to rotate freely.
   * This can help avoid singularities.
   *
   * @param request the service request
   * @param response the service response
   * @return true if the adjustment was made
   */
  bool changeDriftDimensions(moveit_msgs::ChangeDriftDimensions::Request& req,
                             moveit_msgs::ChangeDriftDimensions::Response& res);

  /** \brief Start the main calculation thread */
  // Service callback for changing jogging dimensions
  bool changeControlDimensions(moveit_msgs::ChangeControlDimensions::Request& req,
                               moveit_msgs::ChangeControlDimensions::Response& res);

  /** \brief Provide a Cartesian velocity command to the jogger.
   * The units are determined by settings in the yaml file.
   */
  void provideTwistStampedCommand(const geometry_msgs::TwistStampedConstPtr& msg);

  /** \brief Send joint position(s) commands */
  void provideJointCommand(const control_msgs::JointJogConstPtr& msg);

  /**
   * Returns the most recent JointState that the jogger has received.
   * May eliminate the need to create your own joint_state subscriber.
   *
   * @return the most recent joints known to the jogger
   */
  sensor_msgs::JointState getJointState();

  /**
   * Get the MoveIt planning link transform.
   * The transform from the MoveIt planning frame to robot_link_command_frame
   *
   * @param transform the transform that will be calculated
   * @return true if a valid transform was available
   */
  bool getCommandFrameTransform(Eigen::Isometry3d& transform);

  /**
   * Get the status of the jogger.
   *
   * @return 0 for no warning. The meaning of nonzero values can be seen in status_codes.h
   */
  StatusCode getJoggerStatus() const;

private:
  bool readParameters();
  void run(const ros::TimerEvent& timer_event);
  void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryPtr& msg);

  ros::NodeHandle nh_;

  // Pointer to the collision environment
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Store the parameters that were read from ROS server
  JogArmParameters parameters_;

  // Share data between threads
  JogArmShared shared_variables_;

  // Jog calcs
  std::unique_ptr<JogCalcs> jog_calcs_;

  // Collision checks
  std::unique_ptr<CollisionCheckThread> collision_checker_;

  // ROS
  ros::Timer timer_;
  ros::Duration period_;
  ros::Publisher outgoing_cmd_pub_;
  ros::Publisher twist_stamped_pub_;
  ros::Publisher joint_jog_pub_;
  ros::ServiceServer drift_dimensions_server_;
  ros::ServiceServer dims_server_;
  ros::Subscriber joint_trajectory_sub_;

  // latest_state_mutex_ is used to protect the state below it
  mutable std::mutex latest_state_mutex_;
  trajectory_msgs::JointTrajectoryPtr latest_joint_trajectory_;
};

}  // namespace moveit_jog_arm
