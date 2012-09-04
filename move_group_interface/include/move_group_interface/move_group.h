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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#ifndef MOVEIT_MOVE_GROUP_INTERFACE_MOVE_GROUP_
#define MOVEIT_MOVE_GROUP_INTERFACE_MOVE_GROUP_

#include <planning_models/kinematic_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/PlannerInterfaceDescription.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/shared_ptr.hpp>
#include <tf/tf.h>

/** \brief Simple interface to the MoveGroup action */
namespace move_group_interface
{

/** \brief Client class for the MoveGroup action */
class MoveGroup
{
public:
  
  static const std::string ROBOT_DESCRIPTION;
  static const std::string JOINT_STATE_TOPIC;

  struct Options
  {
    Options(const std::string &group_name) : group_name_(group_name),
                                             joint_state_topic_(JOINT_STATE_TOPIC),
                                             robot_description_(ROBOT_DESCRIPTION)
    {
    }
    std::string group_name_;
    std::string joint_state_topic_;
    std::string robot_description_;
    planning_models::KinematicModelConstPtr kinematic_model_;
  };
  
  struct Plan
  {
    moveit_msgs::RobotState start_state_;
    moveit_msgs::RobotTrajectory trajectory_;
  };
  
  MoveGroup(const Options &opt, const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>(), const ros::Duration &wait_for_server = ros::Duration(0, 0)); 
  MoveGroup(const std::string &group, const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>(), const ros::Duration &wait_for_server = ros::Duration(0, 0));

  ~MoveGroup(void);
  
  const std::string& getName(void) const;

  bool getInterfaceDescription(moveit_msgs::PlannerInterfaceDescription &desc);
  
  /** \brief Plan and execute a trajectory that takes the group of joints declared in the constructor to the specified target.
      This call is not blocking (does not wait for the execution of the trajectory to complete). */
  bool asyncMove(void);
  
  /** \brief Plan and execute a trajectory that takes the group of joints declared in the constructor to the specified target.
      This call is always blocking (waits for the execution of the trajectory to complete). */
  bool move(void);

  /** \brief Compute a motion plan that takes the group declared in the constructor from the current state to the specified
      target. No execution is performed. The resulting plan is stored in \e plan*/
  bool plan(Plan &plan);
  
  bool asyncExecute(const Plan &plan);
  
  bool execute(const Plan &plan);

  /** \brief Stop any trajectory execution, if one is active */
  void stop(void);
  
  /** \brief Specify whether the robot is allowed to look around before moving if it determines it should (default is true) */
  void allowLooking(bool flag);

  /** \brief Specify whether the robot is allowed to replan if it detects changes in the environment */
  void allowReplanning(bool flag);
  
  void setPlannerId(const std::string &planner_id);
  
  /** \brief If a different start state should be considered instead of the current state of the robot, this function sets that state */
  void setStartState(const planning_models::KinematicState &start_state);
  
  void setStartStateToCurrentState(void);
  
  void setJointValueTarget(const std::vector<double> &group_variable_values);

  void setJointValueTarget(const std::map<std::string, double> &variable_values);

  void setJointValueTarget(const planning_models::KinematicState &kinematic_state);

  void setJointValueTarget(const planning_models::KinematicState::JointStateGroup &joint_state_group);
  
  void setJointValueTarget(const planning_models::KinematicState::JointState &joint_state);

  void setJointValueTarget(const std::string &joint_name, const std::vector<double> &values);

  void setJointValueTarget(const std::string &joint_name, double value);
  
  void setJointValueTarget(const sensor_msgs::JointState &state);

  const planning_models::KinematicState::JointStateGroup& getJointValueTarget(void) const;

  void setPositionTarget(double x, double y, double z);
  
  void setOrientationTarget(double x, double y, double z);

  void setPoseTarget(const Eigen::Affine3d &end_effector_pose);
  
  void setPoseTarget(const geometry_msgs::Pose &target);

  void setPoseTarget(const geometry_msgs::PoseStamped &target);

  void setPoseReferenceFrame(const std::string &pose_reference_frame);

  const Eigen::Affine3d& getPoseTarget(void) const;

  const std::string& getPoseReferenceFrame(void) const;

  void setEndEffectorLink(const std::string &link_name);

  const std::string& getEndEffectorLink(void) const;

  const std::vector<std::string>& getJoints(void) const;

  double getGoalTolerance(void) const;
  
  void setGoalTolerance(double tolerance);

  void setRandomTarget(void);
  
  /** \brief Set the current joint values to be ones previously remembered by rememberJointValues() or, if not found, that are specified in the SRDF under the name \e name */
  bool setNamedTarget(const std::string &name);
  
  /** \brief Remember the current joint values (of the robot being monitored) under \e name. These can be used by setNamedTarget() */
  void rememberJointValues(const std::string &name);

  /** \brief Remember the specified joint values  under \e name. These can be used by setNamedTarget() */
  void rememberJointValues(const std::string &name, const std::vector<double> &values);

  std::vector<double> getCurrentJointValues(void);

  std::vector<double> getRandomJointValues(void);

  Eigen::Affine3d getCurrentPose(void);

  planning_models::KinematicStatePtr getCurrentState(void);

  void forgetJointValues(const std::string &name);
  
  const std::map<std::string, std::vector<double> >& getRememberedJointValues(void) const
  {
    return remembered_joint_values_;
  }
  
private:

  std::map<std::string, std::vector<double> > remembered_joint_values_;
  class MoveGroupImpl;
  MoveGroupImpl *impl_;
  
};

}
#endif
