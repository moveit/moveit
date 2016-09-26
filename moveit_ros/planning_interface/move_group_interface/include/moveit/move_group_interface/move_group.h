/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, SRI International
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

/* Author: Ioan Sucan, Sachin Chitta */

#ifndef MOVEIT_MOVE_GROUP_INTERFACE_MOVE_GROUP_
#define MOVEIT_MOVE_GROUP_INTERFACE_MOVE_GROUP_

#include <moveit/macros/class_forward.h>
#include <moveit/macros/deprecation.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/PlannerInterfaceDescription.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/shared_ptr.hpp>
#include <tf/tf.h>

namespace moveit
{

/** \brief Simple interface to MoveIt! components */
namespace planning_interface
{

class MoveItErrorCode : public moveit_msgs::MoveItErrorCodes
{
public:
  MoveItErrorCode() { val = 0; };
  MoveItErrorCode(int code) { val = code; };
  MoveItErrorCode(const moveit_msgs::MoveItErrorCodes &code) { val = code.val; };
  operator bool() const { return val == moveit_msgs::MoveItErrorCodes::SUCCESS; }
};

MOVEIT_CLASS_FORWARD(MoveGroup);

/** \brief Client class for the MoveGroup action. This class includes many default settings to make things easy to use. */
class MoveGroup
{
public:

  /** \brief Default ROS parameter name from where to read the robot's URDF. Set to 'robot_description' */
  static const std::string ROBOT_DESCRIPTION;

  /** \brief Specification of options to use when constructing the MoveGroup client class */
  struct Options
  {
    Options(const std::string &group_name,
            const std::string &desc = ROBOT_DESCRIPTION,
	    const ros::NodeHandle &node_handle = ros::NodeHandle()) :
      group_name_(group_name),
      robot_description_(desc),
      node_handle_(node_handle)
    {
    }

    /// The group to construct the class instance for
    std::string group_name_;

    /// The robot description parameter name (if different from default)
    std::string robot_description_;

    /// Optionally, an instance of the RobotModel to use can be also specified
    robot_model::RobotModelConstPtr robot_model_;

    ros::NodeHandle node_handle_;
  };

  MOVEIT_STRUCT_FORWARD(Plan);

  /// The representation of a motion plan (as ROS messasges)
  struct Plan
  {
    /// The full starting state used for planning
    moveit_msgs::RobotState start_state_;

    /// The trajectory of the robot (may not contain joints that are the same as for the start_state_)
    moveit_msgs::RobotTrajectory trajectory_;

    /// The amount of time it took to generate the plan
    double planning_time_;
  };

  /**
      \brief Construct a client for the MoveGroup action using a specified set of options \e opt.

      \param opt. A MoveGroup::Options structure, if you pass a ros::NodeHandle with a specific callback queue, it has to be of type ros::CallbackQueue
        (which is the default type of callback queues used in ROS)
      \param tf. Specify a TF instance to use. If not specified, one will be constructed internally.
      \param wait_for_servers. Timeout for connecting to action servers. Zero time means unlimited waiting.
    */
  MoveGroup(const Options &opt, const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>(),
            const ros::WallDuration &wait_for_servers = ros::WallDuration());
  MOVEIT_DEPRECATED MoveGroup(const Options &opt, const boost::shared_ptr<tf::Transformer> &tf,
                              const ros::Duration &wait_for_servers);

  /**
      \brief Construct a client for the MoveGroup action for a particular \e group.

      \param tf. Specify a TF instance to use. If not specified, one will be constructed internally.
      \param wait_for_servers. Timeout for connecting to action servers. Zero time means unlimited waiting.
    */
  MoveGroup(const std::string &group, const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>(),
            const ros::WallDuration &wait_for_servers = ros::WallDuration());
  MOVEIT_DEPRECATED MoveGroup(const std::string &group, const boost::shared_ptr<tf::Transformer> &tf,
                              const ros::Duration &wait_for_servers);

  ~MoveGroup();

  /** \brief Get the name of the group this instance operates on */
  const std::string& getName() const;

  /** \brief Get the names of the named robot states available as targets, both either remembered states or default states from srdf */
  const std::vector<std::string> getNamedTargets();

  /** \brief Get the RobotModel object. */
  robot_model::RobotModelConstPtr getRobotModel() const;

  /** \brief Get the ROS node handle of this instance operates on */
  const ros::NodeHandle& getNodeHandle() const;

  /** \brief Get the name of the frame in which the robot is planning */
  const std::string& getPlanningFrame() const;

  /** \brief Get vector of names of joints available in move group */
  const std::vector<std::string>& getJointNames();

  /** \brief Get the joint angles for targets specified by name */
  std::map<std::string,double> getNamedTargetValues(const std::string& name);

  /** \brief Get only the active (actuated) joints this instance operates on */
  const std::vector<std::string>& getActiveJoints() const;

  /** \brief Get all the joints this instance operates on (including fixed joints)*/
  const std::vector<std::string>& getJoints() const;

  /** \brief Get the number of variables used to describe the state of this group. This is larger or equal to the number of DOF. */
  unsigned int getVariableCount() const;

  /** \brief Get the description of the planning plugin loaded by the action server */
  bool getInterfaceDescription(moveit_msgs::PlannerInterfaceDescription &desc);

  /** \brief Get the planner parameters for given group and planner_id */
  std::map<std::string, std::string> getPlannerParams(const std::string &planner_id,
                                                      const std::string &group="");

  /** \brief Set the planner parameters for given group and planner_id */
  void setPlannerParams(const std::string &planner_id,
                        const std::string &group,
                        const std::map<std::string, std::string> &params,
                        bool bReplace = false);

  /** \brief Get the default planner for a given group (or global default) */
  std::string getDefaultPlannerId(const std::string &group="") const;

  /** \brief Specify a planner to be used for further planning */
  void setPlannerId(const std::string &planner_id);

  /** \brief Specify the maximum amount of time to use when planning */
  void setPlanningTime(double seconds);

  /** \brief Set the number of times the motion plan is to be computed from scratch before the shortest solution is returned. The default value is 1.*/
  void setNumPlanningAttempts(unsigned int num_planning_attempts);

  /** \brief Set a scaling factor for optionally reducing the maximum joint velocity.
      Allowed values are in (0,1]. The maximum joint velocity specified
      in the robot model is multiplied by the factor. If outside valid range
      (imporantly, this includes it being set to 0.0), the factor is set to a
      default value of 1.0 internally (i.e. maximum joint velocity) */
  void setMaxVelocityScalingFactor(double max_velocity_scaling_factor);

  /** \brief Set a scaling factor for optionally reducing the maximum joint acceleration.
      Allowed values are in (0,1]. The maximum joint acceleration specified
      in the robot model is multiplied by the factor. If outside valid range
      (imporantly, this includes it being set to 0.0), the factor is set to a
      default value of 1.0 internally (i.e. maximum joint acceleration) */  
  void setMaxAccelerationScalingFactor(double max_acceleration_scaling_factor);

  /** \brief Get the number of seconds set by setPlanningTime() */
  double getPlanningTime() const;

  /** \brief Get the tolerance that is used for reaching a joint goal. This is distance for each joint in configuration space */
  double getGoalJointTolerance() const;

  /** \brief Get the tolerance that is used for reaching a position goal. This is be the radius of a sphere where the end-effector must reach.*/
  double getGoalPositionTolerance() const;

  /** \brief Get the tolerance that is used for reaching an orientation goal. This is the tolerance for roll, pitch and yaw, in radians. */
  double getGoalOrientationTolerance() const;

  /** \brief Set the tolerance that is used for reaching the goal. For
      joint state goals, this will be distance for each joint, in the
      configuration space (radians or meters depending on joint type). For pose
      goals this will be the radius of a sphere where the end-effector must
      reach. This function simply triggers calls to setGoalPositionTolerance(),
      setGoalOrientationTolerance() and setGoalJointTolerance(). */
  void setGoalTolerance(double tolerance);

  /** \brief Set the joint tolerance (for each joint) that is used for reaching the goal when moving to a joint value target. */
  void setGoalJointTolerance(double tolerance);

  /** \brief Set the position tolerance that is used for reaching the goal when moving to a pose. */
  void setGoalPositionTolerance(double tolerance);

  /** \brief Set the orientation tolerance that is used for reaching the goal when moving to a pose. */
  void setGoalOrientationTolerance(double tolerance);

  /** \brief Specify the workspace bounding box.
       The box is specified in the planning frame (i.e. relative to the robot root link start position).
       This is useful when the MoveGroup's group contains the root joint of the robot -- i.e. when planning motion for the robot relative to the world. */
  void setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz);

  /** \brief If a different start state should be considered instead of the current state of the robot, this function sets that state */
  void setStartState(const moveit_msgs::RobotState &start_state);

  /** \brief If a different start state should be considered instead of the current state of the robot, this function sets that state */
  void setStartState(const robot_state::RobotState &start_state);

  /** \brief Set the starting state for planning to be that reported by the robot's joint state publication */
  void setStartStateToCurrentState();

  /** \brief For pick/place operations, the name of the support surface is used to specify the fact that attached objects are allowed to touch the support surface */
  void setSupportSurfaceName(const std::string &name);

  /**
   * \name Setting a joint state target (goal)
   *
   * There are 2 types of goal targets:
   * \li a JointValueTarget (aka JointStateTarget) specifies an absolute value for each joint (angle for rotational joints or position for prismatic joints).
   * \li a PoseTarget (Position, Orientation, or Pose) specifies the pose of one or more end effectors (and the planner can use any joint values that reaches the pose(s)).
   *
   * Only one or the other is used for planning.  Calling any of the
   * set*JointValueTarget() functions sets the current goal target to the
   * JointValueTarget.  Calling any of the setPoseTarget(),
   * setOrientationTarget(), setRPYTarget(), setPositionTarget() functions sets
   * the current goal target to the Pose target.
   */
  /**@{*/

  /** \brief Set the JointValueTarget and use it for future planning requests.

      \e group_variable_values MUST contain exactly one value per joint
      variable in the same order as returned by
      getJointValueTarget().getJointModelGroup(getName())->getVariableNames().

      This always sets all of the group's joint values.

      After this call, the JointValueTarget is used \b instead of any
      previously set Position, Orientation, or Pose targets.

      If these values are out of bounds then false is returned BUT THE VALUES
      ARE STILL SET AS THE GOAL. */
  bool setJointValueTarget(const std::vector<double> &group_variable_values);

  /** \brief Set the JointValueTarget and use it for future planning requests.

      \e variable_values is a map of joint variable names to values.  Joints in
      the group are used to set the JointValueTarget.  Joints in the model but
      not in the group are ignored.  An exception is thrown if a joint name is
      not found in the model.  Joint variables in the group that are missing
      from \e variable_values remain unchanged (to reset all target variables
      to their current values in the robot use
      setJointValueTarget(getCurrentJointValues())).

      After this call, the JointValueTarget is used \b instead of any
      previously set Position, Orientation, or Pose targets.

      If these values are out of bounds then false is returned BUT THE VALUES
      ARE STILL SET AS THE GOAL. */
  bool setJointValueTarget(const std::map<std::string, double> &variable_values);

  /** \brief Set the JointValueTarget and use it for future planning requests.

      The target for all joints in the group are set to the value in \e robot_state.

      After this call, the JointValueTarget is used \b instead of any
      previously set Position, Orientation, or Pose targets.

      If these values are out of bounds then false is returned BUT THE VALUES
      ARE STILL SET AS THE GOAL. */
  bool setJointValueTarget(const robot_state::RobotState &robot_state);

  /** \brief Set the JointValueTarget and use it for future planning requests.

      \e values MUST have one value for each variable in joint \e joint_name.
      \e values are set as the target for this joint.
      Other joint targets remain unchanged.

      After this call, the JointValueTarget is used \b instead of any
      previously set Position, Orientation, or Pose targets.

      If these values are out of bounds then false is returned BUT THE VALUES
      ARE STILL SET AS THE GOAL. */
  bool setJointValueTarget(const std::string &joint_name, const std::vector<double> &values);

  /** \brief Set the JointValueTarget and use it for future planning requests.

      Joint \e joint_name must be a 1-DOF joint.
      \e value is set as the target for this joint.
      Other joint targets remain unchanged.

      After this call, the JointValueTarget is used \b instead of any
      previously set Position, Orientation, or Pose targets.

      If these values are out of bounds then false is returned BUT THE VALUES
      ARE STILL SET AS THE GOAL. */
  bool setJointValueTarget(const std::string &joint_name, double value);

  /** \brief Set the JointValueTarget and use it for future planning requests.

      \e state is used to set the target joint state values.
      Values not specified in \e state remain unchanged.

      After this call, the JointValueTarget is used \b instead of any
      previously set Position, Orientation, or Pose targets.

      If these values are out of bounds then false is returned BUT THE VALUES
      ARE STILL SET AS THE GOAL. */
  bool setJointValueTarget(const sensor_msgs::JointState &state);

  /** \brief Set the joint state goal for a particular joint by computing IK.

      This is different from setPoseTarget() in that a single IK state (aka
      JointValueTarget) is computed using IK, and the resulting
      JointValueTarget is used as the target for planning.

      After this call, the JointValueTarget is used \b instead of any
      previously set Position, Orientation, or Pose targets.

      If IK fails to find a solution then false is returned BUT THE PARTIAL
      RESULT OF IK IS STILL SET AS THE GOAL. */
  bool setJointValueTarget(const geometry_msgs::Pose &eef_pose, const std::string &end_effector_link = "");

  /** \brief Set the joint state goal for a particular joint by computing IK.

      This is different from setPoseTarget() in that a single IK state (aka
      JointValueTarget) is computed using IK, and the resulting
      JointValueTarget is used as the target for planning.

      After this call, the JointValueTarget is used \b instead of any
      previously set Position, Orientation, or Pose targets.

      If IK fails to find a solution then false is returned BUT THE PARTIAL
      RESULT OF IK IS STILL SET AS THE GOAL. */
  bool setJointValueTarget(const geometry_msgs::PoseStamped &eef_pose, const std::string &end_effector_link = "");

  /** \brief Set the joint state goal for a particular joint by computing IK.

      This is different from setPoseTarget() in that a single IK state (aka
      JointValueTarget) is computed using IK, and the resulting
      JointValueTarget is used as the target for planning.

      After this call, the JointValueTarget is used \b instead of any
      previously set Position, Orientation, or Pose targets.

      If IK fails to find a solution then false is returned BUT THE PARTIAL
      RESULT OF IK IS STILL SET AS THE GOAL. */
  bool setJointValueTarget(const Eigen::Affine3d &eef_pose, const std::string &end_effector_link = "");

  /** \brief Set the joint state goal for a particular joint by computing IK.

      This is different from setPoseTarget() in that a single IK state (aka
      JointValueTarget) is computed using IK, and the resulting
      JointValueTarget is used as the target for planning.

      After this call, the JointValueTarget is used \b instead of any
      previously set Position, Orientation, or Pose targets.

      If IK fails to find a solution then an approximation is used. */
  bool setApproximateJointValueTarget(const geometry_msgs::Pose &eef_pose, const std::string &end_effector_link = "");

  /** \brief Set the joint state goal for a particular joint by computing IK.

      This is different from setPoseTarget() in that a single IK state (aka
      JointValueTarget) is computed using IK, and the resulting
      JointValueTarget is used as the target for planning.

      After this call, the JointValueTarget is used \b instead of any
      previously set Position, Orientation, or Pose targets.

      If IK fails to find a solution then an approximation is used. */
  bool setApproximateJointValueTarget(const geometry_msgs::PoseStamped &eef_pose, const std::string &end_effector_link = "");

  /** \brief Set the joint state goal for a particular joint by computing IK.

      This is different from setPoseTarget() in that a single IK state (aka
      JointValueTarget) is computed using IK, and the resulting
      JointValueTarget is used as the target for planning.

      After this call, the JointValueTarget is used \b instead of any
      previously set Position, Orientation, or Pose targets.

      If IK fails to find a solution then an approximation is used. */
  bool setApproximateJointValueTarget(const Eigen::Affine3d &eef_pose, const std::string &end_effector_link = "");

  /** \brief Set the joint state goal to a random joint configuration

      After this call, the JointValueTarget is used \b instead of any
      previously set Position, Orientation, or Pose targets. */
  void setRandomTarget();

  /** \brief Set the current joint values to be ones previously remembered by rememberJointValues() or, if not found,
      that are specified in the SRDF under the name \e name as a group state*/
  bool setNamedTarget(const std::string &name);

  /// Get the currently set joint state goal
  const robot_state::RobotState& getJointValueTarget() const;

  /**@}*/


  /**
   * \name Setting a pose target (goal)
   *
   * Setting a Pose (or Position or Orientation) target disables any previously
   * set JointValueTarget.
   *
   * For groups that have multiple end effectors, a pose can be set for each
   * end effector in the group.  End effectors which do not have a pose target
   * set will end up in arbitrary positions.
   */
  /**@{*/

  /** \brief Set the goal position of the end-effector \e end_effector_link to be (\e x, \e y, \e z).

      If \e end_effector_link is empty then getEndEffectorLink() is used.

      This new position target replaces any pre-existing JointValueTarget or
      pre-existing Position, Orientation, or Pose target for this \e
      end_effector_link. */
  bool setPositionTarget(double x, double y, double z, const std::string &end_effector_link = "");

  /** \brief Set the goal orientation of the end-effector \e end_effector_link to be (\e roll,\e pitch,\e yaw) radians.

      If \e end_effector_link is empty then getEndEffectorLink() is used.

      This new orientation target replaces any pre-existing JointValueTarget or
      pre-existing Position, Orientation, or Pose target for this \e
      end_effector_link. */
  bool setRPYTarget(double roll, double pitch, double yaw, const std::string &end_effector_link = "");

  /** \brief Set the goal orientation of the end-effector \e end_effector_link to be the quaternion (\e x,\e y,\e z,\e w).

      If \e end_effector_link is empty then getEndEffectorLink() is used.

      This new orientation target replaces any pre-existing JointValueTarget or
      pre-existing Position, Orientation, or Pose target for this \e
      end_effector_link. */
  bool setOrientationTarget(double x, double y, double z, double w, const std::string &end_effector_link = "");

  /** \brief Set the goal pose of the end-effector \e end_effector_link.

      If \e end_effector_link is empty then getEndEffectorLink() is used.

      This new pose target replaces any pre-existing JointValueTarget or
      pre-existing Position, Orientation, or Pose target for this \e
      end_effector_link. */
  bool setPoseTarget(const Eigen::Affine3d &end_effector_pose, const std::string &end_effector_link = "");

  /** \brief Set the goal pose of the end-effector \e end_effector_link.

      If \e end_effector_link is empty then getEndEffectorLink() is used.

      This new orientation target replaces any pre-existing JointValueTarget or
      pre-existing Position, Orientation, or Pose target for this \e
      end_effector_link. */
  bool setPoseTarget(const geometry_msgs::Pose &target, const std::string &end_effector_link = "");

  /** \brief Set the goal pose of the end-effector \e end_effector_link.

      If \e end_effector_link is empty then getEndEffectorLink() is used.

      This new orientation target replaces any pre-existing JointValueTarget or
      pre-existing Position, Orientation, or Pose target for this \e
      end_effector_link. */
  bool setPoseTarget(const geometry_msgs::PoseStamped &target, const std::string &end_effector_link = "");

  /** \brief Set goal poses for \e end_effector_link.

      If \e end_effector_link is empty then getEndEffectorLink() is used.

      When planning, the planner will find a path to one (arbitrarily chosen)
      pose from the list.  If this group contains multiple end effectors then
      all end effectors in the group should have the same number of pose
      targets.  If planning is successful then the result of the plan will
      place all end effectors at a pose from the same index in the list.  (In
      other words, if one end effector ends up at the 3rd pose in the list then
      all end effectors in the group will end up at the 3rd pose in their
      respective lists.  End effectors which do not matter (i.e. can end up in
      any position) can have their pose targets disabled by calling
      clearPoseTarget() for that end_effector_link.

      This new orientation target replaces any pre-existing JointValueTarget or
      pre-existing Position, Orientation, or Pose target(s) for this \e
      end_effector_link. */
  bool setPoseTargets(const EigenSTL::vector_Affine3d &end_effector_pose, const std::string &end_effector_link = "");

  /** \brief Set goal poses for \e end_effector_link.

      If \e end_effector_link is empty then getEndEffectorLink() is used.

      When planning, the planner will find a path to one (arbitrarily chosen)
      pose from the list.  If this group contains multiple end effectors then
      all end effectors in the group should have the same number of pose
      targets.  If planning is successful then the result of the plan will
      place all end effectors at a pose from the same index in the list.  (In
      other words, if one end effector ends up at the 3rd pose in the list then
      all end effectors in the group will end up at the 3rd pose in their
      respective lists.  End effectors which do not matter (i.e. can end up in
      any position) can have their pose targets disabled by calling
      clearPoseTarget() for that end_effector_link.

      This new orientation target replaces any pre-existing JointValueTarget or
      pre-existing Position, Orientation, or Pose target(s) for this \e
      end_effector_link. */
  bool setPoseTargets(const std::vector<geometry_msgs::Pose> &target, const std::string &end_effector_link = "");

  /** \brief Set goal poses for \e end_effector_link.

      If \e end_effector_link is empty then getEndEffectorLink() is used.

      When planning, the planner will find a path to one (arbitrarily chosen)
      pose from the list.  If this group contains multiple end effectors then
      all end effectors in the group should have the same number of pose
      targets.  If planning is successful then the result of the plan will
      place all end effectors at a pose from the same index in the list.  (In
      other words, if one end effector ends up at the 3rd pose in the list then
      all end effectors in the group will end up at the 3rd pose in their
      respective lists.  End effectors which do not matter (i.e. can end up in
      any position) can have their pose targets disabled by calling
      clearPoseTarget() for that end_effector_link.

      This new orientation target replaces any pre-existing JointValueTarget or
      pre-existing Position, Orientation, or Pose target(s) for this \e
      end_effector_link. */
  bool setPoseTargets(const std::vector<geometry_msgs::PoseStamped> &target, const std::string &end_effector_link = "");

  /// Specify which reference frame to assume for poses specified without a reference frame.
  void setPoseReferenceFrame(const std::string &pose_reference_frame);

  /** \brief Specify the parent link of the end-effector.
      This \e end_effector_link will be used in calls to pose target functions
      when end_effector_link is not explicitly specified. */
  bool setEndEffectorLink(const std::string &end_effector_link);

  /** \brief Specify the name of the end-effector to use.
      This is equivalent to setting the EndEffectorLink to the parent link of this end effector. */
  bool setEndEffector(const std::string &eef_name);

  /// Forget pose(s) specified for \e end_effector_link
  void clearPoseTarget(const std::string &end_effector_link = "");

  /// Forget any poses specified for all end-effectors.
  void clearPoseTargets();

  /** Get the currently set pose goal for the end-effector \e end_effector_link.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed.
      If multiple targets are set for \e end_effector_link this will return the first one.
      If no pose target is set for this \e end_effector_link then an empty pose will be returned (check for orientation.xyzw == 0). */
  const geometry_msgs::PoseStamped& getPoseTarget(const std::string &end_effector_link = "") const;

  /** Get the currently set pose goal for the end-effector \e end_effector_link. The pose goal can consist of multiple poses,
      if corresponding setPoseTarget() calls were made. Otherwise, only one pose is returned in the vector.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed  */
  const std::vector<geometry_msgs::PoseStamped>& getPoseTargets(const std::string &end_effector_link = "") const;

  /** \brief Get the current end-effector link.
      This returns the value set by setEndEffectorLink() (or indirectly by setEndEffector()).
      If setEndEffectorLink() was not called, this function reports the link name that serves as parent
      of an end-effector attached to this group. If there are multiple end-effectors, one of them is returned.
      If no such link is known, the empty string is returned. */
  const std::string& getEndEffectorLink() const;

  /** \brief Get the current end-effector name.
      This returns the value set by setEndEffector() (or indirectly by setEndEffectorLink()).
      If setEndEffector() was not called, this function reports an end-effector attached to this group.
      If there are multiple end-effectors, one of them is returned. If no end-effector is known, the empty string is returned. */
  const std::string& getEndEffector() const;

  /** \brief Get the reference frame set by setPoseReferenceFrame(). By default this is the reference frame of the robot model */
  const std::string& getPoseReferenceFrame() const;

  /**@}*/

  /**
   * \name Planning a path from the start position to the Target (goal) position, and executing that plan.
   */
  /**@{*/

  /** \brief Plan and execute a trajectory that takes the group of joints declared in the constructor to the specified target.
      This call is not blocking (does not wait for the execution of the trajectory to complete). */
  MoveItErrorCode asyncMove();

  /** \brief Plan and execute a trajectory that takes the group of joints declared in the constructor to the specified target.
      This call is always blocking (waits for the execution of the trajectory to complete) and requires an asynchronous spinner to be started.*/
  MoveItErrorCode move();

  /** \brief Compute a motion plan that takes the group declared in the constructor from the current state to the specified
      target. No execution is performed. The resulting plan is stored in \e plan*/
  MoveItErrorCode plan(Plan &plan);

  /** \brief Given a \e plan, execute it without waiting for completion. Return true on success. */
  MoveItErrorCode asyncExecute(const Plan &plan);

  /** \brief Given a \e plan, execute it while waiting for completion. Return true on success. */
  MoveItErrorCode execute(const Plan &plan);

  /** \brief Compute a Cartesian path that follows specified waypoints with a step size of at most \e eef_step meters
      between end effector configurations of consecutive points in the result \e trajectory. The reference frame for the
      waypoints is that specified by setPoseReferenceFrame(). No more than \e jump_threshold
      is allowed as change in distance in the configuration space of the robot (this is to prevent 'jumps' in IK solutions).
      Collisions are avoided if \e avoid_collisions is set to true. If collisions cannot be avoided, the function fails.
      Return a value that is between 0.0 and 1.0 indicating the fraction of the path achieved as described by the waypoints.
      Return -1.0 in case of error. */
  double computeCartesianPath(const std::vector<geometry_msgs::Pose> &waypoints, double eef_step, double jump_threshold,
                              moveit_msgs::RobotTrajectory &trajectory, bool avoid_collisions = true, moveit_msgs::MoveItErrorCodes *error_code = NULL);

  /** \brief Compute a Cartesian path that follows specified waypoints with a step size of at most \e eef_step meters
      between end effector configurations of consecutive points in the result \e trajectory. The reference frame for the
      waypoints is that specified by setPoseReferenceFrame(). No more than \e jump_threshold
      is allowed as change in distance in the configuration space of the robot (this is to prevent 'jumps' in IK solutions).
      Kinematic constraints for the path given by \e path_constraints will be met for every point along the trajectory, if they are not met, a partial solution will be returned.
      Constraints are checked (collision and kinematic) if \e avoid_collisions is set to true. If constraints cannot be met, the function fails.
      Return a value that is between 0.0 and 1.0 indicating the fraction of the path achieved as described by the waypoints.
      Return -1.0 in case of error. */
  double computeCartesianPath(const std::vector<geometry_msgs::Pose> &waypoints, double eef_step, double jump_threshold,
                              moveit_msgs::RobotTrajectory &trajectory, const moveit_msgs::Constraints &path_constraints, bool avoid_collisions = true, moveit_msgs::MoveItErrorCodes *error_code = NULL);

  /** \brief Stop any trajectory execution, if one is active */
  void stop();

  /** \brief Specify whether the robot is allowed to look around before moving if it determines it should (default is true) */
  void allowLooking(bool flag);

  /** \brief Specify whether the robot is allowed to replan if it detects changes in the environment */
  void allowReplanning(bool flag);

  /**@}*/

  /**
   * \name High level actions that trigger a sequence of plans and actions.
   */
  /**@{*/

  /** \brief Pick up an object */
  MoveItErrorCode pick(const std::string &object);

  /** \brief Pick up an object given a grasp pose */
  MoveItErrorCode pick(const std::string &object, const moveit_msgs::Grasp &grasp);

  /** \brief Pick up an object given possible grasp poses */
  MoveItErrorCode pick(const std::string &object, const std::vector<moveit_msgs::Grasp> &grasps);

  /** \brief Place an object somewhere safe in the world (a safe location will be detected) */
  MoveItErrorCode place(const std::string &object);

  /** \brief Place an object at one of the specified possible locations */
  MoveItErrorCode place(const std::string &object, const std::vector<moveit_msgs::PlaceLocation> &locations);

  /** \brief Place an object at one of the specified possible locations */
  MoveItErrorCode place(const std::string &object, const std::vector<geometry_msgs::PoseStamped> &poses);

  /** \brief Place an object at one of the specified possible location */
  MoveItErrorCode place(const std::string &object, const geometry_msgs::PoseStamped &pose);

  /** \brief Given the name of an object in the planning scene, make
      the object attached to a link of the robot.  If no link name is
      specified, the end-effector is used. If there is no
      end-effector, the first link in the group is used. If the object
      name does not exist an error will be produced in move_group, but
      the request made by this interface will succeed. */
  bool attachObject(const std::string &object, const std::string &link = "");

  /** \brief Given the name of an object in the planning scene, make
      the object attached to a link of the robot. The set of links the
      object is allowed to touch without considering that a collision
      is specified by \e touch_links.  If \e link is empty, the
      end-effector link is used. If there is no end-effector, the
      first link in the group is used. If the object name does not
      exist an error will be produced in move_group, but the request
      made by this interface will succeed. */
  bool attachObject(const std::string &object, const std::string &link, const std::vector<std::string> &touch_links);

  /** \brief Detach an object. \e name specifies the name of the
      object attached to this group, or the name of the link the
      object is attached to. If there is no name specified, and there
      is only one attached object, that object is detached. An error
      is produced if no object to detach is identified. */
  bool detachObject(const std::string &name = "");

  /**@}*/

  /**
   * \name Query current robot state
   */
  /**@{*/

  /** \brief When reasoning about the current state of a robot, a
      CurrentStateMonitor instance is automatically constructed.  This
      function allows triggering the construction of that object from
      the beginning, so that future calls to functions such as
      getCurrentState() will not take so long and are less likely to fail. */
  bool startStateMonitor(double wait = 1.0);

  /** \brief Get the current joint values for the joints planned for by this instance (see getJoints()) */
  std::vector<double> getCurrentJointValues();

  /** \brief Get the current state of the robot */
  robot_state::RobotStatePtr getCurrentState();

  /** \brief Get the pose for the end-effector \e end_effector_link.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed */
  geometry_msgs::PoseStamped getCurrentPose(const std::string &end_effector_link = "");

  /** \brief Get the roll-pitch-yaw (XYZ) for the end-effector \e end_effector_link.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed */
  std::vector<double> getCurrentRPY(const std::string &end_effector_link = "");

  /** \brief Get random joint values for the joints planned for by this instance (see getJoints()) */
  std::vector<double> getRandomJointValues();

  /** \brief Get a random reachable pose for the end-effector \e end_effector_link.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed */
  geometry_msgs::PoseStamped getRandomPose(const std::string &end_effector_link = "");

  /**@}*/

  /**
   * \name Manage named joint configurations
   */
  /**@{*/

  /** \brief Remember the current joint values (of the robot being monitored) under \e name.
      These can be used by setNamedTarget().
      These values are remembered locally in the client.  Other clients will
      not have access to them. */
  void rememberJointValues(const std::string &name);

  /** \brief Remember the specified joint values  under \e name.
      These can be used by setNamedTarget().
      These values are remembered locally in the client.  Other clients will
      not have access to them. */
  void rememberJointValues(const std::string &name, const std::vector<double> &values);

  /** \brief Get the currently remembered map of names to joint values. */
  const std::map<std::string, std::vector<double> >& getRememberedJointValues() const
  {
    return remembered_joint_values_;
  }

  /** \brief Forget the joint values remebered under \e name */
  void forgetJointValues(const std::string &name);

  /**@}*/

  /**
   * \name Manage planning constraints
   */
  /**@{*/

  /** \brief Specify where the database server that holds known constraints resides */
  void setConstraintsDatabase(const std::string &host, unsigned int port);

  /** \brief Get the names of the known constraints as read from the Mongo database, if a connection was achieved. */
  std::vector<std::string> getKnownConstraints() const;

  /** \brief Get the actual set of constraints for this MoveGroup.
      @return A copy of the current path constraints set for this move_group
      */
  moveit_msgs::Constraints getPathConstraints() const;

  /** \brief Specify a set of path constraints to use.
      The constraints are looked up by name from the Mongo database server.
      This replaces any path constraints set in previous calls to setPathConstraints(). */
  bool setPathConstraints(const std::string &constraint);

  /** \brief Specify a set of path constraints to use.
      This version does not require a database server.
      This replaces any path constraints set in previous calls to setPathConstraints(). */
  void setPathConstraints(const moveit_msgs::Constraints &constraint);

  /** \brief Specify that no path constraints are to be used.
      This removes any path constraints set in previous calls to setPathConstraints(). */
  void clearPathConstraints();

  /**@}*/

private:

  std::map<std::string, std::vector<double> > remembered_joint_values_;
  class MoveGroupImpl;
  MoveGroupImpl *impl_;

};

}

}

#endif
