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

#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/PlannerInterfaceDescription.h>
#include <moveit_msgs/Constraints.h>
#include <manipulation_msgs/Grasp.h>
#include <manipulation_msgs/PlaceLocation.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/shared_ptr.hpp>
#include <tf/tf.h>

/** \brief Simple interface to the MoveGroup action */
namespace move_group_interface
{

/** \brief Client class for the MoveGroup action. This class includes many default settings to make things easy to use. */
class MoveGroup
{
public:
  
  /** \brief Default ROS parameter name from where to read the robot's URDF. Set to 'robot_description' */
  static const std::string ROBOT_DESCRIPTION;
  
  /** \brief Default ROS topic from where to read the robot's joint states. Set to 'joint_states' */
  static const std::string JOINT_STATE_TOPIC;

  /** \brief Specification of options to use when constructing the MoveGroup client class */
  struct Options
  {
    Options(const std::string &group_name) : group_name_(group_name),
                                             joint_state_topic_(JOINT_STATE_TOPIC),
                                             robot_description_(ROBOT_DESCRIPTION)
    {
    }

    /// The group to construct the class instance for
    std::string group_name_;

    /// The joint states topic (if different from default)
    std::string joint_state_topic_;

    /// The robot description parameter name (if different from default)
    std::string robot_description_;

    /// Optionally, an instance of the RobotModel to use can be also specified
    robot_model::RobotModelConstPtr kinematic_model_;
  };

  /// The representation of a motion plan (as ROS messasges)
  struct Plan
  {
    /// The full starting state used for planning
    moveit_msgs::RobotState start_state_;

    /// The trajectory of the robot (may not contain joints that are the same as for the start_state_)
    moveit_msgs::RobotTrajectory trajectory_;
  };
  
  /** \brief Construct a client for the MoveGroup action using a specified set of options \e opt. Optionally, specify a TF instance to use. 
      If not specified, one will be constructed internally. A timeout for connecting to the action server can also be specified. If it is not specified, 
      the wait time is unlimited. */
  MoveGroup(const Options &opt, const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>(),
            const ros::Duration &wait_for_server = ros::Duration(0, 0)); 

  /** \brief Construct a client for the MoveGroup action for a particular \e group. Optionally, specify a TF instance to use. 
      If not specified, one will be constructed internally. A timeout for connecting to the action server can also be specified. If it is not specified, 
      the wait time is unlimited. */
  MoveGroup(const std::string &group, const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>(),
            const ros::Duration &wait_for_server = ros::Duration(0, 0));
  
  ~MoveGroup();
  
  /** \brief Get the name of the group this instance operates on */
  const std::string& getName() const;
  
  /** \brief Get the joints this instance operates on */
  const std::vector<std::string>& getJoints() const;

  /** \brief Get the description of the planning plugin loaded by the action server */
  bool getInterfaceDescription(moveit_msgs::PlannerInterfaceDescription &desc);
  
  /** \brief Plan and execute a trajectory that takes the group of joints declared in the constructor to the specified target.
      This call is not blocking (does not wait for the execution of the trajectory to complete). */
  bool asyncMove();
  
  /** \brief Plan and execute a trajectory that takes the group of joints declared in the constructor to the specified target.
      This call is always blocking (waits for the execution of the trajectory to complete). */
  bool move();

  /** \brief Compute a motion plan that takes the group declared in the constructor from the current state to the specified
      target. No execution is performed. The resulting plan is stored in \e plan*/
  bool plan(Plan &plan);
  
  /** \brief Given a \e plan, execute it without waiting for completion. Return true on success. */
  bool asyncExecute(const Plan &plan);
  
  /** \brief Given a \e plan, execute it while waiting for completion. Return true on success. */
  bool execute(const Plan &plan);

  /** \brief Pick up an object */
  bool pick(const std::string &object);

  /** \brief Pick up an object given possible grasp poses */
  bool pick(const std::string &object, const std::vector<manipulation_msgs::Grasp> &grasps);

  /** \brief Place an object somewhere safe in the world (a safe location will be detected) */
  bool place(const std::string &object);

  /** \brief Place an object at one of the specified possible locations */
  bool place(const std::string &object, const std::vector<manipulation_msgs::PlaceLocation> &locations);
  
  /** \brief Stop any trajectory execution, if one is active */
  void stop();
  
  /** \brief Specify whether the robot is allowed to look around before moving if it determines it should (default is true) */
  void allowLooking(bool flag);

  /** \brief Specify whether the robot is allowed to replan if it detects changes in the environment */
  void allowReplanning(bool flag);
  
  /** \brief Specify a planner name to be used for further planning */
  void setPlannerId(const std::string &planner_id);

  /** \brief Specify the maximum amount of time to use when planning */
  void setPlanningTime(double seconds);

  /** \brief Get the tolerance that is used for reaching the goal. For
      joint state goals, this will be distance for each joint, in the
      configuration space. For pose goals this will be the side of a
      cube where the end-effector must reach.*/
  double getGoalTolerance() const;
  
  /** \brief Set the tolerance that is used for reaching the goal. For
      joint state goals, this will be distance for each joint, in the
      configuration space. For pose goals this will be the side of a
      cube where the end-effector must reach.*/
  void setGoalTolerance(double tolerance);

  /** \brief Specify the workspace bounding box */
  void setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz);
  
  /** \brief If a different start state should be considered instead of the current state of the robot, this function sets that state */
  void setStartState(const robot_state::RobotState &start_state);
  
  /** \brief Set the starting state for planning to be that reported by the robot's joint state publication */
  void setStartStateToCurrentState();

  /** \brief For pick/place operations, the name of the support surface is used to specify the fact that attached objects are allowed to touch the support surface */
  void setSupportSurfaceName(const std::string &name);
  
  /**
   * \defgroup set_joint_goal Setting a joint state goal
   */
  /**@{*/

  /** \brief Given a vector of real values in the same order as expected by the group, set those as the joint state goal */
  void setJointValueTarget(const std::vector<double> &group_variable_values);

  /** \brief Given a map of joint names to real values, set those as the joint state goal */
  void setJointValueTarget(const std::map<std::string, double> &variable_values);

  /** \brief Set the joint state goal from corresponding joint values from the specified state */
  void setJointValueTarget(const robot_state::RobotState &kinematic_state);

  /** \brief Set the joint state goal from corresponding joint values from the specified state */
  void setJointValueTarget(const robot_state::JointStateGroup &joint_state_group);
  
  /** \brief Set the joint state goal for a particula joint */
  void setJointValueTarget(const robot_state::JointState &joint_state);

  /** \brief Set the joint state goal for a particular joint */
  void setJointValueTarget(const std::string &joint_name, const std::vector<double> &values);

  /** \brief Set the joint state goal for a particular joint */
  void setJointValueTarget(const std::string &joint_name, double value);
  
  /** \brief Set the joint state goal for a particular joint */
  void setJointValueTarget(const sensor_msgs::JointState &state);

  /** \brief Set the joint state goal to a random joint configuration */
  void setRandomTarget();
  
  /** \brief Set the current joint values to be ones previously remembered by rememberJointValues() or, if not found,
      that are specified in the SRDF under the name \e name as a group state*/
  bool setNamedTarget(const std::string &name);

  /// Get the currently set joint state goal
  const robot_state::JointStateGroup& getJointValueTarget() const;

  /**@}*/


  /**
   * \defgroup set_pose_goal Setting a pose goal
   */
  /**@{*/

  /** \brief Set the goal position of the end-effector \e end_effector_link to be (\e x, \e y, \e z). If \e end_effector_link
      is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed */
  void setPositionTarget(double x, double y, double z, const std::string &end_effector_link = "");
  
  /** \brief Set the goal orientation of the end-effector \e end_effector_link to be (\e x,\e y,\e z) radians about the (X,
      Y, Z) axes. If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed  */
  void setOrientationTarget(double x, double y, double z, const std::string &end_effector_link = "");

  /** \brief Set the goal orientation of the end-effector \e end_effector_link to be the quaternion (\e x,\e y,\e z,\e w).
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed  */
  void setOrientationTarget(double x, double y, double z, double w, const std::string &end_effector_link = "");
  
  /** \brief Set the goal pose of the end-effector \e end_effector_link.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed  */
  void setPoseTarget(const Eigen::Affine3d &end_effector_pose, const std::string &end_effector_link = "");
  
  /** \brief Set the goal pose of the end-effector \e end_effector_link.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed  */
  void setPoseTarget(const geometry_msgs::Pose &target, const std::string &end_effector_link = "");

  /** \brief Set the goal pose of the end-effector \e end_effector_link.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed  */
  void setPoseTarget(const geometry_msgs::PoseStamped &target, const std::string &end_effector_link = "");

  /** \brief Set the goal pose of the end-effector \e end_effector_link. In this case the goal pose can be any of the ones specified in the array.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed  */
  void setPoseTargets(const EigenSTL::vector_Affine3d &end_effector_pose, const std::string &end_effector_link = "");
  
  /** \brief Set the goal pose of the end-effector \e end_effector_link. In this case the goal pose can be any of the ones specified in the array.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed  */
  void setPoseTargets(const std::vector<geometry_msgs::Pose> &target, const std::string &end_effector_link = "");

  /** \brief Set the goal pose of the end-effector \e end_effector_link. In this case the goal pose can be any of the ones specified in the array.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed  */
  void setPoseTargets(const std::vector<geometry_msgs::PoseStamped> &target, const std::string &end_effector_link = "");

  /// Specify which reference frame to assume for poses specified without a reference frame.
  void setPoseReferenceFrame(const std::string &pose_reference_frame);

  /// Specify the link the end-effector to be considered is attached to
  void setEndEffectorLink(const std::string &link_name);

  /// Specify the name of the end-effector to use
  void setEndEffector(const std::string &eef_name);
  
  /// Forget pose specified for the end-effector \e end_effector_link
  void clearPoseTarget(const std::string &end_effector_link = "");

  /// Forget any poses specified for any end-effector
  void clearPoseTargets();

  /** Get the currently set pose goal for the end-effector \e end_effector_link.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed  */
  const geometry_msgs::PoseStamped& getPoseTarget(const std::string &end_effector_link = "") const;

  /** Get the currently set pose goal for the end-effector \e end_effector_link. The pose goal can consist of multiple poses,
      if corresponding setPoseTarget() calls were made. Otherwise, only one pose is returned in the vector.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed  */
  const std::vector<geometry_msgs::PoseStamped>& getPoseTargets(const std::string &end_effector_link = "") const;
  
  /** \brief Get the current end-effector link. This returns the value set by setEndEffectorLink().
      If setEndEffectorLink() was not called, this function reports the link name that serves as parent
      of an end-effector attached to this group. If there are multiple end-effectors, one of them is returned.
      If no such link is known, the empty string is returned. */
  const std::string& getEndEffectorLink() const;

  /** \brief Get the current end-effector name. This returns the value set by setEndEffector().
      If setEndEffector() was not called, this function reports an end-effector attached to this group.
      If there are multiple end-effectors, one of them is returned. If no end-effector is known, the empty string is returned. */
  const std::string& getEndEffector() const;
  
  /** \brief Get the reference frame set by setPoseReferenceFrame(). By default this is the reference frame of the kinematic model */
  const std::string& getPoseReferenceFrame() const;

  /**@}*/


  /**
   * \defgroup follow_existing_traj Follow an existing sequence of constraints
   */
  /**@{*/

  /** \brief Follow the specified sequence of constraints */
  void followConstraints(const std::vector<moveit_msgs::Constraints> &constraints);

  /** \brief Follow a trajectory that takes the specified end-effector link through the specified sequence of poses.
      The tolerance for achieving the position is \e tolerance_pos and the tolerance for achieving the orientation is \e tolerance_angle.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed */
  void followConstraints(const std::vector<geometry_msgs::PoseStamped> &poses,  double tolerance_pos = 1e-3, double tolerance_angle = 1e-2, const std::string &end_effector_link = "");

  /** \brief Follow a trajectory that takes the specified end-effector link through the specified sequence of poses. The
      reference frame for the poses is assumed to be that returned by getPoseReferenceFrame().
      The tolerance for achieving the position is \e tolerance_pos and the tolerance for achieving the orientation is \e tolerance_angle.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed */
  void followConstraints(const std::vector<geometry_msgs::Pose> &poses,  double tolerance_pos = 1e-3, double tolerance_angle = 1e-2, const std::string &end_effector_link = "");
  
  /** \brief Follow a trajectory that takes the specified end-effector link through the specified sequence of poses. The
      reference frame for the poses is assumed to be that returned by getPoseReferenceFrame().
      The tolerance for achieving the position is \e tolerance_pos and the tolerance for achieving the orientation is \e tolerance_angle.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed */
  void followConstraints(const EigenSTL::vector_Affine3d &poses, double tolerance_pos = 1e-3, double tolerance_angle = 1e-2, const std::string &end_effector_link = "");
  
  /**@}*/

  /**
   * \defgroup query_robot_state Query current robot state
   */
  /**@{*/
  
  /** \brief Get the current joint values for the joints planned for by this instance (see getJoints()) */
  std::vector<double> getCurrentJointValues();
  
  /** \brief Get the current state of the robot */
  robot_state::RobotStatePtr getCurrentState();

  /** \brief Get the pose for the end-effector \e end_effector_link. 
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed */
  geometry_msgs::PoseStamped getCurrentPose(const std::string &end_effector_link = "");

  /** \brief Get random joint values for the joints planned for by this instance (see getJoints()) */
  std::vector<double> getRandomJointValues();

  /** \brief Get a random reachable pose for the end-effector \e end_effector_link. 
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed */
  geometry_msgs::PoseStamped getRandomPose(const std::string &end_effector_link = "");

  /**@}*/
  
  /**
   * \defgroup named_joint_goals Manage named joint configurations
   */
  /**@{*/

  /** \brief Remember the current joint values (of the robot being monitored) under \e name. These can be used by setNamedTarget() */
  void rememberJointValues(const std::string &name);

  /** \brief Remember the specified joint values  under \e name. These can be used by setNamedTarget() */
  void rememberJointValues(const std::string &name, const std::vector<double> &values);

  /** \brief Get the currently remembered map of names to joint values */
  const std::map<std::string, std::vector<double> >& getRememberedJointValues() const
  {
    return remembered_joint_values_;
  }
  
  /** \brief Forget the joint values remebered under \e name */
  void forgetJointValues(const std::string &name);
  
  /**@}*/

  /**
   * \defgroup move_group_interface_constraints_management Manage planning constraints
   */
  /**@{*/
  
  /** \brief Specify where the MongoDB server that holds known constraints resides */
  void setConstraintsDatabase(const std::string &host, unsigned int port);
  
  /** \brief Get the names of the constraints known (as read from the warehouse, if a connection was achieved) */
  std::vector<std::string> getKnownConstraints() const;
  
  /** \brief Specify a set of path constraints to use */
  bool setPathConstraints(const std::string &constraint);

  /** \brief Specify a set of path constraints to use */
  void setPathConstraints(const moveit_msgs::Constraints &constraint);

  /** \brief Specify that no path constraints are to be used */
  void clearPathConstraints();

  /**@}*/

private:

  std::map<std::string, std::vector<double> > remembered_joint_values_;
  class MoveGroupImpl;
  MoveGroupImpl *impl_;
  
};

}
#endif
