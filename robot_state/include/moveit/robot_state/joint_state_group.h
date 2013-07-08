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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef MOVEIT_ROBOT_STATE_JOINT_STATE_GROUP_
#define MOVEIT_ROBOT_STATE_JOINT_STATE_GROUP_

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/link_state.h>
#include <moveit/robot_state/joint_state.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <boost/scoped_ptr.hpp>
#include <boost/function.hpp>

namespace robot_state
{

class JointStateGroup;

typedef boost::function<bool(JointStateGroup *joint_state_group, const std::vector<double> &joint_group_variable_values)> StateValidityCallbackFn;
typedef boost::function<bool(const JointStateGroup *joint_state_group, Eigen::VectorXd &stvector)> SecondaryTaskFn;

class RobotState;

/** @class JointStateGroup
 *  @brief The joint state corresponding to a group
 */
class JointStateGroup
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   *  @brief Default constructor
   *  @param state A pointer to the kinematic state
   *  @param jmg The joint model group corresponding to this joint state
   */
  JointStateGroup(RobotState *state, const robot_model::JointModelGroup *jmg);
  ~JointStateGroup();

  /** \brief Get the kinematic state this link is part of */
  const RobotState *getRobotState() const
  {
    return kinematic_state_;
  }

  /** \brief Get the kinematic state this link is part of */
  RobotState *getRobotState()
  {
    return kinematic_state_;
  }

  /** \brief Get the joint model corresponding to this joint state group */
  const robot_model::JointModelGroup* getJointModelGroup() const
  {
    return joint_model_group_;
  }

  /** \brief Get the name of the joint model group corresponding to this joint state*/
  const std::string& getName() const
  {
    return joint_model_group_->getName();
  }

  /** \brief Get the number of (active) DOFs for the joint model group corresponding to this state*/
  unsigned int getVariableCount() const
  {
    return joint_model_group_->getVariableCount();
  }

  /** \brief Perform forward kinematics starting at the roots
      within a group. Links that are not in the group are also
      updated, but transforms for joints that are not in the
      group are not recomputed.  */
  bool setVariableValues(const std::vector<double>& joint_state_values);

  /** \brief Perform forward kinematics starting at the roots
      within a group. Links that are not in the group are also
      updated, but transforms for joints that are not in the
      group are not recomputed.
      \warning Creates a temporary std::vector<double> */
  bool setVariableValues(const Eigen::VectorXd& joint_state_values);

  /** \brief Perform forward kinematics starting at the roots
      within a group. Links that are not in the group are also
      updated, but transforms for joints that are not in the
      group are not recomputed.  */
  void setVariableValues(const std::map<std::string, double>& joint_state_map);

  /** \brief Perform forward kinematics starting at the roots
      within a group. Links that are not in the group are also
      updated, but transforms for joints that are not in the
      group are not recomputed.  */
  void setVariableValues(const sensor_msgs::JointState& js);

  /** Compute transforms using current joint values */
  void updateLinkTransforms();

  /** \brief Check if a joint is part of this group */
  bool hasJointState(const std::string &joint) const;

  /** \brief Get a joint state by its name */
  JointState* getJointState(const std::string &joint) const;

  /** \brief Get current joint values */
  void getVariableValues(std::vector<double>& joint_state_values) const;

  /** \brief Get current joint values */
  void getVariableValues(Eigen::VectorXd& joint_state_values) const;

  /** \brief Get a map between variable names and joint state values */
  void getVariableValues(std::map<std::string, double>& joint_state_values) const;

  /** \brief Bring the group to a default state. All joints are
      at 0. If 0 is not within the bounds of the joint, the
      middle of the bounds is used. */
  void setToDefaultValues();

  /** \brief Set the group to a named default state. Return false on failure */
  bool setToDefaultState(const std::string &name);

  /** \brief Sample a random state in accordance with the type of joints employed */
  void setToRandomValues();

  /** \brief Sample a random state in accordance with
      the type of joints employed, near the specified joint state.
      The distance map specifies distances according to joint type. */
  void setToRandomValuesNearBy(const std::vector<double> &near, const std::map<robot_model::JointModel::JointType, double> &distance_map);

  /** \brief Sample a random state in accordance with
      the type of joints employed, near the specified joint state.
      The distances vector specifies a distance for each joint model.
      The distance computation uses an InfinityNorm computation
      - see also infinityNormDistance() */
  void setToRandomValuesNearBy(const std::vector<double> &near, const std::vector<double> &distances);

  /** \brief Checks if the current joint state values are all within the bounds set in the model */
  bool satisfiesBounds(double margin = 0.0) const;

  /** \brief Force the joint to be inside bounds and normalized. Quaternions are normalized, continuous joints are made between -Pi and Pi. */
  void enforceBounds();

  /** \brief Get the infinity norm distance between two joint states */
  double infinityNormDistance(const JointStateGroup *other) const;

  double distance(const JointStateGroup *other) const;

  /** \brief Returns the minimum distance of a joint to the joint limits for the group.
      Will not consider planar joints if they are not bounded in all DOFs.
      If planar joints are bounded, this function will use the distance
      function defined for planar joints.
      This function will not consider floating joints at all.
      @return A std::pair with the required distance and index of joint corresponding to distance
  */
  std::pair<double, int> getMinDistanceToBounds() const;

  void interpolate(const JointStateGroup *to, const double t, JointStateGroup *dest) const;

  /** \brief Get the state corresponding to root joints in this group*/
  const std::vector<JointState*>& getJointRoots() const
  {
    return joint_roots_;
  }

  /** \brief Get the joint names corresponding to this joint state*/
  const std::vector<std::string>& getJointNames() const
  {
    return joint_model_group_->getJointModelNames();
  }

  /** \brief Get the vector of joint state for this group*/
  const std::vector<JointState*>& getJointStateVector() const
  {
    return joint_state_vector_;
  }

  /** \brief Return the instance of a random number generator */
  random_numbers::RandomNumberGenerator& getRandomNumberGenerator();

  /** \brief Given a set of joint angles, compute the jacobian with reference to a particular point on a given link
   * \param link_name The name of the link
   * \param reference_point_position The reference point position (with respect to the link specified in link_name)
   * \param jacobian The resultant jacobian
   * \param use_quaternion_representation Flag indicating if the Jacobian should use a quaternion representation (default is false)
   * \return True if jacobian was successfully computed, false otherwise
   */
  bool getJacobian(const std::string &link_name, const Eigen::Vector3d &reference_point_position, Eigen::MatrixXd& jacobian,
                   bool use_quaterion_representation = false) const;

  /** \brief Get the default IK timeout */
  double getDefaultIKTimeout() const
  {
    return joint_model_group_->getDefaultIKTimeout();
  }

  /** \brief Get the default number of ik attempts */
  unsigned getDefaultIKAttempts() const
  {
    return joint_model_group_->getDefaultIKAttempts();
  }

  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be set by computing inverse kinematics.
      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
      @param pose The pose the \e tip  link in the chain needs to achieve
      @param tip The name of the link the pose is specified for
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIK(const geometry_msgs::Pose &pose, const std::string &tip, unsigned int attempts = 0, double timeout = 0.0,
                 const StateValidityCallbackFn &constraint = StateValidityCallbackFn(), const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions());

  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be set by computing inverse kinematics.
      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
      @param pose The pose the last link in the chain needs to achieve
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt */
  bool setFromIK(const geometry_msgs::Pose &pose, unsigned int attempts = 0, double timeout = 0.0, const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions());

  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be set by computing inverse kinematics.
      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
      @param pose The pose the last link in the chain needs to achieve
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIK(const geometry_msgs::Pose &pose, unsigned int attempts = 0, double timeout = 0.0, const StateValidityCallbackFn &constraint =  StateValidityCallbackFn(), const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions());

  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be set by computing inverse kinematics.
      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
      @param pose The pose the last link in the chain needs to achieve
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIK(const Eigen::Affine3d &pose, unsigned int attempts = 0, double timeout = 0.0,
                 const StateValidityCallbackFn &constraint = StateValidityCallbackFn(), const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions());

  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be set by computing inverse kinematics.
      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
      @param pose The pose the last link in the chain needs to achieve
      @param tip The name of the link the pose is specified for
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt */
  bool setFromIK(const Eigen::Affine3d &pose_in, const std::string &tip_in, unsigned int attempts, double timeout, const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions());

  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be set by computing inverse kinematics.
      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
      @param pose The pose the last link in the chain needs to achieve
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt */
  bool setFromIK(const Eigen::Affine3d &pose_in, unsigned int attempts, double timeout, const kinematics::KinematicsQueryOptions &options);

  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be set by computing inverse kinematics.
      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
      @param pose The pose the last link in the chain needs to achieve
      @param tip The name of the frame for which IK is attempted.
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIK(const Eigen::Affine3d &pose, const std::string &tip, unsigned int attempts = 0, double timeout = 0.0,
                 const StateValidityCallbackFn &constraint = StateValidityCallbackFn(), const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions());

  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be set by computing inverse kinematics.
      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
      @param pose The pose the last link in the chain needs to achieve
      @param tip The name of the frame for which IK is attempted.
      @param consistency_limits This specifies the desired distance between the solution and the seed state
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIK(const Eigen::Affine3d &pose, const std::string &tip,
                 const std::vector<double> &consistency_limits, unsigned int attempts = 0, double timeout = 0.0,
                 const StateValidityCallbackFn &constraint = StateValidityCallbackFn(), const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions());

  /** \brief  Warning: This function inefficiently copies all transforms around.
      If the group consists of a set of sub-groups that are each a chain and a solver
      is available for each sub-group, then the joint values can be set by computing inverse kinematics.
      The poses are assumed to be in the reference frame of the kinematic model. The poses are assumed
      to be in the same order as the order of the sub-groups in this group. Returns true on success.
      @param poses The poses the last link in each chain needs to achieve
      @param tips The names of the frames for which IK is attempted.
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIK(const EigenSTL::vector_Affine3d &poses, const std::vector<std::string> &tips, unsigned int attempts = 0, double timeout = 0.0, const StateValidityCallbackFn &constraint = StateValidityCallbackFn(), const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions());

  /** \brief Warning: This function inefficiently copies all transforms around.
      If the group consists of a set of sub-groups that are each a chain and a solver
      is available for each sub-group, then the joint values can be set by computing inverse kinematics.
      The poses are assumed to be in the reference frame of the kinematic model. The poses are assumed
      to be in the same order as the order of the sub-groups in this group. Returns true on success.
      @param poses The poses the last link in each chain needs to achieve
      @param tips The names of the frames for which IK is attempted.
      @param consistency_limits This specifies the desired distance between the solution and the seed state
      @param attempts The number of times IK is attempted
      @param timeout The timeout passed to the kinematics solver on each attempt
      @param constraint A state validity constraint to be required for IK solutions */
  bool setFromIK(const EigenSTL::vector_Affine3d &poses, const std::vector<std::string> &tips, const std::vector<std::vector<double> > &consistency_limits, unsigned int attempts = 0, double timeout = 0.0, const StateValidityCallbackFn &constraint = StateValidityCallbackFn(), const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions());

  /** \brief Set the joint values from a cartesian velocity applied during a time dt
   * @param twist a cartesian velocity on the 'tip' frame
   * @param tip the frame for which the twist is given
   * @param dt a time interval (seconds)
   * @param st a secondary task computation function
   */
  bool setFromDiffIK(const Eigen::VectorXd &twist, const std::string &tip, double dt, const StateValidityCallbackFn &constraint = StateValidityCallbackFn(), const SecondaryTaskFn &st = SecondaryTaskFn());

  /** \brief Set the joint values from a cartesian velocity applied during a time dt
   * @param twist a cartesian velocity on the 'tip' frame
   * @param tip the frame for which the twist is given
   * @param dt a time interval (seconds)
   * @param st a secondary task computation function
   */
  bool setFromDiffIK(const geometry_msgs::Twist &twist, const std::string &tip, double dt, const StateValidityCallbackFn &constraint = StateValidityCallbackFn(), const SecondaryTaskFn &st = SecondaryTaskFn());

  /** \brief Given a twist for a particular link (\e tip), and an optional secondary task (\e st), compute the corresponding joint velocity and store it in \e qdot */
  void computeJointVelocity(Eigen::VectorXd &qdot, const Eigen::VectorXd &twist, const std::string &tip, const SecondaryTaskFn &st = SecondaryTaskFn()) const;

  /** \brief Given the velocities for the joints in this group (\e qdot) and an amount of time (\e dt), update the current state using the Euler forward method.
      If the constraint specified is satisfied, return true, otherwise return false. */
  bool integrateJointVelocity(const Eigen::VectorXd &qdot, double dt, const StateValidityCallbackFn &constraint = StateValidityCallbackFn());

  /** \brief Secondary task that tries to keep away from joint limits THIS FUNCTION NEEDS TO MOVE ELSEWHERE
   * @param joint_state_group the joint state group for which to compute the task
   * @param stvector the output of the function: a vector with joint velocities
   * @param activation_threshold A percentage of the range from which the task is activated, i.e. activate if q > qmax - range * threshold. Typically between 0 and 0.5
   * @param gain a gain for this task, multiplies the output velocities
   */
  bool avoidJointLimitsSecondaryTask(const JointStateGroup *joint_state_group, Eigen::VectorXd &stvector,
                                     double activation_threshold, double gain) const;

  /** \brief Compute the sequence of joint values that correspond to a straight Cartesian path.

      The Cartesian path to be followed is specified as a direction of motion (\e direction, unit vector) for the origin of a robot
      link (\e link_name). The direction is assumed to be either in a global reference frame or in the local reference frame of the
      link. In the latter case (\e global_reference_frame is false) the \e direction is rotated accordingly. The link needs to move in a
      straight line, following the specified direction, for the desired \e distance. The resulting joint values are stored in
      the vector \e traj, one by one. The maximum distance in Cartesian space between consecutive points on the resulting path
      is specified by \e max_step.  If a \e validCallback is specified, this is passed to the internal call to
      setFromIK(). In case of IK failure, the computation of the path stops and the value returned corresponds to the distance that
      was computed and for which corresponding states were added to the path.  At the end of the function call, the state of the
      group corresponds to the last attempted Cartesian pose.  During the computation of the trajectory, it is sometimes preferred if
      consecutive joint values do not 'jump' by a large amount in joint space, even if the Cartesian distance between the
      corresponding points is as expected. To account for this, the \e jump_threshold parameter is provided.  As the joint values
      corresponding to the Cartesian path are computed, distances in joint space between consecutive points are also computed. Once
      the sequence of joint values is computed, the average distance between consecutive points (in joint space) is also computed. It
      is then verified that none of the computed distances is above the average distance by a factor larger than \e jump_threshold. If
      a point in joint is found such that it is further away than the previous one by more than average_consecutive_distance * \e jump_threshold,
      that is considered a failure and the returned path is truncated up to just before the jump. The jump detection can be disabled
      by setting \e jump_threshold to 0.0*/
  double computeCartesianPath(std::vector<boost::shared_ptr<RobotState> > &traj, const std::string &link_name, const Eigen::Vector3d &direction, bool global_reference_frame,
                              double distance, double max_step, double jump_threshold, const StateValidityCallbackFn &validCallback = StateValidityCallbackFn(), const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions());

  /** \brief Compute the sequence of joint values that correspond to a straight Cartesian path.

      The Cartesian path to be followed is specified as a target frame to be reached (\e target) for the origin of a robot
      link (\e link_name). The target frame is assumed to be either in a global reference frame or in the local reference frame of the
      link. In the latter case (\e global_reference_frame is false) the \e target is rotated accordingly. The link needs to move in a
      straight line towards the target. The resulting joint values are stored in
      the vector \e traj, one by one. The maximum distance in Cartesian space between consecutive points on the resulting path
      is specified by \e max_step.  If a \e validCallback is specified, this is passed to the internal call to
      setFromIK(). In case of IK failure, the computation of the path stops and the value returned corresponds to the percentage of the
      path (between 0 and 1) that was completed and for which corresponding states were added to the path.  At the end of the function call,
      the state of the group corresponds to the last attempted Cartesian pose.  During the computation of the trajectory, it is sometimes preferred if
      consecutive joint values do not 'jump' by a large amount in joint space, even if the Cartesian distance between the
      corresponding points is as expected. To account for this, the \e jump_threshold parameter is provided.  As the joint values
      corresponding to the Cartesian path are computed, distances in joint space between consecutive points are also computed. Once
      the sequence of joint values is computed, the average distance between consecutive points (in joint space) is also computed. It
      is then verified that none of the computed distances is above the average distance by a factor larger than \e jump_threshold. If
      a point in joint is found such that it is further away than the previous one by more than average_consecutive_distance * \e jump_threshold,
      that is considered a failure and the returned path is truncated up to just before the jump. The jump detection can be disabled
      by setting \e jump_threshold to 0.0*/
  double computeCartesianPath(std::vector<boost::shared_ptr<RobotState> > &traj, const std::string &link_name, const Eigen::Affine3d &target, bool global_reference_frame,
                              double max_step, double jump_threshold, const StateValidityCallbackFn &validCallback = StateValidityCallbackFn(), const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions());

  /** \brief Compute the sequence of joint values that perform a general Cartesian path.

      The Cartesian path to be followed is specified as a set of \e waypoints to be sequentially reached for the origin of a robot
      link (\e link_name). The waypoints are transforms given either in a global reference frame or in the local reference frame of the
      link at the immediately preceeding waypoint. The link needs to move in a straight line between two consecutive waypoints.
      The resulting joint values are stored in the vector \e traj, one by one. The maximum distance in Cartesian space between
      consecutive points on the resulting path is specified by \e max_step.  If a \e validCallback is specified, this is passed to the
      internal call to setFromIK(). In case of IK failure, the computation of the path stops and the value returned corresponds to the
      percentage of the path (between 0 and 1) that was completed and for which corresponding states were added to the path.  At the end
      of the function call, the state of the group corresponds to the last attempted Cartesian pose.  During the computation of the
      trajectory, it is sometimes preferred if consecutive joint values do not 'jump' by a large amount in joint space, even if the
      Cartesian distance between the corresponding points is as expected. To account for this, the \e jump_threshold parameter is
      provided.  As the joint values corresponding to the Cartesian path are computed, distances in joint space between consecutive
      points are also computed. Once the sequence of joint values is computed, the average distance between consecutive points (in
      joint space) is also computed. It is then verified that none of the computed distances is above the average distance by a
      factor larger than \e jump_threshold. If a point in joint is found such that it is further away than the previous one by more
      than average_consecutive_distance * \e jump_threshold, that is considered a failure and the returned path is truncated up to
      just before the jump. The jump detection can be disabled by setting \e jump_threshold to 0.0*/
  double computeCartesianPath(std::vector<boost::shared_ptr<RobotState> > &traj, const std::string &link_name, const EigenSTL::vector_Affine3d &waypoints,
                              bool global_reference_frame, double max_step, double jump_threshold, const StateValidityCallbackFn &validCallback = StateValidityCallbackFn(), const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions());

  JointStateGroup& operator=(const JointStateGroup &other);

private:

  /** \brief Copy the values from another joint state group */
  void copyFrom(const JointStateGroup &other_jsg);

  /** \brief This function converts output from the IK plugin to the proper ordering of values expected by this group and passes it to \e constraint */
  void ikCallbackFnAdapter(const StateValidityCallbackFn &constraint, const geometry_msgs::Pose &ik_pose,
                           const std::vector<double> &ik_sol, moveit_msgs::MoveItErrorCodes &error_code);

  /** \brief The kinematic state this group is part of */
  RobotState *kinematic_state_;

  /** \brief The model of the group that corresponds to this state */
  const robot_model::JointModelGroup     *joint_model_group_;

  /** \brief Joint instances in the order they appear in the group state */
  std::vector<JointState*>                joint_state_vector_;

  /** \brief A map from joint names to their instances */
  std::map<std::string, JointState*>      joint_state_map_;

  /** \brief The list of joints that are roots in this group */
  std::vector<JointState*>                joint_roots_;

  /** \brief The list of links that are updated when computeTransforms() is called, in the order they are updated */
  std::vector<LinkState*>                 updated_links_;

  /** \brief For certain operations a group needs a random number generator. However, it may be slightly expensive
      to allocate the random number generator if many state instances are generated. For this reason, the generator
      is allocated on a need basis, by the getRandomNumberGenerator() function. Never use the rng_ member directly, but call
      getRandomNumberGenerator() instead. */
  boost::scoped_ptr<random_numbers::RandomNumberGenerator> rng_;

};
}

#endif
