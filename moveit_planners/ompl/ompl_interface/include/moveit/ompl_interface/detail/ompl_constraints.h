/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Jeroen De Maeyer
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

/* Author: Jeroen De Maeyer */

#pragma once

#include <ompl/base/Constraint.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>

namespace ompl_interface
{
namespace ob = ompl::base;

/** \brief Represents upper and lower bound on a scalar value (double).
 *
 * Equality constraints can be represented by setting
 * the upper bound and lower bound almost equal.
 * I (jeroendm) assume that it is better to not have them exactly equal
 * for numerical reasons. Not sure.
 * **/
struct Bounds
{
  double lower, upper;

  /** \brief Distance to region inside bounds
   *
   * Distance of a given value outside the bounds,
   * zero inside the bounds.
   *
   * Creates a penalty function that looks like this:
   *
   *  \         /
   *   \       /
   *    \_____/
   * (how does ascii art work??)
   * */
  double distance(double value) const;
};

/** \brief Abstract base class for differen types of constraints, implementations of ompl::base::Constraint
 *
 * To create a constrained state space in OMPL, we need a model of the constraints.
 * Any generic model that can be written in the form of equality constraints F(joint_positions) = 0
 * will work. In this code we use bounds on scalar values:
 *    lower_bound < scalar value < upper bound
 * and convert them to equality constraints using the distance function explained in the class Bounds above.
 *
 * The 'scalar value' can be any error in general.
 * For planning in MoveIt, this can be error on the position or orientation of a link relative to a
 * desired reference position.
 * */
class BaseConstraint : public ob::Constraint
{
public:
  BaseConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group, const unsigned int num_dofs,
                 const unsigned int num_cons_ = 3);

  /** \brief initialize constraint based on message content.
   *
   * This is necessary because we cannot call the pure virtual
   * parseConstraintsMsg method from the constructor of this class.
   * */
  void init(const moveit_msgs::Constraints& constraints);

  /** OMPL's main constraint evaluation function.
   *
   *  OMPL requires you to override at least "function" which represents the constraint F(q) = 0
   * */
  virtual void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const;

  /** Optionally you can also provide dF(q)/dq, the Jacobian of  the constriants.
   *
   * */
  virtual void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const;

  /** \brief Wrapper for forward kinematics calculated by MoveIt's Robot State.
   *
   * TODO(jeroendm) Are these actually const, as the robot state is modified? How come it works?
   * Also, output arguments could be significantly more performant,
   * but MoveIt's robot state does not support passing Eigen::Ref objects at the moment.
   * */
  Eigen::Isometry3d forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** \brief Calculate the geometric Jacobian using MoveIt's Robot State.
   *
   * Ideally I would pass the output agrument from OMPL's jacobian function directly,
   * but I cannot pass an object of type , Eigen::Ref<Eigen::MatrixXd> to MoveIt's
   * Jacobian method.
   * */
  Eigen::MatrixXd geometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** \brief Parse bounds on position and orientation parameters from MoveIt's constraint message.
   *
   * This can be non-trivial given the often complex structure of these messages.
   * For the current example with equality constraints it could be to simple
   * to have this separate function instead of using the init function directly.
   * */
  virtual void parseConstraintMsg(const moveit_msgs::Constraints& constraints) = 0;

  /** \brief For inequality constraints: calculate the value of the parameter that is being constraint by the bounds.
   *
   * In this Position constraints case, it calculates the x, y and z position
   * of the end-effector. This error is then converted in generic equality constraints in the implementation of
   * BaseConstraint::function.
   * */
  virtual Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
  {
    ROS_ERROR_STREAM("Constraint method calcError was not overridded, so it should not be used.");
    return Eigen::VectorXd::Zero(getCoDimension());
  }

  /** \brief For inequality constraints: calculate the Jacobian for the current parameters that are being constraints.
   *
   * TODO(jeroendm), maybe also provide output agruments similar to the jacobian function
   * so we can default to ob::Constraint::jacobian(x, out) when needed.
   *
   * This error jacobian, as the name suggests, is only the jacobian of the position / orientation / ... error.
   * It does not take into acount the derivative of the distance functions defined in the Bounds class.
   * This correction is added in the implementation of of BaseConstraint::jacobian.
   * */
  virtual Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
  {
    ROS_ERROR_STREAM("Constraint method calcErrorJacobian was not overridded, so it should not be used.");
    return Eigen::MatrixXd::Zero(getCoDimension(), n_);
  }

protected:
  // MoveIt's robot representation for kinematic calculations
  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;

  /** \brief Robot link the constraints are applied to. */
  std::string link_name_;

  /** \brief Upper and lower bounds on constrained variables. */
  std::vector<Bounds> bounds_;

  /** \brief target for equality constraints, nominal value for inequality constraints. */
  Eigen::Vector3d target_position_;

  /** \brief target for equality constraints, nominal value for inequality constraints. */
  Eigen::Quaterniond target_orientation_;
};

/** \brief Box shaped position constraints
 *
 * Reads bounds on x, y and z position from a position constraint
 * at constraint_region.primitives[0].dimensions.
 * Where the primitive has to be of type BOX.
 *
 * These bounds are applied around the nominal position and orientation
 * of the box.
 *
 * */
class PositionConstraint : public BaseConstraint
{
public:
  PositionConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group, const unsigned int num_dofs)
    : BaseConstraint(robot_model, group, num_dofs)
  {
  }
  virtual void parseConstraintMsg(const moveit_msgs::Constraints& constraints) override;
  virtual Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
  virtual Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
};

/** \brief orientation constraints based on angle-axis error.
 *
 * (aka exponential coordinates)
 * This is analog to how orientation Error is calculated in the TrajOpt motion planner.
 * It is NOT how orientation error is handled in the default MoveIt constraint samplers.
 * (There, XYZ intrinsic euler angles are used.)
 *
 * */
class AngleAxisConstraint : public BaseConstraint
{
public:
  AngleAxisConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                      const unsigned int num_dofs)
    : BaseConstraint(robot_model, group, num_dofs)
  {
  }

  virtual void parseConstraintMsg(const moveit_msgs::Constraints& constraints) override;
  virtual Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
  virtual Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
};

/** \brief Position and orientation constraint
 *
 * Combine position constraints modelled as a box with orientation constraints
 * using exponential coordinates, but ignore the last rotation around the z-axis.
 * This is a common constraint for grasping, welding, ...
 * */
class PoseConstraint : public BaseConstraint
{
public:
  PoseConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group, const unsigned int num_dofs)
    : BaseConstraint(robot_model, group, num_dofs, 5)
  {
  }

  void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const override
  {
    ob::Constraint::jacobian(x, out);
  }

  virtual void parseConstraintMsg(const moveit_msgs::Constraints& constraints) override;
  virtual Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
  virtual Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
};

/** \brief Extract position constraints from the MoveIt message.
 *
 * Assumes there is a single primitive of type box.
 * Only the dimensions of this box are used here.
 * These are bounds on the deviation of the end-effector from
 * the desired position given as the position of the box in the field
 * constraint_regions.primitive_poses[0].position.
 *
 * @todo: also use the link name in future?
 * Now we assume the constraints are for the end-effector link.
 * */
std::vector<Bounds> positionConstraintMsgToBoundVector(const moveit_msgs::PositionConstraint& pos_con);

/** \brief Extract orientation constraints from the MoveIt message
 *
 * These bounds are assumed to be centered around the nominal orientation / desired orientation
 * given in the field "orientation" in the message.
 * These bounds are therefore bounds on the orientation error between the desired orientation
 * and the current orientation of the end-effector.
 *
 * The three bounds x, y, and z, can be applied to different parameterizations of the rotation error.
 * (Roll, pithc, and yaw or exponential coordinates or something else.)
 *
 * */
std::vector<Bounds> orientationConstraintMsgToBoundVector(const moveit_msgs::OrientationConstraint& ori_con);

/** \brief Factory to create constraints based on what is in the MoveIt constraint message. **/
std::shared_ptr<BaseConstraint> createConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                                                 const moveit_msgs::Constraints& constraints);

/** \brief Conversion matrix to go from angular velocity in the world frame to
 * angle axis equivalent.
 *
 * Based on:
 * https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
 *
 * */
Eigen::Matrix3d angularVelocityToAngleAxis(double angle, const Eigen::Vector3d& axis);

}  // namespace ompl_interface