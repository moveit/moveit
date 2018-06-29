/**
 * @file kinematics.h
 * @brief This defines kinematic related utilities.
 *
 * @author Jorge Nicho
 * @date June 3, 2016
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
#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UTILS_KINEMATICS_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UTILS_KINEMATICS_H_

#include <ros/console.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_list_macros.h>
#include <XmlRpcException.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>


namespace stomp_moveit
{
namespace utils
{

/**
 * @namespace stomp_moveit::utils::kinematics
 * @brief Utility functions related to finding Inverse Kinematics solutions
 */
namespace kinematics
{

  const static double EPSILON = 0.011;  /**< @brief Used in dampening the matrix pseudo inverse calculation */
  const static double LAMBDA = 0.01;    /**< @brief Used in dampening the matrix pseudo inverse calculation */

  /**
   * @struct stomp_moveit::utils::KinematicConfig
   * @brief Convenience structure that contains the variables used in solving for an ik solution.
   */
  struct KinematicConfig
  {
    /** @name Constraint Parameters
     * The constraints on the cartesian goal
     */
    Eigen::Array<int,6,1> constrained_dofs = Eigen::Array<int,6,1>::Ones(); /**< @brief  A vector of the form [x y z rx ry rz] filled with 0's and 1's
                                                                                        to indicate an unconstrained or fully constrained DOF. **/
    Eigen::Affine3d tool_goal_pose = Eigen::Affine3d::Identity();           /**< @brief  The desired tool pose. **/

    /** @name Support Parameters
     * Used at each iteration until a solution is found
     */
    Eigen::ArrayXd joint_update_rates = Eigen::ArrayXd::Zero(1,1);          /**< @brief The weights to be applied to each update during every iteration [num_dimensions x 1]. **/
    Eigen::Array<double,6,1> cartesian_convergence_thresholds = Eigen::Array<double,6,1>::Zero(); /**< @brief  The error margin for each dimension of the twist vector [6 x 1]. **/
    Eigen::VectorXd init_joint_pose = Eigen::ArrayXd::Zero(1,1);            /**< @brief  Seed joint pose [num_dimension x 1]. **/
    int max_iterations = 100;                                               /**< @brief  The maximum number of iterations that the algorithm will run until convergence is reached. **/

    /** @name Null Space Parameters
     * Used in exploding the task manifold null space
     */
    Eigen::ArrayXd null_proj_weights = Eigen::ArrayXd::Zero(1,1);  /**< @brief Weights to be applied to the null space vector */
    Eigen::VectorXd null_space_vector = Eigen::VectorXd::Zero(1);  /**< @brief Null space vector that is used to exploit the Jacobian's null space */

  };

  /**
   * @brief Populates a Kinematic Config struct from the position and orientation constraints requested;
   * @param group             A pointer to the JointModelGroup
   * @param pc                The position constraint message
   * @param oc                The orientation constraint message.  Absolute tolerances greater than 2PI will be considered unconstrained.
   * @param init_joint_pose   The initial joint values
   * @param kc                The output KinematicConfig object
   * @return  True if succeeded, false otherwise
   */
  static bool createKinematicConfig(const moveit::core::JointModelGroup* group,
                                               const moveit_msgs::PositionConstraint& pc,const moveit_msgs::OrientationConstraint& oc,
                                               const Eigen::VectorXd& init_joint_pose,KinematicConfig& kc)
  {
    int num_joints = group->getActiveJointModelNames().size();
    if(num_joints != init_joint_pose.size())
    {
      ROS_ERROR("Initial joint pose has an incorrect number of joints");
      return false;
    }

    // tool pose position
    kc.tool_goal_pose.setIdentity();
    auto& v = pc.constraint_region.primitive_poses[0].position;
    kc.tool_goal_pose.translation() = Eigen::Vector3d(v.x,v.y,v.z);

    // tool pose orientation
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(oc.orientation,q);
    kc.tool_goal_pose.rotate(q);

    // defining constraints
    kc.constrained_dofs << 1, 1, 1, 1, 1, 1;

    // position tolerance
    const shape_msgs::SolidPrimitive& bv = pc.constraint_region.primitives[0];
    if(bv.type != shape_msgs::SolidPrimitive::BOX || bv.dimensions.size() != 3)
    {
      ROS_ERROR("Position constraint incorrectly defined, a BOX region is expected");
      return false;
    }

    using SP = shape_msgs::SolidPrimitive;
    kc.cartesian_convergence_thresholds[0] = bv.dimensions[SP::BOX_X];
    kc.cartesian_convergence_thresholds[1] = bv.dimensions[SP::BOX_Y];
    kc.cartesian_convergence_thresholds[2] = bv.dimensions[SP::BOX_Z];

    // orientation tolerance
    kc.cartesian_convergence_thresholds[3] = oc.absolute_x_axis_tolerance;
    kc.cartesian_convergence_thresholds[4] = oc.absolute_y_axis_tolerance;
    kc.cartesian_convergence_thresholds[5] = oc.absolute_z_axis_tolerance;

    // unconstraining orientation dofs that exceed 2pi
    for(std::size_t i = 3; i < kc.cartesian_convergence_thresholds.size(); i++)
    {
      auto v = kc.cartesian_convergence_thresholds[i];
      kc.constrained_dofs[i] = v >= 2*M_PI ? 0 : 1;
    }

    // additional variables
    kc.joint_update_rates = Eigen::ArrayXd::Constant(num_joints,0.5f);
    kc.init_joint_pose = init_joint_pose;
    kc.max_iterations = 100;

    return true;
  }

  /**
   * @brief Populates a Kinematic Config struct from the position and orientation constraints requested;
   * @param group         A pointer to the JointModelGroup
   * @param pc            The position constraint message
   * @param oc            The orientation constraint message.  Absolute tolerances greater than 2PI will be considered unconstrained.
   * @param start_state   The start robot state message
   * @param kc            The output KinematicConfig object
   * @return    True if succeeded, false otherwise
   */
  static bool createKinematicConfig(const moveit::core::JointModelGroup* group,
                                               const moveit_msgs::PositionConstraint& pc,const moveit_msgs::OrientationConstraint& oc,
                                               const moveit_msgs::RobotState& start_state,KinematicConfig& kc)
  {
    const auto& joint_names= group->getActiveJointModelNames();
    const auto& jstate = start_state.joint_state;
    std::size_t ind = 0;
    Eigen::VectorXd joint_vals = Eigen::VectorXd::Zero(joint_names.size());
    for(std::size_t i = 0; i < joint_names.size();i ++)
    {
      auto pos = std::find(jstate.name.begin(),jstate.name.end(),joint_names[i]);
      if(pos == jstate.name.end())
      {
        return false;
      }

      ind = std::distance(jstate.name.begin(),pos);
      joint_vals(i) = jstate.position[ind];
    }

    return createKinematicConfig(group,pc,oc,joint_vals,kc);
  }

/**
 * @brief Computes the twist vector [vx vy vz wx wy wz]'  relative to the current tool coordinate system.  The rotational part is
 *        composed of the product between the angle times the axis about which it rotates.
 *
 * @param p0        start tool pose in world coordinates
 * @param pf        final tool pose in world coordinates
 * @param nullity   array of 0's and 1's indicating which cartesian DOF's are unconstrained (0)
 * @param twist     the twist vector in tool coordinates (change from p0 to pf) [6 x 1].
 */
  static void computeTwist(const Eigen::Affine3d& p0,
                                          const Eigen::Affine3d& pf,
                                          const Eigen::ArrayXi& nullity,Eigen::VectorXd& twist)
  {
    twist.resize(nullity.size());
    twist.setConstant(0);

    // relative transform
    auto p0_inv = p0.inverse();
    Eigen::Affine3d t = (p0_inv) * pf;

    Eigen::Vector3d twist_pos = p0_inv.rotation()*(pf.translation() - p0.translation());

    // relative rotation -> R = inverse(R0) * Rf
    Eigen::AngleAxisd relative_rot(t.rotation());
    double angle = relative_rot.angle();
    Eigen::Vector3d axis = relative_rot.axis();

    // forcing angle to range [-pi , pi]
    while( (angle > M_PI) || (angle < -M_PI))
    {
      angle = (angle >  M_PI) ? (angle - 2*M_PI) : angle;
      angle = (angle < -M_PI )? (angle + 2*M_PI) : angle;
    }

    // creating twist rotation relative to tool
    Eigen::Vector3d twist_rot = axis.normalized() * angle;

    // assigning into full 6dof twist vector
    twist.head(3) = twist_pos;
    twist.tail(3) = twist_rot;

    // zeroing all underconstrained cartesian dofs
    twist = (nullity == 0).select(0,twist);
  }

  /**
   * @brief Convenience function to remove entire rows of the Jacobian matrix.
   * @param jacb          The jacobian matrix of size [num_dimensions x 6]
   * @param indices       An indices vector where each entry indicates an row index of the jacobian that will be kept.
   * @param jacb_reduced  The reduced jacobian containing the only the rows indicated by the 'indices' array.
   */
  static void reduceJacobian(const Eigen::MatrixXd& jacb,
                                            const std::vector<int>& indices,Eigen::MatrixXd& jacb_reduced)
  {
    jacb_reduced.resize(indices.size(),jacb.cols());
    for(auto i = 0u; i < indices.size(); i++)
    {
      jacb_reduced.row(i) = jacb.row(indices[i]);
    }
  }

  /**
   * @brief Calculates the damped pseudo inverse of a matrix using singular value decomposition
   * @param jacb              The jacobian matrix
   * @param jacb_pseudo_inv   The pseudo inverse of the matrix
   * @param eps               Used to threshold the singular values
   * @param lambda            Used in preventing division by small singular values from generating large numbers.
   */
  static void calculateDampedPseudoInverse(const Eigen::MatrixXd &jacb, Eigen::MatrixXd &jacb_pseudo_inv,
                                           double eps = 0.011, double lambda = 0.01)
  {
    using namespace Eigen;


    //Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are real)
    //in order to solve Ax=b -> x*=A+ b
    Eigen::JacobiSVD<MatrixXd> svd(jacb, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const MatrixXd &U = svd.matrixU();
    const VectorXd &Sv = svd.singularValues();
    const MatrixXd &V = svd.matrixV();

    // calculate the reciprocal of Singular-Values
    // damp inverse with lambda so that inverse doesn't oscillate near solution
    size_t nSv = Sv.size();
    VectorXd inv_Sv(nSv);
    for(size_t i=0; i< nSv; ++i)
    {
      if (fabs(Sv(i)) > eps)
      {
        inv_Sv(i) = 1/Sv(i);
      }
      else
      {
        inv_Sv(i) = Sv(i) / (Sv(i)*Sv(i) + lambda*lambda);
      }
    }

    jacb_pseudo_inv = V * inv_Sv.asDiagonal() * U.transpose();
  }

  /**
   * @brief Solves the inverse kinematics for a given tool pose using a gradient descent method.  It can handle under constrained DOFs for
   *  the cartesian tool pose and can also apply a vector onto the null space of the jacobian in order to meet a secondary objective.  It
   *  also checks for joint limits.
   * @param robot_state                       A pointer to the robot state.
   * @param group_name                        The name of the kinematic group. The tool link name is assumed to be the last link in this group.
   * @param constrained_dofs                  A vector of the form [x y z rx ry rz] filled with 0's and 1's to indicate an unconstrained or fully constrained DOF.
   * @param joint_update_rates                The weights to be applied to each update during every iteration [num_dimensions x 1].
   * @param cartesian_convergence_thresholds  The error margin for each dimension of the twist vector [6 x 1].
   * @param null_proj_weights                 The weights to be multiplied to the null space vector [num_dimension x 1].
   * @param null_space_vector                 The null space vector which is applied into the jacobian's null space [num_dimension x 1].
   * @param max_iterations                    The maximum number of iterations that the algorithm will run until convergence is reached.
   * @param tool_goal_pose                    The desired tool pose.
   * @param init_joint_pose                   Seed joint pose [num_dimension x 1].
   * @param joint_pose                        IK joint solution [num_dimension x 1].
   * @return  True if a solution was found, false otherwise.
   */
  static bool solveIK(moveit::core::RobotStatePtr robot_state, const std::string& group_name,
                      const Eigen::Array<int,6,1>& constrained_dofs,
                      const Eigen::ArrayXd& joint_update_rates,
                      const Eigen::Array<double,6,1>& cartesian_convergence_thresholds,
                      const Eigen::ArrayXd& null_proj_weights,
                      const Eigen::VectorXd& null_space_vector,
                      int max_iterations,
                      const Eigen::Affine3d& tool_goal_pose,
                      const Eigen::VectorXd& init_joint_pose,
                      Eigen::VectorXd& joint_pose)
  {

    using namespace Eigen;
    using namespace moveit::core;

    // joint variables
    VectorXd delta_j = VectorXd::Zero(init_joint_pose.size());
    joint_pose = init_joint_pose;
    const JointModelGroup* joint_group = robot_state->getJointModelGroup(group_name);
    robot_state->setJointGroupPositions(joint_group,joint_pose);
    const auto& joint_names = joint_group->getActiveJointModelNames();
    std::string tool_link = joint_group->getLinkModelNames().back();
    Affine3d tool_current_pose = robot_state->getGlobalLinkTransform(tool_link);

    auto harmonize_joints = [&joint_group,&joint_names](Eigen::VectorXd& joint_vals) -> bool
    {
      if(joint_names.size() != joint_vals.size())
      {
        return false;
      }

      const double incr = 2*M_PI;
      for(std::size_t i = 0; i < joint_names.size();i++)
      {
        if( joint_group->getJointModel(joint_names[i])->getType() != JointModel::REVOLUTE )
        {
          continue;
        }

        const JointModel::Bounds& bounds = joint_group->getJointModel(joint_names[i])->getVariableBounds();

        double j = joint_vals(i);
        for(const VariableBounds& b: bounds)
        {

          while(j > b.max_position_)
          {
            j-= incr;
          }

          while(j < b.min_position_)
          {
            j += incr;
          }
        }

        joint_vals(i) = j;
      }

      return true;
    };

    // tool twist variables
    VectorXd tool_twist, tool_twist_reduced;
    std::vector<int> indices;
    for(auto i = 0u; i < constrained_dofs.size(); i++)
    {
      if(constrained_dofs(i) != 0)
      {
        indices.push_back(i);
      }
    }
    tool_twist_reduced = VectorXd::Zero(indices.size());

    // jacobian calculation variables
    static MatrixXd jacb_transform(6,6);
    MatrixXd jacb, jacb_reduced, jacb_pseudo_inv;
    MatrixXd identity = MatrixXd::Identity(init_joint_pose.size(),init_joint_pose.size());
    VectorXd null_space_proj;
    bool project_into_nullspace = (null_proj_weights.size()> 0) &&  (null_proj_weights >1e-8).any();

    unsigned int iteration_count = 0;
    bool converged = false;
    while(iteration_count < max_iterations)
    {
      // computing twist vector
      computeTwist(tool_current_pose,tool_goal_pose,constrained_dofs,tool_twist);

      // check convergence
      if((tool_twist.cwiseAbs().array() <= cartesian_convergence_thresholds).all())
      {
        if(robot_state->satisfiesBounds(joint_group))
        {
          converged = true;
        }
        else
        {
          ROS_DEBUG("IK joint solution violates bounds");
        }
        break;
      }

      // updating reduced tool twist
      for(auto i = 0u; i < indices.size(); i++)
      {
        tool_twist_reduced(i) = tool_twist(indices[i]);
      }

      // computing jacobian
      if(!robot_state->getJacobian(joint_group,robot_state->getLinkModel(tool_link),Vector3d::Zero(),jacb))
      {
        ROS_ERROR("Failed to get Jacobian for link %s",tool_link.c_str());
        return false;
      }

      // transform jacobian to tool coordinates
      auto rot = tool_current_pose.inverse().rotation();
      jacb_transform.setZero();
      jacb_transform.block(0,0,3,3) = rot;
      jacb_transform.block(3,3,3,3) = rot;
      jacb = jacb_transform*jacb;

      // reduce jacobian and compute its pseudo inverse
      reduceJacobian(jacb,indices,jacb_reduced);
      calculateDampedPseudoInverse(jacb_reduced,jacb_pseudo_inv,EPSILON,LAMBDA);

      if(project_into_nullspace)
      {
        null_space_proj = (identity - jacb_pseudo_inv*jacb_reduced)*null_space_vector;
        null_space_proj = (null_space_proj.array() * null_proj_weights).matrix();

        // computing joint change
        delta_j = (jacb_pseudo_inv*tool_twist_reduced) + null_space_proj;
      }
      else
      {
        // computing joint change
        delta_j = (jacb_pseudo_inv*tool_twist_reduced);
      }

      // updating joint values
      joint_pose += (joint_update_rates* delta_j.array()).matrix();
      harmonize_joints(joint_pose);

      // updating tool pose
      robot_state->setJointGroupPositions(joint_group,joint_pose);
      robot_state->updateLinkTransforms();
      tool_current_pose = robot_state->getGlobalLinkTransform(tool_link);

      iteration_count++;
    }

    ROS_DEBUG_STREAM_COND(!converged,"Error tool twist "<<tool_twist.transpose());

    return converged;
  }

  /**
   * @brief Solves the inverse kinematics for a given tool pose using a gradient descent method.  It can handle under constrained DOFs for
   *  the cartesian tool pose and can also apply a vector onto the null space of the jacobian in order to meet a secondary objective.  It
   *  also checks for joint limits.
   * @param robot_state A pointer to the robot state.
   * @param group_name  The name of the kinematic group. The tool link name is assumed to be the last link in this group.
   * @param config      A structure containing the variables to be used in finding a solution.
   * @param joint_pose  IK joint solution [num_dimension x 1].
   * @return  True if a solution was found, false otherwise.
   */
  static bool solveIK(moveit::core::RobotStatePtr robot_state, const std::string& group_name,const KinematicConfig& config, Eigen::VectorXd& joint_pose)
  {

    return solveIK(robot_state,
                   group_name,
                   config.constrained_dofs,
                   config.joint_update_rates,
                   config.cartesian_convergence_thresholds,
                   config.null_proj_weights,
                   config.null_space_vector,
                   config.max_iterations,
                   config.tool_goal_pose,
                   config.init_joint_pose,
                   joint_pose);
  }

  /**
   * @brief Convenience function to calculate the Jacobian's null space matrix for an under constrained tool cartesian pose.
   * @param state             A pointer to the robot state.
   * @param group             The name of the kinematic group.
   * @param tool_link         The tool link name
   * @param constrained_dofs  A vector of the form [x y z rx ry rz] filled with 0's and 1's to indicate an unconstrained or fully constrained DOF.
   * @param joint_pose        The joint pose at which to compute the jacobian matrix.
   * @param jacb_nullspace    The jacobian null space matrix [num_dimensions x num_dimensions]
   * @return True if a solution was found, false otherwise.
   */
  static bool computeJacobianNullSpace(moveit::core::RobotStatePtr state,std::string group,std::string tool_link,
                                       const Eigen::ArrayXi& constrained_dofs,const Eigen::VectorXd& joint_pose,
                                       Eigen::MatrixXd& jacb_nullspace)
  {
    using namespace Eigen;
    using namespace moveit::core;

    // robot state
    const JointModelGroup* joint_group = state->getJointModelGroup(group);
    state->setJointGroupPositions(joint_group,joint_pose);
    Affine3d tool_pose = state->getGlobalLinkTransform(tool_link);

    // jacobian calculations
    static MatrixXd jacb_transform(6,6);
    MatrixXd jacb, jacb_reduced, jacb_pseudo_inv;
    jacb_transform.setZero();

    if(!state->getJacobian(joint_group,state->getLinkModel(tool_link),Vector3d::Zero(),jacb))
    {
      ROS_ERROR("Failed to get Jacobian for link %s",tool_link.c_str());
      return false;
    }

    // transform jacobian rotational part to tool coordinates
    auto rot = tool_pose.inverse().rotation();
    jacb_transform.setZero();
    jacb_transform.block(0,0,3,3) = rot;
    jacb_transform.block(3,3,3,3) = rot;
    jacb = jacb_transform*jacb;

    // reduce jacobian and compute its pseudo inverse
    std::vector<int> indices;
    for(auto i = 0u; i < constrained_dofs.size(); i++)
    {
      if(constrained_dofs(i) != 0)
      {
        indices.push_back(i);
      }
    }
    reduceJacobian(jacb,indices,jacb_reduced);
    calculateDampedPseudoInverse(jacb_reduced,jacb_pseudo_inv,EPSILON,LAMBDA);

    int num_joints = joint_pose.size();
    jacb_nullspace = MatrixXd::Identity(num_joints,num_joints) - jacb_pseudo_inv*jacb_reduced;

    return true;
  }

  /**
   * @brief Solves the inverse kinematics for a given tool pose using a gradient descent method.  It can handle under constrained DOFs for
   *  the cartesian tool pose.
   * @param robot_state                       A pointer to the robot state.
   * @param group_name                        The name of the kinematic group. The tool link name is assumed to be the last link in this group.
   * @param constrained_dofs                  A vector of the form [x y z rx ry rz] filled with 0's and 1's to indicate an unconstrained or fully constrained DOF.
   * @param joint_update_rates                The weights to be applied to each update during every iteration [num_dimensions x 1].
   * @param cartesian_convergence_thresholds  The error margin for each dimension of the twist vector [6 x 1].
   * @param max_iterations                    The maximum number of iterations that the algorithm will run to reach convergence.
   * @param tool_goal_pose                    The cartesian pose of the tool relative to the world.
   * @param init_joint_pose                   Seed joint pose [num_dimension x 1].
   * @param joint_pose                        IK joint solution [num_dimension x 1].
   * @return  True if a solution was found, false otherwise.
   * @return
   */
  static bool solveIK(moveit::core::RobotStatePtr robot_state, const std::string& group_name, const Eigen::ArrayXi& constrained_dofs,
               const Eigen::ArrayXd& joint_update_rates,const Eigen::ArrayXd& cartesian_convergence_thresholds, int max_iterations,
               const Eigen::Affine3d& tool_goal_pose,const Eigen::VectorXd& init_joint_pose,
               Eigen::VectorXd& joint_pose)
  {
    using namespace Eigen;
    using namespace moveit::core;

    return solveIK(robot_state,group_name,constrained_dofs,joint_update_rates,cartesian_convergence_thresholds,
            ArrayXd::Zero(joint_update_rates.size()),VectorXd::Zero(joint_update_rates.size()),max_iterations,
            tool_goal_pose,init_joint_pose,joint_pose);
  }


} // kinematics
} // utils
} // stomp_moveit



#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UTILS_KINEMATICS_H_ */
