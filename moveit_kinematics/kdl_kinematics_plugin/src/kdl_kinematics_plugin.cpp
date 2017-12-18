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

#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
#include <class_loader/class_loader.h>

//#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl_parser/kdl_parser.hpp>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <moveit/rdf_loader/rdf_loader.h>

// register KDLKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(kdl_kinematics_plugin::KDLKinematicsPlugin, kinematics::KinematicsBase)

namespace kdl_kinematics_plugin
{
KDLKinematicsPlugin::KDLKinematicsPlugin() : active_(false)
{
}

void KDLKinematicsPlugin::getRandomConfiguration(KDL::JntArray& jnt_array, bool lock_redundancy) const
{
  std::vector<double> jnt_array_vector(dimension_, 0.0);
  state_->setToRandomPositions(joint_model_group_);
  state_->copyJointGroupPositions(joint_model_group_, &jnt_array_vector[0]);
  for (std::size_t i = 0; i < dimension_; ++i)
  {
    if (lock_redundancy)
      if (isRedundantJoint(i))
        continue;
    jnt_array(i) = jnt_array_vector[i];
  }
}

bool KDLKinematicsPlugin::isRedundantJoint(unsigned int index) const
{
  for (std::size_t j = 0; j < redundant_joint_indices_.size(); ++j)
    if (redundant_joint_indices_[j] == index)
      return true;
  return false;
}

void KDLKinematicsPlugin::getRandomConfiguration(const KDL::JntArray& seed_state,
                                                 const std::vector<double>& consistency_limits,
                                                 KDL::JntArray& jnt_array, bool lock_redundancy) const
{
  std::vector<double> values(dimension_, 0.0);
  std::vector<double> near(dimension_, 0.0);
  for (std::size_t i = 0; i < dimension_; ++i)
    near[i] = seed_state(i);

  // Need to resize the consistency limits to remove mimic joints
  std::vector<double> consistency_limits_mimic;
  for (std::size_t i = 0; i < dimension_; ++i)
  {
    if (!mimic_joints_[i].active)
      continue;
    consistency_limits_mimic.push_back(consistency_limits[i]);
  }

  joint_model_group_->getVariableRandomPositionsNearBy(state_->getRandomNumberGenerator(), values, near,
                                                       consistency_limits_mimic);

  for (std::size_t i = 0; i < dimension_; ++i)
  {
    bool skip = false;
    if (lock_redundancy)
      for (std::size_t j = 0; j < redundant_joint_indices_.size(); ++j)
        if (redundant_joint_indices_[j] == i)
        {
          skip = true;
          break;
        }
    if (skip)
      continue;
    jnt_array(i) = values[i];
  }
}

bool KDLKinematicsPlugin::checkConsistency(const KDL::JntArray& seed_state,
                                           const std::vector<double>& consistency_limits,
                                           const KDL::JntArray& solution) const
{
  for (std::size_t i = 0; i < dimension_; ++i)
    if (fabs(seed_state(i) - solution(i)) > consistency_limits[i])
      return false;
  return true;
}

bool KDLKinematicsPlugin::initialize(const std::string& robot_description, const std::string& group_name,
                                     const std::string& base_frame, const std::string& tip_frame,
                                     double search_discretization)
{
  setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);

  rdf_loader::RDFLoader rdf_loader(robot_description_);
  const srdf::ModelSharedPtr& srdf = rdf_loader.getSRDF();
  const urdf::ModelInterfaceSharedPtr& urdf_model = rdf_loader.getURDF();

  if (!urdf_model || !srdf)
  {
    ROS_ERROR_NAMED("kdl", "URDF and SRDF must be loaded for KDL kinematics solver to work.");
    return false;
  }

  robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

  robot_model::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group)
    return false;

  if (!joint_model_group->isChain())
  {
    ROS_ERROR_NAMED("kdl", "Group '%s' is not a chain", group_name.c_str());
    return false;
  }
  if (!joint_model_group->isSingleDOFJoints())
  {
    ROS_ERROR_NAMED("kdl", "Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
    return false;
  }

  KDL::Tree kdl_tree;

  if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
  {
    ROS_ERROR_NAMED("kdl", "Could not initialize tree object");
    return false;
  }
  if (!kdl_tree.getChain(base_frame_, getTipFrame(), kdl_chain_))
  {
    ROS_ERROR_NAMED("kdl", "Could not initialize chain object");
    return false;
  }

  dimension_ = joint_model_group->getActiveJointModels().size() + joint_model_group->getMimicJointModels().size();
  for (std::size_t i = 0; i < joint_model_group->getJointModels().size(); ++i)
  {
    if (joint_model_group->getJointModels()[i]->getType() == moveit::core::JointModel::REVOLUTE ||
        joint_model_group->getJointModels()[i]->getType() == moveit::core::JointModel::PRISMATIC)
    {
      ik_chain_info_.joint_names.push_back(joint_model_group->getJointModelNames()[i]);
      const std::vector<moveit_msgs::JointLimits>& jvec =
          joint_model_group->getJointModels()[i]->getVariableBoundsMsg();
      ik_chain_info_.limits.insert(ik_chain_info_.limits.end(), jvec.begin(), jvec.end());
    }
  }

  fk_chain_info_.joint_names = ik_chain_info_.joint_names;
  fk_chain_info_.limits = ik_chain_info_.limits;

  if (!joint_model_group->hasLinkModel(getTipFrame()))
  {
    ROS_ERROR_NAMED("kdl", "Could not find tip name in joint group '%s'", group_name.c_str());
    return false;
  }
  ik_chain_info_.link_names.push_back(getTipFrame());
  fk_chain_info_.link_names = joint_model_group->getLinkModelNames();

  joint_min_.resize(ik_chain_info_.limits.size());
  joint_max_.resize(ik_chain_info_.limits.size());

  for (unsigned int i = 0; i < ik_chain_info_.limits.size(); i++)
  {
    joint_min_(i) = ik_chain_info_.limits[i].min_position;
    joint_max_(i) = ik_chain_info_.limits[i].max_position;
  }

  // Get Solver Parameters
  int max_solver_iterations;
  double epsilon;
  bool position_ik;

  lookupParam("max_solver_iterations", max_solver_iterations, 500);
  lookupParam("epsilon", epsilon, 1e-5);
  lookupParam("position_only_ik", position_ik, false);
  ROS_DEBUG_NAMED("kdl", "Looking for param name: position_only_ik");

  if (position_ik)
    ROS_INFO_NAMED("kdl", "Using position only ik");

  num_possible_redundant_joints_ =
      kdl_chain_.getNrOfJoints() - joint_model_group->getMimicJointModels().size() - (position_ik ? 3 : 6);

  // Check for mimic joints
  std::vector<unsigned int> redundant_joints_map_index;

  std::vector<JointMimic> mimic_joints;
  unsigned int joint_counter = 0;
  for (std::size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
  {
    const robot_model::JointModel* jm = robot_model_->getJointModel(kdl_chain_.segments[i].getJoint().getName());

    // first check whether it belongs to the set of active joints in the group
    if (jm->getMimic() == NULL && jm->getVariableCount() > 0)
    {
      JointMimic mimic_joint;
      mimic_joint.reset(joint_counter);
      mimic_joint.joint_name = kdl_chain_.segments[i].getJoint().getName();
      mimic_joint.active = true;
      mimic_joints.push_back(mimic_joint);
      ++joint_counter;
      continue;
    }
    if (joint_model_group->hasJointModel(jm->getName()))
    {
      if (jm->getMimic() && joint_model_group->hasJointModel(jm->getMimic()->getName()))
      {
        JointMimic mimic_joint;
        mimic_joint.reset(joint_counter);
        mimic_joint.joint_name = kdl_chain_.segments[i].getJoint().getName();
        mimic_joint.offset = jm->getMimicOffset();
        mimic_joint.multiplier = jm->getMimicFactor();
        mimic_joints.push_back(mimic_joint);
        continue;
      }
    }
  }
  for (std::size_t i = 0; i < mimic_joints.size(); ++i)
  {
    if (!mimic_joints[i].active)
    {
      const robot_model::JointModel* joint_model =
          joint_model_group->getJointModel(mimic_joints[i].joint_name)->getMimic();
      for (std::size_t j = 0; j < mimic_joints.size(); ++j)
      {
        if (mimic_joints[j].joint_name == joint_model->getName())
        {
          mimic_joints[i].map_index = mimic_joints[j].map_index;
        }
      }
    }
  }
  mimic_joints_ = mimic_joints;

  // Setup the joint state groups that we need
  state_.reset(new robot_state::RobotState(robot_model_));
  state_2_.reset(new robot_state::RobotState(robot_model_));

  // Store things for when the set of redundant joints may change
  position_ik_ = position_ik;
  joint_model_group_ = joint_model_group;
  max_solver_iterations_ = max_solver_iterations;
  epsilon_ = epsilon;

  active_ = true;
  ROS_DEBUG_NAMED("kdl", "KDL solver initialized");
  return true;
}

bool KDLKinematicsPlugin::setRedundantJoints(const std::vector<unsigned int>& redundant_joints)
{
  if (num_possible_redundant_joints_ < 0)
  {
    ROS_ERROR_NAMED("kdl", "This group cannot have redundant joints");
    return false;
  }
  if (static_cast<int>(redundant_joints.size()) > num_possible_redundant_joints_)
  {
    ROS_ERROR_NAMED("kdl", "This group can only have %d redundant joints", num_possible_redundant_joints_);
    return false;
  }
  /*
    XmlRpc::XmlRpcValue joint_list;
    if(private_handle.getParam(group_name+"/redundant_joints", joint_list))
    {
      ROS_ASSERT(joint_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      std::vector<std::string> redundant_joints;
      for (std::size_t i = 0; i < joint_list.size(); ++i)
      {
        ROS_ASSERT(joint_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        redundant_joints.push_back(static_cast<std::string>(joint_list[i]));
        ROS_INFO_NAMED("kdl","Designated joint: %s as redundant joint", redundant_joints.back().c_str());
      }
    }
  */
  std::vector<unsigned int> redundant_joints_map_index;
  unsigned int counter = 0;
  for (std::size_t i = 0; i < dimension_; ++i)
  {
    bool is_redundant_joint = false;
    for (std::size_t j = 0; j < redundant_joints.size(); ++j)
    {
      if (i == redundant_joints[j])
      {
        is_redundant_joint = true;
        counter++;
        break;
      }
    }
    if (!is_redundant_joint)
    {
      // check for mimic
      if (mimic_joints_[i].active)
      {
        redundant_joints_map_index.push_back(counter);
        counter++;
      }
    }
  }
  for (std::size_t i = 0; i < redundant_joints_map_index.size(); ++i)
    ROS_DEBUG_NAMED("kdl", "Redundant joint map index: %d %d", (int)i, (int)redundant_joints_map_index[i]);

  redundant_joints_map_index_ = redundant_joints_map_index;
  redundant_joint_indices_ = redundant_joints;
  return true;
}

int KDLKinematicsPlugin::getJointIndex(const std::string& name) const
{
  for (unsigned int i = 0; i < ik_chain_info_.joint_names.size(); i++)
  {
    if (ik_chain_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

int KDLKinematicsPlugin::getKDLSegmentIndex(const std::string& name) const
{
  int i = 0;
  while (i < (int)kdl_chain_.getNrOfSegments())
  {
    if (kdl_chain_.getSegment(i).getName() == name)
    {
      return i + 1;
    }
    i++;
  }
  return -1;
}

bool KDLKinematicsPlugin::timedOut(const ros::WallTime& start_time, double duration) const
{
  return ((ros::WallTime::now() - start_time).toSec() >= duration);
}

bool KDLKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                        std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                        const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, solution, solution_callback, error_code,
                          consistency_limits, options);
}

bool KDLKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, std::vector<double>& solution,
                                           moveit_msgs::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool KDLKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, const std::vector<double>& consistency_limits,
                                           std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool KDLKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, std::vector<double>& solution,
                                           const IKCallbackFn& solution_callback,
                                           moveit_msgs::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool KDLKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, const std::vector<double>& consistency_limits,
                                           std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                           moveit_msgs::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool KDLKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, std::vector<double>& solution,
                                           const IKCallbackFn& solution_callback,
                                           moveit_msgs::MoveItErrorCodes& error_code,
                                           const std::vector<double>& consistency_limits,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  ros::WallTime n1 = ros::WallTime::now();
  if (!active_)
  {
    ROS_ERROR_NAMED("kdl", "kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED("kdl", "Seed state must have size " << dimension_ << " instead of size "
                                                               << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (!consistency_limits.empty() && consistency_limits.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED("kdl", "Consistency limits be empty or must have size " << dimension_ << " instead of size "
                                                                                   << consistency_limits.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  KDL::JntArray jnt_seed_state(dimension_);
  KDL::JntArray jnt_pos_in(dimension_);
  KDL::JntArray jnt_pos_out(dimension_);

  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);
  KDL::ChainIkSolverVel_pinv_mimic ik_solver_vel(kdl_chain_, joint_model_group_->getMimicJointModels().size(),
                                                 redundant_joint_indices_.size(), position_ik_);
  KDL::ChainIkSolverPos_NR_JL_Mimic ik_solver_pos(kdl_chain_, joint_min_, joint_max_, fk_solver, ik_solver_vel,
                                                  max_solver_iterations_, epsilon_, position_ik_);
  ik_solver_vel.setMimicJoints(mimic_joints_);
  ik_solver_pos.setMimicJoints(mimic_joints_);

  if ((redundant_joint_indices_.size() > 0) && !ik_solver_vel.setRedundantJointsMapIndex(redundant_joints_map_index_))
  {
    ROS_ERROR_NAMED("kdl", "Could not set redundant joints");
    return false;
  }

  if (options.lock_redundant_joints)
  {
    ik_solver_vel.lockRedundantJoints();
  }

  solution.resize(dimension_);

  KDL::Frame pose_desired;
  tf::poseMsgToKDL(ik_pose, pose_desired);

  ROS_DEBUG_STREAM_NAMED("kdl", "searchPositionIK2: Position request pose is "
                                    << ik_pose.position.x << " " << ik_pose.position.y << " " << ik_pose.position.z
                                    << " " << ik_pose.orientation.x << " " << ik_pose.orientation.y << " "
                                    << ik_pose.orientation.z << " " << ik_pose.orientation.w);
  // Do the IK
  for (unsigned int i = 0; i < dimension_; i++)
    jnt_seed_state(i) = ik_seed_state[i];
  jnt_pos_in = jnt_seed_state;

  unsigned int counter(0);
  while (1)
  {
    //    ROS_DEBUG_NAMED("kdl","Iteration: %d, time: %f, Timeout:
    //    %f",counter,(ros::WallTime::now()-n1).toSec(),timeout);
    counter++;
    if (timedOut(n1, timeout))
    {
      ROS_DEBUG_NAMED("kdl", "IK timed out");
      error_code.val = error_code.TIMED_OUT;
      ik_solver_vel.unlockRedundantJoints();
      return false;
    }
    int ik_valid = ik_solver_pos.CartToJnt(jnt_pos_in, pose_desired, jnt_pos_out);
    ROS_DEBUG_NAMED("kdl", "IK valid: %d", ik_valid);
    if (!consistency_limits.empty())
    {
      getRandomConfiguration(jnt_seed_state, consistency_limits, jnt_pos_in, options.lock_redundant_joints);
      if ((ik_valid < 0 && !options.return_approximate_solution) ||
          !checkConsistency(jnt_seed_state, consistency_limits, jnt_pos_out))
      {
        ROS_DEBUG_NAMED("kdl", "Could not find IK solution: does not match consistency limits");
        continue;
      }
    }
    else
    {
      getRandomConfiguration(jnt_pos_in, options.lock_redundant_joints);
      ROS_DEBUG_NAMED("kdl", "New random configuration");
      for (unsigned int j = 0; j < dimension_; j++)
        ROS_DEBUG_NAMED("kdl", "%d %f", j, jnt_pos_in(j));

      if (ik_valid < 0 && !options.return_approximate_solution)
      {
        ROS_DEBUG_NAMED("kdl", "Could not find IK solution");
        continue;
      }
    }
    ROS_DEBUG_NAMED("kdl", "Found IK solution");
    for (unsigned int j = 0; j < dimension_; j++)
      solution[j] = jnt_pos_out(j);
    if (!solution_callback.empty())
      solution_callback(ik_pose, solution, error_code);
    else
      error_code.val = error_code.SUCCESS;

    if (error_code.val == error_code.SUCCESS)
    {
      ROS_DEBUG_STREAM_NAMED("kdl", "Solved after " << counter << " iterations");
      ik_solver_vel.unlockRedundantJoints();
      return true;
    }
  }
  ROS_DEBUG_NAMED("kdl", "An IK that satisifes the constraints and is collision free could not be found");
  error_code.val = error_code.NO_IK_SOLUTION;
  ik_solver_vel.unlockRedundantJoints();
  return false;
}

bool KDLKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                        const std::vector<double>& joint_angles,
                                        std::vector<geometry_msgs::Pose>& poses) const
{
  ros::WallTime n1 = ros::WallTime::now();
  if (!active_)
  {
    ROS_ERROR_NAMED("kdl", "kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != dimension_)
  {
    ROS_ERROR_NAMED("kdl", "Joint angles vector must have size: %d", dimension_);
    return false;
  }

  KDL::Frame p_out;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;

  KDL::JntArray jnt_pos_in(dimension_);
  for (unsigned int i = 0; i < dimension_; i++)
  {
    jnt_pos_in(i) = joint_angles[i];
  }

  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);

  bool valid = true;
  for (unsigned int i = 0; i < poses.size(); i++)
  {
    ROS_DEBUG_NAMED("kdl", "End effector index: %d", getKDLSegmentIndex(link_names[i]));
    if (fk_solver.JntToCart(jnt_pos_in, p_out, getKDLSegmentIndex(link_names[i])) >= 0)
    {
      tf::poseKDLToMsg(p_out, poses[i]);
    }
    else
    {
      ROS_ERROR_NAMED("kdl", "Could not compute FK for %s", link_names[i].c_str());
      valid = false;
    }
  }
  return valid;
}

const std::vector<std::string>& KDLKinematicsPlugin::getJointNames() const
{
  return ik_chain_info_.joint_names;
}

const std::vector<std::string>& KDLKinematicsPlugin::getLinkNames() const
{
  return ik_chain_info_.link_names;
}

}  // namespace
