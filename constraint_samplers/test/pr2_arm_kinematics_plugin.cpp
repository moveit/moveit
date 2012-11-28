/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Sachin Chitta
 */

#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <algorithm>
#include <numeric>

#include "pr2_arm_kinematics_plugin.h"

using namespace KDL;
using namespace std;

namespace pr2_arm_kinematics {

PR2ArmKinematicsPlugin::PR2ArmKinematicsPlugin():active_(false){}

bool PR2ArmKinematicsPlugin::isActive()
{
  if(active_)
    return true;
  return false;
}

void PR2ArmKinematicsPlugin::setRobotModel(boost::shared_ptr<urdf::ModelInterface>& robot_model)
{
  robot_model_ = robot_model;
}

bool PR2ArmKinematicsPlugin::initialize(const std::string& group_name,
                                        const std::string& base_name,
                                        const std::string& tip_name,
                                        double search_discretization)
{
  setValues(group_name, base_name, tip_name,search_discretization);

  std::string xml_string;
  dimension_ = 7;

  ROS_DEBUG("Loading KDL Tree");
  if(!getKDLChain(*robot_model_.get(),base_frame_,tip_frame_,kdl_chain_))
  {
    active_ = false;
    ROS_ERROR("Could not load kdl tree");
  }
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  free_angle_ = 2;

  pr2_arm_ik_solver_.reset(new pr2_arm_kinematics::PR2ArmIKSolver(*robot_model_.get(), base_frame_,tip_frame_, search_discretization_,free_angle_));
  if(!pr2_arm_ik_solver_->active_)
  {
    ROS_ERROR("Could not load ik");
    active_ = false;
  }
  else
  {

    pr2_arm_ik_solver_->getSolverInfo(ik_solver_info_);
    pr2_arm_kinematics::getKDLChainInfo(kdl_chain_,fk_solver_info_);
    fk_solver_info_.joint_names = ik_solver_info_.joint_names;

    for(unsigned int i=0; i < ik_solver_info_.joint_names.size(); i++)
    {
      ROS_DEBUG("PR2Kinematics:: joint name: %s",ik_solver_info_.joint_names[i].c_str());
    }
    for(unsigned int i=0; i < ik_solver_info_.link_names.size(); i++)
    {
      ROS_DEBUG("PR2Kinematics can solve IK for %s",ik_solver_info_.link_names[i].c_str());
    }
    for(unsigned int i=0; i < fk_solver_info_.link_names.size(); i++)
    {
      ROS_DEBUG("PR2Kinematics can solve FK for %s",fk_solver_info_.link_names[i].c_str());
    }
    ROS_DEBUG("PR2KinematicsPlugin::active for %s",group_name.c_str());
    active_ = true;
  }    
  return active_;
}

bool PR2ArmKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code) const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION; 
    return false;
  }
    
  KDL::Frame pose_desired;
  Eigen::Affine3d tp;
  tf::poseMsgToEigen(ik_pose, tp);
  tf::transformEigenToKDL(tp, pose_desired);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  int ik_valid = pr2_arm_ik_solver_->CartToJnt(jnt_pos_in,
                                               pose_desired,
                                               jnt_pos_out);
  if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
  {
    error_code.val = error_code.NO_IK_SOLUTION; 
    return false;
  }

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    error_code.val = error_code.SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    error_code.val = error_code.NO_IK_SOLUTION; 
    return false;
  }
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code) const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.PLANNING_FAILED; 
    return false;
  }
  KDL::Frame pose_desired;
  Eigen::Affine3d tp;
  tf::poseMsgToEigen(ik_pose, tp);
  tf::transformEigenToKDL(tp, pose_desired);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  int ik_valid = pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                     pose_desired,
                                                     jnt_pos_out,
                                                     timeout);
  if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
  {
    error_code.val = error_code.NO_IK_SOLUTION; 
    return false;
  }

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    error_code.val = error_code.SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    error_code.val = error_code.NO_IK_SOLUTION; 
    return false;
  }
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              unsigned int redundancy,
                                              double consistency_limit,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code) const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.PLANNING_FAILED; 
    return false;
  }
  KDL::Frame pose_desired;
  Eigen::Affine3d tp;
  tf::poseMsgToEigen(ik_pose, tp);
  tf::transformEigenToKDL(tp, pose_desired);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  unsigned int old_free_angle = pr2_arm_ik_solver_->getFreeAngle();
  pr2_arm_ik_solver_->setFreeAngle(redundancy);
  int ik_valid = pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                     pose_desired,
                                                     consistency_limit,
                                                     jnt_pos_out,
                                                     timeout);
  pr2_arm_ik_solver_->setFreeAngle(old_free_angle);

  if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
  {
    error_code.val = error_code.NO_IK_SOLUTION; 
    return false;
  }

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    error_code.val = error_code.SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    error_code.val = error_code.NO_IK_SOLUTION; 
    return false;
  }
}


void PR2ArmKinematicsPlugin::desiredPoseCallback(const KDL::JntArray& jnt_array, 
                                                 const KDL::Frame& ik_pose,
                                                 moveit_msgs::MoveItErrorCodes& error_code) const
{
  std::vector<double> ik_seed_state;
  ik_seed_state.resize(dimension_);
  for(int i=0; i < dimension_; i++)
    ik_seed_state[i] = jnt_array(i);

  geometry_msgs::Pose ik_pose_msg;
  Eigen::Affine3d tp;
  tf::transformKDLToEigen(ik_pose, tp);
  tf::poseEigenToMsg(tp, ik_pose_msg);

  desiredPoseCallback_(ik_pose_msg,ik_seed_state,error_code);
}


void PR2ArmKinematicsPlugin::jointSolutionCallback(const KDL::JntArray& jnt_array, 
                                                   const KDL::Frame& ik_pose,
                                                   moveit_msgs::MoveItErrorCodes& error_code) const
{
  std::vector<double> ik_seed_state;
  ik_seed_state.resize(dimension_);
  for(int i=0; i < dimension_; i++)
    ik_seed_state[i] = jnt_array(i);

  geometry_msgs::Pose ik_pose_msg;
  Eigen::Affine3d tp;
  tf::transformKDLToEigen(ik_pose, tp);
  tf::poseEigenToMsg(tp, ik_pose_msg);

  solutionCallback_(ik_pose_msg,ik_seed_state,error_code);
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code) const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.PLANNING_FAILED;
    return false;
  }
  KDL::Frame pose_desired;
  Eigen::Affine3d tp;
  tf::poseMsgToEigen(ik_pose, tp);
  tf::transformEigenToKDL(tp, pose_desired);

  //  desiredPoseCallback_ = desired_pose_callback;
  //  solutionCallback_    = solution_callback;

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  int ik_valid = pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                     pose_desired,
                                                     jnt_pos_out,
                                                     timeout,
                                                     error_code,
                                                     boost::bind(solution_callback,_1, _2, _3),
                                                     boost::bind(solution_callback, _1, _2, _3));
  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    return false;
  }
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              unsigned int redundancy,
                                              double consistency_limit,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code) const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.PLANNING_FAILED;
    return false;
  }
  KDL::Frame pose_desired;

  Eigen::Affine3d tp;
  tf::poseMsgToEigen(ik_pose, tp);
  tf::transformEigenToKDL(tp, pose_desired);

  //  desiredPoseCallback_ = desired_pose_callback;
  //  solutionCallback_    = solution_callback;

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  unsigned int old_free_angle = pr2_arm_ik_solver_->getFreeAngle();
  pr2_arm_ik_solver_->setFreeAngle(redundancy);
  int ik_valid = pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                     pose_desired,
                                                     jnt_pos_out,
                                                     timeout,
                                                     consistency_limit,
                                                     error_code,
                                                     boost::bind(solution_callback, _1, _2, _3),
                                                     boost::bind(solution_callback, _1, _2, _3));
  pr2_arm_ik_solver_->setFreeAngle(old_free_angle);
  if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
    return false;

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    return false;
  }
}

bool PR2ArmKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                           const std::vector<double> &joint_angles,
                                           std::vector<geometry_msgs::Pose> &poses) const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    return false;
  }

  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;

  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = joint_angles[i];
    ROS_DEBUG("Joint angle: %d %f",i,joint_angles[i]);
  }

  poses.resize(link_names.size());

  bool valid = true;
  for(unsigned int i=0; i < poses.size(); i++)
  {
    ROS_DEBUG("End effector index: %d",pr2_arm_kinematics::getKDLSegmentIndex(kdl_chain_,link_names[i]));
    if(jnt_to_pose_solver_->JntToCart(jnt_pos_in,p_out,pr2_arm_kinematics::getKDLSegmentIndex(kdl_chain_,link_names[i])) >=0)
    {
      Eigen::Affine3d tp;
      tf::transformKDLToEigen(p_out, tp);
      tf::poseEigenToMsg(tp, poses[i]);
    }
    else
    {
      ROS_ERROR("Could not compute FK for %s",link_names[i].c_str());
      valid = false;
    }
  }
  return valid;
}

const std::vector<std::string>& PR2ArmKinematicsPlugin::getJointNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return ik_solver_info_.joint_names;
}

const std::vector<std::string>& PR2ArmKinematicsPlugin::getLinkNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return fk_solver_info_.link_names;
}

} // namespace
