/*********************************************************************
*
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
*
* Author: Sachin Chitta, David Lu!!, Ugo Cupcic
*********************************************************************/

#include <kdl_kinematics_plugin/kdl_kinematics_plugin.h>
#include <pluginlib/class_list_macros.h>

//#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl_parser/kdl_parser.hpp>

// URDF, SRDF
#include <urdf_interface/model.h>
#include <urdf/model.h>
#include <srdf/model.h>

// Kinematic model
#include <planning_models/kinematic_model.h>

static const double MAX_TIMEOUT_KDL_PLUGIN = 5.0;
 
//register KDLKinematics as a KinematicsBase implementation
PLUGINLIB_DECLARE_CLASS(kdl_kinematics_plugin,KDLKinematicsPlugin, kdl_kinematics_plugin::KDLKinematicsPlugin, kinematics::KinematicsBase)

namespace kdl_kinematics_plugin
{

KDLKinematicsPlugin::KDLKinematicsPlugin():active_(false){}


void KDLKinematicsPlugin::getRandomConfiguration(KDL::JntArray &jnt_array) const
{
  for(unsigned int i=0; i < dimension_; ++i)
    jnt_array(i) = random_number_generator_.uniformReal(joint_min_(i),joint_max_(i));
}

void KDLKinematicsPlugin::getRandomConfiguration(const KDL::JntArray& seed_state,
                                                 const unsigned int& redundancy,
                                                 const double& consistency_limit,
                                                 KDL::JntArray& jnt_array) const
{
  for(unsigned int i=0; i < dimension_; ++i) {
    if(i != redundancy) {
      jnt_array(i) = random_number_generator_.uniformReal(joint_min_(i),joint_max_(i));
    } else {
      double jmin = fmin(joint_min_(i), seed_state(i)-consistency_limit);
      double jmax = fmax(joint_max_(i), seed_state(i)+consistency_limit);
      jnt_array(i) = random_number_generator_.uniformReal(jmin, jmax);
    }
  }
}

bool KDLKinematicsPlugin::checkConsistency(const KDL::JntArray& seed_state,
                                           const unsigned int& redundancy,
                                           const double& consistency_limit,
                                           const KDL::JntArray& solution) const
{
  if (redundancy >= dimension_)
    return false;
  double jmin = fmin(joint_min_(redundancy), seed_state(redundancy)-consistency_limit);
  double jmax = fmax(joint_max_(redundancy), seed_state(redundancy)+consistency_limit);
  if(solution(redundancy) < jmin || solution(redundancy) > jmax) 
    return false;
  return true;
}

bool KDLKinematicsPlugin::initialize(const std::string& group_name,
                                     const std::string& base_frame,
                                     const std::string& tip_frame,
                                     double search_discretization)
{
  ROS_INFO("Initializing kdl solver");  
  setValues(group_name, base_frame, tip_frame, search_discretization);

  ros::NodeHandle private_handle("~");  
  planning_models::KinematicModelPtr kinematic_model;
  const boost::shared_ptr<srdf::Model> &srdf = robot_model_loader_.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = robot_model_loader_.getURDF();

  kinematic_model.reset(new planning_models::KinematicModel(urdf_model, srdf));

  if(!kinematic_model->hasJointModelGroup(group_name))
  {
    ROS_ERROR("Kinematic model does not contain group %s",group_name.c_str());
    return false;
  }  
  planning_models::KinematicModel::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_name);
  if(!joint_model_group->isChain())
  {
    ROS_ERROR("Group is not a chain");
    return false;
  }
  
  KDL::Tree kdl_tree;
  
  if (!kdl_parser::treeFromUrdfModel((const urdf::Model&) *urdf_model, kdl_tree)) 
  {
    ROS_ERROR("Could not initialize tree object");
    return false;
  }
  if (!kdl_tree.getChain(base_frame_, tip_frame_, kdl_chain_)) 
  {
    ROS_ERROR("Could not initialize chain object");
    return false;
  }

  dimension_ = joint_model_group->getVariableCount();
  jnt_seed_state_.resize(dimension_);
  jnt_pos_in_.resize(dimension_);
  jnt_pos_out_.resize(dimension_);
  ik_chain_info_.joint_names = joint_model_group->getJointModelNames();
  ik_chain_info_.limits = joint_model_group->getVariableLimits();   
  fk_chain_info_.joint_names = ik_chain_info_.joint_names;
  fk_chain_info_.limits = ik_chain_info_.limits;  

  if(!joint_model_group->hasLinkModel(tip_frame_))
  {
    ROS_ERROR("Could not find tip name in joint group");
    return false;    
  }
  ik_chain_info_.link_names.push_back(tip_frame_);
  fk_chain_info_.link_names = joint_model_group->getLinkModelNames();
  
  joint_min_.resize(ik_chain_info_.limits.size());
  joint_max_.resize(ik_chain_info_.limits.size());
  
  for(unsigned int i=0; i < ik_chain_info_.limits.size(); i++)
  {
    joint_min_(i) = ik_chain_info_.limits[i].min_position;
    joint_max_(i) = ik_chain_info_.limits[i].max_position;    
  }
  
  // Get Solver Parameters
  int max_solver_iterations;
  double epsilon;

  private_handle.param("max_solver_iterations", max_solver_iterations, 500);
  private_handle.param("max_search_iterations", max_search_iterations_, 1000);
  private_handle.param("epsilon", epsilon, 1e-5);

  // Build Solvers
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  ik_solver_vel_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
  ik_solver_pos_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, joint_min_, joint_max_,*fk_solver_, *ik_solver_vel_, max_solver_iterations, epsilon));

  active_ = true;  
  ROS_INFO("KDL solver initialized");  
  return true;
}

int KDLKinematicsPlugin::getJointIndex(const std::string &name) const
{
  for (unsigned int i=0; i < ik_chain_info_.joint_names.size(); i++) {
    if (ik_chain_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

int KDLKinematicsPlugin::getKDLSegmentIndex(const std::string &name) const
{
  int i=0; 
  while (i < (int)kdl_chain_.getNrOfSegments()) {
    if (kdl_chain_.getSegment(i).getName() == name) {
      return i+1;
    }
    i++;
  }
  return -1;
}

bool KDLKinematicsPlugin::timedOut(const ros::WallTime &start_time, double duration) const
{
  return ((ros::WallTime::now()-start_time).toSec() >= duration);  
}

bool KDLKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                        const std::vector<double> &ik_seed_state,
                                        std::vector<double> &solution,
                                        moveit_msgs::MoveItErrorCodes &error_code) const
{
  const IKCallbackFn desired_pose_callback = 0;
  const IKCallbackFn solution_callback = 0;  

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          MAX_TIMEOUT_KDL_PLUGIN,
                          solution,
                          desired_pose_callback,
                          solution_callback,
                          error_code,
                          1,
                          false);
}

bool KDLKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code) const
{
  const IKCallbackFn desired_pose_callback = 0;
  const IKCallbackFn solution_callback = 0;  

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          desired_pose_callback,
                          solution_callback,
                          error_code,
                          max_search_iterations_,
                          false);
}
    
bool KDLKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           unsigned int redundancy,
                                           double consistency_limit,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code) const
{
  const IKCallbackFn desired_pose_callback = 0;
  const IKCallbackFn solution_callback = 0;  

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          desired_pose_callback,
                          solution_callback,
                          error_code,
                          max_search_iterations_,
                          true,
                          redundancy,
                          consistency_limit);
}

bool KDLKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &desired_pose_callback,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code) const
{
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          desired_pose_callback,
                          solution_callback,
                          error_code,
                          max_search_iterations_,
                          false);
}

bool KDLKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           unsigned int redundancy,
                                           double consistency_limit,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &desired_pose_callback,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code) const
{
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          desired_pose_callback,
                          solution_callback,
                          error_code,
                          max_search_iterations_,
                          true,
                          redundancy,
                          consistency_limit);
}

bool KDLKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &desired_pose_callback,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           unsigned int max_search_iterations,
                                           bool check_consistency,
                                           unsigned int redundancy,
                                           double consistency_limit) const
{
  ros::WallTime n1 = ros::WallTime::now();
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if(solution.size() != dimension_)
  {
    ROS_ERROR("Solution vector must have size: %d",dimension_);
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;    
  }
  
  KDL::Frame pose_desired;
  tf::PoseMsgToKDL(ik_pose, pose_desired);

  ROS_DEBUG_STREAM("searchPositionIK2: Position request pose is " <<
                   ik_pose.position.x << " " <<
                   ik_pose.position.y << " " <<
                   ik_pose.position.z << " " <<
                   ik_pose.orientation.x << " " << 
                   ik_pose.orientation.y << " " << 
                   ik_pose.orientation.z << " " << 
                   ik_pose.orientation.w);

  //Do the IK
  for(unsigned int i=0; i < dimension_; i++)
    jnt_seed_state_(i) = ik_seed_state[i]; 
  jnt_pos_in_ = jnt_seed_state_;

  if(!desired_pose_callback.empty())
  {
    desired_pose_callback(ik_pose,ik_seed_state,error_code);
    if(error_code.val != error_code.SUCCESS)
    {
      ROS_DEBUG("Could not find inverse kinematics for desired end-effector pose since the pose may be in collision");
      return false;
    }
  }
  
  for(int i=0; i < (int) max_search_iterations; i++)
  {
    int ik_valid = ik_solver_pos_->CartToJnt(jnt_pos_in_,pose_desired,jnt_pos_out_);                     
    if(check_consistency) 
    {
      getRandomConfiguration(jnt_seed_state_, redundancy, consistency_limit, jnt_pos_in_);
      if(ik_valid < 0 || !checkConsistency(jnt_seed_state_, redundancy, consistency_limit, jnt_pos_out_))
        continue;
    }
    else
    {
      getRandomConfiguration(jnt_pos_in_);
      if(ik_valid < 0)
        continue;
    }
    
    for(unsigned int j=0; j < dimension_; j++)
      solution[j] = jnt_pos_out_(j);
    if(!solution_callback.empty())
      solution_callback(ik_pose,solution,error_code);
    else
      error_code.val = error_code.SUCCESS;
    
    if(error_code.val == error_code.SUCCESS)
    {
      ROS_DEBUG_STREAM("Solved after " << i+1 << " iterations");
      return true;
    }
    if(timedOut(n1,timeout))
    {
      ROS_ERROR("IK timed out");
      error_code.val = error_code.TIMED_OUT;
      return false;      
    }    
  }
  ROS_DEBUG("An IK that satisifes the constraints and is collision free could not be found");   
  error_code.val = error_code.NO_IK_SOLUTION;
  return false;
}

bool KDLKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                        const std::vector<double> &joint_angles,
                                        std::vector<geometry_msgs::Pose> &poses) const
{
  ros::WallTime n1 = ros::WallTime::now();
  if(!active_)
  {
    ROS_ERROR("kinematics not active");    
    return false;
  }
  if(poses.size() != link_names.size())
  {
    ROS_ERROR("Poses vector must have size: %zu",link_names.size());
    return false;    
  }
  if(joint_angles.size() != dimension_)
  {
    ROS_ERROR("Joint angles vector must have size: %d",dimension_);
    return false;    
  }
  
  KDL::Frame p_out;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;
  
  for(unsigned int i=0; i < dimension_; i++)
  {
    jnt_pos_in_(i) = joint_angles[i];
  }
  
  bool valid = true;
  for(unsigned int i=0; i < poses.size(); i++)
  {
    ROS_DEBUG("End effector index: %d",getKDLSegmentIndex(link_names[i]));
    if(fk_solver_->JntToCart(jnt_pos_in_,p_out,getKDLSegmentIndex(link_names[i])) >=0)
    {
      tf::PoseKDLToMsg(p_out,poses[i]);
    }
    else
    {
      ROS_ERROR("Could not compute FK for %s",link_names[i].c_str());
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

} // namespace
