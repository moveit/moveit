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

/* Author: Sachin Chitta */

#include <dynamics_solver/dynamics_solver.h>

// KDL
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace dynamics_solver
{
DynamicsSolver::DynamicsSolver()
{
}

bool DynamicsSolver::initialize(const boost::shared_ptr<const urdf::Model> &urdf_model,
                                const boost::shared_ptr<const srdf::Model> &srdf_model,
                                const std::string &group_name)
{
  urdf_model_ = urdf_model;
  srdf_model_ = srdf_model;
  group_name_ = group_name;
  const std::vector<srdf::Model::Group> groups = srdf_model_->getGroups();
  bool found_group = false;    
  std::string base_name,tip_name;
  for(unsigned int i=0; i < groups.size(); i++)
  {
    if(groups[i].name_ == group_name)
    {
      if(groups[i].chains_.empty())
      {
        ROS_ERROR("SRDF does not contain chain information for group. SRDF must contain chain information to setup group.");
        return false;
      }
      base_name = groups[i].chains_[0].first;
      tip_name = groups[i].chains_[0].second;
      found_group = true; // For now assume this is a chain until we get isChain function implemented.
    }      
  }
  if(!found_group)
  {
    ROS_ERROR("Did not find the group %s in SRDF",group_name.c_str());
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*urdf_model_, tree)) 
  {
    ROS_ERROR("Could not initialize tree object");
  }
  if (!tree.getChain(base_name, tip_name, kdl_chain_)) 
  {
    ROS_ERROR("Could not initialize chain object");
  }
  num_joints_ = kdl_chain_.getNrOfJoints();
  max_torques_.resize(num_joints_);

  kinematic_model_.reset(new planning_models::KinematicModel(urdf_model_,srdf_model_));
  if(!kinematic_model_->hasJointModelGroup(group_name))
  {
    ROS_ERROR("Did not find the group %s in robot model",group_name.c_str());
    return false;
  }  
  joint_model_group_ = kinematic_model_->getJointModelGroup(group_name);

  const std::vector<std::string> joint_model_names = joint_model_group_->getJointModelNames();
  for(unsigned int i=0; i < joint_model_names.size(); i++)
    max_torques_.push_back(urdf_model_->getJoint(joint_model_names[i])->limits->effort);

  KDL::Vector gravity(0.0,0.0,9.81);//Not sure if KDL expects the negative of this
  chain_id_solver_.reset(new KDL::ChainIdSolver_RNE(kdl_chain_,gravity));
  return true;
}

bool DynamicsSolver::getTorques(const std::vector<double> &joint_angles,
                                const std::vector<double> &joint_velocities,
                                const std::vector<double> &joint_accelerations,
                                const std::vector<geometry_msgs::Wrench> &wrenches,
                                std::vector<double> &torques) const
{
  if(joint_angles.size() != num_joints_)
  {
    ROS_ERROR("Joint angles vector should be size %d",num_joints_);
    return false;
  }
  if(joint_velocities.size() != num_joints_)
  {
    ROS_ERROR("Joint velocities vector should be size %d",num_joints_);
    return false;
  }
  if(joint_accelerations.size() != num_joints_)
  {
    ROS_ERROR("Joint accelerations vector should be size %d",num_joints_);
    return false;
  }
  if(wrenches.size() != num_joints_)
  {
    ROS_ERROR("Wrenches vector should be size %d",num_joints_);
    return false;
  }
  if(torques.size() != num_joints_)
  {
    ROS_ERROR("Torques vector should be size %d",num_joints_);
    return false;
  }

  KDL::JntArray kdl_angles(num_joints_), kdl_velocities(num_joints_), kdl_accelerations(num_joints_), kdl_torques(num_joints_);
  KDL::Wrenches kdl_wrenches(num_joints_);

  for(unsigned int i=0; i < num_joints_; i++)
  {
    kdl_angles(i) = joint_angles[i];
    kdl_velocities(i) = joint_velocities[i];
    kdl_accelerations(i) = joint_accelerations[i];

    kdl_wrenches[i](0) = wrenches[i].force.x;
    kdl_wrenches[i](1) = wrenches[i].force.y;
    kdl_wrenches[i](2) = wrenches[i].force.z;

    kdl_wrenches[i](3) = wrenches[i].torque.x;
    kdl_wrenches[i](4) = wrenches[i].torque.y;
    kdl_wrenches[i](5) = wrenches[i].torque.z;    
  }
  chain_id_solver_->CartToJnt(kdl_angles,kdl_velocities,kdl_accelerations,kdl_wrenches,kdl_torques);

  for(unsigned int i=0; i < num_joints_; i++)
    torques[i] = kdl_torques(i);

  return true;
}

bool DynamicsSolver::getMaxPayload(const std::vector<double> &joint_angles,
                                   double &payload) const
{
  if(joint_angles.size() != num_joints_)
  {
    ROS_ERROR("Joint angles vector should be size %d",num_joints_);
    return false;
  }
  std::vector<double> joint_velocities(num_joints_,0.0), joint_accelerations(num_joints_,0.0), torques(num_joints_,0.0);
  std::vector<geometry_msgs::Wrench> wrenches(num_joints_);
  wrenches.back().force.z = -1.0;
  if(!getTorques(joint_angles,joint_velocities,joint_accelerations,wrenches,torques))
    return false;
  double multiplier = findMaxTorqueMultiplier(torques);
  payload = fabs(multiplier);
  return true;
}

double DynamicsSolver::findMaxTorqueMultiplier(const std::vector<double> &joint_torques) const
{
  double multiplier = 0.0;
  for(unsigned int i=0; i < num_joints_; i++)
  {
    if(fabs(joint_torques[i]/max_torques_[i]) > multiplier)
      multiplier = fabs(joint_torques[i]/max_torques_[i]);
  }
  if(multiplier == 0.0)
    multiplier = 1.0;
  else
    multiplier = 1.0/multiplier;
  return multiplier;
}
}
/*
int ChainIdSolver_RNE::CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques)
*/
