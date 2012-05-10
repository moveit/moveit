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
* Author: Sachin Chitta
*********************************************************************/

#include <jacobian_solver/jacobian_solver.h>
#include <ros/console.h>

namespace jacobian_solver 
{
JacobianSolver::JacobianSolver()
{
  //  srand ( time(NULL) );
}

bool JacobianSolver::initialize(const boost::shared_ptr<const urdf::Model> &urdf_model,
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
    kinematic_model_.reset(new planning_models::KinematicModel(urdf_model,srdf_model));
    if(!kinematic_model_->hasJointModelGroup(group_name))
    {
      ROS_ERROR("Did not find the group %s in robot model",group_name.c_str());
      return false;
    }
    kinematic_model_const_ = kinematic_model_;

    kinematic_state_.reset(new planning_models::KinematicState(kinematic_model_));
    kinematic_state_->setToDefaultValues();
    joint_model_group_ = kinematic_model_->getJointModelGroup(group_name);
    joint_state_group_ = kinematic_state_->getJointStateGroup(group_name);
    const std::vector<const planning_models::KinematicModel::JointModel*> roots = joint_model_group_->getJointRoots();
    if(roots.size() > 1)
    {
      ROS_ERROR("This solver only works on chains");
      return false;
    }
    root_joint_model_ = roots[0];
    ROS_DEBUG_STREAM("ROOT_LINK" << root_joint_model_->getParentLinkModel()->getName());
    root_link_state_ = kinematic_state_->getLinkState(root_joint_model_->getParentLinkModel()->getName());
    reference_transform_ = root_link_state_ ? root_link_state_->getGlobalLinkTransform() : kinematic_state_->getRootTransform();
    reference_transform_ = reference_transform_.inverse();

    const std::vector<const planning_models::KinematicModel::JointModel*> joint_models = joint_model_group_->getJointModels();//assumption that this comes in the order base-tip;
    unsigned int index =0;
    num_joints_chain_ = joint_model_group_->getVariableCount();
    for(unsigned int i=0; i < joint_models.size(); i++)
    {
      if(joint_models[i]->getType() == planning_models::KinematicModel::JointModel::REVOLUTE || joint_models[i]->getType() == planning_models::KinematicModel::JointModel::PRISMATIC || joint_models[i]->getType() == planning_models::KinematicModel::JointModel::PLANAR)
      {
        joint_index_map_[joint_models[i]->getName()] = index;
        index += joint_models[i]->getVariableCount();
      }
    }
    group_link_names_ = joint_model_group_->getLinkModelNames();
    updated_link_names_ = joint_model_group_->getUpdatedLinkModelNames();
    ROS_DEBUG("Root joint : %s",root_joint_model_->getName().c_str());
    return true;
}

bool JacobianSolver::getJacobian(const std::string &link_name,
                                 const std::vector<double> &joint_angles, 
                                 const Eigen::Vector3d &reference_point_position, 
                                 Eigen::MatrixXd& jacobian) const
{
  if(std::find(group_link_names_.begin(),group_link_names_.end(),link_name) == group_link_names_.end() && std::find(updated_link_names_.begin(),updated_link_names_.end(),link_name) == updated_link_names_.end())
  {
    ROS_ERROR("Link name does not exist in this chain or is not a child for this chain");
    return false;
  }
  jacobian = Eigen::MatrixXd::Zero(6, num_joints_chain_);
  joint_state_group_->setStateValues(joint_angles);
  const planning_models::KinematicState::LinkState *link_state = kinematic_state_->getLinkState(link_name);
  Eigen::Affine3d link_transform = reference_transform_*link_state->getGlobalLinkTransform();
  Eigen::Vector3d point_transform = link_transform*reference_point_position;

  ROS_DEBUG("Point from reference origin expressed in world coordinates: %f %f %f",
            point_transform.x(),
            point_transform.y(),
            point_transform.z());

  Eigen::Vector3d joint_axis;
  Eigen::Affine3d joint_transform;

  while(link_state)
  {
    ROS_DEBUG("Link: %s, %f %f %f",link_state->getName().c_str(),
             link_state->getGlobalLinkTransform().translation().x(),
             link_state->getGlobalLinkTransform().translation().y(),
             link_state->getGlobalLinkTransform().translation().z());    
    ROS_DEBUG("Joint: %s",link_state->getParentJointState()->getName().c_str());
    if(link_state->getParentJointState()->getJointModel()->getType() == planning_models::KinematicModel::JointModel::REVOLUTE)
    {
      unsigned int joint_index = joint_index_map_.find(link_state->getParentJointState()->getJointModel()->getName())->second;
      joint_transform = reference_transform_*link_state->getGlobalLinkTransform();
      joint_axis = joint_transform.rotation()*(dynamic_cast<const planning_models::KinematicModel::RevoluteJointModel*>(link_state->getParentJointState()->getJointModel()))->getAxis();
      jacobian.block<3,1>(0,joint_index) = joint_axis.cross(point_transform - joint_transform.translation());
      jacobian.block<3,1>(3,joint_index) = joint_axis;
    }
    if(link_state->getParentJointState()->getJointModel()->getType() == planning_models::KinematicModel::JointModel::PRISMATIC)
    {
     unsigned int joint_index = joint_index_map_.find(link_state->getParentJointState()->getJointModel()->getName())->second;
      joint_transform = reference_transform_*link_state->getGlobalLinkTransform();
      joint_axis = joint_transform*(dynamic_cast<const planning_models::KinematicModel::PrismaticJointModel*>(link_state->getParentJointState()->getJointModel()))->getAxis();
      jacobian.block<3,1>(0,joint_index) = joint_axis;
    }
    if(link_state->getParentJointState()->getJointModel()->getType() == planning_models::KinematicModel::JointModel::PLANAR)
    {
     unsigned int joint_index = joint_index_map_.find(link_state->getParentJointState()->getJointModel()->getName())->second;
      joint_transform = reference_transform_*link_state->getGlobalLinkTransform();
      joint_axis = joint_transform*Eigen::Vector3d(1.0,0.0,0.0);
      jacobian.block<3,1>(0,joint_index) = joint_axis;
      joint_axis = joint_transform*Eigen::Vector3d(0.0,1.0,0.0);
      jacobian.block<3,1>(0,joint_index+1) = joint_axis;
      joint_axis = joint_transform*Eigen::Vector3d(0.0,0.0,1.0);
      jacobian.block<3,1>(0,joint_index+2) = joint_axis.cross(point_transform - joint_transform.translation());
      jacobian.block<3,1>(3,joint_index+2) = joint_axis;
    }
    if(link_state->getParentJointState()->getJointModel() == root_joint_model_)
      break;
    link_state = link_state->getParentLinkState();
  }
  return true;
}

}
