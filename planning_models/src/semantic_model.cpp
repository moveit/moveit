/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: E. Gil Jones */

#include <planning_models/semantic_model.h>

planning_models::SemanticModel::SemanticModel(const planning_models::KinematicModelConstPtr& kmodel,
                                              const boost::shared_ptr<const srdf::Model>& srdf_model) :
  kmodel_(kmodel)
{
  const std::map<std::string, KinematicModel::JointModelGroup*>& joint_model_group_map = kmodel_->getJointModelGroupMap();
  const std::vector<srdf::Model::Group>& srdf_group_vector = srdf_model->getGroups();
  std::map<std::string, const srdf::Model::Group*> srdf_group_map;
  for(unsigned int i = 0; i < srdf_group_vector.size(); i++) {
    srdf_group_map[srdf_group_vector[i].name_] = &srdf_group_vector[i]; 
  }
  std::map<std::string, const srdf::Model::EndEffector*> srdf_end_effector_component_map;
  std::map<std::string, const srdf::Model::EndEffector*> srdf_end_effector_parents_map;
  const std::vector<srdf::Model::EndEffector>& srdf_end_effector_vector = srdf_model->getEndEffectors();
  for(unsigned int i = 0; i < srdf_end_effector_vector.size(); i++) {
    if(srdf_end_effector_parents_map.find(srdf_end_effector_vector[i].parent_link_) != 
       srdf_end_effector_parents_map.end()) {
      ROS_WARN_STREAM("Two end effector groups have the same parent link " << srdf_end_effector_vector[i].parent_link_);
      continue;
    }
    srdf_end_effector_parents_map[srdf_end_effector_vector[i].parent_link_] = &srdf_end_effector_vector[i];
    srdf_end_effector_component_map[srdf_end_effector_vector[i].component_group_] = &srdf_end_effector_vector[i];
  }
  
  std::map<std::string, bool> is_arm_map;
  for(std::map<std::string, KinematicModel::JointModelGroup*>::const_iterator it = joint_model_group_map.begin();
      it != joint_model_group_map.end(); 
      it++) {
    SemanticGroup sg(it->first);
    const srdf::Model::Group* srdf_group = srdf_group_map.at(it->first);
    if(srdf_group->chains_.size() == 1) {
      sg.is_arm_ = true;
      sg.base_link_ = srdf_group->chains_[0].first;
      sg.tip_link_ = srdf_group->chains_[0].second;
      if(srdf_end_effector_parents_map.find(sg.tip_link_) != srdf_end_effector_parents_map.end()) {
        sg.has_end_effector_ = true;
        sg.end_effector_name_ = srdf_end_effector_parents_map.at(sg.tip_link_)->component_group_;
      }
      is_arm_map[it->first] = true;
    } else {
      is_arm_map[it->first] = false;
      if(srdf_end_effector_component_map.find(it->first) != srdf_end_effector_component_map.end()) {
        sg.is_end_effector_ = true;
        sg.attach_link_ = joint_model_group_map.at(it->first)->getLinkModelNames().back();
      }
    }
    //complicated, but gets around calling default constructor
    semantic_group_map_.insert(std::map<std::string, SemanticGroup>::value_type(it->first, sg));
  }
  
  //TODO - multi-arm groups
}

std::vector<std::string> planning_models::SemanticModel::getGroupLinks(const std::string& group_name) const
{
  std::vector<std::string> ret_vec;
  const SemanticGroup* sem = getGroup(group_name);
  if(!sem) {
    return ret_vec;
  }
  const planning_models::KinematicModel::JointModelGroup* jmg = kmodel_->getJointModelGroup(group_name);
  ret_vec = jmg->getLinkModelNames();
  return ret_vec;
}

std::vector<std::string> planning_models::SemanticModel::getGroupJoints(const std::string& group_name) const
{
  std::vector<std::string> ret_vec;
  const SemanticGroup* sem = getGroup(group_name);
  if(!sem) {
    return ret_vec;
  }
  const planning_models::KinematicModel::JointModelGroup* jmg = kmodel_->getJointModelGroup(group_name);
  ret_vec = jmg->getJointModelNames();
  return ret_vec;
}

std::vector<std::string> planning_models::SemanticModel::getEndEffectorLinks(const std::string& group_name) const
{
  std::vector<std::string> ret_vec;
  const SemanticGroup* sem = getGroup(group_name);
  if(!sem) {
    return ret_vec;
  }
  if(sem->has_end_effector_ == false) {
    return ret_vec;
  }
  const planning_models::KinematicModel::JointModelGroup* jmg = kmodel_->getJointModelGroup(sem->end_effector_name_);
  ret_vec = jmg->getLinkModelNames();
  return ret_vec;
}

std::string planning_models::SemanticModel::getAttachLink(const std::string& group_name) const
{
  std::string ret;
  const SemanticGroup* sem = getGroup(group_name);
  if(!sem) {
    return ret;
  }
  if(sem->is_end_effector_ == false) {
    return ret;
  }
  return sem->attach_link_;
}

std::string planning_models::SemanticModel::getTipLink(const std::string& group_name) const
{
  std::string ret;
  const SemanticGroup* sem = getGroup(group_name);
  if(!sem) {
    return ret;
  }
  if(sem->is_arm_ == false) {
    return ret;
  }
  return sem->tip_link_;
}

std::string planning_models::SemanticModel::getBaseLink(const std::string& group_name) const
{
  std::string ret;
  const SemanticGroup* sem = getGroup(group_name);
  if(!sem) {
    return ret;
  }
  if(sem->is_arm_ == false) {
    return ret;
  }
  return sem->base_link_;
}

std::string planning_models::SemanticModel::getEndEffector(const std::string& group_name) const
{
  std::string ret;
  const SemanticGroup* sem = getGroup(group_name);
  if(!sem) {
    return ret;
  }
  if(sem->has_end_effector_ == false) {
    return ret;
  }
  return sem->end_effector_name_;
}

bool planning_models::SemanticModel::isArm(const std::string& group_name) const
{
  const SemanticGroup* sem = getGroup(group_name);
  if(!sem) {
    return false;
  }
  return sem->is_arm_;
}

bool planning_models::SemanticModel::isEndEffector(const std::string& group_name) const
{
  const SemanticGroup* sem = getGroup(group_name);
  if(!sem) {
    return false;
  }
  return sem->is_end_effector_;
}

bool planning_models::SemanticModel::hasEndEffector(const std::string& group_name) const
{
  const SemanticGroup* sem = getGroup(group_name);
  if(!sem) {
    return false;
  }
  return sem->has_end_effector_;
}

const planning_models::SemanticGroup* planning_models::SemanticModel::getGroup(const std::string& group_name) const 
{
  std::map<std::string, SemanticGroup>::const_iterator it = semantic_group_map_.find(group_name);
  if(it == semantic_group_map_.end()) {
    return NULL;
  }
  return &it->second;
}
