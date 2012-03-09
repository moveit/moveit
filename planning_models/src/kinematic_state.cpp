/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
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

/* Author: Ioan Sucan, E. Gil Jones */

#include <planning_models/kinematic_state.h>
#include <geometric_shapes/shape_operations.h>
#include <planning_models/transforms.h>

planning_models::KinematicState::KinematicState(const KinematicModelConstPtr &kinematic_model) : kinematic_model_(kinematic_model)
{
  root_transform_.setIdentity();
  buildState();
}

random_numbers::RandomNumberGenerator& planning_models::KinematicState::getRandomNumberGenerator(void)
{
  if (!rng_)
    rng_.reset(new random_numbers::RandomNumberGenerator());
  return *rng_;
}

void planning_models::KinematicState::buildState(void)
{
  const std::vector<KinematicModel::JointModel*>& joint_model_vector = kinematic_model_->getJointModels();
  joint_state_vector_.resize(joint_model_vector.size());
  
  // create joint states
  for (unsigned int i = 0; i < joint_model_vector.size(); ++i)
  {
    joint_state_vector_[i] = new JointState(joint_model_vector[i]);
    joint_state_map_[joint_state_vector_[i]->getName()] = joint_state_vector_[i];
  }
  
  // create link states
  const std::vector<KinematicModel::LinkModel*>& link_model_vector = kinematic_model_->getLinkModels();
  link_state_vector_.resize(link_model_vector.size());
  for (unsigned int i = 0; i < link_model_vector.size(); ++i)
  {
    link_state_vector_[i] = new LinkState(this, link_model_vector[i]);
    link_state_map_[link_state_vector_[i]->getName()] = link_state_vector_[i];
  }
  
  // now we need to figure out who are the link parents are
  for(unsigned int i = 0; i < link_state_vector_.size(); ++i)
  {
    const KinematicModel::JointModel* parent_joint_model = link_state_vector_[i]->getLinkModel()->getParentJointModel();
    link_state_vector_[i]->parent_joint_state_ = joint_state_map_[parent_joint_model->getName()];
    if (parent_joint_model->getParentLinkModel() != NULL)
      link_state_vector_[i]->parent_link_state_ = link_state_map_[parent_joint_model->getParentLinkModel()->getName()];
  }
  
  // compute mimic joint state pointers
  for (std::size_t i = 0; i < joint_state_vector_.size(); ++i)
  {
    const std::vector<const KinematicModel::JointModel*> &mr = joint_state_vector_[i]->joint_model_->getMimicRequests();
    for (std::size_t j = 0 ; j < mr.size() ; ++j)
      joint_state_vector_[i]->mimic_requests_.push_back(joint_state_map_[mr[j]->getName()]);
  }
  
  // now make joint_state_groups
  const std::map<std::string,KinematicModel::JointModelGroup*>& joint_model_group_map = kinematic_model_->getJointModelGroupMap();
  for (std::map<std::string,KinematicModel::JointModelGroup*>::const_iterator it = joint_model_group_map.begin();
       it != joint_model_group_map.end(); ++it)
    joint_state_group_map_[it->first] = new JointStateGroup(this, it->second);
}

planning_models::KinematicState::KinematicState(const KinematicState &ks)
{
  copyFrom(ks);
}

planning_models::KinematicState& planning_models::KinematicState::operator=(const KinematicState &other)
{
  copyFrom(other);
  return *this;
}

void planning_models::KinematicState::copyFrom(const KinematicState &ks)
{
  kinematic_model_ = ks.getKinematicModel();
  root_transform_ = ks.root_transform_;
  
  // construct state
  buildState();
  // copy attached bodies
  for (unsigned int i = 0 ; i < ks.link_state_vector_.size() ; ++i)
  {
    std::vector<const AttachedBody*> ab;
    ks.link_state_vector_[i]->getAttachedBodies(ab);
    LinkState *ls = link_state_map_[ks.link_state_vector_[i]->getName()];
    for (std::size_t j = 0 ; j < ab.size() ; ++j)
      ls->attachBody(ab[j]->properties_);
  }
  std::map<std::string, double> current_joint_values;
  ks.getStateValues(current_joint_values);
  setStateValues(current_joint_values);
}

planning_models::KinematicState::~KinematicState(void)
{
  for(unsigned int i = 0; i < joint_state_vector_.size(); i++)
    delete joint_state_vector_[i];
  for(unsigned int i = 0; i < link_state_vector_.size(); i++)
    delete link_state_vector_[i];
  for (std::map<std::string, JointStateGroup*>::iterator it = joint_state_group_map_.begin();
       it != joint_state_group_map_.end(); ++it)
    delete it->second;
}

bool planning_models::KinematicState::setStateValues(const std::vector<double>& joint_state_values)
{
  if (joint_state_values.size() != getVariableCount())
  {
    ROS_ERROR("Incorrect variable count specified for array of joint values. Expected %u but got %u values",
              getVariableCount(), (int)joint_state_values.size());
    return false;
  }
  
  unsigned int value_counter = 0;
  for(unsigned int i = 0; i < joint_state_vector_.size(); i++)
  {
    unsigned int dim = joint_state_vector_[i]->getVariableCount();
    if (dim != 0 && joint_state_vector_[i]->getJointModel()->getMimic() == NULL)
    {
      joint_state_vector_[i]->setVariableValues(&joint_state_values[value_counter]);
      value_counter += dim;
    }
  }
  updateLinkTransforms();
  return true;
}

void planning_models::KinematicState::setStateValues(const std::map<std::string, double>& joint_state_map)
{
  for (unsigned int i = 0 ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->setVariableValues(joint_state_map);
  updateLinkTransforms();
}

void planning_models::KinematicState::setStateValues(const std::map<std::string, double>& joint_state_map, std::vector<std::string>& missing)
{
  missing.clear();
  for(unsigned int i = 0 ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->setVariableValues(joint_state_map, missing);
  updateLinkTransforms();
}

void planning_models::KinematicState::setStateValues(const sensor_msgs::JointState& js)
{
  std::map<std::string, double> vals;
  unsigned int position_size = js.position.size();
  for(unsigned int i = 0 ; i < js.name.size() ; ++i)
    if (i < position_size)
      vals[js.name[i]] = js.position[i];
  setStateValues(vals);
}

void planning_models::KinematicState::setStateValues(const std::vector<std::string>& joint_names,
                                                     const std::vector<double>& joint_values)
{
  std::map<std::string, double> vals;
  unsigned int position_size = joint_values.size();
  for(unsigned int i = 0 ; i < joint_names.size() ; ++i)
    if (i < position_size)
      vals[joint_names[i]] = joint_values[i];
  setStateValues(vals);
}

void planning_models::KinematicState::getStateValues(std::vector<double>& joint_state_values) const
{
  joint_state_values.clear();
  for(unsigned int i = 0; i < joint_state_vector_.size(); i++)
  {
    const std::vector<double> &jv = joint_state_vector_[i]->getVariableValues();
    joint_state_values.insert(joint_state_values.end(), jv.begin(), jv.end());
  }
}

void planning_models::KinematicState::getStateValues(std::map<std::string,double>& joint_state_values) const
{
  joint_state_values.clear();
  for (std::size_t i = 0; i < joint_state_vector_.size(); ++i)
  {
    const std::vector<double> &jsv = joint_state_vector_[i]->getVariableValues();
    const std::vector<std::string> &jsn = joint_state_vector_[i]->getVariableNames();
    for (std::size_t j = 0 ; j < jsv.size(); ++j)
      joint_state_values[jsn[j]] = jsv[j];
  }
}

void planning_models::KinematicState::getStateValues(sensor_msgs::JointState& js) const
{
  std::map<std::string, double> joint_state_values;
  getStateValues(joint_state_values);
  js.name.resize(joint_state_values.size());
  js.position.resize(joint_state_values.size());
  
  unsigned int i = 0;
  for(std::map<std::string, double>::iterator it = joint_state_values.begin() ; it != joint_state_values.end() ; ++it, ++i)
  {
    js.name[i] = it->first;
    js.position[i] = it->second;
  }
}

void planning_models::KinematicState::updateLinkTransforms(void)
{
  for(unsigned int i = 0; i < link_state_vector_.size(); i++)
    link_state_vector_[i]->computeTransform();
}

bool planning_models::KinematicState::updateStateWithLinkAt(const std::string& link_name, const Eigen::Affine3d& transform)
{
  if (!hasLinkState(link_name))
    return false;
  link_state_map_[link_name]->updateGivenGlobalLinkTransform(transform);
  std::vector<const KinematicModel::LinkModel*> child_link_models;
  kinematic_model_->getChildLinkModels(kinematic_model_->getLinkModel(link_name), child_link_models);
  // the zeroith link will be the link itself, which shouldn't be getting updated, so we start at 1
  for(unsigned int i = 1 ; i < child_link_models.size() ; ++i)
    link_state_map_[child_link_models[i]->getName()]->computeTransform();
  return true;
}

const Eigen::Affine3d& planning_models::KinematicState::getRootTransform(void) const
{
  return root_transform_;
}

void planning_models::KinematicState::setRootTransform(const Eigen::Affine3d &transform)
{
  root_transform_ = transform;
}

void planning_models::KinematicState::setToDefaultValues(void)
{
  std::vector<double> default_joint_states;
  kinematic_model_->getDefaultValues(default_joint_states);
  setStateValues(default_joint_states);
}

void planning_models::KinematicState::setToRandomValues(void)
{
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  std::vector<double> random_joint_states;
  kinematic_model_->getRandomValues(rng, random_joint_states);
  setStateValues(random_joint_states);
}

bool planning_models::KinematicState::satisfiesBounds(const std::string& joint) const
{
  std::vector<std::string> j(1, joint);
  return satisfiesBounds(j);
}

bool planning_models::KinematicState::satisfiesBounds(const std::vector<std::string>& joints) const
{
  for (std::vector<std::string>::const_iterator it = joints.begin(); it != joints.end(); ++it)
  {
    const JointState* joint_state = getJointState(*it);
    if(joint_state == NULL)
    {
      ROS_WARN_STREAM("No joint with name " << *it);
      return false;
    }
    if (!joint_state->satisfiesBounds())
      return false;
  }
  return true;
}

const planning_models::KinematicState::JointStateGroup* planning_models::KinematicState::getJointStateGroup(const std::string &name) const
{
  if(joint_state_group_map_.find(name) == joint_state_group_map_.end()) return NULL;
  return joint_state_group_map_.find(name)->second;
}

planning_models::KinematicState::JointStateGroup* planning_models::KinematicState::getJointStateGroup(const std::string &name)
{
  if(joint_state_group_map_.find(name) == joint_state_group_map_.end()) return NULL;
  return joint_state_group_map_.find(name)->second;
}

bool planning_models::KinematicState::hasJointStateGroup(const std::string &name) const
{
  return joint_state_group_map_.find(name) != joint_state_group_map_.end();
}

void planning_models::KinematicState::getJointStateGroupNames(std::vector<std::string>& names) const
{
  for(std::map<std::string, JointStateGroup*>::const_iterator it = joint_state_group_map_.begin();
      it != joint_state_group_map_.end();
      it++) {
    names.push_back(it->first);
  }
}

bool planning_models::KinematicState::hasJointState(const std::string &joint) const
{
  return joint_state_map_.find(joint) != joint_state_map_.end();
}

bool planning_models::KinematicState::hasLinkState(const std::string& link) const
{
  return link_state_map_.find(link) != link_state_map_.end();
}

planning_models::KinematicState::JointState* planning_models::KinematicState::getJointState(const std::string &name) const
{
  std::map<std::string, JointState*>::const_iterator it = joint_state_map_.find(name);
  if (it == joint_state_map_.end())
  {
    ROS_ERROR("Joint state '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

planning_models::KinematicState::LinkState* planning_models::KinematicState::getLinkState(const std::string &name) const
{
  std::map<std::string, LinkState*>::const_iterator it = link_state_map_.find(name);
  if (it == link_state_map_.end())
  {
    ROS_ERROR("Link state '%s' not found", name.c_str());
    return NULL; 
  }
  else
    return it->second;
}

bool planning_models::KinematicState::hasAttachedBody(const std::string &id) const
{
  return attached_body_map_.find(id) != attached_body_map_.end();
}

const planning_models::KinematicState::AttachedBody* planning_models::KinematicState::getAttachedBody(const std::string &id) const
{
  std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.find(id);
  if (it == attached_body_map_.end())
  {
    ROS_ERROR("Attached body '%s' not found", id.c_str());
    return NULL;
  }
  else
    return it->second;
}

void planning_models::KinematicState::getAttachedBodies(std::vector<const AttachedBody*> &attached_bodies) const
{
  attached_bodies.clear();
  attached_bodies.reserve(attached_body_map_.size());
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
    attached_bodies.push_back(it->second);
}

void planning_models::KinematicState::clearAttachedBodies(void)
{
  attached_body_map_.clear();
  for (std::size_t i = 0 ; i < link_state_vector_.size() ; ++i)
    link_state_vector_[i]->clearAttachedBodies();
}

//-------------------- JointState ---------------------

planning_models::KinematicState::JointState::JointState(const planning_models::KinematicModel::JointModel *jm) : joint_model_(jm)
{
  joint_state_values_.resize(getVariableCount());
  
  variable_transform_.setIdentity();
  std::vector<double> values;
  joint_model_->getDefaultValues(values);
  setVariableValues(values);
}

planning_models::KinematicState::JointState::~JointState(void)
{
}

bool planning_models::KinematicState::JointState::setVariableValue(const std::string &variable, double value)
{
  std::map<std::string, unsigned int>::const_iterator it = getVariableIndexMap().find(variable);
  if (it != getVariableIndexMap().end())
  {
    joint_state_values_[it->second] = value;
    updateMimicJoints();
    return true;
  }
  else
    return false;
}

bool planning_models::KinematicState::JointState::setVariableValues(const std::vector<double>& joint_state_values)
{
  if (joint_state_values.size() != getVariableCount())
    return false;
  joint_state_values_ = joint_state_values;
  joint_model_->updateTransform(joint_state_values, variable_transform_);
  updateMimicJoints();
  return true;
}

void planning_models::KinematicState::JointState::setVariableValues(const double *joint_state_values)
{
  std::copy(joint_state_values, joint_state_values + joint_state_values_.size(), joint_state_values_.begin());
  joint_model_->updateTransform(joint_state_values_, variable_transform_);
  updateMimicJoints();
}

void planning_models::KinematicState::JointState::setVariableValues(const std::map<std::string, double>& joint_value_map, std::vector<std::string>& missing)
{
  bool has_any = false;
  const std::map<std::string, unsigned int> &vim = getVariableIndexMap();
  for (std::map<std::string, unsigned int>::const_iterator it = vim.begin(); it != vim.end(); ++it)
  {
    std::map<std::string, double>::const_iterator it2 = joint_value_map.find(it->first);
    if (it2 == joint_value_map.end())
      missing.push_back(it->first);
    else
    {
      has_any = true;
      joint_state_values_[it->second] = it2->second;
    }
  }
  
  if (has_any)
  {
    joint_model_->updateTransform(joint_state_values_, variable_transform_);
    updateMimicJoints();
  }
}

void planning_models::KinematicState::JointState::setVariableValues(const std::map<std::string, double>& joint_value_map)
{
  bool update = false;
  const std::map<std::string, unsigned int> &vim = getVariableIndexMap();
  // iterate over the shorter list, for efficiency reasons
  if (joint_value_map.size() <= vim.size())
    for (std::map<std::string, double>::const_iterator it = joint_value_map.begin() ; it != joint_value_map.end() ; ++it)
    {
      std::map<std::string, unsigned int>::const_iterator it2 = vim.find(it->first);
      if (it2 != vim.end())
      {
        joint_state_values_[it2->second] = it->second;
        update = true;
      }
    }
  else
    for (std::map<std::string, unsigned int>::const_iterator it = vim.begin(); it != vim.end(); ++it)
    {
      std::map<std::string, double>::const_iterator it2 = joint_value_map.find(it->first);
      if (it2 != joint_value_map.end())
      {
        update = true;
        joint_state_values_[it->second] = it2->second;
      }
    }
  if (update)
  {
    joint_model_->updateTransform(joint_state_values_, variable_transform_);
    updateMimicJoints();
  }
}

void planning_models::KinematicState::JointState::setVariableValues(const Eigen::Affine3d& transform)
{
  joint_model_->computeJointStateValues(transform, joint_state_values_);
  joint_model_->updateTransform(joint_state_values_, variable_transform_);
  updateMimicJoints();
}

void planning_models::KinematicState::JointState::updateMimicJoints(void)
{
  for (std::size_t i = 0 ; i < mimic_requests_.size() ; ++i)
  {
    std::vector<double> mim_val(joint_state_values_.size());
    for (std::size_t j = 0 ; j < mim_val.size() ; ++j)
      mim_val[j] = joint_state_values_[j] * joint_model_->getMimicFactor() + joint_model_->getMimicOffset();
    mimic_requests_[i]->setVariableValues(&mim_val[0]);
  }
}

bool planning_models::KinematicState::JointState::allVariablesAreDefined(const std::map<std::string, double>& joint_value_map) const
{
  const std::map<std::string, unsigned int> &vim = getVariableIndexMap();
  for (std::map<std::string, unsigned int>::const_iterator it = vim.begin() ; it != vim.end() ; ++it)
    if (joint_value_map.find(it->first) == joint_value_map.end())
      return false;
  return true;
}

bool planning_models::KinematicState::JointState::satisfiesBounds(void) const
{
  const std::vector<std::string> &vn = getVariableNames();
  for (std::size_t i = 0 ; i < vn.size() ; ++i)
    if (!joint_model_->isVariableWithinBounds(vn[i], joint_state_values_[i]))
      return false;
  return true;
}

//-------------------- LinkState ---------------------

planning_models::KinematicState::LinkState::LinkState(KinematicState *state, const planning_models::KinematicModel::LinkModel* lm) :
  kinematic_state_(state), link_model_(lm), parent_joint_state_(NULL), parent_link_state_(NULL)
{
  global_link_transform_.setIdentity();
  global_collision_body_transform_.setIdentity();
}

planning_models::KinematicState::LinkState::~LinkState(void)
{
  clearAttachedBodies();
}

void planning_models::KinematicState::LinkState::updateGivenGlobalLinkTransform(const Eigen::Affine3d& transform)
{
  global_link_transform_ = transform;
  global_collision_body_transform_ = global_link_transform_* link_model_->getCollisionOriginTransform();
  updateAttachedBodies();
}

void planning_models::KinematicState::LinkState::computeTransform(void)
{
  global_link_transform_ = (parent_link_state_ ? parent_link_state_->global_link_transform_ : kinematic_state_->getRootTransform()) * link_model_->getJointOriginTransform();
  global_link_transform_ = global_link_transform_ * parent_joint_state_->getVariableTransform();
  global_collision_body_transform_ = global_link_transform_*link_model_->getCollisionOriginTransform();
  
  updateAttachedBodies();
}

void planning_models::KinematicState::LinkState::updateAttachedBodies(void)
{  
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
    it->second->computeTransform();
}

void planning_models::KinematicState::LinkState::attachBody(const std::string &id,
                                                            const std::vector<shapes::Shape*> &shapes,
                                                            const std::vector<Eigen::Affine3d> &attach_trans,
                                                            const std::vector<std::string> &touch_links)
{
  AttachedBody *ab = new AttachedBody(this, id, shapes, attach_trans, touch_links);
  attached_body_map_[id] = ab;
  kinematic_state_->attached_body_map_[id] = ab;
  ab->computeTransform();
}

void planning_models::KinematicState::LinkState::attachBody(const boost::shared_ptr<AttachedBodyProperties> &properties)
{ 
  AttachedBody *ab = new AttachedBody(this, properties);
  attached_body_map_[ab->getName()] = ab;
  kinematic_state_->attached_body_map_[ab->getName()] = ab;
  ab->computeTransform();
}

bool planning_models::KinematicState::LinkState::hasAttachedBody(const std::string &id) const
{
  return attached_body_map_.find(id) != attached_body_map_.end();
}

const planning_models::KinematicState::AttachedBody* planning_models::KinematicState::LinkState::getAttachedBody(const std::string &id) const
{
  std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.find(id);
  if (it == attached_body_map_.end())
  {
    ROS_ERROR("Attached body '%s' not found on link '%s'", id.c_str(), getName().c_str());
    return NULL;
  }
  else
    return it->second;
}

void planning_models::KinematicState::LinkState::getAttachedBodies(std::vector<const AttachedBody*> &attached_bodies) const
{
  attached_bodies.clear();
  attached_bodies.reserve(attached_body_map_.size());
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
    attached_bodies.push_back(it->second);
}

bool planning_models::KinematicState::LinkState::clearAttachedBody(const std::string &id)
{  
  std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.find(id);
  if (it != attached_body_map_.end())
  {
    delete it->second;
    attached_body_map_.erase(id);
    kinematic_state_->attached_body_map_.erase(id);
    return true;
  }
  return false;
}

void planning_models::KinematicState::LinkState::clearAttachedBodies(void)
{ 
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
  {
    kinematic_state_->attached_body_map_.erase(it->first);
    delete it->second;
  }
  attached_body_map_.clear();
}


//-------------------- AttachedBody ---------------------

planning_models::KinematicState::AttachedBodyProperties::AttachedBodyProperties(void)
{
}

planning_models::KinematicState::AttachedBodyProperties::AttachedBodyProperties(const AttachedBodyProperties &other)
{
  for (std::size_t i = 0 ; i < other.shapes_.size() ; ++i)
    shapes_.push_back(other.shapes_[i]->clone());
  
  attach_trans_ = other.attach_trans_;
  touch_links_ = other.touch_links_;
  id_ = other.id_;
}

planning_models::KinematicState::AttachedBodyProperties::~AttachedBodyProperties(void)
{
  for (std::size_t i = 0 ; i < shapes_.size() ; ++i)
    delete shapes_[i];
}

planning_models::KinematicState::AttachedBody::AttachedBody(const planning_models::KinematicState::LinkState* parent_link_state,
                                                            const std::string &id, const std::vector<shapes::Shape*> &shapes,
                                                            const std::vector<Eigen::Affine3d> &attach_trans,
                                                            const std::vector<std::string> &touch_links) :
  parent_link_state_(parent_link_state)
{
  properties_.reset(new AttachedBodyProperties());
  properties_->id_ = id;
  properties_->attach_trans_ = attach_trans;
  properties_->touch_links_.insert(touch_links.begin(), touch_links.end());
  properties_->shapes_ = shapes;
  
  global_collision_body_transforms_.resize(attach_trans.size());
  for(std::size_t i = 0 ; i < global_collision_body_transforms_.size() ; ++i)
    global_collision_body_transforms_[i].setIdentity();
}

planning_models::KinematicState::AttachedBody::AttachedBody(const planning_models::KinematicState::LinkState* parent_link_state,
                                                            const boost::shared_ptr<AttachedBodyProperties> &properties) :
  parent_link_state_(parent_link_state), properties_(properties)
{
  global_collision_body_transforms_.resize(properties_->attach_trans_.size());
  for(std::size_t i = 0 ; i < global_collision_body_transforms_.size() ; ++i)
    global_collision_body_transforms_[i].setIdentity();
}

planning_models::KinematicState::AttachedBody::~AttachedBody(void)
{
}

void planning_models::KinematicState::AttachedBody::setScale(double scale)
{
  AttachedBodyProperties *ab = new AttachedBodyProperties(*properties_);
  for (std::size_t i = 0 ; i < ab->shapes_.size() ; ++i)
    ab->shapes_[i]->scale(scale);
  properties_.reset(ab);
}

void planning_models::KinematicState::AttachedBody::setPadding(double padding)
{
  AttachedBodyProperties *ab = new AttachedBodyProperties(*properties_);
  for (std::size_t i = 0 ; i < ab->shapes_.size() ; ++i)
    ab->shapes_[i]->padd(padding);
  properties_.reset(ab);
}

void planning_models::KinematicState::AttachedBody::computeTransform(void)
{
  for(std::size_t i = 0; i < global_collision_body_transforms_.size() ; ++i)
    global_collision_body_transforms_[i] = parent_link_state_->getGlobalLinkTransform() * properties_->attach_trans_[i];
}

//--------------------- JointStateGroup --------------------------

planning_models::KinematicState::JointStateGroup::JointStateGroup(planning_models::KinematicState *state,
                                                                  const planning_models::KinematicModel::JointModelGroup *jmg) :
  kinematic_state_(state), joint_model_group_(jmg)
{
  const std::vector<const KinematicModel::JointModel*>& joint_model_vector = jmg->getJointModels();
  for (std::size_t i = 0; i < joint_model_vector.size() ; ++i)
  {
    if (!kinematic_state_->hasJointState(joint_model_vector[i]->getName()))
    {
      ROS_ERROR_STREAM("No joint state for group joint name " << joint_model_vector[i]->getName());
      continue;
    }
    JointState* js = kinematic_state_->getJointState(joint_model_vector[i]->getName());
    joint_state_vector_.push_back(js);
    joint_state_map_[joint_model_vector[i]->getName()] = js;
  }
  const std::vector<const KinematicModel::LinkModel*>& link_model_vector = jmg->getUpdatedLinkModels();
  for (unsigned int i = 0; i < link_model_vector.size(); i++)
  {
    if (!kinematic_state_->hasLinkState(link_model_vector[i]->getName()))
    {
      ROS_ERROR_STREAM("No link state for link joint name " << link_model_vector[i]->getName());
      continue;
    }
    LinkState* ls = kinematic_state_->getLinkState(link_model_vector[i]->getName());
    updated_links_.push_back(ls);
  }
  
  const std::vector<const KinematicModel::JointModel*>& joint_root_vector = jmg->getJointRoots();
  for (std::size_t i = 0; i < joint_root_vector.size(); ++i)
  {
    JointState* js = kinematic_state_->getJointState(joint_root_vector[i]->getName());
    if (js)
      joint_roots_.push_back(js);
  }
}

planning_models::KinematicState::JointStateGroup::~JointStateGroup(void)
{
}

random_numbers::RandomNumberGenerator& planning_models::KinematicState::JointStateGroup::getRandomNumberGenerator(void)
{
  if (!rng_)
    rng_.reset(new random_numbers::RandomNumberGenerator());
  return *rng_;
}

bool planning_models::KinematicState::JointStateGroup::hasJointState(const std::string &joint) const
{
  return joint_state_map_.find(joint) != joint_state_map_.end();
}

bool planning_models::KinematicState::JointStateGroup::updatesLinkState(const std::string& link) const
{
  for (std::size_t i = 0 ; i < updated_links_.size() ; ++i)
    if (updated_links_[i]->getName() == link)
      return true;
  return false;
}

bool planning_models::KinematicState::JointStateGroup::setStateValues(const std::vector<double> &joint_state_values)
{
  if (joint_state_values.size() != getVariableCount())
  {
    ROS_ERROR("Incorrect variable count specified for array of joint values. Expected %u but got %u values",
              getVariableCount(), (int)joint_state_values.size());
    return false;
  }
  
  unsigned int value_counter = 0;
  for(unsigned int i = 0; i < joint_state_vector_.size(); i++)
  {
    unsigned int dim = joint_state_vector_[i]->getVariableCount();
    if (dim != 0)
    {
      joint_state_vector_[i]->setVariableValues(&joint_state_values[value_counter]);
      value_counter += dim;
    }
  }
  updateLinkTransforms();
  return true;
}

void planning_models::KinematicState::JointStateGroup::setStateValues(const std::map<std::string, double>& joint_state_map)
{
  for(unsigned int i = 0; i < joint_state_vector_.size(); ++i)
    joint_state_vector_[i]->setVariableValues(joint_state_map);
  updateLinkTransforms();
}

void planning_models::KinematicState::JointStateGroup::updateLinkTransforms(void)
{
  for(unsigned int i = 0; i < updated_links_.size(); ++i)
    updated_links_[i]->computeTransform();
}

void planning_models::KinematicState::JointStateGroup::setToDefaultValues(void)
{
  std::map<std::string, double> default_joint_values;
  for (std::size_t i = 0  ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->getJointModel()->getDefaultValues(default_joint_values);
  setStateValues(default_joint_values);
}

bool planning_models::KinematicState::JointStateGroup::setToDefaultState(const std::string &name)
{
  std::map<std::string, double> default_joint_values;
  if (!joint_model_group_->getDefaultValues(name, default_joint_values))
    return false;
  setStateValues(default_joint_values);
  return true;
}

void planning_models::KinematicState::JointStateGroup::setToRandomValues(void)
{
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  std::vector<double> random_joint_states;
  joint_model_group_->getRandomValues(rng, random_joint_states);
  setStateValues(random_joint_states);
}

void planning_models::KinematicState::JointStateGroup::getGroupStateValues(std::vector<double>& joint_state_values) const
{
  joint_state_values.clear();
  for(unsigned int i = 0; i < joint_state_vector_.size(); i++)
  {
    const std::vector<double> &jv = joint_state_vector_[i]->getVariableValues();
    joint_state_values.insert(joint_state_values.end(), jv.begin(), jv.end());
  }
}

void planning_models::KinematicState::JointStateGroup::getGroupStateValues(std::map<std::string,double>& joint_state_values) const
{
  joint_state_values.clear();
  for (std::size_t i = 0; i < joint_state_vector_.size(); ++i)
  {
    const std::vector<double> &jsv = joint_state_vector_[i]->getVariableValues();
    const std::vector<std::string> &jsn = joint_state_vector_[i]->getVariableNames();
    for (std::size_t j = 0 ; j < jsv.size(); ++j)
      joint_state_values[jsn[j]] = jsv[j];
  }
}

planning_models::KinematicState::JointState* planning_models::KinematicState::JointStateGroup::getJointState(const std::string &name) const
{
  std::map<std::string, JointState*>::const_iterator it = joint_state_map_.find(name);
  if (it == joint_state_map_.end())
  {
    ROS_ERROR("Joint '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

double planning_models::KinematicState::distance(const KinematicState &state) const
{
  double d = 0.0;
  std::vector<double> j1, j2;
  getStateValues(j1);
  state.getStateValues(j2);
  for (std::size_t i = 0 ; i < j1.size() ; ++i)
  {
    double di = j1[i] - j2[i];
    d += di * di;
  }
  return sqrt(d);
}

const Eigen::Affine3d* planning_models::KinematicState::getFrameTransform(const std::string &id) const
{
  std::map<std::string, LinkState*>::const_iterator it = link_state_map_.find(id);
  if (it != link_state_map_.end())
    return &(it->second->getGlobalLinkTransform());
  std::map<std::string, AttachedBody*>::const_iterator jt = attached_body_map_.find(id);
  if (jt == attached_body_map_.end())
  {
    ROS_ERROR("Transform from frame '%s' to frame '%s' is not known ('%s' should be a link name or an attached body id).",
	      kinematic_model_->getModelFrame().c_str(), id.c_str(), id.c_str());
    return NULL;
  }
  const std::vector<Eigen::Affine3d> &tf = jt->second->getGlobalCollisionBodyTransforms();
  if (tf.empty())
  {
    ROS_ERROR("Attached body '%s' has no geometry associated to it. No transform to return.", id.c_str());
    return NULL;
  }
  if (tf.size() > 1)
    ROS_WARN("There are multiple geometries associated to attached body '%s'. Returning the transform for the first one.", id.c_str());
  return &(tf[0]);
}

bool planning_models::KinematicState::knowsFrameTransform(const std::string &id) const
{ 
  if (hasLinkState(id))
    return true;
  std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.find(id);
  return it != attached_body_map_.end() && it->second->getGlobalCollisionBodyTransforms().size() == 1;
}

// ------ marker functions ------

void planning_models::KinematicState::getRobotMarkers(const std_msgs::ColorRGBA& color,
                                                      const std::string& ns,
                                                      const ros::Duration& dur,
                                                      visualization_msgs::MarkerArray& arr) const
{
  getRobotMarkers(color, ns, dur, arr, kinematic_model_->getLinkModelNames());
}

void planning_models::KinematicState::getRobotMarkers(visualization_msgs::MarkerArray& arr) const
{
  getRobotMarkers(arr, kinematic_model_->getLinkModelNames());
}

void planning_models::KinematicState::getRobotMarkers(const std_msgs::ColorRGBA& color,
                                                      const std::string& ns,
                                                      const ros::Duration& dur,
                                                      visualization_msgs::MarkerArray& arr,
                                                      const std::vector<std::string> &link_names) const
{
  std::size_t cur_num = arr.markers.size();
  getRobotMarkers(arr, link_names);
  unsigned int id = 0;
  for(std::size_t i = cur_num ; i < arr.markers.size() ; ++i, ++id)
  {
    arr.markers[i].ns = ns;
    arr.markers[i].id = id;
    arr.markers[i].lifetime = dur;
    arr.markers[i].color = color;
  }
}

void planning_models::KinematicState::getRobotMarkers(visualization_msgs::MarkerArray& arr,
                                                      const std::vector<std::string> &link_names) const
{
  for(std::size_t i = 0; i < link_names.size(); ++i)
  {
    ROS_DEBUG_STREAM("Trying to get marker for link " << link_names[i]);
    visualization_msgs::Marker mark;
    const LinkState* ls = getLinkState(link_names[i]);
    if(!ls) continue;
    std::vector<const AttachedBody*> attached_bodies;
    ls->getAttachedBodies(attached_bodies);
    for(unsigned int j = 0; j < attached_bodies.size(); j++) {
      if(attached_bodies[j]->getShapes().size() > 0) {
        visualization_msgs::Marker att_mark;
        att_mark.header.frame_id = kinematic_model_->getModelFrame();
        att_mark.header.stamp = ros::Time::now();
        shapes::constructMarkerFromShape(attached_bodies[j]->getShapes()[0],
                                         att_mark);
        msgFromPose(attached_bodies[j]->getGlobalCollisionBodyTransforms()[0],
                    att_mark.pose);
        arr.markers.push_back(att_mark);
      }
    }
    if(!ls->getLinkModel() || !ls->getLinkModel()->getShape()) continue;
    mark.header.frame_id = kinematic_model_->getModelFrame();
    mark.header.stamp = ros::Time::now();
    msgFromPose(ls->getGlobalCollisionBodyTransform(), mark.pose);
    if(ls->getLinkModel()->getFilename().empty()) {
      shapes::constructMarkerFromShape(ls->getLinkModel()->getShape().get(),
                                       mark);
    } else {
      mark.type = mark.MESH_RESOURCE;
      if(!ls->getLinkModel()->getVisualFilename().empty()) {
        mark.mesh_use_embedded_materials = true;
        mark.mesh_resource = ls->getLinkModel()->getVisualFilename();
      } else {
        mark.mesh_resource = ls->getLinkModel()->getFilename();
      }
      ROS_DEBUG_STREAM("Using filename " << mark.mesh_resource);
      //TODO - deal with scale, potentially get visual markers
      mark.scale.x = mark.scale.y = mark.scale.z = 1.0;
    }
    arr.markers.push_back(mark);
  }
}

// ------ printing transforms -----

void planning_models::KinematicState::printStateInfo(std::ostream &out) const
{
  std::map<std::string,double> val;
  getStateValues(val);
  for (std::map<std::string, double>::iterator it = val.begin() ; it != val.end() ; ++it)
    std::cout << it->first << " = " << it->second << std::endl;
}

void planning_models::KinematicState::printTransform(const std::string &st, const Eigen::Affine3d &t, std::ostream &out) const
{
  out << st << std::endl;
  const Eigen::Vector3d &v = t.translation();
  out << "  origin: " << v.x() << ", " << v.y() << ", " << v.z() << std::endl;
  Eigen::Quaterniond q(t.rotation());
  out << "  quaternion: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
}

void planning_models::KinematicState::printTransforms(std::ostream &out) const
{
  out << "Joint transforms:" << std::endl;
  for (unsigned int i = 0 ; i < joint_state_vector_.size() ; ++i)
  {
    printTransform(joint_state_vector_[i]->getName(), joint_state_vector_[i]->getVariableTransform(), out);
    out << std::endl;
  }
  out << "Link poses:" << std::endl;
  for (unsigned int i = 0 ; i < link_state_vector_.size() ; ++i)
  {
    printTransform(link_state_vector_[i]->getName(), link_state_vector_[i]->getGlobalCollisionBodyTransform(), out);
    out << std::endl;
  }
}
