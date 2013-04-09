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

#include <moveit/robot_state/robot_state.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>

robot_state::RobotState::RobotState(const robot_model::RobotModelConstPtr &kinematic_model) :
  kinematic_model_(kinematic_model)
{
  root_transform_.setIdentity();
  buildState();
}

random_numbers::RandomNumberGenerator& robot_state::RobotState::getRandomNumberGenerator()
{
  if (!rng_)
    rng_.reset(new random_numbers::RandomNumberGenerator());
  return *rng_;
}

void robot_state::RobotState::buildState()
{
  const std::vector<const robot_model::JointModel*>& joint_model_vector = kinematic_model_->getJointModels();
  joint_state_vector_.resize(joint_model_vector.size());
  
  // create joint states
  for (std::size_t i = 0; i < joint_model_vector.size() ; ++i)
  {
    joint_state_vector_[i] = new JointState(joint_model_vector[i]);
    joint_state_map_[joint_state_vector_[i]->getName()] = joint_state_vector_[i];
  }
  
  // create link states
  const std::vector<const robot_model::LinkModel*>& link_model_vector = kinematic_model_->getLinkModels();
  link_state_vector_.resize(link_model_vector.size());
  for (std::size_t i = 0 ; i < link_model_vector.size() ; ++i)
  {
    link_state_vector_[i] = new LinkState(this, link_model_vector[i]);
    link_state_map_[link_state_vector_[i]->getName()] = link_state_vector_[i];
  }
  
  // now we need to figure out who the link parents are
  for (std::size_t i = 0; i < link_state_vector_.size(); ++i)
  {
    const robot_model::JointModel* parent_joint_model = link_state_vector_[i]->getLinkModel()->getParentJointModel();
    link_state_vector_[i]->parent_joint_state_ = joint_state_map_[parent_joint_model->getName()];
    if (parent_joint_model->getParentLinkModel() != NULL)
      link_state_vector_[i]->parent_link_state_ = link_state_map_[parent_joint_model->getParentLinkModel()->getName()];
  }
  
  // compute mimic joint state pointers
  for (std::size_t i = 0; i < joint_state_vector_.size(); ++i)
  {
    const std::vector<const robot_model::JointModel*> &mr = joint_state_vector_[i]->joint_model_->getMimicRequests();
    for (std::size_t j = 0 ; j < mr.size() ; ++j)
      joint_state_vector_[i]->mimic_requests_.push_back(joint_state_map_[mr[j]->getName()]);
  }
  
  // now make joint_state_groups
  const std::map<std::string, robot_model::JointModelGroup*>& joint_model_group_map = kinematic_model_->getJointModelGroupMap();
  for (std::map<std::string, robot_model::JointModelGroup*>::const_iterator it = joint_model_group_map.begin() ;
       it != joint_model_group_map.end() ; ++it)
    joint_state_group_map_[it->first] = new JointStateGroup(this, it->second);
}

robot_state::RobotState::RobotState(const RobotState &ks)
{
  copyFrom(ks);
}

robot_state::RobotState& robot_state::RobotState::operator=(const RobotState &other)
{
  copyFrom(other);
  return *this;
}

void robot_state::RobotState::copyFrom(const RobotState &ks)
{
  //need to delete anything already in the state
  for (std::size_t i = 0; i < joint_state_vector_.size(); i++)
    delete joint_state_vector_[i];
  for (std::size_t i = 0; i < link_state_vector_.size(); i++)
    delete link_state_vector_[i];
  for (std::map<std::string, JointStateGroup*>::iterator it = joint_state_group_map_.begin();
       it != joint_state_group_map_.end(); ++it)
    delete it->second;

  kinematic_model_ = ks.getRobotModel();
  root_transform_ = ks.root_transform_;
  
  // construct state
  buildState();

  // copy attached bodies
  clearAttachedBodies(); 
  for (std::map<std::string, AttachedBody*>::const_iterator it = ks.attached_body_map_.begin() ; it != ks.attached_body_map_.end() ; ++it)
    attachBody(it->second->id_, it->second->shapes_, it->second->attach_trans_, it->second->touch_links_, it->second->getAttachedLinkName());

  std::map<std::string, double> current_joint_values;
  ks.getStateValues(current_joint_values);
  setStateValues(current_joint_values);
}

robot_state::RobotState::~RobotState()
{
  clearAttachedBodies(); // we call this instead of just deleting so we get the attached body callbacks 
  for (std::size_t i = 0; i < joint_state_vector_.size(); i++)
    delete joint_state_vector_[i];
  for (std::size_t i = 0; i < link_state_vector_.size(); i++)
    delete link_state_vector_[i];
  for (std::map<std::string, JointStateGroup*>::iterator it = joint_state_group_map_.begin();
       it != joint_state_group_map_.end(); ++it)
    delete it->second;
}

bool robot_state::RobotState::setStateValues(const std::vector<double>& joint_state_values)
{
  if (joint_state_values.size() != getVariableCount())
  {
    logError("RobotState: Incorrect variable count specified for array of joint values. Expected %u but got %u values",
             getVariableCount(), (int)joint_state_values.size());
    return false;
  }
  
  unsigned int value_counter = 0;
  for(std::size_t i = 0; i < joint_state_vector_.size(); i++)
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

void robot_state::RobotState::setStateValues(const std::map<std::string, double>& joint_state_map)
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    if (joint_state_vector_[i]->getJointModel()->getMimic() == NULL)
      joint_state_vector_[i]->setVariableValues(joint_state_map);
  updateLinkTransforms();
}

void robot_state::RobotState::setStateValues(const std::map<std::string, double>& joint_state_map, std::vector<std::string>& missing)
{
  missing.clear();
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    if (joint_state_vector_[i]->getJointModel()->getMimic() == NULL)
      joint_state_vector_[i]->setVariableValues(joint_state_map, missing);
  updateLinkTransforms();
}

void robot_state::RobotState::setStateValues(const sensor_msgs::JointState& js)
{
  std::map<std::string, double> vals;
  std::size_t position_size = js.position.size();
  for(std::size_t i = 0 ; i < js.name.size() ; ++i)
    if (i < position_size)
      vals[js.name[i]] = js.position[i];
  setStateValues(vals);
}

void robot_state::RobotState::setStateValues(const std::vector<std::string>& joint_names,
                                                     const std::vector<double>& joint_values)
{
  std::map<std::string, double> vals;
  unsigned int position_size = joint_values.size();
  for(unsigned int i = 0 ; i < joint_names.size() ; ++i)
    if (i < position_size)
      vals[joint_names[i]] = joint_values[i];
  setStateValues(vals);
}

void robot_state::RobotState::getStateValues(std::vector<double>& joint_state_values) const
{
  joint_state_values.clear();
  for (std::size_t i = 0; i < joint_state_vector_.size(); i++)
    if (joint_state_vector_[i]->getJointModel()->getMimic() == NULL)
    {
      const std::vector<double> &jv = joint_state_vector_[i]->getVariableValues();
      joint_state_values.insert(joint_state_values.end(), jv.begin(), jv.end());
    }
}

void robot_state::RobotState::getStateValues(std::map<std::string,double>& joint_state_values) const
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

void robot_state::RobotState::getStateValues(sensor_msgs::JointState& js) const
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

void robot_state::RobotState::updateLinkTransforms()
{
  for(unsigned int i = 0; i < link_state_vector_.size(); i++)
    link_state_vector_[i]->computeTransform();
}

bool robot_state::RobotState::updateStateWithLinkAt(const std::string& link_name, const Eigen::Affine3d& transform)
{
  if (!hasLinkState(link_name))
    return false;
  
  link_state_map_[link_name]->updateGivenGlobalLinkTransform(transform);
  std::vector<const robot_model::LinkModel*> child_link_models;
  kinematic_model_->getChildLinkModels(kinematic_model_->getLinkModel(link_name), child_link_models);
  // the zeroith link will be the link itself, which shouldn't be updated, so we start at 1
  for(unsigned int i = 1 ; i < child_link_models.size() ; ++i)
    link_state_map_[child_link_models[i]->getName()]->computeTransform();
  
  const robot_model::LinkModel::AssociatedFixedTransformMap& assoc = kinematic_model_->getLinkModel(link_name)->getAssociatedFixedTransforms();
  for (robot_model::LinkModel::AssociatedFixedTransformMap::const_iterator it = assoc.begin() ; it != assoc.end() ; ++it)
    link_state_map_[it->first->getName()]->updateGivenGlobalLinkTransform(transform * it->second);
  
  return true;
}

const Eigen::Affine3d& robot_state::RobotState::getRootTransform() const
{
  return root_transform_;
}

void robot_state::RobotState::setRootTransform(const Eigen::Affine3d &transform)
{
  root_transform_ = transform;
}

void robot_state::RobotState::setToDefaultValues()
{
  std::vector<double> default_joint_states;
  kinematic_model_->getVariableDefaultValues(default_joint_states);
  setStateValues(default_joint_states);
}

void robot_state::RobotState::setToRandomValues()
{
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  std::vector<double> random_joint_states;
  kinematic_model_->getVariableRandomValues(rng, random_joint_states);
  setStateValues(random_joint_states);
}

double robot_state::RobotState::infinityNormDistance(const robot_state::RobotState *other) const
{
  if (joint_state_vector_.empty())
    return 0.0;
  double max_d = joint_state_vector_[0]->distance(other->joint_state_vector_[0]);
  for (std::size_t i = 1 ; i < joint_state_vector_.size() ; ++i)
  {
    double d = joint_state_vector_[i]->distance(other->joint_state_vector_[i]);
    if (d > max_d)
      max_d = d;
  }
  return max_d;
}

bool robot_state::RobotState::satisfiesBounds(const std::string& joint) const
{
  std::vector<std::string> j(1, joint);
  return satisfiesBounds(j);
}

bool robot_state::RobotState::satisfiesBounds(const std::vector<std::string>& joints) const
{
  for (std::vector<std::string>::const_iterator it = joints.begin(); it != joints.end(); ++it)
  {
    const JointState* joint_state = getJointState(*it);
    if (joint_state == NULL)
    {
      logWarn("No joint with name '%s'", it->c_str());
      return false;
    }
    if (!joint_state->satisfiesBounds())
      return false;
  }
  return true;
}

bool robot_state::RobotState::satisfiesBounds() const
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    if (!joint_state_vector_[i]->satisfiesBounds())
      return false;
  return true;
}

void robot_state::RobotState::enforceBounds()
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->enforceBounds();
  updateLinkTransforms();
}

const robot_state::JointStateGroup* robot_state::RobotState::getJointStateGroup(const std::string &name) const
{
  if (joint_state_group_map_.find(name) == joint_state_group_map_.end())
    return NULL;
  return joint_state_group_map_.find(name)->second;
}

robot_state::JointStateGroup* robot_state::RobotState::getJointStateGroup(const std::string &name)
{
  if (joint_state_group_map_.find(name) == joint_state_group_map_.end())
    return NULL;
  return joint_state_group_map_.find(name)->second;
}

bool robot_state::RobotState::hasJointStateGroup(const std::string &name) const
{
  return joint_state_group_map_.find(name) != joint_state_group_map_.end();
}

void robot_state::RobotState::getJointStateGroupNames(std::vector<std::string>& names) const
{
  for (std::map<std::string, JointStateGroup*>::const_iterator it = joint_state_group_map_.begin();
       it != joint_state_group_map_.end() ; ++it)
    names.push_back(it->first);
}

bool robot_state::RobotState::hasJointState(const std::string &joint) const
{
  return joint_state_map_.find(joint) != joint_state_map_.end();
}

bool robot_state::RobotState::hasLinkState(const std::string& link) const
{
  return link_state_map_.find(link) != link_state_map_.end();
}

robot_state::JointState* robot_state::RobotState::getJointState(const std::string &name) const
{
  std::map<std::string, JointState*>::const_iterator it = joint_state_map_.find(name);
  if (it == joint_state_map_.end())
  {
    logError("Joint state '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

robot_state::LinkState* robot_state::RobotState::getLinkState(const std::string &name) const
{
  std::map<std::string, LinkState*>::const_iterator it = link_state_map_.find(name);
  if (it == link_state_map_.end())
  {
    logError("Link state '%s' not found", name.c_str());
    return NULL; 
  }
  else
    return it->second;
}

bool robot_state::RobotState::hasAttachedBody(const std::string &id) const
{
  return attached_body_map_.find(id) != attached_body_map_.end();
}

const robot_state::AttachedBody* robot_state::RobotState::getAttachedBody(const std::string &id) const
{
  std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.find(id);
  if (it == attached_body_map_.end())
  {
    logError("Attached body '%s' not found", id.c_str());
    return NULL;
  }
  else
    return it->second;
}

void robot_state::RobotState::attachBody(AttachedBody *attached_body)
{
  LinkState *ls = getLinkState(attached_body->getAttachedLinkName());
  if (ls)
  {
    attached_body_map_[attached_body->getName()] = attached_body;
    ls->attached_body_map_[attached_body->getName()] = attached_body;
    attached_body->computeTransform(ls->getGlobalLinkTransform());
    if (attached_body_update_callback_)
      attached_body_update_callback_(attached_body, true);
  }
}

void robot_state::RobotState::attachBody(const std::string &id,
                                         const std::vector<shapes::ShapeConstPtr> &shapes,
                                         const EigenSTL::vector_Affine3d &attach_trans,
                                         const std::set<std::string> &touch_links,
                                         const std::string &link)
{
  LinkState *ls = getLinkState(link);
  if (ls)
  {
    AttachedBody *ab = new AttachedBody(ls->getLinkModel(), id, shapes, attach_trans, touch_links);
    ls->attached_body_map_[id] = ab;
    attached_body_map_[id] = ab;
    ab->computeTransform(ls->getGlobalLinkTransform());
    if (attached_body_update_callback_)
      attached_body_update_callback_(ab, true);
  }
}

void robot_state::RobotState::attachBody(const std::string &id,
                                         const std::vector<shapes::ShapeConstPtr> &shapes,
                                         const EigenSTL::vector_Affine3d &attach_trans,
                                         const std::vector<std::string> &touch_links,
                                         const std::string &link)
{
  std::set<std::string> touch_links_set(touch_links.begin(), touch_links.end());
  attachBody(id, shapes, attach_trans, touch_links_set, link);
}

void robot_state::RobotState::getAttachedBodies(std::vector<const AttachedBody*> &attached_bodies) const
{
  attached_bodies.clear();
  attached_bodies.reserve(attached_body_map_.size());
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
    attached_bodies.push_back(it->second);
}

void robot_state::RobotState::clearAttachedBodies()
{
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
  {
    if (attached_body_update_callback_)
      attached_body_update_callback_(it->second, false);
    LinkState *ls = getLinkState(it->second->getAttachedLinkName());
    if (ls)
      ls->attached_body_map_.clear();
    delete it->second;
  }
  attached_body_map_.clear();
}

void robot_state::RobotState::clearAttachedBodies(const std::string &link_name)
{
  LinkState *ls = getLinkState(link_name);
  if (ls)
  {
    for (std::map<std::string, AttachedBody*>::const_iterator it = ls->attached_body_map_.begin() ; it != ls->attached_body_map_.end() ;  ++it)
    {
      if (attached_body_update_callback_)
        attached_body_update_callback_(it->second, false);
      attached_body_map_.erase(it->first);
      delete it->second;
    }
    ls->attached_body_map_.clear();
  }
}

bool robot_state::RobotState::clearAttachedBody(const std::string &id)
{
  std::map<std::string, AttachedBody*>::iterator it = attached_body_map_.find(id);
  if (it != attached_body_map_.end())
  {
    if (attached_body_update_callback_)
      attached_body_update_callback_(it->second, false);
    LinkState *ls = getLinkState(it->second->getAttachedLinkName());
    if (ls)
    {
      std::map<std::string, AttachedBody*>::iterator jt = ls->attached_body_map_.find(id);
      ls->attached_body_map_.erase(jt);
    }
    delete it->second;
    attached_body_map_.erase(it);
    return true;
  }
  else
    return false;
}

namespace 
{
static inline void updateAABB(const Eigen::Affine3d &t, const Eigen::Vector3d &e, std::vector<double> &aabb)
{
  Eigen::Vector3d v = e / 2.0;
  Eigen::Vector3d c2 = t * v;
  v = -v;
  Eigen::Vector3d c1 = t * v;
  if (aabb.empty())
  {
    aabb.resize(6);
    aabb[0] = c1.x();
    aabb[2] = c1.y();
    aabb[4] = c1.z();
    aabb[1] = c2.x();
    aabb[3] = c2.y();
    aabb[5] = c2.z();
  }
  else
  {
    if (aabb[0] > c1.x())
      aabb[0] = c1.x();
    if (aabb[2] > c1.y())
      aabb[2] = c1.y();
    if (aabb[4] > c1.z())
      aabb[4] = c1.z();
    if (aabb[1] < c2.x())
      aabb[1] = c2.x();
    if (aabb[3] < c2.y())
      aabb[3] = c2.y();
    if (aabb[5] < c2.z())
      aabb[5] = c2.z();
  }
}
}

void robot_state::RobotState::computeAABB(std::vector<double> &aabb) const
{
  aabb.clear();
  for (std::size_t i = 0; i < link_state_vector_.size(); ++i)
  {
    const Eigen::Affine3d &t = link_state_vector_[i]->getGlobalCollisionBodyTransform();
    const Eigen::Vector3d &e = link_state_vector_[i]->getLinkModel()->getShapeExtentsAtOrigin();
    updateAABB(t, e, aabb);
  }
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ; ++it)
  {
    const EigenSTL::vector_Affine3d &ts = it->second->getGlobalCollisionBodyTransforms();
    const std::vector<shapes::ShapeConstPtr> &ss = it->second->getShapes();
    for (std::size_t i = 0 ; i < ts.size() ; ++i)
    {
      Eigen::Vector3d e = shapes::computeShapeExtents(ss[i].get());
      updateAABB(ts[i], e, aabb);
    }
  }
  if (aabb.empty())
    aabb.resize(6, 0.0);
}

double robot_state::RobotState::distance(const RobotState &state) const
{
  double d = 0.0;
  const std::vector<JointState*> &other = state.getJointStateVector();
  for (std::size_t i = 0; i < joint_state_vector_.size(); ++i)
    d += joint_state_vector_[i]->distance(other[i]) * joint_state_vector_[i]->getJointModel()->getDistanceFactor();
  return d;
}

void robot_state::RobotState::interpolate(const RobotState &to, const double t, RobotState &dest) const
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->interpolate(to.joint_state_vector_[i], t, dest.joint_state_vector_[i]);
  dest.updateLinkTransforms();
}

const Eigen::Affine3d& robot_state::RobotState::getFrameTransform(const std::string &id) const
{
  if (!id.empty() && id[0] == '/')
    return getFrameTransform(id.substr(1));
  static const Eigen::Affine3d identity_transform = Eigen::Affine3d::Identity();
  std::map<std::string, LinkState*>::const_iterator it = link_state_map_.find(id);
  if (it != link_state_map_.end())
    return it->second->getGlobalLinkTransform();
  std::map<std::string, AttachedBody*>::const_iterator jt = attached_body_map_.find(id);
  if (jt == attached_body_map_.end())
  {
    logError("Transform from frame '%s' to frame '%s' is not known ('%s' should be a link name or an attached body id).",
             id.c_str(), kinematic_model_->getModelFrame().c_str(), id.c_str());
    return identity_transform;
  }
  const EigenSTL::vector_Affine3d &tf = jt->second->getGlobalCollisionBodyTransforms();
  if (tf.empty())
  {
    logError("Attached body '%s' has no geometry associated to it. No transform to return.", id.c_str());
    return identity_transform;
  }
  if (tf.size() > 1)
    logWarn("There are multiple geometries associated to attached body '%s'. Returning the transform for the first one.", id.c_str());
  return tf[0];
}

bool robot_state::RobotState::knowsFrameTransform(const std::string &id) const
{   
  if (!id.empty() && id[0] == '/')
    return knowsFrameTransform(id.substr(1));
  if (hasLinkState(id))
    return true;
  std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.find(id);
  return it != attached_body_map_.end() && it->second->getGlobalCollisionBodyTransforms().size() == 1;
}

// ------ marker functions ------

void robot_state::RobotState::getRobotMarkers(visualization_msgs::MarkerArray& arr,
                                              const std::vector<std::string> &link_names,
                                              const std_msgs::ColorRGBA& color,
                                              const std::string& ns,
                                              const ros::Duration& dur, 
                                              bool include_attached) const
{
  std::size_t cur_num = arr.markers.size();
  getRobotMarkers(arr, link_names, include_attached);
  unsigned int id = cur_num;
  for (std::size_t i = cur_num ; i < arr.markers.size() ; ++i, ++id)
  {
    arr.markers[i].ns = ns;
    arr.markers[i].id = id;
    arr.markers[i].lifetime = dur;
    arr.markers[i].color = color;
  }
}

void robot_state::RobotState::getRobotMarkers(visualization_msgs::MarkerArray& arr, const std::vector<std::string> &link_names, bool include_attached) const
{
  ros::Time tm = ros::Time::now();
  for (std::size_t i = 0; i < link_names.size(); ++i)
  {
    logDebug("Trying to get marker for link '%s'", link_names[i].c_str());
    visualization_msgs::Marker mark;
    const LinkState* ls = getLinkState(link_names[i]);
    if (!ls)
      continue;
    if (include_attached)
    {
      std::vector<const AttachedBody*> attached_bodies;
      ls->getAttachedBodies(attached_bodies);
      for (std::size_t j = 0; j < attached_bodies.size(); ++j)
        if (attached_bodies[j]->getShapes().size() > 0)
        {
          visualization_msgs::Marker att_mark;
          att_mark.header.frame_id = kinematic_model_->getModelFrame();
          att_mark.header.stamp = tm;
          shapes::constructMarkerFromShape(attached_bodies[j]->getShapes()[0].get(), att_mark);
          tf::poseEigenToMsg(attached_bodies[j]->getGlobalCollisionBodyTransforms()[0], att_mark.pose);
          arr.markers.push_back(att_mark);
        }
    }
    if (!ls->getLinkModel() || !ls->getLinkModel()->getShape())
      continue;
    mark.header.frame_id = kinematic_model_->getModelFrame();
    mark.header.stamp = tm;
    tf::poseEigenToMsg(ls->getGlobalCollisionBodyTransform(), mark.pose);

    // we prefer using the visual mesh, if a mesh is available
    const std::string& mesh_resource = ls->getLinkModel()->getVisualMeshFilename();
    if (mesh_resource.empty())
    {
      if (!shapes::constructMarkerFromShape(ls->getLinkModel()->getShape().get(), mark))
        continue;
      // if the object is invisible (0 volume) we skip it
      if (fabs(mark.scale.x * mark.scale.y * mark.scale.z) < std::numeric_limits<float>::epsilon())
        continue;
    }
    else
    {
      mark.type = mark.MESH_RESOURCE;
      mark.mesh_use_embedded_materials = false;
      mark.mesh_resource = mesh_resource; 
      const Eigen::Vector3d &mesh_scale = ls->getLinkModel()->getVisualMeshScale(); 
     
      mark.scale.x = mesh_scale[0];
      mark.scale.y = mesh_scale[1];
      mark.scale.z = mesh_scale[2];
    }
    arr.markers.push_back(mark);
  }
}

void robot_state::RobotState::setAttachedBodyUpdateCallback(const AttachedBodyCallback &callback)
{
  attached_body_update_callback_ = callback;
}

// ------ printing transforms -----

void robot_state::RobotState::printStateInfo(std::ostream &out) const
{
  std::map<std::string,double> val;
  getStateValues(val);
  for (std::map<std::string, double>::iterator it = val.begin() ; it != val.end() ; ++it)
    out << it->first << " = " << it->second << std::endl;
}

void robot_state::RobotState::getPoseString(std::stringstream& ss, const Eigen::Affine3d& pose, const std::string& pfx)
{
  ss.precision(3);
  for (int y=0;y<4;y++) 
  {
    ss << pfx;
    for (int x=0;x<4;x++) 
    {
      ss << std::setw(8) << pose(y,x) << " ";
    }
    ss << std::endl;
  }
} 

void robot_state::RobotState::getStateTreeJointString(std::stringstream& ss, const robot_state::JointState* js, const std::string& pfx0, bool last) const
{
  std::string pfx = pfx0 + "+--";

  const robot_model::JointModel* jm = js->getJointModel();
  ss << pfx << "Joint: " << jm->getName() << std::endl;

  pfx = pfx0 + (last ? "   " : "|  ");

  for (int i=0; i<js->getVariableCount(); i++)
  {
    ss.precision(3);
    ss << pfx << js->getVariableNames()[i] << std::setw(8) << js->getVariableValues()[i] << std::endl;
  }

  const robot_model::LinkModel* lm = jm->getChildLinkModel();
  const LinkState* ls = getLinkState(jm->getChildLinkModel()->getName());

  ss << pfx << "Link: " << lm->getName() << std::endl;
  getPoseString(ss, lm->getJointOriginTransform(), pfx + "joint_origin:");
  getPoseString(ss, js->getVariableTransform(), pfx + "joint_variable:");
  getPoseString(ss, ls->getGlobalLinkTransform(), pfx + "link_global:");

  for (std::vector<robot_model::JointModel*>::const_iterator it = lm->getChildJointModels().begin() ; it != lm->getChildJointModels().end() ; ++it)
  {
    const robot_model::JointModel* cjm = *it;
    getStateTreeJointString(ss, getJointState(cjm->getName()), pfx, it+1 == lm->getChildJointModels().end());
  }
}

std::string robot_state::RobotState::getStateTreeString(const std::string& prefix) const
{
  std::stringstream ss;
  ss << "ROBOT: " << getRobotModel()->getName() << std::endl;
  getPoseString(ss, getRootTransform(), "   Root: ");
  getStateTreeJointString(ss, getJointState(getRobotModel()->getRoot()->getName()), "   ", true);
  return ss.str();
}

void robot_state::RobotState::printTransform(const std::string &st, const Eigen::Affine3d &t, std::ostream &out) const
{
  out << st << std::endl;
  const Eigen::Vector3d &v = t.translation();
  out << "  origin: " << v.x() << ", " << v.y() << ", " << v.z() << std::endl;
  Eigen::Quaterniond q(t.rotation());
  out << "  quaternion: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
}

void robot_state::RobotState::printTransforms(std::ostream &out) const
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
