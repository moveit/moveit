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

#include <moveit/kinematic_state/kinematic_state.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>

kinematic_state::KinematicState::KinematicState(const kinematic_model::KinematicModelConstPtr &kinematic_model) :
  kinematic_model_(kinematic_model)
{
  root_transform_.setIdentity();
  buildState();
}

random_numbers::RandomNumberGenerator& kinematic_state::KinematicState::getRandomNumberGenerator(void)
{
  if (!rng_)
    rng_.reset(new random_numbers::RandomNumberGenerator());
  return *rng_;
}

void kinematic_state::KinematicState::buildState(void)
{
  const std::vector<const kinematic_model::JointModel*>& joint_model_vector = kinematic_model_->getJointModels();
  joint_state_vector_.resize(joint_model_vector.size());
  
  // create joint states
  for (std::size_t i = 0; i < joint_model_vector.size() ; ++i)
  {
    joint_state_vector_[i] = new JointState(joint_model_vector[i]);
    joint_state_map_[joint_state_vector_[i]->getName()] = joint_state_vector_[i];
  }
  
  // create link states
  const std::vector<const kinematic_model::LinkModel*>& link_model_vector = kinematic_model_->getLinkModels();
  link_state_vector_.resize(link_model_vector.size());
  for (std::size_t i = 0 ; i < link_model_vector.size() ; ++i)
  {
    link_state_vector_[i] = new LinkState(this, link_model_vector[i]);
    link_state_map_[link_state_vector_[i]->getName()] = link_state_vector_[i];
  }
  
  // now we need to figure out who the link parents are
  for (std::size_t i = 0; i < link_state_vector_.size(); ++i)
  {
    const kinematic_model::JointModel* parent_joint_model = link_state_vector_[i]->getLinkModel()->getParentJointModel();
    link_state_vector_[i]->parent_joint_state_ = joint_state_map_[parent_joint_model->getName()];
    if (parent_joint_model->getParentLinkModel() != NULL)
      link_state_vector_[i]->parent_link_state_ = link_state_map_[parent_joint_model->getParentLinkModel()->getName()];
  }
  
  // compute mimic joint state pointers
  for (std::size_t i = 0; i < joint_state_vector_.size(); ++i)
  {
    const std::vector<const kinematic_model::JointModel*> &mr = joint_state_vector_[i]->joint_model_->getMimicRequests();
    for (std::size_t j = 0 ; j < mr.size() ; ++j)
      joint_state_vector_[i]->mimic_requests_.push_back(joint_state_map_[mr[j]->getName()]);
  }
  
  // now make joint_state_groups
  const std::map<std::string, kinematic_model::JointModelGroup*>& joint_model_group_map = kinematic_model_->getJointModelGroupMap();
  for (std::map<std::string, kinematic_model::JointModelGroup*>::const_iterator it = joint_model_group_map.begin() ;
       it != joint_model_group_map.end() ; ++it)
    joint_state_group_map_[it->first] = new JointStateGroup(this, it->second);
}

kinematic_state::KinematicState::KinematicState(const KinematicState &ks)
{
  copyFrom(ks);
}

kinematic_state::KinematicState& kinematic_state::KinematicState::operator=(const KinematicState &other)
{
  copyFrom(other);
  return *this;
}

void kinematic_state::KinematicState::copyFrom(const KinematicState &ks)
{
  //need to delete anything already in the state
  for (std::size_t i = 0; i < joint_state_vector_.size(); i++)
    delete joint_state_vector_[i];
  for (std::size_t i = 0; i < link_state_vector_.size(); i++)
    delete link_state_vector_[i];
  for (std::map<std::string, JointStateGroup*>::iterator it = joint_state_group_map_.begin();
       it != joint_state_group_map_.end(); ++it)
    delete it->second;

  kinematic_model_ = ks.getKinematicModel();
  root_transform_ = ks.root_transform_;
  
  // construct state
  buildState();
  // copy attached bodies
  for (std::size_t i = 0 ; i < ks.link_state_vector_.size() ; ++i)
  {
    std::vector<const AttachedBody*> ab;
    ks.link_state_vector_[i]->getAttachedBodies(ab);
    LinkState *ls = link_state_map_[ks.link_state_vector_[i]->getName()];
    for (std::size_t j = 0 ; j < ab.size() ; ++j)
    {
      std::vector<std::string> tl;
      tl.insert(tl.end(), ab[j]->touch_links_.begin(), ab[j]->touch_links_.end());
      ls->attachBody(ab[j]->id_, ab[j]->shapes_, ab[j]->attach_trans_, tl);
    }
  }
  std::map<std::string, double> current_joint_values;
  ks.getStateValues(current_joint_values);
  setStateValues(current_joint_values);
}

kinematic_state::KinematicState::~KinematicState(void)
{
  for (std::size_t i = 0; i < joint_state_vector_.size(); i++)
    delete joint_state_vector_[i];
  for (std::size_t i = 0; i < link_state_vector_.size(); i++)
    delete link_state_vector_[i];
  for (std::map<std::string, JointStateGroup*>::iterator it = joint_state_group_map_.begin();
       it != joint_state_group_map_.end(); ++it)
    delete it->second;
}

bool kinematic_state::KinematicState::setStateValues(const std::vector<double>& joint_state_values)
{
  if (joint_state_values.size() != getVariableCount())
  {
    logError("Incorrect variable count specified for array of joint values. Expected %u but got %u values",
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

void kinematic_state::KinematicState::setStateValues(const std::map<std::string, double>& joint_state_map)
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    if (joint_state_vector_[i]->getJointModel()->getMimic() == NULL)
      joint_state_vector_[i]->setVariableValues(joint_state_map);
  updateLinkTransforms();
}

void kinematic_state::KinematicState::setStateValues(const std::map<std::string, double>& joint_state_map, std::vector<std::string>& missing)
{
  missing.clear();
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    if (joint_state_vector_[i]->getJointModel()->getMimic() == NULL)
      joint_state_vector_[i]->setVariableValues(joint_state_map, missing);
  updateLinkTransforms();
}

void kinematic_state::KinematicState::setStateValues(const sensor_msgs::JointState& js)
{
  std::map<std::string, double> vals;
  std::size_t position_size = js.position.size();
  for(std::size_t i = 0 ; i < js.name.size() ; ++i)
    if (i < position_size)
      vals[js.name[i]] = js.position[i];
  setStateValues(vals);
}

void kinematic_state::KinematicState::setStateValues(const std::vector<std::string>& joint_names,
                                                     const std::vector<double>& joint_values)
{
  std::map<std::string, double> vals;
  unsigned int position_size = joint_values.size();
  for(unsigned int i = 0 ; i < joint_names.size() ; ++i)
    if (i < position_size)
      vals[joint_names[i]] = joint_values[i];
  setStateValues(vals);
}

void kinematic_state::KinematicState::getStateValues(std::vector<double>& joint_state_values) const
{
  joint_state_values.clear();
  for (std::size_t i = 0; i < joint_state_vector_.size(); i++)
    if (joint_state_vector_[i]->getJointModel()->getMimic() == NULL)
    {
      const std::vector<double> &jv = joint_state_vector_[i]->getVariableValues();
      joint_state_values.insert(joint_state_values.end(), jv.begin(), jv.end());
    }
}

void kinematic_state::KinematicState::getStateValues(std::map<std::string,double>& joint_state_values) const
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

void kinematic_state::KinematicState::getStateValues(sensor_msgs::JointState& js) const
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

void kinematic_state::KinematicState::updateLinkTransforms(void)
{
  for(unsigned int i = 0; i < link_state_vector_.size(); i++)
    link_state_vector_[i]->computeTransform();
}

bool kinematic_state::KinematicState::updateStateWithLinkAt(const std::string& link_name, const Eigen::Affine3d& transform)
{
  if (!hasLinkState(link_name))
    return false;
  
  link_state_map_[link_name]->updateGivenGlobalLinkTransform(transform);
  std::vector<const kinematic_model::LinkModel*> child_link_models;
  kinematic_model_->getChildLinkModels(kinematic_model_->getLinkModel(link_name), child_link_models);
  // the zeroith link will be the link itself, which shouldn't be updated, so we start at 1
  for(unsigned int i = 1 ; i < child_link_models.size() ; ++i)
    link_state_map_[child_link_models[i]->getName()]->computeTransform();
  
  const kinematic_model::LinkModel::AssociatedFixedTransformMap& assoc = kinematic_model_->getLinkModel(link_name)->getAssociatedFixedTransforms();
  for (kinematic_model::LinkModel::AssociatedFixedTransformMap::const_iterator it = assoc.begin() ; it != assoc.end() ; ++it)
    link_state_map_[it->first->getName()]->updateGivenGlobalLinkTransform(transform * it->second);
  
  return true;
}

const Eigen::Affine3d& kinematic_state::KinematicState::getRootTransform(void) const
{
  return root_transform_;
}

void kinematic_state::KinematicState::setRootTransform(const Eigen::Affine3d &transform)
{
  root_transform_ = transform;
}

void kinematic_state::KinematicState::setToDefaultValues(void)
{
  std::vector<double> default_joint_states;
  kinematic_model_->getVariableDefaultValues(default_joint_states);
  setStateValues(default_joint_states);
}

void kinematic_state::KinematicState::setToRandomValues(void)
{
  random_numbers::RandomNumberGenerator &rng = getRandomNumberGenerator();
  std::vector<double> random_joint_states;
  kinematic_model_->getVariableRandomValues(rng, random_joint_states);
  setStateValues(random_joint_states);
}

double kinematic_state::KinematicState::infinityNormDistance(const kinematic_state::KinematicState *other) const
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

bool kinematic_state::KinematicState::satisfiesBounds(const std::string& joint) const
{
  std::vector<std::string> j(1, joint);
  return satisfiesBounds(j);
}

bool kinematic_state::KinematicState::satisfiesBounds(const std::vector<std::string>& joints) const
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

bool kinematic_state::KinematicState::satisfiesBounds(void) const
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    if (!joint_state_vector_[i]->satisfiesBounds())
      return false;
  return true;
}

void kinematic_state::KinematicState::enforceBounds(void)
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->enforceBounds();
  updateLinkTransforms();
}

const kinematic_state::JointStateGroup* kinematic_state::KinematicState::getJointStateGroup(const std::string &name) const
{
  if (joint_state_group_map_.find(name) == joint_state_group_map_.end())
    return NULL;
  return joint_state_group_map_.find(name)->second;
}

kinematic_state::JointStateGroup* kinematic_state::KinematicState::getJointStateGroup(const std::string &name)
{
  if (joint_state_group_map_.find(name) == joint_state_group_map_.end())
    return NULL;
  return joint_state_group_map_.find(name)->second;
}

bool kinematic_state::KinematicState::hasJointStateGroup(const std::string &name) const
{
  return joint_state_group_map_.find(name) != joint_state_group_map_.end();
}

void kinematic_state::KinematicState::getJointStateGroupNames(std::vector<std::string>& names) const
{
  for (std::map<std::string, JointStateGroup*>::const_iterator it = joint_state_group_map_.begin();
       it != joint_state_group_map_.end() ; ++it)
    names.push_back(it->first);
}

bool kinematic_state::KinematicState::hasJointState(const std::string &joint) const
{
  return joint_state_map_.find(joint) != joint_state_map_.end();
}

bool kinematic_state::KinematicState::hasLinkState(const std::string& link) const
{
  return link_state_map_.find(link) != link_state_map_.end();
}

kinematic_state::JointState* kinematic_state::KinematicState::getJointState(const std::string &name) const
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

kinematic_state::LinkState* kinematic_state::KinematicState::getLinkState(const std::string &name) const
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

bool kinematic_state::KinematicState::hasAttachedBody(const std::string &id) const
{
  return attached_body_map_.find(id) != attached_body_map_.end();
}

const kinematic_state::AttachedBody* kinematic_state::KinematicState::getAttachedBody(const std::string &id) const
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

void kinematic_state::KinematicState::attachBody(AttachedBody *attached_body)
{
  LinkState *ls = getLinkState(attached_body->getAttachedLinkName());
  if (ls)
  {
    attached_body_map_[attached_body->getName()] = attached_body;
    ls->attached_body_map_[attached_body->getName()] = attached_body;
    attached_body->computeTransform();
  }
}

void kinematic_state::KinematicState::getAttachedBodies(std::vector<const AttachedBody*> &attached_bodies) const
{
  attached_bodies.clear();
  attached_bodies.reserve(attached_body_map_.size());
  for (std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.begin() ; it != attached_body_map_.end() ;  ++it)
    attached_bodies.push_back(it->second);
}

void kinematic_state::KinematicState::clearAttachedBodies(void)
{
  attached_body_map_.clear();
  for (std::size_t i = 0 ; i < link_state_vector_.size() ; ++i)
    link_state_vector_[i]->clearAttachedBodies();
}

bool kinematic_state::KinematicState::clearAttachedBody(const std::string &id)
{
  std::map<std::string, AttachedBody*>::iterator it = attached_body_map_.find(id);
  if (it != attached_body_map_.end())
  {
    LinkState *ls = getLinkState(it->second->getAttachedLinkName());
    if (ls)
      return ls->clearAttachedBody(id);
    else
      return false;
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

void kinematic_state::KinematicState::computeAABB(std::vector<double> &aabb) const
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

double kinematic_state::KinematicState::distance(const KinematicState &state) const
{
  double d = 0.0;
  const std::vector<JointState*> &other = state.getJointStateVector();
  for (std::size_t i = 0; i < joint_state_vector_.size(); ++i)
    d += joint_state_vector_[i]->distance(other[i]) * joint_state_vector_[i]->getJointModel()->getDistanceFactor();
  return d;
}

void kinematic_state::KinematicState::interpolate(const KinematicState &to, const double t, KinematicState &dest) const
{
  for (std::size_t i = 0 ; i < joint_state_vector_.size() ; ++i)
    joint_state_vector_[i]->interpolate(to.joint_state_vector_[i], t, dest.joint_state_vector_[i]);
  dest.updateLinkTransforms();
}

const Eigen::Affine3d* kinematic_state::KinematicState::getFrameTransform(const std::string &id) const
{
  std::map<std::string, LinkState*>::const_iterator it = link_state_map_.find(id);
  if (it != link_state_map_.end())
    return &(it->second->getGlobalLinkTransform());
  std::map<std::string, AttachedBody*>::const_iterator jt = attached_body_map_.find(id);
  if (jt == attached_body_map_.end())
  {
    logError("Transform from frame '%s' to frame '%s' is not known ('%s' should be a link name or an attached body id).",
             kinematic_model_->getModelFrame().c_str(), id.c_str(), id.c_str());
    return NULL;
  }
  const EigenSTL::vector_Affine3d &tf = jt->second->getGlobalCollisionBodyTransforms();
  if (tf.empty())
  {
    logError("Attached body '%s' has no geometry associated to it. No transform to return.", id.c_str());
    return NULL;
  }
  if (tf.size() > 1)
    logWarn("There are multiple geometries associated to attached body '%s'. Returning the transform for the first one.", id.c_str());
  return &(tf[0]);
}

bool kinematic_state::KinematicState::knowsFrameTransform(const std::string &id) const
{ 
  if (hasLinkState(id))
    return true;
  std::map<std::string, AttachedBody*>::const_iterator it = attached_body_map_.find(id);
  return it != attached_body_map_.end() && it->second->getGlobalCollisionBodyTransforms().size() == 1;
}

// ------ marker functions ------

void kinematic_state::KinematicState::getRobotMarkers(visualization_msgs::MarkerArray& arr,
                                                      const std::vector<std::string> &link_names,
                                                      const std_msgs::ColorRGBA& color,
                                                      const std::string& ns,
                                                      const ros::Duration& dur) const
{
  std::size_t cur_num = arr.markers.size();
  getRobotMarkers(arr, link_names);
  unsigned int id = 0;
  for (std::size_t i = cur_num ; i < arr.markers.size() ; ++i, ++id)
  {
    arr.markers[i].ns = ns;
    arr.markers[i].id = id;
    arr.markers[i].lifetime = dur;
    arr.markers[i].color = color;
  }
}

void kinematic_state::KinematicState::getRobotMarkers(visualization_msgs::MarkerArray& arr, const std::vector<std::string> &link_names) const
{
  ros::Time tm = ros::Time::now();
  for (std::size_t i = 0; i < link_names.size(); ++i)
  {
    logDebug("Trying to get marker for link '%s'", link_names[i].c_str());
    visualization_msgs::Marker mark;
    const LinkState* ls = getLinkState(link_names[i]);
    if (!ls)
      continue;
    std::vector<const AttachedBody*> attached_bodies;
    ls->getAttachedBodies(attached_bodies);
    for(unsigned int j = 0; j < attached_bodies.size(); ++j)
      if(attached_bodies[j]->getShapes().size() > 0)
      {
        visualization_msgs::Marker att_mark;
        att_mark.header.frame_id = kinematic_model_->getModelFrame();
        att_mark.header.stamp = tm;
        shapes::constructMarkerFromShape(attached_bodies[j]->getShapes()[0].get(), att_mark);
        tf::poseEigenToMsg(attached_bodies[j]->getGlobalCollisionBodyTransforms()[0], att_mark.pose);
        arr.markers.push_back(att_mark);
      }
    if (!ls->getLinkModel() || !ls->getLinkModel()->getShape())
      continue;
    mark.header.frame_id = kinematic_model_->getModelFrame();
    mark.header.stamp = tm;
    tf::poseEigenToMsg(ls->getGlobalCollisionBodyTransform(), mark.pose);
    if (ls->getLinkModel()->getMeshFilename().empty())
      shapes::constructMarkerFromShape(ls->getLinkModel()->getShape().get(), mark);
    else
    {
      mark.type = mark.MESH_RESOURCE;
      if (!ls->getLinkModel()->getVisualMeshFilename().empty())
      {
        mark.mesh_use_embedded_materials = false;
        mark.mesh_resource = ls->getLinkModel()->getVisualMeshFilename();
      } 
      else
        mark.mesh_resource = ls->getLinkModel()->getMeshFilename();
      mark.scale.x = mark.scale.y = mark.scale.z = 1.0;
    }
    arr.markers.push_back(mark);
  }
}

// ------ printing transforms -----

void kinematic_state::KinematicState::printStateInfo(std::ostream &out) const
{
  std::map<std::string,double> val;
  getStateValues(val);
  for (std::map<std::string, double>::iterator it = val.begin() ; it != val.end() ; ++it)
    std::cout << it->first << " = " << it->second << std::endl;
}

void kinematic_state::KinematicState::printTransform(const std::string &st, const Eigen::Affine3d &t, std::ostream &out) const
{
  out << st << std::endl;
  const Eigen::Vector3d &v = t.translation();
  out << "  origin: " << v.x() << ", " << v.y() << ", " << v.z() << std::endl;
  Eigen::Quaterniond q(t.rotation());
  out << "  quaternion: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
}

void kinematic_state::KinematicState::printTransforms(std::ostream &out) const
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
