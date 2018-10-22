/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Ioan A. Sucan
*  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Dave Coleman */

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/revolute_joint_model.h>
#include <moveit/exceptions/exceptions.h>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include "order_robot_model_items.inc"

namespace moveit
{
namespace core
{
namespace
{
// check if a parent or ancestor of joint is included in this group
bool includesParent(const JointModel* joint, const JointModelGroup* group)
{
  bool found = false;
  // if we find that an ancestor is also in the group, then the joint is not a root
  while (joint->getParentLinkModel() != nullptr)
  {
    joint = joint->getParentLinkModel()->getParentJointModel();
    if (group->hasJointModel(joint->getName()) && joint->getVariableCount() > 0 && joint->getMimic() == nullptr)
    {
      found = true;
      break;
    }
    else if (joint->getMimic() != nullptr)
    {
      const JointModel* mjoint = joint->getMimic();
      if (group->hasJointModel(mjoint->getName()) && mjoint->getVariableCount() > 0 && mjoint->getMimic() == nullptr)
        found = true;
      else if (includesParent(mjoint, group))
        found = true;
      if (found)
        break;
    }
  }
  return found;
}

// check if joint a is right below b, in the kinematic chain, with no active DOF missing
bool jointPrecedes(const JointModel* a, const JointModel* b)
{
  if (!a->getParentLinkModel())
    return false;
  const JointModel* p = a->getParentLinkModel()->getParentJointModel();
  while (p)
  {
    if (p == b)
      return true;
    if (p->getType() == JointModel::FIXED)
      p = p->getParentLinkModel() ? p->getParentLinkModel()->getParentJointModel() : nullptr;
    else
      break;
  }

  return false;
}
}

JointModelGroup::JointModelGroup(const std::string& group_name, const srdf::Model::Group& config,
                                 const std::vector<const JointModel*>& unsorted_group_joints,
                                 const RobotModel* parent_model)
  : parent_model_(parent_model)
  , name_(group_name)
  , common_root_(nullptr)
  , variable_count_(0)
  , is_contiguous_index_list_(true)
  , is_chain_(false)
  , is_single_dof_(true)
  , config_(config)
{
  // sort joints in Depth-First order
  joint_model_vector_ = unsorted_group_joints;
  std::sort(joint_model_vector_.begin(), joint_model_vector_.end(), OrderJointsByIndex());
  joint_model_name_vector_.reserve(joint_model_vector_.size());

  // figure out active joints, mimic joints, fixed joints
  // construct index maps, list of variables
  for (std::size_t i = 0; i < joint_model_vector_.size(); ++i)
  {
    joint_model_name_vector_.push_back(joint_model_vector_[i]->getName());
    joint_model_map_[joint_model_vector_[i]->getName()] = joint_model_vector_[i];
    unsigned int vc = joint_model_vector_[i]->getVariableCount();
    if (vc > 0)
    {
      if (vc > 1)
        is_single_dof_ = false;
      const std::vector<std::string>& name_order = joint_model_vector_[i]->getVariableNames();
      if (joint_model_vector_[i]->getMimic() == nullptr)
      {
        active_joint_model_vector_.push_back(joint_model_vector_[i]);
        active_joint_model_name_vector_.push_back(joint_model_vector_[i]->getName());
        active_joint_model_start_index_.push_back(variable_count_);
        active_joint_models_bounds_.push_back(&joint_model_vector_[i]->getVariableBounds());
      }
      else
        mimic_joints_.push_back(joint_model_vector_[i]);
      for (std::size_t j = 0; j < name_order.size(); ++j)
      {
        variable_names_.push_back(name_order[j]);
        variable_names_set_.insert(name_order[j]);
      }

      int first_index = joint_model_vector_[i]->getFirstVariableIndex();
      for (std::size_t j = 0; j < name_order.size(); ++j)
      {
        variable_index_list_.push_back(first_index + j);
        joint_variables_index_map_[name_order[j]] = variable_count_ + j;
      }
      joint_variables_index_map_[joint_model_vector_[i]->getName()] = variable_count_;

      if (joint_model_vector_[i]->getType() == JointModel::REVOLUTE &&
          static_cast<const RevoluteJointModel*>(joint_model_vector_[i])->isContinuous())
        continuous_joint_model_vector_.push_back(joint_model_vector_[i]);

      variable_count_ += vc;
    }
    else
      fixed_joints_.push_back(joint_model_vector_[i]);
  }

  // now we need to find all the set of joints within this group
  // that root distinct subtrees
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
  {
    // if we find that an ancestor is also in the group, then the joint is not a root
    if (!includesParent(active_joint_model_vector_[i], this))
      joint_roots_.push_back(active_joint_model_vector_[i]);
  }

  // when updating this group within a state, it is useful to know
  // if the full state of a group is contiguous within the full state of the robot
  if (variable_index_list_.empty())
    is_contiguous_index_list_ = false;
  else
    for (std::size_t i = 1; i < variable_index_list_.size(); ++i)
      if (variable_index_list_[i] != variable_index_list_[i - 1] + 1)
      {
        is_contiguous_index_list_ = false;
        break;
      }

  // when updating/sampling a group state only, only mimic joints that have their parent within the group get updated.
  for (std::size_t i = 0; i < mimic_joints_.size(); ++i)
    // if the joint we mimic is also in this group, we will need to do updates when sampling
    if (hasJointModel(mimic_joints_[i]->getMimic()->getName()))
    {
      int src = joint_variables_index_map_[mimic_joints_[i]->getMimic()->getName()];
      int dest = joint_variables_index_map_[mimic_joints_[i]->getName()];
      GroupMimicUpdate mu(src, dest, mimic_joints_[i]->getMimicFactor(), mimic_joints_[i]->getMimicOffset());
      group_mimic_update_.push_back(mu);
    }

  // now we need to make another pass for group links (we include the fixed joints here)
  std::set<const LinkModel*> group_links_set;
  for (std::size_t i = 0; i < joint_model_vector_.size(); ++i)
    group_links_set.insert(joint_model_vector_[i]->getChildLinkModel());
  for (std::set<const LinkModel*>::iterator it = group_links_set.begin(); it != group_links_set.end(); ++it)
    link_model_vector_.push_back(*it);
  std::sort(link_model_vector_.begin(), link_model_vector_.end(), OrderLinksByIndex());

  for (std::size_t i = 0; i < link_model_vector_.size(); ++i)
  {
    link_model_map_[link_model_vector_[i]->getName()] = link_model_vector_[i];
    link_model_name_vector_.push_back(link_model_vector_[i]->getName());
    if (!link_model_vector_[i]->getShapes().empty())
    {
      link_model_with_geometry_vector_.push_back(link_model_vector_[i]);
      link_model_with_geometry_name_vector_.push_back(link_model_vector_[i]->getName());
    }
  }

  // compute the common root of this group
  if (!joint_roots_.empty())
  {
    common_root_ = joint_roots_[0];
    for (std::size_t i = 1; i < joint_roots_.size(); ++i)
      common_root_ = parent_model->getCommonRoot(joint_roots_[i], common_root_);
  }

  // compute updated links
  for (std::size_t i = 0; i < joint_roots_.size(); ++i)
  {
    const std::vector<const LinkModel*>& links = joint_roots_[i]->getDescendantLinkModels();
    updated_link_model_set_.insert(links.begin(), links.end());
  }
  for (std::set<const LinkModel*>::iterator it = updated_link_model_set_.begin(); it != updated_link_model_set_.end();
       ++it)
  {
    updated_link_model_name_set_.insert((*it)->getName());
    updated_link_model_vector_.push_back(*it);
    if (!(*it)->getShapes().empty())
    {
      updated_link_model_with_geometry_vector_.push_back(*it);
      updated_link_model_with_geometry_set_.insert(*it);
      updated_link_model_with_geometry_name_set_.insert((*it)->getName());
    }
  }
  std::sort(updated_link_model_vector_.begin(), updated_link_model_vector_.end(), OrderLinksByIndex());
  std::sort(updated_link_model_with_geometry_vector_.begin(), updated_link_model_with_geometry_vector_.end(),
            OrderLinksByIndex());
  for (std::size_t i = 0; i < updated_link_model_vector_.size(); ++i)
    updated_link_model_name_vector_.push_back(updated_link_model_vector_[i]->getName());
  for (std::size_t i = 0; i < updated_link_model_with_geometry_vector_.size(); ++i)
    updated_link_model_with_geometry_name_vector_.push_back(updated_link_model_with_geometry_vector_[i]->getName());

  // check if this group should actually be a chain
  if (joint_roots_.size() == 1 && active_joint_model_vector_.size() >= 1)
  {
    bool chain = true;
    // due to our sorting, the joints are sorted in a DF fashion, so looking at them in reverse,
    // we should always get to the parent.
    for (std::size_t k = joint_model_vector_.size() - 1; k > 0; --k)
      if (!jointPrecedes(joint_model_vector_[k], joint_model_vector_[k - 1]))
      {
        chain = false;
        break;
      }
    if (chain)
      is_chain_ = true;
  }
}

JointModelGroup::~JointModelGroup() = default;

void JointModelGroup::setSubgroupNames(const std::vector<std::string>& subgroups)
{
  subgroup_names_ = subgroups;
  subgroup_names_set_.clear();
  for (std::size_t i = 0; i < subgroup_names_.size(); ++i)
    subgroup_names_set_.insert(subgroup_names_[i]);
}

void JointModelGroup::getSubgroups(std::vector<const JointModelGroup*>& sub_groups) const
{
  sub_groups.resize(subgroup_names_.size());
  for (std::size_t i = 0; i < subgroup_names_.size(); ++i)
    sub_groups[i] = parent_model_->getJointModelGroup(subgroup_names_[i]);
}

bool JointModelGroup::hasJointModel(const std::string& joint) const
{
  return joint_model_map_.find(joint) != joint_model_map_.end();
}

bool JointModelGroup::hasLinkModel(const std::string& link) const
{
  return link_model_map_.find(link) != link_model_map_.end();
}

const LinkModel* JointModelGroup::getLinkModel(const std::string& name) const
{
  LinkModelMapConst::const_iterator it = link_model_map_.find(name);
  if (it == link_model_map_.end())
  {
    ROS_ERROR_NAMED("robot_model.jmg", "Link '%s' not found in group '%s'", name.c_str(), name_.c_str());
    return nullptr;
  }
  return it->second;
}

const JointModel* JointModelGroup::getJointModel(const std::string& name) const
{
  JointModelMapConst::const_iterator it = joint_model_map_.find(name);
  if (it == joint_model_map_.end())
  {
    ROS_ERROR_NAMED("robot_model.jmg", "Joint '%s' not found in group '%s'", name.c_str(), name_.c_str());
    return nullptr;
  }
  return it->second;
}

void JointModelGroup::getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values,
                                                 const JointBoundsVector& active_joint_bounds) const
{
  assert(active_joint_bounds.size() == active_joint_model_vector_.size());
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    active_joint_model_vector_[i]->getVariableRandomPositions(rng, values + active_joint_model_start_index_[i],
                                                              *active_joint_bounds[i]);

  updateMimicJoints(values);
}

void JointModelGroup::getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator& rng, double* values,
                                                       const JointBoundsVector& active_joint_bounds, const double* near,
                                                       double distance) const
{
  assert(active_joint_bounds.size() == active_joint_model_vector_.size());
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    active_joint_model_vector_[i]->getVariableRandomPositionsNearBy(
        rng, values + active_joint_model_start_index_[i], *active_joint_bounds[i],
        near + active_joint_model_start_index_[i], distance);
  updateMimicJoints(values);
}

void JointModelGroup::getVariableRandomPositionsNearBy(
    random_numbers::RandomNumberGenerator& rng, double* values, const JointBoundsVector& active_joint_bounds,
    const double* near, const std::map<JointModel::JointType, double>& distance_map) const
{
  assert(active_joint_bounds.size() == active_joint_model_vector_.size());
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
  {
    double distance = 0.0;
    std::map<JointModel::JointType, double>::const_iterator iter =
        distance_map.find(active_joint_model_vector_[i]->getType());
    if (iter != distance_map.end())
      distance = iter->second;
    else
      ROS_WARN_NAMED("robot_model.jmg", "Did not pass in distance for '%s'",
                     active_joint_model_vector_[i]->getName().c_str());
    active_joint_model_vector_[i]->getVariableRandomPositionsNearBy(
        rng, values + active_joint_model_start_index_[i], *active_joint_bounds[i],
        near + active_joint_model_start_index_[i], distance);
  }
  updateMimicJoints(values);
}

void JointModelGroup::getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator& rng, double* values,
                                                       const JointBoundsVector& active_joint_bounds, const double* near,
                                                       const std::vector<double>& distances) const
{
  assert(active_joint_bounds.size() == active_joint_model_vector_.size());
  if (distances.size() != active_joint_model_vector_.size())
    throw Exception("When sampling random values nearby for group '" + name_ +
                    "', distances vector should be of size " +
                    boost::lexical_cast<std::string>(active_joint_model_vector_.size()) + ", but it is of size " +
                    boost::lexical_cast<std::string>(distances.size()));
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    active_joint_model_vector_[i]->getVariableRandomPositionsNearBy(
        rng, values + active_joint_model_start_index_[i], *active_joint_bounds[i],
        near + active_joint_model_start_index_[i], distances[i]);
  updateMimicJoints(values);
}

bool JointModelGroup::satisfiesPositionBounds(const double* state, const JointBoundsVector& active_joint_bounds,
                                              double margin) const
{
  assert(active_joint_bounds.size() == active_joint_model_vector_.size());
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    if (!active_joint_model_vector_[i]->satisfiesPositionBounds(state + active_joint_model_start_index_[i],
                                                                *active_joint_bounds[i], margin))
      return false;
  return true;
}

bool JointModelGroup::enforcePositionBounds(double* state, const JointBoundsVector& active_joint_bounds) const
{
  assert(active_joint_bounds.size() == active_joint_model_vector_.size());
  bool change = false;
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    if (active_joint_model_vector_[i]->enforcePositionBounds(state + active_joint_model_start_index_[i],
                                                             *active_joint_bounds[i]))
      change = true;
  if (change)
    updateMimicJoints(state);
  return change;
}

double JointModelGroup::getMaximumExtent(const JointBoundsVector& active_joint_bounds) const
{
  double max_distance = 0.0;
  for (std::size_t j = 0; j < active_joint_model_vector_.size(); ++j)
    max_distance += active_joint_model_vector_[j]->getMaximumExtent(*active_joint_bounds[j]) *
                    active_joint_model_vector_[j]->getDistanceFactor();
  return max_distance;
}

double JointModelGroup::distance(const double* state1, const double* state2) const
{
  double d = 0.0;
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    d += active_joint_model_vector_[i]->getDistanceFactor() *
         active_joint_model_vector_[i]->distance(state1 + active_joint_model_start_index_[i],
                                                 state2 + active_joint_model_start_index_[i]);
  return d;
}

void JointModelGroup::interpolate(const double* from, const double* to, double t, double* state) const
{
  // we interpolate values only for active joint models (non-mimic)
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    active_joint_model_vector_[i]->interpolate(from + active_joint_model_start_index_[i],
                                               to + active_joint_model_start_index_[i], t,
                                               state + active_joint_model_start_index_[i]);

  // now we update mimic as needed
  updateMimicJoints(state);
}

void JointModelGroup::updateMimicJoints(double* values) const
{
  // update mimic (only local joints as we are dealing with a local group state)
  for (std::size_t i = 0; i < group_mimic_update_.size(); ++i)
    values[group_mimic_update_[i].dest] =
        values[group_mimic_update_[i].src] * group_mimic_update_[i].factor + group_mimic_update_[i].offset;
}

void JointModelGroup::addDefaultState(const std::string& name, const std::map<std::string, double>& default_state)
{
  default_states_[name] = default_state;
  default_states_names_.push_back(name);
}

bool JointModelGroup::getVariableDefaultPositions(const std::string& name, std::map<std::string, double>& values) const
{
  std::map<std::string, std::map<std::string, double> >::const_iterator it = default_states_.find(name);
  if (it == default_states_.end())
    return false;
  values = it->second;
  return true;
}

void JointModelGroup::getVariableDefaultPositions(double* values) const
{
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    active_joint_model_vector_[i]->getVariableDefaultPositions(values + active_joint_model_start_index_[i]);
  updateMimicJoints(values);
}

void JointModelGroup::getVariableDefaultPositions(std::map<std::string, double>& values) const
{
  std::vector<double> tmp(variable_count_);
  getVariableDefaultPositions(&tmp[0]);
  for (std::size_t i = 0; i < variable_names_.size(); ++i)
    values[variable_names_[i]] = tmp[i];
}

void JointModelGroup::setEndEffectorName(const std::string& name)
{
  end_effector_name_ = name;
}

void JointModelGroup::setEndEffectorParent(const std::string& group, const std::string& link)
{
  end_effector_parent_.first = group;
  end_effector_parent_.second = link;
}

void JointModelGroup::attachEndEffector(const std::string& eef_name)
{
  attached_end_effector_names_.push_back(eef_name);
}

bool JointModelGroup::getEndEffectorTips(std::vector<std::string>& tips) const
{
  // Get a vector of tip links
  std::vector<const LinkModel*> tip_links;
  if (!getEndEffectorTips(tip_links))
    return false;

  // Convert to string names
  tips.clear();
  for (std::size_t i = 0; i < tip_links.size(); ++i)
  {
    tips.push_back(tip_links[i]->getName());
  }
  return true;
}

bool JointModelGroup::getEndEffectorTips(std::vector<const LinkModel*>& tips) const
{
  for (std::size_t i = 0; i < getAttachedEndEffectorNames().size(); ++i)
  {
    const JointModelGroup* eef = parent_model_->getEndEffector(getAttachedEndEffectorNames()[i]);
    if (!eef)
    {
      ROS_ERROR_NAMED("robot_model.jmg", "Unable to find joint model group for eef");
      return false;
    }
    const std::string& eef_parent = eef->getEndEffectorParentGroup().second;

    const LinkModel* eef_link = parent_model_->getLinkModel(eef_parent);
    if (!eef_link)
    {
      ROS_ERROR_NAMED("robot_model.jmg", "Unable to find end effector link for eef");
      return false;
    }

    tips.push_back(eef_link);
  }
  return true;
}

const LinkModel* JointModelGroup::getOnlyOneEndEffectorTip() const
{
  std::vector<const LinkModel*> tips;
  getEndEffectorTips(tips);
  if (tips.size() == 1)
    return tips.front();
  else if (tips.size() > 1)
    ROS_ERROR_NAMED("robot_model.jmg", "More than one end effector tip found for joint model group, "
                                       "so cannot return only one");
  else
    ROS_ERROR_NAMED("robot_model.jmg", "No end effector tips found in joint model group");
  return nullptr;
}

int JointModelGroup::getVariableGroupIndex(const std::string& variable) const
{
  VariableIndexMap::const_iterator it = joint_variables_index_map_.find(variable);
  if (it == joint_variables_index_map_.end())
  {
    ROS_ERROR_NAMED("robot_model.jmg", "Variable '%s' is not part of group '%s'", variable.c_str(), name_.c_str());
    return -1;
  }
  return it->second;
}

void JointModelGroup::setDefaultIKTimeout(double ik_timeout)
{
  group_kinematics_.first.default_ik_timeout_ = ik_timeout;
  if (group_kinematics_.first.solver_instance_)
    group_kinematics_.first.solver_instance_->setDefaultTimeout(ik_timeout);
  for (KinematicsSolverMap::iterator it = group_kinematics_.second.begin(); it != group_kinematics_.second.end(); ++it)
    it->second.default_ik_timeout_ = ik_timeout;
}

void JointModelGroup::setDefaultIKAttempts(unsigned int ik_attempts)
{
  group_kinematics_.first.default_ik_attempts_ = ik_attempts;
  for (KinematicsSolverMap::iterator it = group_kinematics_.second.begin(); it != group_kinematics_.second.end(); ++it)
    it->second.default_ik_attempts_ = ik_attempts;
}

bool JointModelGroup::computeIKIndexBijection(const std::vector<std::string>& ik_jnames,
                                              std::vector<unsigned int>& joint_bijection) const
{
  joint_bijection.clear();
  for (std::size_t i = 0; i < ik_jnames.size(); ++i)
  {
    VariableIndexMap::const_iterator it = joint_variables_index_map_.find(ik_jnames[i]);
    if (it == joint_variables_index_map_.end())
    {
      // skip reported fixed joints
      if (hasJointModel(ik_jnames[i]) && getJointModel(ik_jnames[i])->getType() == JointModel::FIXED)
        continue;
      ROS_ERROR_NAMED("robot_model.jmg", "IK solver computes joint values for joint '%s' "
                                         "but group '%s' does not contain such a joint.",
                      ik_jnames[i].c_str(), getName().c_str());
      return false;
    }
    const JointModel* jm = getJointModel(ik_jnames[i]);
    for (unsigned int k = 0; k < jm->getVariableCount(); ++k)
      joint_bijection.push_back(it->second + k);
  }
  return true;
}

void JointModelGroup::setSolverAllocators(const std::pair<SolverAllocatorFn, SolverAllocatorMapFn>& solvers)
{
  if (solvers.first)
  {
    group_kinematics_.first.allocator_ = solvers.first;
    group_kinematics_.first.solver_instance_ = solvers.first(this);
    if (group_kinematics_.first.solver_instance_)
    {
      group_kinematics_.first.solver_instance_->setDefaultTimeout(group_kinematics_.first.default_ik_timeout_);
      group_kinematics_.first.solver_instance_const_ = group_kinematics_.first.solver_instance_;
      if (!computeIKIndexBijection(group_kinematics_.first.solver_instance_->getJointNames(),
                                   group_kinematics_.first.bijection_))
        group_kinematics_.first.reset();
    }
  }
  else
    // we now compute a joint bijection only if we have a solver map
    for (SolverAllocatorMapFn::const_iterator it = solvers.second.begin(); it != solvers.second.end(); ++it)
      if (it->first->getSolverInstance())
      {
        KinematicsSolver& ks = group_kinematics_.second[it->first];
        ks.allocator_ = it->second;
        ks.solver_instance_ = const_cast<JointModelGroup*>(it->first)->getSolverInstance();
        ks.solver_instance_const_ = ks.solver_instance_;
        ks.default_ik_timeout_ = group_kinematics_.first.default_ik_timeout_;
        ks.default_ik_attempts_ = group_kinematics_.first.default_ik_attempts_;
        if (!computeIKIndexBijection(ks.solver_instance_->getJointNames(), ks.bijection_))
        {
          group_kinematics_.second.clear();
          break;
        }
      }
}

bool JointModelGroup::canSetStateFromIK(const std::string& tip) const
{
  const kinematics::KinematicsBaseConstPtr& solver = getSolverInstance();
  if (!solver || tip.empty())
    return false;

  const std::vector<std::string>& tip_frames = solver->getTipFrames();

  if (tip_frames.empty())
  {
    ROS_DEBUG_NAMED("robot_model.jmg", "Group %s has no tip frame(s)", name_.c_str());
    return false;
  }

  // loop through all tip frames supported by the JMG
  for (std::size_t i = 0; i < tip_frames.size(); ++i)
  {
    // remove frame reference, if specified
    const std::string& tip_local = tip[0] == '/' ? tip.substr(1) : tip;
    const std::string& tip_frame_local = tip_frames[i][0] == '/' ? tip_frames[i].substr(1) : tip_frames[i];
    ROS_DEBUG_NAMED("robot_model.jmg", "comparing input tip: %s to this groups tip: %s ", tip_local.c_str(),
                    tip_frame_local.c_str());

    // Check if the IK solver's tip is the same as the frame of inquiry
    if (tip_local != tip_frame_local)
    {
      // If not the same, check if this planning group includes the frame of inquiry
      if (hasLinkModel(tip_frame_local))
      {
        const LinkModel* lm = getLinkModel(tip_frame_local);
        const LinkTransformMap& fixed_links = lm->getAssociatedFixedTransforms();
        // Check if our frame of inquiry is located anywhere further down the chain (towards the tip of the arm)
        for (LinkTransformMap::const_iterator it = fixed_links.begin(); it != fixed_links.end(); ++it)
        {
          if (it->first->getName() == tip_local)
            return true;
        }
      }
    }
    else
      return true;
  }

  // Did not find any valid tip frame links to use
  return false;
}

void JointModelGroup::printGroupInfo(std::ostream& out) const
{
  out << "Group '" << name_ << "' using " << variable_count_ << " variables" << std::endl;
  out << "  * Joints:" << std::endl;
  for (std::size_t i = 0; i < joint_model_vector_.size(); ++i)
    out << "    '" << joint_model_vector_[i]->getName() << "' (" << joint_model_vector_[i]->getTypeName() << ")"
        << std::endl;
  out << "  * Variables:" << std::endl;
  for (std::size_t i = 0; i < variable_names_.size(); ++i)
  {
    int local_idx = joint_variables_index_map_.find(variable_names_[i])->second;
    const JointModel* jm = parent_model_->getJointOfVariable(variable_names_[i]);
    out << "    '" << variable_names_[i] << "', index "
        << (jm->getFirstVariableIndex() + jm->getLocalVariableIndex(variable_names_[i])) << " in full state, index "
        << local_idx << " in group state";
    if (jm->getMimic())
      out << ", mimic '" << jm->getMimic()->getName() << "'";
    out << std::endl;
    out << "        " << parent_model_->getVariableBounds(variable_names_[i]) << std::endl;
  }
  out << "  * Variables Index List:" << std::endl;
  out << "    ";
  for (std::size_t i = 0; i < variable_index_list_.size(); ++i)
    out << variable_index_list_[i] << " ";
  if (is_contiguous_index_list_)
    out << "(contiguous)";
  else
    out << "(non-contiguous)";
  out << std::endl;
  if (group_kinematics_.first)
  {
    out << "  * Kinematics solver bijection:" << std::endl;
    out << "    ";
    for (std::size_t i = 0; i < group_kinematics_.first.bijection_.size(); ++i)
      out << group_kinematics_.first.bijection_[i] << " ";
    out << std::endl;
  }
  if (!group_kinematics_.second.empty())
  {
    out << "  * Compound kinematics solver:" << std::endl;
    for (KinematicsSolverMap::const_iterator it = group_kinematics_.second.begin();
         it != group_kinematics_.second.end(); ++it)
    {
      out << "    " << it->first->getName() << ":";
      for (std::size_t i = 0; i < it->second.bijection_.size(); ++i)
        out << " " << it->second.bijection_[i];
      out << std::endl;
    }
  }

  if (!group_mimic_update_.empty())
  {
    out << "  * Local Mimic Updates:" << std::endl;
    for (std::size_t i = 0; i < group_mimic_update_.size(); ++i)
      out << "    [" << group_mimic_update_[i].dest << "] = " << group_mimic_update_[i].factor << " * ["
          << group_mimic_update_[i].src << "] + " << group_mimic_update_[i].offset << std::endl;
  }
  out << std::endl;
}

}  // end of namespace core
}  // end of namespace moveit
