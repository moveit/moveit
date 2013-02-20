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

/* Author: Ioan Sucan */

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <algorithm>

namespace robot_model
{
namespace
{

bool orderLinksByIndex(const LinkModel *a, const LinkModel *b)
{
  return a->getTreeIndex() < b->getTreeIndex();
}

bool orderJointsByIndex(const JointModel *a, const JointModel *b)
{
  return a->getTreeIndex() < b->getTreeIndex();
}

bool includesParent(const JointModel *joint, const JointModelGroup *group)
{
  bool found = false;
  // if we find that an ancestor is also in the group, then the joint is not a root
  while (joint->getParentLinkModel() != NULL)
  {
    joint = joint->getParentLinkModel()->getParentJointModel();
    if (group->hasJointModel(joint->getName()) && joint->getVariableCount() > 0 && joint->getMimic() == NULL)
    {
      found = true;
      break;
    }
    else
      if (joint->getMimic() != NULL)
      {
        const JointModel *mjoint = joint->getMimic();
        if (group->hasJointModel(mjoint->getName()) && mjoint->getVariableCount() > 0 && mjoint->getMimic() == NULL)
          found = true;
        else
          if (includesParent(mjoint, group))
            found = true;
        if (found)
          break;
      }
  }
  return found;
}

}
}

robot_model::JointModelGroup::JointModelGroup(const std::string& group_name,
                                                  const std::vector<const JointModel*> &unsorted_group_joints,
                                                  const RobotModel* parent_model) :
  parent_model_(parent_model), name_(group_name),
  variable_count_(0), is_end_effector_(false), is_chain_(false),
  default_ik_timeout_(0.5), default_ik_attempts_(2)
{
  // sort joints in Depth-First order
  std::vector<const JointModel*> group_joints = unsorted_group_joints;
  std::sort(group_joints.begin(), group_joints.end(), &orderJointsByIndex);
  
  for (std::size_t i = 0 ; i < group_joints.size() ; ++i)
  {
    joint_model_map_[group_joints[i]->getName()] = group_joints[i];
    unsigned int vc = group_joints[i]->getVariableCount();
    if (vc > 0)
    {
      if (group_joints[i]->getMimic() == NULL)
      {
        joint_model_vector_.push_back(group_joints[i]);
        variable_count_ += vc;
      }
      else
        mimic_joints_.push_back(group_joints[i]);

      if (group_joints[i]->getType() == JointModel::REVOLUTE && static_cast<const RevoluteJointModel*>(group_joints[i])->isContinuous())
        continuous_joint_model_vector_const_.push_back(group_joints[i]);
    }
    else
      fixed_joints_.push_back(group_joints[i]);
  }
  
  //now we need to find all the set of joints within this group
  //that root distinct subtrees
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
  {
    joint_model_name_vector_.push_back(joint_model_vector_[i]->getName());
    bool found = false;
    const JointModel *joint = joint_model_vector_[i];
    // if we find that an ancestor is also in the group, then the joint is not a root
    if (!includesParent(joint, this))
      joint_roots_.push_back(joint_model_vector_[i]);
  }
  
  // compute joint_variables_index_map_
  unsigned int vector_index_counter = 0;
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
  {
    const std::vector<std::string>& name_order = joint_model_vector_[i]->getVariableNames();
    for (std::size_t j = 0; j < name_order.size(); ++j)
    {
      joint_variables_index_map_[name_order[j]] = vector_index_counter + j;
      active_variable_names_.push_back(name_order[j]);
      active_variable_names_set_.insert(name_order[j]);
    }
    joint_variables_index_map_[joint_model_vector_[i]->getName()] = vector_index_counter;
    vector_index_counter += name_order.size();
  }

  for (std::size_t i = 0 ; i < mimic_joints_.size() ; ++i)
  {
    const std::vector<std::string>& name_order = mimic_joints_[i]->getVariableNames();
    const std::vector<std::string>& mim_name_order = mimic_joints_[i]->getMimic()->getVariableNames();
    for (std::size_t j = 0; j < name_order.size(); ++j)
      joint_variables_index_map_[name_order[j]] = joint_variables_index_map_[mim_name_order[j]];
    joint_variables_index_map_[mimic_joints_[i]->getName()] = joint_variables_index_map_[mimic_joints_[i]->getMimic()->getName()];
  }

  // now we need to make another pass for group links (we include the fixed joints here)
  std::set<const LinkModel*> group_links_set;
  for (std::size_t i = 0 ; i < group_joints.size() ; ++i)
    group_links_set.insert(group_joints[i]->getChildLinkModel());
  
  for (std::set<const LinkModel*>::iterator it = group_links_set.begin(); it != group_links_set.end(); ++it)
    link_model_vector_.push_back(*it);
  
  std::sort(link_model_vector_.begin(), link_model_vector_.end(), &orderLinksByIndex);
  for (std::size_t i = 0 ; i < link_model_vector_.size() ; ++i)
  {
    link_model_name_vector_.push_back(link_model_vector_[i]->getName());
    if (link_model_vector_[i]->getShape())
      link_model_with_geometry_name_vector_.push_back(link_model_vector_[i]->getName());
  }
  
  // compute updated links
  std::set<const LinkModel*> u_links;
  for (std::size_t i = 0 ; i < joint_roots_.size() ; ++i)
  {
    std::vector<const LinkModel*> links;
    parent_model->getChildLinkModels(joint_roots_[i], links);
    u_links.insert(links.begin(), links.end());
  }
  for (std::set<const LinkModel*>::iterator it = u_links.begin(); it != u_links.end(); ++it)
  {
    updated_link_model_vector_.push_back(*it);
    updated_link_model_set_.insert(*it);
    updated_link_model_name_set_.insert((*it)->getName());
    if ((*it)->getShape())
    {
      updated_link_model_with_geometry_vector_.push_back(*it);
      updated_link_model_with_geometry_set_.insert(*it); 
      updated_link_model_with_geometry_name_set_.insert((*it)->getName());
    }
  }
  std::sort(updated_link_model_vector_.begin(), updated_link_model_vector_.end(), &orderLinksByIndex);
  std::sort(updated_link_model_with_geometry_vector_.begin(), updated_link_model_with_geometry_vector_.end(), &orderLinksByIndex);
  for (std::size_t i = 0; i < updated_link_model_vector_.size(); ++i)
    updated_link_model_name_vector_.push_back(updated_link_model_vector_[i]->getName());
  for (std::size_t i = 0; i < updated_link_model_with_geometry_vector_.size(); ++i)
    updated_link_model_with_geometry_name_vector_.push_back(updated_link_model_with_geometry_vector_[i]->getName());

  // check if this group should actually be a chain
  if (joint_roots_.size() == 1 && joint_model_vector_.size() > 1 && !is_chain_)
  {
    bool chain = true;
    // this would be the leaf of the chain, due to our sorting
    for (std::size_t k = group_joints.size() - 1 ; k > 0 ; --k)
      if (!group_joints[k]->getParentLinkModel() || group_joints[k]->getParentLinkModel()->getParentJointModel() != group_joints[k-1])
      {
        chain = false;
        break;
      }    
    if (chain)
      is_chain_ = true;  
  }
}

robot_model::JointModelGroup::~JointModelGroup()
{
}

bool robot_model::JointModelGroup::isSubgroup(const std::string& group) const
{
  for (std::size_t i = 0; i < subgroup_names_.size(); ++i)
    if (group == subgroup_names_[i])
      return true;
  return false;
}

bool robot_model::JointModelGroup::hasJointModel(const std::string &joint) const
{
  return joint_model_map_.find(joint) != joint_model_map_.end();
}

bool robot_model::JointModelGroup::hasLinkModel(const std::string &link) const
{
  return std::find(link_model_name_vector_.begin(), link_model_name_vector_.end(), link) != link_model_name_vector_.end();
}

const robot_model::JointModel* robot_model::JointModelGroup::getJointModel(const std::string &name) const
{
  std::map<std::string, const JointModel*>::const_iterator it = joint_model_map_.find(name);
  if (it == joint_model_map_.end())
  {
    logError("Joint '%s' not found in group '%s'", name.c_str(), name_.c_str());
    return NULL;
  }
  else
    return it->second;
}

void robot_model::JointModelGroup::getVariableRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values) const
{
  for (std::size_t i = 0  ; i < joint_model_vector_.size() ; ++i)
    joint_model_vector_[i]->getVariableRandomValues(rng, values);
}

void robot_model::JointModelGroup::getVariableRandomValuesNearBy(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values, const std::vector<double> &near, const std::map<robot_model::JointModel::JointType, double> &distance_map) const
{
  if (near.size() != variable_count_)
  {
    logError("The specification of the near-by variable values to sample around for group '%s' should be of size %u but it is of size %u",
             name_.c_str(), variable_count_, (unsigned int)near.size());
    return;
  }  
  
  for (std::size_t i = 0  ; i < joint_model_vector_.size() ; ++i)
  {
    double distance = 0.0;    
    std::map<robot_model::JointModel::JointType, double>::const_iterator iter = distance_map.find(joint_model_vector_[i]->getType());
    if (iter != distance_map.end())
      distance = iter->second;    
    else
      logWarn("Did not pass in distance for %s",joint_model_vector_[i]->getName().c_str());    
    joint_model_vector_[i]->getVariableRandomValuesNearBy(rng, values, near, distance);
  }  
}

void robot_model::JointModelGroup::getVariableRandomValuesNearBy(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values, const std::vector<double> &near, const std::vector<double> &distances) const
{
  if (distances.size() != joint_model_vector_.size())
  {
    logError("When sampling random values nearby for group '%s', distances vector should be of size %u, but it is of size %u",
             name_.c_str(), (unsigned int)joint_model_vector_.size(), (unsigned int)distances.size());
    return;    
  } 
  if (near.size() != variable_count_)
  {
    logError("The specification of the near-by variable values to sample around for group '%s' should be of size %u but it is of size %u",
             name_.c_str(), variable_count_, (unsigned int)near.size());
    return;
  }  

  for (std::size_t i = 0  ; i < joint_model_vector_.size() ; ++i)
    joint_model_vector_[i]->getVariableRandomValuesNearBy(rng, values, near, distances[i]);
}

double robot_model::JointModelGroup::getMaximumExtent(void) const
{
  double max_distance = 0.0;
  for (std::size_t j = 0 ; j < joint_model_vector_.size() ; ++j)
    max_distance += joint_model_vector_[j]->getMaximumExtent() * joint_model_vector_[j]->getDistanceFactor();
  return max_distance;
}

void robot_model::JointModelGroup::getKnownDefaultStates(std::vector<std::string> &default_states) const
{  
  default_states.clear();
  default_states.reserve(default_states_.size());
  for (std::map<std::string, std::map<std::string, double> >::const_iterator it = default_states_.begin() ; it != default_states_.end() ; ++it)
    default_states.push_back(it->first);
}

bool robot_model::JointModelGroup::getVariableDefaultValues(const std::string &name, std::map<std::string, double> &values) const
{
  std::map<std::string, std::map<std::string, double> >::const_iterator it = default_states_.find(name);
  if (it == default_states_.end())
    return false;
  values = it->second;
  return true;
}

void robot_model::JointModelGroup::getVariableDefaultValues(std::vector<double> &values) const
{
  values.reserve(values.size() + joint_model_vector_.size());
  for (std::size_t i = 0  ; i < joint_model_vector_.size() ; ++i)
    joint_model_vector_[i]->getVariableDefaultValues(values);
}

void robot_model::JointModelGroup::getVariableDefaultValues(std::map<std::string, double> &values) const
{
  for (std::size_t i = 0  ; i < joint_model_vector_.size() ; ++i)
    joint_model_vector_[i]->getVariableDefaultValues(values);
}

std::vector<moveit_msgs::JointLimits> robot_model::JointModelGroup::getVariableDefaultLimits() const
{
  std::vector<moveit_msgs::JointLimits> ret_vec;
  for(unsigned int i = 0; i < joint_model_vector_.size(); i++)
  {
    const std::vector<moveit_msgs::JointLimits> &jvec = joint_model_vector_[i]->getVariableDefaultLimits();
    ret_vec.insert(ret_vec.end(), jvec.begin(), jvec.end());
  }
  return ret_vec;
}

std::vector<moveit_msgs::JointLimits> robot_model::JointModelGroup::getVariableLimits() const
{
  std::vector<moveit_msgs::JointLimits> ret_vec;
  for(unsigned int i = 0; i < joint_model_vector_.size(); i++)
  {
    const std::vector<moveit_msgs::JointLimits> &jvec = joint_model_vector_[i]->getVariableLimits();
    ret_vec.insert(ret_vec.end(), jvec.begin(), jvec.end());
  }
  return ret_vec;
}

void robot_model::JointModelGroup::setVariableLimits(const std::vector<moveit_msgs::JointLimits>& jlim)
{
  // the following const_cast is safe because we are in a non-const function that operates on the same model
  // the joint is part of
  for (unsigned int i = 0; i < joint_model_vector_.size(); i++)
    const_cast<JointModel*>(joint_model_vector_[i])->setVariableLimits(jlim);
}

void robot_model::JointModelGroup::setSolverAllocators(const std::pair<SolverAllocatorFn, SolverAllocatorMapFn> &solvers)
{
  solver_allocators_ = solvers;
  if (solver_allocators_.first)
  {
    solver_instance_ = solver_allocators_.first(this);
    if (solver_instance_)
    {
      ik_joint_bijection_.clear();   
      const std::vector<std::string> &ik_jnames = solver_instance_->getJointNames(); 
      for (std::size_t i = 0 ; i < ik_jnames.size() ; ++i)
      {
        std::map<std::string, unsigned int>::const_iterator it = joint_variables_index_map_.find(ik_jnames[i]);
        if (it == joint_variables_index_map_.end())
        {
          logError("IK solver computes joint values for joint '%s' but group '%s' does not contain such a joint.", ik_jnames[i].c_str(), getName().c_str());
          solver_instance_.reset();
          return;
        }
        const robot_model::JointModel *jm = getJointModel(ik_jnames[i]);
        for (unsigned int k = 0 ; k < jm->getVariableCount() ; ++k)
          ik_joint_bijection_.push_back(it->second + k);
      }
    }
  }
}

bool robot_model::JointModelGroup::canSetStateFromIK(const std::string &tip) const
{
  const kinematics::KinematicsBaseConstPtr& solver = getSolverInstance();
  if (!solver)
    return false;
  const std::string &tip_frame = solver->getTipFrame();
  if (tip != tip_frame)
  {
    const LinkModel *lm = getParentModel()->getLinkModel(tip);
    if (!lm)
      return false;
    const LinkModel::AssociatedFixedTransformMap &fixed_links = lm->getAssociatedFixedTransforms();
    for (std::map<const LinkModel*, Eigen::Affine3d>::const_iterator it = fixed_links.begin() ; it != fixed_links.end() ; ++it)
      if (it->first->getName() == tip_frame)
        return true;
    return false;
  }
  else
    return true;
}

void robot_model::JointModelGroup::printGroupInfo(std::ostream &out) const
{
  out << "Group '" << name_ << "':" << std::endl;
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
  {
    const std::vector<std::string> &vn = joint_model_vector_[i]->getVariableNames();
    for (std::vector<std::string>::const_iterator it = vn.begin() ; it != vn.end() ; ++it)
    {
      out << "   " << *it << " [";
      std::pair<double, double> b;
      joint_model_vector_[i]->getVariableBounds(*it, b);
      if (b.first <= -std::numeric_limits<double>::max())
        out << "DBL_MIN";
      else
        out << b.first;
      out << ", ";
      if (b.second >= std::numeric_limits<double>::max())
        out << "DBL_MAX";
      else
        out << b.second;
      out << "]";
      if (joint_model_vector_[i]->getMimic())
        out << " *";
      out << std::endl;
    }
  }
}
