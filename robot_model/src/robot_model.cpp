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

/* Author: Ioan Sucan, E. Gil Jones */

#include <moveit/robot_model/robot_model.h>
#include <geometric_shapes/shape_operations.h>
#include <boost/math/constants/constants.hpp>
#include <moveit/profiler/profiler.h>
#include <algorithm>
#include <limits>
#include <queue>
#include <cmath>

/* ------------------------ RobotModel ------------------------ */

robot_model::RobotModel::RobotModel(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
                                    const boost::shared_ptr<const srdf::Model> &srdf_model)
{
  root_joint_ = NULL;
  urdf_ = urdf_model;
  srdf_ = srdf_model;
  if (urdf_model->getRoot())
  {
    const urdf::Link *root = urdf_model->getRoot().get();
    buildModel(urdf_model, srdf_model, root->name);
  }
  else
    logWarn("No root link found");
}

robot_model::RobotModel::RobotModel(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
                                    const boost::shared_ptr<const srdf::Model> &srdf_model,
                                    const std::string &root_link)
{
  root_joint_ = NULL;
  urdf_ = urdf_model;
  srdf_ = srdf_model;
  buildModel(urdf_model, srdf_model, root_link);
}

robot_model::RobotModel::~RobotModel()
{
  for (std::map<std::string, JointModelGroup*>::iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end() ; ++it)
    delete it->second;
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
    delete joint_model_vector_[i];
  for (std::size_t i = 0 ; i < link_model_vector_.size() ; ++i)
    delete link_model_vector_[i];
}

void robot_model::RobotModel::computeTreeStructure(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model, const std::string &root_link,
                                                   std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> >& parent_map,
                                                   std::map<const urdf::Link*, std::vector<const urdf::Link*> >& child_map)
{
  // construct a bidirectional graph that represents the URDF tree
  std::map<const urdf::Link*, std::map<const urdf::Link*, const urdf::Joint*> > graph;
  std::queue<const urdf::Link*> q;
  q.push(urdf_model->getRoot().get());
  while (!q.empty())
  {
    const urdf::Link *l = q.front();
    q.pop();
    for (unsigned int i = 0 ; i < l->child_links.size() ; ++i)
    {
      graph[l][l->child_links[i].get()] = graph[l->child_links[i].get()][l] = l->child_links[i]->parent_joint.get();
      q.push(l->child_links[i].get());
    }
  }

  // construct a tree from the graph such that the root is root_link
  class NewParentTree
  {
  public:
    NewParentTree(const std::map<const urdf::Link*, std::map<const urdf::Link*, const urdf::Joint*> > *graph) : graph_(graph)
    {
    }

    void constructTree(const urdf::Link *current, const urdf::Link *parent)
    {
      if (graph_->find(current) == graph_->end())
        return;
      const std::map<const urdf::Link*, const urdf::Joint*> &child = graph_->at(current);
      for (std::map<const urdf::Link*, const urdf::Joint*>::const_iterator it = child.begin() ; it != child.end() ; ++it)
        if (it->first != parent)
        {
          constructTree(it->first, current);
          parent_map_[it->first] = std::make_pair(current, it->second);
          child_map_[current].push_back(it->first);
        }
    }

    const std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> >& getParentMap() const
    {
      return parent_map_;
    }

    const std::map<const urdf::Link*, std::vector<const urdf::Link*> >& getChildMap() const
    {
      return child_map_;
    }

  private:
    const std::map<const urdf::Link*, std::map<const urdf::Link*, const urdf::Joint*> > *graph_;
    std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> > parent_map_;
    std::map<const urdf::Link*, std::vector<const urdf::Link*> > child_map_;
  };

  NewParentTree npt(&graph);
  const urdf::Link *root_link_ptr = urdf_model->getLink(root_link).get();
  if (root_link_ptr)
  {
    npt.constructTree(root_link_ptr, NULL);
    parent_map = npt.getParentMap();
    child_map = npt.getChildMap();
  }
  else
  {
    parent_map.clear();
    child_map.clear();
  }
}

void robot_model::RobotModel::buildModel(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
                                         const boost::shared_ptr<const srdf::Model> &srdf_model,
                                         const std::string &root_link)
{
  moveit::Profiler::ScopedStart prof_start;
  moveit::Profiler::ScopedBlock prof_block("RobotModel::buildModel");

  root_joint_ = NULL;
  model_name_ = urdf_model->getName();
  if (urdf_model->getRoot())
  {
    const urdf::Link *root_link_ptr = urdf_model->getLink(root_link).get();
    if (root_link_ptr)
    {
      model_frame_ = root_link;

      std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> > parent_map;
      std::map<const urdf::Link*, std::vector<const urdf::Link*> > child_map;
      computeTreeStructure(urdf_model, root_link, parent_map, child_map);

      root_joint_ = buildRecursive(NULL, root_link_ptr, parent_map, child_map, *srdf_model);
      root_link_ = link_model_map_[root_link];
      buildMimic(urdf_model);
      buildJointInfo();

      if (link_models_with_collision_geometry_vector_.empty())
        logWarn("No geometry is associated to any robot links");

      // build groups
      buildGroups(srdf_model);
      buildGroupStates(srdf_model);

      std::stringstream ss;
      printModelInfo(ss);
      logDebug("%s", ss.str().c_str());
    }
    else
      logError("Link '%s' (to be used as root) was not found in model '%s'. Cannot construct model", root_link.c_str(), model_name_.c_str());
  }
  else
    logWarn("No root link found");
}

void robot_model::RobotModel::buildJointInfo()
{
  // construct additional maps for easy access by name
  variable_count_ = 0;
  std::vector<JointModel*> later;
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
  {
    const std::vector<std::string> &name_order = joint_model_vector_[i]->getVariableNames();
    for (std::size_t j = 0 ; j < name_order.size() ; ++j)
      joint_model_vector_[i]->getVariableBounds(name_order[j], variable_bounds_[name_order[j]]);
    if (joint_model_vector_[i]->mimic_ == NULL)
    {
      // compute index map
      if (name_order.size() > 0)
      {
        for (std::size_t j = 0; j < name_order.size(); ++j)
        {
          joint_variables_index_map_[name_order[j]] = variable_count_ + j;
          active_variable_names_.push_back(name_order[j]);
        }
        joint_variables_index_map_[joint_model_vector_[i]->getName()] = variable_count_;

        // compute variable count
        variable_count_ += joint_model_vector_[i]->getVariableCount();
      }
    }
    else
      later.push_back(joint_model_vector_[i]);
  }

  for (std::size_t i = 0 ; i < later.size() ; ++i)
  {
    const std::vector<std::string>& name_order = later[i]->getVariableNames();
    const std::vector<std::string>& mim_name_order = later[i]->mimic_->getVariableNames();
    for (std::size_t j = 0; j < name_order.size(); ++j)
      joint_variables_index_map_[name_order[j]] = joint_variables_index_map_[mim_name_order[j]];
    joint_variables_index_map_[later[i]->getName()] = joint_variables_index_map_[later[i]->mimic_->getName()];
  }

  for (std::size_t i = 0 ; i < link_model_vector_.size() ; ++i)
  {
    LinkModelToAffine3dMap associated_transforms;
    computeFixedTransforms(link_model_vector_[i], Eigen::Affine3d::Identity(), associated_transforms);
    if (associated_transforms.size() > 1)
    {
      for (LinkModelToAffine3dMap::iterator it = associated_transforms.begin() ; it != associated_transforms.end() ; ++it)
      {
        it->first->associated_fixed_transforms_[link_model_vector_[i]] = it->second.inverse();
        link_model_vector_[i]->associated_fixed_transforms_[it->first] = it->second;
      }
    }
  }
}

void robot_model::RobotModel::buildGroupStates(const boost::shared_ptr<const srdf::Model> &srdf_model)
{
  // copy the default states to the groups
  const std::vector<srdf::Model::GroupState> &ds = srdf_model->getGroupStates();
  for (std::size_t i = 0 ; i < ds.size() ; ++i)
  {
    std::map<std::string, JointModelGroup*>::const_iterator it = joint_model_group_map_.find(ds[i].group_);
    if (it != joint_model_group_map_.end())
      for (std::map<std::string, std::vector<double> >::const_iterator jt = ds[i].joint_values_.begin() ; jt != ds[i].joint_values_.end() ; ++jt)
      {
        const JointModel* jm = it->second->getJointModel(jt->first);
        if (jm)
        {
          const std::vector<std::string> &vn = jm->getVariableNames();
          if (vn.size() == jt->second.size())
            for (std::size_t j = 0 ; j < vn.size() ; ++j)
              it->second->default_states_[ds[i].name_][vn[j]] = jt->second[j];
          else
            logError("The model for joint '%s' requires %d variable values, but only %d variable values were supplied in default state '%s' for group '%s'",
                     jt->first.c_str(), (int)vn.size(), (int)jt->second.size(), ds[i].name_.c_str(), it->first.c_str());
        }
        else
          logError("Group state '%s' specifies value for joint '%s', but that joint is not part of group '%s'", ds[i].name_.c_str(),
                   jt->first.c_str(), it->first.c_str());
      }
    else
      logError("Group state '%s' specified for group '%s', but that group does not exist", ds[i].name_.c_str(), ds[i].group_.c_str());
  }
}

void robot_model::RobotModel::buildMimic(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model)
{
  // compute mimic joints
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
  {
    const urdf::Joint *jm = urdf_model->getJoint(joint_model_vector_[i]->getName()).get();
    if (jm)
      if (jm->mimic)
      {
        joint_model_vector_[i]->mimic_offset_ = jm->mimic->offset;
        joint_model_vector_[i]->mimic_factor_ = jm->mimic->multiplier;
        std::map<std::string, JointModel*>::const_iterator jit = joint_model_map_.find(jm->mimic->joint_name);
        if (jit != joint_model_map_.end())
        {
          if (joint_model_vector_[i]->getVariableCount() == jit->second->getVariableCount())
            joint_model_vector_[i]->mimic_ = jit->second;
          else
            logError("Join '%s' cannot mimic joint '%s' because they have different number of DOF",
                     joint_model_vector_[i]->getName().c_str(), jm->mimic->joint_name.c_str());
        }
        else
          logError("Joint '%s' cannot mimic unknown joint '%s'", joint_model_vector_[i]->getName().c_str(), jm->mimic->joint_name.c_str());
      }
  }
  // in case we have a joint that mimics a joint that already mimics another joint, we can simplify things:
  bool change = true;
  while (change)
  {
    change = false;
    for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
      if (joint_model_vector_[i]->mimic_)
        if (joint_model_vector_[i]->mimic_->mimic_)
        {
          joint_model_vector_[i]->mimic_ = joint_model_vector_[i]->mimic_->mimic_;
          joint_model_vector_[i]->mimic_offset_ += joint_model_vector_[i]->mimic_factor_ * joint_model_vector_[i]->mimic_->mimic_offset_;
          joint_model_vector_[i]->mimic_factor_ *= joint_model_vector_[i]->mimic_->mimic_factor_;
          change = true;
        }
  }
  // build mimic requests
  for (std::size_t i = 0 ; i < joint_model_vector_.size() ; ++i)
    if (joint_model_vector_[i]->mimic_ == joint_model_vector_[i])
    {
      joint_model_vector_[i]->mimic_ = NULL;
      logError("Joint '%s' mimicks itself. This is not allowed.", joint_model_vector_[i]->getName().c_str());
    }
    else
      if (joint_model_vector_[i]->mimic_)
        joint_model_vector_[i]->mimic_->mimic_requests_.push_back(joint_model_vector_[i]);
}

bool robot_model::RobotModel::hasEndEffector(const std::string& eef) const
{
  return end_effectors_.find(eef) != end_effectors_.end();
}

const robot_model::JointModelGroup* robot_model::RobotModel::getEndEffector(const std::string& name) const
{
  std::map<std::string, JointModelGroup*>::const_iterator it = end_effectors_.find(name);
  if (it == end_effectors_.end())
  {
    it = joint_model_group_map_.find(name);
    if (it != joint_model_group_map_.end() && it->second->isEndEffector())
      return it->second;
    logError("End-effector '%s' not found in model %s", name.c_str(), model_name_.c_str());
    return NULL;
  }
  return it->second;
}

robot_model::JointModelGroup* robot_model::RobotModel::getEndEffector(const std::string& name)
{
  std::map<std::string, JointModelGroup*>::const_iterator it = end_effectors_.find(name);
  if (it == end_effectors_.end())
  {
    it = joint_model_group_map_.find(name);
    if (it != joint_model_group_map_.end() && it->second->isEndEffector())
      return it->second;
    logError("End-effector '%s' not found in model %s", name.c_str(), model_name_.c_str());
    return NULL;
  }
  return it->second;
}

bool robot_model::RobotModel::hasJointModelGroup(const std::string &name) const
{
  return joint_model_group_map_.find(name) != joint_model_group_map_.end();
}

const robot_model::JointModelGroup* robot_model::RobotModel::getJointModelGroup(const std::string& name) const
{
  std::map<std::string, JointModelGroup*>::const_iterator it = joint_model_group_map_.find(name);
  if (it == joint_model_group_map_.end())
  {
    logError("Group '%s' not found in model %s", name.c_str(), model_name_.c_str());
    return NULL;
  }
  return it->second;
}

robot_model::JointModelGroup* robot_model::RobotModel::getJointModelGroup(const std::string& name)
{
  std::map<std::string, JointModelGroup*>::const_iterator it = joint_model_group_map_.find(name);
  if (it == joint_model_group_map_.end())
  {
    logError("Group '%s' not found in model %s", name.c_str(), model_name_.c_str());
    return NULL;
  }
  return it->second;
}

void robot_model::RobotModel::buildGroups(const boost::shared_ptr<const srdf::Model> &srdf_model)
{
  const std::vector<srdf::Model::Group>& group_configs = srdf_model->getGroups();

  //the only thing tricky is dealing with subgroups
  std::vector<bool> processed(group_configs.size(), false);

  bool added = true;
  while (added)
  {
    added = false;

    //going to make passes until we can't do anything else
    for(unsigned int i = 0 ; i < group_configs.size() ; ++i)
      if (!processed[i])
      {
        //if we haven't processed, check and see if the dependencies are met yet
        bool all_subgroups_added = true;
        for(unsigned int j = 0; j < group_configs[i].subgroups_.size(); ++j)
          if (joint_model_group_map_.find(group_configs[i].subgroups_[j]) == joint_model_group_map_.end())
          {
            all_subgroups_added = false;
            break;
          }
        if (all_subgroups_added)
        {
          added = true;
          processed[i] = true;
          if (!addJointModelGroup(group_configs[i]))
            logWarn("Failed to add group '%s'", group_configs[i].name_.c_str());
        }
      }
  }

  for (unsigned int i = 0 ; i < processed.size() ; ++i)
    if (!processed[i])
      logWarn("Could not process group '%s' due to unmet subgroup dependencies", group_configs[i].name_.c_str());

  buildGroupsInfo_Subgroups(srdf_model);
  buildGroupsInfo_EndEffectors(srdf_model);
}

void robot_model::RobotModel::buildGroupsInfo_Subgroups(const boost::shared_ptr<const srdf::Model> &srdf_model)
{
  // compute subgroups
  for (std::map<std::string, JointModelGroup*>::const_iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end(); ++it)
  {
    JointModelGroup *jmg = it->second;
    jmg->subgroup_names_.clear();
    std::set<const JointModel*> joints(jmg->getJointModels().begin(), jmg->getJointModels().end());
    for (std::map<std::string, JointModelGroup*>::const_iterator jt = joint_model_group_map_.begin() ; jt != joint_model_group_map_.end(); ++jt)
      if (jt->first != it->first)
      {
        bool ok = true;
        JointModelGroup *sub_jmg = jt->second;
        const std::vector<const JointModel*> &sub_joints = sub_jmg->getJointModels();
        for (std::size_t k = 0 ; k < sub_joints.size() ; ++k)
          if (joints.find(sub_joints[k]) == joints.end())
          {
            ok = false;
            break;
          }
        if (ok)
          jmg->subgroup_names_.push_back(sub_jmg->getName());
      }
  }
}

void robot_model::RobotModel::buildGroupsInfo_EndEffectors(const boost::shared_ptr<const srdf::Model> &srdf_model)
{
  // set the end-effector flags
  const std::vector<srdf::Model::EndEffector> &eefs = srdf_model->getEndEffectors();
  for (std::map<std::string, JointModelGroup*>::const_iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end(); ++it)
  {
    // check if this group is a known end effector
    for (std::size_t k = 0 ; k < eefs.size() ; ++k)
      if (eefs[k].component_group_ == it->first)
      {
        // if it is, mark it as such
        it->second->is_end_effector_ = true;
        it->second->end_effector_name_ = eefs[k].name_;
        end_effectors_[eefs[k].name_] = it->second;

        JointModelGroup *eef_parent_group = NULL;

        // if a parent group is specified in SRDF, try to use it
        if (!eefs[k].parent_group_.empty())
        {
          std::map<std::string, JointModelGroup*>::const_iterator jt = joint_model_group_map_.find(eefs[k].parent_group_);
          if (jt != joint_model_group_map_.end())
          {
            if (jt->second->hasLinkModel(eefs[k].parent_link_))
            {
              if (jt->second != it->second)
                eef_parent_group = jt->second;
              else
                logError("Group '%s' for end-effector '%s' cannot be its own parent", eefs[k].parent_group_.c_str(), eefs[k].name_.c_str());
            }
            else
              logError("Group '%s' was specified as parent group for end-effector '%s' but it does not include the parent link '%s'",
                       eefs[k].parent_group_.c_str(), eefs[k].name_.c_str(), eefs[k].parent_link_.c_str());
          }
          else
            logError("Group name '%s' not found (specified as parent group for end-effector '%s')",
                     eefs[k].parent_group_.c_str(), eefs[k].name_.c_str());
        }

        if (eef_parent_group == NULL)
        {
          // check to see if there are groups that contain the parent link of this end effector.
          // record this information if found
          std::vector<JointModelGroup*> possible_parent_groups;
          for (std::map<std::string, JointModelGroup*>::const_iterator jt = joint_model_group_map_.begin() ; jt != joint_model_group_map_.end(); ++jt)
            if (jt->first != it->first)
            {
              if (jt->second->hasLinkModel(eefs[k].parent_link_))
                possible_parent_groups.push_back(jt->second);
            }
          if (!possible_parent_groups.empty())
          {
            // if there are multiple options for the group that contains this end-effector,
            // we pick the group with fewest joints.
            std::size_t best = 0;
            for (std::size_t g = 1 ; g < possible_parent_groups.size() ; ++g)
              if (possible_parent_groups[g]->getJointModels().size() < possible_parent_groups[best]->getJointModels().size())
                best = g;
            eef_parent_group = possible_parent_groups[best];
          }
        }
        if (eef_parent_group)
        {
          eef_parent_group->attached_end_effector_names_.push_back(eefs[k].name_);
          it->second->end_effector_parent_.first = eef_parent_group->getName();
        }
        else
          logWarn("Could not identify parent group for end-effector '%s'", eefs[k].name_.c_str());
        it->second->end_effector_parent_.second = eefs[k].parent_link_;
        break;
      }
  }
}

bool robot_model::RobotModel::addJointModelGroup(const srdf::Model::Group& gc)
{
  if (joint_model_group_map_.find(gc.name_) != joint_model_group_map_.end())
  {
    logWarn("A group named '%s' already exists. Not adding.",  gc.name_.c_str());
    return false;
  }

  std::set<const JointModel*> jset;

  // add joints from chains
  for (std::size_t i = 0 ; i < gc.chains_.size() ; ++i)
  {
    const LinkModel* base_link = getLinkModel(gc.chains_[i].first);
    const LinkModel* tip_link = getLinkModel(gc.chains_[i].second);
    if (base_link && tip_link)
    {
      // go from tip, up the chain, until we hit the root or we find the base_link
      const LinkModel* lm = tip_link;
      std::vector<const JointModel*> cj;
      while (lm)
      {
        if (lm == base_link)
          break;
        cj.push_back(lm->getParentJointModel());
        lm = lm->getParentJointModel()->getParentLinkModel();
      }
      // if we did not find the base_link, we could have a chain like e.g.,
      // from one end-effector to another end-effector, so the root is in between
      if (lm != base_link)
      {
        // we go up the chain from the base this time, and see where we intersect the other chain
        lm = base_link;
        std::size_t index = 0;
        std::vector<const JointModel*> cj2;
        while (lm)
        {
          for (std::size_t j = 0 ; j < cj.size() ; ++j)
            if (cj[j] == lm->getParentJointModel())
            {
              index = j + 1;
              break;
            }
          if (index > 0)
            break;
          cj2.push_back(lm->getParentJointModel());
          lm = lm->getParentJointModel()->getParentLinkModel();
        }
        if (index > 0)
        {
          jset.insert(cj.begin(), cj.begin() + index);
          jset.insert(cj2.begin(), cj2.end());
        }
      }
      else
        // if we have a simple chain, just add the joints
        jset.insert(cj.begin(), cj.end());
    }
  }

  // add joints
  for (std::size_t i = 0 ; i < gc.joints_.size() ; ++i)
  {
    const JointModel *j = getJointModel(gc.joints_[i]);
    if (j)
      jset.insert(j);
  }

  // add joints that are parents of included links
  for (std::size_t i = 0 ; i < gc.links_.size() ; ++i)
  {
    const LinkModel *l = getLinkModel(gc.links_[i]);
    if (l)
      jset.insert(l->getParentJointModel());
  }

  // add joints from subgroups
  for (std::size_t i = 0 ; i < gc.subgroups_.size() ; ++i)
  {
    const JointModelGroup *sg = getJointModelGroup(gc.subgroups_[i]);
    if (sg)
    {
      //active joints
      const std::vector<const JointModel*> &js = sg->getJointModels();
      for (std::size_t j = 0 ; j < js.size() ; ++j)
        jset.insert(js[j]);

      //fixed joints
      const std::vector<const JointModel*> &fs = sg->getFixedJointModels();
      for (std::size_t j = 0 ; j < fs.size() ; ++j)
        jset.insert(fs[j]);

      //mimic joints
      const std::vector<const JointModel*> &ms = sg->getMimicJointModels();
      for (std::size_t j = 0 ; j < ms.size() ; ++j)
        jset.insert(ms[j]);
    }
  }

  if (jset.empty())
  {
    logWarn("Group '%s' must have at least one valid joint", gc.name_.c_str());
    return false;
  }

  std::vector<const JointModel*> joints;
  for (std::set<const JointModel*>::iterator it = jset.begin() ; it != jset.end() ; ++it)
    joints.push_back(*it);

  JointModelGroup *jmg = new JointModelGroup(gc.name_, joints, this);
  joint_model_group_map_[gc.name_] = jmg;
  joint_model_group_config_map_[gc.name_] = gc;
  joint_model_group_names_.push_back(gc.name_);

  // if the group is defined as a single chain, then we mark is as a chain aleady
  // (this is for the case where the chain does not consist of consecutive joints and would not be detected as a chain later)
  if (gc.chains_.size() == 1 && gc.joints_.empty() && gc.links_.empty() && gc.subgroups_.empty())
    jmg->is_chain_ = true;

  return true;
}

robot_model::JointModel* robot_model::RobotModel::buildRecursive(LinkModel *parent, const urdf::Link *link,
                                                                 const std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> > &parent_map,
                                                                 const std::map<const urdf::Link*, std::vector<const urdf::Link*> > &child_map,
                                                                 const srdf::Model &srdf_model)
{
  std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> >::const_iterator pmi = parent_map.find(link);
  JointModel *joint = (pmi != parent_map.end()) ? constructJointModel(pmi->second.second, link, srdf_model) : constructJointModel(NULL, link, srdf_model);
  if (joint == NULL)
    return NULL;
  joint_model_map_[joint->name_] = joint;
  joint->tree_index_ = joint_model_vector_.size();
  joint_model_vector_.push_back(joint);
  joint_model_vector_const_.push_back(joint);
  joint_model_names_vector_.push_back(joint->getName());
  if (joint->getType() == JointModel::REVOLUTE && static_cast<const RevoluteJointModel*>(joint)->isContinuous())
    continuous_joint_model_vector_const_.push_back(joint);
  joint->parent_link_model_ = parent;
  joint->child_link_model_ = constructLinkModel(link, parent_map);
  link_model_map_[joint->child_link_model_->name_] = joint->child_link_model_;
  joint->child_link_model_->tree_index_ = link_model_vector_.size();
  link_model_vector_.push_back(joint->child_link_model_);
  link_model_vector_const_.push_back(joint->child_link_model_);
  link_model_names_vector_.push_back(link_model_vector_.back()->getName());
  if (joint->child_link_model_->shape_)
  {
    link_models_with_collision_geometry_vector_.push_back(joint->child_link_model_);
    link_model_names_with_collision_geometry_vector_.push_back(link_models_with_collision_geometry_vector_.back()->getName());
  }
  joint->child_link_model_->parent_joint_model_ = joint;
  std::map<const urdf::Link*, std::vector<const urdf::Link*> >::const_iterator cmi = child_map.find(link);
  if (cmi != child_map.end())
    for (unsigned int i = 0 ; i < cmi->second.size() ; ++i)
    {
      JointModel* jm = buildRecursive(joint->child_link_model_, cmi->second[i], parent_map, child_map, srdf_model);
      if (jm)
        joint->child_link_model_->child_joint_models_.push_back(jm);
    }
  return joint;
}

robot_model::JointModel* robot_model::RobotModel::constructJointModel(const urdf::Joint *urdf_joint, const urdf::Link *child_link,
                                                                      const srdf::Model &srdf_model)
{
  JointModel* result = NULL;

  // must be the root link transform
  if (urdf_joint)
  {
    switch (urdf_joint->type)
    {
    case urdf::Joint::REVOLUTE:
      {
        RevoluteJointModel *j = new RevoluteJointModel(urdf_joint->name);
        if (urdf_joint->safety)
        {
          j->variable_bounds_[0] = std::make_pair(urdf_joint->safety->soft_lower_limit, urdf_joint->safety->soft_upper_limit);
          if (urdf_joint->limits)
          {
            if (urdf_joint->limits->lower > j->variable_bounds_[0].first)
              j->variable_bounds_[0].first = urdf_joint->limits->lower;
            if (urdf_joint->limits->upper < j->variable_bounds_[0].second)
              j->variable_bounds_[0].second = urdf_joint->limits->upper;
          }
        }
        else
        {
          if (urdf_joint->limits)
            j->variable_bounds_[0] = std::make_pair(urdf_joint->limits->lower, urdf_joint->limits->upper);
          else
            j->variable_bounds_[0] = std::make_pair(0.0, 0.0);
        }
        if (urdf_joint->limits)
          j->max_velocity_ = fabs(urdf_joint->limits->velocity);
        j->continuous_ = false;
        j->axis_ = Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
        result = j;
      }
      break;
    case urdf::Joint::CONTINUOUS:
      {
        RevoluteJointModel *j = new RevoluteJointModel(urdf_joint->name);
        j->continuous_ = true;
        j->variable_bounds_[0] = std::make_pair(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>());
        if (urdf_joint->limits)
          j->max_velocity_ = fabs(urdf_joint->limits->velocity);
        j->axis_ = Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
        result = j;
      }
      break;
    case urdf::Joint::PRISMATIC:
      {
        PrismaticJointModel *j = new PrismaticJointModel(urdf_joint->name);
        if(urdf_joint->safety)
        {
          j->variable_bounds_[0] = std::make_pair(urdf_joint->safety->soft_lower_limit, urdf_joint->safety->soft_upper_limit);
          if (urdf_joint->limits)
          {
            if (urdf_joint->limits->lower > j->variable_bounds_[0].first)
              j->variable_bounds_[0].first = urdf_joint->limits->lower;
            if (urdf_joint->limits->upper < j->variable_bounds_[0].second)
              j->variable_bounds_[0].second = urdf_joint->limits->upper;
          }
        }
        else
        {
          if (urdf_joint->limits)
            j->variable_bounds_[0] = std::make_pair(urdf_joint->limits->lower, urdf_joint->limits->upper);
          else
            j->variable_bounds_[0] = std::make_pair(0.0, 0.0);
        }
        if (urdf_joint->limits)
          j->max_velocity_ = fabs(urdf_joint->limits->velocity);
        j->axis_ = Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
        result = j;
      }
      break;
    case urdf::Joint::FLOATING:
      result = new FloatingJointModel(urdf_joint->name);
      break;
    case urdf::Joint::PLANAR:
      result = new PlanarJointModel(urdf_joint->name);
      break;
    case urdf::Joint::FIXED:
      result = new FixedJointModel(urdf_joint->name);
      break;
    default:
      logError("Unknown joint type: %d", (int)urdf_joint->type);
      break;
    }
  }
  else
  {
    const std::vector<srdf::Model::VirtualJoint> &vjoints = srdf_model.getVirtualJoints();
    for (std::size_t i = 0 ; i < vjoints.size() ; ++i)
      if (vjoints[i].child_link_ == child_link->name)
      {
        if (vjoints[i].type_ == "fixed")
          result = new FixedJointModel(vjoints[i].name_);
        else if (vjoints[i].type_ == "planar")
          result = new PlanarJointModel(vjoints[i].name_);
        else if (vjoints[i].type_ == "floating")
          result = new FloatingJointModel(vjoints[i].name_);
        if (result)
        {
          // for fixed frames we still use the robot root link
          if (vjoints[i].type_ != "fixed")
            model_frame_ = vjoints[i].parent_frame_;
          break;
        }
      }
    if (!result)
    {
      logInform("No root joint specified. Assuming fixed joint");
      result = new FixedJointModel("ASSUMED_FIXED_ROOT_JOINT");
    }
  }

  if (result)
  {
    for (std::size_t i = 0 ; i < result->variable_names_.size() ; ++i)
      result->variable_index_[result->variable_names_[i]] = i;
    result->setDistanceFactor(result->getStateSpaceDimension());

    const std::vector<srdf::Model::PassiveJoint> &pjoints = srdf_model.getPassiveJoints();
    for (std::size_t i = 0 ; i < pjoints.size() ; ++i)
    {
      if (result->getName() == pjoints[i].name_)
      {
        result->passive_ = true;
        break;
      }
    }
    result->computeDefaultVariableLimits();
  }
  return result;
}

namespace robot_model
{
static inline Eigen::Affine3d urdfPose2Affine3d(const urdf::Pose &pose)
{
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  Eigen::Affine3d af(Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z)*q.toRotationMatrix());
  return af;
}

}

robot_model::LinkModel* robot_model::RobotModel::constructLinkModel(const urdf::Link *urdf_link, const std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> > &parent_map)
{
  LinkModel *result = new LinkModel();
  result->name_ = urdf_link->name;
  if (urdf_link->collision && urdf_link->collision->geometry)
  {
    result->collision_origin_transform_ = urdfPose2Affine3d(urdf_link->collision->origin);
    result->shape_ = constructShape(urdf_link->collision->geometry.get());
    if (result->shape_)
    {
      if (shapes::constructMsgFromShape(result->shape_.get(), result->shape_msg_))
        result->shape_extents_ = shapes::computeShapeExtents(result->shape_msg_);
      else
        result->shape_extents_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
  }
  else if (urdf_link->visual && urdf_link->visual->geometry)
  {
    result->collision_origin_transform_ = urdfPose2Affine3d(urdf_link->visual->origin);
    result->shape_ = constructShape(urdf_link->visual->geometry.get());
    if (result->shape_)
    {
      if (shapes::constructMsgFromShape(result->shape_.get(), result->shape_msg_))
        result->shape_extents_ = shapes::computeShapeExtents(result->shape_msg_);
      else
        result->shape_extents_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
  }
  else
  {
    result->collision_origin_transform_.setIdentity();
    result->shape_.reset();
    result->shape_extents_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  }

  // figure out visual mesh (try visual urdf tag first, collision tag otherwise
  if (urdf_link->visual && urdf_link->visual->geometry)
  {
    if (urdf_link->visual->geometry->type == urdf::Geometry::MESH)
    {
      const urdf::Mesh *mesh = static_cast<const urdf::Mesh*>(urdf_link->visual->geometry.get());
      if (!mesh->filename.empty())
      {
        result->visual_mesh_filename_ = mesh->filename;
        result->visual_mesh_scale_ = Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z);
      }
    }
  }
  else
    if (urdf_link->collision && urdf_link->collision->geometry)
    {
      if (urdf_link->collision->geometry->type == urdf::Geometry::MESH)
      {
        const urdf::Mesh *mesh = static_cast<const urdf::Mesh*>(urdf_link->collision->geometry.get());
        if (!mesh->filename.empty())
        {
          result->visual_mesh_filename_ = mesh->filename;
          result->visual_mesh_scale_ = Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        }
      }
    }
  
  std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> >::const_iterator pmi = parent_map.find(urdf_link);
  if (pmi != parent_map.end())
  {
    // if we consider the kinematic tree as in the URDF, then we just take the transform as is
    if (urdf_link->parent_joint.get() == pmi->second.second)
    {
      result->joint_origin_transform_ = urdfPose2Affine3d(urdf_link->parent_joint->parent_to_joint_origin_transform);
      result->reverse_joint_ = false;
    }
    else
    {
      // if not, it means we are viewing the transform in reverse, so we take the inverse:
      result->joint_origin_transform_ = urdfPose2Affine3d(pmi->second.second->parent_to_joint_origin_transform);
      result->reverse_joint_ = true;
    }
  }
  else
  {
    result->joint_origin_transform_.setIdentity();
    result->reverse_joint_ = false;
  }

  return result;
}

shapes::ShapePtr robot_model::RobotModel::constructShape(const urdf::Geometry *geom)
{ 
  moveit::Profiler::ScopedBlock prof_block("RobotModel::constructShape");

  shapes::Shape *result = NULL;
  switch (geom->type)
  {
  case urdf::Geometry::SPHERE:
    result = new shapes::Sphere(static_cast<const urdf::Sphere*>(geom)->radius);
    break;
  case urdf::Geometry::BOX:
    {
      urdf::Vector3 dim = static_cast<const urdf::Box*>(geom)->dim;
      result = new shapes::Box(dim.x, dim.y, dim.z);
    }
    break;
  case urdf::Geometry::CYLINDER:
    result = new shapes::Cylinder(static_cast<const urdf::Cylinder*>(geom)->radius,
                                  static_cast<const urdf::Cylinder*>(geom)->length);
    break;
  case urdf::Geometry::MESH:
    {
      const urdf::Mesh *mesh = static_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty())
      {
        Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        shapes::Mesh *m = shapes::createMeshFromResource(mesh->filename, scale);
        // merge vertices up to 0.1 mm apart
        if (m)
          m->mergeVertices(1e-4);
        result = m;
      }
    }
    break;
  default:
    logError("Unknown geometry type: %d", (int)geom->type);
    break;
  }
  
  return shapes::ShapePtr(result);
}

const std::string& robot_model::RobotModel::getRootJointName() const
{
  static const std::string empty;
  return getRoot() ? getRoot()->getName() : empty;
}

const std::string& robot_model::RobotModel::getRootLinkName() const
{
  static const std::string empty;
  return getRootLink() ? getRootLink()->getName() : empty;
}

bool robot_model::RobotModel::hasJointModel(const std::string &name) const
{
  return joint_model_map_.find(name) != joint_model_map_.end();
}

bool robot_model::RobotModel::hasLinkModel(const std::string &name) const
{
  return link_model_map_.find(name) != link_model_map_.end();
}

const robot_model::JointModel* robot_model::RobotModel::getJointModel(const std::string &name) const
{
  std::map<std::string, JointModel*>::const_iterator it = joint_model_map_.find(name);
  if (it == joint_model_map_.end())
  {
    logError("Joint '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
    return NULL;
  }
  else
    return it->second;
}

const robot_model::LinkModel* robot_model::RobotModel::getLinkModel(const std::string &name) const
{
  std::map<std::string, LinkModel*>::const_iterator it = link_model_map_.find(name);
  if (it == link_model_map_.end())
  {
    logError("Link '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

void robot_model::RobotModel::getChildLinkModels(const LinkModel *parent, std::vector<const LinkModel*> &links) const
{
  links.clear();
  links.push_back(parent);
  std::queue<const LinkModel*> q;
  std::set<const LinkModel*> seen;
  q.push(parent);
  while (!q.empty())
  {
    const LinkModel* t = q.front();
    q.pop();
    if (seen.insert(t).second)
      for (std::size_t i = 0 ; i < t->child_joint_models_.size() ; ++i)
      {
        links.push_back(t->child_joint_models_[i]->child_link_model_);
        q.push(t->child_joint_models_[i]->child_link_model_);
        for (std::size_t j = 0 ; j < t->child_joint_models_[i]->mimic_requests_.size() ; ++j)
        {
          links.push_back(t->child_joint_models_[i]->mimic_requests_[j]->child_link_model_);
          q.push(t->child_joint_models_[i]->mimic_requests_[j]->child_link_model_);
        }
      }
  }
}

void robot_model::RobotModel::getChildLinkModels(const JointModel *parent, std::vector<const LinkModel*> &links) const
{
  getChildLinkModels(parent->child_link_model_, links);
}

void robot_model::RobotModel::getChildJointModels(const LinkModel *parent, std::vector<const JointModel*> &joints) const
{
  joints.clear();
  std::queue<const LinkModel*> q;
  std::set<const LinkModel*> seen;
  q.push(parent);

  while (!q.empty())
  {
    const LinkModel* t = q.front();
    q.pop();
    if (seen.insert(t).second)
      for (unsigned int i = 0 ; i < t->child_joint_models_.size() ; ++i)
      {
        joints.push_back(t->child_joint_models_[i]);
        q.push(t->child_joint_models_[i]->child_link_model_);
        for (std::size_t j = 0 ; j < t->child_joint_models_[i]->mimic_requests_.size() ; ++j)
        {
          joints.push_back(t->child_joint_models_[i]->mimic_requests_[j]);
          q.push(t->child_joint_models_[i]->mimic_requests_[j]->child_link_model_);
        }
      }
  }
}

void robot_model::RobotModel::getChildJointModels(const JointModel *parent, std::vector<const JointModel*> &joints) const
{
  getChildJointModels(parent->child_link_model_, joints);
  joints.insert(joints.begin(), parent);
}

std::vector<std::string> robot_model::RobotModel::getChildLinkModelNames(const LinkModel *parent) const
{
  std::vector<const LinkModel*> links;
  getChildLinkModels(parent, links);
  std::vector<std::string> ret_vec(links.size());
  for (std::size_t i = 0; i < links.size(); ++i)
    ret_vec[i] = links[i]->getName();
  return ret_vec;
}

std::vector<std::string> robot_model::RobotModel::getChildLinkModelNames(const JointModel *parent) const
{
  std::vector<const LinkModel*> links;
  getChildLinkModels(parent, links);
  std::vector<std::string> ret_vec(links.size());
  for(unsigned int i = 0; i < links.size(); ++i)
    ret_vec[i] = links[i]->getName();
  return ret_vec;
}

std::vector<std::string> robot_model::RobotModel::getChildJointModelNames(const LinkModel *parent) const
{
  std::vector<const JointModel*> joints;
  getChildJointModels(parent, joints);
  std::vector<std::string> ret_vec(joints.size());
  for(unsigned int i = 0 ; i < joints.size() ; ++i)
    ret_vec[i] = joints[i]->getName();
  return ret_vec;
}

std::vector<std::string> robot_model::RobotModel::getChildJointModelNames(const JointModel *parent) const
{
  std::vector<const JointModel*> joints;
  getChildJointModels(parent, joints);
  std::vector<std::string> ret_vec(joints.size());
  for(unsigned int i = 0 ; i < joints.size(); ++i)
    ret_vec[i] = joints[i]->getName();
  return ret_vec;
}


void robot_model::RobotModel::getVariableRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values) const
{
  for (std::size_t i = 0  ; i < joint_model_vector_.size() ; ++i)
    if (joint_model_vector_[i]->mimic_ == NULL)
      joint_model_vector_[i]->getVariableRandomValues(rng, values);
}

void robot_model::RobotModel::getVariableRandomValues(random_numbers::RandomNumberGenerator &rng, std::map<std::string, double> &values) const
{
  for (std::size_t i = 0  ; i < joint_model_vector_.size() ; ++i)
    if (joint_model_vector_[i]->mimic_ == NULL)
      joint_model_vector_[i]->getVariableRandomValues(rng, values);
}

void robot_model::RobotModel::getVariableDefaultValues(std::vector<double> &values) const
{
  for (std::size_t i = 0  ; i < joint_model_vector_.size() ; ++i)
    if (joint_model_vector_[i]->mimic_ == NULL)
      joint_model_vector_[i]->getVariableDefaultValues(values);
}

void robot_model::RobotModel::getVariableDefaultValues(std::map<std::string, double> &values) const
{
  for (std::size_t i = 0  ; i < joint_model_vector_.size() ; ++i)
    if (joint_model_vector_[i]->mimic_ == NULL)
      joint_model_vector_[i]->getVariableDefaultValues(values);
}

void robot_model::RobotModel::setKinematicsAllocators(const std::map<std::string, SolverAllocatorFn> &allocators)
{
  for (std::map<std::string, JointModelGroup*>::const_iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end() ; ++it)
  {
    JointModelGroup *jmg = it->second;
    std::pair<SolverAllocatorFn, SolverAllocatorMapFn> result;
    std::map<std::string, SolverAllocatorFn>::const_iterator jt = allocators.find(jmg->getName());
    if (jt == allocators.end())
    {
      // if an kinematics allocator is NOT available for this group, we try to see if we can use subgroups for IK
      std::set<const JointModel*> joints;
      joints.insert(jmg->getJointModels().begin(), jmg->getJointModels().end());

      std::vector<const JointModelGroup*> subs;

      // go through the groups that we know have IK allocators and see if they are included in the group that does not; if so, put that group in sub
      for (std::map<std::string, SolverAllocatorFn>::const_iterator kt = allocators.begin() ; kt != allocators.end() ; ++kt)
      {
        const JointModelGroup *sub = getJointModelGroup(kt->first);
        if (!sub)
        {
          subs.clear();
          break;
        }
        std::set<const JointModel*> sub_joints;
        sub_joints.insert(sub->getJointModels().begin(), sub->getJointModels().end());

        if (std::includes(joints.begin(), joints.end(), sub_joints.begin(), sub_joints.end()))
        {
          std::set<const JointModel*> resultj;
          std::set_difference(joints.begin(), joints.end(), sub_joints.begin(), sub_joints.end(),
                              std::inserter(resultj, resultj.end()));
          subs.push_back(sub);
          joints.swap(resultj);
        }
      }

      // if we found subgroups, pass that information to the planning group
      if (!subs.empty())
      {
        std::stringstream ss;
        for (std::size_t i = 0 ; i < subs.size() ; ++i)
        {
          ss << subs[i]->getName() << " ";
          result.second[subs[i]] = allocators.find(subs[i]->getName())->second;
        }
        logDebug("Added sub-group IK allocators for group '%s': [ %s]", jmg->getName().c_str(), ss.str().c_str());
      }
    }
    else
      // if the IK allocator is for this group, we use it
      result.first = jt->second;
    jmg->setSolverAllocators(result);
  }
}

void robot_model::RobotModel::printModelInfo(std::ostream &out) const
{
  out << "Model " << model_name_ << " in frame " << model_frame_ << ", of dimension " << getVariableCount() << std::endl;

  std::ios_base::fmtflags old_flags = out.flags();
  out.setf(std::ios::fixed, std::ios::floatfield);
  std::streamsize old_prec = out.precision();
  out.precision(5);
  out << "Joint values bounds: " << std::endl;
  for (unsigned int i = 0 ; i < joint_model_vector_.size() ; ++i)
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
      if (joint_model_vector_[i]->mimic_)
        out << " *";
      if (joint_model_vector_[i]->passive_)
        out << " +";
      out << std::endl;
    }
  }
  out << std::endl;
  out.precision(old_prec);
  out.flags(old_flags);

  out << "Available groups: " << std::endl;
  for (std::map<std::string, JointModelGroup*>::const_iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end() ; ++it)
  {
    out << "   " << it->first << " (of dimension " << it->second->getVariableCount() << "):" << std::endl;
    out << "     joints:" << std::endl;
    const std::vector<std::string> &jnt = it->second->getJointModelNames();
    for (std::size_t k = 0 ; k < jnt.size() ; ++k)
      out << "      " << jnt[k] << std::endl;
    out << "     links:" << std::endl;
    const std::vector<std::string> &lnk = it->second->getLinkModelNames();
    for (std::size_t k = 0 ; k < lnk.size() ; ++k)
      out << "      " << lnk[k] << std::endl;
    out << "     roots:" << std::endl;
    const std::vector<const JointModel*> &jr = it->second->getJointRoots();
    for (std::size_t k = 0 ; k < jr.size() ; ++k)
      out << "      " << jr[k]->getName() << std::endl;

  }
}

void robot_model::RobotModel::computeFixedTransforms(LinkModel *link, const Eigen::Affine3d &transform, LinkModelToAffine3dMap &associated_transforms)
{
  associated_transforms[link] = transform;
  for (std::size_t i = 0 ; i < link->getChildJointModels().size() ; ++i)
    if (link->getChildJointModels()[i]->getType() == JointModel::FIXED)
      computeFixedTransforms(link->getChildJointModels()[i]->child_link_model_, transform * link->getJointOriginTransform(), associated_transforms);
}
