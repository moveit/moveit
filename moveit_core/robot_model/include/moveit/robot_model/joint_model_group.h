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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef MOVEIT_ROBOT_MODEL_JOINT_MODEL_GROUP_
#define MOVEIT_ROBOT_MODEL_JOINT_MODEL_GROUP_

#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <boost/function.hpp>
#include <set>

namespace robot_model
{

class RobotModel;
class JointModelGroup;

/// function type that allocates a kinematics solver for a particular group
typedef boost::function<kinematics::KinematicsBasePtr(const JointModelGroup*)> SolverAllocatorFn;

/// function type that allocates a kinematics solvers for subgroups of a group
typedef std::map<const JointModelGroup*, SolverAllocatorFn> SolverAllocatorMapFn;

class JointModelGroup
{
  friend class RobotModel;

public:

  JointModelGroup(const std::string& name, const std::vector<const JointModel*>& joint_vector, const RobotModel *parent_model);

  ~JointModelGroup();

  /** \brief Get the kinematic model this group is part of */
  const RobotModel* getParentModel() const
  {
    return parent_model_;
  }

  /** \brief Get the name of the joint group */
  const std::string& getName() const
  {
    return name_;
  }

  /** \brief Check if a joint is part of this group */
  bool hasJointModel(const std::string &joint) const;

  /** \brief Check if a link is part of this group */
  bool hasLinkModel(const std::string &link) const;

  /** \brief Get a joint by its name. Return NULL if the joint is not part of this group. */
  const JointModel* getJointModel(const std::string &joint) const;

  /** \brief Get a joint by its name. Return NULL if the joint is not part of this group. */
  const LinkModel* getLinkModel(const std::string &joint) const;

  /** \brief Get the active joints in this group (that  have controllable DOF).
      This may not be the complete set of joints (see getFixedJointModels() and getMimicJointModels() ) */
  const std::vector<const JointModel*>& getJointModels() const
  {
    return joint_model_vector_;
  }

  /** \brief Get the names of the active joints in this group. These are the names of the joints returned by getJointModels(). */
  const std::vector<std::string>& getJointModelNames() const
  {
    return joint_model_name_vector_;
  }

  /** \brief Get the fixed joints that are part of this group */
  const std::vector<const JointModel*>& getFixedJointModels() const
  {
    return fixed_joints_;
  }

  /** \brief Get the mimic joints that are part of this group */
  const std::vector<const JointModel*>& getMimicJointModels() const
  {
    return mimic_joints_;
  }

  /** \brief Get the array of continuous joints used in thir group. */
  const std::vector<const JointModel*>& getContinuousJointModels() const
  {
    return continuous_joint_model_vector_const_;
  }

  /** \brief Get the names of the variables that make up the joints included in this group. Only active joints (not
      fixed, not mimic) are included. Effectively, these are the names of the DOF for this group. The number of
      returned elements is always equal to getVariableCount() */
  const std::vector<std::string>& getVariableNames() const
  {
    return active_variable_names_;
  }

  /** \brief Unlike a complete kinematic model, a group may
      contain disconnected parts of the kinematic tree -- a
      set of smaller trees. This function gives the roots of
      those smaller trees. Furthermore, it is ensured that
      the roots are on different branches in the kinematic
      tree. This means that in following any root in the given
      list, none of the other returned roots will be encountered. */
  const std::vector<const JointModel*>& getJointRoots() const
  {
    return joint_roots_;
  }

  /** \brief Get the links that are part of this joint group */
  const std::vector<const LinkModel*>& getLinkModels() const
  {
    return link_model_vector_;
  }

  /** \brief Get the names of the links that are part of this joint group */
  const std::vector<std::string>& getLinkModelNames() const
  {
    return link_model_name_vector_;
  }

  /** \brief Get the names of the links that are part of this joint group and also have geometry associated with them */
  const std::vector<std::string>& getLinkModelNamesWithCollisionGeometry() const
  {
    return link_model_with_geometry_name_vector_;
  }

  /** \brief Get the names of the links that are to be updated when the state of this group changes. This
      includes links that are in the kinematic model but outside this group, if those links are descendants of
      joints in this group that have their values updated. The order is the correct order for updating the corresponding states. */
  const std::vector<const LinkModel*>& getUpdatedLinkModels() const
  {
    return updated_link_model_vector_;
  }

  /** \brief Get the names of the links returned by getUpdatedLinkModels() */
  const std::vector<std::string>& getUpdatedLinkModelNames() const
  {
    return updated_link_model_name_vector_;
  }

  /** \brief Get the names of the links that are to be updated when the state of this group changes. This
      includes links that are in the kinematic model but outside this group, if those links are descendants of
      joints in this group that have their values updated. */
  const std::vector<const LinkModel*>& getUpdatedLinkModelsWithGeometry() const
  {
    return updated_link_model_with_geometry_vector_;
  }

  /** \brief Return the same data as getUpdatedLinkModelsWithGeometry() but as a set */
  const std::set<const LinkModel*>& getUpdatedLinkModelsWithGeometrySet() const
  {
    return updated_link_model_with_geometry_set_;
  }

  /** \brief Get the names of the links returned by getUpdatedLinkModels() */
  const std::vector<std::string>& getUpdatedLinkModelsWithGeometryNames() const
  {
    return updated_link_model_with_geometry_name_vector_;
  }

  /** \brief Get the names of the links returned by getUpdatedLinkModels() */
  const std::set<std::string>& getUpdatedLinkModelsWithGeometryNamesSet() const
  {
    return updated_link_model_with_geometry_name_set_;
  }

  /** \brief True if this name is in the set of links that are to be updated when the state of this group changes. This
      includes links that are in the kinematic model but outside this group, if those links are descendants of
      joints in this group that have their values updated. */
  bool isLinkUpdated(const std::string &name) const
  {
    return updated_link_model_name_set_.find(name) != updated_link_model_name_set_.end();
  }

  /** \brief Is the joint in the list of active joints in this group (that  have controllable DOF).
      This may not be the complete set of joints (see getFixedJointModels() and getMimicJointModels() ) */
  bool isActiveDOF(const std::string &name) const
  {
    return active_variable_names_set_.find(name) != active_variable_names_set_.end();
  }

  /** \brief A joint group consists of an array of joints. Each joint has a specific ordering of its variables.
      Given the ordering of joints the group maintains, an ordering of all the variables of the group can be then constructed.
      The map from variable names to their position in the joint group state is given by this function */
  const std::map<std::string, unsigned int>& getJointVariablesIndexMap() const
  {
    return joint_variables_index_map_;
  }

  /** \brief get the names of the known default states (as specified in the SRDF) */
  void getKnownDefaultStates(std::vector<std::string> &default_states) const;

  /** \brief Get the values that correspond to a named state as read from the URDF. Return false on failure. */
  bool getVariableDefaultValues(const std::string &name, std::map<std::string, double> &values) const;

  /** \brief Compute the default values for the joint group */
  void getVariableDefaultValues(std::vector<double> &values) const;

  /** \brief Compute the default values for the joint group */
  void getVariableDefaultValues(std::map<std::string, double> &values) const;

  /** \brief Compute random values for the state of the joint group */
  void getVariableRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values) const;

  /** \brief Compute random values for the state of the joint group */
  void getVariableRandomValuesNearBy(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values, const std::vector<double> &near, const std::map<robot_model::JointModel::JointType, double> &distance_map) const;

  /** \brief Compute random values for the state of the joint group */
  void getVariableRandomValuesNearBy(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values, const std::vector<double> &near, const std::vector<double> &distances) const;

  /** \brief Get the number of variables that describe this joint group */
  unsigned int getVariableCount() const
  {
    return variable_count_;
  }

  /** \brief Get the names of the groups that are subsets of this one (in terms of joints set) */
  const std::vector<std::string>& getSubgroupNames() const
  {
    return subgroup_names_;
  }

  /** \brief Check if the joints of group \e group are a subset of the joints in this group */
  bool isSubgroup(const std::string& group) const;

  /** \brief Check if this group is a linear chain */
  bool isChain() const
  {
    return is_chain_;
  }

  /** \brief Check if this group was designated as an end-effector in the SRDF */
  bool isEndEffector() const
  {
    return is_end_effector_;
  }

  /** \brief Return the name of the end effector, if this group is an end-effector */
  const std::string& getEndEffectorName() const
  {
    return end_effector_name_;
  }

  /** \brief Get the name of the group this end-effector attaches to (first) and the name of the link in that group (second) */
  const std::pair<std::string, std::string>& getEndEffectorParentGroup() const
  {
    return end_effector_parent_;
  }

  /** \brief Get the names of the end effectors attached to this group */
  const std::vector<std::string>& getAttachedEndEffectorNames() const
  {
    return attached_end_effector_names_;
  }

  /** \brief Get the extent of the state space (the maximum value distance() can ever report for this group) */
  double getMaximumExtent(void) const;

  /** \brief Get the joint limits as read from the URDF */
  std::vector<moveit_msgs::JointLimits> getVariableDefaultLimits() const;

  /** \brief Get the joint limits specified by the user with setJointLimits() or the default joint limits using getVariableDefaultLimits(), if no joint limits were specified. */
  std::vector<moveit_msgs::JointLimits> getVariableLimits() const;

  /** \brief Override joint limits */
  void setVariableLimits(const std::vector<moveit_msgs::JointLimits>& jlim);

  const std::pair<SolverAllocatorFn, SolverAllocatorMapFn>& getSolverAllocators() const
  {
    return solver_allocators_;
  }

  void setSolverAllocators(const SolverAllocatorFn &solver, const SolverAllocatorMapFn &solver_map = SolverAllocatorMapFn())
  {
    setSolverAllocators(std::make_pair(solver, solver_map));
  }

  void setSolverAllocators(const std::pair<SolverAllocatorFn, SolverAllocatorMapFn> &solvers);

  const kinematics::KinematicsBaseConstPtr& getSolverInstance() const
  {
    return solver_instance_const_;
  }

  const kinematics::KinematicsBasePtr& getSolverInstance()
  {
    return solver_instance_;
  }

  bool canSetStateFromIK(const std::string &tip) const;

  bool setRedundantJoints(const std::vector<unsigned int> &joints)
  {
    if(solver_instance_)
      return (solver_instance_->setRedundantJoints(joints));
    return false;
  }

  /** \brief Get the default IK timeout */
  double getDefaultIKTimeout() const
  {
    return default_ik_timeout_;
  }

  /** \brief Set the default IK timeout */
  void setDefaultIKTimeout(double ik_timeout);

  /** \brief Get the default IK attempts */
  unsigned int getDefaultIKAttempts() const
  {
    return default_ik_attempts_;
  }

  /** \brief Set the default IK attempts */
  void setDefaultIKAttempts(unsigned int ik_attempts)
  {
    default_ik_attempts_ = ik_attempts;
  }

  /** \brief Return the mapping between the order of the joints in this group and the order of the joints in the kinematics solver */
  const std::vector<unsigned int>& getKinematicsSolverJointBijection() const
  {
    return ik_joint_bijection_;
  }

  /** \brief Print information about the constructed model */
  void printGroupInfo(std::ostream &out = std::cout) const;

protected:

  /** \brief Owner model */
  const RobotModel                                 *parent_model_;

  /** \brief Name of group */
  std::string                                           name_;

  /** \brief Names of joints in the order they appear in the group state */
  std::vector<std::string>                              joint_model_name_vector_;

  /** \brief Joint instances in the order they appear in the group state */
  std::vector<const JointModel*>                        joint_model_vector_;

  /** \brief A map from joint names to their instances */
  std::map<std::string, const JointModel*>              joint_model_map_;

  /** \brief The list of joint models that are roots in this group */
  std::vector<const JointModel*>                        joint_roots_;

  /** \brief The group includes all the joint variables that make up the joints the group consists of.
      This map gives the position in the state vector of the group for each of these variables.
      Additionaly, it includes the names of the joints and the index for the first variable of that joint. */
  std::map<std::string, unsigned int>                   joint_variables_index_map_;

  /** \brief The joints that have no DOF (fixed) */
  std::vector<const JointModel*>                        fixed_joints_;

  /** \brief Joints that mimic other joints */
  std::vector<const JointModel*>                        mimic_joints_;

  /** \brief The set of continuous joints this group contains */
  std::vector<const JointModel*>                        continuous_joint_model_vector_const_;

  /** \brief The names of the DOF that make up this group (this is just a sequence of joint variable names; not necessarily joint names!) */
  std::vector<std::string>                              active_variable_names_;

  /** \brief The names of the DOF that make up this group (this is just a sequence of joint variable names; not necessarily joint names!) */
  std::set<std::string>                                 active_variable_names_set_;

  /** \brief The links that are on the direct lineage between joints
      and joint_roots_, as well as the children of the joint leafs.
      May not be in any particular order */
  std::vector<const LinkModel*>                         link_model_vector_;

  /** \brief The names of the links in this group */
  std::vector<std::string>                              link_model_name_vector_;

  /** \brief The names of the links in this group that also have geometry */
  std::vector<std::string>                              link_model_with_geometry_name_vector_;

  /** \brief The list of downstream link models in the order they should be updated (may include links that are not in this group) */
  std::vector<const LinkModel*>                         updated_link_model_vector_;

  /** \brief The list of downstream link models in the order they should be updated (may include links that are not in this group) */
  std::set<const LinkModel*>                            updated_link_model_set_;

  /** \brief The list of downstream link names in the order they should be updated (may include links that are not in this group) */
  std::vector<std::string>                              updated_link_model_name_vector_;

  /** \brief The list of downstream link names in the order they should be updated (may include links that are not in this group) */
  std::set<std::string>                                 updated_link_model_name_set_;

  /** \brief The list of downstream link models in the order they should be updated (may include links that are not in this group) */
  std::vector<const LinkModel*>                         updated_link_model_with_geometry_vector_;

  /** \brief The list of downstream link models in the order they should be updated (may include links that are not in this group) */
  std::set<const LinkModel*>                            updated_link_model_with_geometry_set_;

  /** \brief The list of downstream link names in the order they should be updated (may include links that are not in this group) */
  std::vector<std::string>                              updated_link_model_with_geometry_name_vector_;

  /** \brief The list of downstream link names in the order they should be updated (may include links that are not in this group) */
  std::set<std::string>                                 updated_link_model_with_geometry_name_set_;

  /** \brief The number of variables necessary to describe this group of joints */
  unsigned int                                          variable_count_;

  /** \brief The set of labelled subgroups that are included in this group */
  std::vector<std::string>                              subgroup_names_;

  /** \brief Flag indicating whether this group is an end effector */
  bool                                                  is_end_effector_;

  /** \brief If an end-effector is attached to this group, the name of that end-effector is stored in this variable */
  std::vector<std::string>                              attached_end_effector_names_;

  /** \brief First: name of the group that is parent to this end-effector group; Second: the link this in the parent group that this group attaches to */
  std::pair<std::string, std::string>                   end_effector_parent_;

  /** \brief The name of the end effector, if this group is an end-effector */
  std::string                                           end_effector_name_;

  bool                                                  is_chain_;

  std::pair<SolverAllocatorFn, SolverAllocatorMapFn>    solver_allocators_;

  kinematics::KinematicsBaseConstPtr                    solver_instance_const_;

  kinematics::KinematicsBasePtr                         solver_instance_;

  std::vector<unsigned int>                             ik_joint_bijection_;

  double                                                default_ik_timeout_;

  unsigned int                                          default_ik_attempts_;

  /** \brief The set of default states specified for this group in the SRDF */
  std::map<std::string, std::map<std::string, double> > default_states_;
};

}

#endif
