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

/*------------------------------------------------------*/
/*   DO NOT INCLUDE THIS FILE DIRECTLY                  */
/*------------------------------------------------------*/

class JointModelGroup
{
  friend class KinematicModel;
public:
  
  JointModelGroup(const std::string& name, const std::vector<const JointModel*>& joint_vector, const KinematicModel *parent_model);
  
  virtual ~JointModelGroup(void);
  
  /** \brief Get the kinematic model this group is part of */
  const KinematicModel* getParentModel(void) const
  {
    return parent_model_;
  }
  
  /** \brief Get the name of the joint group */
  const std::string& getName(void) const
  {
    return name_;
  }
  
  /** \brief Check if a joint is part of this group */
  bool hasJointModel(const std::string &joint) const;
  
  /** \brief Check if a link is part of this group */
  bool hasLinkModel(const std::string &link) const;
   
  /** \brief Get a joint by its name. Return NULL if the joint is not part of this group. */
  const JointModel* getJointModel(const std::string &joint) const;
  
  /** \brief Get the active joints in this group (that  have controllable DOF).
      This may not be the complete set of joints (see getFixedJointModels() and getMimicJointModels() ) */
  const std::vector<const JointModel*>& getJointModels(void) const
  {
    return joint_model_vector_;
  }
  
  /** \brief Get the fixed joints that are part of this group */
  const std::vector<const JointModel*>& getFixedJointModels(void) const
  {
    return fixed_joints_;
  }
  
  /** \brief Get the mimic joints that are part of this group */
  const std::vector<const JointModel*>& getMimicJointModels(void) const
  {
    return mimic_joints_;
  }
  
  /** \brief Get the names of the active joints in this group. These are the names of the joints returned by getJointModels(). */
  const std::vector<std::string>& getJointModelNames(void) const
  {
    return joint_model_name_vector_;
  }
  
  /** \brief Get the names of the variables that make up the joints included in this group. Only active joints (not
      fixed, not mimic) are included. Effectively, these are the names of the DOF for this group. The number of
      returned elements is always equal to getVariableCount() */
  const std::vector<std::string>& getActiveDOFNames(void) const
  {
    return active_dof_names_;
  }
  
  /** \brief Unlike a complete kinematic model, a group may
      contain disconnected parts of the kinematic tree -- a
      set of smaller trees. This function gives the roots of
      those smaller trees. Furthermore, it is ensured that
      the roots are on different branches in the kinematic
      tree. This means that in following any root in the given
      list, none of the other returned roots will be encountered. */
  const std::vector<const JointModel*>& getJointRoots(void) const
  {
    return joint_roots_;
  }
  
  /** \brief Get the links that are part of this joint group */
  const std::vector<const LinkModel*>& getLinkModels(void) const
  {
    return link_model_vector_;
  }
  
  /** \brief Get the names of the links that are part of this joint group */
  const std::vector<std::string>& getLinkModelNames(void) const
  {
    return link_model_name_vector_;
  }
  
  /** \brief Get the names of the links that are to be updated when the state of this group changes. This
      includes links that are in the kinematic model but outside this group, if those links are descendants of
      joints in this group that have their values updated. */
  const std::vector<const LinkModel*>& getUpdatedLinkModels(void) const
  {
    return updated_link_model_vector_;
  }
  
  /** \brief Get the names of the links returned by getUpdatedLinkModels() */
  const std::vector<std::string>& getUpdatedLinkModelNames(void) const
  {
    return updated_link_model_name_vector_;
  }

  /** \brief Get the names of the links that are to be updated when the state of this group changes. This
      includes links that are in the kinematic model but outside this group, if those links are descendants of
      joints in this group that have their values updated. */
  const std::vector<const LinkModel*>& getUpdatedLinkModelsWithGeometry(void) const
  {
    return updated_link_model_with_geometry_vector_;
  }
  
  /** \brief Get the names of the links returned by getUpdatedLinkModels() */
  const std::vector<std::string>& getUpdatedLinkModelsWithGeometryNames(void) const
  {
    return updated_link_model_with_geometry_name_vector_;
  }
  
  /** \brief True if this name is in the set of links that are to be updated when the state of this group changes. This
      includes links that are in the kinematic model but outside this group, if those links are descendants of
      joints in this group that have their values updated. */
  bool isUpdatedLink(const std::string &name) const
  {
    if(std::find(updated_link_model_name_vector_.begin(),updated_link_model_name_vector_.end(),name) == updated_link_model_name_vector_.end())
      return false;
    return true;
  }
  
  /** \brief Is the joint in the list of active joints in this group (that  have controllable DOF).
      This may not be the complete set of joints (see getFixedJointModels() and getMimicJointModels() ) */
  bool isActiveDOF(const std::string &name) const
  {
    if(std::find(active_dof_names_.begin(),active_dof_names_.end(),name) == active_dof_names_.end())
      return false;
    return true;    
  }
  
  /** \brief A joint group consists of an array of joints. Each joint has a specific ordering of its variables.
      Given the ordering of joints the group maintains, an ordering of all the variables of the group can be then constructed.
      The map from variable names to their position in the joint group state is given by this function */
  const std::map<std::string, unsigned int>& getJointVariablesIndexMap(void) const
  {
    return joint_variables_index_map_;
  }
  
  /** \brief Get the values that correspond to a named state as read from the URDF. Return false on failure. */
  bool getDefaultValues(const std::string &name, std::map<std::string, double> &values) const;
  
  /** \brief Compute the default values for the joint group */
  void getDefaultValues(std::vector<double> &values) const;
  
  /** \brief Compute the default values for the joint group */
  void getDefaultValues(std::map<std::string, double> &values) const;
  
  /** \brief Compute random values for the state of the joint group */
  void getRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values) const;
  
  /** \brief Get the number of variables that describe this joint group */
  unsigned int getVariableCount(void) const
  {
    return variable_count_;
  }
  
  /** \brief Get the names of the groups that are subsets of this one (in terms of joints set) */
  const std::vector<std::string>& getSubgroupNames(void) const
  {
    return subgroup_names_;
  }
  
  /** \brief Check if the joints of group \e group are a subset of the joints in this group */
  bool isSubgroup(const std::string& group) const;
  
  /** \brief Get the name of the group that makes up the end effector attached to this group's */
  const std::string& getAttachedEndEffectorGroupName(void) const
  {
    return attached_end_effector_group_name_;
  }

  /** \brief Check if this group was designated as an end-effector in the SRDF */
  bool isEndEffector(void) const
  {
    return is_end_effector_;
  }

  /** \brief Get the name of the group this end-effector attaches to (first) and the name of the link in that group (second) */
  const std::pair<std::string, std::string>& getEndEffectorParentGroup(void) const
  {
    return end_effector_parent_;
  }
  
  /** \brief Check if this group is a linear chain */
  bool isChain(void) const
  {
    return is_chain_;
  }
  
  /** \brief Get the joint limits as read from the URDF */
  virtual std::vector<moveit_msgs::JointLimits> getVariableLimits(void) const;
  
  /** \brief Override joint limits */
  void setJointLimits(const std::vector<moveit_msgs::JointLimits>& jlim)
  {
    user_specified_joint_limits_ = jlim;
  }
  
  /** \brief Get the joint limits specified by the user with setJointLimits() or the default joint limits using getVariableLimits(), if no joint limits were specified. */
  std::vector<moveit_msgs::JointLimits> getJointLimits(void) const
  {
    return user_specified_joint_limits_.empty() ? getVariableLimits() : user_specified_joint_limits_;
  }
  
  const std::pair<SolverAllocatorFn, SolverAllocatorMapFn>& getSolverAllocators(void) const
  {
    return solver_allocators_;
  }

  void setSolverAllocators(const std::pair<SolverAllocatorFn, SolverAllocatorMapFn> &solvers);

  const kinematics::KinematicsBaseConstPtr& getSolverInstance(void) const
  {
    return solver_instance_;
  }
  
  /** \brief Return the mapping between the order of the joints in this group and the order of the joints in the kinematics solver */
  const std::vector<unsigned int>& getKinematicsSolverJointBijection(void) const
  {
    return ik_joint_bijection_;
  }
  
  /** \brief Print information about the constructed model */
  void printGroupInfo(std::ostream &out = std::cout) const;
  
protected:
  
  /** \brief Owner model */
  const KinematicModel                                 *parent_model_;
  
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
  
  /** \brief The names of the DOF that make up this group (this is just a sequence of joint variable names; not necessarily joint names!) */
  std::vector<std::string>                              active_dof_names_;
  
  /** \brief The links that are on the direct lineage between joints
      and joint_roots_, as well as the children of the joint leafs.
      May not be in any particular order */
  std::vector<const LinkModel*>                         link_model_vector_;
  
  /** \brief The names of the links in this group */
  std::vector<std::string>                              link_model_name_vector_;
  
  /** \brief The list of downstream link models in the order they should be updated (may include links that are not in this group) */
  std::vector<const LinkModel*>                         updated_link_model_vector_;
  
  /** \brief The list of downstream link names in the order they should be updated (may include links that are not in this group) */
  std::vector<std::string>                              updated_link_model_name_vector_;

  /** \brief The list of downstream link models in the order they should be updated (may include links that are not in this group) */
  std::vector<const LinkModel*>                         updated_link_model_with_geometry_vector_;
  
  /** \brief The list of downstream link names in the order they should be updated (may include links that are not in this group) */
  std::vector<std::string>                              updated_link_model_with_geometry_name_vector_;
  
  /** \brief The number of variables necessary to describe this group of joints */
  unsigned int                                          variable_count_;
  
  /** \brief The set of labelled subgroups that are included in this group */
  std::vector<std::string>                              subgroup_names_;
  
  /** \brief Flag indicating whether this group is an end effector */
  bool                                                  is_end_effector_;
  
  /** \brief If a group that is in fact an end-effector is attached to this one, the name of that group is stored in this variable */
  std::string                                           attached_end_effector_group_name_;
  
  /***\brief First: name of the group that is parent to this end-effector group; Second: the link this in the parent group that this group attaches to */
  std::pair<std::string, std::string>                   end_effector_parent_;
  
  bool                                                  is_chain_;

  std::vector<moveit_msgs::JointLimits>                 user_specified_joint_limits_;
  
  std::pair<SolverAllocatorFn, SolverAllocatorMapFn>    solver_allocators_;
  
  kinematics::KinematicsBaseConstPtr                    solver_instance_;
  
  std::vector<unsigned int>                             ik_joint_bijection_;
 
  /** \brief The set of default states specified for this group in the SRDF */
  std::map<std::string, std::map<std::string, double> > default_states_;
};

