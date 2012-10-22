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

/*------------------------------------------------------*/
/*   DO NOT INCLUDE THIS FILE DIRECTLY                  */
/*------------------------------------------------------*/

/** @class JointStateGroup
 *  @brief The joint state corresponding to a group
 */
class JointStateGroup
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  /**
   *  @brief Default constructor
   *  @param state A pointer to the kinematic state
   *  @param jmg The joint model group corresponding to this joint state
   */
  JointStateGroup(KinematicState *state, const KinematicModel::JointModelGroup *jmg);
  ~JointStateGroup(void);
  
  /** \brief Get the kinematic state this link is part of */
  const KinematicState* getParentState(void) const
  {
    return kinematic_state_;
  }
  
  /** \brief Get the kinematic state this link is part of */
  KinematicState* getKinematicState(void)
  {
    return kinematic_state_;
  }
  
  /** \brief Get the joint model corresponding to this joint state group */
  const KinematicModel::JointModelGroup* getJointModelGroup(void)
  {
    return joint_model_group_;
  }
  
  /** \brief Get the name of the joint model group corresponding to this joint state*/
  const std::string& getName(void) const
  {
    return joint_model_group_->getName();
  }
  
  /** \brief Get the number of (active) DOFs for the joint model group corresponding to this state*/
  unsigned int getVariableCount(void) const
  {
    return joint_model_group_->getVariableCount();
  }
  
  /** \brief Perform forward kinematics starting at the roots
      within a group. Links that are not in the group are also
      updated, but transforms for joints that are not in the
      group are not recomputed.  */
  bool setStateValues(const std::vector<double>& joint_state_values);
  
  /** \brief Perform forward kinematics starting at the roots
      within a group. Links that are not in the group are also
      updated, but transforms for joints that are not in the
      group are not recomputed.  */
  void setStateValues(const std::map<std::string, double>& joint_state_map);
  
  /** \brief Perform forward kinematics starting at the roots
      within a group. Links that are not in the group are also
      updated, but transforms for joints that are not in the
      group are not recomputed.  */
  void setStateValues(const sensor_msgs::JointState& js);
  
  /** Compute transforms using current joint values */
  void updateLinkTransforms(void);
  
  /** \brief Check if a joint is part of this group */
  bool hasJointState(const std::string &joint) const;
  
  /** \brief Check if a link is updated by this group */
  bool updatesLinkState(const std::string& joint) const;
  
  /** \brief Get a joint state by its name */
  JointState* getJointState(const std::string &joint) const;
  
  /** \brief Get current joint values */
  void getGroupStateValues(std::vector<double>& joint_state_values) const;
  
  /** \brief Get a map between variable names and joint state values */
  void getGroupStateValues(std::map<std::string, double>& joint_state_values) const;
  
  /** \brief Copy the values from another joint state group */
  void copyFrom(const JointStateGroup *other_jsg);
  
  /** \brief Bring the group to a default state. All joints are
      at 0. If 0 is not within the bounds of the joint, the
      middle of the bounds is used. */
  void setToDefaultValues(void);
  
  /** \brief Set the group to a named default state. Return false on failure */
  bool setToDefaultState(const std::string &name);
  
  /** \brief Sample a random state in accordance with the type of joints employed */
  void setToRandomValues(void);

  /** \brief Checks if the current joint state values are all within the bounds set in the model */
  bool satisfiesBounds(void) const;
  
  /** \brief Force the joint to be inside bounds and normalized. Quaternions are normalized, continuous joints are made between -Pi and Pi. */
  void enforceBounds(void);
  
  double distance(const JointStateGroup *other) const;
  
  void interpolate(const JointStateGroup *to, const double t, JointStateGroup *dest) const;

  /** \brief Get the state corresponding to root joints in this group*/
  const std::vector<JointState*>& getJointRoots(void) const
  {
    return joint_roots_;
  }
  
  /** \brief Get the joint names corresponding to this joint state*/
  const std::vector<std::string>& getJointNames(void) const
  {
    return joint_model_group_->getJointModelNames();
  }
  
  /** \brief Get the vector of joint state for this group*/
  const std::vector<JointState*>& getJointStateVector(void) const
  {
    return joint_state_vector_;
  }
  
  /** \brief Return the instance of a random number generator */
  random_numbers::RandomNumberGenerator& getRandomNumberGenerator(void);

  /** \brief Given a set of joint angles, compute the jacobian with reference to a particular point on a given link
   * \param link_name The name of the link 
   * \param reference_point_position The reference point position (with respect to the link specified in link_index)
   * \param jacobian The resultant jacobian
   * \return True if jacobian was successfully computed, false otherwise
   */    
  bool getJacobian(const std::string &link_name, const Eigen::Vector3d &reference_point_position, Eigen::MatrixXd& jacobian) const;
  
  

  bool setFromIK(const geometry_msgs::Pose &pose, const std::string &tip, double timeout);
  
  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be set by computing inverse kinematics.
      The transform is assumed to be in the reference frame of the kinematic model. Returns true on success. */
  bool setFromIK(const geometry_msgs::Pose &pose, double timeout);

  /** \brief If the group this state corresponds to is a chain and a solver is available, then the joint values can be set by computing inverse kinematics.
      The transform is assumed to be in the reference frame of the kinematic model. Returns true on success. */
  bool setFromIK(const Eigen::Affine3d &pose, double timeout);

  bool setFromIK(const Eigen::Affine3d &pose, const std::string &tip, double timeout);
      
private:
  
  /** \brief The kinematic state this group is part of */
  KinematicState                        *kinematic_state_;
  
  /** \brief The model of the group that corresponds to this state */
  const KinematicModel::JointModelGroup *joint_model_group_;
  
  /** \brief Joint instances in the order they appear in the group state */
  std::vector<JointState*>               joint_state_vector_;
  
  /** \brief A map from joint names to their instances */
  std::map<std::string, JointState*>     joint_state_map_;
  
  /** \brief The list of joints that are roots in this group */
  std::vector<JointState*>               joint_roots_;
  
  /** \brief The list of links that are updated when computeTransforms() is called, in the order they are updated */
  std::vector<LinkState*>                updated_links_;
  
  /** \brief For certain operations a group needs a random number generator. However, it may be slightly expensive
      to allocate the random number generator if many state instances are generated. For this reason, the generator
      is allocated on a need basis, by the getRandomNumberGenerator() function. Never use the rng_ member directly, but call
      getRandomNumberGenerator() instead. */
  boost::scoped_ptr<random_numbers::RandomNumberGenerator> rng_;

};
