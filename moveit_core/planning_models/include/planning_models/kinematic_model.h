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

#ifndef MOVEIT_PLANNING_MODELS_KINEMATIC_MODEL_
#define MOVEIT_PLANNING_MODELS_KINEMATIC_MODEL_

#include <urdf/model.h>
#include <srdf/model.h>
#include <geometric_shapes/shapes.h>
#include <random_numbers/random_numbers.h>

#include <boost/shared_ptr.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <moveit_msgs/JointLimits.h>

#include <Eigen/Geometry>


/** \brief Main namespace for representing robot planning models */
namespace planning_models
{

/** \brief Definition of a kinematic model. This class is not thread
    safe, however multiple instances can be created */
class KinematicModel
{
public:
  /** \brief Forward definition of a joint */
  class JointModel;
  
  /** \brief Forward definition of a link */
  class LinkModel;
  
  /** \brief A joint from the robot. Models the transform that
      this joint applies in the kinematic chain. A joint
      consists of multiple variables. In the simplest case, when
      the joint is a single DOF, there is only one variable and
      its name is the same as the joint's name. For multi-DOF
      joints, each variable has a local name (e.g., \e x, \e y)
      but the full variable name as seen from the outside of
      this class is a concatenation of the "joint name"."local
      name" (e.g., a joint named 'base' with local variables 'x'
      and 'y' will store its full variable names as 'base.x' and
      'base.y'). Local names are never used to reference
      variables directly. */
  class JointModel
  {
    friend class KinematicModel;
  public:
    
    enum JointType
      {
        UNKNOWN, REVOLUTE, PRISMATIC, PLANAR, FLOATING, FIXED
      };
    
    /** \brief Construct a joint named \e name */
    JointModel(const std::string& name);
    
    virtual ~JointModel(void);
    
    /** \brief Get the name of the joint */
    const std::string& getName(void) const
    {
      return name_;
    }
    
    /** \brief Get the type of joint */
    JointType getType(void) const
    {
      return type_;
    }
    
    /** \brief The index of this joint when traversing the kinematic tree in depth first fashion */
    int getTreeIndex(void) const
    {
      return tree_index_;
    }
    
    /** \brief Get the link that this joint connects to. The
        robot is assumed to start with a joint, so the root
        joint will return a NULL pointer here. */
    const LinkModel* getParentLinkModel(void) const
    {
      return parent_link_model_;
    }
    
    /** \brief Get the link that this joint connects to. There will always be such a link */
    const LinkModel* getChildLinkModel(void) const
    {
      return child_link_model_;
    }
    
    /** \brief Get the lower and upper bounds for a variable. Return false if the variable was not found */
    bool getVariableBounds(const std::string& variable, std::pair<double, double>& bounds) const;
    
    /** \brief Provide a default value for the joint given the joint bounds.
        Most joints will use the default implementation provided in this base class, but the quaternion
        for example needs a different implementation. The map is NOT cleared; elements are only added (or overwritten). */
    void getDefaultValues(std::map<std::string, double> &values) const;
    
    /** \brief Provide random values for the joint variables (within bounds). The map is NOT cleared; elements are only added (or overwritten). */
    void getRandomValues(random_numbers::RandomNumberGenerator &rng, std::map<std::string, double> &values) const;
    
    /** \brief Provide a default value for the joint given the joint variable bounds.
        Most joints will use the default implementation provided in this base class, but the quaternion
        for example needs a different implementation. The vector is NOT cleared; elements are only added with push_back */
    virtual void getDefaultValues(std::vector<double> &values) const;
    
    /** \brief Provide random values for the joint variables (within bounds). The vector is NOT cleared; elements are only added with push_back */
    virtual void getRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values) const;
    
    /** \brief Check if a particular variable satisfies the specified bounds */
    virtual bool isVariableWithinBounds(const std::string& variable, double value) const;
    
    /** \brief Check if the set of values for the variables of this joint are within bounds. */
    virtual bool satisfiesBounds(const std::vector<double> &values) const;

    /** \brief Force the specified values to be inside bounds and normalized. Quaternions are normalized, continuous joints are made between -Pi and Pi. */
    virtual void enforceBounds(std::vector<double> &values) const;

    /** \brief Get the names of the variables that make up this joint, in the order they appear in corresponding states.
        For single DOF joints, this will be just the joint name. */
    const std::vector<std::string>& getVariableNames(void) const
    {
      return variable_names_;
    }
    
    /** \brief Get the names of the variable suffixes that are attached to joint names to construct the variable names. For single DOF joints, this will be empty. */
    const std::vector<std::string>& getLocalVariableNames(void) const
    {
      return local_names_;
    }
    
    /** \brief Get the variable bounds for this joint, in the same order as the names returned by getVariableNames() */
    const std::vector<std::pair<double, double> > &getVariableBounds(void) const
    {
      return variable_bounds_;
    }
    
    /** \brief Check if a particular variable is known to this joint */
    bool hasVariable(const std::string &variable) const
    {
      return variable_index_.find(variable) != variable_index_.end();
    }
    
    /** \brief Get the number of variables that describe this joint */
    unsigned int getVariableCount(void) const
    {
      return variable_names_.size();
    }
    
    /** \brief The set of variables that make up the state value of a joint are stored in some order. This map
        gives the position of each variable in that order, for each variable name */
    const std::map<std::string, unsigned int>& getVariableIndexMap(void) const
    {
      return variable_index_;
    }
    
    /** \brief Get the joint this one is mimicking */
    const JointModel* getMimic(void) const
    {
      return mimic_;
    }
    
    /** \brief If mimicking a joint, this is the offset added to that joint's value */
    double getMimicOffset(void) const
    {
      return mimic_offset_;
    }
    
    /** \brief If mimicking a joint, this is the multiplicative factor for that joint's value */
    double getMimicFactor(void) const
    {
      return mimic_factor_;
    }
    
    /** \brief The joint models whose values would be modified if the value of this joint changed */
    const std::vector<const JointModel*>& getMimicRequests(void) const
    {
      return mimic_requests_;
    }
    
    /** \brief Get the maximum velocity of this joint. If the result is zero, the value is considered not to be specified. */
    double getMaximumVelocity(void) const
    {
      return max_velocity_;
    }

    virtual std::vector<moveit_msgs::JointLimits> getJointLimits(void) const;
    
    /** \brief Given the joint values for a joint, compute the corresponding transform */
    virtual void computeTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const = 0;
    
    /** \brief Given the transform generated by joint, compute the corresponding joint values */
    virtual void computeJointStateValues(const Eigen::Affine3d& transform, std::vector<double> &joint_values) const = 0;
    
    /** \brief Update a transform so that it corresponds to
        the new joint values assuming that \e transf was
        previously set to identity and that only calls to updateTransform() were issued afterwards */
    virtual void updateTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const = 0;
    
  protected:
    
    /** \brief Name of the joint */
    std::string                                       name_;
    
    /** \brief The type of joint */
    JointType                                         type_;
    
    /** \brief The local names to use for the variables that make up this joint */
    std::vector<std::string>                          local_names_;
    
    /** \brief The full names to use for the variables that make up this joint */
    std::vector<std::string>                          variable_names_;
    
    /** \brief The bounds for each variable (low, high) in the same order as variable_names_ */
    std::vector<std::pair<double, double> >           variable_bounds_;
    
    /** \brief The maximum velocity of this joint. If zero, the value is considered not to be specified. */
    double                                            max_velocity_;
    
    /** \brief Map from variable names to the corresponding index in variable_names_ */
    std::map<std::string, unsigned int>               variable_index_;
    
    /** \brief The link before this joint */
    LinkModel                                        *parent_link_model_;
    
    /** \brief The link after this joint */
    LinkModel                                        *child_link_model_;
    
    /** \brief The joint this one mimics (NULL for joints that do not mimic) */
    JointModel                                       *mimic_;
    
    /** \brief The offset to the mimic joint */
    double                                            mimic_factor_;
    
    /** \brief The multiplier to the mimic joint */
    double                                            mimic_offset_;
    
    /** \brief The set of joints that should get a value copied to them when this joint changes */
    std::vector<const JointModel*>                    mimic_requests_;
    
    /** \brief The index assigned to this joint when traversing the kinematic tree in depth first fashion */
    int                                               tree_index_;
  };
  
  /** \brief A fixed joint */
  class FixedJointModel : public JointModel
  {
    friend class KinematicModel;
  public:
    
    FixedJointModel(const std::string &name);
    
    virtual void computeTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const;
    virtual void computeJointStateValues(const Eigen::Affine3d& trans, std::vector<double>& joint_values) const;
    virtual void updateTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const;
  };
  
  /** \brief A planar joint */
  class PlanarJointModel : public JointModel
  {
    friend class KinematicModel;
  public:
    
    PlanarJointModel(const std::string& name);
    
    virtual bool isVariableWithinBounds(const std::string& variable, double value) const;
    virtual void enforceBounds(std::vector<double> &values) const;

    /// Make the yaw component of a state's value vector be in the range [-Pi, Pi]. enforceBounds() also calls this function
    void normalizeRotation(std::vector<double> &values) const;
    
    virtual void computeTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const;
    virtual void computeJointStateValues(const Eigen::Affine3d& transf, std::vector<double>& joint_values) const;
    virtual void updateTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const;
  };
  
  /** \brief A floating joint */
  class FloatingJointModel : public JointModel
  {
    friend class KinematicModel;
  public:
    
    FloatingJointModel(const std::string& name);
    virtual void enforceBounds(std::vector<double> &values) const;
    
    /// Normalize the quaternion (warn if norm is 0, and set to identity)
    void normalizeRotation(std::vector<double> &values) const;

    virtual void computeTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const;
    virtual void computeJointStateValues(const Eigen::Affine3d& transf, std::vector<double>& joint_values) const;
    virtual void updateTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const;
    virtual void getRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values) const;
    virtual void getDefaultValues(std::vector<double>& values) const;
  };
  
  /** \brief A prismatic joint */
  class PrismaticJointModel : public JointModel
  {
    friend class KinematicModel;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PrismaticJointModel(const std::string& name);
    
    virtual void computeTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const;
    virtual void computeJointStateValues(const Eigen::Affine3d& transf, std::vector<double> &joint_values) const;
    virtual void updateTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const;
    
    /** \brief Get the axis of translation */
    const Eigen::Vector3d& getAxis(void) const
    {
      return axis_;
    }
    
  protected:
    /** \brief The axis of the joint */
    Eigen::Vector3d axis_;
  };
  
  /** \brief A revolute joint */
  class RevoluteJointModel : public JointModel
  {
    friend class KinematicModel;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RevoluteJointModel(const std::string& name);

    virtual std::vector<moveit_msgs::JointLimits> getJointLimits(void) const;
    virtual bool isVariableWithinBounds(const std::string& variable, double value) const;
    virtual void enforceBounds(std::vector<double> &values) const;

    virtual void computeTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const;
    virtual void computeJointStateValues(const Eigen::Affine3d& transf, std::vector<double> &joint_values) const;
    virtual void updateTransform(const std::vector<double>& joint_values, Eigen::Affine3d &transf) const;
    
    /** \brief Check if this joint wraps around */
    bool isContinuous(void) const
    {
      return continuous_;
    }
    
    /** \brief Get the axis of rotation */
    const Eigen::Vector3d& getAxis(void) const
    {
      return axis_;
    }
    
  protected:
    /** \brief The axis of the joint */
    Eigen::Vector3d axis_;
    
    /** \brief Flag indicating whether this joint wraps around */
    bool      continuous_;
  };
  
  /** \brief A link from the robot. Contains the constant transform applied to the link and its geometry */
  class LinkModel
  {
    friend class KinematicModel;
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinkModel(void);
    ~LinkModel(void);
    
    /** \brief The name of this link */
    const std::string& getName(void) const
    {
      return name_;
    }
    
    /** \brief Get the filename of the mesh resource for this link */
    const std::string& getFilename(void) const
    {
      return filename_;
    }

    /** \brief Get the filename of the mesh resource for this link */
    const std::string& getVisualFilename(void) const
    {
      return visual_filename_;
    }
    
    /** \brief The index of this joint when traversing the kinematic tree in depth first fashion */
    int getTreeIndex(void) const
    {
      return tree_index_;
    }
    
    /** \brief Get the joint model whose child this link is. There will always be a parent joint */
    const JointModel* getParentJointModel(void) const
    {
      return parent_joint_model_;
    }
    
    /** \brief A link may have 0 or more child joints. From those joints there will certainly be other descendant links */
    const std::vector<JointModel*>& getChildJointModels(void) const
    {
      return child_joint_models_;
    }
    
    /** \brief When transforms are computed for this link,
        they are usually applied to the link's origin. The
        joint origin transform acts as an offset -- it is
        pre-applied before any other transform */
    const Eigen::Affine3d& getJointOriginTransform(void) const
    {
      return joint_origin_transform_;
    }
    
    /** \brief In addition to the link transform, the geometry
        of a link that is used for collision checking may have
        a different offset itself, with respect to the origin */
    const Eigen::Affine3d& getCollisionOriginTransform(void) const
    {
      return collision_origin_transform_;
    }
    
    /** \brief Get shape associated to the collision geometry for this link */
    const shapes::ShapeConstPtr& getShape(void) const
    {
      return shape_;
    }
    
  private:
    
    /** \brief Name of the link */
    std::string               name_;
    
    /** \brief JointModel that connects this link to the parent link */
    JointModel               *parent_joint_model_;
    
    /** \brief List of descending joints (each connects to a child link) */
    std::vector<JointModel*>  child_joint_models_;
    
    /** \brief The constant transform applied to the link (local) */
    Eigen::Affine3d           joint_origin_transform_;
    
    /** \brief The constant transform applied to the collision geometry of the link (local) */
    Eigen::Affine3d           collision_origin_transform_;
    
    /** \brief The collision geometry of the link */
    shapes::ShapeConstPtr     shape_;

    /** \brief Filename associated with the collision geometry mesh of this link (loaded in shape_). If empty, no mesh was used. */
    std::string               filename_;

    /** \brief Filename associated with the visual geometry mesh of this link (loaded in shape_). If empty, no mesh was used. */
    std::string               visual_filename_;
    
    /** \brief The index assigned to this link when traversing the kinematic tree in depth first fashion */
    int                       tree_index_;

  };
  
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
    
    /** \brief A joint group consists of an array of joints. Each joint has a specific ordering of its variables.
        Given the ordering of joints the group maintains, an ordering of all the variables of the group can be then constructed.
        The map from variable names to their position in the joint group state is given by this function */
    const std::map<std::string, unsigned int>& getJointVariablesIndexMap(void) const
    {
      return joint_variables_index_map_;
    }
    
    /** \brief Get the values that correspond to a named state as read from the URDF. Return false on failure. */
    bool getDefaultValues(const std::string &name, std::map<std::string, double> &values) const;
    
    /** \brief Compute random values for the state of the joint group */
    void getRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values) const;
    
    /** \brief Get the number of variables that describe this joint group */
    unsigned int getVariableCount(void) const
    {
      return variable_count_;
    }

    const std::vector<std::string>& getSubgroupNames() const {
      return subgroup_names_;
    }

    bool isSubgroup(const std::string& group) const {
      for(unsigned int i = 0; i < subgroup_names_.size(); i++) {
        if(group == subgroup_names_[i]) return true;
      }
      return false;
    }

    virtual std::vector<moveit_msgs::JointLimits> getJointLimits() const;
    
    
  protected:
    
    /** \brief Owner model */
    const KinematicModel                       *parent_model_;
    
    /** \brief Name of group */
    std::string                                 name_;
    
    /** \brief Names of joints in the order they appear in the group state */
    std::vector<std::string>                    joint_model_name_vector_;
    
    /** \brief Joint instances in the order they appear in the group state */
    std::vector<const JointModel*>              joint_model_vector_;
    
    /** \brief A map from joint names to their instances */
    std::map<std::string, const JointModel*>    joint_model_map_;
    
    /** \brief The list of joint models that are roots in this group */
    std::vector<const JointModel*>              joint_roots_;
    
    /** \brief The group includes all the joint variables that make up the joints the group consists of.
        This map gives the position in the state vector of the group for each of these variables.
        Additionaly, it includes the names of the joints and the index for the first variable of that joint. */
    std::map<std::string, unsigned int>         joint_variables_index_map_;
    
    /** \brief The joints that have no DOF (fixed) */
    std::vector<const JointModel*>              fixed_joints_;
    
    /** \brief Joints that mimic other joints */
    std::vector<const JointModel*>              mimic_joints_;
    
    /** \brief The names of the DOF that make up this group (this is just a sequence of joint variable names; not necessarily joint names!) */
    std::vector<std::string>                    active_dof_names_;
    
    /** \brief The links that are on the direct lineage between joints
        and joint_roots_, as well as the children of the joint leafs.
        May not be in any particular order */
    std::vector<const LinkModel*>               link_model_vector_;
    
    /** \brief The names of the links in this group */
    std::vector<std::string>                    link_model_name_vector_;
    
    /** \brief The list of downstream link models in the order they should be updated (may include links that are not in this group) */
    std::vector<const LinkModel*>               updated_link_model_vector_;
    
    /** \brief The list of downstream link names in the order they should be updated (may include links that are not in this group) */
    std::vector<std::string>                    updated_link_model_name_vector_;
    
    /** \brief The number of variables necessary to describe this group of joints */
    unsigned int                                variable_count_;

    /** \brief The set of labelled subgroups that are included in this group */
    std::vector<std::string> subgroup_names_;
    
    /** \brief The set of default states specified for this group in the SRDF */
    std::map<std::string, std::map<std::string,
                                   double> >    default_states_;
  };
  
  /** \brief Construct a kinematic model from a parsed description and a list of planning groups */
  KinematicModel(const boost::shared_ptr<const urdf::Model> &urdf_model,
                 const boost::shared_ptr<const srdf::Model> &srdf_model);

  /** \brief Construct a kinematic model from a parsed description and a list of planning groups */
  KinematicModel(const boost::shared_ptr<const urdf::Model> &urdf_model,
                 const boost::shared_ptr<const srdf::Model> &srdf_model,
                 const std::string &root_link);
  
  /** \brief Destructor. Clear all memory. */
  virtual ~KinematicModel(void);
  
  /** \brief General the model name **/
  const std::string& getName(void) const;
  
  /** \brief Get a link by its name */
  const LinkModel* getLinkModel(const std::string &link) const;
  
  /** \brief Check if a link exists */
  bool hasLinkModel(const std::string &name) const;
  
  /** \brief Get a joint by its name */
  const JointModel* getJointModel(const std::string &joint) const;
  
  /** \brief Check if a joint exists */
  bool hasJointModel(const std::string &name) const;
  
  /** \brief Get the set of link models that follow a parent link in the kinematic chain */
  void getChildLinkModels(const LinkModel* parent, std::vector<const LinkModel*> &links) const;
  
  /** \brief Get the set of link models that follow a parent joint in the kinematic chain */
  void getChildLinkModels(const JointModel* parent, std::vector<const LinkModel*> &links) const;
  
  /** \brief Get the set of joint models that follow a parent link in the kinematic chain */
  void getChildJointModels(const LinkModel* parent, std::vector<const JointModel*> &links) const;
  
  /** \brief Get the set of joint models that follow a parent joint in the kinematic chain */
  void getChildJointModels(const JointModel* parent, std::vector<const JointModel*> &links) const;
  
  /** \brief Get the set of link names that follow a parent link in the kinematic chain */
  std::vector<std::string> getChildLinkModelNames(const LinkModel* parent) const;
  
  /** \brief Get the set of joint names that follow a parent link in the kinematic chain */
  std::vector<std::string> getChildJointModelNames(const LinkModel* parent) const;
  
  /** \brief Get the set of joint names that follow a parent joint in the kinematic chain */
  std::vector<std::string> getChildJointModelNames(const JointModel* parent) const;
  
  /** \brief Get the set of link names that follow a parent link in the kinematic chain */
  std::vector<std::string> getChildLinkModelNames(const JointModel* parent) const;
  
  /** \brief Get the array of joints, in the order they appear
      in the robot state. */
  const std::vector<JointModel*>& getJointModels(void) const
  {
    return joint_model_vector_;
  }
  
  /** \brief Get the array of joint names, in the order they appear in the robot state. */
  const std::vector<std::string>& getJointModelNames(void) const
  {
    return joint_model_names_vector_;
  }
  
  /** \brief Get the array of joints, in the order they should be
      updated*/
  const std::vector<LinkModel*>& getLinkModels(void) const
  {
    return link_model_vector_;
  }
  
  /** \brief Get the link models that have some collision geometry associated to themselves */
  const std::vector<LinkModel*>& getLinkModelsWithCollisionGeometry(void) const
  {
    return link_models_with_collision_geometry_vector_;
  }
  
  /** \brief Get the names of the link models that have some collision geometry associated to themselves */
  const std::vector<std::string>& getLinkModelNamesWithCollisionGeometry(void) const
  {
    return link_model_names_with_collision_geometry_vector_;
  }
  
  /** \brief Get the link names (of all links) */
  const std::vector<std::string>& getLinkModelNames(void) const
  {
    return link_model_names_vector_;
  }
  
  /** \brief Get the root joint. There will always be one root
      joint. This is either extracted from the SRDF, or a fixed
      joint is assumed, if no specification is given.  */
  const JointModel* getRoot(void) const;
  
  const std::string& getRootJointName(void) const
  {
    return getRoot()->getName();
  }
  
  /** \brief Get the physical root link of the robot. */
  
  const LinkModel* getRootLink(void) const
  {
    return root_link_;
  }
  
  const std::string& getRootLinkName(void) const
  {
    return getRootLink()->getName();
  }
  
  /** \brief Get the frame in which the transforms for this
      model are computed (when using a planning_models::KinematicState). This frame depends on
      the root joint. As such, the frame is either extracted from SRDF, or it is assumed to be the name of the root
      link (immediate descendant of the root joint) */
  const std::string& getModelFrame(void) const
  {
    return model_frame_;
  }
  
  /** \brief Compute the random values for a KinematicState */
  void getRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values) const;
  
  /** \brief Compute the default values for a KinematicState */
  void getDefaultValues(std::vector<double> &values) const;
  
  /** \brief Print information about the constructed model */
  void printModelInfo(std::ostream &out = std::cout) const;
  
  /** \brief Check if the JointModelGroup \e group exists */
  bool hasJointModelGroup(const std::string& group) const;
  
  /** \brief Construct a JointModelGroup given a SRDF description \e group */
  bool addJointModelGroup(const srdf::Model::Group& group);
  
  /** \brief Remove a group from this model */
  void removeJointModelGroup(const std::string& group);
  
  /** \brief Get a joint group from this model (by name) */
  const JointModelGroup* getJointModelGroup(const std::string& name) const;
  
  /** \brief Get the map between joint group names and the groups */
  const std::map<std::string, JointModelGroup*>& getJointModelGroupMap(void) const
  {
    return joint_model_group_map_;
  }
  
  /** \brief Get the map between joint group names and the SRDF group object */
  const std::map<std::string, srdf::Model::Group>& getJointModelGroupConfigMap(void) const
  {
    return joint_model_group_config_map_;
  }
  
  /** \brief Get the names of all groups that are defined for this model */
  const std::vector<std::string>& getJointModelGroupNames(void) const;
  
  /** \brief Get the number of variables that describe this model */
  unsigned int getVariableCount(void) const
  {
    return variable_count_;
  }
  
  /** \brief Get the names of the variables that make up the joints that form this state. Only active joints (not
      fixed, not mimic) are included. Effectively, these are the names of the DOF for this group. The number of
      returned elements is always equal to getVariableCount() */
  const std::vector<std::string>& getActiveDOFNames(void) const
  {
    return active_dof_names_;
  }
  
  /** \brief Get bounds for all the variables in this model. Bounds are returned as a std::pair<lower,upper> */
  const std::map<std::string, std::pair<double, double> >& getAllVariableBounds(void) const
  {
    return variable_bounds_;
  }
  
  /** \brief Get the joint variables index map.
      The state includes all the joint variables that make up the joints the state consists of.
      This map gives the position in the state vector of the group for each of these variables.
      Additionaly, it includes the names of the joints and the index for the first variable of that joint.*/
  const std::map<std::string, unsigned int>& getJointVariablesIndexMap(void) const
  {
    return joint_variables_index_map_;
  }
  
protected:
  
  /** \brief The name of the model */
  std::string                               model_name_;
  
  /** \brief The reference frame for this model */
  std::string                               model_frame_;
  
  /** \brief A map from link names to their instances */
  std::map<std::string, LinkModel*>         link_model_map_;
  
  /** \brief The vector of links that are updated when computeTransforms() is called, in the order they are updated */
  std::vector<LinkModel*>                   link_model_vector_;
  
  /** \brief The vector of link names that corresponds to link_model_vector_ */
  std::vector<std::string>                  link_model_names_vector_;
  
  /** \brief Only links that have collision geometry specified */
  std::vector<LinkModel*>                   link_models_with_collision_geometry_vector_;
  
  /** \brief The vector of link names that corresponds to link_models_with_collision_geometry_vector_ */
  std::vector<std::string>                  link_model_names_with_collision_geometry_vector_;
  
  /** \brief A map from joint names to their instances */
  std::map<std::string, JointModel*>        joint_model_map_;
  
  /** \brief The vector of joints in the model, in the order they appear in the state vector */
  std::vector<JointModel*>                  joint_model_vector_;
  
  /** \brief The vector of joint names that corresponds to joint_model_vector_ */
  std::vector<std::string>                  joint_model_names_vector_;
  
  /** \brief The names of the DOF that make up this state (this is just a sequence of joint variable names; not necessarily joint names!) */
  std::vector<std::string>                  active_dof_names_;
  
  /** \brief Get the number of variables necessary to describe this model */
  unsigned int                              variable_count_;
  
  /** \brief The bounds for all the variables that make up the joints in this model */
  std::map<std::string,
           std::pair<double, double> >      variable_bounds_;
  
  /** \brief The state includes all the joint variables that make up the joints the state consists of.
      This map gives the position in the state vector of the group for each of these variables.
      Additionaly, it includes the names of the joints and the index for the first variable of that joint. */
  std::map<std::string, unsigned int>       joint_variables_index_map_;
  
  /** \brief The root joint */
  JointModel                               *root_joint_;
  
  /** \brief The first physical link for the robot */
  LinkModel                                *root_link_;
  
  /** \brief A map from group names to joint groups */
  std::map<std::string, JointModelGroup*>   joint_model_group_map_;
  
  /** \brief A vector of all group names */
  std::vector<std::string>                  joint_model_group_names_;
  
  /** \brief A vector of all group names */
  std::map<std::string, srdf::Model::Group> joint_model_group_config_map_;
  
  /** \brief Given an URDF model and a SRDF model, build a full kinematic model */
  void buildModel(const boost::shared_ptr<const urdf::Model> &urdf_model,
                  const boost::shared_ptr<const srdf::Model> &srdf_model,
                  const std::string &root_link);
  
  /** \brief Given a SRDF model describing the groups, build up the groups in this kinematic model */
  void buildGroups(const std::vector<srdf::Model::Group> &group_config);
  
  /** \brief Given the URDF model, build up the mimic joints (mutually constrained joints) */
  void buildMimic(const boost::shared_ptr<const urdf::Model> &urdf_model);
  
  /** \brief (This function is mostly intended for internal use). Given a parent link, build up (recursively), the kinematic model by walking  down the tree*/
  JointModel* buildRecursive(LinkModel *parent, const urdf::Link *link,
                             const std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> > &parent_map,
                             const std::map<const urdf::Link*, std::vector<const urdf::Link*> > &child_map,
                             const std::vector<srdf::Model::VirtualJoint> &vjoints);
  
  /** \brief Given a urdf joint model, a child link and a set of virtual joints,
      build up the corresponding JointModel object*/
  JointModel* constructJointModel(const urdf::Joint *urdf_joint_model, const urdf::Link *child_link, const std::vector<srdf::Model::VirtualJoint> &vjoints);
  
  /** \brief Given a urdf link, build the corresponding LinkModel object*/
  LinkModel* constructLinkModel(const urdf::Link *urdf_link);
  
  /** \brief Given a geometry spec from the URDF and a filename (for a mesh), construct the corresponding shape object*/
  shapes::ShapePtr constructShape(const urdf::Geometry *geom, std::string& filename);
};

typedef boost::shared_ptr<KinematicModel> KinematicModelPtr;
typedef boost::shared_ptr<const KinematicModel> KinematicModelConstPtr;
}

#endif
