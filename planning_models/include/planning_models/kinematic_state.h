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

/** \author E. Gil Jones, Ioan Sucan */

#ifndef PLANNING_MODELS_KINEMATIC_STATE_
#define PLANNING_MODELS_KINEMATIC_STATE_

#include "kinematic_model.h"

/** \brief Main namespace */
namespace planning_models
{

/** \brief Definition of a kinematic state - the parts of the robot
    state which can change.  It is not thread safe, however multiple 
    instances can be created */
class KinematicState 
{

public:
  
  class JointState; /** \brief Forward definition of a joint state */;
  
  class LinkState; /** \brief Forward definition of a link state */

  class AttachedBodyState; /** \brief Forward definition of an attached body state */

  class JointGroupState; /** \brief Forward definition of a joint group state */

  class JointState
  {
  public: 
    
    /** brief Constructs the joint state from the model */
    JointState(const KinematicModel::JointModel* jm);

    ~JointState()
    {
    }

    /** \brief Sets the internal values from the joint_value_map */
    bool setJointStateValues(const std::map<std::string, double>& joint_value_map);

    /** \brief Sets the internal values from the joint_value_map, returning states that aren't being set */
    bool setJointStateValues(const std::map<std::string, double>& joint_value_map,
                             std::vector<std::string>& missing_states);
    
    /** \brief Sets the internal values from the supplied vector, which are assumed to be in the required order*/
    bool setJointStateValues(const std::vector<double>& joint_value_vector);
    
    /** \brief Sets the internal values from the supplied array, which are assumed to be in the required order*/
    bool setJointStateValues(const double *joint_value_vector);

    /** \brief Sets the internal values from the transform */
    bool setJointStateValues(const btTransform& transform);

    /** \brief Specifies whether or not all values associated with a joint are defined in the 
        supplied joint value map */
    bool allJointStateValuesAreDefined(const std::map<std::string, double>& joint_value_map) const;

    /** \brief returns a pair of the low and high bounds for the given joint value.
        Supplied name should be externally defined name. */
    bool getJointValueBounds(const std::string& value_name, double& low, double& high) const;
    
    /** \brief gets all bounds for a particular joint */
    const std::map<std::string, std::pair<double, double> >& getAllJointValueBounds() const;

    /** \brief Checks if the current joint state values are all within the bounds set in the model */
    bool areJointStateValuesWithinBounds() const;

    const std::string& getName() const 
    {
      return joint_model_->getName();
    }
    
    /** \brief Gets the number of dimensions for a joint */
    unsigned int getDimension() 
    {
      return joint_state_values_.size();
    }

    /** \brief Gets the joint state values stored in the required order */
    const std::vector<double>& getJointStateValues() const;

    /** \brief Gets the required name order for the joint state values */
    const std::vector<std::string>& getJointStateNameOrder() const;

    /** \brief Gets the current variable transform */
    const btTransform& getVariableTransform() const 
    {
      return variable_transform_;
    }
    
    /** \brief Gets the joint model */
    const KinematicModel::JointModel* getJointModel() const
    {
      return joint_model_;
    }

    const std::map<std::string, unsigned int>& getJointStateIndexMap() const
    {
      return joint_state_index_map_;
    }

    const std::string& getParentFrameId() const
    {
      return joint_model_->getParentFrameId();
    }

    const std::string& getChildFrameId() const
    {
      return joint_model_->getChildFrameId();
    }

  private:
    
    const KinematicModel::JointModel* joint_model_;
    
    /** \brief the local transform (computed by forward kinematics) */
    btTransform variable_transform_; 
      
    std::map<std::string, unsigned int> joint_state_index_map_;

    std::vector<std::string> joint_state_name_order_;

    std::vector<double> joint_state_values_;
  };

  class LinkState 
  {
  public:
    LinkState(const KinematicModel::LinkModel* lm);

    ~LinkState();

    const std::string& getName() const 
    {
      return link_model_->getName();
    }

    void setParentJointState(const JointState* js) 
    {
      parent_joint_state_ = js;
    }
    
    void setParentLinkState(const LinkState* ls)
    {
      parent_link_state_ = ls;
    }

    void updateGivenGlobalLinkTransform(const btTransform& transform)
    {
      global_link_transform_ = transform;
      global_collision_body_transform_.mult(global_link_transform_, link_model_->getCollisionOriginTransform());
      updateAttachedBodies();
    }
    
    /** \brief Recompute global_collision_body_transform and global_link_transform */
    void computeTransform(void);
    
    //updates all attached bodies given set link transforms
    void updateAttachedBodies();

    const KinematicModel::LinkModel* getLinkModel() const 
    {
      return link_model_;
    }
    
    const JointState* getParentJointState() const
    {
      return parent_joint_state_;
    }

    const LinkState* getParentLinkState() const
    {
      return parent_link_state_;
    }

    const std::vector<AttachedBodyState*>& getAttachedBodyStateVector() const
    {
      return attached_body_state_vector_;
    }

    const btTransform& getGlobalLinkTransform() const 
    {
      return global_link_transform_;
    }

    const btTransform& getGlobalCollisionBodyTransform() const
    {
      return global_collision_body_transform_;
    }

  private:
    
    const KinematicModel::LinkModel* link_model_;

    const JointState* parent_joint_state_;

    const LinkState* parent_link_state_;

    std::vector<AttachedBodyState*> attached_body_state_vector_;

    /** \brief The global transform this link forwards (computed by forward kinematics) */
    btTransform global_link_transform_;
    
    /** \brief The global transform for this link (computed by forward kinematics) */
    btTransform global_collision_body_transform_;    
  };

  class AttachedBodyState 
  {
  public:
    AttachedBodyState(const KinematicModel::AttachedBodyModel* abm, const LinkState* parent_link_state);
    
    ~AttachedBodyState()
    {
    }

    const std::string& getName() const
    {
      return attached_body_model_->getName();
    }

    const std::string& getAttachedLinkName() const
    {
      return attached_body_model_->getAttachedLinkModel()->getName();
    }
    
    const KinematicModel::AttachedBodyModel* getAttachedBodyModel() const
    {
      return attached_body_model_;
    }

    /** \brief Recompute global_collision_body_transform */
    void computeTransform(void);

    const std::vector<btTransform>& getGlobalCollisionBodyTransforms() const
    {
      return global_collision_body_transforms_;
    }

  private:
    const KinematicModel::AttachedBodyModel* attached_body_model_;

    const LinkState* parent_link_state_;

    /** \brief The global transforms for these attached bodies (computed by forward kinematics) */
    std::vector<btTransform> global_collision_body_transforms_;
    
  };

  class JointStateGroup
  {
  public:
	    
    JointStateGroup(const KinematicModel::JointModelGroup* jmg, const KinematicState* owner);

    //JointGroupState(const JointGroupState* jgs)

    ~JointStateGroup(void)
    {
    }

    const KinematicModel::JointModelGroup* getJointModelGroup() {
      return joint_model_group_;
    }

    const std::string& getName() const 
    {
      return joint_model_group_->getName();
    }

    unsigned int getDimension() const
    {
      return dimension_;
    }

    /** \brief Perform forward kinematics starting at the roots
        within a group. Links that are not in the group are also
        updated, but transforms for joints that are not in the
        group are not recomputed.  */    
    bool setKinematicState(const std::vector<double>& joint_state_values);
    
    /** \brief Perform forward kinematics starting at the roots
        within a group. Links that are not in the group are also
        updated, but transforms for joints that are not in the
        group are not recomputed.  */
    bool setKinematicState(const std::map<std::string, double>& joint_state_map);

    /** compute transforms using current joint values */
    void updateKinematicLinks();	

    /** \brief Check if a joint is part of this group */
    bool hasJointState(const std::string &joint) const;

    /** \brief Check if a link is updated by this group */
    bool updatesLinkState(const std::string& joint) const;

    /** \brief Get a joint state by its name */
    JointState* getJointState(const std::string &joint) const;

    void getKinematicStateValues(std::vector<double>& joint_state_values) const;
    
    void getKinematicStateValues(std::map<std::string, double>& joint_state_values) const;
   
    /** \brief Bring the group to a default state. All joints are
        at 0. If 0 is not within the bounds of the joint, the
        middle of the bounds is used. */
    void setKinematicStateToDefault();

    const std::vector<JointState*>& getJointRoots() const 
    {
      return joint_roots_;
    }

    const std::map<std::string, unsigned int>& getKinematicStateIndexMap() const
    {
      return kinematic_state_index_map_;
    }
    
    const std::vector<std::string>& getJointNames() const
    {
      return joint_names_;
    }

    const std::vector<JointState*>& getJointStateVector() const
    {
      return joint_state_vector_;
    }
	    
  private:
    
    /** \brief The kinematic model that owns the group */
    boost::shared_ptr<KinematicModel> kinematic_model_;	    

    const KinematicModel::JointModelGroup* joint_model_group_;
    
    unsigned int dimension_;
    std::map<std::string, unsigned int> kinematic_state_index_map_;

    /** \brief Names of joints in the order they appear in the group state */
    std::vector<std::string> joint_names_;

    /** \brief Joint instances in the order they appear in the group state */
    std::vector<JointState*> joint_state_vector_;

    /** \brief A map from joint names to their instances */
    std::map<std::string, JointState*> joint_state_map_;

     /** \brief The list of joints that are roots in this group */
    std::vector<JointState*> joint_roots_;

    /** \brief The list of links that are updated when computeTransforms() is called, in the order they are updated */
    std::vector<LinkState*> updated_links_;	    
  };

  KinematicState(const KinematicModel* kinematic_model);
  
  KinematicState(const KinematicState& state);

  ~KinematicState(void);

  bool setKinematicState(const std::vector<double>& joint_state_values);
  
  bool setKinematicState(const std::map<std::string, double>& joint_state_map);

  bool setKinematicState(const std::map<std::string, double>& joint_state_map,
                         std::vector<std::string>& missing_links);

  void getKinematicStateValues(std::vector<double>& joint_state_values) const;

  void getKinematicStateValues(std::map<std::string, double>& joint_state_values) const;

  void updateKinematicLinks();

  bool updateKinematicStateWithLinkAt(const std::string& link_name, const btTransform& transform);

  const KinematicModel* getKinematicModel() const 
  {
    return kinematic_model_;
  } 

  unsigned int getDimension() const {
    return dimension_;
  }
  
  std::vector<LinkState*> getChildLinkStates(const std::string& link_name) const;

  void setKinematicStateToDefault();

  bool areJointsWithinBounds(const std::vector<std::string>& joints) const;

  bool isJointWithinBounds(const std::string& joint) const;

  /** \brief Get a group by its name */
  const JointStateGroup* getJointStateGroup(const std::string &name) const;

  /** \brief Get a group by its name */
  JointStateGroup* getJointStateGroup(const std::string &name);
	
  /** \brief Check if a group exists */
  bool hasJointStateGroup(const std::string &name) const;

  /** \brief Check if a joint is part of this state */
  bool hasJointState(const std::string &joint) const;
  
  /** \brief Check if a link is updated by this state */
  bool hasLinkState(const std::string& joint) const;

  /** \brief Get a joint state by its name */
  JointState* getJointState(const std::string &joint) const;

  /** \brief Get a link state by its name */
  LinkState* getLinkState(const std::string &link) const;

  /** \brief Get an attached body state by its name */
  AttachedBodyState* getAttachedBodyState(const std::string &attached) const;

  const std::vector<JointState*>& getJointStateVector() const 
  {
    return joint_state_vector_;
  }

  std::vector<JointState*>& getJointStateVector()
  {
    return joint_state_vector_;
  }

  const std::vector<LinkState*>& getLinkStateVector() const 
  {
    return link_state_vector_;
  } 
  
  const std::vector<const AttachedBodyState*>& getAttachedBodyStateVector() const
  {
    return attached_body_state_vector_;
  }
  
  const std::map<std::string, JointStateGroup*>& getJointStateGroupMap() const
  {
    return joint_state_group_map_;
  }

  void getJointStateGroupNames(std::vector<std::string>& names) const;
		
  const std::map<std::string, unsigned int> getKinematicStateIndexMap() const
  {
    return kinematic_state_index_map_;
  }

  /** \brief Print information about the constructed model */
  void printStateInfo(std::ostream &out = std::cout) const;

  /** \brief Print the pose of every link */
  void printTransforms(std::ostream &out = std::cout) const;

  void printTransform(const std::string &st, const btTransform &t, std::ostream &out = std::cout) const;

  //  const btTransform& getRootTransform() const;
  
private:

  void setLinkStatesParents();

  const KinematicModel* kinematic_model_;

  unsigned int dimension_;
  std::map<std::string, unsigned int> kinematic_state_index_map_;

  std::vector<JointState*> joint_state_vector_;
  std::map<std::string, JointState*> joint_state_map_;

  std::vector<LinkState*> link_state_vector_;
  std::map<std::string, LinkState*> link_state_map_;

  //vector of bodies, owned by states
  std::vector<const AttachedBodyState*> attached_body_state_vector_;
  
  std::map<std::string, JointStateGroup*> joint_state_group_map_;
};

}

#endif
