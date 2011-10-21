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

/** \author Ioan Sucan, E. Gil Jones */

#ifndef PLANNING_MODELS_KINEMATIC_STATE_
#define PLANNING_MODELS_KINEMATIC_STATE_

#include "planning_models/kinematic_model.h"

/** \brief Main namespace */
namespace planning_models
{
    
    /** \brief Definition of a kinematic state - the parts of the robot
	state which can change. Const members are thread safe */
    class KinematicState 
    {
	
    public:
	
	/** \brief Forward definition of a joint state */;
	class JointState;
	
	/** \brief Forward definition of a link state */
	class LinkState;
	
	/** \brief Forward definition of an attached body */
	class AttachedBody; 
	
	/** \brief Forward definition of a joint group state */
	class JointGroupState; 
	
	class JointState
	{
	    friend class KinematicState;
	public: 
	    
	    /** brief Constructs the joint state from the model */
	    JointState(const KinematicState *state, const KinematicModel::JointModel* jm);
	    ~JointState(void);
	    
	    /** \brief Sets the internal values from a map of joint variable names to actual values */
	    bool setJointStateValues(const std::map<std::string, double>& joint_value_map);

	    /** \brief Sets the internal values from a map of joint
		variable names to actual values. The function also
		fills the missing vector with the variable names that
		were not set. */
	    bool setJointStateValues(const std::map<std::string, double>& joint_value_map,
				     std::vector<std::string>& missing);
	    
	    /** \brief Sets the internal values from the supplied vector, which are assumed to be in the required order */
	    bool setJointStateValues(const std::vector<double>& joint_value_vector);
	    
	    /** \brief Sets the internal values from the supplied
		array, which are assumed to be in the required
		order. This function is intended to be fast, does no
		input checking and should be used carefully. */
	    bool setJointStateValues(const double *joint_value_vector);
	    
	    /** \brief Sets the internal values from the transform */
	    bool setJointStateValues(const btTransform& transform);
	    
	    /** \brief Specifies whether or not all values associated with a joint are defined in the 
		supplied joint value map */
	    bool allJointStateValuesAreDefined(const std::map<std::string, double>& joint_value_map) const;
	    
	    /** \brief Checks if the current joint state values are all within the bounds set in the model */
	    bool satisfiesBounds(void) const;
	    
	    const std::string& getName(void) const 
	    {
		return joint_model_->getName();
	    }

	    const KinematicState* getKinematicState(void) const
	    {
		return kinematic_state_;
	    }
	    
	    unsigned int getDimension(void) const
	    {
		return joint_model_->getVariableCount();
	    }

	    /** \brief Gets the joint state values stored in the required order */
	    const std::vector<double>& getJointStateValues(void) const
	    {
		return joint_state_values_;
	    }
	    
	    /** \brief Gets the required name order for the joint state values */
	    const std::vector<std::string>& getJointStateNameOrder(void) const
	    {
		return joint_state_name_order_;
	    }
	    
	    /** \brief Gets the current variable transform */
	    const btTransform& getVariableTransform(void) const 
	    {
		return variable_transform_;
	    }
	    
	    /** \brief Gets the joint model */
	    const KinematicModel::JointModel* getJointModel(void) const
	    {
		return joint_model_;
	    }
	    
	    const std::map<std::string, unsigned int>& getJointStateIndexMap(void) const
	    {
		return joint_variables_index_map_;
	    }
	    
	private:
	    
	    /** \brief The kinematic state this joint is part of */
	    const KinematicState               *kinematic_state_;
	    
	    /** \brief The joint model this state corresponds to */
	    const KinematicModel::JointModel   *joint_model_;
	    
	    /** \brief Tthe local transform (computed by forward kinematics) */
	    btTransform                         variable_transform_; 
	    
	    /** \brief The joint values given in the order indicated by joint_variables_index_map_ */
	    std::vector<double>                 joint_state_values_;
	    
	    /** \brief This map ensures the same ordering of variable values with respect to variable names */
	    std::map<std::string, unsigned int> joint_variables_index_map_;
	    
	    /** \brief The expected order of variable names for this joint */
	    std::vector<std::string>            joint_state_name_order_;
	};
		struct AttachedBodyProperties
	{
	    /** \brief The geometries of the attached body */
	    boost::shared_ptr<shapes::ShapeVector> shapes_;
	    
	    /** \brief The constant transforms applied to the link (need to be specified by user) */
	    std::vector<btTransform>               attach_trans_;
	    
	    /** \brief The set of links this body is allowed to touch */
	    std::vector<std::string>               touch_links_;
	    
	    /** string id for reference */
	    std::string                            id_;
	};
	
	/** \brief Class defining bodies that can be attached to robot
	    links. This is useful when handling objects picked up by
	    the robot. */
	class AttachedBody
	{
	    friend class KinematicState;	    
	public:
	    
	    /** \brief Construct an attached body for a specified \e link. The name of this body is \e id and it consists of \e shapes that 
		attach to the link by the transforms \e attach_trans. The set of links that are allowed to be touched by this object is specified by \e touch_links. */
	    AttachedBody(const LinkState *link, const std::string &id,
			 const boost::shared_ptr<shapes::ShapeVector> &shapes,
			 const std::vector<btTransform> &attach_trans,
			 const std::vector<std::string> &touch_links);

	    /** \brief Construct an attached body for a specified \e
		link, but share its properties (name, shapes, fixed
		transforms, touched links) with another instance. This
		allows more efficient copying of states. */
	    AttachedBody(const LinkState *link, const boost::shared_ptr<AttachedBodyProperties> &properties);
	    
	    ~AttachedBody(void);

	    const std::string& getName(void) const
	    {
		return properties_->id_;
	    }
	    
	    const std::string& getAttachedLinkName(void) const
	    {
		return parent_link_state_->getName();
	    }

	    const shapes::ShapeVector* getShapes(void) const 
	    {
		return properties_->shapes_.get();
	    }
	    
	    const std::vector<btTransform>& getAttachedBodyFixedTransforms(void) const
	    {
		return properties_->attach_trans_;
	    }
	    
	    const std::vector<std::string>& getTouchLinks() const
	    {
		return properties_->touch_links_;
	    }

	    const std::vector<btTransform>& getGlobalCollisionBodyTransforms(void) const
	    {
		return global_collision_body_transforms_;
	    }

	    /** \brief Recompute global_collision_body_transform */
	    void computeTransform(void);

	private:
	    
	    /** \brief The link that owns this attached body */
	    const LinkState                          *parent_link_state_;
	    boost::shared_ptr<AttachedBodyProperties> properties_;
	    
	    /** \brief The global transforms for these attached bodies (computed by forward kinematics) */
	    std::vector<btTransform>                  global_collision_body_transforms_;	    
	};
	
	class LinkState 
	{
	    friend class KinematicState;
	public:

	    LinkState(const KinematicState *state, const KinematicModel::LinkModel* lm);
	    ~LinkState(void);
	    
	    const std::string& getName(void) const 
	    {
		return link_model_->getName();
	    }

	    const KinematicState* getKinematicState(void) const
	    {
		return kinematic_state_;
	    }

	    void updateGivenGlobalLinkTransform(const btTransform& transform);
	    
	    /** \brief Recompute global_collision_body_transform and global_link_transform */
	    void computeTransform(void);
	    
	    /** \brief Update all attached bodies given set link transforms */
	    void updateAttachedBodies(void);
	    
	    const KinematicModel::LinkModel* getLinkModel(void) const 
	    {
		return link_model_;
	    }
	    
	    const JointState* getParentJointState(void) const
	    {
		return parent_joint_state_;
	    }
	    
	    const LinkState* getParentLinkState(void) const
	    {
		return parent_link_state_;
	    }
	    
	    const std::vector<AttachedBody*>& getAttachedBodyVector(void) const
	    {
		return attached_body_vector_;
	    }
	    
	    const btTransform& getGlobalLinkTransform(void) const 
	    {
		return global_link_transform_;
	    }
	    
	    const btTransform& getGlobalCollisionBodyTransform(void) const
	    {
		return global_collision_body_transform_;
	    }
	    
	    void attachBody(const std::string &id,
			    const boost::shared_ptr<shapes::ShapeVector> &shapes,
			    const std::vector<btTransform> &attach_trans,
			    const std::vector<std::string> &touch_links);
	    
	    void attachBody(const boost::shared_ptr<AttachedBodyProperties> &properties);

	    void clearAttachedBodies(void);
	    
	private:
	    
	    const KinematicState            *kinematic_state_;
	    
	    const KinematicModel::LinkModel *link_model_;
	    
	    const JointState                *parent_joint_state_;
	    
	    const LinkState                 *parent_link_state_;
	    
	    std::vector<AttachedBody*>       attached_body_vector_;
	    
	    /** \brief The global transform this link forwards (computed by forward kinematics) */
	    btTransform                      global_link_transform_;
	    
	    /** \brief The global transform for this link (computed by forward kinematics) */
	    btTransform                      global_collision_body_transform_;    
	};


	class JointStateGroup
	{
	public:
	    
	    JointStateGroup(KinematicState *state, const KinematicModel::JointModelGroup *jmg);
	    ~JointStateGroup(void);
	    
	    const KinematicState* getKinematicState(void) const
	    {
		return kinematic_state_;
	    }

	    const KinematicModel::JointModelGroup* getJointModelGroup(void)
	    {
		return joint_model_group_;
	    }
	    
	    const std::string& getName(void) const 
	    {
		return joint_model_group_->getName();
	    }
	    
	    unsigned int getDimension(void) const
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
	    bool setStateValues(const std::map<std::string, double>& joint_state_map);
	    
	    /** Compute transforms using current joint values */
	    void updateLinkTransforms(void);	
	    
	    /** \brief Check if a joint is part of this group */
	    bool hasJointState(const std::string &joint) const;
	    
	    /** \brief Check if a link is updated by this group */
	    bool updatesLinkState(const std::string& joint) const;
	    
	    /** \brief Get a joint state by its name */
	    JointState* getJointState(const std::string &joint) const;
	    
	    void getGroupStateValues(std::vector<double>& joint_state_values) const;
	    
	    void getGroupStateValues(std::map<std::string, double>& joint_state_values) const;
	    
	    /** \brief Bring the group to a default state. All joints are
		at 0. If 0 is not within the bounds of the joint, the
		middle of the bounds is used. */
	    void setDefaultValues(void);
	    
	    const std::vector<JointState*>& getJointRoots() const 
	    {
		return joint_roots_;
	    }
	    
	    const std::map<std::string, unsigned int>& getJointVariablesIndexMap(void) const
	    {
		return joint_variables_index_map_;
	    }
	    
	    const std::vector<std::string>& getJointNames(void) const
	    {
		return joint_names_;
	    }
	    
	    const std::vector<JointState*>& getJointStateVector(void) const
	    {
		return joint_state_vector_;
	    }
	    
	private:
	    
	    KinematicState                        *kinematic_state_;
	    const KinematicModel::JointModelGroup *joint_model_group_;
	    
	    std::map<std::string, unsigned int>    joint_variables_index_map_;
	    
	    /** \brief Names of joint variables in the order they appear in the group state */
	    std::vector<std::string>               joint_names_;
	    
	    /** \brief Joint instances in the order they appear in the group state */
	    std::vector<JointState*>               joint_state_vector_;
	    
	    /** \brief A map from joint names to their instances */
	    std::map<std::string, JointState*>     joint_state_map_;
	    
	    /** \brief The list of joints that are roots in this group */
	    std::vector<JointState*>               joint_roots_;
	    
	    /** \brief The list of links that are updated when computeTransforms() is called, in the order they are updated */
	    std::vector<LinkState*>                updated_links_;	    
	};
	
	KinematicState(const KinematicModelPtr &kinematic_model);
	
	KinematicState(const KinematicState& state);
	
	~KinematicState(void);
	
	bool setStateValues(const std::vector<double>& joint_state_values);
	
	bool setStateValues(const std::map<std::string, double>& joint_state_map);
	
	bool setStateValues(const std::map<std::string, double>& joint_state_map,
			    std::vector<std::string>& missing);
	
	void getStateValues(std::vector<double>& joint_state_values) const;
	
	void getStateValues(std::map<std::string, double>& joint_state_values) const;
	
	void updateLinkTransforms(void);
	
	bool updateStateWithLinkAt(const std::string& link_name, const btTransform& transform);
	
	const KinematicModelPtr& getKinematicModel(void) const 
	{
	    return kinematic_model_;
	} 
	
	unsigned int getDimension(void) const
	{
	    return kinematic_model_->getVariableCount();
	}
	
	void setDefaultValues(void);
	
	bool satisfiesBounds(const std::vector<std::string>& joints) const;
	
	bool satisfiesBounds(const std::string& joint) const;
	
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
	const JointState* getJointState(const std::string &joint) const;

	/** \brief Get a joint state by its name */
	JointState* getJointState(const std::string &joint);

	/** \brief Get a link state by its name */
	const LinkState* getLinkState(const std::string &link) const;

	/** \brief Get a link state by its name */
	LinkState* getLinkState(const std::string &link);
	
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
	
	const std::map<std::string, JointStateGroup*>& getJointStateGroupMap(void) const
	{
	    return joint_state_group_map_;
	}
	
	void getJointStateGroupNames(std::vector<std::string>& names) const;
	
	const std::map<std::string, unsigned int> getKinematicStateIndeMxap(void) const
	{
	    return joint_variables_index_map_;
	}
	
	/** \brief Print information about the constructed model */
	void printStateInfo(std::ostream &out = std::cout) const;
	
	/** \brief Print the pose of every link */
	void printTransforms(std::ostream &out = std::cout) const;
	
	void printTransform(const std::string &st, const btTransform &t, std::ostream &out = std::cout) const;
	
	/** \brief Get the global transform applied to the entire tree of links */
	const btTransform& getRootTransform(void) const;
	
	/** \brief Set the global transform applied to the entire tree of links */
	void setRootTransform(const btTransform &transform);
	
    private:
	
	void buildState(void);	

	KinematicModelPtr                       kinematic_model_;
	
	std::map<std::string, unsigned int>     joint_variables_index_map_;
	
	std::vector<JointState*>                joint_state_vector_;
	std::map<std::string, JointState*>      joint_state_map_;
	
	std::vector<LinkState*>                 link_state_vector_;
	std::map<std::string, LinkState*>       link_state_map_;
	
	/** \brief Additional transform to be applied to the tree of links */
	btTransform                             root_transform_;

	std::map<std::string, JointStateGroup*> joint_state_group_map_;
    };
    
    typedef boost::shared_ptr<KinematicState> KinematicStatePtr;
}

#endif
