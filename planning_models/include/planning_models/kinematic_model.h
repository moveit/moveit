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

/** \author Ioan Sucan */

#ifndef PLANNING_MODELS_KINEMATIC_MODEL_
#define PLANNING_MODELS_KINEMATIC_MODEL_

#include <geometric_shapes/shapes.h>

#include <urdf/model.h>
#include <LinearMath/btTransform.h>
#include <boost/thread/shared_mutex.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <boost/bimap.hpp>

/** \brief Main namespace */
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

  /** \brief Forward definition of an attached body */
  class AttachedBodyModel;
	
  struct MultiDofConfig
  {
    MultiDofConfig(std::string n) : name(n) {
    }
    
    ~MultiDofConfig() {
    }
    
    /** \brief The name of the joint */
    std::string name;
    
    /** \brief The type of multi-dof joint */
    std::string type;
    
    /** \brief The parent frame in which the joint state will be supplied */
    std::string parent_frame_id;

    /** \brief The child frame into which to convert the supplied transform */
    std::string child_frame_id;

    /** \brief The mapping between internally defined DOF names and externally defined DOF names */
    std::map<std::string, std::string> name_equivalents;
  };

  struct GroupConfig
  {
    GroupConfig() { 
    }

    GroupConfig(std::string name, 
                std::string base_link,
                std::string tip_link) 
      : name_(name), base_link_(base_link), tip_link_(tip_link)
    {
    }

    GroupConfig(std::string name,
                std::vector<std::string> joints,
                std::vector<std::string> subgroups) 
      : name_(name)
    {
      joints_ = joints;
      subgroups_ = subgroups;
    }

    std::string name_;
    std::string base_link_;
    std::string tip_link_;
    std::vector<std::string> joints_;
    std::vector<std::string> subgroups_;
  };
  
  /** \brief A joint from the robot. Contains the transform applied by the joint type */
  class JointModel
  {
    friend class KinematicModel;
  public:
    JointModel(const std::string& name);

    JointModel(const JointModel* joint);

    void initialize(const std::vector<std::string>& local_names,
                    const MultiDofConfig* multi_dof_config = NULL);

    virtual ~JointModel(void);

    typedef boost::bimap< std::string, std::string > js_type;

    const std::string& getName() const 
    {
      return name_;
    }

    const LinkModel* getParentLinkModel() const 
    {
      return parent_link_model_;
    }
    
    const LinkModel* getChildLinkModel() const
    {
      return child_link_model_;
    }
    
    const std::string& getParentFrameId() const
    {
      return parent_frame_id_;
    }

    const std::string& getChildFrameId() const
    {
      return child_frame_id_;
    }

    const js_type& getJointStateEquivalents() const
    {
      return joint_state_equivalents_;
    }

    const std::map<unsigned int, std::string>& getComputatationOrderMapIndex() const
    {
      return computation_order_map_index_;
    }
    /** \brief Gets the joint state equivalent for given name */  
    std::string getEquiv(const std::string& name) const;

    /** \brief Gets the lower and upper bounds for a variable */
    bool getVariableBounds(const std::string& variable, std::pair<double, double>& bounds) const;
   
    /** \brief Sets the lower and upper bounds for a variable */
    bool setVariableBounds(const std::string& variable, double low, double high);

    /** \brief Provides a default value for the joint given the joint bounds.
        Most joints will use the default, but the quaternion for floating
        point values needs something else */
    virtual void getVariableDefaultValuesGivenBounds(std::map<std::string, double>& ret_map) const;
    
    virtual bool isValueWithinVariableBounds(const std::string& variable, const double& value, bool& within_bounds) const; 

    const std::map<std::string, std::pair<double, double> >& getAllVariableBounds() const {
      return joint_state_bounds_;
    }

    bool hasVariable(const std::string var) const
    {
      return(joint_state_equivalents_.right.find(var) != joint_state_equivalents_.right.end());
    }
   
    virtual btTransform computeTransform(const std::vector<double>& joint_values) const = 0;
    
    virtual std::vector<double> computeJointStateValues(const btTransform& transform) const = 0;

  private:

    /** \brief Name of the joint */
    std::string name_;
    
    /** \brief The link before this joint */
    LinkModel *parent_link_model_;
    
    /** \brief The link after this joint */
    LinkModel *child_link_model_;

    //local names on the left, config names on the right
    js_type joint_state_equivalents_;

    //map for high and low bounds
    std::map<std::string, std::pair<double, double> > joint_state_bounds_;
    
    //correspondance between index into computation array and external name
    std::map<unsigned int, std::string> computation_order_map_index_;
   
    /** The parent frame id for this joint.  May be empty unless specified as multi-dof*/
    std::string parent_frame_id_;
    
    /** The child frame id for this joint.  May be empty unless specified as multi-dof*/
    std::string child_frame_id_;
  };

  /** \brief A fixed joint */
  class FixedJointModel : public JointModel
  {
  public:
	    
    FixedJointModel(const std::string name, const MultiDofConfig* multi_dof_config) :
      JointModel(name)
    {
      std::vector<std::string> local_names;
      initialize(local_names, multi_dof_config);
    }
	    
    FixedJointModel(const FixedJointModel* joint): JointModel(joint)
    {
    }

    virtual btTransform computeTransform(const std::vector<double>& joint_values) const {
      btTransform ident;
      ident.setIdentity();
      return ident;
    }
    
    virtual std::vector<double> computeJointStateValues(const btTransform& transform) const {
      std::vector<double> ret;
      return ret;
    }

  };

  /** \brief A planar joint */
  class PlanarJointModel : public JointModel
  {
  public:
	    
    PlanarJointModel(const std::string& name, const MultiDofConfig* multi_dof_config);
	    
    PlanarJointModel(const PlanarJointModel* joint): JointModel(joint)
    {
    }

    virtual btTransform computeTransform(const std::vector<double>& joint_values) const;
    
    virtual std::vector<double> computeJointStateValues(const btTransform& transform) const;

  };

  /** \brief A floating joint */
  class FloatingJointModel : public JointModel
  {
  public:
	    
    FloatingJointModel(const std::string& name, const MultiDofConfig* multi_dof_config);

    FloatingJointModel(const FloatingJointModel* joint) : JointModel(joint)
    {
    }

    virtual btTransform computeTransform(const std::vector<double>& joint_values) const;
    
    virtual std::vector<double> computeJointStateValues(const btTransform& transform) const;

    virtual void getVariableDefaultValuesGivenBounds(std::map<std::string, double>& ret_map) const;

  };

  /** \brief A prismatic joint */
  class PrismaticJointModel : public JointModel
  {
  public:
	    
    PrismaticJointModel(const std::string& name, const MultiDofConfig* multi_dof_config);
    
    PrismaticJointModel(const PrismaticJointModel* joint) : JointModel(joint){
      axis_ = joint->axis_;
    }
	    
    btVector3 axis_;
    
    virtual btTransform computeTransform(const std::vector<double>& joint_values) const;
    
    virtual std::vector<double> computeJointStateValues(const btTransform& transform) const;
    
  };
	
  /** \brief A revolute joint */
  class RevoluteJointModel : public JointModel
  {
  public:
	    
    RevoluteJointModel(const std::string& name, const MultiDofConfig* multi_dof_config);

    RevoluteJointModel(const RevoluteJointModel* joint) : JointModel(joint){
      axis_ = joint->axis_;
      continuous_ = joint->continuous_;
    }
	    	    
    btVector3 axis_;
    bool      continuous_;

    virtual btTransform computeTransform(const std::vector<double>& joint_values) const;
    
    virtual std::vector<double> computeJointStateValues(const btTransform& transform) const;
    
    //so we can return true for continuous joints
    virtual bool isValueWithinVariableBounds(const std::string& variable, const double& value, bool& within_bounds) const;
  };
	
  /** \brief A link from the robot. Contains the constant transform applied to the link and its geometry */
  class LinkModel
  {
    friend class KinematicModel;
  public:
    
    LinkModel(const KinematicModel* kinematic_model);
    
    LinkModel(const LinkModel* link_model);

    ~LinkModel(void);

    const std::string& getName() const {
      return name_;
    }
    
    const JointModel* getParentJointModel() const {
      return parent_joint_model_;
    }

    const std::vector<JointModel*>& getChildJointModels() const {
      return child_joint_models_;
    }

    const btTransform& getJointOriginTransform() const {
      return joint_origin_transform_;
    }

    const btTransform& getCollisionOriginTransform() const {
      return collision_origin_transform_;
    }

    const shapes::Shape* getLinkShape() const {
      return shape_;
    }
    
    const std::vector<AttachedBodyModel*>& getAttachedBodyModels() const {
      return attached_body_models_;
    }

  private:
     
    /** \brief Removes all attached body models from this link, requiring an exclusive lock */
    void clearAttachedBodyModels();
    
    /** \brief Removes all attached body models from this link, and replaces them with the supplied vector,
        requiring an exclusive lock
     */
    void replaceAttachedBodyModels(std::vector<AttachedBodyModel*>& attached_body_vector);
   
    void clearLinkAttachedBodyModel(const std::string& att_name);

    void addAttachedBodyModel(AttachedBodyModel* attached_body_model);

    /** \brief Name of the link */
    std::string name_;
    
    /** \brief KinematicState point for accessing locks */
    const KinematicModel* kinematic_model_;

    /** \brief JointModel that connects this link to the parent link */
    JointModel *parent_joint_model_;
    
    /** \brief List of descending joints (each connects to a child link) */
    std::vector<JointModel*> child_joint_models_;
    
    /** \brief The constant transform applied to the link (local) */
    btTransform joint_origin_transform_;
    
    /** \brief The constant transform applied to the collision geometry of the link (local) */
    btTransform  collision_origin_transform_;
    
    /** \brief The geometry of the link */
    shapes::Shape *shape_;
    
    /** \brief Attached bodies */
    std::vector<AttachedBodyModel*> attached_body_models_;	        
  };
  
  /** \brief Class defining bodies that can be attached to robot
      links. This is useful when handling objects picked up by
      the robot. */
  class AttachedBodyModel
  {
  public:
    
    AttachedBodyModel(const LinkModel *link, 
                      const std::string& id,
                      const std::vector<btTransform>& attach_trans,
                      const std::vector<std::string>& touch_links,
                      std::vector<shapes::Shape*>& shapes);

    ~AttachedBodyModel(void);
    
    const std::string& getName() const 
    {
      return id_;
    }
    
    const LinkModel* getAttachedLinkModel() const 
    {
      return attached_link_model_;
    }

    const std::vector<shapes::Shape*>& getShapes() const 
    {
      return shapes_;
    }

    const std::vector<btTransform>& getAttachedBodyFixedTransforms() const
    {
      return attach_trans_;
    }

    const std::vector<std::string>& getTouchLinks() const
    {
      return touch_links_;
    }
    
  private:

    /** \brief The link that owns this attached body */
    const LinkModel *attached_link_model_;
    
    /** \brief The geometries of the attached body */
    std::vector<shapes::Shape*> shapes_;
    
    /** \brief The constant transforms applied to the link (need to be specified by user) */
    std::vector<btTransform> attach_trans_;
    
    /** \brief The set of links this body is allowed to touch */
    std::vector<std::string> touch_links_;
    
    /** string id for reference */
    std::string id_;
  };

  class JointModelGroup
  {
    friend class KinematicModel;
  public:
	    
    JointModelGroup(const std::string& name,
                    const std::vector<const JointModel*>& joint_vector,
                    const std::vector<const JointModel*>& fixed_joint_vector,
                    const KinematicModel* parent_model);
    
    ~JointModelGroup(void);

    const std::string& getName() const
    {
      return name_;
    }
	    
    /** \brief Check if a joint is part of this group */
    bool hasJointModel(const std::string &joint) const;

    /** \brief Get a joint by its name */
    const JointModel* getJointModel(const std::string &joint);

    const std::vector<const JointModel*>& getJointModels() const
    {
      return joint_model_vector_;
    }

    const std::vector<const JointModel*>& getFixedJointModels() const
    {
      return fixed_joint_model_vector_;
    }
    
    const std::vector<std::string>& getJointModelNames() const
    {
      return joint_model_name_vector_;
    }

    const std::vector<const JointModel*>& getJointRoots() const
    {
      return joint_roots_;
    }

    const std::vector<const LinkModel*>& getGroupLinkModels() const
    {
      return group_link_model_vector_;
    }

    bool supportsIK(void) const
    {
	return false;
    }
    
    std::vector<std::string> getGroupLinkNames() const
    {
      std::vector<std::string> ret_vec;
      for(unsigned int i = 0; i < group_link_model_vector_.size(); i++) {
        ret_vec.push_back(group_link_model_vector_[i]->getName());
      }
      return ret_vec;
    }

    const std::vector<const LinkModel*>& getUpdatedLinkModels() const
    {
      return updated_link_model_vector_;
    }

    std::vector<std::string> getUpdatedLinkModelNames() const
    {
      std::vector<std::string> ret_vec;
      for(unsigned int i = 0; i < updated_link_model_vector_.size(); i++) {
        ret_vec.push_back(updated_link_model_vector_[i]->getName());
      }
      return ret_vec;
    }    

  private:

    bool is_valid_;

    /** \brief Name of group */
    std::string name_;

    /** \brief Names of joints in the order they appear in the group state */
    std::vector<std::string> joint_model_name_vector_;

    /** \brief Joint instances in the order they appear in the group state */
    std::vector<const JointModel*> joint_model_vector_;

    /** \brief Fixed joint instances in this group */
    std::vector<const JointModel*> fixed_joint_model_vector_;

    /** \brief A map from joint names to their instances */
    std::map<std::string, const JointModel*> joint_model_map_;

    /** \brief The list of joint models that are roots in this group */
    std::vector<const JointModel*> joint_roots_;

    /** \brief The links that are on the direct lineage between joints
        and joint_roots_, as well as the children of the joint leafs.
        May not be in any particular order */
    std::vector<const LinkModel*> group_link_model_vector_;

    /** \brief The list of downstream link models in the order they should be updated */
    std::vector<const LinkModel*> updated_link_model_vector_;

  };

  /** \brief Construct a kinematic model from another one */
  KinematicModel(const KinematicModel &source);

  /** \brief Construct a kinematic model from a parsed description and a list of planning groups */
  KinematicModel(const urdf::Model &model, 
                 const std::vector<GroupConfig>& group_configs,
                 const std::vector<MultiDofConfig>& multi_dof_configs);
	
  /** \brief Destructor. Clear all memory. */
  ~KinematicModel(void);

  void copyFrom(const KinematicModel &source);

  /** \brief General the model name **/
  const std::string& getName(void) const;

  /** \brief Get a link by its name */
  const LinkModel* getLinkModel(const std::string &link) const;

  /** \brief Check if a link exists */
  bool hasLinkModel(const std::string &name) const;

  /** \brief Get the link names in the order they should be updated */
  void getLinkModelNames(std::vector<std::string> &links) const;

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

  /** \brief Get a joint by its name */
  const JointModel* getJointModel(const std::string &joint) const;

  /** \brief Check if a joint exists */
  bool hasJointModel(const std::string &name) const;
	
  /** \brief Get the array of joints, in the order they appear
      in the robot state. */
  const std::vector<JointModel*>& getJointModels() const
  {
    return joint_model_vector_;
  }
 /** \brief Get the array of joints, in the order they should be
     updated*/
  const std::vector<LinkModel*>& getLinkModels() const
  {
    return link_model_vector_;
  }

  const std::vector<LinkModel*>& getLinkModelsWithCollisionGeometry() const
  {
    return link_models_with_collision_geometry_vector_;
  }
	
  /** \brief Get the array of joint names, in the order they
      appear in the robot state. */
  void getJointModelNames(std::vector<std::string> &joints) const;

  /** \brief Get the root joint */
  const JointModel* getRoot(void) const;
	
  /** \brief Provide interface to get an exclusive lock to change the model. Use carefully! */
  void exclusiveLock(void) const;
	
  /** \brief Provide interface to release an exclusive lock. Use carefully! */
  void exclusiveUnlock(void) const;

  /** \brief Provide interface to get a shared lock for reading model data */
  void sharedLock(void) const;
  
  /** \brief Provide interface to release a shared lock */
  void sharedUnlock(void) const;

  /** \brief Print information about the constructed model */
  void printModelInfo(std::ostream &out = std::cout) const;

  bool hasModelGroup(const std::string& group) const;

  bool addModelGroup(const GroupConfig& group);

  void removeModelGroup(const std::string& group);

  const JointModelGroup* getModelGroup(const std::string& name) const
  {
    if(!hasModelGroup(name)) return NULL;
    return joint_model_group_map_.find(name)->second;
  }

  const std::map<std::string, JointModelGroup*>& getJointModelGroupMap() const
  {
    return joint_model_group_map_;
  }

  const std::map<std::string, GroupConfig>& getJointModelGroupConfigMap() const {
    return joint_model_group_config_map_;
  }

  void getModelGroupNames(std::vector<std::string>& getModelGroupNames) const;

  void clearAllAttachedBodyModels();

  void clearLinkAttachedBodyModels(const std::string& link_name);

  void clearLinkAttachedBodyModel(const std::string& link_name, const std::string& att_name);

  void replaceAttachedBodyModels(const std::string& link_name, std::vector<AttachedBodyModel*>& attached_body_vector);

  void addAttachedBodyModel(const std::string& link_name, AttachedBodyModel* att_body_model);

  std::vector<const AttachedBodyModel*> getAttachedBodyModels() const;

  std::string getRobotName() const {
    return model_name_;
  }
	
private:
	
  /** \brief Shared lock for changing models */ 
  mutable boost::shared_mutex lock_;

  /** \brief The name of the model */
  std::string model_name_;	

  /** \brief A map from link names to their instances */
  std::map<std::string, LinkModel*> link_model_map_;

  /** \brief A map from joint names to their instances */
  std::map<std::string, JointModel*> joint_model_map_;

  /** \brief The vector of joints in the model, in the order they appear in the state vector */
  std::vector<JointModel*> joint_model_vector_;
	
  /** \brief The vector of links that are updated when computeTransforms() is called, in the order they are updated */
  std::vector<LinkModel*> link_model_vector_;	
	
  /** \brief Only links that have collision geometry specified */
  std::vector<LinkModel*> link_models_with_collision_geometry_vector_;

  /** \brief The root joint */
  JointModel *root_;
	
  std::map<std::string, JointModelGroup*> joint_model_group_map_;
  std::map<std::string, GroupConfig> joint_model_group_config_map_;

  void buildGroups(const std::vector<GroupConfig>&);
  JointModel* buildRecursive(LinkModel *parent, const urdf::Link *link, 
                             const std::vector<MultiDofConfig>& multi_dof_configs);
  JointModel* constructJointModel(const urdf::Joint *urdfJointModel,  const urdf::Link *child_link,
                        const std::vector<MultiDofConfig>& multi_dof_configs);
  LinkModel* constructLinkModel(const urdf::Link *urdfLink);
  shapes::Shape* constructShape(const urdf::Geometry *geom);

  JointModel* copyJointModel(const JointModel *joint);
  JointModel* copyRecursive(LinkModel *parent, const LinkModel *link);
	
};

}

#endif
