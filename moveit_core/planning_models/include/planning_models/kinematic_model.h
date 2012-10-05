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

/* Author: Ioan Sucan, E. Gil Jones */

#ifndef MOVEIT_PLANNING_MODELS_KINEMATIC_MODEL_
#define MOVEIT_PLANNING_MODELS_KINEMATIC_MODEL_

#include <urdf_interface/model.h>
#include <srdfdom/model.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <random_numbers/random_numbers.h>
#include <kinematics_base/kinematics_base.h>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <set>
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
  

  /** \brief Forward definition of a joint group */
  class JointModelGroup;
  
  /// function type that allocates a kinematics solver for a particular group
  typedef boost::function<kinematics::KinematicsBasePtr(const JointModelGroup*)> SolverAllocatorFn;
  
  /// function type that allocates a kinematics solvers for subgroups of a group
  typedef std::map<const JointModelGroup*, SolverAllocatorFn> SolverAllocatorMapFn;
  
// include the headers defining joints
#include "planning_models/kinematic_model/joint_model-inc.h"
#include "planning_models/kinematic_model/fixed_joint_model-inc.h"
#include "planning_models/kinematic_model/floating_joint_model-inc.h"
#include "planning_models/kinematic_model/planar_joint_model-inc.h"
#include "planning_models/kinematic_model/prismatic_joint_model-inc.h"
#include "planning_models/kinematic_model/revolute_joint_model-inc.h"
  
// include the header defining a link model
#include "planning_models/kinematic_model/link_model-inc.h"

// include the header defining a joint group model
#include "planning_models/kinematic_model/joint_model_group-inc.h"

  /** \brief Construct a kinematic model from a parsed description and a list of planning groups */
  KinematicModel(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
                 const boost::shared_ptr<const srdf::Model> &srdf_model);

  /** \brief Construct a kinematic model from a parsed description and a list of planning groups */
  KinematicModel(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
                 const boost::shared_ptr<const srdf::Model> &srdf_model,
                 const std::string &root_link);
  
  /** \brief Destructor. Clear all memory. */
  virtual ~KinematicModel(void);
  
  /** \brief Get the model name **/
  const std::string& getName(void) const;

  /** \brief Get the parsed URDF model*/
  const boost::shared_ptr<const urdf::ModelInterface>& getURDF(void) const
  {
    return urdf_;
  }
  
  /** \brief Get the parsed SRDF model*/
  const boost::shared_ptr<const srdf::Model>& getSRDF(void) const
  {
    return srdf_;
  }

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
  const std::vector<const JointModel*>& getJointModels(void) const
  {
    return joint_model_vector_const_;
  }

  /** \brief Get the array of joints, in the order they appear
      in the robot state. */
  const std::vector<JointModel*>& getJointModels(void)
  {
    return joint_model_vector_;
  }
  
  /** \brief Get the array of joint names, in the order they appear in the robot state. */
  const std::vector<std::string>& getJointModelNames(void) const
  {
    return joint_model_names_vector_;
  }
  
  /** \brief Get the array of links  */
  const std::vector<const LinkModel*>& getLinkModels(void) const
  {
    return link_model_vector_const_;
  }

  /** \brief Get the array of links  */
  const std::vector<LinkModel*>& getLinkModels(void)
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
  
  /** \brief Get the name of the root link of the robot. */
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

  /** \brief Compute the random values for a KinematicState */
  void getRandomValues(random_numbers::RandomNumberGenerator &rng, std::map<std::string, double> &values) const;
  
  /** \brief Compute the default values for a KinematicState */
  void getDefaultValues(std::vector<double> &values) const;

  /** \brief Compute the default values for a KinematicState */
  void getDefaultValues(std::map<std::string, double> &values) const;
  
  /** \brief Print information about the constructed model */
  void printModelInfo(std::ostream &out = std::cout) const;
  
  /** \brief Check if the JointModelGroup \e group exists */
  bool hasJointModelGroup(const std::string& group) const;
    
  /** \brief Get a joint group from this model (by name) */
  const JointModelGroup* getJointModelGroup(const std::string& name) const;

  /** \brief Get a joint group from this model (by name) */
  JointModelGroup* getJointModelGroup(const std::string& name);
  
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

  /// A map of known kinematics solvers (associated to their group name)
  void setKinematicsAllocators(const std::map<std::string, SolverAllocatorFn> &allocators);

protected:

  typedef std::map<LinkModel*, Eigen::Affine3d, std::less<LinkModel*>, 
                   Eigen::aligned_allocator<std::pair<const LinkModel*, Eigen::Affine3d> > > LinkModelToAffine3dMap;

  void computeFixedTransforms(LinkModel *link, const Eigen::Affine3d &transform, LinkModelToAffine3dMap &associated_transforms);
  
  /** \brief The name of the model */
  std::string                               model_name_;
  
  /** \brief The reference frame for this model */
  std::string                               model_frame_;
  
  /** \brief A map from link names to their instances */
  std::map<std::string, LinkModel*>         link_model_map_;
  
  /** \brief The vector of links that are updated when computeTransforms() is called, in the order they are updated */
  std::vector<LinkModel*>                   link_model_vector_;
  
  /** \brief The vector of links that are updated when computeTransforms() is called, in the order they are updated */
  std::vector<const LinkModel*>             link_model_vector_const_;
  
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

  /** \brief The vector of joints in the model, in the order they appear in the state vector */
  std::vector<const JointModel*>            joint_model_vector_const_;
  
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

  boost::shared_ptr<const srdf::Model>      srdf_;

  boost::shared_ptr<const urdf::ModelInterface> urdf_;

  /** \brief Given an URDF model and a SRDF model, build a full kinematic model */
  void buildModel(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
                  const boost::shared_ptr<const srdf::Model> &srdf_model,
                  const std::string &root_link);
  
  /** \brief Given a SRDF model describing the groups, build up the groups in this kinematic model */
  void buildGroups(const std::vector<srdf::Model::Group> &group_config);
  
  /** \brief Given the URDF model, build up the mimic joints (mutually constrained joints) */
  void buildMimic(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model);

  /** \brief Given a SRDF model describing the groups, build the default states defined in the SRDF */
  void buildGroupStates(const boost::shared_ptr<const srdf::Model> &srdf_model);
  
  /** \brief Compute helpful information about groups (that can be queried later) */
  void buildGroupInfo(const boost::shared_ptr<const srdf::Model> &srdf_model);
  
  /** \brief Compute helpful information about joints */
  void buildJointInfo(void);
  
  /** \brief If constructing a kinematic model that has a parent different than the one specified by the URDF, this function 
      computes the parent and child maps for URDF links */
  void computeTreeStructure(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model, const std::string &root_link,
                            std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> >& parent_map,
                            std::map<const urdf::Link*, std::vector<const urdf::Link*> >& child_map);
  
  /** \brief (This function is mostly intended for internal use). Given a parent link, build up (recursively), the kinematic model by walking  down the tree*/
  JointModel* buildRecursive(LinkModel *parent, const urdf::Link *link,
                             const std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> > &parent_map,
                             const std::map<const urdf::Link*, std::vector<const urdf::Link*> > &child_map,
                             const std::vector<srdf::Model::VirtualJoint> &vjoints);

  /** \brief Construct a JointModelGroup given a SRDF description \e group */
  bool addJointModelGroup(const srdf::Model::Group& group);
  
  /** \brief Given a urdf joint model, a child link and a set of virtual joints,
      build up the corresponding JointModel object*/
  JointModel* constructJointModel(const urdf::Joint *urdf_joint_model, const urdf::Link *child_link, const std::vector<srdf::Model::VirtualJoint> &vjoints);
  
  /** \brief Given a urdf link, build the corresponding LinkModel object*/
  LinkModel* constructLinkModel(const urdf::Link *urdf_link, const std::map<const urdf::Link*, std::pair<const urdf::Link*, const urdf::Joint*> > &parent_map);
  
  /** \brief Given a geometry spec from the URDF and a filename (for a mesh), construct the corresponding shape object*/
  shapes::ShapePtr constructShape(const urdf::Geometry *geom, std::string& filename);
};

typedef boost::shared_ptr<KinematicModel> KinematicModelPtr;
typedef boost::shared_ptr<const KinematicModel> KinematicModelConstPtr;

}

#endif
