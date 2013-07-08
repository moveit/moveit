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

#ifndef MOVEIT_ROBOT_MODEL_ROBOT_MODEL_
#define MOVEIT_ROBOT_MODEL_ROBOT_MODEL_

#include <urdf_model/model.h>
#include <srdfdom/model.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <random_numbers/random_numbers.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/fixed_joint_model.h>
#include <moveit/robot_model/floating_joint_model.h>
#include <moveit/robot_model/planar_joint_model.h>
#include <moveit/robot_model/revolute_joint_model.h>
#include <moveit/robot_model/prismatic_joint_model.h>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <set>

#include <Eigen/Geometry>

#include <console_bridge/console.h>

/** \brief Main namespace for representing robot planning models */
namespace robot_model
{

/** \brief Definition of a kinematic model. This class is not thread
    safe, however multiple instances can be created */
class RobotModel
{
public:

  /** \brief Construct a kinematic model from a parsed description and a list of planning groups */
  RobotModel(const boost::shared_ptr<const urdf::ModelInterface> &urdf_model,
             const boost::shared_ptr<const srdf::Model> &srdf_model);

  /** \brief Destructor. Clear all memory. */
  virtual ~RobotModel();

  /** \brief Get the model name **/
  const std::string& getName() const
  {
    return model_name_;
  }

  /** \brief Get the parsed URDF model*/
  const boost::shared_ptr<const urdf::ModelInterface>& getURDF() const
  {
    return urdf_;
  }

  /** \brief Get the parsed SRDF model*/
  const boost::shared_ptr<const srdf::Model>& getSRDF() const
  {
    return srdf_;
  }

  /** \brief Check if a link exists */
  bool hasLinkModel(const std::string &name) const;

  /** \brief Get a link by its name */
  const LinkModel* getLinkModel(const std::string &link) const;

  /** \brief Check if a joint exists */
  bool hasJointModel(const std::string &name) const;

  /** \brief Get a joint by its name */
  const JointModel* getJointModel(const std::string &joint) const;

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
  const std::vector<const JointModel*>& getJointModels() const
  {
    return joint_model_vector_const_;
  }

  /** \brief Get the array of joints, in the order they appear
      in the robot state. */
  const std::vector<JointModel*>& getJointModels()
  {
    return joint_model_vector_;
  }

  /** \brief Get the array of joint names, in the order they appear in the robot state. */
  const std::vector<std::string>& getJointModelNames() const
  {
    return joint_model_names_vector_;
  }

  /** \brief Get the array of continuous joints, in the order they appear
      in the robot state. */
  const std::vector<const JointModel*>& getContinuousJointModels() const
  {
    return continuous_joint_model_vector_const_;
  }

  /** \brief Get the array of links  */
  const std::vector<const LinkModel*>& getLinkModels() const
  {
    return link_model_vector_const_;
  }

  /** \brief Get the array of links  */
  const std::vector<LinkModel*>& getLinkModels()
  {
    return link_model_vector_;
  }

  /** \brief Get the link names (of all links) */
  const std::vector<std::string>& getLinkModelNames() const
  {
    return link_model_names_vector_;
  }

  /** \brief Get the link models that have some collision geometry associated to themselves */
  const std::vector<LinkModel*>& getLinkModelsWithCollisionGeometry() const
  {
    return link_models_with_collision_geometry_vector_;
  }

  /** \brief Get the names of the link models that have some collision geometry associated to themselves */
  const std::vector<std::string>& getLinkModelNamesWithCollisionGeometry() const
  {
    return link_model_names_with_collision_geometry_vector_;
  }

  /** \brief Get the root joint. There will always be one root
      joint. This is either extracted from the SRDF, or a fixed
      joint is assumed, if no specification is given.  */
  const JointModel* getRoot() const
  {
    return root_joint_;
  }

  const std::string& getRootJointName() const;

  /** \brief Get the physical root link of the robot. */
  const LinkModel* getRootLink() const
  {
    return root_link_;
  }

  /** \brief Get the name of the root link of the robot. */
  const std::string& getRootLinkName() const;

  /** \brief Get the frame in which the transforms for this
      model are computed (when using a planning_models::RobotState *). This frame depends on
      the root joint. As such, the frame is either extracted from SRDF, or it is assumed to be the name of the root
      link (immediate descendant of the root joint) */
  const std::string& getModelFrame() const
  {
    return model_frame_;
  }

  /** \brief Compute the random values for a RobotState */
  void getVariableRandomValues(random_numbers::RandomNumberGenerator &rng, std::vector<double> &values) const;

  /** \brief Compute the random values for a RobotState */
  void getVariableRandomValues(random_numbers::RandomNumberGenerator &rng, std::map<std::string, double> &values) const;

  /** \brief Compute the default values for a RobotState */
  void getVariableDefaultValues(std::vector<double> &values) const;

  /** \brief Compute the default values for a RobotState */
  void getVariableDefaultValues(std::map<std::string, double> &values) const;

  /** \brief Print information about the constructed model */
  void printModelInfo(std::ostream &out = std::cout) const;

  /** \brief Check if the JointModelGroup \e group exists */
  bool hasJointModelGroup(const std::string& group) const;

  /** \brief Get a joint group from this model (by name) */
  const JointModelGroup* getJointModelGroup(const std::string& name) const;

  /** \brief Get a joint group from this model (by name) */
  JointModelGroup* getJointModelGroup(const std::string& name);

  /** \brief Get the map between joint group names and the groups */
  const std::map<std::string, JointModelGroup*>& getJointModelGroupMap() const
  {
    return joint_model_group_map_;
  }

  /** \brief Get the names of all groups that are defined for this model */
  const std::vector<std::string>& getJointModelGroupNames() const
  {
    return joint_model_group_names_;
  }

  /** \brief Get the map between joint group names and the SRDF group object */
  const std::map<std::string, srdf::Model::Group>& getJointModelGroupConfigMap() const
  {
    return joint_model_group_config_map_;
  }

  /** \brief Check if an end effector exists */
  bool hasEndEffector(const std::string& eef) const;

  /** \brief Get the joint group that corresponds to a given end-effector name */
  const JointModelGroup* getEndEffector(const std::string& name) const;

  /** \brief Get the joint group that corresponds to a given end-effector name */
  JointModelGroup* getEndEffector(const std::string& name);

  /** \brief Get the map between end effector names and the groups they correspond to */
  const std::map<std::string, JointModelGroup*>& getEndEffectorsMap() const
  {
    return end_effectors_;
  }

  /** \brief Get the number of variables that describe this model */
  unsigned int getVariableCount() const
  {
    return variable_count_;
  }

  /** \brief Get the names of the variables that make up the joints that form this state. Only active joints (not
      fixed, not mimic) are included. Effectively, these are the names of the DOF for this group. The number of
      returned elements is always equal to getVariableCount() */
  const std::vector<std::string>& getVariableNames() const
  {
    return active_variable_names_;
  }

  /** \brief Get bounds for all the variables in this model. Bounds are returned as a std::pair<lower,upper> */
  const std::map<std::string, std::pair<double, double> >& getAllVariableBounds() const
  {
    return variable_bounds_;
  }

  /** \brief Get the joint variables index map.
      The state includes all the joint variables that make up the joints the state consists of.
      This map gives the position in the state vector of the group for each of these variables.
      Additionaly, it includes the names of the joints and the index for the first variable of that joint.*/
  const std::map<std::string, unsigned int>& getJointVariablesIndexMap() const
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
  std::string                                   model_name_;

  /** \brief The reference frame for this model */
  std::string                                   model_frame_;

  /** \brief A map from link names to their instances */
  std::map<std::string, LinkModel*>             link_model_map_;

  /** \brief The vector of links that are updated when computeTransforms() is called, in the order they are updated */
  std::vector<LinkModel*>                       link_model_vector_;

  /** \brief The vector of links that are updated when computeTransforms() is called, in the order they are updated */
  std::vector<const LinkModel*>                 link_model_vector_const_;

  /** \brief The vector of link names that corresponds to link_model_vector_ */
  std::vector<std::string>                      link_model_names_vector_;

  /** \brief Only links that have collision geometry specified */
  std::vector<LinkModel*>                       link_models_with_collision_geometry_vector_;

  /** \brief The vector of link names that corresponds to link_models_with_collision_geometry_vector_ */
  std::vector<std::string>                      link_model_names_with_collision_geometry_vector_;

  /** \brief A map from joint names to their instances */
  std::map<std::string, JointModel*>            joint_model_map_;

  /** \brief The vector of joints in the model, in the order they appear in the state vector */
  std::vector<JointModel*>                      joint_model_vector_;

  /** \brief The vector of joints in the model, in the order they appear in the state vector */
  std::vector<const JointModel*>                joint_model_vector_const_;

  /** \brief The vector of joint names that corresponds to joint_model_vector_ */
  std::vector<std::string>                      joint_model_names_vector_;

  /** \brief The set of continuous joints this model contains */
  std::vector<const JointModel*>                continuous_joint_model_vector_const_;

  /** \brief The names of the DOF that make up this state (this is just a sequence of joint variable names; not necessarily joint names!) */
  std::vector<std::string>                      active_variable_names_;

  /** \brief Get the number of variables necessary to describe this model */
  unsigned int                                  variable_count_;

  /** \brief The bounds for all the variables that make up the joints in this model */
  std::map<std::string,
           std::pair<double, double> >          variable_bounds_;

  /** \brief The state includes all the joint variables that make up the joints the state consists of.
      This map gives the position in the state vector of the group for each of these variables.
      Additionaly, it includes the names of the joints and the index for the first variable of that joint. */
  std::map<std::string, unsigned int>           joint_variables_index_map_;

  /** \brief The root joint */
  JointModel                                   *root_joint_;

  /** \brief The first physical link for the robot */
  LinkModel                                    *root_link_;

  /** \brief A map from group names to joint groups */
  std::map<std::string, JointModelGroup*>       joint_model_group_map_;

  /** \brief A vector of all group names */
  std::vector<std::string>                      joint_model_group_names_;

  /** \brief A map of all group names */
  std::map<std::string, srdf::Model::Group>     joint_model_group_config_map_;

  /** \brief The known end effectors */
  std::map<std::string, JointModelGroup*>       end_effectors_;

  boost::shared_ptr<const srdf::Model>          srdf_;

  boost::shared_ptr<const urdf::ModelInterface> urdf_;

  /** \brief Given an URDF model and a SRDF model, build a full kinematic model */
  void buildModel(const urdf::ModelInterface &urdf_model, const srdf::Model &srdf_model);

  /** \brief Given a SRDF model describing the groups, build up the groups in this kinematic model */
  void buildGroups(const srdf::Model &srdf_model);

  /** \brief Compute helpful information about groups (that can be queried later) */
  void buildGroupsInfo_Subgroups(const srdf::Model &srdf_model);

  /** \brief Compute helpful information about groups (that can be queried later) */
  void buildGroupsInfo_EndEffectors(const srdf::Model &srdf_model);

  /** \brief Given the URDF model, build up the mimic joints (mutually constrained joints) */
  void buildMimic(const urdf::ModelInterface &urdf_model);

  /** \brief Given a SRDF model describing the groups, build the default states defined in the SRDF */
  void buildGroupStates(const srdf::Model &srdf_model);

  /** \brief Compute helpful information about joints */
  void buildJointInfo();

  /** \brief (This function is mostly intended for internal use). Given a parent link, build up (recursively),
      the kinematic model by walking  down the tree*/
  JointModel* buildRecursive(LinkModel *parent, const urdf::Link *link, const srdf::Model &srdf_model);

  /** \brief Construct a JointModelGroup given a SRDF description \e group */
  bool addJointModelGroup(const srdf::Model::Group& group);

  /** \brief Given a urdf joint model, a child link and a set of virtual joints,
      build up the corresponding JointModel object*/
  JointModel* constructJointModel(const urdf::Joint *urdf_joint_model, const urdf::Link *child_link, const srdf::Model &srdf_model);

  /** \brief Given a urdf link, build the corresponding LinkModel object*/
  LinkModel* constructLinkModel(const urdf::Link *urdf_link);

  /** \brief Given a geometry spec from the URDF and a filename (for a mesh), construct the corresponding shape object*/
  shapes::ShapePtr constructShape(const urdf::Geometry *geom);
};

typedef boost::shared_ptr<RobotModel> RobotModelPtr;
typedef boost::shared_ptr<const RobotModel> RobotModelConstPtr;

}

#endif
