/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Peter Mitrano
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
 *   * The name of Peter Mitrano may not be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
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

/* Author: Peter Mitrano */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <moveit/robot_model/robot_model.h>
#include <urdf_model/model.h>
#include <moveit/py_bindings_tools/ros_msg_typecasters.h>

namespace py = pybind11;
using namespace robot_model;

void def_robot_model_bindings(py::module& m)
{
  m.doc() = "Definition of a kinematic model. Not thread safe, however multiple instances can be created.";
  py::class_<JointModelGroup, JointModelGroupPtr>(m, "JointModelGroup")
      .def("get_link_model_names", &JointModelGroup::getLinkModelNames)
      .def("get_joint_model_names", &JointModelGroup::getJointModelNames)
      .def("get_active_joint_model_names", &JointModelGroup::getActiveJointModelNames)
      .def("get_default_state_names", &JointModelGroup::getDefaultStateNames)
      .def("get_end_effector_tips",
           [](const JointModelGroup& self) {
             std::vector<std::string> tips;
             self.getEndEffectorTips(tips);
             return tips;
           })
      // keep semicolon on next line
      ;

  py::class_<RobotModel, RobotModelPtr>(m, "RobotModel")
      .def(py::init<const urdf::ModelInterfaceSharedPtr&, const srdf::ModelConstSharedPtr&>(), py::arg("urdf_model"),
           py::arg("srdf_model"))
      .def("get_name", &RobotModel::getName)
      .def("get_model_frame", &RobotModel::getModelFrame)
      .def("get_active_joint_models_bounds", &RobotModel::getActiveJointModelsBounds, py::return_value_policy::reference)
      .def("get_joint_model_group_names", &RobotModel::getJointModelGroupNames)
      .def("get_joint_model_group", py::overload_cast<const std::string&>(&RobotModel::getJointModelGroup, py::const_),
           py::return_value_policy::reference_internal, py::arg("group"))
      .def("get_maximum_extent", py::overload_cast<>(&RobotModel::getMaximumExtent, py::const_))
      .def("get_maximum_extent", py::overload_cast<const JointBoundsVector&>(&RobotModel::getMaximumExtent, py::const_))
      .def("get_variable_bounds", &RobotModel::getVariableBounds)
      .def("get_variable_count", &RobotModel::getVariableCount)
      .def("get_variable_default_positions",
           py::overload_cast<std::map<std::string, double>&>(&RobotModel::getVariableDefaultPositions, py::const_))
      .def("get_variable_default_positions",
           py::overload_cast<std::vector<double>&>(&RobotModel::getVariableDefaultPositions, py::const_))
      .def("get_variable_index", &RobotModel::getVariableIndex)
      .def("get_variable_names", &RobotModel::getVariableNames)
      .def("get_root_joint", &RobotModel::getRootJoint, py::return_value_policy::reference)
      .def("get_root_joint_name", &RobotModel::getRootJointName)
      .def("get_root_link", &RobotModel::getRootLink, py::return_value_policy::reference)
      .def("get_link_model_names", &RobotModel::getLinkModelNames)
      .def("get_joint_model", py::overload_cast<std::string const&>(&RobotModel::getJointModel, py::const_),
           py::return_value_policy::reference)
      .def("get_joint_model_names", &RobotModel::getJointModelNames)
      .def("get_active_joint_model_names", &RobotModel::getActiveJointModelNames)
      .def("get_joint_of_variable", py::overload_cast<int>(&RobotModel::getJointOfVariable, py::const_))
      .def("get_joint_of_variable", py::overload_cast<const std::string&>(&RobotModel::getJointOfVariable, py::const_))
      .def("get_joint_model_count", &RobotModel::getJointModelCount)
      .def("get_planning_frame", &RobotModel::getModelFrame)
      .def("get_robot_root_link", &RobotModel::getRootLinkName)
      .def("has_group", &RobotModel::hasJointModelGroup, py::arg("group"))
      // keep semicolon on next line
      ;

  py::class_<JointModelGroup, JointModelGroupPtr>(m, "JointModelGroup")
      .def("can_set_state_from_ik", &JointModelGroup::canSetStateFromIK)
      .def("get_active_joint_model_names", &JointModelGroup::getActiveJointModelNames)
      .def("get_active_joint_models", &jointModelGroup::getActiveJointModels, py::return_value_policy::reference)
      .def("get_active_joint_models_bounds", &JointModelGroup::getActiveJointModelsBounds,
           py::return_value_policy::reference)
      .def("get_attached_end_effector_names", &JointModelGroup::getAttachedEndEffectorNames)
      .def("get_common_root", &JointModelGroup::getCommonRoot, py::return_value_policy::reference)
      .def("get_continuous_joint_models", &JointModelGroup::getContinuousJointModels, py::return_value_policy::reference)
      .def("get_default_ik_timeout", &JointModelGroup::getDefaultIKTimeout)
      .def("get_default_state_names", &JointModelGroup::getDefaultStateNames)
      .def("get_end_effector_name", &JointModelGroup::getEndEffectorName)
      .def("get_end_effector_parent_group", &JointModelGroup::getEndEffectorParentGroup, py::return_value_policy::reference)
      .def("get_end_effector_tips",
           py::overload_cast<std::vector<const LinkModel*>&>(&JointModelGroup::getEndEffectorTips, py::const_))
      .def("get_end_effector_tips",
           py::overload_cast<std::vector<std::string>&>(&JointModelGroup::getEndEffectorTips, py::const_))
      .def("get_fixed_joint_models", &JointModelGroup::getFixedJointModels, py::return_value_policy::reference)
      .def("get_joint_model", &JointModelGroup::getJointModel, py::return_value_policy::reference)
      .def("get_joint_model_names", &JointModelGroup::getJointModelNames)
      .def("get_joint_models", &JointModelGroup::getJointModels, py::return_value_policy::reference)
      .def("get_joint_roots", &JointModelGroup::getJointRoots, py::return_value_policy::reference)
      .def("get_link_model", &JointModelGroup::getLinkModel, py::return_value_policy::reference)
      .def("get_link_model_names", &JointModelGroup::getLinkModelNames)
      .def("get_link_model_names_with_collision_geometry", &JointModelGroup::getLinkModelNamesWithCollisionGeometry)
      .def("get_link_models", &JointModelGroup::getLinkModels, py::return_value_policy::reference)
      .def("get_maximum_extent", py::overload_cast<>(&JointModelGroup::getMaximumExtent, py::const_))
      .def("get_maximum_extent",
           py::overload_cast<const JointBoundsVector&>(&JointModelGroup::getMaximumExtent, py::const_))
      .def("get_mimic_joint_models", &JointModelGroup::getMimicJointModels, py::return_value_policy::reference)
      .def("get_name", &JointModelGroup::getName)
      .def("get_only_one_end_effector_tip", &JointModelGroup::getOnlyOneEndEffectorTip, py::return_value_policy::reference)
      .def("get_parent_model", &JointModelGroup::getParentModel, py::return_value_policy::reference)
      .def("get_subgroup_names", &JointModelGroup::getSubgroupNames)
      .def("get_subgroups", &_jointModelGroup::getSubgroups, py::return_value_policy::reference)
      .def("get_updated_link_model_names", &JointModelGroup::getUpdatedLinkModelNames)
      .def("get_updated_link_models", &JointModelGroup::getUpdatedLinkModels, py::return_value_policy::reference)
      .def("get_updated_link_models_set", &JointModelGroup::getUpdatedLinkModelsSet, py::return_value_policy::reference)
      .def("get_updated_link_models_with_geometry", &JointModelGroup::getUpdatedLinkModelsWithGeometry,
           py::return_value_policy::reference)
      .def("get_updated_link_models_with_geometry_names", &JointModelGroup::getUpdatedLinkModelsWithGeometryNames)
      .def("get_updated_link_models_with_geometry_names_set", &JointModelGroup::getUpdatedLinkModelsWithGeometryNamesSet)
      .def("get_updated_link_models_with_geometry_set", &JointModelGroup::getUpdatedLinkModelsWithGeometrySet,
           py::return_value_policy::reference)
      .def("get_variable_count", &JointModelGroup::getVariableCount)
      .def("get_variable_default_positions", py::overload_cast<const std::string&, std::map<std::string, double>&>(
                                              &JointModelGroup::getVariableDefaultPositions, py::const_))
      .def("get_variableDefaultPositions",
           py::overload_cast<double*>(&_joint_modelGroup::getVariableDefaultPositions, py::const_))
      .def("get_variable_defaultPositions",
           py::overload_cast<std::map<std::string, double>&>(&_jointModelGroup::getVariableDefaultPositions, py::const_))
      .def("get_variable_defaultPositions",
           py::overload_cast<std::vector<double>&>(&_jointModelGroup::getVariableDefaultPositions, py::const_))
      .def("get_variable_group_index", &JointModelGroup::getVariableGroupIndex)
      .def("get_variable_index_list", &JointModelGroup::getVariableIndexList)
      .def("get_variable_names", &JointModelGroup::getVariableNames)
      .def("has_joint_model", &JointModelGroup::hasJointModel)
      .def("has_link_model", &JointModelGroup::hasLinkModel)
      .def("is_chain", &_jointmodelGroup::isChain)
      .def("is_contiguous_within_state", &JointModelGroup::isContiguousWithinState)
      .def("is_end_effector", &JointModelGroup::isEndEffector)
      .def("is_link_updated", &JointModelGroup::isLinkUpdated)
      .def("is_single_dof_joints", &JointModelGroup::isSingleDOFJoints)
      .def("is_subgroup", &_jointModelGroup::isSubgroup)
      .def("print_group_info",
           [](const JointModelGroup& jmg) {
             std::stringstream ss;
             jmg.printGroupInfo(ss);
             return ss.str();
           })
      .def("set_default_ik_timeout", &JointModelGroup::setDefaultIKTimeout)
      .def("set_end_effector_name", &JointModelGroup::setEndEffectorName)
      .def("set_end_effector_parent", &JointModelGroup::setEndEffectorParent)
      .def("set_redundant_joints", &JointModelGroup::setRedundantJoints)
      .def("set_subgroup_names", &JointModelGroup::setSubgroupNames)
      .def("__repr__",
           [](const JointModelGroup& jmg) {
             std::stringstream ss;
             jmg.printGroupInfo(ss);
             return ss.str();
           })
      //
      ;
  py::class_<LinkModel>(m, "LinkModel")
      .def(py::init<std::string>())
      .def("get_name", &LinkModel::getName)
      .def("are_collision_origin_transforms_identity", &LinkModel::areCollisionOriginTransformsIdentity)
      .def("get_centered_bounding_box_offset", &LinkModel::getCenteredBoundingBoxOffset)
      .def("get_collision_origin_transforms",
           [&](LinkModel const& link) {
             std::vector<Eigen::Matrix4d> matrices;
             auto const& transforms = link.getCollisionOriginTransforms();
             std::transform(transforms.cbegin(), transforms.cend(), std::back_inserter(matrices),
                            [&](Eigen::Isometry3d t) { return t.matrix(); });
             return matrices;
           })
      .def("get_first_collision_body_transform_index", &LinkModel::getFirstCollisionBodyTransformIndex)
      .def("get_joint_origin_transform", [&](LinkModel const& link) { link.getJointOriginTransform().matrix(); })
      .def("get_link_index", &LinkModel::getLinkIndex)
      .def("get_shape_extents_at_origin", &LinkModel::getShapeExtentsAtOrigin)
      .def("get_visual_mesh_filename", &LinkModel::getVisualMeshFilename)
      .def("get_visual_mesh_origin", [&](LinkModel const& link) { return link.getVisualMeshOrigin().matrix(); })
      .def("get_visual_mesh_scale", &LinkModel::getVisualMeshScale)
      .def("joint_origin_transform_is_identity", &LinkModel::jointOriginTransformIsIdentity)
      .def("parent_joint_is_fixed", &LinkModel::parentJointIsFixed)
      //
      ;
  py::class_<JointModel>(m, "JointModel")
      .def("getName", &JointModel::getName)
      .def("add_descendant_joint_model", &JointModel::addDescendantJointModel)
      .def("add_descendant_link_model", &JointModel::addDescendantLinkModel)
      .def("get_child_link_model", &JointModel::getChildLinkModel)
      .def("get_descendant_joint_models", &JointModel::getDescendantJointModels, py::return_value_policy::reference)
      .def("get_descendant_link_models", &JointModel::getDescendantLinkModels, py::return_value_policy::reference)
      .def("get_distance_factor", &JointModel::getDistanceFactor)
      .def("get_maximum_extent", py::overload_cast<>(&JointModel::getMaximumExtent, py::const_))
      .def("get_mimic", &jointModel::getMimic)
      .def("get_mimic_factor", &JointModel::getMimicFactor)
      .def("get_mimic_offset", &JointModel::getMimicOffset)
      .def("get_mimic_requests", &JointModel::getMimicRequests)
      .def("get_non_fixed_descendant_joint_models", &JointModel::getNonFixedDescendantJointModels,
           py::return_value_policy::reference)
      .def("get_parent_link_model", &JointModel::getParentLinkModel)
      .def("get_state_space_dimension", &JointModel::getStateSpaceDimension)
      .def("get_type_name", &JointModel::getTypeName)
      .def("is_passive", &JointModel::isPassive)
      .def("set_child_link_model", &JointModel::setChildLinkModel, py::keep_alive<1, 2>())
      .def("set_distance_factor", &JointModel::setDistanceFactor)
      .def("set_mimic", &JointModel::setMimic, py::keep_alive<1, 2>())
      .def("set_parent_link_model", &JointModel::setParentLinkModel, py::keep_alive<1, 2>())
      .def("set_passive", &JointModel::setPassive)
      //
      ;
  py::class_<FloatingJointModel, JointModel>(m, "FloatingJointModel").def(py::init<std::string>())
      //
      ;
  py::class_<PlanarJointModel, JointModel>(m, "PlanarJointModel").def(py::init<std::string>())
      //
      ;
  py::class_<PrismaticJointModel, JointModel>(m, "PrismaticJointModel").def(py::init<std::string>())
      //
      ;
  py::class_<FixedJointModel, JointModel>(m, "FixedJointModel").def(py::init<std::string>())
      //
      ;
  py::class_<RevoluteJointModel, JointModel>(m, "RevoluteJointModel").def(py::init<std::string>())
      //
      ;
  py::class_<VariableBounds, std::shared_ptr<VariableBounds>>(m, "VariableBounds")
      .def(py::init<>())
      .def_readwrite("min_position_", &VariableBounds::min_position_)
      .def_readwrite("max_position_", &VariableBounds::max_position_)
      .def_readwrite("position_bounded_", &VariableBounds::position_bounded_)
      .def_readwrite("min_velocity_", &VariableBounds::min_velocity_)
      .def_readwrite("max_velocity_", &VariableBounds::max_velocity_)
      .def_readwrite("velocity_bounded_", &VariableBounds::velocity_bounded_)
      .def_readwrite("min_acceleration_", &VariableBounds::min_acceleration_)
      .def_readwrite("max_acceleration_", &VariableBounds::max_acceleration_)
      .def_readwrite("acceleration_bounded_", &VariableBounds::acceleration_bounded_)
      //
>>>>>>> 439575af9 (Add more complete API to RobotModel)
      ;
}
