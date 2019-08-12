/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/py_conversions.h>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <moveit_msgs/RobotState.h>
#include <visualization_msgs/MarkerArray.h>

#include <stdexcept>
#include <boost/python.hpp>
#include <Python.h>

/** @cond IGNORE */

namespace bp = boost::python;

namespace moveit
{
class RobotInterfacePython : protected py_bindings_tools::ROScppInitializer
{
public:
  RobotInterfacePython(const std::string& robot_description, const std::string& ns = "",
                       bool monitor_current_state = true)
    : py_bindings_tools::ROScppInitializer()
  {
    robot_model_ = planning_interface::getSharedRobotModel(robot_description);
    if (!robot_model_)
      throw std::runtime_error("RobotInterfacePython: invalid robot model");
    robot_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
    if (monitor_current_state)
      current_state_monitor_ =
          planning_interface::getSharedStateMonitor(robot_model_, planning_interface::getSharedTF(), ns);
  }

  const char* getRobotName() const
  {
    return robot_model_->getName().c_str();
  }

  bp::list getJointNames() const
  {
    return py_bindings_tools::listFromString(robot_model_->getJointModelNames());
  }

  bp::list getGroupJointNames(const std::string& group) const
  {
    const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
      return py_bindings_tools::listFromString(jmg->getJointModelNames());
    else
      return bp::list();
  }

  bp::list getGroupJointTips(const std::string& group) const
  {
    const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
    {
      std::vector<std::string> tips;
      jmg->getEndEffectorTips(tips);
      return py_bindings_tools::listFromString(tips);
    }
    else
      return bp::list();
  }

  bp::list getLinkNames() const
  {
    return py_bindings_tools::listFromString(robot_model_->getLinkModelNames());
  }

  bp::list getGroupLinkNames(const std::string& group) const
  {
    const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
      return py_bindings_tools::listFromString(jmg->getLinkModelNames());
    else
      return bp::list();
  }

  bp::list getGroupNames() const
  {
    return py_bindings_tools::listFromString(robot_model_->getJointModelGroupNames());
  }

  bp::list getJointLimits(const std::string& name) const
  {
    bp::list result;
    const robot_model::JointModel* jm = robot_model_->getJointModel(name);
    if (jm)
    {
      const std::vector<moveit_msgs::JointLimits>& lim = jm->getVariableBoundsMsg();
      for (const moveit_msgs::JointLimits& joint_limit : lim)
      {
        bp::list l;
        l.append(joint_limit.min_position);
        l.append(joint_limit.max_position);
        result.append(l);
      }
    }
    return result;
  }

  const char* getPlanningFrame() const
  {
    return robot_model_->getModelFrame().c_str();
  }

  bp::list getLinkPose(const std::string& name)
  {
    ensureCurrentState();

    bp::list l;

    const robot_model::LinkModel* lm = robot_state_->getLinkModel(name);
    if (lm)
    {
      const Eigen::Isometry3d& t = robot_state_->getGlobalLinkTransform(lm);
      std::vector<double> v(7);
      v[0] = t.translation().x();
      v[1] = t.translation().y();
      v[2] = t.translation().z();
      Eigen::Quaterniond q(t.rotation());
      v[3] = q.x();
      v[4] = q.y();
      v[5] = q.z();
      v[6] = q.w();
      l = py_bindings_tools::listFromDouble(v);
    }
    return l;
  }

  bp::list getDefaultStateNames(const std::string& group)
  {
    bp::list l;
    const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
    {
      for (auto& known_state : jmg->getDefaultStateNames())
      {
        l.append(known_state);
      }
    }
    return l;
  }

  bp::list getCurrentJointValues(const std::string& name)
  {
    ensureCurrentState();

    bp::list l;
    const robot_model::JointModel* jm = robot_state_->getJointModel(name);
    if (jm)
    {
      const double* pos = robot_state_->getJointPositions(jm);
      const unsigned int sz = jm->getVariableCount();
      for (unsigned int i = 0; i < sz; ++i)
        l.append(pos[i]);
    }

    return l;
  }

  void setCurrentJointValues(const std::string& name, bp::list& values)
  {
    ensureNotMonitoring();

    const robot_model::JointModel* jm = robot_state_->getJointModel(name);
    if (!jm)
    {
      ROS_ERROR_STREAM("joint '" << name << "' does not exist");
      return;
    }

    const size_t var_cnt = jm->getVariableCount();
    if (var_cnt != static_cast<size_t>(bp::len(values)))
    {
      ROS_ERROR_STREAM("wrong number of variables passed for joint '" << name << "'");
      return;
    }

    std::vector<double> val_vec(var_cnt);
    for (size_t i = 0; i < var_cnt; ++i)
      val_vec[i] = bp::extract<double>(values[i]);

    robot_state_->setJointPositions(name, val_vec);
  }

  bp::dict getJointValues(const std::string& group, const std::string& named_state)
  {
    const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (!jmg)
      return boost::python::dict();
    std::map<std::string, double> values;
    jmg->getVariableDefaultPositions(named_state, values);
    return py_bindings_tools::dictFromType(values);
  }

  bool ensureCurrentState(double wait = 1.0)
  {
    if (!current_state_monitor_)
    {
      // we do not monitor the current state
      return false;
    }

    // if needed, start the monitor and wait up to 1 second for a full robot state
    if (!current_state_monitor_->isActive())
    {
      current_state_monitor_->startStateMonitor();
      if (!current_state_monitor_->waitForCompleteState(wait))
        ROS_WARN("Joint values for monitored state are requested but the full state is not known");
    }
    return true;
  }

  bool ensureNotMonitoring()
  {
    if (current_state_monitor_)
      ROS_ERROR(
          "RobotInterface monitors current state but was given a custom state. It will be overwritten on next update.");
    return static_cast<bool>(current_state_monitor_);
  }

  std::string getCurrentState()
  {
    ensureCurrentState();

    moveit_msgs::RobotState msg;
    robot_state::robotStateToRobotStateMsg(*robot_state_, msg);
    return py_bindings_tools::serializeMsg(msg);
  }

  bool setCurrentState(const std::string& state_str)
  {
    ensureNotMonitoring();

    moveit_msgs::RobotState msg;
    py_bindings_tools::deserializeMsg(state_str, msg);
    return moveit::core::robotStateMsgToRobotState(msg, *robot_state_);
  }

  bp::tuple getEndEffectorParentGroup(const std::string& group)
  {
    // name of the group that is parent to this end-effector group;
    // Second: the link this in the parent group that this group attaches to
    const robot_state::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (!jmg)
      return boost::python::make_tuple("", "");
    std::pair<std::string, std::string> parent_group = jmg->getEndEffectorParentGroup();
    return boost::python::make_tuple(parent_group.first, parent_group.second);
  }

  std::string getRobotMarkersPythonDictList(bp::dict& values, bp::list& links)
  {
    ensureCurrentState();

    bp::list k = values.keys();
    int l = bp::len(k);
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(l);
    joint_state.position.resize(l);
    for (int i = 0; i < l; ++i)
    {
      joint_state.name[i] = bp::extract<std::string>(k[i]);
      joint_state.position[i] = bp::extract<double>(values[k[i]]);
    }
    robot_state_->setVariableValues(joint_state);
    visualization_msgs::MarkerArray msg;
    robot_state_->getRobotMarkers(msg, py_bindings_tools::stringFromList(links));

    return py_bindings_tools::serializeMsg(msg);
  }

  std::string getRobotMarkersPythonDict(bp::dict& values)
  {
    bp::list links = py_bindings_tools::listFromString(robot_model_->getLinkModelNames());
    return getRobotMarkersPythonDictList(values, links);
  }

  std::string getRobotMarkersFromMsg(const std::string& state_str)
  {
    moveit_msgs::RobotState state_msg;
    robot_state::RobotState state(robot_model_);
    py_bindings_tools::deserializeMsg(state_str, state_msg);
    moveit::core::robotStateMsgToRobotState(state_msg, state);

    visualization_msgs::MarkerArray msg;
    state.getRobotMarkers(msg, state.getRobotModel()->getLinkModelNames());

    return py_bindings_tools::serializeMsg(msg);
  }

  std::string getRobotMarkers()
  {
    ensureCurrentState();

    visualization_msgs::MarkerArray msg;
    robot_state_->getRobotMarkers(msg, robot_state_->getRobotModel()->getLinkModelNames());

    return py_bindings_tools::serializeMsg(msg);
  }

  std::string getRobotMarkersPythonList(const bp::list& links)
  {
    ensureCurrentState();

    visualization_msgs::MarkerArray msg;
    robot_state_->getRobotMarkers(msg, py_bindings_tools::stringFromList(links));

    return py_bindings_tools::serializeMsg(msg);
  }

  std::string getRobotMarkersGroup(const std::string& group)
  {
    ensureCurrentState();

    const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    visualization_msgs::MarkerArray msg;
    if (jmg)
    {
      robot_state_->getRobotMarkers(msg, jmg->getLinkModelNames());
    }

    return py_bindings_tools::serializeMsg(msg);
  }

  std::string getRobotMarkersGroupPythonDict(const std::string& group, bp::dict& values)
  {
    const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (!jmg)
      return "";
    bp::list links = py_bindings_tools::listFromString(jmg->getLinkModelNames());
    return getRobotMarkersPythonDictList(values, links);
  }

  bp::dict getCurrentVariableValues()
  {
    ensureCurrentState();

    bp::dict d;
    const double* pos = robot_state_->getVariablePositions();
    const std::vector<std::string>& names = robot_state_->getVariableNames();
    for (std::size_t i = 0; i < names.size(); ++i)
      d[names[i]] = pos[i];

    return d;
  }

  void setCurrentVariableValues(bp::dict& values)
  {
    ensureNotMonitoring();

    bp::list keys = values.keys();
    int n = bp::len(keys);
    for (int i = 0; i < n; ++i)
      robot_state_->setVariablePosition(bp::extract<std::string>(keys[n]), bp::extract<double>(values[keys[n]]));
  }

  const char* getRobotRootLink() const
  {
    return robot_model_->getRootLinkName().c_str();
  }

  bool hasGroup(const std::string& group) const
  {
    return robot_model_->hasJointModelGroup(group);
  }

private:
  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;
  ros::NodeHandle nh_;
};
}  // namespace moveit

static void wrap_robot_interface()
{
  using namespace moveit;

  bp::class_<RobotInterfacePython> robot_class(
      "RobotInterface", bp::init<std::string, std::string, bool>("", (bp::arg("robot_description"), bp::arg("ns") = "",
                                                                      bp::arg("monitor_current_state") = true)));

  robot_class.def("get_joint_names", &RobotInterfacePython::getJointNames);
  robot_class.def("get_group_joint_names", &RobotInterfacePython::getGroupJointNames);
  robot_class.def("get_group_default_states", &RobotInterfacePython::getDefaultStateNames);
  robot_class.def("get_group_joint_tips", &RobotInterfacePython::getGroupJointTips);
  robot_class.def("get_group_names", &RobotInterfacePython::getGroupNames);
  robot_class.def("get_link_names", &RobotInterfacePython::getLinkNames);
  robot_class.def("get_group_link_names", &RobotInterfacePython::getGroupLinkNames);
  robot_class.def("get_joint_limits", &RobotInterfacePython::getJointLimits);
  robot_class.def("get_link_pose", &RobotInterfacePython::getLinkPose);
  robot_class.def("get_planning_frame", &RobotInterfacePython::getPlanningFrame);
  robot_class.def("get_current_state", &RobotInterfacePython::getCurrentState);
  robot_class.def("set_current_state", &RobotInterfacePython::setCurrentState);
  robot_class.def("get_current_variable_values", &RobotInterfacePython::getCurrentVariableValues);
  robot_class.def("set_current_variable_values", &RobotInterfacePython::setCurrentVariableValues);
  robot_class.def("get_current_joint_values", &RobotInterfacePython::getCurrentJointValues);
  robot_class.def("set_current_joint_values", &RobotInterfacePython::setCurrentJointValues);
  robot_class.def("get_joint_values", &RobotInterfacePython::getJointValues);
  robot_class.def("get_robot_root_link", &RobotInterfacePython::getRobotRootLink);
  robot_class.def("has_group", &RobotInterfacePython::hasGroup);
  robot_class.def("get_robot_name", &RobotInterfacePython::getRobotName);
  robot_class.def("get_robot_markers", &RobotInterfacePython::getRobotMarkers);
  robot_class.def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonList);
  robot_class.def("get_robot_markers", &RobotInterfacePython::getRobotMarkersFromMsg);
  robot_class.def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonDictList);
  robot_class.def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonDict);
  robot_class.def("get_group_markers", &RobotInterfacePython::getRobotMarkersGroup);
  robot_class.def("get_group_markers", &RobotInterfacePython::getRobotMarkersGroupPythonDict);
  robot_class.def("get_parent_group", &RobotInterfacePython::getEndEffectorParentGroup);
}

BOOST_PYTHON_MODULE(_moveit_robot_interface)
{
  wrap_robot_interface();
}

/** @endcond */
