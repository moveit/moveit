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
  RobotInterfacePython(const std::string& robot_description, const std::string& ns = "")
    : py_bindings_tools::ROScppInitializer()
  {
    robot_model_ = planning_interface::getSharedRobotModel(robot_description);
    if (!robot_model_)
      throw std::runtime_error("RobotInterfacePython: invalid robot model");
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
      for (std::size_t i = 0; i < lim.size(); ++i)
      {
        bp::list l;
        l.append(lim[i].min_position);
        l.append(lim[i].max_position);
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
    bp::list l;
    if (!ensureCurrentState())
      return l;
    robot_state::RobotStatePtr state = current_state_monitor_->getCurrentState();
    const robot_model::LinkModel* lm = state->getLinkModel(name);
    if (lm)
    {
      const Eigen::Affine3d& t = state->getGlobalLinkTransform(lm);
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
    bp::list l;
    if (!ensureCurrentState())
      return l;
    robot_state::RobotStatePtr state = current_state_monitor_->getCurrentState();
    const robot_model::JointModel* jm = state->getJointModel(name);
    if (jm)
    {
      const double* pos = state->getJointPositions(jm);
      const unsigned int sz = jm->getVariableCount();
      for (unsigned int i = 0; i < sz; ++i)
        l.append(pos[i]);
    }

    return l;
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
      ROS_ERROR("Unable to get current robot state");
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

  std::string getCurrentState()
  {
    if (!ensureCurrentState())
      return "";
    robot_state::RobotStatePtr s = current_state_monitor_->getCurrentState();
    moveit_msgs::RobotState msg;
    robot_state::robotStateToRobotStateMsg(*s, msg);
    return py_bindings_tools::serializeMsg(msg);
  }

  bp::tuple getEndEffectorParentGroup(std::string group)
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
    robot_state::RobotStatePtr state;
    if (ensureCurrentState())
    {
      state = current_state_monitor_->getCurrentState();
    }
    else
    {
      state.reset(new robot_state::RobotState(robot_model_));
    }

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
    state->setVariableValues(joint_state);
    visualization_msgs::MarkerArray msg;
    state->getRobotMarkers(msg, py_bindings_tools::stringFromList(links));

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
    if (!ensureCurrentState())
      return "";
    robot_state::RobotStatePtr s = current_state_monitor_->getCurrentState();
    visualization_msgs::MarkerArray msg;
    s->getRobotMarkers(msg, s->getRobotModel()->getLinkModelNames());

    return py_bindings_tools::serializeMsg(msg);
  }

  std::string getRobotMarkersPythonList(bp::list links)
  {
    if (!ensureCurrentState())
      return "";
    robot_state::RobotStatePtr s = current_state_monitor_->getCurrentState();
    visualization_msgs::MarkerArray msg;
    s->getRobotMarkers(msg, py_bindings_tools::stringFromList(links));

    return py_bindings_tools::serializeMsg(msg);
  }

  std::string getRobotMarkersGroup(std::string group)
  {
    if (!ensureCurrentState())
      return "";
    robot_state::RobotStatePtr s = current_state_monitor_->getCurrentState();
    const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    visualization_msgs::MarkerArray msg;
    if (jmg)
    {
      s->getRobotMarkers(msg, jmg->getLinkModelNames());
    }

    return py_bindings_tools::serializeMsg(msg);
  }

  std::string getRobotMarkersGroupPythonDict(std::string group, bp::dict& values)
  {
    const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (!jmg)
      return "";
    bp::list links = py_bindings_tools::listFromString(jmg->getLinkModelNames());
    return getRobotMarkersPythonDictList(values, links);
  }

  bp::dict getCurrentVariableValues()
  {
    bp::dict d;

    if (!ensureCurrentState())
      return d;

    const std::map<std::string, double>& vars = current_state_monitor_->getCurrentStateValues();
    for (std::map<std::string, double>::const_iterator it = vars.begin(); it != vars.end(); ++it)
      d[it->first] = it->second;

    return d;
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
  planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;
  ros::NodeHandle nh_;
};
}

static void wrap_robot_interface()
{
  using namespace moveit;

  bp::class_<RobotInterfacePython> RobotClass("RobotInterface", bp::init<std::string, bp::optional<std::string>>());

  RobotClass.def("get_joint_names", &RobotInterfacePython::getJointNames);
  RobotClass.def("get_group_joint_names", &RobotInterfacePython::getGroupJointNames);
  RobotClass.def("get_group_default_states", &RobotInterfacePython::getDefaultStateNames);
  RobotClass.def("get_group_joint_tips", &RobotInterfacePython::getGroupJointTips);
  RobotClass.def("get_group_names", &RobotInterfacePython::getGroupNames);
  RobotClass.def("get_link_names", &RobotInterfacePython::getLinkNames);
  RobotClass.def("get_group_link_names", &RobotInterfacePython::getGroupLinkNames);
  RobotClass.def("get_joint_limits", &RobotInterfacePython::getJointLimits);
  RobotClass.def("get_link_pose", &RobotInterfacePython::getLinkPose);
  RobotClass.def("get_planning_frame", &RobotInterfacePython::getPlanningFrame);
  RobotClass.def("get_current_state", &RobotInterfacePython::getCurrentState);
  RobotClass.def("get_current_variable_values", &RobotInterfacePython::getCurrentVariableValues);
  RobotClass.def("get_current_joint_values", &RobotInterfacePython::getCurrentJointValues);
  RobotClass.def("get_joint_values", &RobotInterfacePython::getJointValues);
  RobotClass.def("get_robot_root_link", &RobotInterfacePython::getRobotRootLink);
  RobotClass.def("has_group", &RobotInterfacePython::hasGroup);
  RobotClass.def("get_robot_name", &RobotInterfacePython::getRobotName);
  RobotClass.def("get_robot_markers", &RobotInterfacePython::getRobotMarkers);
  RobotClass.def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonList);
  RobotClass.def("get_robot_markers", &RobotInterfacePython::getRobotMarkersFromMsg);
  RobotClass.def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonDictList);
  RobotClass.def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonDict);
  RobotClass.def("get_group_markers", &RobotInterfacePython::getRobotMarkersGroup);
  RobotClass.def("get_group_markers", &RobotInterfacePython::getRobotMarkersGroupPythonDict);
  RobotClass.def("get_parent_group", &RobotInterfacePython::getEndEffectorParentGroup);
}

BOOST_PYTHON_MODULE(_moveit_robot_interface)
{
  wrap_robot_interface();
}

/** @endcond */
