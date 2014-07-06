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

#include <moveit/move_group_interface/move_group.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/py_conversions.h>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>
#include <Python.h>

/** @cond IGNORE */

namespace bp = boost::python;

namespace moveit
{
namespace planning_interface
{

class MoveGroupWrapper : protected py_bindings_tools::ROScppInitializer,
                         public MoveGroup
{
public:

  // ROSInitializer is constructed first, and ensures ros::init() was called, if needed
  MoveGroupWrapper(const std::string &group_name, const std::string &robot_description) :
    py_bindings_tools::ROScppInitializer(),
    MoveGroup(Options(group_name, robot_description), boost::shared_ptr<tf::Transformer>(), ros::Duration(5, 0))
  {
  }

  bool setJointValueTargetPerJointPythonList(const std::string &joint, bp::list &values)
  {
    return setJointValueTarget(joint, py_bindings_tools::doubleFromList(values));
  }

  bool setJointValueTargetPythonList(bp::list &values)
  {
    return setJointValueTarget(py_bindings_tools::doubleFromList(values));
  }

  bool setJointValueTargetPythonDict(bp::dict &values)
  {
    bp::list k = values.keys();
    int l = bp::len(k);
    std::map<std::string, double> v;
    for (int i = 0; i < l ; ++i)
      v[bp::extract<std::string>(k[i])] = bp::extract<double>(values[k[i]]);
    return setJointValueTarget(v);
  }
  
  bool setJointValueTargetFromPosePython(const std::string &pose_str, const std::string &eef, bool approx)
  {
    geometry_msgs::Pose pose_msg;    
    py_bindings_tools::deserializeMsg(pose_str, pose_msg);
    return approx ? setApproximateJointValueTarget(pose_msg, eef) : setJointValueTarget(pose_msg, eef);
  }

  bool setJointValueTargetFromPoseStampedPython(const std::string &pose_str, const std::string &eef, bool approx)
  {
    geometry_msgs::PoseStamped pose_msg;    
    py_bindings_tools::deserializeMsg(pose_str, pose_msg);
    return approx ? setApproximateJointValueTarget(pose_msg, eef) : setJointValueTarget(pose_msg, eef);
  }

  bool setJointValueTargetFromJointStatePython(const std::string &js_str)
  {
    sensor_msgs::JointState js_msg;
    py_bindings_tools::deserializeMsg(js_str, js_msg);
    return setJointValueTarget(js_msg);
  }
  
  void rememberJointValuesFromPythonList(const std::string &string, bp::list &values)
  {
    rememberJointValues(string, py_bindings_tools::doubleFromList(values));
  }

  const char* getPlanningFrameCStr() const
  {
    return getPlanningFrame().c_str();
  }

  bp::list getActiveJointsList() const
  {
    return py_bindings_tools::listFromString(getActiveJoints());
  }

  bp::list getJointsList() const
  {
    return py_bindings_tools::listFromString(getJoints());
  }

  bp::list getCurrentJointValuesList()
  {
    return py_bindings_tools::listFromDouble(getCurrentJointValues());
  }

  bp::list getRandomJointValuesList()
  {
    return py_bindings_tools::listFromDouble(getRandomJointValues());
  }

  bp::dict getRememberedJointValuesPython() const
  {
    const std::map<std::string, std::vector<double> > &rv = getRememberedJointValues();
    bp::dict d;
    for (std::map<std::string, std::vector<double> >::const_iterator it = rv.begin() ; it != rv.end() ; ++it)
      d[it->first] = py_bindings_tools::listFromDouble(it->second);
    return d;
  }

  bp::list convertPoseToList(const geometry_msgs::Pose &pose) const
  {
    std::vector<double> v(7);
    v[0] = pose.position.x;
    v[1] = pose.position.y;
    v[2] = pose.position.z;
    v[3] = pose.orientation.x;
    v[4] = pose.orientation.y;
    v[5] = pose.orientation.z;
    v[6] = pose.orientation.w;
    return moveit::py_bindings_tools::listFromDouble(v);
  }

  bp::list convertTransformToList(const geometry_msgs::Transform &tr) const
  {
    std::vector<double> v(7);
    v[0] = tr.translation.x;
    v[1] = tr.translation.y;
    v[2] = tr.translation.z;
    v[3] = tr.rotation.x;
    v[4] = tr.rotation.y;
    v[5] = tr.rotation.z;
    v[6] = tr.rotation.w;
    return py_bindings_tools::listFromDouble(v);
  }

  void convertListToTransform(const bp::list &l, geometry_msgs::Transform &tr) const
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(l);
    tr.translation.x =  v[0];
    tr.translation.y = v[1];
    tr.translation.z = v[2];
    tr.rotation.x = v[3];
    tr.rotation.y = v[4];
    tr.rotation.z = v[5];
    tr.rotation.w = v[6];
  }

  void convertListToPose(const bp::list &l, geometry_msgs::Pose &p) const
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(l);
    p.position.x =  v[0];
    p.position.y = v[1];
    p.position.z = v[2];
    p.orientation.x = v[3];
    p.orientation.y = v[4];
    p.orientation.z = v[5];
    p.orientation.w = v[6];
  }

  bp::list getCurrentRPYPython(const std::string &end_effector_link = "")
  {
    return py_bindings_tools::listFromDouble(getCurrentRPY(end_effector_link));
  }

  bp::list getCurrentPosePython(const std::string &end_effector_link = "")
  {
    geometry_msgs::PoseStamped pose = getCurrentPose(end_effector_link);
    return convertPoseToList(pose.pose);
  }
  
  bp::list getRandomPosePython(const std::string &end_effector_link = "")
  {
    geometry_msgs::PoseStamped pose = getRandomPose(end_effector_link);
    return convertPoseToList(pose.pose);
  }

  bp::list getKnownConstraintsList() const
  {
    return py_bindings_tools::listFromString(getKnownConstraints());
  }

  bool placePython(const std::string &object_name, const bp::list &pose)
  {
    geometry_msgs::PoseStamped msg;
    convertListToPose(pose, msg.pose);
    msg.header.frame_id = getPoseReferenceFrame();
    msg.header.stamp = ros::Time::now();
    return place(object_name, msg);
  }

  void convertListToArrayOfPoses(const bp::list &poses, std::vector<geometry_msgs::Pose> &msg)
  {
    int l = bp::len(poses);
    for (int i = 0; i < l ; ++i)
    {
      const bp::list &pose = bp::extract<bp::list>(poses[i]);
      std::vector<double> v = py_bindings_tools::doubleFromList(pose);
      if (v.size() == 6 || v.size() == 7)
      {
        Eigen::Affine3d p;
        if (v.size() == 6)
        {
          Eigen::Quaterniond q;
          tf::quaternionTFToEigen(tf::createQuaternionFromRPY(v[3], v[4], v[5]), q);
          p = Eigen::Affine3d(q);
        }
        else
          p = Eigen::Affine3d(Eigen::Quaterniond(v[6], v[3], v[4], v[5]));
        p.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
        geometry_msgs::Pose pm;
        tf::poseEigenToMsg(p, pm);
        msg.push_back(pm);
      }
      else
        ROS_WARN("Incorrect number of values for a pose: %u", (unsigned int)v.size());
    }
  }

  void setStartStatePython(const std::string &msg_str)
  {
    moveit_msgs::RobotState msg;    
    py_bindings_tools::deserializeMsg(msg_str, msg);
    setStartState(msg);
  }
  
  bool setPoseTargetsPython(bp::list &poses, const std::string &end_effector_link = "")
  {
    std::vector<geometry_msgs::Pose> msg;
    convertListToArrayOfPoses(poses, msg);
    return setPoseTargets(msg, end_effector_link);
  }

  bool setPoseTargetPython(bp::list &pose, const std::string &end_effector_link = "")
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(pose);
    geometry_msgs::Pose msg;
    if (v.size() == 6)
      tf::quaternionTFToMsg(tf::createQuaternionFromRPY(v[3], v[4], v[5]), msg.orientation);
    else
      if (v.size() == 7)
      {
        msg.orientation.x = v[3];
        msg.orientation.y = v[4];
        msg.orientation.z = v[5];
        msg.orientation.w = v[6];
      }
      else
      {
        ROS_ERROR("Pose description expected to consist of either 6 or 7 values");
        return false;
      }
    msg.position.x = v[0];
    msg.position.y = v[1];
    msg.position.z = v[2];
    return setPoseTarget(msg, end_effector_link);
  }

  const char* getEndEffectorLinkCStr() const
  {
    return getEndEffectorLink().c_str();
  }

  const char* getPoseReferenceFrameCStr() const
  {
    return getPoseReferenceFrame().c_str();
  }

  const char* getNameCStr() const
  {
    return getName().c_str();
  }

  bool movePython()
  {
    return move();
  } 

  bool asyncMovePython()
  {
    return asyncMove();
  }

  bool attachObjectPython(const std::string &object_name, const std::string &link_name, const bp::list &touch_links)
  {
    return attachObject(object_name, link_name, py_bindings_tools::stringFromList(touch_links));
  }
  
  bool executePython(const std::string &plan_str)
  {
    MoveGroup::Plan plan;
    py_bindings_tools::deserializeMsg(plan_str, plan.trajectory_);
    return execute(plan);
  }
  
  std::string getPlanPython()
  {
    MoveGroup::Plan plan;
    MoveGroup::plan(plan);
    return py_bindings_tools::serializeMsg(plan.trajectory_);
  }

  bp::tuple computeCartesianPathPython(const bp::list &waypoints, double eef_step, double jump_threshold, bool avoid_collisions)
  {
    std::vector<geometry_msgs::Pose> poses;
    convertListToArrayOfPoses(waypoints, poses);
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = computeCartesianPath(poses, eef_step, jump_threshold, trajectory, avoid_collisions);
    return bp::make_tuple(py_bindings_tools::serializeMsg(trajectory), fraction);
  }
  
  int pickGrasp(const std::string &object, const std::string &grasp_str)
  {
    moveit_msgs::Grasp grasp;    
    py_bindings_tools::deserializeMsg(grasp_str, grasp);
    return pick(object, grasp).val;
  } 

  int pickGrasps(const std::string &object, const bp::list &grasp_list)
  {
    int l = bp::len(grasp_list);
    std::vector<moveit_msgs::Grasp> grasps(l);
    for (int i = 0; i < l ; ++i)
      py_bindings_tools::deserializeMsg(bp::extract<std::string>(grasp_list[i]), grasps[i]);
    return pick(object, grasps).val;
  }

  void setPathConstraintsFromMsg(const std::string &constraints_str)
  {
      moveit_msgs::Constraints constraints_msg;
      py_bindings_tools::deserializeMsg(constraints_str,constraints_msg);
      setPathConstraints(constraints_msg);
  } 

  std::string getPathConstraintsPython()
  {
     moveit_msgs::Constraints constraints_msg(getPathConstraints());
     std::string constraints_str = py_bindings_tools::serializeMsg(constraints_msg);
     return constraints_str;
  }
  
};

static void wrap_move_group_interface()
{
  bp::class_<MoveGroupWrapper> MoveGroupClass("MoveGroup", bp::init<std::string, std::string>());

  MoveGroupClass.def("async_move", &MoveGroupWrapper::asyncMovePython);
  MoveGroupClass.def("move", &MoveGroupWrapper::movePython);
  MoveGroupClass.def("execute", &MoveGroupWrapper::executePython);
  moveit::planning_interface::MoveItErrorCode (MoveGroupWrapper::*pick_1)(const std::string&) = &MoveGroupWrapper::pick;
  MoveGroupClass.def("pick", pick_1);
  MoveGroupClass.def("pick", &MoveGroupWrapper::pickGrasp);
  MoveGroupClass.def("pick", &MoveGroupWrapper::pickGrasps);
  MoveGroupClass.def("place", &MoveGroupWrapper::placePython);
  MoveGroupClass.def("stop", &MoveGroupWrapper::stop);

  MoveGroupClass.def("get_name", &MoveGroupWrapper::getNameCStr);
  MoveGroupClass.def("get_planning_frame", &MoveGroupWrapper::getPlanningFrameCStr);

  MoveGroupClass.def("get_active_joints", &MoveGroupWrapper::getActiveJointsList);
  MoveGroupClass.def("get_joints", &MoveGroupWrapper::getJointsList);
  MoveGroupClass.def("get_variable_count", &MoveGroupWrapper::getVariableCount);
  MoveGroupClass.def("allow_looking", &MoveGroupWrapper::allowLooking);
  MoveGroupClass.def("allow_replanning", &MoveGroupWrapper::allowReplanning);

  MoveGroupClass.def("set_pose_reference_frame", &MoveGroupWrapper::setPoseReferenceFrame);

  MoveGroupClass.def("set_pose_reference_frame", &MoveGroupWrapper::setPoseReferenceFrame);
  MoveGroupClass.def("set_end_effector_link", &MoveGroupWrapper::setEndEffectorLink);
  MoveGroupClass.def("get_end_effector_link", &MoveGroupWrapper::getEndEffectorLinkCStr);
  MoveGroupClass.def("get_pose_reference_frame", &MoveGroupWrapper::getPoseReferenceFrameCStr);

  MoveGroupClass.def("set_pose_target", &MoveGroupWrapper::setPoseTargetPython);

  MoveGroupClass.def("set_pose_targets", &MoveGroupWrapper::setPoseTargetsPython);

  MoveGroupClass.def("set_position_target", &MoveGroupWrapper::setPositionTarget);
  MoveGroupClass.def("set_rpy_target", &MoveGroupWrapper::setRPYTarget);
  MoveGroupClass.def("set_orientation_target", &MoveGroupWrapper::setOrientationTarget);

  MoveGroupClass.def("get_current_pose", &MoveGroupWrapper::getCurrentPosePython);
  MoveGroupClass.def("get_current_rpy", &MoveGroupWrapper::getCurrentRPYPython);

  MoveGroupClass.def("get_random_pose", &MoveGroupWrapper::getRandomPosePython);

  MoveGroupClass.def("clear_pose_target", &MoveGroupWrapper::clearPoseTarget);
  MoveGroupClass.def("clear_pose_targets", &MoveGroupWrapper::clearPoseTargets);

  MoveGroupClass.def("set_joint_value_target", &MoveGroupWrapper::setJointValueTargetPythonList);
  MoveGroupClass.def("set_joint_value_target", &MoveGroupWrapper::setJointValueTargetPythonDict);

  MoveGroupClass.def("set_joint_value_target", &MoveGroupWrapper::setJointValueTargetPerJointPythonList);
  bool (MoveGroupWrapper::*setJointValueTarget_4)(const std::string&, double) = &MoveGroupWrapper::setJointValueTarget;
  MoveGroupClass.def("set_joint_value_target", setJointValueTarget_4);

  MoveGroupClass.def("set_joint_value_target_from_pose", &MoveGroupWrapper::setJointValueTargetFromPosePython);
  MoveGroupClass.def("set_joint_value_target_from_pose_stamped", &MoveGroupWrapper::setJointValueTargetFromPoseStampedPython);
  MoveGroupClass.def("set_joint_value_target_from_joint_state_message", &MoveGroupWrapper::setJointValueTargetFromJointStatePython);

  MoveGroupClass.def("set_named_target", &MoveGroupWrapper::setNamedTarget);
  MoveGroupClass.def("set_random_target", &MoveGroupWrapper::setRandomTarget);

  void (MoveGroupWrapper::*rememberJointValues_2)(const std::string&) = &MoveGroupWrapper::rememberJointValues;
  MoveGroupClass.def("remember_joint_values", rememberJointValues_2);

  MoveGroupClass.def("remember_joint_values",  &MoveGroupWrapper::rememberJointValuesFromPythonList);

  MoveGroupClass.def("start_state_monitor",  &MoveGroupWrapper::startStateMonitor);
  MoveGroupClass.def("get_current_joint_values",  &MoveGroupWrapper::getCurrentJointValuesList);
  MoveGroupClass.def("get_random_joint_values",  &MoveGroupWrapper::getRandomJointValuesList);
  MoveGroupClass.def("get_remembered_joint_values",  &MoveGroupWrapper::getRememberedJointValuesPython);

  MoveGroupClass.def("forget_joint_values", &MoveGroupWrapper::forgetJointValues);

  MoveGroupClass.def("get_goal_joint_tolerance", &MoveGroupWrapper::getGoalJointTolerance);
  MoveGroupClass.def("get_goal_position_tolerance", &MoveGroupWrapper::getGoalPositionTolerance);
  MoveGroupClass.def("get_goal_orientation_tolerance", &MoveGroupWrapper::getGoalOrientationTolerance);

  MoveGroupClass.def("set_goal_joint_tolerance", &MoveGroupWrapper::setGoalJointTolerance);
  MoveGroupClass.def("set_goal_position_tolerance", &MoveGroupWrapper::setGoalPositionTolerance);
  MoveGroupClass.def("set_goal_orientation_tolerance", &MoveGroupWrapper::setGoalOrientationTolerance);
  MoveGroupClass.def("set_goal_tolerance", &MoveGroupWrapper::setGoalTolerance);

  MoveGroupClass.def("set_start_state_to_current_state", &MoveGroupWrapper::setStartStateToCurrentState);  
  MoveGroupClass.def("set_start_state", &MoveGroupWrapper::setStartStatePython);  

  bool (MoveGroupWrapper::*setPathConstraints_1)(const std::string&) = &MoveGroupWrapper::setPathConstraints;
  MoveGroupClass.def("set_path_constraints", setPathConstraints_1);
  MoveGroupClass.def("set_path_constraints_from_msg", &MoveGroupWrapper::setPathConstraintsFromMsg);
  MoveGroupClass.def("get_path_constraints", &MoveGroupWrapper::getPathConstraintsPython);
  MoveGroupClass.def("clear_path_constraints", &MoveGroupWrapper::clearPathConstraints);
  MoveGroupClass.def("get_known_constraints", &MoveGroupWrapper::getKnownConstraintsList);
  MoveGroupClass.def("set_constraints_database", &MoveGroupWrapper::setConstraintsDatabase);
  MoveGroupClass.def("set_workspace", &MoveGroupWrapper::setWorkspace);
  MoveGroupClass.def("set_planning_time", &MoveGroupWrapper::setPlanningTime);
  MoveGroupClass.def("get_planning_time", &MoveGroupWrapper::getPlanningTime);
  MoveGroupClass.def("set_planner_id", &MoveGroupWrapper::setPlannerId);
  MoveGroupClass.def("compute_plan", &MoveGroupWrapper::getPlanPython);
  MoveGroupClass.def("compute_cartesian_path", &MoveGroupWrapper::computeCartesianPathPython);
  MoveGroupClass.def("set_support_surface_name", &MoveGroupWrapper::setSupportSurfaceName);
  MoveGroupClass.def("attach_object", &MoveGroupWrapper::attachObjectPython);
  MoveGroupClass.def("detach_object", &MoveGroupWrapper::detachObject);
}

}
}

BOOST_PYTHON_MODULE(_moveit_move_group_interface)
{
  using namespace moveit::planning_interface;
  wrap_move_group_interface();
}

/** @endcond */
