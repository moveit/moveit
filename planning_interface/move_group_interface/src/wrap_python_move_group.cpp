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

/* Author: Ioan Sucan */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/py_conversions.h>
#include <eigen_conversions/eigen_msg.h>

#include <boost/function.hpp>
#include <boost/python.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>

#include <boost/shared_ptr.hpp>
#include <Python.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

namespace bp = boost::python;

namespace move_group_interface
{

class MoveGroupWrapper : protected moveit_py_bindings_tools::ROScppInitializer,
                         public MoveGroup
{
public:

  // ROSInitializer is constructed first, and ensures ros::init() was called, if needed
  MoveGroupWrapper(const std::string &group_name) : moveit_py_bindings_tools::ROScppInitializer(),
                                                    MoveGroup(group_name, boost::shared_ptr<tf::Transformer>(), ros::Duration(5, 0))
  {
  }
  
  void setJointValueTargetPerJointPythonList(const std::string &joint, bp::list &values)
  {
    setJointValueTarget(joint, moveit_py_bindings_tools::doubleFromList(values));
  }
  
  void setJointValueTargetPythonList(bp::list &values)
  {
    setJointValueTarget(moveit_py_bindings_tools::doubleFromList(values));
  }

  void setJointValueTargetPythonDict(bp::dict &values)
  {
    bp::list k = values.keys(); 
    int l = bp::len(k);
    std::map<std::string, double> v;
    for (int i = 0; i < l ; ++i)
      v[bp::extract<std::string>(k[i])] = bp::extract<double>(values[k[i]]);
    setJointValueTarget(v);
  }
  
  void rememberJointValuesFromPythonList(const std::string &string, bp::list &values)
  {
    rememberJointValues(string, moveit_py_bindings_tools::doubleFromList(values));
  }

  bp::list getJointsList()
  {
    return moveit_py_bindings_tools::listFromString(getJoints());
  }

  bp::list getCurrentJointValuesList()
  {
    return moveit_py_bindings_tools::listFromDouble(getCurrentJointValues());
  }
  
  bp::list getRandomJointValuesList()
  {
    return moveit_py_bindings_tools::listFromDouble(getRandomJointValues());
  }
  
  bp::dict getRememberedJointValuesPython() const
  {
    const std::map<std::string, std::vector<double> > &rv = getRememberedJointValues();
    bp::dict d;
    for (std::map<std::string, std::vector<double> >::const_iterator it = rv.begin() ; it != rv.end() ; ++it)
      d[it->first] = moveit_py_bindings_tools::listFromDouble(it->second);
    return d;
  }
  
  bp::list poseToList(const geometry_msgs::PoseStamped &pose)
  {
    std::vector<double> v(7);
    v[0] = pose.pose.position.x;
    v[1] = pose.pose.position.y;
    v[2] = pose.pose.position.z;
    v[3] = pose.pose.orientation.x;
    v[4] = pose.pose.orientation.y;
    v[5] = pose.pose.orientation.z;
    v[6] = pose.pose.orientation.w;
    return moveit_py_bindings_tools::listFromDouble(v);
  }
  
  bp::list getCurrentPosePython(const std::string &end_effector_link = "")
  {
    geometry_msgs::PoseStamped pose = getCurrentPose(end_effector_link);
    return poseToList(pose);
  }

  bp::list getRandomPosePython(const std::string &end_effector_link = "")
  {
    geometry_msgs::PoseStamped pose = getRandomPose(end_effector_link);
    return poseToList(pose);
  }

  bp::list getKnownConstraintsList() const
  {
    return moveit_py_bindings_tools::listFromString(getKnownConstraints());
  }

  void convertToArrayOfPoses(bp::list &poses, std::vector<geometry_msgs::Pose> &msg)
  { 
    int l = bp::len(poses);
    for (int i = 0; i < l ; ++i)
    {
      bp::list pose = bp::extract<bp::list>(poses[i]);
      std::vector<double> v = moveit_py_bindings_tools::doubleFromList(pose);
      if (v.size() == 6 || v.size() == 7)
      {
        Eigen::Affine3d p = v.size() == 6 ? 
          Eigen::Affine3d(Eigen::AngleAxisd(v[3], Eigen::Vector3d::UnitX())
                          * Eigen::AngleAxisd(v[4], Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(v[5], Eigen::Vector3d::UnitZ())) :
          Eigen::Affine3d(Eigen::Quaterniond(v[6], v[3], v[4], v[5]));
        p.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
        geometry_msgs::Pose pm;
        tf::poseEigenToMsg(p, pm);
        msg.push_back(pm);
      }
      else
        ROS_WARN("Incorrect number of values for a pose: %u", (unsigned int)v.size());
    }
  }
  
  void setPoseTargetsPython(bp::list &poses, const std::string &end_effector_link = "")
  {
    std::vector<geometry_msgs::Pose> msg;
    convertToArrayOfPoses(poses, msg);
    setPoseTargets(msg, end_effector_link);
  }

  void followConstraintsPython(bp::list &poses, const std::string &end_effector_link = "")
  {
    std::vector<geometry_msgs::Pose> msg;
    convertToArrayOfPoses(poses, msg);
    followConstraints(msg, 1e-3, 1e-2, end_effector_link);
  }
  
  void setPoseTargetPython(bp::list &pose, const std::string &end_effector_link = "")
  {
    std::vector<double> v = moveit_py_bindings_tools::doubleFromList(pose);
    if (v.size() == 6)
    {
      setPositionTarget(v[0], v[1], v[2], end_effector_link);
      setOrientationTarget(v[3], v[4], v[5], end_effector_link);
    }
    else
      if (v.size() == 7)
      {
        setPositionTarget(v[0], v[1], v[2], end_effector_link);
        setOrientationTarget(v[3], v[4], v[5], v[6], end_effector_link);
      }
      else
        ROS_ERROR("Pose description expected to consist of either 6 or 7 values");
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

  bp::dict getPlanPythonDict()
  {
    MoveGroup::Plan plan;
    MoveGroup::plan(plan);
    bp::list joint_names = moveit_py_bindings_tools::listFromString(plan.trajectory_.joint_trajectory.joint_names);
    bp::dict plan_dict, joint_trajectory, multi_dof_joint_trajectory;
    joint_trajectory["joint_names"] = joint_names;
    multi_dof_joint_trajectory["joint_names"] = joint_names;
    bp::list joint_traj_points, multi_dof_traj_points, poses;
    bp::dict joint_traj_point, multi_dof_traj_point, pose, position, orientation;

    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it = plan.trajectory_.joint_trajectory.points.begin() ;
         it != plan.trajectory_.joint_trajectory.points.end() ; ++it)
    {
      joint_traj_point["positions"] = moveit_py_bindings_tools::listFromDouble(it->positions);
      joint_traj_point["velocities"] = moveit_py_bindings_tools::listFromDouble(it->velocities);
      joint_traj_point["accelerations"] = moveit_py_bindings_tools::listFromDouble(it->accelerations);
      joint_traj_points.append(joint_traj_point);
    }

    joint_trajectory["points"] = joint_traj_points;
    
    for (std::vector<moveit_msgs::MultiDOFJointTrajectoryPoint>::const_iterator it = plan.trajectory_.multi_dof_joint_trajectory.points.begin() ;
         it != plan.trajectory_.multi_dof_joint_trajectory.points.end() ; ++it)
    {
      for (std::vector<geometry_msgs::Transform>::const_iterator itr = it->transforms.begin() ; itr != it->transforms.end() ; ++itr)
      {
        position["x"] = itr->translation.x;
        position["y"] = itr->translation.y;
        position["z"] = itr->translation.z;
        pose["translation"] = position;
        
        orientation["x"] = itr->rotation.x;
        orientation["y"] = itr->rotation.y;
        orientation["z"] = itr->rotation.z;
        orientation["w"] = itr->rotation.w;
        pose["rotation"] = orientation;
        poses.append(pose);
      }
      multi_dof_traj_point["poses"] = poses;
      multi_dof_traj_points.append(multi_dof_traj_point);
    }
    
    multi_dof_joint_trajectory["points"] = multi_dof_traj_points;

    plan_dict["joint_trajectory"] = joint_trajectory;
    plan_dict["multi_dof_joint_trajectory"] = multi_dof_joint_trajectory;
    return plan_dict;
  }

};  
  
void wrap_move_group_interface()
{
  bp::class_<MoveGroupWrapper> MoveGroupClass("MoveGroup", bp::init<std::string>());

  MoveGroupClass.def("async_move", &MoveGroupWrapper::asyncMove);
  MoveGroupClass.def("move", &MoveGroupWrapper::move);
  MoveGroupClass.def("execute", &MoveGroupWrapper::execute); 
  bool (MoveGroupWrapper::*pick_1)(const std::string&) = &MoveGroupWrapper::pick;
  MoveGroupClass.def("pick", pick_1);
  bool (MoveGroupWrapper::*place_1)(const std::string&) = &MoveGroupWrapper::place;
  MoveGroupClass.def("place", place_1);
  MoveGroupClass.def("stop", &MoveGroupWrapper::stop);

  MoveGroupClass.def("get_name", &MoveGroupWrapper::getNameCStr);
  MoveGroupClass.def("get_joints", &MoveGroupWrapper::getJointsList);
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
  
  void (MoveGroupWrapper::*setOrientationTarget_1)(double, double, double, const std::string&) = &MoveGroupWrapper::setOrientationTarget;
  MoveGroupClass.def("set_orientation_target", setOrientationTarget_1);

  void (MoveGroupWrapper::*setOrientationTarget_2)(double, double, double, double, const std::string&) = &MoveGroupWrapper::setOrientationTarget;
  MoveGroupClass.def("set_orientation_target", setOrientationTarget_2);
  
  MoveGroupClass.def("get_current_pose", &MoveGroupWrapper::getCurrentPosePython);
  MoveGroupClass.def("get_random_pose", &MoveGroupWrapper::getRandomPosePython);

  MoveGroupClass.def("clear_pose_target", &MoveGroupWrapper::clearPoseTarget);
  MoveGroupClass.def("clear_pose_targets", &MoveGroupWrapper::clearPoseTargets);

  MoveGroupClass.def("set_joint_value_target", &MoveGroupWrapper::setJointValueTargetPythonList);
  MoveGroupClass.def("set_joint_value_target", &MoveGroupWrapper::setJointValueTargetPythonDict);
  MoveGroupClass.def("set_joint_value_target", &MoveGroupWrapper::setJointValueTargetPerJointPythonList);
  void (MoveGroupWrapper::*setJointValueTarget_4)(const std::string&, double) = &MoveGroupWrapper::setJointValueTarget;
  MoveGroupClass.def("set_joint_value_target", setJointValueTarget_4);

  void (MoveGroupWrapper::*setJointValueTarget_5)(const sensor_msgs::JointState &) = &MoveGroupWrapper::setJointValueTarget;
  MoveGroupClass.def("set_joint_value_target", setJointValueTarget_5);

  MoveGroupClass.def("set_named_target", &MoveGroupWrapper::setNamedTarget); 
  MoveGroupClass.def("set_random_target", &MoveGroupWrapper::setRandomTarget); 

  MoveGroupClass.def("follow_constraints", &MoveGroupWrapper::followConstraintsPython);
  
  void (MoveGroupWrapper::*rememberJointValues_2)(const std::string&) = &MoveGroupWrapper::rememberJointValues;
  MoveGroupClass.def("remember_joint_values", rememberJointValues_2);
  
  MoveGroupClass.def("remember_joint_values",  &MoveGroupWrapper::rememberJointValuesFromPythonList);

  MoveGroupClass.def("get_current_joint_values",  &MoveGroupWrapper::getCurrentJointValuesList);
  MoveGroupClass.def("get_random_joint_values",  &MoveGroupWrapper::getRandomJointValuesList);
  MoveGroupClass.def("get_remembered_joint_values",  &MoveGroupWrapper::getRememberedJointValuesPython);

  MoveGroupClass.def("forget_joint_values", &MoveGroupWrapper::forgetJointValues); 

  MoveGroupClass.def("get_goal_tolerance", &MoveGroupWrapper::getGoalTolerance); 
  MoveGroupClass.def("set_goal_tolerance", &MoveGroupWrapper::setGoalTolerance); 

  bool (MoveGroupWrapper::*setPathConstraints_1)(const std::string&) = &MoveGroupWrapper::setPathConstraints;
  MoveGroupClass.def("set_path_constraints", setPathConstraints_1);

  MoveGroupClass.def("clear_path_constraints", &MoveGroupWrapper::clearPathConstraints); 
  MoveGroupClass.def("get_known_constraints", &MoveGroupWrapper::getKnownConstraintsList);
  MoveGroupClass.def("set_constraints_database", &MoveGroupWrapper::setConstraintsDatabase); 
  MoveGroupClass.def("set_workspace", &MoveGroupWrapper::setWorkspace);
  MoveGroupClass.def("set_planning_time", &MoveGroupWrapper::setPlanningTime);
  MoveGroupClass.def("get_plan", &MoveGroupWrapper::getPlanPythonDict);  
  MoveGroupClass.def("set_support_surface_name", &MoveGroupWrapper::setSupportSurfaceName);
}

}

BOOST_PYTHON_MODULE(_moveit_move_group_interface)
{
  using namespace move_group_interface;
  wrap_move_group_interface();
}
