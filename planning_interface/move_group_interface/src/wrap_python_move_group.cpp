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

  bp::list getJointsList() const
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
  
  bp::list poseToList(const geometry_msgs::Pose &pose) const
  {
    std::vector<double> v(7);
    v[0] = pose.position.x;
    v[1] = pose.position.y;
    v[2] = pose.position.z;
    v[3] = pose.orientation.x;
    v[4] = pose.orientation.y;
    v[5] = pose.orientation.z;
    v[6] = pose.orientation.w;
    return moveit_py_bindings_tools::listFromDouble(v);
  }

  bp::list transformToList(const geometry_msgs::Transform &tr) const
  {
    std::vector<double> v(7);
    v[0] = tr.translation.x;
    v[1] = tr.translation.y;
    v[2] = tr.translation.z;
    v[3] = tr.rotation.x;
    v[4] = tr.rotation.y;
    v[5] = tr.rotation.z;
    v[6] = tr.rotation.w;
    return moveit_py_bindings_tools::listFromDouble(v);
  }

  void convertListToTransform(const bp::list &l, geometry_msgs::Transform &tr) const
  {
    std::vector<double> v = moveit_py_bindings_tools::doubleFromList(l);
    tr.translation.x =  v[0];
    tr.translation.y = v[1];
    tr.translation.z = v[2];
    tr.rotation.x = v[3];
    tr.rotation.y = v[4];
    tr.rotation.z = v[5];
    tr.rotation.w = v[6];
  }

  bp::list getCurrentRPYPython(const std::string &end_effector_link = "")
  {
    return moveit_py_bindings_tools::listFromDouble(getCurrentRPY(end_effector_link));
  }
  
  bp::list getCurrentPosePython(const std::string &end_effector_link = "")
  {
    geometry_msgs::PoseStamped pose = getCurrentPose(end_effector_link);
    return poseToList(pose.pose);
  }

  bp::list getRandomPosePython(const std::string &end_effector_link = "")
  {
    geometry_msgs::PoseStamped pose = getRandomPose(end_effector_link);
    return poseToList(pose.pose);
  }

  bp::list getKnownConstraintsList() const
  {
    return moveit_py_bindings_tools::listFromString(getKnownConstraints());
  }

  void convertListToArrayOfPoses(const bp::list &poses, std::vector<geometry_msgs::Pose> &msg)
  { 
    int l = bp::len(poses);
    for (int i = 0; i < l ; ++i)
    {
      const bp::list &pose = bp::extract<bp::list>(poses[i]);
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
    convertListToArrayOfPoses(poses, msg);
    setPoseTargets(msg, end_effector_link);
  }

  void setPoseTargetPython(bp::list &pose, const std::string &end_effector_link = "")
  {
    std::vector<double> v = moveit_py_bindings_tools::doubleFromList(pose);
    if (v.size() == 6)
    {
      setPositionTarget(v[0], v[1], v[2], end_effector_link);
      setRPYTarget(v[3], v[4], v[5], end_effector_link);
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

  const char* getRobotRootLinkCStr() const
  {
    return getRobotRootLink().c_str();
  }

  const char* getPlanningFrameCStr() const
  {
    return getPlanningFrame().c_str();
  }

  bool executePython(bp::dict &plan_dict)
  {
    MoveGroup::Plan plan;
    convertDictToTrajectory(plan_dict, plan.trajectory_);
    return execute(plan);
  }
  
  void convertDictToTrajectory(const bp::dict &plan, moveit_msgs::RobotTrajectory &traj) const
  {
    traj.joint_trajectory.joint_names = moveit_py_bindings_tools::stringFromList(bp::extract<bp::list>(plan["joint_trajectory"]["joint_names"]));
    traj.multi_dof_joint_trajectory.joint_names = moveit_py_bindings_tools::stringFromList(bp::extract<bp::list>(plan["multi_dof_joint_trajectory"]["joint_names"]));
    traj.joint_trajectory.header.frame_id = bp::extract<std::string>(plan["joint_trajectory"]["frame_id"]);
    traj.multi_dof_joint_trajectory.header.frame_id = bp::extract<std::string>(plan["multi_dof_joint_trajectory"]["frame_id"]);

    const bp::list &joint_trajectory_points = bp::extract<bp::list>(plan["joint_trajectory"]["points"]);
    int l = boost::python::len(joint_trajectory_points);
    for (int i = 0 ; i < l ; ++i)
    {
      trajectory_msgs::JointTrajectoryPoint pt;
      pt.positions = moveit_py_bindings_tools::doubleFromList(bp::extract<bp::list>(joint_trajectory_points[i]["positions"]));
      pt.velocities = moveit_py_bindings_tools::doubleFromList(bp::extract<bp::list>(joint_trajectory_points[i]["velocities"]));
      pt.accelerations = moveit_py_bindings_tools::doubleFromList(bp::extract<bp::list>(joint_trajectory_points[i]["accelerations"]));
      pt.time_from_start = ros::Duration(bp::extract<double>(joint_trajectory_points[i]["time_from_start"]));
      traj.joint_trajectory.points.push_back(pt);       
    }
    const bp::list &multi_dof_joint_trajectory_points = bp::extract<bp::list>(plan["multi_dof_joint_trajectory"]["points"]);
    l = boost::python::len(multi_dof_joint_trajectory_points);
    for (int i = 0 ; i < l ; ++i)
    {
      moveit_msgs::MultiDOFJointTrajectoryPoint pt;
      const bp::list &tf = bp::extract<bp::list>(multi_dof_joint_trajectory_points[i]["transforms"]);
      int lk = boost::python::len(tf);
      for (int k = 0 ; k < lk ; ++k)
      {
	geometry_msgs::Transform tf_msg;
	convertListToTransform(bp::extract<bp::list>(tf[k]), tf_msg);
	pt.transforms.push_back(tf_msg);
      }
      pt.time_from_start = ros::Duration(bp::extract<double>(multi_dof_joint_trajectory_points[i]["time_from_start"]));
      traj.multi_dof_joint_trajectory.points.push_back(pt);       
    }
     
  }
  
  bp::dict convertTrajectoryToDict(moveit_msgs::RobotTrajectory &traj) const
  {
    bp::list joint_names = moveit_py_bindings_tools::listFromString(traj.joint_trajectory.joint_names);
    bp::dict joint_trajectory, multi_dof_joint_trajectory;
    joint_trajectory["joint_names"] = joint_names;
    multi_dof_joint_trajectory["joint_names"] = joint_names;
    joint_trajectory["frame_id"] = traj.joint_trajectory.header.frame_id;
    multi_dof_joint_trajectory["frame_id"] = traj.multi_dof_joint_trajectory.header.frame_id;
    
    bp::list joint_traj_points;
    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it = traj.joint_trajectory.points.begin() ;
         it != traj.joint_trajectory.points.end() ; ++it)
    {
      bp::dict joint_traj_point;
      joint_traj_point["positions"] = moveit_py_bindings_tools::listFromDouble(it->positions);
      joint_traj_point["velocities"] = moveit_py_bindings_tools::listFromDouble(it->velocities);
      joint_traj_point["accelerations"] = moveit_py_bindings_tools::listFromDouble(it->accelerations);
      joint_traj_point["time_from_start"] = it->time_from_start.toSec();
      joint_traj_points.append(joint_traj_point);
    }

    joint_trajectory["points"] = joint_traj_points;
    
    bp::list multi_dof_traj_points;
    for (std::vector<moveit_msgs::MultiDOFJointTrajectoryPoint>::const_iterator it = traj.multi_dof_joint_trajectory.points.begin() ;
         it != traj.multi_dof_joint_trajectory.points.end() ; ++it)
    {
      bp::dict multi_dof_traj_point;
      bp::list transforms;
      for (std::vector<geometry_msgs::Transform>::const_iterator itr = it->transforms.begin() ; itr != it->transforms.end() ; ++itr)
        transforms.append(transformToList(*itr));
      multi_dof_traj_point["transforms"] = transforms;
      multi_dof_traj_point["time_from_start"] = it->time_from_start.toSec();
      multi_dof_traj_points.append(multi_dof_traj_point);
    }
    multi_dof_joint_trajectory["points"] = multi_dof_traj_points;

    bp::dict plan_dict;
    plan_dict["joint_trajectory"] = joint_trajectory;
    plan_dict["multi_dof_joint_trajectory"] = multi_dof_joint_trajectory;
    return plan_dict;
  }

  bp::dict getPlanPythonDict()
  {
    MoveGroup::Plan plan;
    MoveGroup::plan(plan);
    return convertTrajectoryToDict(plan.trajectory_);
  }

  bp::tuple computeCartesianPathPython(const bp::list &waypoints, double eef_step, double jump_threshold, bool avoid_collisions)
  {
    std::vector<geometry_msgs::Pose> poses;
    convertListToArrayOfPoses(waypoints, poses);
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = computeCartesianPath(poses, eef_step, jump_threshold, trajectory, avoid_collisions);
    return bp::make_tuple(convertTrajectoryToDict(trajectory), fraction);
  }

};  
  
void wrap_move_group_interface()
{
  void (*init_fn)(const std::string&, bp::list&) = &moveit_py_bindings_tools::roscpp_init;
  bp::def("roscpp_init", init_fn);
  bp::def("roscpp_shutdown", &moveit_py_bindings_tools::roscpp_shutdown);

  bp::class_<MoveGroupWrapper> MoveGroupClass("MoveGroup", bp::init<std::string>());

  MoveGroupClass.def("async_move", &MoveGroupWrapper::asyncMove);
  MoveGroupClass.def("move", &MoveGroupWrapper::move);
  MoveGroupClass.def("execute", &MoveGroupWrapper::executePython); 
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
  void (MoveGroupWrapper::*setJointValueTarget_4)(const std::string&, double) = &MoveGroupWrapper::setJointValueTarget;
  MoveGroupClass.def("set_joint_value_target", setJointValueTarget_4);

  void (MoveGroupWrapper::*setJointValueTarget_5)(const sensor_msgs::JointState &) = &MoveGroupWrapper::setJointValueTarget;
  MoveGroupClass.def("set_joint_value_target", setJointValueTarget_5);

  MoveGroupClass.def("set_named_target", &MoveGroupWrapper::setNamedTarget); 
  MoveGroupClass.def("set_random_target", &MoveGroupWrapper::setRandomTarget); 

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
  MoveGroupClass.def("get_planning_time", &MoveGroupWrapper::getPlanningTime);
  MoveGroupClass.def("set_planner_id", &MoveGroupWrapper::setPlannerId);
  MoveGroupClass.def("compute_plan", &MoveGroupWrapper::getPlanPythonDict);
  MoveGroupClass.def("compute_cartesian_path", &MoveGroupWrapper::computeCartesianPathPython);
  MoveGroupClass.def("set_support_surface_name", &MoveGroupWrapper::setSupportSurfaceName);

  MoveGroupClass.def("get_robot_root_link", &MoveGroupWrapper::getRobotRootLinkCStr);
  MoveGroupClass.def("get_planning_frame", &MoveGroupWrapper::getPlanningFrameCStr);
}

}

BOOST_PYTHON_MODULE(_moveit_move_group_interface)
{
  using namespace move_group_interface;
  wrap_move_group_interface();
}
