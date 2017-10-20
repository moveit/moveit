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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/py_conversions.h>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
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
class MoveGroupInterfaceWrapper : protected py_bindings_tools::ROScppInitializer, public MoveGroupInterface
{
public:
  // ROSInitializer is constructed first, and ensures ros::init() was called, if
  // needed
  MoveGroupInterfaceWrapper(const std::string& group_name, const std::string& robot_description)
    : py_bindings_tools::ROScppInitializer()
    , MoveGroupInterface(Options(group_name, robot_description), boost::shared_ptr<tf::Transformer>(),
                         ros::WallDuration(5, 0))
  {
  }

  bool setJointValueTargetPerJointPythonList(const std::string& joint, bp::list& values)
  {
    return setJointValueTarget(joint, py_bindings_tools::doubleFromList(values));
  }

  bool setJointValueTargetPythonIterable(bp::object& values)
  {
    return setJointValueTarget(py_bindings_tools::doubleFromList(values));
  }

  bool setJointValueTargetPythonDict(bp::dict& values)
  {
    bp::list k = values.keys();
    int l = bp::len(k);
    std::map<std::string, double> v;
    for (int i = 0; i < l; ++i)
      v[bp::extract<std::string>(k[i])] = bp::extract<double>(values[k[i]]);
    return setJointValueTarget(v);
  }

  bool setJointValueTargetFromPosePython(const std::string& pose_str, const std::string& eef, bool approx)
  {
    geometry_msgs::Pose pose_msg;
    py_bindings_tools::deserializeMsg(pose_str, pose_msg);
    return approx ? setApproximateJointValueTarget(pose_msg, eef) : setJointValueTarget(pose_msg, eef);
  }

  bool setJointValueTargetFromPoseStampedPython(const std::string& pose_str, const std::string& eef, bool approx)
  {
    geometry_msgs::PoseStamped pose_msg;
    py_bindings_tools::deserializeMsg(pose_str, pose_msg);
    return approx ? setApproximateJointValueTarget(pose_msg, eef) : setJointValueTarget(pose_msg, eef);
  }

  bool setJointValueTargetFromJointStatePython(const std::string& js_str)
  {
    sensor_msgs::JointState js_msg;
    py_bindings_tools::deserializeMsg(js_str, js_msg);
    return setJointValueTarget(js_msg);
  }

  bp::list getJointValueTargetPythonList()
  {
    const robot_state::RobotState& values = moveit::planning_interface::MoveGroupInterface::getJointValueTarget();
    bp::list l;
    for (const double *it = values.getVariablePositions(), *end = it + getVariableCount(); it != end; ++it)
      l.append(*it);
    return l;
  }

  void rememberJointValuesFromPythonList(const std::string& string, bp::list& values)
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
    const std::map<std::string, std::vector<double> >& rv = getRememberedJointValues();
    bp::dict d;
    for (std::map<std::string, std::vector<double> >::const_iterator it = rv.begin(); it != rv.end(); ++it)
      d[it->first] = py_bindings_tools::listFromDouble(it->second);
    return d;
  }

  bp::list convertPoseToList(const geometry_msgs::Pose& pose) const
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

  bp::list convertTransformToList(const geometry_msgs::Transform& tr) const
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

  void convertListToTransform(const bp::list& l, geometry_msgs::Transform& tr) const
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(l);
    tr.translation.x = v[0];
    tr.translation.y = v[1];
    tr.translation.z = v[2];
    tr.rotation.x = v[3];
    tr.rotation.y = v[4];
    tr.rotation.z = v[5];
    tr.rotation.w = v[6];
  }

  void convertListToPose(const bp::list& l, geometry_msgs::Pose& p) const
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(l);
    p.position.x = v[0];
    p.position.y = v[1];
    p.position.z = v[2];
    p.orientation.x = v[3];
    p.orientation.y = v[4];
    p.orientation.z = v[5];
    p.orientation.w = v[6];
  }

  bp::list getCurrentRPYPython(const std::string& end_effector_link = "")
  {
    return py_bindings_tools::listFromDouble(getCurrentRPY(end_effector_link));
  }

  bp::list getCurrentPosePython(const std::string& end_effector_link = "")
  {
    geometry_msgs::PoseStamped pose = getCurrentPose(end_effector_link);
    return convertPoseToList(pose.pose);
  }

  bp::list getRandomPosePython(const std::string& end_effector_link = "")
  {
    geometry_msgs::PoseStamped pose = getRandomPose(end_effector_link);
    return convertPoseToList(pose.pose);
  }

  bp::list getKnownConstraintsList() const
  {
    return py_bindings_tools::listFromString(getKnownConstraints());
  }

  bool placePose(const std::string& object_name, const bp::list& pose)
  {
    geometry_msgs::PoseStamped msg;
    convertListToPose(pose, msg.pose);
    msg.header.frame_id = getPoseReferenceFrame();
    msg.header.stamp = ros::Time::now();
    return place(object_name, msg) == MoveItErrorCode::SUCCESS;
  }

  bool placeLocation(const std::string& object_name, const std::string& location_str)
  {
    std::vector<moveit_msgs::PlaceLocation> locations(1);
    py_bindings_tools::deserializeMsg(location_str, locations[0]);
    return place(object_name, locations) == MoveItErrorCode::SUCCESS;
  }

  bool placeAnywhere(const std::string& object_name)
  {
    return place(object_name) == MoveItErrorCode::SUCCESS;
  }

  void convertListToArrayOfPoses(const bp::list& poses, std::vector<geometry_msgs::Pose>& msg)
  {
    int l = bp::len(poses);
    for (int i = 0; i < l; ++i)
    {
      const bp::list& pose = bp::extract<bp::list>(poses[i]);
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

  bp::dict getCurrentStateBoundedPython()
  {
    robot_state::RobotStatePtr current = getCurrentState();
    current->enforceBounds();
    moveit_msgs::RobotState rsmv;
    robot_state::robotStateToRobotStateMsg(*current, rsmv);
    bp::dict output;
    for (size_t x = 0; x < rsmv.joint_state.name.size(); ++x)
      output[rsmv.joint_state.name[x]] = rsmv.joint_state.position[x];
    return output;
  }

  void setStartStatePython(const std::string& msg_str)
  {
    moveit_msgs::RobotState msg;
    py_bindings_tools::deserializeMsg(msg_str, msg);
    setStartState(msg);
  }

  bool setPoseTargetsPython(bp::list& poses, const std::string& end_effector_link = "")
  {
    std::vector<geometry_msgs::Pose> msg;
    convertListToArrayOfPoses(poses, msg);
    return setPoseTargets(msg, end_effector_link);
  }

  bool setPoseTargetPython(bp::list& pose, const std::string& end_effector_link = "")
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(pose);
    geometry_msgs::Pose msg;
    if (v.size() == 6)
      tf::quaternionTFToMsg(tf::createQuaternionFromRPY(v[3], v[4], v[5]), msg.orientation);
    else if (v.size() == 7)
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

  bp::dict getNamedTargetValuesPython(const std::string& name)
  {
    bp::dict output;
    std::map<std::string, double> positions = getNamedTargetValues(name);
    std::map<std::string, double>::iterator iterator;

    for (iterator = positions.begin(); iterator != positions.end(); iterator++)
      output[iterator->first] = iterator->second;
    return output;
  }

  bp::list getNamedTargetsPython()
  {
    return py_bindings_tools::listFromString(getNamedTargets());
  }

  bool movePython()
  {
    return move() == MoveItErrorCode::SUCCESS;
  }

  bool asyncMovePython()
  {
    return asyncMove() == MoveItErrorCode::SUCCESS;
  }

  bool attachObjectPython(const std::string& object_name, const std::string& link_name, const bp::list& touch_links)
  {
    return attachObject(object_name, link_name, py_bindings_tools::stringFromList(touch_links));
  }

  bool executePython(const std::string& plan_str)
  {
    MoveGroupInterface::Plan plan;
    py_bindings_tools::deserializeMsg(plan_str, plan.trajectory_);
    return execute(plan) == MoveItErrorCode::SUCCESS;
  }

  bool asyncExecutePython(const std::string& plan_str)
  {
    MoveGroupInterface::Plan plan;
    py_bindings_tools::deserializeMsg(plan_str, plan.trajectory_);
    return asyncExecute(plan) == MoveItErrorCode::SUCCESS;
  }

  std::string getPlanPython()
  {
    MoveGroupInterface::Plan plan;
    MoveGroupInterface::plan(plan);
    return py_bindings_tools::serializeMsg(plan.trajectory_);
  }

  bp::tuple computeCartesianPathPython(const bp::list& waypoints, double eef_step, double jump_threshold,
                                       bool avoid_collisions)
  {
    std::vector<geometry_msgs::Pose> poses;
    convertListToArrayOfPoses(waypoints, poses);
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = computeCartesianPath(poses, eef_step, jump_threshold, trajectory, avoid_collisions);
    return bp::make_tuple(py_bindings_tools::serializeMsg(trajectory), fraction);
  }

  int pickGrasp(const std::string& object, const std::string& grasp_str)
  {
    moveit_msgs::Grasp grasp;
    py_bindings_tools::deserializeMsg(grasp_str, grasp);
    return pick(object, grasp).val;
  }

  int pickGrasps(const std::string& object, const bp::list& grasp_list)
  {
    int l = bp::len(grasp_list);
    std::vector<moveit_msgs::Grasp> grasps(l);
    for (int i = 0; i < l; ++i)
      py_bindings_tools::deserializeMsg(bp::extract<std::string>(grasp_list[i]), grasps[i]);
    return pick(object, grasps).val;
  }

  void setPathConstraintsFromMsg(const std::string& constraints_str)
  {
    moveit_msgs::Constraints constraints_msg;
    py_bindings_tools::deserializeMsg(constraints_str, constraints_msg);
    setPathConstraints(constraints_msg);
  }

  std::string getPathConstraintsPython()
  {
    moveit_msgs::Constraints constraints_msg(getPathConstraints());
    std::string constraints_str = py_bindings_tools::serializeMsg(constraints_msg);
    return constraints_str;
  }

  std::string retimeTrajectory(const std::string& ref_state_str, const std::string& traj_str,
                               double velocity_scaling_factor)
  {
    // Convert reference state message to object
    moveit_msgs::RobotState ref_state_msg;
    py_bindings_tools::deserializeMsg(ref_state_str, ref_state_msg);
    moveit::core::RobotState ref_state_obj(getRobotModel());
    if (moveit::core::robotStateMsgToRobotState(ref_state_msg, ref_state_obj, true))
    {
      // Convert trajectory message to object
      moveit_msgs::RobotTrajectory traj_msg;
      py_bindings_tools::deserializeMsg(traj_str, traj_msg);
      robot_trajectory::RobotTrajectory traj_obj(getRobotModel(), getName());
      traj_obj.setRobotTrajectoryMsg(ref_state_obj, traj_msg);

      // Do the actual retiming
      trajectory_processing::IterativeParabolicTimeParameterization time_param;
      time_param.computeTimeStamps(traj_obj, velocity_scaling_factor);

      // Convert the retimed trajectory back into a message
      traj_obj.getRobotTrajectoryMsg(traj_msg);
      std::string traj_str = py_bindings_tools::serializeMsg(traj_msg);

      // Return it.
      return traj_str;
    }
    else
    {
      ROS_ERROR("Unable to convert RobotState message to RobotState instance.");
      return "";
    }
  }
};

class MoveGroupWrapper : public MoveGroupInterfaceWrapper
{
public:
  MoveGroupWrapper(const std::string& group_name, const std::string& robot_description)
    : MoveGroupInterfaceWrapper(group_name, robot_description)
  {
    ROS_WARN("The MoveGroup class is deprecated and will be removed in ROS lunar. Please use MoveGroupInterface "
             "instead.");
  }
};

static void wrap_move_group_interface()
{
  bp::class_<MoveGroupInterfaceWrapper, boost::noncopyable> MoveGroupInterfaceClass(
      "MoveGroupInterface", bp::init<std::string, std::string>());

  MoveGroupInterfaceClass.def("async_move", &MoveGroupInterfaceWrapper::asyncMovePython);
  MoveGroupInterfaceClass.def("move", &MoveGroupInterfaceWrapper::movePython);
  MoveGroupInterfaceClass.def("execute", &MoveGroupInterfaceWrapper::executePython);
  MoveGroupInterfaceClass.def("async_execute", &MoveGroupInterfaceWrapper::asyncExecutePython);
  moveit::planning_interface::MoveItErrorCode (MoveGroupInterfaceWrapper::*pick_1)(const std::string&) =
      &MoveGroupInterfaceWrapper::pick;
  MoveGroupInterfaceClass.def("pick", pick_1);
  MoveGroupInterfaceClass.def("pick", &MoveGroupInterfaceWrapper::pickGrasp);
  MoveGroupInterfaceClass.def("pick", &MoveGroupInterfaceWrapper::pickGrasps);
  MoveGroupInterfaceClass.def("place", &MoveGroupInterfaceWrapper::placePose);
  MoveGroupInterfaceClass.def("place", &MoveGroupInterfaceWrapper::placeLocation);
  MoveGroupInterfaceClass.def("place", &MoveGroupInterfaceWrapper::placeAnywhere);
  MoveGroupInterfaceClass.def("stop", &MoveGroupInterfaceWrapper::stop);

  MoveGroupInterfaceClass.def("get_name", &MoveGroupInterfaceWrapper::getNameCStr);
  MoveGroupInterfaceClass.def("get_planning_frame", &MoveGroupInterfaceWrapper::getPlanningFrameCStr);

  MoveGroupInterfaceClass.def("get_active_joints", &MoveGroupInterfaceWrapper::getActiveJointsList);
  MoveGroupInterfaceClass.def("get_joints", &MoveGroupInterfaceWrapper::getJointsList);
  MoveGroupInterfaceClass.def("get_variable_count", &MoveGroupInterfaceWrapper::getVariableCount);
  MoveGroupInterfaceClass.def("allow_looking", &MoveGroupInterfaceWrapper::allowLooking);
  MoveGroupInterfaceClass.def("allow_replanning", &MoveGroupInterfaceWrapper::allowReplanning);

  MoveGroupInterfaceClass.def("set_pose_reference_frame", &MoveGroupInterfaceWrapper::setPoseReferenceFrame);

  MoveGroupInterfaceClass.def("set_pose_reference_frame", &MoveGroupInterfaceWrapper::setPoseReferenceFrame);
  MoveGroupInterfaceClass.def("set_end_effector_link", &MoveGroupInterfaceWrapper::setEndEffectorLink);
  MoveGroupInterfaceClass.def("get_end_effector_link", &MoveGroupInterfaceWrapper::getEndEffectorLinkCStr);
  MoveGroupInterfaceClass.def("get_pose_reference_frame", &MoveGroupInterfaceWrapper::getPoseReferenceFrameCStr);

  MoveGroupInterfaceClass.def("set_pose_target", &MoveGroupInterfaceWrapper::setPoseTargetPython);

  MoveGroupInterfaceClass.def("set_pose_targets", &MoveGroupInterfaceWrapper::setPoseTargetsPython);

  MoveGroupInterfaceClass.def("set_position_target", &MoveGroupInterfaceWrapper::setPositionTarget);
  MoveGroupInterfaceClass.def("set_rpy_target", &MoveGroupInterfaceWrapper::setRPYTarget);
  MoveGroupInterfaceClass.def("set_orientation_target", &MoveGroupInterfaceWrapper::setOrientationTarget);

  MoveGroupInterfaceClass.def("get_current_pose", &MoveGroupInterfaceWrapper::getCurrentPosePython);
  MoveGroupInterfaceClass.def("get_current_rpy", &MoveGroupInterfaceWrapper::getCurrentRPYPython);

  MoveGroupInterfaceClass.def("get_random_pose", &MoveGroupInterfaceWrapper::getRandomPosePython);

  MoveGroupInterfaceClass.def("clear_pose_target", &MoveGroupInterfaceWrapper::clearPoseTarget);
  MoveGroupInterfaceClass.def("clear_pose_targets", &MoveGroupInterfaceWrapper::clearPoseTargets);

  MoveGroupInterfaceClass.def("set_joint_value_target", &MoveGroupInterfaceWrapper::setJointValueTargetPythonIterable);
  MoveGroupInterfaceClass.def("set_joint_value_target", &MoveGroupInterfaceWrapper::setJointValueTargetPythonDict);

  MoveGroupInterfaceClass.def("set_joint_value_target",
                              &MoveGroupInterfaceWrapper::setJointValueTargetPerJointPythonList);
  bool (MoveGroupInterfaceWrapper::*setJointValueTarget_4)(const std::string&, double) =
      &MoveGroupInterfaceWrapper::setJointValueTarget;
  MoveGroupInterfaceClass.def("set_joint_value_target", setJointValueTarget_4);

  MoveGroupInterfaceClass.def("set_joint_value_target_from_pose",
                              &MoveGroupInterfaceWrapper::setJointValueTargetFromPosePython);
  MoveGroupInterfaceClass.def("set_joint_value_target_from_pose_stamped",
                              &MoveGroupInterfaceWrapper::setJointValueTargetFromPoseStampedPython);
  MoveGroupInterfaceClass.def("set_joint_value_target_from_joint_state_message",
                              &MoveGroupInterfaceWrapper::setJointValueTargetFromJointStatePython);

  MoveGroupInterfaceClass.def("get_joint_value_target", &MoveGroupInterfaceWrapper::getJointValueTargetPythonList);

  MoveGroupInterfaceClass.def("set_named_target", &MoveGroupInterfaceWrapper::setNamedTarget);
  MoveGroupInterfaceClass.def("set_random_target", &MoveGroupInterfaceWrapper::setRandomTarget);

  void (MoveGroupInterfaceWrapper::*rememberJointValues_2)(const std::string&) =
      &MoveGroupInterfaceWrapper::rememberJointValues;
  MoveGroupInterfaceClass.def("remember_joint_values", rememberJointValues_2);

  MoveGroupInterfaceClass.def("remember_joint_values", &MoveGroupInterfaceWrapper::rememberJointValuesFromPythonList);

  MoveGroupInterfaceClass.def("start_state_monitor", &MoveGroupInterfaceWrapper::startStateMonitor);
  MoveGroupInterfaceClass.def("get_current_joint_values", &MoveGroupInterfaceWrapper::getCurrentJointValuesList);
  MoveGroupInterfaceClass.def("get_random_joint_values", &MoveGroupInterfaceWrapper::getRandomJointValuesList);
  MoveGroupInterfaceClass.def("get_remembered_joint_values",
                              &MoveGroupInterfaceWrapper::getRememberedJointValuesPython);

  MoveGroupInterfaceClass.def("forget_joint_values", &MoveGroupInterfaceWrapper::forgetJointValues);

  MoveGroupInterfaceClass.def("get_goal_joint_tolerance", &MoveGroupInterfaceWrapper::getGoalJointTolerance);
  MoveGroupInterfaceClass.def("get_goal_position_tolerance", &MoveGroupInterfaceWrapper::getGoalPositionTolerance);
  MoveGroupInterfaceClass.def("get_goal_orientation_tolerance",
                              &MoveGroupInterfaceWrapper::getGoalOrientationTolerance);

  MoveGroupInterfaceClass.def("set_goal_joint_tolerance", &MoveGroupInterfaceWrapper::setGoalJointTolerance);
  MoveGroupInterfaceClass.def("set_goal_position_tolerance", &MoveGroupInterfaceWrapper::setGoalPositionTolerance);
  MoveGroupInterfaceClass.def("set_goal_orientation_tolerance",
                              &MoveGroupInterfaceWrapper::setGoalOrientationTolerance);
  MoveGroupInterfaceClass.def("set_goal_tolerance", &MoveGroupInterfaceWrapper::setGoalTolerance);

  MoveGroupInterfaceClass.def("set_start_state_to_current_state",
                              &MoveGroupInterfaceWrapper::setStartStateToCurrentState);
  MoveGroupInterfaceClass.def("set_start_state", &MoveGroupInterfaceWrapper::setStartStatePython);

  bool (MoveGroupInterfaceWrapper::*setPathConstraints_1)(const std::string&) =
      &MoveGroupInterfaceWrapper::setPathConstraints;
  MoveGroupInterfaceClass.def("set_path_constraints", setPathConstraints_1);
  MoveGroupInterfaceClass.def("set_path_constraints_from_msg", &MoveGroupInterfaceWrapper::setPathConstraintsFromMsg);
  MoveGroupInterfaceClass.def("get_path_constraints", &MoveGroupInterfaceWrapper::getPathConstraintsPython);
  MoveGroupInterfaceClass.def("clear_path_constraints", &MoveGroupInterfaceWrapper::clearPathConstraints);
  MoveGroupInterfaceClass.def("get_known_constraints", &MoveGroupInterfaceWrapper::getKnownConstraintsList);
  MoveGroupInterfaceClass.def("set_constraints_database", &MoveGroupInterfaceWrapper::setConstraintsDatabase);
  MoveGroupInterfaceClass.def("set_workspace", &MoveGroupInterfaceWrapper::setWorkspace);
  MoveGroupInterfaceClass.def("set_planning_time", &MoveGroupInterfaceWrapper::setPlanningTime);
  MoveGroupInterfaceClass.def("get_planning_time", &MoveGroupInterfaceWrapper::getPlanningTime);
  MoveGroupInterfaceClass.def("set_max_velocity_scaling_factor",
                              &MoveGroupInterfaceWrapper::setMaxVelocityScalingFactor);
  MoveGroupInterfaceClass.def("set_max_acceleration_scaling_factor",
                              &MoveGroupWrapper::setMaxAccelerationScalingFactor);
  MoveGroupInterfaceClass.def("set_planner_id", &MoveGroupInterfaceWrapper::setPlannerId);
  MoveGroupInterfaceClass.def("set_num_planning_attempts", &MoveGroupInterfaceWrapper::setNumPlanningAttempts);
  MoveGroupInterfaceClass.def("compute_plan", &MoveGroupInterfaceWrapper::getPlanPython);
  MoveGroupInterfaceClass.def("compute_cartesian_path", &MoveGroupInterfaceWrapper::computeCartesianPathPython);
  MoveGroupInterfaceClass.def("set_support_surface_name", &MoveGroupInterfaceWrapper::setSupportSurfaceName);
  MoveGroupInterfaceClass.def("attach_object", &MoveGroupInterfaceWrapper::attachObjectPython);
  MoveGroupInterfaceClass.def("detach_object", &MoveGroupInterfaceWrapper::detachObject);
  MoveGroupInterfaceClass.def("retime_trajectory", &MoveGroupInterfaceWrapper::retimeTrajectory);
  MoveGroupInterfaceClass.def("get_named_targets", &MoveGroupInterfaceWrapper::getNamedTargetsPython);
  MoveGroupInterfaceClass.def("get_named_target_values", &MoveGroupInterfaceWrapper::getNamedTargetValuesPython);
  MoveGroupInterfaceClass.def("get_current_state_bounded", &MoveGroupInterfaceWrapper::getCurrentStateBoundedPython);

  bp::class_<MoveGroupWrapper, bp::bases<MoveGroupInterfaceWrapper>, boost::noncopyable> MoveGroupClass(
      "MoveGroup", bp::init<std::string, std::string>());
}
}
}

BOOST_PYTHON_MODULE(_moveit_move_group_interface)
{
  using namespace moveit::planning_interface;
  wrap_move_group_interface();
}

/** @endcond */
