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

#ifndef PY_SSIZE_T_CLEAN
#define PY_SSIZE_T_CLEAN
#endif

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/ros_msg_typecasters.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <memory>

/** @cond IGNORE */

namespace py = pybind11;

using moveit::python::throwDeserializationError;

namespace moveit
{
namespace planning_interface
{
class MoveGroupInterfaceWrapper : protected py_bindings_tools::ROScppInitializer, public MoveGroupInterface
{
public:
  // ROSInitializer is constructed first, and ensures ros::init() was called, if needed
  MoveGroupInterfaceWrapper(const std::string& group_name, const std::string& robot_description, const std::string& ns,
                            double wait_for_servers)
    : py_bindings_tools::ROScppInitializer()
    , MoveGroupInterface(Options(group_name, robot_description, ros::NodeHandle(ns)),
                         std::shared_ptr<tf2_ros::Buffer>(), ros::WallDuration(wait_for_servers))
  {
  }

  bool setJointValueTargetFromPosePython(const geometry_msgs::Pose& pose_msg, const std::string& eef, bool approx)
  {
    return approx ? setApproximateJointValueTarget(pose_msg, eef) : setJointValueTarget(pose_msg, eef);
  }

  bool setJointValueTargetFromPoseStampedPython(const geometry_msgs::PoseStamped& pose_msg, const std::string& eef,
                                                bool approx)
  {
    return approx ? setApproximateJointValueTarget(pose_msg, eef) : setJointValueTarget(pose_msg, eef);
  }

  std::vector<double> getJointValueTargetPythonList()
  {
    std::vector<double> values;
    MoveGroupInterface::getJointValueTarget(values);
    return values;
  }

  void rememberJointValuesFromPythonList(const std::string& string, const std::vector<double>& values)
  {
    rememberJointValues(string, values);
  }

  const char* getPlanningFrameCStr() const
  {
    return getPlanningFrame().c_str();
  }

  moveit_msgs::PlannerInterfaceDescription getInterfaceDescriptionPython()
  {
    moveit_msgs::PlannerInterfaceDescription msg;
    getInterfaceDescription(msg);
    return msg;
  }

  bool placePose(const std::string& object_name, geometry_msgs::Pose pose, bool plan_only = false)
  {
    geometry_msgs::PoseStamped msg;
    msg.pose = pose;
    msg.header.frame_id = getPoseReferenceFrame();
    msg.header.stamp = ros::Time::now();
    py::gil_scoped_release gr;
    return place(object_name, msg, plan_only) == MoveItErrorCode::SUCCESS;
  }

  bool placePoses(const std::string& object_name, std::vector<geometry_msgs::PoseStamped> const& poses_list,
                  bool plan_only = false)
  {
    py::gil_scoped_release gr;
    return place(object_name, poses_list, plan_only) == MoveItErrorCode::SUCCESS;
  }

  bool placeLocations(const std::string& object_name, std::vector<moveit_msgs::PlaceLocation> location_list,
                      bool plan_only = false)
  {
    py::gil_scoped_release gr;
    return place(object_name, std::move(location_list), plan_only) == MoveItErrorCode::SUCCESS;
  }

  bool placeAnywhere(const std::string& object_name, bool plan_only = false)
  {
    py::gil_scoped_release gr;
    return place(object_name, plan_only) == MoveItErrorCode::SUCCESS;
  }

  moveit_msgs::RobotState getCurrentStateBoundedPython()
  {
    moveit::core::RobotStatePtr current = getCurrentState();
    current->enforceBounds();
    moveit_msgs::RobotState rsmv;
    moveit::core::robotStateToRobotStateMsg(*current, rsmv);
    return rsmv;
  }

  moveit_msgs::RobotState getCurrentStatePython()
  {
    moveit::core::RobotStatePtr current_state = getCurrentState();
    moveit_msgs::RobotState state_message;
    moveit::core::robotStateToRobotStateMsg(*current_state, state_message);
    return state_message;
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

  const char* getPlannerIdCStr() const
  {
    return getPlannerId().c_str();
  }

  const char* getPlanningPipelineIdCStr() const
  {
    return getPlanningPipelineId().c_str();
  }

  bool movePython()
  {
    py::gil_scoped_release gr;
    return move() == MoveItErrorCode::SUCCESS;
  }

  bool asyncMovePython()
  {
    return asyncMove() == MoveItErrorCode::SUCCESS;
  }

  bool executePython(const moveit_msgs::RobotTrajectory& plan)
  {
    py::gil_scoped_release gr;
    return execute(plan) == MoveItErrorCode::SUCCESS;
  }

  bool asyncExecutePython(const moveit_msgs::RobotTrajectory& plan)
  {
    return asyncExecute(plan) == MoveItErrorCode::SUCCESS;
  }

  std::tuple<moveit_msgs::MoveItErrorCodes, moveit_msgs::RobotTrajectory, double> planPython()
  {
    MoveGroupInterface::Plan plan;
    moveit_msgs::MoveItErrorCodes res;
    py::gil_scoped_release gr;
    res = MoveGroupInterface::plan(plan);
    return { res, plan.trajectory_, plan.planning_time_ };
  }

  moveit_msgs::MotionPlanRequest constructMotionPlanRequestPython()
  {
    moveit_msgs::MotionPlanRequest request;
    constructMotionPlanRequest(request);
    return request;
  }

  std::tuple<moveit_msgs::RobotTrajectory, double>
  computeCartesianPathPython(const std::vector<geometry_msgs::Pose>& waypoints, double eef_step, double jump_threshold,
                             bool avoid_collisions)
  {
    return computeCartesianPathConstrainedPython(waypoints, eef_step, jump_threshold, avoid_collisions, {});
  }

  std::tuple<moveit_msgs::RobotTrajectory, double>
  computeCartesianPathConstrainedPython(const std::vector<geometry_msgs::Pose>& poses, double eef_step,
                                        double jump_threshold, bool avoid_collisions,
                                        const moveit_msgs::Constraints& path_constraints)
  {
    moveit_msgs::RobotTrajectory trajectory;
    double fraction;
    py::gil_scoped_release gr;
    fraction = computeCartesianPath(poses, eef_step, jump_threshold, trajectory, path_constraints, avoid_collisions);
    return { trajectory, fraction };
  }

  moveit_msgs::RobotTrajectory retimeTrajectory(const moveit_msgs::RobotState& ref_state_msg,
                                                const moveit_msgs::RobotTrajectory& traj_msg,
                                                double velocity_scaling_factor, double acceleration_scaling_factor,
                                                const std::string& algorithm)
  {
    py::gil_scoped_release gr;
    // Convert reference state message to object
    moveit::core::RobotState ref_state_obj(getRobotModel());
    if (!moveit::core::robotStateMsgToRobotState(ref_state_msg, ref_state_obj, true))
    {
      ROS_ERROR("Unable to convert RobotState message to RobotState instance.");
      throwDeserializationError();
    }

    // Convert trajectory message to object
    robot_trajectory::RobotTrajectory traj_obj(getRobotModel(), getName());
    traj_obj.setRobotTrajectoryMsg(ref_state_obj, traj_msg);

    // Do the actual retiming
    if (algorithm == "iterative_time_parameterization")
    {
      trajectory_processing::IterativeParabolicTimeParameterization time_param;
      time_param.computeTimeStamps(traj_obj, velocity_scaling_factor, acceleration_scaling_factor);
    }
    else if (algorithm == "iterative_spline_parameterization")
    {
      trajectory_processing::IterativeSplineParameterization time_param;
      time_param.computeTimeStamps(traj_obj, velocity_scaling_factor, acceleration_scaling_factor);
    }
    else if (algorithm == "time_optimal_trajectory_generation")
    {
      trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
      time_param.computeTimeStamps(traj_obj, velocity_scaling_factor, acceleration_scaling_factor);
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("move_group_py", "Unknown time parameterization algorithm: " << algorithm);
      return {};
    }

    moveit_msgs::RobotTrajectory traj_msg_ans;
    // Convert the retimed trajectory back into a message
    traj_obj.getRobotTrajectoryMsg(traj_msg_ans);
    return traj_msg_ans;
  }

  Eigen::MatrixXd getJacobianMatrixPython(const std::vector<double>& joint_values,
                                          const std::array<double, 3>& reference_point)
  {
    moveit::core::RobotState state(getRobotModel());
    state.setToDefaultValues();
    auto group = state.getJointModelGroup(getName());
    state.setJointGroupPositions(group, joint_values);
    return state.getJacobian(group, Eigen::Map<const Eigen::Vector3d>(&reference_point[0]));
  }

  moveit_msgs::RobotState enforceBoundsPython(const moveit_msgs::RobotState& state_msg)
  {
    moveit::core::RobotState state(getRobotModel());
    if (moveit::core::robotStateMsgToRobotState(state_msg, state, true))
    {
      state.enforceBounds();
      moveit_msgs::RobotState ans;
      moveit::core::robotStateToRobotStateMsg(state, ans);
      return ans;
    }
    else
    {
      ROS_ERROR("Unable to convert RobotState message to RobotState instance.");
      throwDeserializationError();
    }
  }
};
}  // namespace planning_interface
}  // namespace moveit

PYBIND11_MODULE(pymoveit_move_group_interface, m)
{
  using moveit::planning_interface::MoveGroupInterface;
  using moveit::planning_interface::MoveGroupInterfaceWrapper;

  py::class_<MoveGroupInterfaceWrapper>(m, "MoveGroupInterface")
      .def(py::init<std::string, std::string, std::string, double>(), py::arg("group_name"),
           py::arg("robot_description") = "robot_description", py::arg("namespace") = std::string{},
           py::arg("wait_for_servers") = 5.0)

      .def("async_move", &MoveGroupInterfaceWrapper::asyncMovePython)
      .def("move", &MoveGroupInterfaceWrapper::movePython)
      .def("execute", &MoveGroupInterfaceWrapper::executePython, py::arg("plan"))
      .def("async_execute", &MoveGroupInterfaceWrapper::asyncExecutePython, py::arg("plan"))

      .def("pick",
           py::overload_cast<const std::string&, std::vector<moveit_msgs::Grasp>, bool>(&MoveGroupInterface::pick),
           py::arg("object"), py::arg("grasps"), py::arg("plan_only") = false)
      .def("place", &MoveGroupInterfaceWrapper::placePose, py::arg("object"), py::arg("pose"),
           py::arg("plan_only") = false)
      .def("place_poses_list", &MoveGroupInterfaceWrapper::placePoses, py::arg("object"), py::arg("poses_list"),
           py::arg("plan_only") = false)
      .def("place_locations_list", &MoveGroupInterfaceWrapper::placeLocations, py::arg("object"),
           py::arg("locations_list"), py::arg("plan_only") = false)
      .def("place", &MoveGroupInterfaceWrapper::placeAnywhere, py::arg("object"), py::arg("plan_only") = false)
      .def("stop", &MoveGroupInterfaceWrapper::stop)

      .def("get_name", &MoveGroupInterfaceWrapper::getNameCStr)
      .def("get_planning_frame", &MoveGroupInterfaceWrapper::getPlanningFrameCStr)
      .def("get_interface_description", &MoveGroupInterfaceWrapper::getInterfaceDescriptionPython)

      .def("get_active_joints", &MoveGroupInterface::getActiveJoints)
      .def("get_joints", &MoveGroupInterface::getJoints)
      .def("get_variable_count", &MoveGroupInterfaceWrapper::getVariableCount)
      .def("allow_looking", &MoveGroupInterfaceWrapper::allowLooking, py::arg("flag"))
      .def("allow_replanning", &MoveGroupInterfaceWrapper::allowReplanning, py::arg("flag"))

      .def("set_pose_reference_frame", &MoveGroupInterfaceWrapper::setPoseReferenceFrame, py::arg("reference_frame"))
      .def("set_end_effector_link", &MoveGroupInterfaceWrapper::setEndEffectorLink, py::arg("end_effector_link"))
      .def("get_end_effector_link", &MoveGroupInterfaceWrapper::getEndEffectorLinkCStr)
      .def("get_pose_reference_frame", &MoveGroupInterfaceWrapper::getPoseReferenceFrameCStr)

      .def("set_pose_target",
           py::overload_cast<const geometry_msgs::PoseStamped&, const std::string&>(&MoveGroupInterface::setPoseTarget),
           py::arg("target_pose"), py::arg("end_effector_link") = std::string{})
      .def("set_pose_target",
           py::overload_cast<const geometry_msgs::Pose&, const std::string&>(&MoveGroupInterface::setPoseTarget),
           py::arg("target_pose"), py::arg("end_effector_link") = std::string{})
      .def("set_pose_targets",
           py::overload_cast<const std::vector<geometry_msgs::Pose>&, std::string const&>(
               &MoveGroupInterface::setPoseTargets),
           py::arg("target_poses"), py::arg("end_effector_link") = std::string{})

      .def("set_position_target", &MoveGroupInterfaceWrapper::setPositionTarget, py::arg("x"), py::arg("y"),
           py::arg("z"), py::arg("end_effector_link") = std::string{})
      .def("set_rpy_target", &MoveGroupInterfaceWrapper::setRPYTarget, py::arg("roll"), py::arg("pitch"),
           py::arg("yaw"), py::arg("end_effector_link") = std::string{})
      .def("set_orientation_target", &MoveGroupInterfaceWrapper::setOrientationTarget, py::arg("x"), py::arg("y"),
           py::arg("z"), py::arg("w"), py::arg("end_effector_link") = std::string{})

      .def("get_current_pose", &MoveGroupInterface::getCurrentPose, py::arg("end_effector_link") = std::string{})
      .def("get_current_rpy", &MoveGroupInterface::getCurrentRPY, py::arg("end_effector_link") = std::string{})

      .def("get_random_pose", &MoveGroupInterface::getRandomPose, py::arg("end_effector_link") = std::string{})

      .def("clear_pose_target", &MoveGroupInterfaceWrapper::clearPoseTarget,
           py::arg("end_effector_link") = std::string{})
      .def("clear_pose_targets", &MoveGroupInterfaceWrapper::clearPoseTargets)

      .def("set_joint_value_target",
           py::overload_cast<const std::vector<double>&>(&MoveGroupInterface::setJointValueTarget),
           py::arg("group_variable_values"))
      .def("set_joint_value_target",
           py::overload_cast<std::map<std::string, double> const&>(&MoveGroupInterface::setJointValueTarget),
           py::arg("group_variabe_names_and_values"))

      .def("set_joint_value_target",
           py::overload_cast<const std::string&, const std::vector<double>&>(&MoveGroupInterface::setJointValueTarget),
           py::arg("joint_name"), py::arg("joint_values"))
      .def("set_joint_value_target",
           py::overload_cast<const std::string&, double>(&MoveGroupInterface::setJointValueTarget),
           py::arg("joint_name"), py::arg("joint_value"))

      .def("set_joint_value_target_from_pose", &MoveGroupInterfaceWrapper::setJointValueTargetFromPosePython,
           py::arg("pose_msg"), py::arg("end_effector_link"), py::arg("approx"))
      .def("set_joint_value_target_from_pose_stamped",
           &MoveGroupInterfaceWrapper::setJointValueTargetFromPoseStampedPython)
      .def("set_joint_value_target",
           py::overload_cast<sensor_msgs::JointState const&>(&MoveGroupInterface::setJointValueTarget),
           py::arg("joint_state"))

      .def("get_joint_value_target", &MoveGroupInterfaceWrapper::getJointValueTargetPythonList)

      .def("set_named_target", &MoveGroupInterfaceWrapper::setNamedTarget, py::arg("name"))
      .def("set_random_target", &MoveGroupInterfaceWrapper::setRandomTarget)

      .def("remember_joint_values", py::overload_cast<const std::string&>(&MoveGroupInterface::rememberJointValues),
           py::arg("name"))

      .def("remember_joint_values",
           py::overload_cast<const std::string&, const std::vector<double>&>(&MoveGroupInterface::rememberJointValues),
           py::arg("name"), py::arg("values"))

      .def("start_state_monitor", &MoveGroupInterfaceWrapper::startStateMonitor, py::arg("wait") = 1.0)
      .def("get_current_joint_values", &MoveGroupInterface::getCurrentJointValues)
      .def("get_random_joint_values", &MoveGroupInterface::getRandomJointValues)
      .def("get_remembered_joint_values", &MoveGroupInterface::getRememberedJointValues)

      .def("forget_joint_values", &MoveGroupInterfaceWrapper::forgetJointValues, py::arg("name"))

      .def("get_goal_joint_tolerance", &MoveGroupInterfaceWrapper::getGoalJointTolerance)
      .def("get_goal_position_tolerance", &MoveGroupInterfaceWrapper::getGoalPositionTolerance)
      .def("get_goal_orientation_tolerance", &MoveGroupInterfaceWrapper::getGoalOrientationTolerance)

      .def("set_goal_joint_tolerance", &MoveGroupInterfaceWrapper::setGoalJointTolerance, py::arg("tolerance"))
      .def("set_goal_position_tolerance", &MoveGroupInterfaceWrapper::setGoalPositionTolerance, py::arg("tolerance"))
      .def("set_goal_orientation_tolerance", &MoveGroupInterfaceWrapper::setGoalOrientationTolerance,
           py::arg("tolerance"))
      .def("set_goal_tolerance", &MoveGroupInterfaceWrapper::setGoalTolerance, py::arg("tolerance"))

      .def("set_start_state_to_current_state", &MoveGroupInterfaceWrapper::setStartStateToCurrentState)
      .def("set_start_state", py::overload_cast<const moveit_msgs::RobotState&>(&MoveGroupInterface::setStartState),
           py::arg("start_state"))

      .def("set_path_constraints", py::overload_cast<const std::string&>(&MoveGroupInterface::setPathConstraints),
           py::arg("constraint_name"))
      .def("set_path_constraints",
           py::overload_cast<moveit_msgs::Constraints const&>(&MoveGroupInterface::setPathConstraints),
           py::arg("constraints"))
      .def("get_path_constraints", &MoveGroupInterface::getPathConstraints)
      .def("clear_path_constraints", &MoveGroupInterfaceWrapper::clearPathConstraints)

      .def("set_trajectory_constraints", &MoveGroupInterface::setTrajectoryConstraints,
           py::arg("trajectory_constraints"))
      .def("get_trajectory_constraints", &MoveGroupInterface::getTrajectoryConstraints)
      .def("clear_trajectory_constraints", &MoveGroupInterfaceWrapper::clearTrajectoryConstraints)
      .def("get_known_constraints", &MoveGroupInterface::getKnownConstraints)
      .def("set_constraints_database", &MoveGroupInterfaceWrapper::setConstraintsDatabase, py::arg("host"),
           py::arg("port"))
      .def("set_workspace", &MoveGroupInterfaceWrapper::setWorkspace, py::arg("minx"), py::arg("miny"), py::arg("minz"),
           py::arg("maxx"), py::arg("maxy"), py::arg("maxz"))
      .def("set_planning_time", &MoveGroupInterfaceWrapper::setPlanningTime, py::arg("seconds"))
      .def("get_planning_time", &MoveGroupInterfaceWrapper::getPlanningTime)
      .def("set_max_velocity_scaling_factor", &MoveGroupInterfaceWrapper::setMaxVelocityScalingFactor,
           py::arg("max_velocity_scaling_factor"))
      .def("set_max_acceleration_scaling_factor", &MoveGroupInterfaceWrapper::setMaxAccelerationScalingFactor,
           py::arg("max_acceleration_scaling_factor"))
      .def("set_planner_id", &MoveGroupInterfaceWrapper::setPlannerId, py::arg("planner_id"))
      .def("get_planner_id", &MoveGroupInterfaceWrapper::getPlannerIdCStr)
      .def("set_planning_pipeline_id", &MoveGroupInterfaceWrapper::setPlanningPipelineId,
           py::arg("planning_pipeline_id"))
      .def("get_planning_pipeline_id", &MoveGroupInterfaceWrapper::getPlanningPipelineIdCStr)
      .def("set_num_planning_attempts", &MoveGroupInterfaceWrapper::setNumPlanningAttempts,
           py::arg("num_planning_attempts"))
      .def("plan", &MoveGroupInterfaceWrapper::planPython)
      .def("construct_motion_plan_request", &MoveGroupInterfaceWrapper::constructMotionPlanRequestPython)
      .def("compute_cartesian_path", &MoveGroupInterfaceWrapper::computeCartesianPathPython)
      .def("compute_cartesian_path", &MoveGroupInterfaceWrapper::computeCartesianPathConstrainedPython,
           py::arg("poses"), py::arg("end_effector_step"), py::arg("jump_threshold"), py::arg("avoid_collisions"),
           py::arg("path_constraints"))
      .def("set_support_surface_name", &MoveGroupInterfaceWrapper::setSupportSurfaceName, py::arg("name"))
      .def("attach_object",
           py::overload_cast<const std::string&, const std::string&, const std::vector<std::string>&>(
               &MoveGroupInterface::attachObject),
           py::arg("object"), py::arg("link"), py::arg("touch_links"))
      .def("detach_object", &MoveGroupInterfaceWrapper::detachObject, py::arg("name") = std::string{})
      .def("retime_trajectory", &MoveGroupInterfaceWrapper::retimeTrajectory, py::arg("ref_state_msg"),
           py::arg("traj_msg"), py::arg("velocity_scaling_factor"), py::arg("acceleration_scaling_factor"),
           py::arg("algorithm"))
      .def("get_named_targets", &MoveGroupInterface::getNamedTargets)
      .def("get_named_target_values", &MoveGroupInterface::getNamedTargetValues, py::arg("name"))
      .def("get_current_state_bounded", &MoveGroupInterfaceWrapper::getCurrentStateBoundedPython)
      .def("get_current_state", &MoveGroupInterfaceWrapper::getCurrentStatePython)
      .def("get_jacobian_matrix", &MoveGroupInterfaceWrapper::getJacobianMatrixPython, py::arg("joint_values"),
           py::arg("reference_point") = std::array<double, 3>{})
      .def("enforce_bounds", &MoveGroupInterfaceWrapper::enforceBoundsPython, py::arg("state_msg"))
      // keep semicolon on next line
      ;
}

/** @endcond */
