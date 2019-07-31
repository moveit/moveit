/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik Robotics
 *  Copyright (c) 2014, SRI International
 *  Copyright (c) 2013, Ioan A. Sucan
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

/* Author: Ioan Sucan, Sachin Chitta, Simon Goldstien */

#include <stdexcept>
#include <sstream>
#include <memory>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/QueryPlannerInterfaces.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit_msgs/GraspPlanning.h>
#include <moveit_msgs/GetPlannerParams.h>
#include <moveit_msgs/SetPlannerParams.h>

#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <ros/console.h>
#include <ros/ros.h>

namespace moveit
{
namespace planning_interface
{

const std::string MoveItCpp::ROBOT_DESCRIPTION =
  "robot_description";  // name of the robot description (a param name, so it can be changed externally)

const std::string GRASP_PLANNING_SERVICE_NAME = "plan_grasps";  // name of the service that can be used to plan grasps
}  //  planning_interface
}  //  moveit
moveit::planning_interface::MoveItCpp::MoveItCpp(const std::string& group_name,
                                                 const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                                 const ros::WallDuration& wait_for_servers)
  : MoveItCpp(Options(group_name), tf_buffer, wait_for_servers)
{
}

moveit::planning_interface::MoveItCpp::MoveItCpp(const std::string& group,
                                                 const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                                 const ros::Duration& wait_for_servers)
  : MoveItCpp(Options(group), tf_buffer, ros::WallDuration(wait_for_servers.toSec()))
{
}

moveit::planning_interface::MoveItCpp::MoveItCpp(const Options& opt,
                                                 const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                                 const ros::WallDuration& wait_for_servers)
  : node_handle_(opt.node_handle_), tf_buffer_(tf_buffer)
{
  group_name_ = opt.group_name_;
  robot_description_ = opt.robot_description_;
  robot_model_ = opt.robot_model_ ? opt.robot_model_ : getSharedRobotModel(opt.robot_description_);
  if (!getRobotModel())
  {
    std::string error = "Unable to construct robot model. Please make sure all needed information is on the "
      "parameter server.";
    ROS_FATAL_STREAM_NAMED("move_group_interface", error);
    throw std::runtime_error(error);
  }

  if (!getRobotModel()->hasJointModelGroup(opt.group_name_))
  {
    std::string error = "Group '" + opt.group_name_ + "' was not found.";
    ROS_FATAL_STREAM_NAMED("move_group_interface", error);
    throw std::runtime_error(error);
  }

  joint_model_group_ = getRobotModel()->getJointModelGroup(opt.group_name_);

  joint_state_target_.reset(new robot_state::RobotState(getRobotModel()));
  joint_state_target_->setToDefaultValues();
  active_target_ = JOINT;
  can_look_ = false;
  can_replan_ = false;
  replan_delay_ = 2.0;
  goal_joint_tolerance_ = 1e-4;
  goal_position_tolerance_ = 1e-4;     // 0.1 mm
  goal_orientation_tolerance_ = 1e-3;  // ~0.1 deg
  allowed_planning_time_ = 5.0;
  num_planning_attempts_ = 1;
  max_velocity_scaling_factor_ = 1.0;
  max_acceleration_scaling_factor_ = 1.0;
  initializing_constraints_ = false;

  if (joint_model_group_->isChain())
    end_effector_link_ = joint_model_group_->getLinkModelNames().back();
  pose_reference_frame_ = getRobotModel()->getModelFrame();

  trajectory_event_publisher_ = node_handle_.advertise<std_msgs::String>(trajectory_execution_manager::TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC, 1, false);
  attached_object_publisher_ = node_handle_.advertise<moveit_msgs::AttachedCollisionObject>(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC, 1, false);

  current_state_monitor_ = getSharedStateMonitor(robot_model_, tf_buffer_, node_handle_);

  ros::WallTime timeout_for_servers = ros::WallTime::now() + wait_for_servers;
  if (wait_for_servers == ros::WallDuration())
    timeout_for_servers = ros::WallTime();  // wait for ever
  double allotted_time = wait_for_servers.toSec();

  move_action_client_.reset(
                            new actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>(node_handle_, move_group::MOVE_ACTION, false));
  waitForAction(move_action_client_, move_group::MOVE_ACTION, timeout_for_servers, allotted_time);

  pick_action_client_.reset(
                            new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(node_handle_, move_group::PICKUP_ACTION, false));
  waitForAction(pick_action_client_, move_group::PICKUP_ACTION, timeout_for_servers, allotted_time);

  place_action_client_.reset(
                             new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>(node_handle_, move_group::PLACE_ACTION, false));
  waitForAction(place_action_client_, move_group::PLACE_ACTION, timeout_for_servers, allotted_time);

  execute_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>(
                                                                                                       node_handle_, move_group::EXECUTE_ACTION_NAME, false));
  waitForAction(execute_action_client_, move_group::EXECUTE_ACTION_NAME, timeout_for_servers, allotted_time);

  query_service_ =
    node_handle_.serviceClient<moveit_msgs::QueryPlannerInterfaces>(move_group::QUERY_PLANNERS_SERVICE_NAME);
  get_params_service_ =
    node_handle_.serviceClient<moveit_msgs::GetPlannerParams>(move_group::GET_PLANNER_PARAMS_SERVICE_NAME);
  set_params_service_ =
    node_handle_.serviceClient<moveit_msgs::SetPlannerParams>(move_group::SET_PLANNER_PARAMS_SERVICE_NAME);

  cartesian_path_service_ =
    node_handle_.serviceClient<moveit_msgs::GetCartesianPath>(move_group::CARTESIAN_PATH_SERVICE_NAME);

  plan_grasps_service_ = node_handle_.serviceClient<moveit_msgs::GraspPlanning>(GRASP_PLANNING_SERVICE_NAME);

  ROS_INFO_STREAM_NAMED("move_group_interface", "Ready to take commands for planning group " << opt.group_name_
                        << ".");
  group_name_ = opt.group_name_;
  robot_description_ = opt.robot_description_;
}

moveit::planning_interface::MoveItCpp::MoveItCpp(
    const moveit::planning_interface::MoveItCpp::Options& opt,
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const ros::Duration& wait_for_servers)
  : MoveItCpp(opt, tf_buffer, ros::WallDuration(wait_for_servers.toSec()))
{
}

moveit::planning_interface::MoveItCpp::~MoveItCpp()
{
}

moveit::planning_interface::MoveItCpp::MoveItCpp(MoveItCpp&& other)
{
  other.clearContents();
}

moveit::planning_interface::MoveItCpp& moveit::planning_interface::MoveItCpp::
operator=(MoveItCpp&& other)
{
  if (this != &other)
  {
    this->group_name_ = other.group_name_;
    this->robot_description_ = other.robot_description_;
    this->node_handle_ = other.node_handle_;
    this->tf_buffer_ = other.tf_buffer_;
    this->robot_model_ = other.robot_model_;
    this->current_state_monitor_ = other.current_state_monitor_;
    swap(this->move_action_client_, other.move_action_client_);
    swap(this->execute_action_client_, other.execute_action_client_);
    swap(this->pick_action_client_, other.pick_action_client_);
    swap(this->place_action_client_, other.place_action_client_);

    this->considered_start_state_ = other.considered_start_state_;
    this->workspace_parameters_ = other.workspace_parameters_;
    this->allowed_planning_time_ = other.allowed_planning_time_;
    this->planner_id_ = other.planner_id_;
    this->num_planning_attempts_ = other.num_planning_attempts_;
    this->max_velocity_scaling_factor_ = other.max_velocity_scaling_factor_;
    this->max_acceleration_scaling_factor_ = other.max_acceleration_scaling_factor_;
    this->goal_joint_tolerance_ = other.goal_joint_tolerance_;
    this->goal_position_tolerance_ = other.goal_position_tolerance_;
    this->goal_orientation_tolerance_ = other.goal_orientation_tolerance_;
    this->can_look_ = other.can_look_;
    this->can_replan_ = other.can_replan_;
    this->replan_delay_ = other.replan_delay_;

    this->joint_state_target_ = other.joint_state_target_;
    this->joint_model_group_ = other.joint_model_group_;

    this->pose_targets_ = other.pose_targets_;

    this->active_target_ = other.active_target_;
    swap(this->path_constraints_, other.path_constraints_);
    swap(this->trajectory_constraints_, other.trajectory_constraints_);
    this->end_effector_link_ = other.end_effector_link_;
    this->pose_reference_frame_ = other.pose_reference_frame_;
    this->support_surface_ = other.support_surface_;

    this->trajectory_event_publisher_ = other.trajectory_event_publisher_;
    this->attached_object_publisher_ = other.attached_object_publisher_;
    this->query_service_ = other.query_service_;
    this->get_params_service_ = other.get_params_service_;
    this->set_params_service_ = other.set_params_service_;
    this->cartesian_path_service_ = other.cartesian_path_service_;
    this->plan_grasps_service_ = other.plan_grasps_service_;
    swap(this->constraints_storage_, other.constraints_storage_);
    swap(this->constraints_init_thread_, other.constraints_init_thread_);
    this->initializing_constraints_ = other.initializing_constraints_;

    remembered_joint_values_ = std::move(other.remembered_joint_values_);
    other.clearContents();
  }

  return *this;
}

const std::string& moveit::planning_interface::MoveItCpp::getName() const
{
  return group_name_;
}

const std::vector<std::string> moveit::planning_interface::MoveItCpp::getNamedTargets()
{
  const robot_model::RobotModelConstPtr& robot = getRobotModel();
  const std::string& group = getName();
  const robot_model::JointModelGroup* joint_group = robot->getJointModelGroup(group);

  if (joint_group)
  {
    return joint_group->getDefaultStateNames();
  }

  std::vector<std::string> empty;
  return empty;
}

robot_model::RobotModelConstPtr moveit::planning_interface::MoveItCpp::getRobotModel() const
{
  return robot_model_;
}

const ros::NodeHandle& moveit::planning_interface::MoveItCpp::getNodeHandle() const
{
  return node_handle_;
}

bool moveit::planning_interface::MoveItCpp::getInterfaceDescription(
    moveit_msgs::PlannerInterfaceDescription& desc)
{
  moveit_msgs::QueryPlannerInterfaces::Request req;
  moveit_msgs::QueryPlannerInterfaces::Response res;
  if (query_service_.call(req, res))
    if (!res.planner_interfaces.empty())
    {
      desc = res.planner_interfaces.front();
      return true;
    }
  return false;
}

std::map<std::string, std::string> moveit::planning_interface::MoveItCpp::getPlannerParams(
    const std::string& planner_id, const std::string& group)
{
  moveit_msgs::GetPlannerParams::Request req;
  moveit_msgs::GetPlannerParams::Response res;
  req.planner_config = planner_id;
  req.group = group;
  std::map<std::string, std::string> result;
  if (get_params_service_.call(req, res))
  {
    for (unsigned int i = 0, end = res.params.keys.size(); i < end; ++i)
      result[res.params.keys[i]] = res.params.values[i];
  }
  return result;
}

void moveit::planning_interface::MoveItCpp::setPlannerParams(const std::string& planner_id,
                                                                      const std::string& group,
                                                                      const std::map<std::string, std::string>& params,
                                                                      bool replace)
{
  moveit_msgs::SetPlannerParams::Request req;
  moveit_msgs::SetPlannerParams::Response res;
  req.planner_config = planner_id;
  req.group = group;
  req.replace = replace;
  for (const std::pair<const std::string, std::string>& param : params)
  {
    req.params.keys.push_back(param.first);
    req.params.values.push_back(param.second);
  }
  set_params_service_.call(req, res);
}

std::string moveit::planning_interface::MoveItCpp::getDefaultPlannerId(const std::string& group) const
{
  std::stringstream param_name;
  param_name << "move_group";
  if (!group.empty())
    param_name << "/" << group;
  param_name << "/default_planner_config";

  std::string default_planner_config;
  node_handle_.getParam(param_name.str(), default_planner_config);
  return default_planner_config;
}

void moveit::planning_interface::MoveItCpp::setPlannerId(const std::string& planner_id)
{
  planner_id_ = planner_id;
}

const std::string& moveit::planning_interface::MoveItCpp::getPlannerId() const
{
  return planner_id_;
}

void moveit::planning_interface::MoveItCpp::setNumPlanningAttempts(unsigned int num_planning_attempts)
{
  num_planning_attempts_ = num_planning_attempts;
}

void moveit::planning_interface::MoveItCpp::setMaxVelocityScalingFactor(double max_velocity_scaling_factor)
{
  max_velocity_scaling_factor_ = max_velocity_scaling_factor;
}

void moveit::planning_interface::MoveItCpp::setMaxAccelerationScalingFactor(
    double max_acceleration_scaling_factor)
{
  max_acceleration_scaling_factor_ = max_acceleration_scaling_factor;
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveItCpp::asyncMove()
{
  return move(false);
}

actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>&
moveit::planning_interface::MoveItCpp::getMoveGroupClient() const
{
  return *move_action_client_;
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveItCpp::move()
{
  return move(true);
}

moveit::planning_interface::MoveItErrorCode
moveit::planning_interface::MoveItCpp::asyncExecute(const Plan& plan)
{
  return execute(plan, false);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveItCpp::execute(const Plan& plan)
{
  return execute(plan, true);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveItCpp::plan(Plan& plan)
{
  if (!move_action_client_)
  {
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
  }
  if (!move_action_client_->isServerConnected())
  {
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  moveit_msgs::MoveGroupGoal goal;
  constructGoal(goal);
  goal.planning_options.plan_only = true;
  goal.planning_options.look_around = false;
  goal.planning_options.replan = false;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  move_action_client_->sendGoal(goal);
  if (!move_action_client_->waitForResult())
  {
    ROS_INFO_STREAM_NAMED("move_group_interface", "MoveGroup action returned early");
  }
  if (move_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    plan.trajectory_ = move_action_client_->getResult()->planned_trajectory;
    plan.start_state_ = move_action_client_->getResult()->trajectory_start;
    plan.planning_time_ = move_action_client_->getResult()->planning_time;
    return MoveItErrorCode(move_action_client_->getResult()->error_code);
  }
  else
  {
    ROS_WARN_STREAM_NAMED("move_group_interface", "Fail: " << move_action_client_->getState().toString() << ": "
                          << move_action_client_->getState().getText());
    return MoveItErrorCode(move_action_client_->getResult()->error_code);
  }
}

moveit::planning_interface::MoveItErrorCode
moveit::planning_interface::MoveItCpp::pick(const std::string& object, bool plan_only)
{
  return pick(object, std::vector<moveit_msgs::Grasp>(), plan_only);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveItCpp::pick(
    const std::string& object, const moveit_msgs::Grasp& grasp, bool plan_only)
{
  return pick(object, std::vector<moveit_msgs::Grasp>(1, grasp), plan_only);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveItCpp::pick(
    const std::string& object, const std::vector<moveit_msgs::Grasp>& grasps, bool plan_only)
{
  if (!pick_action_client_)
  {
    ROS_ERROR_STREAM_NAMED("move_group_interface", "Pick action client not found");
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
  }
  if (!pick_action_client_->isServerConnected())
  {
    ROS_ERROR_STREAM_NAMED("move_group_interface", "Pick action server not connected");
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
  }
  moveit_msgs::PickupGoal goal;
  constructGoal(goal, object);
  goal.possible_grasps = grasps;
  goal.planning_options.plan_only = plan_only;
  goal.planning_options.look_around = can_look_;
  goal.planning_options.replan = can_replan_;
  goal.planning_options.replan_delay = replan_delay_;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  pick_action_client_->sendGoal(goal);
  if (!pick_action_client_->waitForResult())
  {
    ROS_INFO_STREAM_NAMED("move_group_interface", "Pickup action returned early");
  }
  if (pick_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    return MoveItErrorCode(pick_action_client_->getResult()->error_code);
  }
  else
  {
    ROS_WARN_STREAM_NAMED("move_group_interface", "Fail: " << pick_action_client_->getState().toString() << ": "
                          << pick_action_client_->getState().getText());
    return MoveItErrorCode(pick_action_client_->getResult()->error_code);
  }
}

moveit::planning_interface::MoveItErrorCode
moveit::planning_interface::MoveItCpp::planGraspsAndPick(const std::string& object, bool plan_only)
{
  if (object.empty())
  {
    return planGraspsAndPick(moveit_msgs::CollisionObject());
  }
  moveit::planning_interface::PlanningSceneInterface psi;

  std::map<std::string, moveit_msgs::CollisionObject> objects = psi.getObjects(std::vector<std::string>(1, object));

  if (objects.empty())
  {
    ROS_ERROR_STREAM_NAMED("move_group_interface", "Asked for grasps for the object '"
                           << object << "', but the object could not be found");
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME);
  }

  return planGraspsAndPick(objects[object], plan_only);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveItCpp::planGraspsAndPick(
    const moveit_msgs::CollisionObject& object, bool plan_only)
{
  if (!plan_grasps_service_)
  {
    ROS_ERROR_STREAM_NAMED("move_group_interface", "Grasp planning service '"
                           << GRASP_PLANNING_SERVICE_NAME
                           << "' is not available."
                           " This has to be implemented and started seperatly.");
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  moveit_msgs::GraspPlanning::Request request;
  moveit_msgs::GraspPlanning::Response response;

  request.group_name = group_name_;
  request.target = object;
  request.support_surfaces.push_back(support_surface_);

  ROS_DEBUG_NAMED("move_group_interface", "Calling grasp planner...");
  if (!plan_grasps_service_.call(request, response) ||
      response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_ERROR_NAMED("move_group_interface", "Grasp planning failed. Unable to pick.");
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  return pick(object.id, response.grasps, plan_only);
}

moveit::planning_interface::MoveItErrorCode
moveit::planning_interface::MoveItCpp::place(const std::string& object, bool plan_only)
{
  return place(object, std::vector<moveit_msgs::PlaceLocation>(), plan_only);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveItCpp::place(
    const std::string& object, const std::vector<moveit_msgs::PlaceLocation>& locations, bool plan_only)
{
  if (!place_action_client_)
  {
    ROS_ERROR_STREAM_NAMED("move_group_interface", "Place action client not found");
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
  }
  if (!place_action_client_->isServerConnected())
  {
    ROS_ERROR_STREAM_NAMED("move_group_interface", "Place action server not connected");
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
  }
  moveit_msgs::PlaceGoal goal;
  constructGoal(goal, object);
  goal.place_locations = locations;
  goal.planning_options.plan_only = plan_only;
  goal.planning_options.look_around = can_look_;
  goal.planning_options.replan = can_replan_;
  goal.planning_options.replan_delay = replan_delay_;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  place_action_client_->sendGoal(goal);
  ROS_DEBUG_NAMED("move_group_interface", "Sent place goal with %d locations", (int)goal.place_locations.size());
  if (!place_action_client_->waitForResult())
  {
    ROS_INFO_STREAM_NAMED("move_group_interface", "Place action returned early");
  }
  if (place_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    return MoveItErrorCode(place_action_client_->getResult()->error_code);
  }
  else
  {
    ROS_WARN_STREAM_NAMED("move_group_interface", "Fail: " << place_action_client_->getState().toString() << ": "
                          << place_action_client_->getState().getText());
    return MoveItErrorCode(place_action_client_->getResult()->error_code);
  }
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveItCpp::place(
    const std::string& object, const std::vector<geometry_msgs::PoseStamped>& poses, bool plan_only)
{
  std::vector<moveit_msgs::PlaceLocation> locations;
  for (const geometry_msgs::PoseStamped& pose : poses)
  {
    moveit_msgs::PlaceLocation location;
    location.pre_place_approach.direction.vector.z = -1.0;
    location.post_place_retreat.direction.vector.x = -1.0;
    location.pre_place_approach.direction.header.frame_id = getRobotModel()->getModelFrame();
    location.post_place_retreat.direction.header.frame_id = end_effector_link_;

    location.pre_place_approach.min_distance = 0.1;
    location.pre_place_approach.desired_distance = 0.2;
    location.post_place_retreat.min_distance = 0.0;
    location.post_place_retreat.desired_distance = 0.2;
    // location.post_place_posture is filled by the pick&place lib with the getDetachPosture from the AttachedBody

    location.place_pose = pose;
    locations.push_back(location);
  }
  ROS_DEBUG_NAMED("move_group_interface", "Move group interface has %u place locations",
                  (unsigned int)locations.size());
  return place(object, locations, plan_only);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveItCpp::place(
    const std::string& object, const geometry_msgs::PoseStamped& pose, bool plan_only)
{
  return place(object, std::vector<geometry_msgs::PoseStamped>(1, pose), plan_only);
}

double moveit::planning_interface::MoveItCpp::computeCartesianPath(
    const std::vector<geometry_msgs::Pose>& waypoints, double eef_step, double jump_threshold,
    moveit_msgs::RobotTrajectory& trajectory, bool avoid_collisions, moveit_msgs::MoveItErrorCodes* error_code)
{
  moveit_msgs::Constraints path_constraints_tmp;
  return computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, path_constraints_tmp, avoid_collisions,
                              error_code);
}

double moveit::planning_interface::MoveItCpp::computeCartesianPath(
    const std::vector<geometry_msgs::Pose>& waypoints, double eef_step, double jump_threshold,
    moveit_msgs::RobotTrajectory& trajectory, const moveit_msgs::Constraints& path_constraints, bool avoid_collisions,
    moveit_msgs::MoveItErrorCodes* error_code)
{
  moveit_msgs::GetCartesianPath::Request req;
  moveit_msgs::GetCartesianPath::Response res;

  if (considered_start_state_)
    robot_state::robotStateToRobotStateMsg(*considered_start_state_, req.start_state);
  else
    req.start_state.is_diff = true;

  req.group_name = group_name_;
  req.header.frame_id = pose_reference_frame_;
  req.header.stamp = ros::Time::now();
  req.waypoints = waypoints;
  req.max_step = eef_step;
  req.jump_threshold = jump_threshold;
  req.path_constraints = path_constraints;
  req.avoid_collisions = avoid_collisions;
  req.link_name = end_effector_link_;

  if (cartesian_path_service_.call(req, res))
  {
    if (error_code)
      *error_code = res.error_code;
    if (res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      trajectory = res.solution;
      return res.fraction;
    }
    else
      {
      return -1.0;
      }
  }
  else
  {
    error_code->val = error_code->FAILURE;
    return -1.0;
  }
}

void moveit::planning_interface::MoveItCpp::stop()
{
  if (trajectory_event_publisher_)
  {
    std_msgs::String event;
    event.data = "stop";
    trajectory_event_publisher_.publish(event);
  }
}

void moveit::planning_interface::MoveItCpp::setStartState(const moveit_msgs::RobotState& start_state)
{
  robot_state::RobotStatePtr rs;
  getCurrentState(rs, 1.0);
  robot_state::robotStateMsgToRobotState(start_state, *rs);
  setStartState(*rs);
}

void moveit::planning_interface::MoveItCpp::setStartState(const robot_state::RobotState& start_state)
{
  considered_start_state_.reset(new robot_state::RobotState(start_state));
}

void moveit::planning_interface::MoveItCpp::setStartStateToCurrentState()
{
  considered_start_state_.reset();
}

void moveit::planning_interface::MoveItCpp::setRandomTarget()
{
  joint_state_target_->setToRandomPositions();
  active_target_=JOINT;
}

const std::vector<std::string>& moveit::planning_interface::MoveItCpp::getJointNames()
{
  return joint_model_group_->getVariableNames();
}

const std::vector<std::string>& moveit::planning_interface::MoveItCpp::getLinkNames()
{
  return joint_model_group_->getLinkModelNames();
}

std::map<std::string, double>
moveit::planning_interface::MoveItCpp::getNamedTargetValues(const std::string& name)
{
  std::map<std::string, std::vector<double> >::const_iterator it = remembered_joint_values_.find(name);
  std::map<std::string, double> positions;

  if (it != remembered_joint_values_.end())
  {
    std::vector<std::string> names = joint_model_group_->getVariableNames();
    for (size_t x = 0; x < names.size(); ++x)
    {
      positions[names[x]] = it->second[x];
    }
  }
  else
  {
    joint_model_group_->getVariableDefaultPositions(name, positions);
  }
  return positions;
}

bool moveit::planning_interface::MoveItCpp::setNamedTarget(const std::string& name)
{
  std::map<std::string, std::vector<double> >::const_iterator it = remembered_joint_values_.find(name);
  if (it != remembered_joint_values_.end())
  {
    return setJointValueTarget(it->second);
  }
  else
  {
    if (joint_state_target_->setToDefaultValues(joint_model_group_, name))
    {
      active_target_ = JOINT;
      return true;
    }
    ROS_ERROR_NAMED("move_group_interface", "The requested named target '%s' does not exist", name.c_str());
    return false;
  }
}

void moveit::planning_interface::MoveItCpp::getJointValueTarget(
    std::vector<double>& group_variable_values) const
{
  joint_state_target_->copyJointGroupPositions(joint_model_group_, group_variable_values);
}

bool moveit::planning_interface::MoveItCpp::setJointValueTarget(const std::vector<double>& joint_values)
{
  if (joint_values.size() != joint_model_group_->getVariableCount())
    return false;
  active_target_ = JOINT;
  joint_state_target_->setJointGroupPositions(joint_model_group_, joint_values);
  return joint_state_target_->satisfiesBounds(joint_model_group_, goal_joint_tolerance_);
}

bool moveit::planning_interface::MoveItCpp::setJointValueTarget(
    const std::map<std::string, double>& variable_values)
{
  const auto& allowed = joint_model_group_->getVariableNames();
  for (const auto& pair : variable_values)
  {
    if (std::find(allowed.begin(), allowed.end(), pair.first) == allowed.end())
    {
      ROS_ERROR_STREAM("joint variable " << pair.first << " is not part of group "
                                         << joint_model_group_->getName());
      return false;
    }
  }

  active_target_ = JOINT;
  joint_state_target_->setVariablePositions(variable_values);
  return joint_state_target_->satisfiesBounds(goal_joint_tolerance_);
}

bool moveit::planning_interface::MoveItCpp::setJointValueTarget(const std::vector<std::string>& variable_names,
                                                                         const std::vector<double>& variable_values)
{
  const auto& allowed = joint_model_group_->getVariableNames();
  for (const auto& variable_name : variable_names)
  {
    if (std::find(allowed.begin(), allowed.end(), variable_name) == allowed.end())
    {
      ROS_ERROR_STREAM("joint variable " << variable_name << " is not part of group "
                                         << joint_model_group_->getName());
      return false;
    }
  }

  active_target_ = JOINT;
  joint_state_target_->setVariablePositions(variable_names, variable_values);
  return joint_state_target_->satisfiesBounds(goal_joint_tolerance_);
}

bool moveit::planning_interface::MoveItCpp::setJointValueTarget(const robot_state::RobotState& rstate)
{
  active_target_ = JOINT;
  *joint_state_target_ = rstate;
  return joint_state_target_->satisfiesBounds(goal_joint_tolerance_);
}

bool moveit::planning_interface::MoveItCpp::setJointValueTarget(const std::string& joint_name, double value)
{
  std::vector<double> values(1, value);
  return setJointValueTarget(joint_name, values);
}

bool moveit::planning_interface::MoveItCpp::setJointValueTarget(const std::string& joint_name,
                                                                         const std::vector<double>& values)
{
  active_target_ = JOINT;
  const robot_model::JointModel* jm = joint_model_group_->getJointModel(joint_name);
  if (jm && jm->getVariableCount() == values.size())
  {
    joint_state_target_->setJointPositions(jm, values);
    return joint_state_target_->satisfiesBounds(jm, goal_joint_tolerance_);
  }

  ROS_ERROR_STREAM("joint " << joint_name << " is not part of group " << joint_model_group_->getName());
  return false;
}

bool moveit::planning_interface::MoveItCpp::setJointValueTarget(const sensor_msgs::JointState& state)
{
  return setJointValueTarget(state.name, state.position);
}

bool moveit::planning_interface::MoveItCpp::setJointValueTarget(const geometry_msgs::Pose& eef_pose,
                                                                         const std::string& end_effector_link)
{
  return setJointValueTarget(eef_pose, end_effector_link, "", false);
}

bool moveit::planning_interface::MoveItCpp::setJointValueTarget(const geometry_msgs::PoseStamped& eef_pose,
                                                                         const std::string& end_effector_link)
{
  return setJointValueTarget(eef_pose.pose, end_effector_link, eef_pose.header.frame_id, false);
}

bool moveit::planning_interface::MoveItCpp::setJointValueTarget(const Eigen::Isometry3d& eef_pose,
                                                                         const std::string& end_effector_link)
{
  geometry_msgs::Pose msg = tf2::toMsg(eef_pose);
  return setJointValueTarget(msg, end_effector_link);
}

bool moveit::planning_interface::MoveItCpp::setApproximateJointValueTarget(
    const geometry_msgs::Pose& eef_pose, const std::string& end_effector_link)
{
  return setJointValueTarget(eef_pose, end_effector_link, "", true);
}

bool moveit::planning_interface::MoveItCpp::setApproximateJointValueTarget(
    const geometry_msgs::PoseStamped& eef_pose, const std::string& end_effector_link)
{
  return setJointValueTarget(eef_pose.pose, end_effector_link, eef_pose.header.frame_id, true);
}

bool moveit::planning_interface::MoveItCpp::setApproximateJointValueTarget(
    const Eigen::Isometry3d& eef_pose, const std::string& end_effector_link)
{
  geometry_msgs::Pose msg = tf2::toMsg(eef_pose);
  return setApproximateJointValueTarget(msg, end_effector_link);
}

const robot_state::RobotState& moveit::planning_interface::MoveItCpp::getJointValueTarget() const
{
  return *joint_state_target_;
}

const robot_state::RobotState& moveit::planning_interface::MoveItCpp::getTargetRobotState() const
{
  return *joint_state_target_;
}

const std::string& moveit::planning_interface::MoveItCpp::getEndEffectorLink() const
{
  return end_effector_link_;
}

const std::string& moveit::planning_interface::MoveItCpp::getEndEffector() const
{
  if (!end_effector_link_.empty())
  {
    const std::vector<std::string>& possible_eefs =
      getRobotModel()->getJointModelGroup(group_name_)->getAttachedEndEffectorNames();
    for (const std::string& possible_eef : possible_eefs)
      if (getRobotModel()->getEndEffector(possible_eef)->hasLinkModel(end_effector_link_))
        return possible_eef;
  }
  static std::string empty;
  return empty;
}

bool moveit::planning_interface::MoveItCpp::setEndEffectorLink(const std::string& link_name)
{
  if (end_effector_link_.empty() || link_name.empty())
    return false;
  end_effector_link_ = link_name;
  active_target_ = POSE;
  return true;
}

bool moveit::planning_interface::MoveItCpp::setEndEffector(const std::string& eef_name)
{
  const robot_model::JointModelGroup* jmg = robot_model_->getEndEffector(eef_name);
  if (jmg)
    return setEndEffectorLink(jmg->getEndEffectorParentGroup().second);
  return false;
}

void moveit::planning_interface::MoveItCpp::clearPoseTarget(const std::string& end_effector_link)
{
  pose_targets_.erase(end_effector_link);
}

void moveit::planning_interface::MoveItCpp::clearPoseTargets()
{
  pose_targets_.clear();
}

bool moveit::planning_interface::MoveItCpp::setPoseTarget(const Eigen::Isometry3d& pose,
                                                                   const std::string& end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> pose_msg(1);
  pose_msg[0].pose = tf2::toMsg(pose);
  pose_msg[0].header.frame_id = getPoseReferenceFrame();
  pose_msg[0].header.stamp = ros::Time::now();
  return setPoseTargets(pose_msg, end_effector_link);
}

bool moveit::planning_interface::MoveItCpp::setPoseTarget(const geometry_msgs::Pose& target,
                                                                   const std::string& end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> pose_msg(1);
  pose_msg[0].pose = target;
  pose_msg[0].header.frame_id = getPoseReferenceFrame();
  pose_msg[0].header.stamp = ros::Time::now();
  return setPoseTargets(pose_msg, end_effector_link);
}

bool moveit::planning_interface::MoveItCpp::setPoseTarget(const geometry_msgs::PoseStamped& target,
                                                                   const std::string& end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> targets(1, target);
  return setPoseTargets(targets, end_effector_link);
}

bool moveit::planning_interface::MoveItCpp::setPoseTargets(const EigenSTL::vector_Isometry3d& target,
                                                                    const std::string& end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> pose_out(target.size());
  ros::Time tm = ros::Time::now();
  const std::string& frame_id = getPoseReferenceFrame();
  for (std::size_t i = 0; i < target.size(); ++i)
  {
    pose_out[i].pose = tf2::toMsg(target[i]);
    pose_out[i].header.stamp = tm;
    pose_out[i].header.frame_id = frame_id;
  }
  return setPoseTargets(pose_out, end_effector_link);
}

bool moveit::planning_interface::MoveItCpp::setPoseTargets(const std::vector<geometry_msgs::Pose>& target,
                                                                    const std::string& end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> target_stamped(target.size());
  ros::Time tm = ros::Time::now();
  const std::string& frame_id = getPoseReferenceFrame();
  for (std::size_t i = 0; i < target.size(); ++i)
  {
    target_stamped[i].pose = target[i];
    target_stamped[i].header.stamp = tm;
    target_stamped[i].header.frame_id = frame_id;
  }
  return setPoseTargets(target_stamped, end_effector_link);
}

bool moveit::planning_interface::MoveItCpp::setPoseTargets(const std::vector<geometry_msgs::PoseStamped>& target, const std::string& end_effector_link)
{
  if (target.empty())
  {
    ROS_ERROR_NAMED("move_group_interface", "No pose specified as goal target");
    return false;
  }
  else
  {
    active_target_ = POSE;
    const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;
    if (eef.empty())
    {
      ROS_ERROR_NAMED("move_group_interface", "No end-effector to set the pose for");
      return false;
    }
    else
    {
      pose_targets_[eef] = target;
      // make sure we don't store an actual stamp, since that will become stale can potentially cause tf errors
      std::vector<geometry_msgs::PoseStamped>& stored_poses = pose_targets_[eef];
      for (geometry_msgs::PoseStamped& stored_pose : stored_poses)
        stored_pose.header.stamp = ros::Time(0);
    }
    return true;
  }
}

const geometry_msgs::PoseStamped& moveit::planning_interface::MoveItCpp::getPoseTarget(const std::string& end_effector_link) const
{
  const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;

  // if multiple pose targets are set, return the first one
  std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator jt = pose_targets_.find(eef);
  if (jt != pose_targets_.end())
    if (!jt->second.empty())
      return jt->second.at(0);

  // or return an error
  static const geometry_msgs::PoseStamped UNKNOWN;
  ROS_ERROR_NAMED("move_group_interface", "Pose for end-effector '%s' not known.", eef.c_str());
  return UNKNOWN;
}

const std::vector<geometry_msgs::PoseStamped>& moveit::planning_interface::MoveItCpp::getPoseTargets(const std::string& end_effector_link) const
{
  const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;

  std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator jt = pose_targets_.find(eef);
  if (jt != pose_targets_.end())
    if (!jt->second.empty())
      return jt->second;

  // or return an error
  static const std::vector<geometry_msgs::PoseStamped> EMPTY;
  ROS_ERROR_NAMED("move_group_interface", "Poses for end-effector '%s' are not known.", eef.c_str());
  return EMPTY;
}

namespace
{
inline void transformPose(const tf2_ros::Buffer& tf_buffer, const std::string& desired_frame,
                          geometry_msgs::PoseStamped& target)
{
  if (desired_frame != target.header.frame_id)
  {
    geometry_msgs::PoseStamped target_in(target);
    tf_buffer.transform(target_in, target, desired_frame);
    // we leave the stamp to ros::Time(0) on purpose
    target.header.stamp = ros::Time(0);
  }
}
}  // namespace

bool moveit::planning_interface::MoveItCpp::setPositionTarget(double x, double y, double z,
                                                                       const std::string& end_effector_link)
{
  geometry_msgs::PoseStamped target;
  if (hasPoseTarget(end_effector_link))
  {
    target = getPoseTarget(end_effector_link);
    transformPose(*tf_buffer_, pose_reference_frame_, target);
  }
  else
  {
    target.pose.orientation.x = 0.0;
    target.pose.orientation.y = 0.0;
    target.pose.orientation.z = 0.0;
    target.pose.orientation.w = 1.0;
    target.header.frame_id = pose_reference_frame_;
  }

  target.pose.position.x = x;
  target.pose.position.y = y;
  target.pose.position.z = z;
  bool result = setPoseTarget(target, end_effector_link);
  active_target_ = POSITION;
  return result;
}

bool moveit::planning_interface::MoveItCpp::setRPYTarget(double r, double p, double y,
                                                                  const std::string& end_effector_link)
{
  geometry_msgs::PoseStamped target;
  if (hasPoseTarget(end_effector_link))
  {
    target = getPoseTarget(end_effector_link);
    transformPose(*tf_buffer_, pose_reference_frame_, target);
  }
  else
  {
    target.pose.position.x = 0.0;
    target.pose.position.y = 0.0;
    target.pose.position.z = 0.0;
    target.header.frame_id = pose_reference_frame_;
  }
  tf2::Quaternion q;
  q.setRPY(r, p, y);
  target.pose.orientation = tf2::toMsg(q);
  bool result = setPoseTarget(target, end_effector_link);
  active_target_ = ORIENTATION;
  return result;
}

bool moveit::planning_interface::MoveItCpp::setOrientationTarget(double x, double y, double z, double w,
                                                                          const std::string& end_effector_link)
{
  geometry_msgs::PoseStamped target;
  if (hasPoseTarget(end_effector_link))
  {
    target = getPoseTarget(end_effector_link);
    transformPose(*tf_buffer_, pose_reference_frame_, target);
  }
  else
  {
    target.pose.position.x = 0.0;
    target.pose.position.y = 0.0;
    target.pose.position.z = 0.0;
    target.header.frame_id = pose_reference_frame_;
  }

  target.pose.orientation.x = x;
  target.pose.orientation.y = y;
  target.pose.orientation.z = z;
  target.pose.orientation.w = w;
  bool result = setPoseTarget(target, end_effector_link);
  active_target_ = ORIENTATION;
  return result;
}

void moveit::planning_interface::MoveItCpp::setPoseReferenceFrame(const std::string& pose_reference_frame)
{
  pose_reference_frame_ = pose_reference_frame;
}

const std::string& moveit::planning_interface::MoveItCpp::getPoseReferenceFrame() const
{
  return pose_reference_frame_;
}

double moveit::planning_interface::MoveItCpp::getGoalJointTolerance() const
{
  return goal_joint_tolerance_;
}

double moveit::planning_interface::MoveItCpp::getGoalPositionTolerance() const
{
  return goal_position_tolerance_;
}

double moveit::planning_interface::MoveItCpp::getGoalOrientationTolerance() const
{
  return goal_orientation_tolerance_;
}

void moveit::planning_interface::MoveItCpp::setGoalTolerance(double tolerance)
{
  setGoalJointTolerance(tolerance);
  setGoalPositionTolerance(tolerance);
  setGoalOrientationTolerance(tolerance);
}

void moveit::planning_interface::MoveItCpp::setGoalJointTolerance(double tolerance)
{
  goal_joint_tolerance_ = tolerance;
}

void moveit::planning_interface::MoveItCpp::setGoalPositionTolerance(double tolerance)
{
  goal_position_tolerance_ = tolerance;
}

void moveit::planning_interface::MoveItCpp::setGoalOrientationTolerance(double tolerance)
{
  goal_orientation_tolerance_ = tolerance;
}

void moveit::planning_interface::MoveItCpp::rememberJointValues(const std::string& name)
{
  rememberJointValues(name, getCurrentJointValues());
}

bool moveit::planning_interface::MoveItCpp::startStateMonitor(double wait)
{
  if (!current_state_monitor_)
  {
    ROS_ERROR_NAMED("move_group_interface", "Unable to monitor current robot state");
    return false;
  }

  // if needed, start the monitor and wait up to 1 second for a full robot state
  if (!current_state_monitor_->isActive())
    current_state_monitor_->startStateMonitor();

  current_state_monitor_->waitForCompleteState(group_name_, wait);
  return true;
}

std::vector<double> moveit::planning_interface::MoveItCpp::getCurrentJointValues()
{
  robot_state::RobotStatePtr current_state;
  std::vector<double> values;
  if (getCurrentState(current_state, 1.0))
    current_state->copyJointGroupPositions(getName(), values);
  return values;
}

std::vector<double> moveit::planning_interface::MoveItCpp::getRandomJointValues()
{
  std::vector<double> r;
  joint_model_group_->getVariableRandomPositions(joint_state_target_->getRandomNumberGenerator(), r);
  return r;
}

geometry_msgs::PoseStamped
moveit::planning_interface::MoveItCpp::getRandomPose(const std::string& end_effector_link)
{
  const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  Eigen::Isometry3d pose;
  pose.setIdentity();
  if (eef.empty())
    ROS_ERROR_NAMED("move_group_interface", "No end-effector specified");
  else
  {
    robot_state::RobotStatePtr current_state;
    if (getCurrentState(current_state, 1.0))
    {
      current_state->setToRandomPositions(joint_model_group_);
      const robot_model::LinkModel* lm = current_state->getLinkModel(eef);
      if (lm)
        pose = current_state->getGlobalLinkTransform(lm);
    }
  }
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = robot_model_->getModelFrame();
  pose_msg.pose = tf2::toMsg(pose);
  return pose_msg;
}

geometry_msgs::PoseStamped
moveit::planning_interface::MoveItCpp::getCurrentPose(const std::string& end_effector_link)
{
  const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  Eigen::Isometry3d pose;
  pose.setIdentity();
  if (eef.empty())
    ROS_ERROR_NAMED("move_group_interface", "No end-effector specified");
  else
  {
    robot_state::RobotStatePtr current_state;
    if (getCurrentState(current_state, 1.0))
    {
      const robot_model::LinkModel* lm = current_state->getLinkModel(eef);
      if (lm)
        pose = current_state->getGlobalLinkTransform(lm);
    }
  }
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = robot_model_->getModelFrame();
  pose_msg.pose = tf2::toMsg(pose);
  return pose_msg;
}

std::vector<double> moveit::planning_interface::MoveItCpp::getCurrentRPY(const std::string& end_effector_link)
{
  std::vector<double> result;
  const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  if (eef.empty())
    ROS_ERROR_NAMED("move_group_interface", "No end-effector specified");
  else
  {
    robot_state::RobotStatePtr current_state;
    if (getCurrentState(current_state, 1.0))
    {
      const robot_model::LinkModel* lm = current_state->getLinkModel(eef);
      if (lm)
      {
        result.resize(3);
        geometry_msgs::TransformStamped tfs = tf2::eigenToTransform(current_state->getGlobalLinkTransform(lm));
        double pitch, roll, yaw;
        tf2::getEulerYPR<geometry_msgs::Quaternion>(tfs.transform.rotation, yaw, pitch, roll);
        result[0] = roll;
        result[1] = pitch;
        result[2] = yaw;
      }
    }
  }
  return result;
}

const std::vector<std::string>& moveit::planning_interface::MoveItCpp::getActiveJoints() const
{
  return joint_model_group_->getActiveJointModelNames();
}

const std::vector<std::string>& moveit::planning_interface::MoveItCpp::getJoints() const
{
  return joint_model_group_->getJointModelNames();
}

unsigned int moveit::planning_interface::MoveItCpp::getVariableCount() const
{
  return joint_model_group_->getVariableCount();
}

robot_state::RobotStatePtr moveit::planning_interface::MoveItCpp::getCurrentState(double wait)
{
  robot_state::RobotStatePtr current_state;
  getCurrentState(current_state, wait);
  return current_state;
}

void moveit::planning_interface::MoveItCpp::rememberJointValues(const std::string& name,
                                                                         const std::vector<double>& values)
{
  remembered_joint_values_[name] = values;
}

void moveit::planning_interface::MoveItCpp::forgetJointValues(const std::string& name)
{
  remembered_joint_values_.erase(name);
}

void moveit::planning_interface::MoveItCpp::allowLooking(bool flag)
{
  can_look_ = flag;
  ROS_INFO_NAMED("move_group_interface", "Looking around: %s", can_look_ ? "yes" : "no");
}

void moveit::planning_interface::MoveItCpp::allowReplanning(bool flag)
{
  can_replan_ = flag;
  ROS_INFO_NAMED("move_group_interface", "Replanning: %s", can_replan_ ? "yes" : "no");
}

std::vector<std::string> moveit::planning_interface::MoveItCpp::getKnownConstraints() const
{
  while (initializing_constraints_)
  {
    static ros::WallDuration d(0.01);
    d.sleep();
  }

  std::vector<std::string> c;
  if (constraints_storage_)
    constraints_storage_->getKnownConstraints(c, robot_model_->getName(), group_name_);

  return c;
}

moveit_msgs::Constraints moveit::planning_interface::MoveItCpp::getPathConstraints() const
{
  if (path_constraints_)
    return *path_constraints_;
  else
    return moveit_msgs::Constraints();
}

bool moveit::planning_interface::MoveItCpp::setPathConstraints(const std::string& constraint)
{
  if (constraints_storage_)
  {
    moveit_warehouse::ConstraintsWithMetadata msg_m;
    if (constraints_storage_->getConstraints(msg_m, constraint, robot_model_->getName(), group_name_))
    {
      path_constraints_.reset(new moveit_msgs::Constraints(static_cast<moveit_msgs::Constraints>(*msg_m)));
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

void moveit::planning_interface::MoveItCpp::setPathConstraints(const moveit_msgs::Constraints& constraint)
{
  path_constraints_.reset(new moveit_msgs::Constraints(constraint));
}

void moveit::planning_interface::MoveItCpp::clearPathConstraints()
{
  path_constraints_.reset();
}

moveit_msgs::TrajectoryConstraints moveit::planning_interface::MoveItCpp::getTrajectoryConstraints() const
{
  if (trajectory_constraints_)
    return *trajectory_constraints_;
  else
    return moveit_msgs::TrajectoryConstraints();
}

void moveit::planning_interface::MoveItCpp::setTrajectoryConstraints(
    const moveit_msgs::TrajectoryConstraints& constraint)
{
  trajectory_constraints_.reset(new moveit_msgs::TrajectoryConstraints(constraint));
}

void moveit::planning_interface::MoveItCpp::clearTrajectoryConstraints()
{
  trajectory_constraints_.reset();
}

void moveit::planning_interface::MoveItCpp::setConstraintsDatabase(const std::string& host, unsigned int port)
{
  initializing_constraints_ = true;
  if (constraints_init_thread_)
    constraints_init_thread_->join();
  constraints_init_thread_.reset(new boost::thread(boost::bind(&MoveItCpp::initializeConstraintsStorageThread, this, host, port)));
}

void moveit::planning_interface::MoveItCpp::setWorkspace(double minx, double miny, double minz, double maxx,
                                                                  double maxy, double maxz)
{
  workspace_parameters_.header.frame_id = getRobotModel()->getModelFrame();
  workspace_parameters_.header.stamp = ros::Time::now();
  workspace_parameters_.min_corner.x = minx;
  workspace_parameters_.min_corner.y = miny;
  workspace_parameters_.min_corner.z = minz;
  workspace_parameters_.max_corner.x = maxx;
  workspace_parameters_.max_corner.y = maxy;
  workspace_parameters_.max_corner.z = maxz;
}

/** \brief Set time allowed to planner to solve problem before aborting */
void moveit::planning_interface::MoveItCpp::setPlanningTime(double seconds)
{
  if (seconds > 0.0)
    allowed_planning_time_ = seconds;
}

/** \brief Get time allowed to planner to solve problem before aborting */
double moveit::planning_interface::MoveItCpp::getPlanningTime() const
{
  return allowed_planning_time_;
}

void moveit::planning_interface::MoveItCpp::setSupportSurfaceName(const std::string& name)
{
  support_surface_ = name;
}

const std::string& moveit::planning_interface::MoveItCpp::getPlanningFrame() const
{
  return robot_model_->getModelFrame();
}

const std::vector<std::string>& moveit::planning_interface::MoveItCpp::getJointModelGroupNames() const
{
  return robot_model_->getJointModelGroupNames();
}

bool moveit::planning_interface::MoveItCpp::attachObject(const std::string& object, const std::string& link)
{
  return attachObject(object, link, std::vector<std::string>());
}

bool moveit::planning_interface::MoveItCpp::attachObject(const std::string& object, const std::string& link,
                                                                  const std::vector<std::string>& touch_links)
{
  std::string l = link.empty() ? getEndEffectorLink() : link;
  if (l.empty())
  {
    const std::vector<std::string>& links = joint_model_group_->getLinkModelNames();
    if (!links.empty())
      l = links[0];
  }
  if (l.empty())
  {
    ROS_ERROR_NAMED("move_group_interface", "No known link to attach object '%s' to", object.c_str());
    return false;
  }
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.id = object;
  aco.link_name.swap(l);
  if (touch_links.empty())
    aco.touch_links.push_back(aco.link_name);
  else
    aco.touch_links = touch_links;
  aco.object.operation = moveit_msgs::CollisionObject::ADD;
  attached_object_publisher_.publish(aco);
  return true;
}

bool moveit::planning_interface::MoveItCpp::detachObject(const std::string& name)
{
  moveit_msgs::AttachedCollisionObject aco;
  // if name is a link
  if (!name.empty() && joint_model_group_->hasLinkModel(name))
    aco.link_name = name;
  else
    aco.object.id = name;
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
  if (aco.link_name.empty() && aco.object.id.empty())
  {
    // we only want to detach objects for this group
    const std::vector<std::string>& lnames = joint_model_group_->getLinkModelNames();
    for (const std::string& lname : lnames)
    {
      aco.link_name = lname;
      attached_object_publisher_.publish(aco);
    }
  }
  else
    attached_object_publisher_.publish(aco);
  return true;
}

void moveit::planning_interface::MoveItCpp::constructMotionPlanRequest(
    moveit_msgs::MotionPlanRequest& request)
{
  request.group_name = group_name_;
  request.num_planning_attempts = num_planning_attempts_;
  request.max_velocity_scaling_factor = max_velocity_scaling_factor_;
  request.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;
  request.allowed_planning_time = allowed_planning_time_;
  request.planner_id = planner_id_;
  request.workspace_parameters = workspace_parameters_;

  if (considered_start_state_)
    robot_state::robotStateToRobotStateMsg(*considered_start_state_, request.start_state);
  else
    request.start_state.is_diff = true;

  if (active_target_ == JOINT)
  {
    request.goal_constraints.resize(1);
    request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
                                                                                  getTargetRobotState(), joint_model_group_, goal_joint_tolerance_);
  }
  else if (active_target_ == POSE || active_target_ == POSITION || active_target_ == ORIENTATION)
  {
    // find out how many goals are specified
    std::size_t goal_count = 0;
    for (std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator it = pose_targets_.begin(); it != pose_targets_.end(); ++it)
      goal_count = std::max(goal_count, it->second.size());

    // start filling the goals;
    // each end effector has a number of possible poses (K) as valid goals
    // but there could be multiple end effectors specified, so we want each end effector
    // to reach the goal that corresponds to the goals of the other end effectors
    request.goal_constraints.resize(goal_count);

    for (std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator it = pose_targets_.begin(); it != pose_targets_.end(); ++it)
      {
        for (std::size_t i = 0; i < it->second.size(); ++i)
        {
          moveit_msgs::Constraints c = kinematic_constraints::constructGoalConstraints(
              it->first, it->second[i], goal_position_tolerance_, goal_orientation_tolerance_);
          if (active_target_ == ORIENTATION)
            c.position_constraints.clear();
          if (active_target_ == POSITION)
            c.orientation_constraints.clear();
          request.goal_constraints[i] = kinematic_constraints::mergeConstraints(request.goal_constraints[i], c);
        }
      }
    }
    else
      ROS_ERROR_NAMED("move_group_interface", "Unable to construct MotionPlanRequest representation");

    if (path_constraints_)
      request.path_constraints = *path_constraints_;
    if (trajectory_constraints_)
      request.trajectory_constraints = *trajectory_constraints_;
}

template <typename T>
void moveit::planning_interface::MoveItCpp::waitForAction(const T& action, const std::string& name, const ros::WallTime& timeout, double allotted_time)
{
  ROS_DEBUG_NAMED("move_group_interface", "Waiting for move_group action server (%s)...", name.c_str());

  // wait for the server (and spin as needed)
  if (timeout == ros::WallTime())  // wait forever
  {
    while (node_handle_.ok() && !action->isServerConnected())
    {
      ros::WallDuration(0.001).sleep();
      // explicit ros::spinOnce on the callback queue used by NodeHandle that manages the action client
      ros::CallbackQueue* queue = dynamic_cast<ros::CallbackQueue*>(node_handle_.getCallbackQueue());
      if (queue)
      {
        queue->callAvailable();
      }
      else  // in case of nodelets and specific callback queue implementations
      {
        ROS_WARN_ONCE_NAMED("move_group_interface", "Non-default CallbackQueue: Waiting for external queue "
                            "handling.");
      }
    }
  }
  else  // wait with timeout
  {
    while (node_handle_.ok() && !action->isServerConnected() && timeout > ros::WallTime::now())
    {
      ros::WallDuration(0.001).sleep();
      // explicit ros::spinOnce on the callback queue used by NodeHandle that manages the action client
      ros::CallbackQueue* queue = dynamic_cast<ros::CallbackQueue*>(node_handle_.getCallbackQueue());
      if (queue)
      {
        queue->callAvailable();
      }
      else  // in case of nodelets and specific callback queue implementations
      {
        ROS_WARN_ONCE_NAMED("move_group_interface", "Non-default CallbackQueue: Waiting for external queue "
                            "handling.");
      }
    }
  }

  if (!action->isServerConnected())
  {
    std::stringstream error;
    error << "Unable to connect to move_group action server '" << name << "' within allotted time (" << allotted_time
          << "s)";
    throw std::runtime_error(error.str());
  }
  else
  {
    ROS_DEBUG_NAMED("move_group_interface", "Connected to '%s'", name.c_str());
  }
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveItCpp::move(bool wait)
{
  if (!move_action_client_)
  {
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
  }
  if (!move_action_client_->isServerConnected())
  {
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  moveit_msgs::MoveGroupGoal goal;
  constructGoal(goal);
  goal.planning_options.plan_only = false;
  goal.planning_options.look_around = can_look_;
  goal.planning_options.replan = can_replan_;
  goal.planning_options.replan_delay = replan_delay_;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  move_action_client_->sendGoal(goal);
  if (!wait)
  {
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::SUCCESS);
  }

  if (!move_action_client_->waitForResult())
  {
    ROS_INFO_STREAM_NAMED("move_group_interface", "MoveGroup action returned early");
  }
  if (!move_action_client_->waitForResult())
  {
    ROS_INFO_STREAM_NAMED("move_group_interface", "MoveGroup action returned early");
  }

  if (move_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    return MoveItErrorCode(move_action_client_->getResult()->error_code);
  }
  else
  {
    ROS_INFO_STREAM_NAMED("move_group_interface", move_action_client_->getState().toString()
                          << ": " << move_action_client_->getState().getText());
    return MoveItErrorCode(move_action_client_->getResult()->error_code);
  }

}

void moveit::planning_interface::MoveItCpp::constructGoal(moveit_msgs::MoveGroupGoal& goal)
{
  constructMotionPlanRequest(goal.request);
}


void moveit::planning_interface::MoveItCpp::constructGoal(moveit_msgs::PickupGoal& goal_out, const std::string& object)
{
  moveit_msgs::PickupGoal goal;
  goal.target_name = object;
  goal.group_name = group_name_;
  goal.end_effector = getEndEffector();
  goal.allowed_planning_time = allowed_planning_time_;
  goal.support_surface_name = support_surface_;
  goal.planner_id = planner_id_;
  if (!support_surface_.empty())
    goal.allow_gripper_support_collision = true;

  if (path_constraints_)
    goal.path_constraints = *path_constraints_;

  goal_out = goal;
}

void moveit::planning_interface::MoveItCpp::constructGoal(moveit_msgs::PlaceGoal& goal_out, const std::string& object)
{
  moveit_msgs::PlaceGoal goal;
  goal.attached_object_name = object;
  goal.group_name = group_name_;
  goal.allowed_planning_time = allowed_planning_time_;
  goal.support_surface_name = support_surface_;
  goal.planner_id = planner_id_;
  if (!support_surface_.empty())
    goal.allow_gripper_support_collision = true;

  if (path_constraints_)
    goal.path_constraints = *path_constraints_;

  goal_out = goal;
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveItCpp::execute(const Plan& plan, bool wait)
{
  if (!execute_action_client_->isServerConnected())
  {
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  moveit_msgs::ExecuteTrajectoryGoal goal;
  goal.trajectory = plan.trajectory_;

  execute_action_client_->sendGoal(goal);
  if (!wait)
  {
    return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::SUCCESS);
  }

  if (!execute_action_client_->waitForResult())
  {
    ROS_INFO_STREAM_NAMED("move_group_interface", "ExecuteTrajectory action returned early");
  }

  if (execute_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    return MoveItErrorCode(execute_action_client_->getResult()->error_code);
  }
  else
  {
    ROS_INFO_STREAM_NAMED("move_group_interface", execute_action_client_->getState().toString()
                          << ": " << execute_action_client_->getState().getText());
    return MoveItErrorCode(execute_action_client_->getResult()->error_code);
  }
}

bool moveit::planning_interface::MoveItCpp::getCurrentState(robot_state::RobotStatePtr& current_state, double wait_seconds)
{
  if (!current_state_monitor_)
  {
    ROS_ERROR_NAMED("move_group_interface", "Unable to get current robot state");
    return false;
  }

  // if needed, start the monitor and wait up to 1 second for a full robot state
  if (!current_state_monitor_->isActive())
    current_state_monitor_->startStateMonitor();

  if (!current_state_monitor_->waitForCurrentState(ros::Time::now(), wait_seconds))
  {
    ROS_ERROR_NAMED("move_group_interface", "Failed to fetch current robot state");
    return false;
  }

  current_state = current_state_monitor_->getCurrentState();
  return true;
}

bool moveit::planning_interface::MoveItCpp::setJointValueTarget(const geometry_msgs::Pose& eef_pose, const std::string& end_effector_link, const std::string& frame, bool approx)
{
  const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;
  // this only works if we have an end-effector
  if (!eef.empty())
  {
    // first we set the goal to be the same as the start state
    moveit::core::RobotStatePtr c = getStartState();
    if (c)
    {
      active_target_ = JOINT;
      c->enforceBounds();
      *joint_state_target_ = *c;
      if (!joint_state_target_->satisfiesBounds(goal_joint_tolerance_))
        return false;
    }
    else
      return false;

    // we may need to do approximate IK
    kinematics::KinematicsQueryOptions o;
    o.return_approximate_solution = approx;

    // if no frame transforms are needed, call IK directly
    if (frame.empty() || moveit::core::Transforms::sameFrame(frame, robot_model_->getModelFrame()))
      return joint_state_target_->setFromIK(joint_model_group_, eef_pose, eef, 0.0,
                                             moveit::core::GroupStateValidityCallbackFn(), o);
    else
    {
      if (c->knowsFrameTransform(frame))
      {
        // transform the pose first if possible, then do IK
        const Eigen::Isometry3d& t = joint_state_target_->getFrameTransform(frame);
        Eigen::Isometry3d p;
        tf2::fromMsg(eef_pose, p);
        return joint_state_target_->setFromIK(joint_model_group_, t * p, eef, 0.0,
                                               moveit::core::GroupStateValidityCallbackFn(), o);
      }
      else
      {
        ROS_ERROR_NAMED("move_group_interface", "Unable to transform from frame '%s' to frame '%s'", frame.c_str(),
                        robot_model_->getModelFrame().c_str());
        return false;
      }
    }
  }
  else
    return false;
}

robot_state::RobotStatePtr moveit::planning_interface::MoveItCpp::getStartState()
{
  if (considered_start_state_)
    return considered_start_state_;
  else
  {
    robot_state::RobotStatePtr s;
    getCurrentState(s, 1.0);
    return s;
  }
}

bool moveit::planning_interface::MoveItCpp::hasPoseTarget(const std::string& end_effector_link) const
{
  const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;
  return pose_targets_.find(eef) != pose_targets_.end();
}

void moveit::planning_interface::MoveItCpp::initializeConstraintsStorageThread(const std::string& host, unsigned int port)
{
  // Set up db
  try
  {
    warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase();
    conn->setParams(host, port);
    if (conn->connect())
    {
      constraints_storage_.reset(new moveit_warehouse::ConstraintsStorage(conn));
    }
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED("move_group_interface", "%s", ex.what());
  }
  initializing_constraints_ = false;
}

void moveit::planning_interface::MoveItCpp::clearContents()
{
  //TODO figure out what else needs to be removed/cleaned.
  group_name_.clear();
  robot_description_.clear();
  //node_handle_;
  tf_buffer_ = nullptr;
  //robot_model_
  current_state_monitor_ = nullptr;
  execute_action_client_ = nullptr;
  pick_action_client_ = nullptr;
  place_action_client_ = nullptr;

  considered_start_state_ = nullptr;
  //workspace_parameters_
  //allowed_planning_time_
  planner_id_.clear();
  //num_planning_attempts_
  //max_velocity_scaling_factor_
  //max_acceleration_scaling_factor_
  //goal_joint_tolerance_;
  //goal_position_tolerance_
  //goal_orientation_tolerance_
  //can_look_
  //can_replan_
  //replan_delay_

  joint_state_target_ = nullptr;
  joint_model_group_ = nullptr;

  pose_targets_.clear();

  //active_target_
  path_constraints_ = nullptr;
  trajectory_constraints_ = nullptr;
  end_effector_link_.clear();
  pose_reference_frame_.clear();
  support_surface_.clear();

  //trajectory_event_publisher_
  //attached_object_publisher_
  //query_service_
  //get_params_service_
  //set_params_service_
  //cartesian_path_service_
  //plan_grasps_service_
  constraints_storage_ = nullptr;
  constraints_init_thread_ = nullptr;
  //initializing_constraints_
}
