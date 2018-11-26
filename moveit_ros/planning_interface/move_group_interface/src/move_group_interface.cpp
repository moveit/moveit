/*********************************************************************
 * Software License Agreement (BSD License)
 *
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

/* Author: Ioan Sucan, Sachin Chitta */

#include <stdexcept>
#include <sstream>
#include <memory>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <moveit/move_group_interface/move_group_interface.h>
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

#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/console.h>
#include <ros/ros.h>

namespace moveit
{
namespace planning_interface
{
const std::string MoveGroupInterface::ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

const std::string GRASP_PLANNING_SERVICE_NAME = "plan_grasps";  // name of the service that can be used to plan grasps

namespace
{
enum ActiveTargetType
{
  JOINT,
  POSE,
  POSITION,
  ORIENTATION
};
}

class MoveGroupInterface::MoveGroupInterfaceImpl
{
public:
  MoveGroupInterfaceImpl(const Options& opt, const boost::shared_ptr<tf::Transformer>& tf,
                         const ros::WallDuration& wait_for_servers)
    : opt_(opt), node_handle_(opt.node_handle_), tf_(tf)
  {
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

    trajectory_event_publisher_ = node_handle_.advertise<std_msgs::String>(
        trajectory_execution_manager::TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC, 1, false);
    attached_object_publisher_ = node_handle_.advertise<moveit_msgs::AttachedCollisionObject>(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC, 1, false);

    current_state_monitor_ = getSharedStateMonitor(robot_model_, tf_, node_handle_);

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
    // TODO: after deprecation period, i.e. for L-turtle, switch back to standard waitForAction function
    // waitForAction(execute_action_client_, move_group::EXECUTE_ACTION_NAME, timeout_for_servers, allotted_time);
    waitForExecuteActionOrService(timeout_for_servers);

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
  }

  template <typename T>
  void waitForAction(const T& action, const std::string& name, const ros::WallTime& timeout, double allotted_time)
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

  void waitForExecuteActionOrService(ros::WallTime timeout)
  {
    ROS_DEBUG("Waiting for move_group action server (%s)...", move_group::EXECUTE_ACTION_NAME.c_str());

    // Deprecated service
    execute_service_ =
        node_handle_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(move_group::EXECUTE_SERVICE_NAME);

    // wait for either of action or service
    if (timeout == ros::WallTime())  // wait forever
    {
      while (!execute_action_client_->isServerConnected() && !execute_service_.exists())
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
      while (!execute_action_client_->isServerConnected() && !execute_service_.exists() &&
             timeout > ros::WallTime::now())
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

    // issue warning
    if (!execute_action_client_->isServerConnected())
    {
      if (execute_service_.exists())
      {
        ROS_WARN_NAMED("move_group_interface",
                       "\nDeprecation warning: Trajectory execution service is deprecated (was replaced by an action)."
                       "\nReplace 'MoveGroupExecuteService' with 'MoveGroupExecuteTrajectoryAction' in "
                       "move_group.launch");
      }
      else
      {
        ROS_ERROR_STREAM_NAMED("move_group_interface",
                               "Unable to find execution action on topic: "
                                   << node_handle_.getNamespace() + move_group::EXECUTE_ACTION_NAME << " or service: "
                                   << node_handle_.getNamespace() + move_group::EXECUTE_SERVICE_NAME);
        throw std::runtime_error("No Trajectory execution capability available.");
      }
      execute_action_client_.reset();
    }
  }

  ~MoveGroupInterfaceImpl()
  {
    if (constraints_init_thread_)
      constraints_init_thread_->join();
  }

  const boost::shared_ptr<tf::Transformer>& getTF() const
  {
    return tf_;
  }

  const Options& getOptions() const
  {
    return opt_;
  }

  const robot_model::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  const robot_model::JointModelGroup* getJointModelGroup() const
  {
    return joint_model_group_;
  }

  actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>& getMoveGroupClient() const
  {
    return *move_action_client_;
  }

  bool getInterfaceDescription(moveit_msgs::PlannerInterfaceDescription& desc)
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

  std::map<std::string, std::string> getPlannerParams(const std::string& planner_id, const std::string& group = "")
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

  void setPlannerParams(const std::string& planner_id, const std::string& group,
                        const std::map<std::string, std::string>& params, bool replace = false)
  {
    moveit_msgs::SetPlannerParams::Request req;
    moveit_msgs::SetPlannerParams::Response res;
    req.planner_config = planner_id;
    req.group = group;
    req.replace = replace;
    for (std::map<std::string, std::string>::const_iterator it = params.begin(), end = params.end(); it != end; ++it)
    {
      req.params.keys.push_back(it->first);
      req.params.values.push_back(it->second);
    }
    set_params_service_.call(req, res);
  }

  std::string getDefaultPlannerId(const std::string& group) const
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

  void setPlannerId(const std::string& planner_id)
  {
    planner_id_ = planner_id;
  }

  const std::string& getPlannerId() const
  {
    return planner_id_;
  }

  void setNumPlanningAttempts(unsigned int num_planning_attempts)
  {
    num_planning_attempts_ = num_planning_attempts;
  }

  void setMaxVelocityScalingFactor(double max_velocity_scaling_factor)
  {
    max_velocity_scaling_factor_ = max_velocity_scaling_factor;
  }

  void setMaxAccelerationScalingFactor(double max_acceleration_scaling_factor)
  {
    max_acceleration_scaling_factor_ = max_acceleration_scaling_factor;
  }

  robot_state::RobotState& getJointStateTarget()
  {
    return *joint_state_target_;
  }

  void setStartState(const robot_state::RobotState& start_state)
  {
    considered_start_state_.reset(new robot_state::RobotState(start_state));
  }

  void setStartStateToCurrentState()
  {
    considered_start_state_.reset();
  }

  robot_state::RobotStatePtr getStartState()
  {
    if (considered_start_state_)
      return considered_start_state_;
    else
    {
      robot_state::RobotStatePtr s;
      getCurrentState(s);
      return s;
    }
  }

  bool setJointValueTarget(const geometry_msgs::Pose& eef_pose, const std::string& end_effector_link,
                           const std::string& frame, bool approx)
  {
    const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
    // this only works if we have an end-effector
    if (!eef.empty())
    {
      // first we set the goal to be the same as the start state
      moveit::core::RobotStatePtr c = getStartState();
      if (c)
      {
        setTargetType(JOINT);
        c->enforceBounds();
        getJointStateTarget() = *c;
        if (!getJointStateTarget().satisfiesBounds(getGoalJointTolerance()))
          return false;
      }
      else
        return false;

      // we may need to do approximate IK
      kinematics::KinematicsQueryOptions o;
      o.return_approximate_solution = approx;

      // if no frame transforms are needed, call IK directly
      if (frame.empty() || moveit::core::Transforms::sameFrame(frame, getRobotModel()->getModelFrame()))
        return getJointStateTarget().setFromIK(getJointModelGroup(), eef_pose, eef, 0, 0.0,
                                               moveit::core::GroupStateValidityCallbackFn(), o);
      else
      {
        if (c->knowsFrameTransform(frame))
        {
          // transform the pose first if possible, then do IK
          const Eigen::Affine3d& t = getJointStateTarget().getFrameTransform(frame);
          Eigen::Affine3d p;
          tf::poseMsgToEigen(eef_pose, p);
          return getJointStateTarget().setFromIK(getJointModelGroup(), t * p, eef, 0, 0.0,
                                                 moveit::core::GroupStateValidityCallbackFn(), o);
        }
        else
        {
          ROS_ERROR_NAMED("move_group_interface", "Unable to transform from frame '%s' to frame '%s'", frame.c_str(),
                          getRobotModel()->getModelFrame().c_str());
          return false;
        }
      }
    }
    else
      return false;
  }

  void setEndEffectorLink(const std::string& end_effector)
  {
    end_effector_link_ = end_effector;
  }

  void clearPoseTarget(const std::string& end_effector_link)
  {
    pose_targets_.erase(end_effector_link);
  }

  void clearPoseTargets()
  {
    pose_targets_.clear();
  }

  const std::string& getEndEffectorLink() const
  {
    return end_effector_link_;
  }

  const std::string& getEndEffector() const
  {
    if (!end_effector_link_.empty())
    {
      const std::vector<std::string>& possible_eefs =
          getRobotModel()->getJointModelGroup(opt_.group_name_)->getAttachedEndEffectorNames();
      for (std::size_t i = 0; i < possible_eefs.size(); ++i)
        if (getRobotModel()->getEndEffector(possible_eefs[i])->hasLinkModel(end_effector_link_))
          return possible_eefs[i];
    }
    static std::string empty;
    return empty;
  }

  bool setPoseTargets(const std::vector<geometry_msgs::PoseStamped>& poses, const std::string& end_effector_link)
  {
    const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;
    if (eef.empty())
    {
      ROS_ERROR_NAMED("move_group_interface", "No end-effector to set the pose for");
      return false;
    }
    else
    {
      pose_targets_[eef] = poses;
      // make sure we don't store an actual stamp, since that will become stale can potentially cause tf errors
      std::vector<geometry_msgs::PoseStamped>& stored_poses = pose_targets_[eef];
      for (std::size_t i = 0; i < stored_poses.size(); ++i)
        stored_poses[i].header.stamp = ros::Time(0);
    }
    return true;
  }

  bool hasPoseTarget(const std::string& end_effector_link) const
  {
    const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;
    return pose_targets_.find(eef) != pose_targets_.end();
  }

  const geometry_msgs::PoseStamped& getPoseTarget(const std::string& end_effector_link) const
  {
    const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;

    // if multiple pose targets are set, return the first one
    std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator jt = pose_targets_.find(eef);
    if (jt != pose_targets_.end())
      if (!jt->second.empty())
        return jt->second.at(0);

    // or return an error
    static const geometry_msgs::PoseStamped unknown;
    ROS_ERROR_NAMED("move_group_interface", "Pose for end-effector '%s' not known.", eef.c_str());
    return unknown;
  }

  const std::vector<geometry_msgs::PoseStamped>& getPoseTargets(const std::string& end_effector_link) const
  {
    const std::string& eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;

    std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator jt = pose_targets_.find(eef);
    if (jt != pose_targets_.end())
      if (!jt->second.empty())
        return jt->second;

    // or return an error
    static const std::vector<geometry_msgs::PoseStamped> empty;
    ROS_ERROR_NAMED("move_group_interface", "Poses for end-effector '%s' are not known.", eef.c_str());
    return empty;
  }

  void setPoseReferenceFrame(const std::string& pose_reference_frame)
  {
    pose_reference_frame_ = pose_reference_frame;
  }

  void setSupportSurfaceName(const std::string& support_surface)
  {
    support_surface_ = support_surface;
  }

  const std::string& getPoseReferenceFrame() const
  {
    return pose_reference_frame_;
  }

  void setTargetType(ActiveTargetType type)
  {
    active_target_ = type;
  }

  ActiveTargetType getTargetType() const
  {
    return active_target_;
  }

  bool startStateMonitor(double wait)
  {
    if (!current_state_monitor_)
    {
      ROS_ERROR_NAMED("move_group_interface", "Unable to monitor current robot state");
      return false;
    }

    // if needed, start the monitor and wait up to 1 second for a full robot state
    if (!current_state_monitor_->isActive())
      current_state_monitor_->startStateMonitor();

    current_state_monitor_->waitForCompleteState(opt_.group_name_, wait);
    return true;
  }

  bool getCurrentState(robot_state::RobotStatePtr& current_state, double wait_seconds = 1.0)
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

  /** \brief Place an object at one of the specified possible locations */
  MoveItErrorCode place(const std::string& object, const std::vector<geometry_msgs::PoseStamped>& poses,
                        bool plan_only = false)
  {
    std::vector<moveit_msgs::PlaceLocation> locations;
    for (std::size_t i = 0; i < poses.size(); ++i)
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

      location.place_pose = poses[i];
      locations.push_back(location);
    }
    ROS_DEBUG_NAMED("move_group_interface", "Move group interface has %u place locations",
                    (unsigned int)locations.size());
    return place(object, locations, plan_only);
  }

  MoveItErrorCode place(const std::string& object, const std::vector<moveit_msgs::PlaceLocation>& locations,
                        bool plan_only = false)
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

  MoveItErrorCode pick(const std::string& object, const std::vector<moveit_msgs::Grasp>& grasps, bool plan_only = false)
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

  MoveItErrorCode planGraspsAndPick(const std::string& object, bool plan_only = false)
  {
    if (object.empty())
    {
      return planGraspsAndPick(moveit_msgs::CollisionObject());
    }
    moveit::planning_interface::PlanningSceneInterface psi;

    std::map<std::string, moveit_msgs::CollisionObject> objects = psi.getObjects(std::vector<std::string>(1, object));

    if (objects.size() < 1)
    {
      ROS_ERROR_STREAM_NAMED("move_group_interface", "Asked for grasps for the object '"
                                                         << object << "', but the object could not be found");
      return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME);
    }

    return planGraspsAndPick(objects[object], plan_only);
  }

  MoveItErrorCode planGraspsAndPick(const moveit_msgs::CollisionObject& object, bool plan_only = false)
  {
    if (!plan_grasps_service_)
    {
      ROS_ERROR_STREAM_NAMED("move_group_interface", "Grasp planning service '"
                                                         << GRASP_PLANNING_SERVICE_NAME
                                                         << "' is not available."
                                                            " This has to be implemented and started separately.");
      return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
    }

    moveit_msgs::GraspPlanning::Request request;
    moveit_msgs::GraspPlanning::Response response;

    request.group_name = opt_.group_name_;
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

  MoveItErrorCode plan(Plan& plan)
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

  MoveItErrorCode move(bool wait)
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

  MoveItErrorCode execute(const Plan& plan, bool wait)
  {
    if (!execute_action_client_)
    {
      // TODO: Remove this backwards compatibility code in L-turtle
      moveit_msgs::ExecuteKnownTrajectory::Request req;
      moveit_msgs::ExecuteKnownTrajectory::Response res;
      req.trajectory = plan.trajectory_;
      req.wait_for_execution = wait;
      if (execute_service_.call(req, res))
      {
        return MoveItErrorCode(res.error_code);
      }
      else
      {
        return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
      }
    }

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

  double computeCartesianPath(const std::vector<geometry_msgs::Pose>& waypoints, double step, double jump_threshold,
                              moveit_msgs::RobotTrajectory& msg, const moveit_msgs::Constraints& path_constraints,
                              bool avoid_collisions, moveit_msgs::MoveItErrorCodes& error_code)
  {
    moveit_msgs::GetCartesianPath::Request req;
    moveit_msgs::GetCartesianPath::Response res;

    if (considered_start_state_)
      robot_state::robotStateToRobotStateMsg(*considered_start_state_, req.start_state);
    else
      req.start_state.is_diff = true;

    req.group_name = opt_.group_name_;
    req.header.frame_id = getPoseReferenceFrame();
    req.header.stamp = ros::Time::now();
    req.waypoints = waypoints;
    req.max_step = step;
    req.jump_threshold = jump_threshold;
    req.path_constraints = path_constraints;
    req.avoid_collisions = avoid_collisions;
    req.link_name = getEndEffectorLink();

    if (cartesian_path_service_.call(req, res))
    {
      error_code = res.error_code;
      if (res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        msg = res.solution;
        return res.fraction;
      }
      else
        return -1.0;
    }
    else
    {
      error_code.val = error_code.FAILURE;
      return -1.0;
    }
  }

  void stop()
  {
    if (trajectory_event_publisher_)
    {
      std_msgs::String event;
      event.data = "stop";
      trajectory_event_publisher_.publish(event);
    }
  }

  bool attachObject(const std::string& object, const std::string& link, const std::vector<std::string>& touch_links)
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

  bool detachObject(const std::string& name)
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
      for (std::size_t i = 0; i < lnames.size(); ++i)
      {
        aco.link_name = lnames[i];
        attached_object_publisher_.publish(aco);
      }
    }
    else
      attached_object_publisher_.publish(aco);
    return true;
  }

  double getGoalPositionTolerance() const
  {
    return goal_position_tolerance_;
  }

  double getGoalOrientationTolerance() const
  {
    return goal_orientation_tolerance_;
  }

  double getGoalJointTolerance() const
  {
    return goal_joint_tolerance_;
  }

  void setGoalJointTolerance(double tolerance)
  {
    goal_joint_tolerance_ = tolerance;
  }

  void setGoalPositionTolerance(double tolerance)
  {
    goal_position_tolerance_ = tolerance;
  }

  void setGoalOrientationTolerance(double tolerance)
  {
    goal_orientation_tolerance_ = tolerance;
  }

  void setPlanningTime(double seconds)
  {
    if (seconds > 0.0)
      allowed_planning_time_ = seconds;
  }

  double getPlanningTime() const
  {
    return allowed_planning_time_;
  }

  void allowLooking(bool flag)
  {
    can_look_ = flag;
    ROS_INFO_NAMED("move_group_interface", "Looking around: %s", can_look_ ? "yes" : "no");
  }

  void allowReplanning(bool flag)
  {
    can_replan_ = flag;
    ROS_INFO_NAMED("move_group_interface", "Replanning: %s", can_replan_ ? "yes" : "no");
  }

  void setReplanningDelay(double delay)
  {
    if (delay >= 0.0)
      replan_delay_ = delay;
  }

  double getReplanningDelay() const
  {
    return replan_delay_;
  }

  void constructMotionPlanRequest(moveit_msgs::MotionPlanRequest& request)
  {
    request.group_name = opt_.group_name_;
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
          getJointStateTarget(), joint_model_group_, goal_joint_tolerance_);
    }
    else if (active_target_ == POSE || active_target_ == POSITION || active_target_ == ORIENTATION)
    {
      // find out how many goals are specified
      std::size_t goal_count = 0;
      for (std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator it = pose_targets_.begin();
           it != pose_targets_.end(); ++it)
        goal_count = std::max(goal_count, it->second.size());

      // start filling the goals;
      // each end effector has a number of possible poses (K) as valid goals
      // but there could be multiple end effectors specified, so we want each end effector
      // to reach the goal that corresponds to the goals of the other end effectors
      request.goal_constraints.resize(goal_count);

      for (std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator it = pose_targets_.begin();
           it != pose_targets_.end(); ++it)
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

  void constructGoal(moveit_msgs::MoveGroupGoal& goal)
  {
    constructMotionPlanRequest(goal.request);
  }

  void constructGoal(moveit_msgs::PickupGoal& goal_out, const std::string& object)
  {
    moveit_msgs::PickupGoal goal;
    goal.target_name = object;
    goal.group_name = opt_.group_name_;
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

  void constructGoal(moveit_msgs::PlaceGoal& goal_out, const std::string& object)
  {
    moveit_msgs::PlaceGoal goal;
    goal.attached_object_name = object;
    goal.group_name = opt_.group_name_;
    goal.allowed_planning_time = allowed_planning_time_;
    goal.support_surface_name = support_surface_;
    goal.planner_id = planner_id_;
    if (!support_surface_.empty())
      goal.allow_gripper_support_collision = true;

    if (path_constraints_)
      goal.path_constraints = *path_constraints_;

    goal_out = goal;
  }

  void setPathConstraints(const moveit_msgs::Constraints& constraint)
  {
    path_constraints_.reset(new moveit_msgs::Constraints(constraint));
  }

  bool setPathConstraints(const std::string& constraint)
  {
    if (constraints_storage_)
    {
      moveit_warehouse::ConstraintsWithMetadata msg_m;
      if (constraints_storage_->getConstraints(msg_m, constraint, robot_model_->getName(), opt_.group_name_))
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

  void clearPathConstraints()
  {
    path_constraints_.reset();
  }

  void setTrajectoryConstraints(const moveit_msgs::TrajectoryConstraints& constraint)
  {
    trajectory_constraints_.reset(new moveit_msgs::TrajectoryConstraints(constraint));
  }

  void clearTrajectoryConstraints()
  {
    trajectory_constraints_.reset();
  }

  std::vector<std::string> getKnownConstraints() const
  {
    while (initializing_constraints_)
    {
      static ros::WallDuration d(0.01);
      d.sleep();
    }

    std::vector<std::string> c;
    if (constraints_storage_)
      constraints_storage_->getKnownConstraints(c, robot_model_->getName(), opt_.group_name_);

    return c;
  }

  moveit_msgs::Constraints getPathConstraints() const
  {
    if (path_constraints_)
      return *path_constraints_;
    else
      return moveit_msgs::Constraints();
  }

  moveit_msgs::TrajectoryConstraints getTrajectoryConstraints() const
  {
    if (trajectory_constraints_)
      return *trajectory_constraints_;
    else
      return moveit_msgs::TrajectoryConstraints();
  }

  void initializeConstraintsStorage(const std::string& host, unsigned int port)
  {
    initializing_constraints_ = true;
    if (constraints_init_thread_)
      constraints_init_thread_->join();
    constraints_init_thread_.reset(
        new boost::thread(boost::bind(&MoveGroupInterfaceImpl::initializeConstraintsStorageThread, this, host, port)));
  }

  void setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz)
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

private:
  void initializeConstraintsStorageThread(const std::string& host, unsigned int port)
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

  Options opt_;
  ros::NodeHandle node_handle_;
  boost::shared_ptr<tf::Transformer> tf_;
  robot_model::RobotModelConstPtr robot_model_;
  planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;
  std::unique_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> > move_action_client_;
  std::unique_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> > execute_action_client_;
  std::unique_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction> > pick_action_client_;
  std::unique_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction> > place_action_client_;

  // general planning params
  robot_state::RobotStatePtr considered_start_state_;
  moveit_msgs::WorkspaceParameters workspace_parameters_;
  double allowed_planning_time_;
  std::string planner_id_;
  unsigned int num_planning_attempts_;
  double max_velocity_scaling_factor_;
  double max_acceleration_scaling_factor_;
  double goal_joint_tolerance_;
  double goal_position_tolerance_;
  double goal_orientation_tolerance_;
  bool can_look_;
  bool can_replan_;
  double replan_delay_;

  // joint state goal
  robot_state::RobotStatePtr joint_state_target_;
  const robot_model::JointModelGroup* joint_model_group_;

  // pose goal;
  // for each link we have a set of possible goal locations;
  std::map<std::string, std::vector<geometry_msgs::PoseStamped> > pose_targets_;

  // common properties for goals
  ActiveTargetType active_target_;
  std::unique_ptr<moveit_msgs::Constraints> path_constraints_;
  std::unique_ptr<moveit_msgs::TrajectoryConstraints> trajectory_constraints_;
  std::string end_effector_link_;
  std::string pose_reference_frame_;
  std::string support_surface_;

  // ROS communication
  ros::Publisher trajectory_event_publisher_;
  ros::Publisher attached_object_publisher_;
  ros::ServiceClient execute_service_;
  ros::ServiceClient query_service_;
  ros::ServiceClient get_params_service_;
  ros::ServiceClient set_params_service_;
  ros::ServiceClient cartesian_path_service_;
  ros::ServiceClient plan_grasps_service_;
  std::unique_ptr<moveit_warehouse::ConstraintsStorage> constraints_storage_;
  std::unique_ptr<boost::thread> constraints_init_thread_;
  bool initializing_constraints_;
};
}
}

moveit::planning_interface::MoveGroupInterface::MoveGroupInterface(const std::string& group_name,
                                                                   const boost::shared_ptr<tf::Transformer>& tf,
                                                                   const ros::WallDuration& wait_for_servers)
{
  if (!ros::ok())
    throw std::runtime_error("ROS does not seem to be running");
  impl_ = new MoveGroupInterfaceImpl(Options(group_name), tf ? tf : getSharedTF(), wait_for_servers);
}

moveit::planning_interface::MoveGroupInterface::MoveGroupInterface(const std::string& group,
                                                                   const boost::shared_ptr<tf::Transformer>& tf,
                                                                   const ros::Duration& wait_for_servers)
  : MoveGroupInterface(group, tf, ros::WallDuration(wait_for_servers.toSec()))
{
}

moveit::planning_interface::MoveGroupInterface::MoveGroupInterface(const Options& opt,
                                                                   const boost::shared_ptr<tf::Transformer>& tf,
                                                                   const ros::WallDuration& wait_for_servers)
{
  impl_ = new MoveGroupInterfaceImpl(opt, tf ? tf : getSharedTF(), wait_for_servers);
}

moveit::planning_interface::MoveGroupInterface::MoveGroupInterface(
    const moveit::planning_interface::MoveGroupInterface::Options& opt, const boost::shared_ptr<tf::Transformer>& tf,
    const ros::Duration& wait_for_servers)
  : MoveGroupInterface(opt, tf, ros::WallDuration(wait_for_servers.toSec()))
{
}

moveit::planning_interface::MoveGroupInterface::~MoveGroupInterface()
{
  delete impl_;
}

moveit::planning_interface::MoveGroupInterface::MoveGroupInterface(MoveGroupInterface&& other)
  : remembered_joint_values_(std::move(other.remembered_joint_values_)), impl_(other.impl_)
{
  other.impl_ = nullptr;
}

moveit::planning_interface::MoveGroupInterface& moveit::planning_interface::MoveGroupInterface::
operator=(MoveGroupInterface&& other)
{
  if (this != &other)
  {
    delete impl_;
    impl_ = std::move(other.impl_);
    remembered_joint_values_ = std::move(other.remembered_joint_values_);
    other.impl_ = nullptr;
  }

  return *this;
}

const std::string& moveit::planning_interface::MoveGroupInterface::getName() const
{
  return impl_->getOptions().group_name_;
}

const std::vector<std::string> moveit::planning_interface::MoveGroupInterface::getNamedTargets()
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

robot_model::RobotModelConstPtr moveit::planning_interface::MoveGroupInterface::getRobotModel() const
{
  return impl_->getRobotModel();
}

const ros::NodeHandle& moveit::planning_interface::MoveGroupInterface::getNodeHandle() const
{
  return impl_->getOptions().node_handle_;
}

bool moveit::planning_interface::MoveGroupInterface::getInterfaceDescription(
    moveit_msgs::PlannerInterfaceDescription& desc)
{
  return impl_->getInterfaceDescription(desc);
}

std::map<std::string, std::string> moveit::planning_interface::MoveGroupInterface::getPlannerParams(
    const std::string& planner_id, const std::string& group)
{
  return impl_->getPlannerParams(planner_id, group);
}

void moveit::planning_interface::MoveGroupInterface::setPlannerParams(const std::string& planner_id,
                                                                      const std::string& group,
                                                                      const std::map<std::string, std::string>& params,
                                                                      bool replace)
{
  impl_->setPlannerParams(planner_id, group, params, replace);
}

std::string moveit::planning_interface::MoveGroupInterface::getDefaultPlannerId(const std::string& group) const
{
  return impl_->getDefaultPlannerId(group);
}

void moveit::planning_interface::MoveGroupInterface::setPlannerId(const std::string& planner_id)
{
  impl_->setPlannerId(planner_id);
}

const std::string& moveit::planning_interface::MoveGroupInterface::getPlannerId() const
{
  return impl_->getPlannerId();
}

void moveit::planning_interface::MoveGroupInterface::setNumPlanningAttempts(unsigned int num_planning_attempts)
{
  impl_->setNumPlanningAttempts(num_planning_attempts);
}

void moveit::planning_interface::MoveGroupInterface::setMaxVelocityScalingFactor(double max_velocity_scaling_factor)
{
  impl_->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
}

void moveit::planning_interface::MoveGroupInterface::setMaxAccelerationScalingFactor(
    double max_acceleration_scaling_factor)
{
  impl_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveGroupInterface::asyncMove()
{
  return impl_->move(false);
}

actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>&
moveit::planning_interface::MoveGroupInterface::getMoveGroupClient() const
{
  return impl_->getMoveGroupClient();
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveGroupInterface::move()
{
  return impl_->move(true);
}

moveit::planning_interface::MoveItErrorCode
moveit::planning_interface::MoveGroupInterface::asyncExecute(const Plan& plan)
{
  return impl_->execute(plan, false);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveGroupInterface::execute(const Plan& plan)
{
  return impl_->execute(plan, true);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveGroupInterface::plan(Plan& plan)
{
  return impl_->plan(plan);
}

moveit::planning_interface::MoveItErrorCode
moveit::planning_interface::MoveGroupInterface::pick(const std::string& object, bool plan_only)
{
  return impl_->pick(object, std::vector<moveit_msgs::Grasp>(), plan_only);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveGroupInterface::pick(
    const std::string& object, const moveit_msgs::Grasp& grasp, bool plan_only)
{
  return impl_->pick(object, std::vector<moveit_msgs::Grasp>(1, grasp), plan_only);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveGroupInterface::pick(
    const std::string& object, const std::vector<moveit_msgs::Grasp>& grasps, bool plan_only)
{
  return impl_->pick(object, grasps, plan_only);
}

moveit::planning_interface::MoveItErrorCode
moveit::planning_interface::MoveGroupInterface::planGraspsAndPick(const std::string& object, bool plan_only)
{
  return impl_->planGraspsAndPick(object, plan_only);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveGroupInterface::planGraspsAndPick(
    const moveit_msgs::CollisionObject& object, bool plan_only)
{
  return impl_->planGraspsAndPick(object, plan_only);
}

moveit::planning_interface::MoveItErrorCode
moveit::planning_interface::MoveGroupInterface::place(const std::string& object, bool plan_only)
{
  return impl_->place(object, std::vector<moveit_msgs::PlaceLocation>(), plan_only);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveGroupInterface::place(
    const std::string& object, const std::vector<moveit_msgs::PlaceLocation>& locations, bool plan_only)
{
  return impl_->place(object, locations, plan_only);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveGroupInterface::place(
    const std::string& object, const std::vector<geometry_msgs::PoseStamped>& poses, bool plan_only)
{
  return impl_->place(object, poses, plan_only);
}

moveit::planning_interface::MoveItErrorCode moveit::planning_interface::MoveGroupInterface::place(
    const std::string& object, const geometry_msgs::PoseStamped& pose, bool plan_only)
{
  return impl_->place(object, std::vector<geometry_msgs::PoseStamped>(1, pose), plan_only);
}

double moveit::planning_interface::MoveGroupInterface::computeCartesianPath(
    const std::vector<geometry_msgs::Pose>& waypoints, double eef_step, double jump_threshold,
    moveit_msgs::RobotTrajectory& trajectory, bool avoid_collisions, moveit_msgs::MoveItErrorCodes* error_code)
{
  moveit_msgs::Constraints path_constraints_tmp;
  return computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, path_constraints_tmp, avoid_collisions,
                              error_code);
}

double moveit::planning_interface::MoveGroupInterface::computeCartesianPath(
    const std::vector<geometry_msgs::Pose>& waypoints, double eef_step, double jump_threshold,
    moveit_msgs::RobotTrajectory& trajectory, const moveit_msgs::Constraints& path_constraints, bool avoid_collisions,
    moveit_msgs::MoveItErrorCodes* error_code)
{
  if (error_code)
  {
    return impl_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, path_constraints,
                                       avoid_collisions, *error_code);
  }
  else
  {
    moveit_msgs::MoveItErrorCodes error_code_tmp;
    return impl_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, path_constraints,
                                       avoid_collisions, error_code_tmp);
  }
}

void moveit::planning_interface::MoveGroupInterface::stop()
{
  impl_->stop();
}

void moveit::planning_interface::MoveGroupInterface::setStartState(const moveit_msgs::RobotState& start_state)
{
  robot_state::RobotStatePtr rs;
  impl_->getCurrentState(rs);
  robot_state::robotStateMsgToRobotState(start_state, *rs);
  setStartState(*rs);
}

void moveit::planning_interface::MoveGroupInterface::setStartState(const robot_state::RobotState& start_state)
{
  impl_->setStartState(start_state);
}

void moveit::planning_interface::MoveGroupInterface::setStartStateToCurrentState()
{
  impl_->setStartStateToCurrentState();
}

void moveit::planning_interface::MoveGroupInterface::setRandomTarget()
{
  impl_->getJointStateTarget().setToRandomPositions();
  impl_->setTargetType(JOINT);
}

const std::vector<std::string>& moveit::planning_interface::MoveGroupInterface::getJointNames()
{
  return impl_->getJointModelGroup()->getVariableNames();
}

const std::vector<std::string>& moveit::planning_interface::MoveGroupInterface::getLinkNames()
{
  return impl_->getJointModelGroup()->getLinkModelNames();
}

std::map<std::string, double>
moveit::planning_interface::MoveGroupInterface::getNamedTargetValues(const std::string& name)
{
  std::map<std::string, std::vector<double> >::const_iterator it = remembered_joint_values_.find(name);
  std::map<std::string, double> positions;

  if (it != remembered_joint_values_.end())
  {
    std::vector<std::string> names = impl_->getJointModelGroup()->getVariableNames();
    for (size_t x = 0; x < names.size(); ++x)
    {
      positions[names[x]] = it->second[x];
    }
  }
  else
  {
    impl_->getJointModelGroup()->getVariableDefaultPositions(name, positions);
  }
  return positions;
}

bool moveit::planning_interface::MoveGroupInterface::setNamedTarget(const std::string& name)
{
  std::map<std::string, std::vector<double> >::const_iterator it = remembered_joint_values_.find(name);
  if (it != remembered_joint_values_.end())
  {
    return setJointValueTarget(it->second);
  }
  else
  {
    if (impl_->getJointStateTarget().setToDefaultValues(impl_->getJointModelGroup(), name))
    {
      impl_->setTargetType(JOINT);
      return true;
    }
    ROS_ERROR_NAMED("move_group_interface", "The requested named target '%s' does not exist", name.c_str());
    return false;
  }
}

bool moveit::planning_interface::MoveGroupInterface::setJointValueTarget(const std::vector<double>& joint_values)
{
  if (joint_values.size() != impl_->getJointModelGroup()->getVariableCount())
    return false;
  impl_->setTargetType(JOINT);
  impl_->getJointStateTarget().setJointGroupPositions(impl_->getJointModelGroup(), joint_values);
  return impl_->getJointStateTarget().satisfiesBounds(impl_->getJointModelGroup(), impl_->getGoalJointTolerance());
}

bool moveit::planning_interface::MoveGroupInterface::setJointValueTarget(
    const std::map<std::string, double>& joint_values)
{
  impl_->setTargetType(JOINT);
  impl_->getJointStateTarget().setVariablePositions(joint_values);
  return impl_->getJointStateTarget().satisfiesBounds(impl_->getGoalJointTolerance());
}

bool moveit::planning_interface::MoveGroupInterface::setJointValueTarget(const robot_state::RobotState& rstate)
{
  impl_->setTargetType(JOINT);
  impl_->getJointStateTarget() = rstate;
  return impl_->getJointStateTarget().satisfiesBounds(impl_->getGoalJointTolerance());
}

bool moveit::planning_interface::MoveGroupInterface::setJointValueTarget(const std::string& joint_name, double value)
{
  std::vector<double> values(1, value);
  return setJointValueTarget(joint_name, values);
}

bool moveit::planning_interface::MoveGroupInterface::setJointValueTarget(const std::string& joint_name,
                                                                         const std::vector<double>& values)
{
  impl_->setTargetType(JOINT);
  const robot_model::JointModel* jm = impl_->getJointStateTarget().getJointModel(joint_name);
  if (jm && jm->getVariableCount() == values.size())
  {
    impl_->getJointStateTarget().setJointPositions(jm, values);
    return impl_->getJointStateTarget().satisfiesBounds(jm, impl_->getGoalJointTolerance());
  }
  return false;
}

bool moveit::planning_interface::MoveGroupInterface::setJointValueTarget(const sensor_msgs::JointState& state)
{
  impl_->setTargetType(JOINT);
  impl_->getJointStateTarget().setVariableValues(state);
  return impl_->getJointStateTarget().satisfiesBounds(impl_->getGoalJointTolerance());
}

bool moveit::planning_interface::MoveGroupInterface::setJointValueTarget(const geometry_msgs::Pose& eef_pose,
                                                                         const std::string& end_effector_link)
{
  return impl_->setJointValueTarget(eef_pose, end_effector_link, "", false);
}

bool moveit::planning_interface::MoveGroupInterface::setJointValueTarget(const geometry_msgs::PoseStamped& eef_pose,
                                                                         const std::string& end_effector_link)
{
  return impl_->setJointValueTarget(eef_pose.pose, end_effector_link, eef_pose.header.frame_id, false);
}

bool moveit::planning_interface::MoveGroupInterface::setJointValueTarget(const Eigen::Affine3d& eef_pose,
                                                                         const std::string& end_effector_link)
{
  geometry_msgs::Pose msg;
  tf::poseEigenToMsg(eef_pose, msg);
  return setJointValueTarget(msg, end_effector_link);
}

bool moveit::planning_interface::MoveGroupInterface::setApproximateJointValueTarget(
    const geometry_msgs::Pose& eef_pose, const std::string& end_effector_link)
{
  return impl_->setJointValueTarget(eef_pose, end_effector_link, "", true);
}

bool moveit::planning_interface::MoveGroupInterface::setApproximateJointValueTarget(
    const geometry_msgs::PoseStamped& eef_pose, const std::string& end_effector_link)
{
  return impl_->setJointValueTarget(eef_pose.pose, end_effector_link, eef_pose.header.frame_id, true);
}

bool moveit::planning_interface::MoveGroupInterface::setApproximateJointValueTarget(
    const Eigen::Affine3d& eef_pose, const std::string& end_effector_link)
{
  geometry_msgs::Pose msg;
  tf::poseEigenToMsg(eef_pose, msg);
  return setApproximateJointValueTarget(msg, end_effector_link);
}

const robot_state::RobotState& moveit::planning_interface::MoveGroupInterface::getJointValueTarget() const
{
  return impl_->getJointStateTarget();
}

const std::string& moveit::planning_interface::MoveGroupInterface::getEndEffectorLink() const
{
  return impl_->getEndEffectorLink();
}

const std::string& moveit::planning_interface::MoveGroupInterface::getEndEffector() const
{
  return impl_->getEndEffector();
}

bool moveit::planning_interface::MoveGroupInterface::setEndEffectorLink(const std::string& link_name)
{
  if (impl_->getEndEffectorLink().empty() || link_name.empty())
    return false;
  impl_->setEndEffectorLink(link_name);
  impl_->setTargetType(POSE);
  return true;
}

bool moveit::planning_interface::MoveGroupInterface::setEndEffector(const std::string& eef_name)
{
  const robot_model::JointModelGroup* jmg = impl_->getRobotModel()->getEndEffector(eef_name);
  if (jmg)
    return setEndEffectorLink(jmg->getEndEffectorParentGroup().second);
  return false;
}

void moveit::planning_interface::MoveGroupInterface::clearPoseTarget(const std::string& end_effector_link)
{
  impl_->clearPoseTarget(end_effector_link);
}

void moveit::planning_interface::MoveGroupInterface::clearPoseTargets()
{
  impl_->clearPoseTargets();
}

bool moveit::planning_interface::MoveGroupInterface::setPoseTarget(const Eigen::Affine3d& pose,
                                                                   const std::string& end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> pose_msg(1);
  tf::poseEigenToMsg(pose, pose_msg[0].pose);
  pose_msg[0].header.frame_id = getPoseReferenceFrame();
  pose_msg[0].header.stamp = ros::Time::now();
  return setPoseTargets(pose_msg, end_effector_link);
}

bool moveit::planning_interface::MoveGroupInterface::setPoseTarget(const geometry_msgs::Pose& target,
                                                                   const std::string& end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> pose_msg(1);
  pose_msg[0].pose = target;
  pose_msg[0].header.frame_id = getPoseReferenceFrame();
  pose_msg[0].header.stamp = ros::Time::now();
  return setPoseTargets(pose_msg, end_effector_link);
}

bool moveit::planning_interface::MoveGroupInterface::setPoseTarget(const geometry_msgs::PoseStamped& target,
                                                                   const std::string& end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> targets(1, target);
  return setPoseTargets(targets, end_effector_link);
}

bool moveit::planning_interface::MoveGroupInterface::setPoseTargets(const EigenSTL::vector_Affine3d& target,
                                                                    const std::string& end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> pose_out(target.size());
  ros::Time tm = ros::Time::now();
  const std::string& frame_id = getPoseReferenceFrame();
  for (std::size_t i = 0; i < target.size(); ++i)
  {
    tf::poseEigenToMsg(target[i], pose_out[i].pose);
    pose_out[i].header.stamp = tm;
    pose_out[i].header.frame_id = frame_id;
  }
  return setPoseTargets(pose_out, end_effector_link);
}

bool moveit::planning_interface::MoveGroupInterface::setPoseTargets(const std::vector<geometry_msgs::Pose>& target,
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

bool moveit::planning_interface::MoveGroupInterface::setPoseTargets(
    const std::vector<geometry_msgs::PoseStamped>& target, const std::string& end_effector_link)
{
  if (target.empty())
  {
    ROS_ERROR_NAMED("move_group_interface", "No pose specified as goal target");
    return false;
  }
  else
  {
    impl_->setTargetType(POSE);
    return impl_->setPoseTargets(target, end_effector_link);
  }
}

const geometry_msgs::PoseStamped&
moveit::planning_interface::MoveGroupInterface::getPoseTarget(const std::string& end_effector_link) const
{
  return impl_->getPoseTarget(end_effector_link);
}

const std::vector<geometry_msgs::PoseStamped>&
moveit::planning_interface::MoveGroupInterface::getPoseTargets(const std::string& end_effector_link) const
{
  return impl_->getPoseTargets(end_effector_link);
}

namespace
{
inline void transformPose(const tf::Transformer& tf, const std::string& desired_frame,
                          geometry_msgs::PoseStamped& target)
{
  if (desired_frame != target.header.frame_id)
  {
    tf::Pose pose;
    tf::poseMsgToTF(target.pose, pose);
    tf::Stamped<tf::Pose> stamped_target(pose, target.header.stamp, target.header.frame_id);
    tf::Stamped<tf::Pose> stamped_target_out;
    tf.transformPose(desired_frame, stamped_target, stamped_target_out);
    target.header.frame_id = stamped_target_out.frame_id_;
    //    target.header.stamp = stamped_target_out.stamp_; // we leave the stamp to ros::Time(0) on purpose
    tf::poseTFToMsg(stamped_target_out, target.pose);
  }
}
}

bool moveit::planning_interface::MoveGroupInterface::setPositionTarget(double x, double y, double z,
                                                                       const std::string& end_effector_link)
{
  geometry_msgs::PoseStamped target;
  if (impl_->hasPoseTarget(end_effector_link))
  {
    target = getPoseTarget(end_effector_link);
    transformPose(*impl_->getTF(), impl_->getPoseReferenceFrame(), target);
  }
  else
  {
    target.pose.orientation.x = 0.0;
    target.pose.orientation.y = 0.0;
    target.pose.orientation.z = 0.0;
    target.pose.orientation.w = 1.0;
    target.header.frame_id = impl_->getPoseReferenceFrame();
  }

  target.pose.position.x = x;
  target.pose.position.y = y;
  target.pose.position.z = z;
  bool result = setPoseTarget(target, end_effector_link);
  impl_->setTargetType(POSITION);
  return result;
}

bool moveit::planning_interface::MoveGroupInterface::setRPYTarget(double r, double p, double y,
                                                                  const std::string& end_effector_link)
{
  geometry_msgs::PoseStamped target;
  if (impl_->hasPoseTarget(end_effector_link))
  {
    target = getPoseTarget(end_effector_link);
    transformPose(*impl_->getTF(), impl_->getPoseReferenceFrame(), target);
  }
  else
  {
    target.pose.position.x = 0.0;
    target.pose.position.y = 0.0;
    target.pose.position.z = 0.0;
    target.header.frame_id = impl_->getPoseReferenceFrame();
  }

  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(r, p, y), target.pose.orientation);
  bool result = setPoseTarget(target, end_effector_link);
  impl_->setTargetType(ORIENTATION);
  return result;
}

bool moveit::planning_interface::MoveGroupInterface::setOrientationTarget(double x, double y, double z, double w,
                                                                          const std::string& end_effector_link)
{
  geometry_msgs::PoseStamped target;
  if (impl_->hasPoseTarget(end_effector_link))
  {
    target = getPoseTarget(end_effector_link);
    transformPose(*impl_->getTF(), impl_->getPoseReferenceFrame(), target);
  }
  else
  {
    target.pose.position.x = 0.0;
    target.pose.position.y = 0.0;
    target.pose.position.z = 0.0;
    target.header.frame_id = impl_->getPoseReferenceFrame();
  }

  target.pose.orientation.x = x;
  target.pose.orientation.y = y;
  target.pose.orientation.z = z;
  target.pose.orientation.w = w;
  bool result = setPoseTarget(target, end_effector_link);
  impl_->setTargetType(ORIENTATION);
  return result;
}

void moveit::planning_interface::MoveGroupInterface::setPoseReferenceFrame(const std::string& pose_reference_frame)
{
  impl_->setPoseReferenceFrame(pose_reference_frame);
}

const std::string& moveit::planning_interface::MoveGroupInterface::getPoseReferenceFrame() const
{
  return impl_->getPoseReferenceFrame();
}

double moveit::planning_interface::MoveGroupInterface::getGoalJointTolerance() const
{
  return impl_->getGoalJointTolerance();
}

double moveit::planning_interface::MoveGroupInterface::getGoalPositionTolerance() const
{
  return impl_->getGoalPositionTolerance();
}

double moveit::planning_interface::MoveGroupInterface::getGoalOrientationTolerance() const
{
  return impl_->getGoalOrientationTolerance();
}

void moveit::planning_interface::MoveGroupInterface::setGoalTolerance(double tolerance)
{
  setGoalJointTolerance(tolerance);
  setGoalPositionTolerance(tolerance);
  setGoalOrientationTolerance(tolerance);
}

void moveit::planning_interface::MoveGroupInterface::setGoalJointTolerance(double tolerance)
{
  impl_->setGoalJointTolerance(tolerance);
}

void moveit::planning_interface::MoveGroupInterface::setGoalPositionTolerance(double tolerance)
{
  impl_->setGoalPositionTolerance(tolerance);
}

void moveit::planning_interface::MoveGroupInterface::setGoalOrientationTolerance(double tolerance)
{
  impl_->setGoalOrientationTolerance(tolerance);
}

void moveit::planning_interface::MoveGroupInterface::rememberJointValues(const std::string& name)
{
  rememberJointValues(name, getCurrentJointValues());
}

bool moveit::planning_interface::MoveGroupInterface::startStateMonitor(double wait)
{
  return impl_->startStateMonitor(wait);
}

std::vector<double> moveit::planning_interface::MoveGroupInterface::getCurrentJointValues()
{
  robot_state::RobotStatePtr current_state;
  std::vector<double> values;
  if (impl_->getCurrentState(current_state))
    current_state->copyJointGroupPositions(getName(), values);
  return values;
}

std::vector<double> moveit::planning_interface::MoveGroupInterface::getRandomJointValues()
{
  std::vector<double> r;
  impl_->getJointModelGroup()->getVariableRandomPositions(impl_->getJointStateTarget().getRandomNumberGenerator(), r);
  return r;
}

geometry_msgs::PoseStamped
moveit::planning_interface::MoveGroupInterface::getRandomPose(const std::string& end_effector_link)
{
  const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  Eigen::Affine3d pose;
  pose.setIdentity();
  if (eef.empty())
    ROS_ERROR_NAMED("move_group_interface", "No end-effector specified");
  else
  {
    robot_state::RobotStatePtr current_state;
    if (impl_->getCurrentState(current_state))
    {
      current_state->setToRandomPositions(impl_->getJointModelGroup());
      const robot_model::LinkModel* lm = current_state->getLinkModel(eef);
      if (lm)
        pose = current_state->getGlobalLinkTransform(lm);
    }
  }
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = impl_->getRobotModel()->getModelFrame();
  tf::poseEigenToMsg(pose, pose_msg.pose);
  return pose_msg;
}

geometry_msgs::PoseStamped
moveit::planning_interface::MoveGroupInterface::getCurrentPose(const std::string& end_effector_link)
{
  const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  Eigen::Affine3d pose;
  pose.setIdentity();
  if (eef.empty())
    ROS_ERROR_NAMED("move_group_interface", "No end-effector specified");
  else
  {
    robot_state::RobotStatePtr current_state;
    if (impl_->getCurrentState(current_state))
    {
      const robot_model::LinkModel* lm = current_state->getLinkModel(eef);
      if (lm)
        pose = current_state->getGlobalLinkTransform(lm);
    }
  }
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = impl_->getRobotModel()->getModelFrame();
  tf::poseEigenToMsg(pose, pose_msg.pose);
  return pose_msg;
}

std::vector<double> moveit::planning_interface::MoveGroupInterface::getCurrentRPY(const std::string& end_effector_link)
{
  std::vector<double> result;
  const std::string& eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  if (eef.empty())
    ROS_ERROR_NAMED("move_group_interface", "No end-effector specified");
  else
  {
    robot_state::RobotStatePtr current_state;
    if (impl_->getCurrentState(current_state))
    {
      const robot_model::LinkModel* lm = current_state->getLinkModel(eef);
      if (lm)
      {
        result.resize(3);
        tf::Matrix3x3 ptf;
        tf::matrixEigenToTF(current_state->getGlobalLinkTransform(lm).linear(), ptf);
        tfScalar pitch, roll, yaw;
        ptf.getRPY(roll, pitch, yaw);
        result[0] = roll;
        result[1] = pitch;
        result[2] = yaw;
      }
    }
  }
  return result;
}

const std::vector<std::string>& moveit::planning_interface::MoveGroupInterface::getActiveJoints() const
{
  return impl_->getJointModelGroup()->getActiveJointModelNames();
}

const std::vector<std::string>& moveit::planning_interface::MoveGroupInterface::getJoints() const
{
  return impl_->getJointModelGroup()->getJointModelNames();
}

unsigned int moveit::planning_interface::MoveGroupInterface::getVariableCount() const
{
  return impl_->getJointModelGroup()->getVariableCount();
}

robot_state::RobotStatePtr moveit::planning_interface::MoveGroupInterface::getCurrentState(double wait)
{
  robot_state::RobotStatePtr current_state;
  impl_->getCurrentState(current_state, wait);
  return current_state;
}

void moveit::planning_interface::MoveGroupInterface::rememberJointValues(const std::string& name,
                                                                         const std::vector<double>& values)
{
  remembered_joint_values_[name] = values;
}

void moveit::planning_interface::MoveGroupInterface::forgetJointValues(const std::string& name)
{
  remembered_joint_values_.erase(name);
}

void moveit::planning_interface::MoveGroupInterface::allowLooking(bool flag)
{
  impl_->allowLooking(flag);
}

void moveit::planning_interface::MoveGroupInterface::allowReplanning(bool flag)
{
  impl_->allowReplanning(flag);
}

std::vector<std::string> moveit::planning_interface::MoveGroupInterface::getKnownConstraints() const
{
  return impl_->getKnownConstraints();
}

moveit_msgs::Constraints moveit::planning_interface::MoveGroupInterface::getPathConstraints() const
{
  return impl_->getPathConstraints();
}

bool moveit::planning_interface::MoveGroupInterface::setPathConstraints(const std::string& constraint)
{
  return impl_->setPathConstraints(constraint);
}

void moveit::planning_interface::MoveGroupInterface::setPathConstraints(const moveit_msgs::Constraints& constraint)
{
  impl_->setPathConstraints(constraint);
}

void moveit::planning_interface::MoveGroupInterface::clearPathConstraints()
{
  impl_->clearPathConstraints();
}

moveit_msgs::TrajectoryConstraints moveit::planning_interface::MoveGroupInterface::getTrajectoryConstraints() const
{
  return impl_->getTrajectoryConstraints();
}

void moveit::planning_interface::MoveGroupInterface::setTrajectoryConstraints(
    const moveit_msgs::TrajectoryConstraints& constraint)
{
  impl_->setTrajectoryConstraints(constraint);
}

void moveit::planning_interface::MoveGroupInterface::clearTrajectoryConstraints()
{
  impl_->clearTrajectoryConstraints();
}

void moveit::planning_interface::MoveGroupInterface::setConstraintsDatabase(const std::string& host, unsigned int port)
{
  impl_->initializeConstraintsStorage(host, port);
}

void moveit::planning_interface::MoveGroupInterface::setWorkspace(double minx, double miny, double minz, double maxx,
                                                                  double maxy, double maxz)
{
  impl_->setWorkspace(minx, miny, minz, maxx, maxy, maxz);
}

/** \brief Set time allowed to planner to solve problem before aborting */
void moveit::planning_interface::MoveGroupInterface::setPlanningTime(double seconds)
{
  impl_->setPlanningTime(seconds);
}

/** \brief Get time allowed to planner to solve problem before aborting */
double moveit::planning_interface::MoveGroupInterface::getPlanningTime() const
{
  return impl_->getPlanningTime();
}

void moveit::planning_interface::MoveGroupInterface::setSupportSurfaceName(const std::string& name)
{
  impl_->setSupportSurfaceName(name);
}

const std::string& moveit::planning_interface::MoveGroupInterface::getPlanningFrame() const
{
  return impl_->getRobotModel()->getModelFrame();
}

bool moveit::planning_interface::MoveGroupInterface::attachObject(const std::string& object, const std::string& link)
{
  return attachObject(object, link, std::vector<std::string>());
}

bool moveit::planning_interface::MoveGroupInterface::attachObject(const std::string& object, const std::string& link,
                                                                  const std::vector<std::string>& touch_links)
{
  return impl_->attachObject(object, link, touch_links);
}

bool moveit::planning_interface::MoveGroupInterface::detachObject(const std::string& name)
{
  return impl_->detachObject(name);
}

void moveit::planning_interface::MoveGroupInterface::constructMotionPlanRequest(
    moveit_msgs::MotionPlanRequest& goal_out)
{
  impl_->constructMotionPlanRequest(goal_out);
}
