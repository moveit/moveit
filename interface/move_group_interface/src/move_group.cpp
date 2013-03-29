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

#include <stdexcept>
#include <moveit/move_group/names.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/QueryPlannerInterfaces.h>

#include <actionlib/client/simple_action_client.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <ros/console.h>
#include <ros/ros.h>

namespace move_group_interface
{

const std::string MoveGroup::ROBOT_DESCRIPTION = "robot_description";    // name of the robot description (a param name, so it can be changed externally)
const std::string MoveGroup::JOINT_STATE_TOPIC = "joint_states";    // name of the topic where joint states are published

namespace
{

struct SharedStorage
{
  SharedStorage()
  {
  }

  ~SharedStorage()
  {
    tf_.reset();
    state_monitors_.clear();
    model_loaders_.clear(); 
  }
  
  boost::mutex lock_;
  boost::shared_ptr<tf::Transformer> tf_;
  std::map<std::string, robot_model_loader::RobotModelLoaderPtr> model_loaders_;
  std::map<std::string, planning_scene_monitor::CurrentStateMonitorPtr> state_monitors_;
};

SharedStorage& getSharedStorage()
{
  static SharedStorage storage;
  return storage;
}

boost::shared_ptr<tf::Transformer> getSharedTF()
{
  SharedStorage &s = getSharedStorage();
  boost::mutex::scoped_lock slock(s.lock_);
  if (!s.tf_)
    s.tf_.reset(new tf::TransformListener());
  return s.tf_;
}

robot_model::RobotModelConstPtr getSharedRobotModel(const std::string &robot_description)
{ 
  SharedStorage &s = getSharedStorage();
  boost::mutex::scoped_lock slock(s.lock_);
  if (s.model_loaders_.find(robot_description) != s.model_loaders_.end())
    return s.model_loaders_[robot_description]->getModel();
  else
  {
    robot_model_loader::RobotModelLoader::Options opt(robot_description);
    opt.load_kinematics_solvers_ = false;
    robot_model_loader::RobotModelLoaderPtr loader(new robot_model_loader::RobotModelLoader(opt));
    s.model_loaders_[robot_description] = loader;
    return loader->getModel();
  }
} 

static planning_scene_monitor::CurrentStateMonitorPtr getSharedStateMonitor(const robot_model::RobotModelConstPtr &kmodel, const boost::shared_ptr<tf::Transformer> &tf)
{  
  SharedStorage &s = getSharedStorage();
  boost::mutex::scoped_lock slock(s.lock_);
  if (s.state_monitors_.find(kmodel->getName()) != s.state_monitors_.end())
    return s.state_monitors_[kmodel->getName()];
  else
  {
    planning_scene_monitor::CurrentStateMonitorPtr monitor(new planning_scene_monitor::CurrentStateMonitor(kmodel, tf));
    s.state_monitors_[kmodel->getName()] = monitor;
    return monitor;
  }
}

}

class MoveGroup::MoveGroupImpl
{
public: 
  
  MoveGroupImpl(const Options &opt, const boost::shared_ptr<tf::Transformer> &tf, const ros::Duration &wait_for_server) : opt_(opt), tf_(tf)
  {
    kinematic_model_ = opt.kinematic_model_ ? opt.kinematic_model_ : getSharedRobotModel(opt.robot_description_);
    if (!getRobotModel())
    {
      std::string error = "Unable to construct robot model. Please make sure all needed information is on the parameter server.";
      ROS_FATAL_STREAM(error);
      throw std::runtime_error(error);
    }

    if (!getRobotModel()->hasJointModelGroup(opt.group_name_))
    {
      std::string error = "Group '" + opt.group_name_ + "' was not found.";
      ROS_FATAL_STREAM(error);
      throw std::runtime_error(error);
    }

    joint_state_target_.reset(new robot_state::RobotState(getRobotModel()));
    joint_state_target_->setToDefaultValues();
    active_target_ = JOINT;
    can_look_ = false;
    can_replan_ = false;
    goal_tolerance_ = 1e-4;
    planning_time_ = 5.0;
    
    const robot_model::JointModelGroup *joint_model_group = getRobotModel()->getJointModelGroup(opt.group_name_);
    if (joint_model_group)
    {
      if (joint_model_group->isChain())
        end_effector_link_ = joint_model_group->getLinkModelNames().back();
      pose_reference_frame_ = getRobotModel()->getModelFrame();
      
      trajectory_event_publisher_ = node_handle_.advertise<std_msgs::String>("trajectory_execution_event", 1, false);
      
      current_state_monitor_ = getSharedStateMonitor(kinematic_model_, tf_);

      move_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>(move_group::MOVE_ACTION, false));
      waitForAction(move_action_client_, wait_for_server, move_group::MOVE_ACTION);

      pick_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(move_group::PICKUP_ACTION, false));
      waitForAction(pick_action_client_, wait_for_server, move_group::PICKUP_ACTION);

      place_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>(move_group::PLACE_ACTION, false));
      waitForAction(place_action_client_, wait_for_server, move_group::PLACE_ACTION);
      
      execute_service_ = node_handle_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("execute_kinematic_path");
      query_service_ = node_handle_.serviceClient<moveit_msgs::QueryPlannerInterfaces>("query_planner_interface");
      initializeConstraintsStorage();
      ROS_INFO_STREAM("Ready to take MoveGroup commands for group " << opt.group_name_ << ".");
    }
    else
      ROS_ERROR("Unable to initialize MoveGroup interface.");
  }

  template<typename T>
  void waitForAction(const T &action, const ros::Duration &wait_for_server, const std::string &name)
  {
    ROS_INFO("Waiting for MoveGroup action server (%s)...", name.c_str());
    
    // in case ROS time is published, wait for the time data to arrive
    ros::Time start_time = ros::Time::now();
    while (start_time == ros::Time::now())
    {
      ros::WallDuration(0.01).sleep();
      ros::spinOnce();
    }
    
    // wait for the server (and spin as needed)
    if (wait_for_server == ros::Duration(0, 0))
    {
      while (node_handle_.ok() && !action->isServerConnected())
      {
        ros::WallDuration(0.02).sleep();
        ros::spinOnce();
      }
    }
    else
    {
      ros::Time final_time = ros::Time::now() + wait_for_server;
      while (node_handle_.ok() && !action->isServerConnected() && final_time > ros::Time::now())
      {
        ros::WallDuration(0.02).sleep();
        ros::spinOnce();
      }
    }
    
    if (!action->isServerConnected())
      throw std::runtime_error("Unable to connect to action server within allotted time");
    else
      ROS_INFO("Connected to '%s'", name.c_str());
  }
  
  ~MoveGroupImpl()
  {
    if (constraints_init_thread_)
    {
      terminate_constraints_init_thread_ = true;
      constraints_init_thread_->join();
    }
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
    return kinematic_model_;
  }

  bool getInterfaceDescription(moveit_msgs::PlannerInterfaceDescription &desc)
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
  
  void setPlannerId(const std::string &planner_id)
  {
    planner_id_ = planner_id;
  }
  
  robot_state::JointStateGroup* getJointStateTarget()
  {
    return joint_state_target_->getJointStateGroup(opt_.group_name_);
  }
  
  void setStartState(const robot_state::RobotState &start_state)
  {
    considered_start_state_.reset(new robot_state::RobotState(start_state));
  }

  void setStartStateToCurrentState()
  {
    considered_start_state_.reset();
  }
  
  void setEndEffectorLink(const std::string &end_effector)
  {
    end_effector_link_ = end_effector;
  }
  
  void clearPoseTarget(const std::string &end_effector_link)
  {
    pose_targets_.erase(end_effector_link);
  }

  void clearPoseTargets()
  {
    pose_targets_.clear();
  }
  
  const std::string &getEndEffectorLink() const
  {
    return end_effector_link_;
  }

  const std::string& getEndEffector() const
  {
    if (!end_effector_link_.empty())
    {   
      const std::vector<std::string> &possible_eefs = getRobotModel()->getJointModelGroup(opt_.group_name_)->getAttachedEndEffectorNames();
      for (std::size_t i = 0 ; i < possible_eefs.size() ; ++i)
        if (getRobotModel()->getEndEffector(possible_eefs[i])->hasLinkModel(end_effector_link_))
          return possible_eefs[i];
    }  
    static std::string empty;
    return empty;
  }

  void setPoseTargets(const std::vector<geometry_msgs::PoseStamped> &poses, const std::string &end_effector_link)
  {  
    const std::string &eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link;
    if (eef.empty())
      ROS_ERROR("No end-effector to set the pose for");
    else
    {
      pose_targets_[eef] = poses;
      // make sure we don't store an actual stamp, since that will become stale can potentially cause tf errors
      std::vector<geometry_msgs::PoseStamped> &stored_poses = pose_targets_[eef];
      for (std::size_t i = 0 ; i < stored_poses.size() ; ++i)
        stored_poses[i].header.stamp = ros::Time(0);
    }
  }
  
  bool hasPoseTarget(const std::string &end_effector_link) const
  {
    const std::string &eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link; 
    return pose_targets_.find(eef) != pose_targets_.end();
  }
  
  const geometry_msgs::PoseStamped& getPoseTarget(const std::string &end_effector_link) const
  {    
    const std::string &eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link; 
    
    // if multiple pose targets are set, return the first one
    std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator jt = pose_targets_.find(eef);
    if (jt != pose_targets_.end())
      if (!jt->second.empty())
        return jt->second.at(0);
    
    // or return an error
    static const geometry_msgs::PoseStamped unknown;
    ROS_ERROR("Pose for end effector '%s' not known.", eef.c_str());
    return unknown;
  } 
  
  const std::vector<geometry_msgs::PoseStamped>& getPoseTargets(const std::string &end_effector_link) const
  {
    const std::string &eef = end_effector_link.empty() ? end_effector_link_ : end_effector_link; 
    
    std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator jt = pose_targets_.find(eef);
    if (jt != pose_targets_.end())
      if (!jt->second.empty())
        return jt->second;
    
    // or return an error
    static const std::vector<geometry_msgs::PoseStamped> empty;
    ROS_ERROR("Poses for end effector '%s' are not known.", eef.c_str());
    return empty;
  }

  void followConstraints(const std::vector<moveit_msgs::Constraints> &constraints)
  {
    follow_constraints_ = constraints;
  }
  
  void setPoseReferenceFrame(const std::string &pose_reference_frame)
  {
    pose_reference_frame_ = pose_reference_frame;
  }
  
  void setSupportSurfaceName(const std::string &support_surface)
  {
    support_surface_ = support_surface;
  }
  
  const std::string& getPoseReferenceFrame() const
  {
    return pose_reference_frame_;
  }
  
  void useJointStateTarget()
  {
    active_target_ = JOINT;    
  }
  
  void usePoseTarget()
  {
    active_target_ = POSE;
  }
  
  void useFollowTarget()
  {
    active_target_ = FOLLOW;
  }
  
  void allowLooking(bool flag)
  {
    can_look_ = flag;
    ROS_INFO("Looking around: %s", can_look_ ? "yes" : "no");
  }

  void allowReplanning(bool flag)
  {
    can_replan_ = flag;
    ROS_INFO("Replanning: %s", can_replan_ ? "yes" : "no");
  }
  
  bool getCurrentState(robot_state::RobotStatePtr &current_state)
  {
    if (!current_state_monitor_)
    {
      ROS_ERROR("Unable to get current robot state");
      return false;
    }
        
    // if needed, start the monitor and wait up to 1 second for a full robot state
    if (!current_state_monitor_->isActive())
    {
      current_state_monitor_->startStateMonitor(opt_.joint_state_topic_);
      double slept_time = 0.0;
      static const double sleep_step = 0.05;
      while (!current_state_monitor_->haveCompleteState() && slept_time < 1.0)
      {
        ros::Duration(sleep_step).sleep();
        slept_time += sleep_step;
      }      
    }
    
    // check to see if we have a fully known state for the joints we want to record
    std::vector<std::string> missing_joints;
    if (!current_state_monitor_->haveCompleteState(missing_joints))
    {
      std::set<std::string> mj;
      mj.insert(missing_joints.begin(), missing_joints.end());
      const std::vector<std::string> &names= getJointStateTarget()->getJointNames();
      bool ok = true;
      for (std::size_t i = 0 ; ok && i < names.size() ; ++i)
        if (mj.find(names[i]) != mj.end())
          ok = false;
      if (!ok)
        ROS_WARN("Joint values for monitored state are requested but the full state is not known");
    }
    
    current_state = current_state_monitor_->getCurrentState();
    return true;
  }

  bool place(const std::string &object, const std::vector<manipulation_msgs::PlaceLocation> &locations)
  {   
    if (!place_action_client_)
      return false;
    if (!place_action_client_->isServerConnected())
      return false; 
    moveit_msgs::PlaceGoal goal;
    constructGoal(goal, object);
    goal.place_locations = locations;
    goal.planning_options.plan_only = false;
    place_action_client_->sendGoal(goal); 
    if (!place_action_client_->waitForResult())
    {
      ROS_INFO_STREAM("Place action returned early");
    }
    if (place_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Fail: " << place_action_client_->getState().toString() << ": " << place_action_client_->getState().getText());
      return false;
    }
  }  

  bool pick(const std::string &object, const std::vector<manipulation_msgs::Grasp> &grasps)
  {
    if (!pick_action_client_)
      return false;
    if (!pick_action_client_->isServerConnected())
      return false;
    moveit_msgs::PickupGoal goal;
    constructGoal(goal, object);
    goal.possible_grasps = grasps;
    goal.planning_options.plan_only = false;
    pick_action_client_->sendGoal(goal); 
    if (!pick_action_client_->waitForResult())
    {
      ROS_INFO_STREAM("Pickup action returned early");
    }
    if (pick_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Fail: " << pick_action_client_->getState().toString() << ": " << pick_action_client_->getState().getText());
      return false;
    }
  }
  
  bool plan(Plan &plan)
  {
    if (!move_action_client_)
      return false;
    if (!move_action_client_->isServerConnected())
      return false;

    moveit_msgs::MoveGroupGoal goal;
    constructGoal(goal);
    goal.planning_options.plan_only = true;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;
    move_action_client_->sendGoal(goal); 
    if (!move_action_client_->waitForResult())
    {
      ROS_INFO_STREAM("MoveGroup action returned early");
    }
    if (move_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      plan.trajectory_ = move_action_client_->getResult()->planned_trajectory;
      plan.start_state_ = move_action_client_->getResult()->trajectory_start;
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Fail: " << move_action_client_->getState().toString() << ": " << move_action_client_->getState().getText());
      return false;
    }
  }
  
  bool move(bool wait)
  {  
    if (!move_action_client_)
      return false;
    if (!move_action_client_->isServerConnected())
      return false;

    moveit_msgs::MoveGroupGoal goal;
    constructGoal(goal);
    goal.planning_options.plan_only = false;
    goal.planning_options.look_around = can_look_;
    goal.planning_options.replan = can_replan_;
    goal.planning_options.replan_delay = 2.0; // this should become a parameter

    move_action_client_->sendGoal(goal);
    if (!wait)
      return true;
    
    if (!move_action_client_->waitForResult())
    {
      ROS_INFO_STREAM("MoveGroup action returned early");
    }
    
    if (move_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      return true;
    else
    {
      ROS_INFO_STREAM(move_action_client_->getState().toString() << ": " << move_action_client_->getState().getText());
      return false;
    }
  }
  
  bool execute(const Plan &plan, bool wait)
  {
    moveit_msgs::ExecuteKnownTrajectory::Request req;
    moveit_msgs::ExecuteKnownTrajectory::Response res;
    req.trajectory = plan.trajectory_;
    req.wait_for_execution = wait;
    if (execute_service_.call(req, res))
      return res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
    else
      return false;
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
  
  double getGoalTolerance() const
  {
    return goal_tolerance_;
  }
  
  void setGoalTolerance(double tolerance)
  {
    goal_tolerance_ = tolerance;
  }

  void setPlanningTime(double seconds)
  {
    if (seconds > 0.0)
      planning_time_ = seconds;
  }
  
  void constructGoal(moveit_msgs::MoveGroupGoal &goal_out)
  {
    moveit_msgs::MoveGroupGoal goal;
    goal.request.group_name = opt_.group_name_;
    goal.request.num_planning_attempts = 1;
    goal.request.allowed_planning_time = planning_time_;
    goal.request.planner_id = planner_id_;
    goal.request.workspace_parameters = workspace_parameters_;
    
    if (considered_start_state_)
      robot_state::robotStateToRobotStateMsg(*considered_start_state_, goal.request.start_state);
    
    if (active_target_ == JOINT)
    {    
      goal.request.goal_constraints.resize(1);
      goal.request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(getJointStateTarget(), goal_tolerance_);
    }
    else
      if (active_target_ == POSE)
      {
        for (std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator it = pose_targets_.begin() ;
             it != pose_targets_.end() ; ++it)
        {
          moveit_msgs::Constraints g;
          for (std::size_t i = 0 ; i < it->second.size() ; ++i)
          {
            moveit_msgs::Constraints c = kinematic_constraints::constructGoalConstraints(it->first, it->second[i], goal_tolerance_);
            g = kinematic_constraints::mergeConstraints(g, c);
          }
          goal.request.goal_constraints.push_back(g);
        }
      }
      else
        if (active_target_ == FOLLOW)
        {
          goal.request.trajectory_constraints.constraints = follow_constraints_;
        }
        else
          ROS_ERROR("Unable to construct goal representation");
    
    if (path_constraints_)
      goal.request.path_constraints = *path_constraints_;
    goal_out = goal;
  }

  void constructGoal(moveit_msgs::PickupGoal &goal_out, const std::string &object)
  {
    moveit_msgs::PickupGoal goal;
    goal.target_name = object;
    goal.group_name = opt_.group_name_;
    goal.end_effector = getEndEffector(); 
    goal.allowed_planning_time = planning_time_;
    goal.support_surface_name = support_surface_; 
    goal.planner_id = planner_id_;

    if (path_constraints_)
      goal.path_constraints = *path_constraints_;

    goal_out = goal;
  }

  void constructGoal(moveit_msgs::PlaceGoal &goal_out, const std::string &object)
  {
    moveit_msgs::PlaceGoal goal;
    goal.attached_object_name = object;
    goal.group_name = opt_.group_name_;
    goal.allowed_planning_time = planning_time_;
    goal.support_surface_name = support_surface_;
    goal.planner_id = planner_id_;

    if (path_constraints_)
      goal.path_constraints = *path_constraints_;
    
    goal_out = goal;
  }
  
  void setPathConstraints(const moveit_msgs::Constraints &constraint)
  {
    path_constraints_.reset(new moveit_msgs::Constraints(constraint));
  }
  
  bool setPathConstraints(const std::string &constraint)
  {
    if (constraints_storage_)
    {
      moveit_warehouse::ConstraintsWithMetadata msg_m;
      if (constraints_storage_->getConstraints(msg_m, constraint, kinematic_model_->getName(), opt_.group_name_))
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
  
  std::vector<std::string> getKnownConstraints() const
  {
    std::vector<std::string> c;
    if (constraints_storage_)
      constraints_storage_->getKnownConstraints(c, kinematic_model_->getName(), opt_.group_name_);
    return c;
  }
  
  void initializeConstraintsStorage(const std::string &host = "", unsigned int port = 0)
  {    
    if (constraints_init_thread_)
    {
      terminate_constraints_init_thread_ = true;
      constraints_init_thread_->join();
    }
    terminate_constraints_init_thread_ = false;
    constraints_init_thread_.reset(new boost::thread(boost::bind(&MoveGroupImpl::initializeConstraintsStorageThread, this, host, port)));
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
  
  void initializeConstraintsStorageThread(const std::string &host, unsigned int port)
  {
    // this is interruptible, allows the thread to quickly terminate if the destructor is 
    // triggered right after the constructor
    ros::WallDuration d(0.01);
    for (int i = 0 ; i < 20 ; ++i)
      if (terminate_constraints_init_thread_)
        return;
      else
        d.sleep();
    try
    {
      constraints_storage_.reset(new moveit_warehouse::ConstraintsStorage(host, port));
      ROS_DEBUG("Connected to constraints database");
    }
    catch(std::runtime_error &ex)
    {
      ROS_DEBUG("%s", ex.what());
    }
  }
  
  enum ActiveTarget
  {
    JOINT, POSE, FOLLOW
  };
  
  Options opt_;
  ros::NodeHandle node_handle_;
  boost::shared_ptr<tf::Transformer> tf_;
  robot_model::RobotModelConstPtr kinematic_model_;
  planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;
  boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> > move_action_client_;
  boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction> > pick_action_client_;
  boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction> > place_action_client_;

  // general planning params
  robot_state::RobotStatePtr considered_start_state_;
  moveit_msgs::WorkspaceParameters workspace_parameters_;
  double planning_time_;
  std::string planner_id_;
  double goal_tolerance_;
  bool can_look_;
  bool can_replan_;
  
  // joint state goal
  robot_state::RobotStatePtr joint_state_target_;

  // pose goal
  std::map<std::string, std::vector<geometry_msgs::PoseStamped> > pose_targets_;

  // follow trajectory goal
  std::vector<moveit_msgs::Constraints> follow_constraints_;

  // common properties for goals
  ActiveTarget active_target_;
  boost::scoped_ptr<moveit_msgs::Constraints> path_constraints_;
  std::string end_effector_link_;
  std::string pose_reference_frame_; 
  std::string support_surface_;
  
  // ROS communication
  ros::Publisher trajectory_event_publisher_;
  ros::ServiceClient execute_service_;
  ros::ServiceClient query_service_;
  boost::scoped_ptr<moveit_warehouse::ConstraintsStorage> constraints_storage_;
  boost::scoped_ptr<boost::thread> constraints_init_thread_;
  bool terminate_constraints_init_thread_;
};

MoveGroup::MoveGroup(const std::string &group_name, const boost::shared_ptr<tf::Transformer> &tf, const ros::Duration &wait_for_server)
{
  if (!ros::ok())
    throw std::runtime_error("ROS does not seem to be running");
  impl_ = new MoveGroupImpl(Options(group_name), tf ? tf : getSharedTF(), wait_for_server);
}

MoveGroup::MoveGroup(const Options &opt, const boost::shared_ptr<tf::Transformer> &tf, const ros::Duration &wait_for_server)
{
  impl_ = new MoveGroupImpl(opt, tf ? tf : getSharedTF(), wait_for_server);
}

MoveGroup::~MoveGroup()
{
  delete impl_;
}

const std::string& MoveGroup::getName() const
{
  return impl_->getOptions().group_name_;
}

bool MoveGroup::getInterfaceDescription(moveit_msgs::PlannerInterfaceDescription &desc) 
{  
  return impl_->getInterfaceDescription(desc);  
}

void MoveGroup::setPlannerId(const std::string &planner_id)
{
  impl_->setPlannerId(planner_id);
}

bool MoveGroup::asyncMove()
{
  return impl_->move(false);
}

bool MoveGroup::move()
{
  return impl_->move(true);
}

bool MoveGroup::asyncExecute(const Plan &plan)
{
  return impl_->execute(plan, false);
}

bool MoveGroup::execute(const Plan &plan)
{
  return impl_->execute(plan, true);
}

bool MoveGroup::plan(Plan &plan)
{
  return impl_->plan(plan);
}

bool MoveGroup::pick(const std::string &object)
{
  return impl_->pick(object, std::vector<manipulation_msgs::Grasp>());
}

bool MoveGroup::pick(const std::string &object, const std::vector<manipulation_msgs::Grasp> &grasps)
{
  return impl_->pick(object, grasps);
}

bool MoveGroup::place(const std::string &object)
{
  return impl_->place(object, std::vector<manipulation_msgs::PlaceLocation>());
}

bool MoveGroup::place(const std::string &object, const std::vector<manipulation_msgs::PlaceLocation> &locations)
{
  return impl_->place(object, locations);
}

void MoveGroup::stop()
{
  impl_->stop();
}

void MoveGroup::setStartState(const robot_state::RobotState &start_state)
{
  impl_->setStartState(start_state);
}

void MoveGroup::setStartStateToCurrentState()
{
  impl_->setStartStateToCurrentState();
}

void MoveGroup::setRandomTarget()
{ 
  impl_->getJointStateTarget()->setToRandomValues();
  impl_->useJointStateTarget();
}

bool MoveGroup::setNamedTarget(const std::string &name)
{ 
  std::map<std::string, std::vector<double> >::const_iterator it = remembered_joint_values_.find(name);
  if (it != remembered_joint_values_.end())
  {
    setJointValueTarget(it->second);
    return true;
  }
  else
  {
    if (impl_->getJointStateTarget()->setToDefaultState(name))
    {
      impl_->useJointStateTarget();
      return true;
    }
    return false;
  }
}

void MoveGroup::setJointValueTarget(const std::vector<double> &joint_values)
{
  impl_->getJointStateTarget()->setVariableValues(joint_values);
  impl_->useJointStateTarget();
}

void MoveGroup::setJointValueTarget(const std::map<std::string, double> &joint_values)
{
  impl_->getJointStateTarget()->setVariableValues(joint_values);
  impl_->useJointStateTarget();
}

void MoveGroup::setJointValueTarget(const robot_state::RobotState &kinematic_state)
{
  setJointValueTarget(*kinematic_state.getJointStateGroup(getName()));
}

void MoveGroup::setJointValueTarget(const robot_state::JointStateGroup &joint_state_group)
{  
  std::map<std::string, double> variable_values;
  joint_state_group.getVariableValues(variable_values);
  setJointValueTarget(variable_values);
}

void MoveGroup::setJointValueTarget(const robot_state::JointState &joint_state)
{
  setJointValueTarget(joint_state.getName(), joint_state.getVariableValues());
}

void MoveGroup::setJointValueTarget(const std::string &joint_name, double value)
{ 
  std::vector<double> values(1, value);
  setJointValueTarget(joint_name, values);
}

void MoveGroup::setJointValueTarget(const std::string &joint_name, const std::vector<double> &values)
{ 
  robot_state::JointState *joint_state = impl_->getJointStateTarget()->getJointState(joint_name);
  if (joint_state)
    if (!joint_state->setVariableValues(values))
      ROS_ERROR("Unable to set target");
  impl_->useJointStateTarget();
}

void MoveGroup::setJointValueTarget(const sensor_msgs::JointState &state)
{
  impl_->getJointStateTarget()->setVariableValues(state); 
  impl_->useJointStateTarget();
}

const robot_state::JointStateGroup& MoveGroup::getJointValueTarget() const
{
  return *impl_->getJointStateTarget();
}

const std::string& MoveGroup::getEndEffectorLink() const
{
  return impl_->getEndEffectorLink();
}

const std::string& MoveGroup::getEndEffector() const
{
  return impl_->getEndEffector();
}

void MoveGroup::setEndEffectorLink(const std::string &link_name)
{
  impl_->setEndEffectorLink(link_name);  
  impl_->usePoseTarget();
}

void MoveGroup::setEndEffector(const std::string &eef_name)
{
  const robot_model::JointModelGroup *jmg = impl_->getRobotModel()->getEndEffector(eef_name);
  if (jmg)
    setEndEffectorLink(jmg->getEndEffectorParentGroup().second);
}

void MoveGroup::clearPoseTarget(const std::string &end_effector_link)
{
  impl_->clearPoseTarget(end_effector_link);
}

void MoveGroup::clearPoseTargets()
{
  impl_->clearPoseTargets();
}

void MoveGroup::setPoseTarget(const Eigen::Affine3d &pose, const std::string &end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> pose_msg(1);
  tf::poseEigenToMsg(pose, pose_msg[0].pose);
  pose_msg[0].header.frame_id = getPoseReferenceFrame();
  pose_msg[0].header.stamp = ros::Time::now();
  setPoseTargets(pose_msg, end_effector_link);
}

void MoveGroup::setPoseTarget(const geometry_msgs::Pose &target, const std::string &end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> pose_msg(1);
  pose_msg[0].pose = target;
  pose_msg[0].header.frame_id = getPoseReferenceFrame();
  pose_msg[0].header.stamp = ros::Time::now();
  setPoseTargets(pose_msg, end_effector_link);
}

void MoveGroup::setPoseTarget(const geometry_msgs::PoseStamped &target, const std::string &end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> targets(1, target);
  setPoseTargets(targets, end_effector_link);
}

void MoveGroup::setPoseTargets(const EigenSTL::vector_Affine3d &target, const std::string &end_effector_link)
{  
  std::vector<geometry_msgs::PoseStamped> pose_out(target.size());
  ros::Time tm = ros::Time::now();
  const std::string &frame_id = getPoseReferenceFrame();
  for (std::size_t i = 0 ; i < target.size() ; ++i)
  {
    tf::poseEigenToMsg(target[i], pose_out[i].pose);
    pose_out[i].header.stamp = tm;
    pose_out[i].header.frame_id = frame_id;
  }
  setPoseTargets(pose_out, end_effector_link);
}

void MoveGroup::setPoseTargets(const std::vector<geometry_msgs::Pose> &target, const std::string &end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> target_stamped(target.size());
  ros::Time tm = ros::Time::now();
  const std::string &frame_id = getPoseReferenceFrame();
  for (std::size_t i = 0 ; i < target.size() ; ++i)
  {
    target_stamped[i].pose = target[i];
    target_stamped[i].header.stamp = tm;
    target_stamped[i].header.frame_id = frame_id;
  }
  setPoseTargets(target_stamped, end_effector_link);
}

void MoveGroup::setPoseTargets(const std::vector<geometry_msgs::PoseStamped> &target, const std::string &end_effector_link)
{
  if (target.empty())
    ROS_ERROR("No pose specified as goal target");
  else
  {
    impl_->setPoseTargets(target, end_effector_link);
    impl_->usePoseTarget();
  }
}

const geometry_msgs::PoseStamped& MoveGroup::getPoseTarget(const std::string &end_effector_link) const
{
  return impl_->getPoseTarget(end_effector_link);
}

const std::vector<geometry_msgs::PoseStamped>& MoveGroup::getPoseTargets(const std::string &end_effector_link) const
{
  return impl_->getPoseTargets(end_effector_link);
}

namespace
{
inline void transformPose(const tf::Transformer& tf, const std::string &desired_frame, geometry_msgs::PoseStamped &target)
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

void MoveGroup::setPositionTarget(double x, double y, double z, const std::string &end_effector_link)
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
  setPoseTarget(target, end_effector_link);
}

void MoveGroup::setOrientationTarget(double x, double y, double z, const std::string &end_effector_link)
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
  
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(x, y, z), target.pose.orientation);
  setPoseTarget(target, end_effector_link);
}

void MoveGroup::setOrientationTarget(double x, double y, double z, double w, const std::string &end_effector_link)
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
  setPoseTarget(target, end_effector_link);
}

void MoveGroup::followConstraints(const std::vector<moveit_msgs::Constraints> &constraints)
{
  impl_->followConstraints(constraints); 
  impl_->useFollowTarget();
}

void MoveGroup::followConstraints(const std::vector<geometry_msgs::PoseStamped> &poses, double tolerance_pos, double tolerance_angle, const std::string &end_effector_link)
{
  const std::string &eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  if (eef.empty())
    ROS_ERROR("No end-effector to specify trajectory following constraints for");
  else
  {
    std::vector<moveit_msgs::Constraints> constraints(poses.size());
    for (std::size_t i = 0 ; i < poses.size() ; ++i)
      constraints[i] = kinematic_constraints::constructGoalConstraints(eef, poses[i], tolerance_pos, tolerance_angle);
    followConstraints(constraints);
  }
}

void MoveGroup::followConstraints(const std::vector<geometry_msgs::Pose> &poses, double tolerance_pos, double tolerance_angle, const std::string &end_effector_link)
{ 
  std::vector<geometry_msgs::PoseStamped> pose_msgs(poses.size()); 
  const std::string &frame_id = getPoseReferenceFrame();
  ros::Time tm = ros::Time(0);
  for (std::size_t i = 0 ; i < poses.size() ; ++i)
  {
    pose_msgs[i].pose = poses[i]; 
    pose_msgs[i].header.frame_id = frame_id;
    pose_msgs[i].header.stamp = tm;
  }  
  followConstraints(pose_msgs, tolerance_pos, tolerance_angle, end_effector_link);
}

void MoveGroup::followConstraints(const EigenSTL::vector_Affine3d &poses, double tolerance_pos, double tolerance_angle, const std::string &end_effector_link)
{
  std::vector<geometry_msgs::PoseStamped> pose_msgs(poses.size()); 
  const std::string &frame_id = getPoseReferenceFrame();
  ros::Time tm = ros::Time(0);
  for (std::size_t i = 0 ; i < poses.size() ; ++i)
  {
    tf::poseEigenToMsg(poses[i], pose_msgs[i].pose);
    pose_msgs[i].header.frame_id = frame_id;
    pose_msgs[i].header.stamp = tm;
  }
  followConstraints(pose_msgs, tolerance_pos, tolerance_angle, end_effector_link);
}

void MoveGroup::setPoseReferenceFrame(const std::string &pose_reference_frame)
{
  impl_->setPoseReferenceFrame(pose_reference_frame);
}

const std::string& MoveGroup::getPoseReferenceFrame() const
{
  return impl_->getPoseReferenceFrame();
}

double MoveGroup::getGoalTolerance() const
{
  return impl_->getGoalTolerance();
}

void MoveGroup::setGoalTolerance(double tolerance)
{
  impl_->setGoalTolerance(tolerance);
}

void MoveGroup::rememberJointValues(const std::string &name)
{
  rememberJointValues(name, getCurrentJointValues());
}

std::vector<double> MoveGroup::getCurrentJointValues()
{ 
  robot_state::RobotStatePtr current_state;
  std::vector<double> values;
  if (impl_->getCurrentState(current_state))
    current_state->getJointStateGroup(getName())->getVariableValues(values);
  return values;
}

std::vector<double> MoveGroup::getRandomJointValues()
{
  std::vector<double> backup;
  impl_->getJointStateTarget()->getVariableValues(backup);
  
  impl_->getJointStateTarget()->setToRandomValues();
  std::vector<double> r;
  impl_->getJointStateTarget()->getVariableValues(r);
  
  impl_->getJointStateTarget()->setVariableValues(backup);
  return r;
}

geometry_msgs::PoseStamped MoveGroup::getRandomPose(const std::string &end_effector_link)
{    
  const std::string &eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  Eigen::Affine3d pose;
  pose.setIdentity();
  if (eef.empty())
    ROS_ERROR("No end-effector specified");
  else
  {  
    robot_state::RobotStatePtr current_state;
    if (impl_->getCurrentState(current_state))
    {
      current_state->getJointStateGroup(getName())->setToRandomValues();
      const robot_state::LinkState *ls = current_state->getLinkState(eef);
      if (ls)
        pose = ls->getGlobalLinkTransform();
    }
  }
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = impl_->getRobotModel()->getModelFrame();
  tf::poseEigenToMsg(pose, pose_msg.pose);
  return pose_msg;
}

geometry_msgs::PoseStamped MoveGroup::getCurrentPose(const std::string &end_effector_link)
{
  const std::string &eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  Eigen::Affine3d pose;
  pose.setIdentity();
  if (eef.empty())
    ROS_ERROR("No end-effector specified");
  else
  {
    robot_state::RobotStatePtr current_state;
    if (impl_->getCurrentState(current_state))
    {
      const robot_state::LinkState *ls = current_state->getLinkState(eef);
      if (ls)
        pose = ls->getGlobalLinkTransform();
    }
  }
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = impl_->getRobotModel()->getModelFrame();
  tf::poseEigenToMsg(pose, pose_msg.pose);
  return pose_msg;
}

const std::vector<std::string>& MoveGroup::getJoints() const
{
  return impl_->getJointStateTarget()->getJointModelGroup()->getJointModelNames();
}

robot_state::RobotStatePtr MoveGroup::getCurrentState()
{
  robot_state::RobotStatePtr current_state;
  impl_->getCurrentState(current_state);
  return current_state;
}

void MoveGroup::rememberJointValues(const std::string &name, const std::vector<double> &values)
{ 
  remembered_joint_values_[name] = values;
}

void MoveGroup::forgetJointValues(const std::string &name)
{
  remembered_joint_values_.erase(name);
}

void MoveGroup::allowLooking(bool flag)
{
  impl_->allowLooking(flag);
}

void MoveGroup::allowReplanning(bool flag)
{
  impl_->allowReplanning(flag);
}

std::vector<std::string> MoveGroup::getKnownConstraints() const
{
  return impl_->getKnownConstraints();
}

bool MoveGroup::setPathConstraints(const std::string &constraint)
{
  return impl_->setPathConstraints(constraint);
}

void MoveGroup::setPathConstraints(const moveit_msgs::Constraints &constraint)
{
  impl_->setPathConstraints(constraint);
}
    
void MoveGroup::clearPathConstraints()
{
  impl_->clearPathConstraints();
}

void MoveGroup::setConstraintsDatabase(const std::string &host, unsigned int port)
{  
  impl_->initializeConstraintsStorage(host, port);
}

void MoveGroup::setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz)
{
  impl_->setWorkspace(minx, miny, minz, maxx, maxy, maxz);
}

void MoveGroup::setPlanningTime(double seconds)
{
  impl_->setPlanningTime(seconds);
}

void MoveGroup::setSupportSurfaceName(const std::string &name)
{
  impl_->setSupportSurfaceName(name);
}

}
