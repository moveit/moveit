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
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_models_loader/kinematic_model_loader.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/kinematic_state/conversions.h>
#include <moveit_msgs/MoveGroupAction.h>
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

static boost::shared_ptr<tf::Transformer> getSharedTF(void)
{
  static boost::shared_ptr<tf::Transformer> tf(new tf::TransformListener());
  return tf;
}

static kinematic_model::KinematicModelConstPtr getSharedKinematicModel(const std::string &robot_description)
{
  static std::map<std::string, planning_models_loader::KinematicModelLoaderPtr> model_loaders;
  static boost::mutex lock;
  boost::mutex::scoped_lock slock(lock);
  if (model_loaders.find(robot_description) != model_loaders.end())
    return model_loaders[robot_description]->getModel();
  else
  {
    planning_models_loader::KinematicModelLoader::Options opt(robot_description);
    opt.load_kinematics_solvers_ = false;
    planning_models_loader::KinematicModelLoaderPtr loader(new planning_models_loader::KinematicModelLoader(opt));
    model_loaders[robot_description] = loader;
    return loader->getModel();
  }
} 

static planning_scene_monitor::CurrentStateMonitorPtr getSharedStateMonitor(const kinematic_model::KinematicModelConstPtr &kmodel, const boost::shared_ptr<tf::Transformer> &tf)
{
  static std::map<std::string, planning_scene_monitor::CurrentStateMonitorPtr> state_monitors;
  static boost::mutex lock;
  boost::mutex::scoped_lock slock(lock);
  if (state_monitors.find(kmodel->getName()) != state_monitors.end())
    return state_monitors[kmodel->getName()];
  else
  {
    planning_scene_monitor::CurrentStateMonitorPtr monitor(new planning_scene_monitor::CurrentStateMonitor(kmodel, tf));
    state_monitors[kmodel->getName()] = monitor;
    return monitor;
  }
}

class MoveGroup::MoveGroupImpl
{
public: 
  
  MoveGroupImpl(const Options &opt, const boost::shared_ptr<tf::Transformer> &tf, const ros::Duration &wait_for_server) : opt_(opt), tf_(tf)
  {
    kinematic_model_ = opt.kinematic_model_ ? opt.kinematic_model_ : getSharedKinematicModel(opt.robot_description_);
    if (!getKinematicModel())
    {
      std::string error = "Unable to construct robot model. Please make sure all needed information is on the parameter server.";
      ROS_FATAL_STREAM(error);
      throw std::runtime_error(error);
    }

    if (!getKinematicModel()->hasJointModelGroup(opt.group_name_))
    {
      std::string error = "Group '" + opt.group_name_ + "' was not found.";
      ROS_FATAL_STREAM(error);
      throw std::runtime_error(error);
    }

    joint_state_target_.reset(new kinematic_state::KinematicState(getKinematicModel()));
    joint_state_target_->setToDefaultValues();
    use_joint_state_target_ = true;
    can_look_ = false;
    can_replan_ = false;
    goal_tolerance_ = 1e-4;
    planning_time_ = ros::Duration(5.0);
    
    const kinematic_model::JointModelGroup *joint_model_group = getKinematicModel()->getJointModelGroup(opt.group_name_);
    if (joint_model_group)
    {
      if (joint_model_group->isChain())
      {
        end_effector_ = joint_model_group->getLinkModelNames().back();
        pose_target_[end_effector_].setIdentity();
      }      
      pose_reference_frame_ = getKinematicModel()->getModelFrame();
      
      trajectory_event_publisher_ = node_handle_.advertise<std_msgs::String>("trajectory_execution_event", 1, false);
      
      current_state_monitor_ = getSharedStateMonitor(kinematic_model_, tf_);
      action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>("move_group", false));
      ROS_INFO_STREAM("Waiting for MoveGroup action server...");

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
        while (node_handle_.ok() && !action_client_->isServerConnected())
        {
          ros::WallDuration(0.02).sleep();
          ros::spinOnce();
        }
      }
      else
      {
        ros::Time final_time = ros::Time::now() + wait_for_server;
        while (node_handle_.ok() && !action_client_->isServerConnected() && final_time > ros::Time::now())
        {
          ros::WallDuration(0.02).sleep();
          ros::spinOnce();
        }
      }
      
      if (!action_client_->isServerConnected())
        throw std::runtime_error("Unable to connect to action server within allotted time");
      
      execute_service_ = node_handle_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("execute_kinematic_path");
      query_service_ = node_handle_.serviceClient<moveit_msgs::QueryPlannerInterfaces>("query_planner_interface");
      initializeConstraintsStorage();
      ROS_INFO_STREAM("Ready to take MoveGroup commands for group " << opt.group_name_ << ".");
    }
    else
      ROS_ERROR("Unable to initialize MoveGroup interface.");
  }

  ~MoveGroupImpl(void)
  {
    if (constraints_init_thread_)
    {
      terminate_constraints_init_thread_ = true;
      constraints_init_thread_->join();
    }
  }
  
  const boost::shared_ptr<tf::Transformer>& getTF(void) const
  {
    return tf_;
  }
  
  const Options& getOptions(void) const
  {
    return opt_;
  }
  
  const kinematic_model::KinematicModelConstPtr& getKinematicModel(void)
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
  
  kinematic_state::JointStateGroup* getJointStateTarget(void)
  {
    return joint_state_target_->getJointStateGroup(opt_.group_name_);
  }
  
  void setStartState(const kinematic_state::KinematicState &start_state)
  {
    considered_start_state_.reset(new kinematic_state::KinematicState(start_state));
  }

  void setStartStateToCurrentState(void)
  {
    considered_start_state_.reset();
  }
  
  void setEndEffector(const std::string &end_effector)
  {
    end_effector_ = end_effector;
    if (pose_target_.find(end_effector_) == pose_target_.end())
      pose_target_[end_effector_].setIdentity();
  }
  
  void clearPoseTarget(const std::string &end_effector_link)
  {
    if (end_effector_link == end_effector_)
      pose_target_[end_effector_].setIdentity();
    else
      pose_target_.erase(end_effector_link);
    pose_targets_[end_effector_link].reset();
  }

  void clearPoseTargets(void)
  {
    pose_target_.clear();
    pose_targets_.clear();
    if (!end_effector_.empty())
      pose_target_[end_effector_].setIdentity();
  }
  
  const std::string &getEndEffector(void) const
  {
    return end_effector_;
  }
  
  void setPoseTarget(const Eigen::Affine3d &eef_pose, const std::string &end_effector_link)
  {
    const std::string &eef = end_effector_link.empty() ? end_effector_ : end_effector_link;
    if (eef.empty())
      ROS_ERROR("No end-effector to set the pose for");
    else
    {
      pose_target_[eef] = eef_pose;
      pose_targets_[eef].reset();
    }
  }
  
  void setPoseTargets(const EigenSTL::vector_Affine3d &poses, const std::string &end_effector_link)
  {  
    const std::string &eef = end_effector_link.empty() ? end_effector_ : end_effector_link;
    if (eef.empty())
      ROS_ERROR("No end-effector to set the pose for");
    else
      pose_targets_[eef].reset(new EigenSTL::vector_Affine3d(poses));
  }  

  const Eigen::Affine3d& getPoseTarget(const std::string &end_effector_link) const
  {    
    const std::string &eef = end_effector_link.empty() ? end_effector_ : end_effector_link; 
    
    // if multiple pose targets are set, return the first one
    std::map<std::string, boost::shared_ptr<EigenSTL::vector_Affine3d> >::const_iterator jt = pose_targets_.find(eef);
    if (jt != pose_targets_.end())
      if (!jt->second->empty())
        return jt->second->at(0);

    // otherwise, return the one that is set,
    EigenSTL::map_string_Affine3d::const_iterator it = pose_target_.find(eef);
    if (it != pose_target_.end())
      return it->second;
    else
    {
      // or return an error
      static const Eigen::Affine3d id(Eigen::Affine3d::Identity());
      ROS_ERROR("Pose for end effector '%s' not known. Assuming identity.", eef.c_str());
      return id;
    }
  } 

  boost::shared_ptr<EigenSTL::vector_Affine3d> getPoseTargets(const std::string &end_effector_link) const
  {
    const std::string &eef = end_effector_link.empty() ? end_effector_ : end_effector_link; 
    
    // if multiple pose targets are set, return the first one
    std::map<std::string, boost::shared_ptr<EigenSTL::vector_Affine3d> >::const_iterator jt = pose_targets_.find(eef);
    if (jt != pose_targets_.end())
      if (!jt->second->empty())
        return jt->second;
    EigenSTL::map_string_Affine3d::const_iterator it = pose_target_.find(eef);
    if (it != pose_target_.end())
      return boost::shared_ptr<EigenSTL::vector_Affine3d>(new EigenSTL::vector_Affine3d(1, it->second));
    ROS_ERROR("Poses for end effector '%s' are not known. Returning NULL pointer.", eef.c_str());
    
    return boost::shared_ptr<EigenSTL::vector_Affine3d>();
  }
  
  void setPoseReferenceFrame(const std::string &pose_reference_frame)
  {
    pose_reference_frame_ = pose_reference_frame;
  }
  
  const std::string& getPoseReferenceFrame(void) const
  {
    return pose_reference_frame_;
  }
  
  void useJointStateTarget(void)
  {
    use_joint_state_target_ = true;
  }
  
  void usePoseTarget(void)
  {
    use_joint_state_target_ = false;
  }
  
  void allowLooking(bool flag)
  {
    can_look_ = flag;
  }

  void allowReplanning(bool flag)
  {
    can_replan_ = flag;
  }
  
  bool getCurrentState(kinematic_state::KinematicStatePtr &current_state)
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
  
  bool plan(Plan &plan)
  {
    if (!action_client_)
      return false;
    if (!action_client_->isServerConnected())
      return false;

    moveit_msgs::MoveGroupGoal goal;
    constructGoal(goal);
    goal.plan_only = true;
    goal.look_around = false;
    goal.replan = false;
    action_client_->sendGoal(goal); 
    if (!action_client_->waitForResult())
    {
      ROS_INFO_STREAM("MoveGroup action returned early");
    }
    if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      plan.trajectory_ = action_client_->getResult()->planned_trajectory;
      plan.start_state_ = action_client_->getResult()->trajectory_start;
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Fail: " << action_client_->getState().toString() << ": " << action_client_->getState().getText());
      return false;
    }
  }
  
  bool move(bool wait)
  {  
    if (!action_client_)
      return false;
    if (!action_client_->isServerConnected())
      return false;

    moveit_msgs::MoveGroupGoal goal;
    constructGoal(goal);
    goal.plan_only = false;
    goal.look_around = can_look_;  
    goal.replan = can_replan_;

    action_client_->sendGoal(goal);
    if (!wait)
      return true;
    
    if (!action_client_->waitForResult())
    {
      ROS_INFO_STREAM("MoveGroup action returned early");
    }
    
    if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      return true;
    else
    {
      ROS_INFO_STREAM(action_client_->getState().toString() << ": " << action_client_->getState().getText());
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
  
  void stop(void)
  {
    if (trajectory_event_publisher_)
    {
      std_msgs::String event;
      event.data = "stop";
      trajectory_event_publisher_.publish(event);
    }
  }
  
  double getGoalTolerance(void) const
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
      planning_time_ = ros::Duration(seconds);
  }
  
  void constructGoal(moveit_msgs::MoveGroupGoal &goal)
  {
    goal.request.group_name = opt_.group_name_;
    goal.request.num_planning_attempts = 1;
    goal.request.allowed_planning_time = planning_time_;
    goal.request.planner_id = planner_id_;
    goal.request.workspace_parameters = workspace_parameters_;
    
    if (considered_start_state_)
      kinematic_state::kinematicStateToRobotState(*considered_start_state_, goal.request.start_state);
    
    if (use_joint_state_target_)
    {    
      goal.request.goal_constraints.resize(1);
      goal.request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(getJointStateTarget(), goal_tolerance_);
    }
    else
    {
      goal.request.goal_constraints.clear();
      ros::Time t = ros::Time::now();
      for (EigenSTL::map_string_Affine3d::const_iterator it = pose_target_.begin() ; it != pose_target_.end() ; ++it)
      {
        geometry_msgs::PoseStamped pose;
        tf::poseEigenToMsg(it->second, pose.pose);
        pose.header.frame_id = pose_reference_frame_;
        pose.header.stamp = t;
        goal.request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(it->first, pose, goal_tolerance_));
      }
    }
    
    if (path_constraints_)
      goal.request.path_constraints = *path_constraints_;
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

  void clearPathConstraints(void)
  {
    path_constraints_.reset();
  }
  
  std::vector<std::string> getKnownConstraints(void) const
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
    workspace_parameters_.header.frame_id = getKinematicModel()->getModelFrame();
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
  
  Options opt_;
  ros::NodeHandle node_handle_;
  boost::shared_ptr<tf::Transformer> tf_;
  kinematic_model::KinematicModelConstPtr kinematic_model_;
  planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;
  boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> > action_client_;
  kinematic_state::KinematicStatePtr considered_start_state_;
  kinematic_state::KinematicStatePtr joint_state_target_;
  boost::scoped_ptr<moveit_msgs::Constraints> path_constraints_;
  moveit_msgs::WorkspaceParameters workspace_parameters_;
  ros::Duration planning_time_;
  EigenSTL::map_string_Affine3d pose_target_;
  std::map<std::string, boost::shared_ptr<EigenSTL::vector_Affine3d> > pose_targets_;
  std::string end_effector_;
  std::string pose_reference_frame_; 
  std::string planner_id_;
  double goal_tolerance_;
  bool can_look_;
  bool can_replan_;
  
  bool use_joint_state_target_;
  
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

MoveGroup::~MoveGroup(void)
{
  delete impl_;
}

const std::string& MoveGroup::getName(void) const
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

bool MoveGroup::asyncMove(void)
{
  return impl_->move(false);
}

bool MoveGroup::move(void)
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

void MoveGroup::stop(void)
{
  impl_->stop();
}

void MoveGroup::setStartState(const kinematic_state::KinematicState &start_state)
{
  impl_->setStartState(start_state);
}

void MoveGroup::setStartStateToCurrentState(void)
{
  impl_->setStartStateToCurrentState();
}

void MoveGroup::setRandomTarget(void)
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

void MoveGroup::setJointValueTarget(const kinematic_state::KinematicState &kinematic_state)
{
  setJointValueTarget(*kinematic_state.getJointStateGroup(getName()));
}

void MoveGroup::setJointValueTarget(const kinematic_state::JointStateGroup &joint_state_group)
{  
  std::map<std::string, double> variable_values;
  joint_state_group.getVariableValues(variable_values);
  setJointValueTarget(variable_values);
}

void MoveGroup::setJointValueTarget(const kinematic_state::JointState &joint_state)
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
  kinematic_state::JointState *joint_state = impl_->getJointStateTarget()->getJointState(joint_name);
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

const kinematic_state::JointStateGroup& MoveGroup::getJointValueTarget(void) const
{
  return *impl_->getJointStateTarget();
}

void MoveGroup::setPoseTarget(const Eigen::Affine3d &pose, const std::string &end_effector_link)
{
  impl_->setPoseTarget(pose, end_effector_link);
  impl_->usePoseTarget();
}

const std::string& MoveGroup::getEndEffectorLink(void) const
{
  return impl_->getEndEffector();
}

void MoveGroup::setEndEffectorLink(const std::string &link_name)
{
  impl_->setEndEffector(link_name);  
  impl_->usePoseTarget();
}

void MoveGroup::clearPoseTarget(const std::string &end_effector_link)
{
  impl_->clearPoseTarget(end_effector_link);
}

void MoveGroup::clearPoseTargets(void)
{
  impl_->clearPoseTargets();
}

void MoveGroup::setPoseTarget(const geometry_msgs::Pose &target, const std::string &end_effector_link)
{
  Eigen::Affine3d m;
  tf::poseMsgToEigen(target, m);
  setPoseTarget(m, end_effector_link);
}

void MoveGroup::setPoseTarget(const geometry_msgs::PoseStamped &target, const std::string &end_effector_link)
{
  // avoid using the TransformListener API, since out pointer is for the base class only;
  tf::Pose pose;
  tf::poseMsgToTF(target.pose, pose);
  tf::Stamped<tf::Pose> stamped_pose(pose, target.header.stamp, target.header.frame_id);
  tf::Stamped<tf::Pose> stamped_pose_out;
  impl_->getTF()->transformPose(impl_->getPoseReferenceFrame(), stamped_pose, stamped_pose_out);
  
  geometry_msgs::Pose pose_out;
  tf::poseTFToMsg(stamped_pose_out, pose_out);
  setPoseTarget(pose_out, end_effector_link);
}

void MoveGroup::setPoseTargets(const EigenSTL::vector_Affine3d &pose, const std::string &end_effector_link)
{
  if (pose.empty())
    ROS_ERROR("No pose specified as goal target");
  else
  {
    impl_->setPoseTargets(pose, end_effector_link);
    impl_->usePoseTarget();
  }
}

void MoveGroup::setPoseTargets(const std::vector<geometry_msgs::Pose> &target, const std::string &end_effector_link)
{
  EigenSTL::vector_Affine3d target_eigen(target.size());
  for (std::size_t i = 0 ; i < target.size() ; ++i)
    tf::poseMsgToEigen(target[i], target_eigen[i]);
  setPoseTargets(target_eigen, end_effector_link);
}

void MoveGroup::setPoseTargets(const std::vector<geometry_msgs::PoseStamped> &target, const std::string &end_effector_link)
{
  std::vector<geometry_msgs::Pose> pose_out_vector(target.size());
  for (std::size_t i = 0 ; i < target.size() ; ++i)
  {
    // avoid using the TransformListener API, since out pointer is for the base class only;
    tf::Pose pose;
    tf::poseMsgToTF(target[i].pose, pose);
    tf::Stamped<tf::Pose> stamped_pose(pose, target[i].header.stamp, target[i].header.frame_id);
    tf::Stamped<tf::Pose> stamped_pose_out;
    impl_->getTF()->transformPose(impl_->getPoseReferenceFrame(), stamped_pose, stamped_pose_out);
    
    tf::poseTFToMsg(stamped_pose_out, pose_out_vector[i]);
  }
  setPoseTargets(pose_out_vector, end_effector_link);
}

const Eigen::Affine3d& MoveGroup::getPoseTarget(const std::string &end_effector_link) const
{
  return impl_->getPoseTarget(end_effector_link);
}

void MoveGroup::getPoseTargets(EigenSTL::vector_Affine3d &poses, const std::string &end_effector_link) const
{
  boost::shared_ptr<EigenSTL::vector_Affine3d> p = impl_->getPoseTargets(end_effector_link);
  if (p)
    poses = *p;
  else
    poses.clear();
}

void MoveGroup::setPositionTarget(double x, double y, double z, const std::string &end_effector_link)
{
  Eigen::Affine3d target = getPoseTarget(end_effector_link);
  target.translation() = Eigen::Vector3d(x,y,z);
  setPoseTarget(target, end_effector_link);
}

void MoveGroup::setOrientationTarget(double x, double y, double z, const std::string &end_effector_link)
{ 
  Eigen::Affine3d target(Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX())
                         * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()));
  target.translation() = getPoseTarget(end_effector_link).translation();
  setPoseTarget(target, end_effector_link);
}

void MoveGroup::setPoseReferenceFrame(const std::string &pose_reference_frame)
{
  impl_->setPoseReferenceFrame(pose_reference_frame);
}

const std::string& MoveGroup::getPoseReferenceFrame(void) const
{
  return impl_->getPoseReferenceFrame();
}

double MoveGroup::getGoalTolerance(void) const
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

std::vector<double> MoveGroup::getCurrentJointValues(void)
{ 
  kinematic_state::KinematicStatePtr current_state;
  std::vector<double> values;
  if (impl_->getCurrentState(current_state))
    current_state->getJointStateGroup(getName())->getVariableValues(values);
  return values;
}

std::vector<double> MoveGroup::getRandomJointValues(void)
{
  std::vector<double> backup;
  impl_->getJointStateTarget()->getVariableValues(backup);
  
  impl_->getJointStateTarget()->setToRandomValues();
  std::vector<double> r;
  impl_->getJointStateTarget()->getVariableValues(r);
  
  impl_->getJointStateTarget()->setVariableValues(backup);
  return r;
}

Eigen::Affine3d MoveGroup::getCurrentPose(const std::string &end_effector_link)
{
  const std::string &eef = end_effector_link.empty() ? getEndEffectorLink() : end_effector_link;
  Eigen::Affine3d pose;
  pose.setIdentity();
  if (eef.empty())
    ROS_ERROR("No end-effector specified");
  else
  {
    kinematic_state::KinematicStatePtr current_state;
    if (impl_->getCurrentState(current_state))
    {
      const kinematic_state::LinkState *ls = current_state->getLinkState(eef);
      if (ls)
        pose = ls->getGlobalLinkTransform();
    }
  }  
  return pose;
}

const std::vector<std::string>& MoveGroup::getJoints(void) const
{
  return impl_->getJointStateTarget()->getJointModelGroup()->getJointModelNames();
}

kinematic_state::KinematicStatePtr MoveGroup::getCurrentState(void)
{
  kinematic_state::KinematicStatePtr current_state;
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

std::vector<std::string> MoveGroup::getKnownConstraints(void) const
{
  return impl_->getKnownConstraints();
}

bool MoveGroup::setPathConstraints(const std::string &constraint)
{
  return impl_->setPathConstraints(constraint);
}

void MoveGroup::clearPathConstraints(void)
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

}
