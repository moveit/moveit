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
#include "move_group_interface/move_group.h"
#include <planning_models_loader/kinematic_model_loader.h>
#include <planning_scene_monitor/current_state_monitor.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kinematic_constraints/utils.h>
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

static planning_models::KinematicModelConstPtr getSharedKinematicModel(const std::string &robot_description)
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

static planning_scene_monitor::CurrentStateMonitorPtr getSharedStateMonitor(const planning_models::KinematicModelConstPtr &kmodel, const boost::shared_ptr<tf::Transformer> &tf)
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
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MoveGroupImpl(const Options &opt, const boost::shared_ptr<tf::Transformer> &tf) : opt_(opt), tf_(tf)
  {
    kinematic_model_ = getSharedKinematicModel(opt.robot_description_);
    if (!getKinematicModel())
    {
      std::string error = "Unable to construct robot model. Make sure all needed information is on the parameter server.";
      ROS_FATAL_STREAM(error);
      throw std::runtime_error(error);
    }
    
    if (!getKinematicModel()->hasJointModelGroup(opt.group_name_))
    {
      std::string error = "Group '" + opt.group_name_ + "' was not found.";
      ROS_FATAL_STREAM(error);
      throw std::runtime_error(error);
    }
    
    joint_state_target_.reset(new planning_models::KinematicState(getKinematicModel()));
    joint_state_target_->setToDefaultValues();
    use_joint_state_target_ = true;
    can_look_ = true;
    
    const planning_models::KinematicModel::JointModelGroup *joint_model_group = getKinematicModel()->getJointModelGroup(opt.group_name_);
    if (joint_model_group)
    {
      if (joint_model_group->isChain())
        end_effector_ = joint_model_group->getLinkModelNames().back();
      pose_target_.setIdentity();
      pose_reference_frame_ = getKinematicModel()->getModelFrame();
      
      trajectory_event_publisher_ = node_handle_.advertise<std_msgs::String>("trajectory_execution_event", 1, false);
      
      current_state_monitor_ = getSharedStateMonitor(kinematic_model_, tf_);
      action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>("move_group", false));
      ROS_INFO_STREAM("Waiting for MoveGroup action server...");
      action_client_->waitForServer();
      ROS_INFO_STREAM("Ready to take MoveGroup commands for group " << opt.group_name_ << ".");
    }
    else
      ROS_ERROR("Unable to initialize MoveGroup interface.");
  }

  const boost::shared_ptr<tf::Transformer>& getTF(void) const
  {
    return tf_;
  }
  
  const Options& getOptions(void) const
  {
    return opt_;
  }
  
  const planning_models::KinematicModelConstPtr& getKinematicModel(void)
  {
    return kinematic_model_;
  }
  
  planning_models::KinematicState::JointStateGroup* getJointStateTarget(void)
  {
    return joint_state_target_->getJointStateGroup(opt_.group_name_);
  }
  
  void setEndEffector(const std::string &end_effector)
  {
    end_effector_ = end_effector;
  }
  
  const std::string &getEndEffector(void) const
  {
    return end_effector_;
  }
  
  void setPoseTarget(const Eigen::Affine3d &eef_pose)
  {
    pose_target_ = eef_pose;
  }
  
  const Eigen::Affine3d& getPoseTarget(void) const
  {
    return pose_target_;
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
  
  std::vector<double> getCurrentJointValues(void)
  {
    if (!current_state_monitor_)
      return std::vector<double>();
    
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
    
    std::vector<double> v;
    current_state_monitor_->getCurrentState()->getJointStateGroup(opt_.group_name_)->getGroupStateValues(v);
    return v;
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
  
  bool move(void)
  {    
    if (!action_client_)
      return false;
    if (!action_client_->isServerConnected())
      return false;
    moveit_msgs::MoveGroupGoal goal;
    constructGoal(goal);
    goal.plan_only = false;
    goal.look_around = can_look_;
    action_client_->sendGoal(goal);
    return true;
  }

  bool move2(unsigned int attempt_count, unsigned int max_attempts)
  {  
    if (!action_client_)
      return false;
    if (!action_client_->isServerConnected())
      return false;

    if (attempt_count >= max_attempts)
    {
      ROS_WARN_STREAM("Unable to get to goal after " << max_attempts << " attempts");
      return false;
    }
    moveit_msgs::MoveGroupGoal goal;
    constructGoal(goal);
    goal.plan_only = false;
    goal.look_around = can_look_;
    action_client_->sendGoal(goal);
    if (!action_client_->waitForResult())
    {
      ROS_INFO_STREAM("MoveGroup action returned early");
    }
    if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      return true;
    else
    {
      const moveit_msgs::MoveItErrorCodes &err = action_client_->getResult()->error_code;
      // under certain conditions we attempt to execute the motion again
      if (err.val == moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE ||
          err.val == moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN ||
          err.val == moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA ||
          err.val == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED)
      {
        attempt_count++;
        ROS_INFO_STREAM("Attempt " << attempt_count << " of " << max_attempts << " failed: " << action_client_->getState().toString() << ": " << action_client_->getState().getText());
        return move2(attempt_count, max_attempts);
      }
      else
        return false;
    }
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
  
  void constructGoal(moveit_msgs::MoveGroupGoal &goal)
  {
    goal.request.group_name = opt_.group_name_;
    goal.request.num_planning_attempts = 1;
    goal.request.allowed_planning_time = ros::Duration(5.0);

    if (use_joint_state_target_)
    {    
      goal.request.goal_constraints.resize(1);
      goal.request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(getJointStateTarget());
    }
    else
    {
      geometry_msgs::PoseStamped pose;
      tf::poseEigenToMsg(pose_target_, pose.pose);
      pose.header.frame_id = pose_reference_frame_;
      pose.header.stamp = ros::Time::now();
      goal.request.goal_constraints.resize(1);
      goal.request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(end_effector_, pose);
    }    
  }
  
private:
  
  Options opt_;
  ros::NodeHandle node_handle_;
  boost::shared_ptr<tf::Transformer> tf_;
  planning_models::KinematicModelConstPtr kinematic_model_;
  planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;
  boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> > action_client_;
  planning_models::KinematicStatePtr joint_state_target_;
  ros::Publisher trajectory_event_publisher_;
  Eigen::Affine3d pose_target_;
  std::string end_effector_;
  std::string pose_reference_frame_;
  bool can_look_;
  
  bool use_joint_state_target_;
};

MoveGroup::MoveGroup(const std::string &group_name, const boost::shared_ptr<tf::Transformer> &tf)
{
  if (!ros::ok())
    throw std::runtime_error("ROS does not seem to be running");
  impl_ = new MoveGroupImpl(Options(group_name), tf ? tf : getSharedTF());
}

MoveGroup::MoveGroup(const Options &opt, const boost::shared_ptr<tf::Transformer> &tf)
{
  impl_ = new MoveGroupImpl(opt, tf ? tf : getSharedTF());
}

MoveGroup::~MoveGroup(void)
{
  delete impl_;
}

const std::string& MoveGroup::getName(void) const
{
  return impl_->getOptions().group_name_;
}

bool MoveGroup::asyncMove(void)
{
  return impl_->move();
}

bool MoveGroup::move(unsigned int max_attempts)
{
  return impl_->move2(0, max_attempts);
}
    
bool MoveGroup::plan(Plan &plan)
{
  return impl_->plan(plan);
}

void MoveGroup::stop(void)
{
  impl_->stop();
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
  impl_->getJointStateTarget()->setStateValues(joint_values);
  impl_->useJointStateTarget();
}

void MoveGroup::setJointValueTarget(const std::map<std::string, double> &joint_values)
{
  impl_->getJointStateTarget()->setStateValues(joint_values);
  impl_->useJointStateTarget();
}

void MoveGroup::setJointValueTarget(const planning_models::KinematicState &kinematic_state)
{
  setJointValueTarget(*kinematic_state.getJointStateGroup(impl_->getOptions().group_name_));
}

void MoveGroup::setJointValueTarget(const planning_models::KinematicState::JointStateGroup &joint_state_group)
{  
  std::map<std::string, double> variable_values;
  joint_state_group.getGroupStateValues(variable_values);
  setJointValueTarget(variable_values);
}

void MoveGroup::setJointValueTarget(const planning_models::KinematicState::JointState &joint_state)
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
  planning_models::KinematicState::JointState *joint_state = impl_->getJointStateTarget()->getJointState(joint_name);
  if (joint_state)
    if (!joint_state->setVariableValues(values))
      ROS_ERROR("Unable to set target");
  impl_->useJointStateTarget();
}

void MoveGroup::setJointValueTarget(const sensor_msgs::JointState &state)
{
  impl_->getJointStateTarget()->setStateValues(state); 
  impl_->useJointStateTarget();
}

const planning_models::KinematicState::JointStateGroup& MoveGroup::getJointValueTarget(void) const
{
  return *impl_->getJointStateTarget();
}

void MoveGroup::setPoseTarget(const Eigen::Affine3d &pose)
{
  impl_->setPoseTarget(pose);
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

void MoveGroup::setPoseTarget(const geometry_msgs::Pose &target)
{
  Eigen::Affine3d m;
  tf::poseMsgToEigen(target, m);
  setPoseTarget(m);
}

void MoveGroup::setPoseTarget(const geometry_msgs::PoseStamped &target)
{
  // avoid using the TransformListener API, since out pointer is for the base class only;
  tf::Pose pose;
  tf::poseMsgToTF(target.pose, pose);
  tf::Stamped<tf::Pose> stamped_pose(pose, target.header.stamp, target.header.frame_id);
  tf::Stamped<tf::Pose> stamped_pose_out;
  impl_->getTF()->transformPose(impl_->getPoseReferenceFrame(), stamped_pose, stamped_pose_out);

  geometry_msgs::Pose pose_out;
  tf::poseTFToMsg(stamped_pose_out, pose_out);
  setPoseTarget(pose_out);
}

const Eigen::Affine3d& MoveGroup::getPoseTarget(void) const
{
  return impl_->getPoseTarget();
}

void MoveGroup::setPoseReferenceFrame(const std::string &pose_reference_frame)
{
  impl_->setPoseReferenceFrame(pose_reference_frame);
}

const std::string& MoveGroup::getPoseReferenceFrame(void) const
{
  return impl_->getPoseReferenceFrame();
}

void MoveGroup::rememberJointValues(const std::string &name)
{
  rememberJointValues(name, impl_->getCurrentJointValues());
}

std::vector<double> MoveGroup::getCurrentJointValues(void)
{
  return impl_->getCurrentJointValues();
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

}
