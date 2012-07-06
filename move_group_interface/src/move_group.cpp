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
    action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>("move_group", false));
    kinematic_model_ = getSharedKinematicModel(opt.robot_description_);
    if (!getKinematicModel())
      ROS_FATAL_STREAM("Unable to construct robot model. Make sure all needed information is on the parameter server.");
    if (!getKinematicModel()->hasJointModelGroup(opt.group_name_))
      ROS_FATAL_STREAM("Group '" + opt.group_name_ + "' was not found.");
    
    joint_state_target_.reset(new planning_models::KinematicState(getKinematicModel()));
    joint_state_target_->setToDefaultValues();
    use_joint_state_target_ = true;
    
    const planning_models::KinematicModel::JointModelGroup *joint_model_group = getKinematicModel()->getJointModelGroup(opt.group_name_);
    if (joint_model_group->isChain())
      end_effector_ = joint_model_group->getLinkModelNames().back();
    pose_target_.setIdentity();
    pose_reference_frame_ = getKinematicModel()->getModelFrame();

    trajectory_event_publisher_ = node_handle_.advertise<std_msgs::String>("trajectory_execution_event", 1, false);
    
    current_state_monitor_ = getSharedStateMonitor(kinematic_model_, tf_);
    action_client_->waitForServer();
    ROS_INFO_STREAM("Ready to take MoveGroup commands for group " << opt.group_name_ << ".");
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

  bool plan(Plan &plan)
  {
    moveit_msgs::MoveGroupGoal goal;
    constructGoal(goal);
    goal.plan_only = true;
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
    moveit_msgs::MoveGroupGoal goal;
    constructGoal(goal);
    goal.plan_only = false;
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
      ROS_WARN_STREAM("Fail: " << action_client_->getState().toString() << ": " << action_client_->getState().getText());
      return false;
    }
  }
  
  void stop(void)
  {
    std_msgs::String event;
    event.data = "stop";
    trajectory_event_publisher_.publish(event);
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
  
  bool use_joint_state_target_;
};

MoveGroup::MoveGroup(const std::string &group_name, const boost::shared_ptr<tf::Transformer> &tf)
{  
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

bool MoveGroup::move(bool wait)
{
  return impl_->move(wait);
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

void MoveGroup::setNamedTarget(const std::string &name)
{ 
  impl_->getJointStateTarget()->setToDefaultState(name);
  impl_->useJointStateTarget();
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


}
