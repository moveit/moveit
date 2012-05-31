/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan, E. Gil Jones */

#include "planning_scene_monitor/planning_scene_monitor.h"
#include <robot_model_loader/robot_model_loader.h>

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const std::string &robot_description) :
  nh_("~")
{
  initialize(planning_scene::PlanningSceneConstPtr(), robot_description);
}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const std::string &robot_description, const boost::shared_ptr<tf::Transformer> &tf) :
  nh_("~"), tf_(tf)
{
  initialize(planning_scene::PlanningSceneConstPtr(), robot_description);
}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const planning_scene::PlanningSceneConstPtr &parent, const std::string &robot_description, const boost::shared_ptr<tf::Transformer> &tf) :
  nh_("~"), tf_(tf)
{
  initialize(parent, robot_description);
}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const std::string &robot_description, const kinematics_plugin_loader::KinematicsPluginLoaderPtr &kpl) :
  nh_("~"), kinematics_loader_(kpl)
{
  initialize(planning_scene::PlanningSceneConstPtr(), robot_description);
}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const std::string &robot_description, const boost::shared_ptr<tf::Transformer> &tf,
                                                                   const kinematics_plugin_loader::KinematicsPluginLoaderPtr &kpl) :
  nh_("~"), tf_(tf), kinematics_loader_(kpl)
{
  initialize(planning_scene::PlanningSceneConstPtr(), robot_description);
}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const planning_scene::PlanningSceneConstPtr &parent, const std::string &robot_description, const boost::shared_ptr<tf::Transformer> &tf,
                                                                   const kinematics_plugin_loader::KinematicsPluginLoaderPtr &kpl) :
  nh_("~"), tf_(tf), kinematics_loader_(kpl)
{
  initialize(parent, robot_description);
}

planning_scene_monitor::PlanningSceneMonitor::~PlanningSceneMonitor(void)
{
  stopStateMonitor();
  stopWorldGeometryMonitor();
  stopSceneMonitor();
}

void planning_scene_monitor::PlanningSceneMonitor::initialize(const planning_scene::PlanningSceneConstPtr &parent, const std::string &robot_description)
{ 
  bounds_error_ = std::numeric_limits<double>::epsilon();
  collision_object_subscriber_ = NULL;
  collision_object_filter_ = NULL;
  attached_collision_object_subscriber_ = NULL;
  collision_map_subscriber_ = NULL;
  collision_map_filter_ = NULL;
  
  robot_model_loader::RobotModelLoader rml(robot_description);
  robot_description_ = rml.getRobotDescription();
  if (rml.getURDF())
  {
    if (parent)
      scene_.reset(new planning_scene::PlanningScene(parent));
    else
      scene_.reset(new planning_scene::PlanningScene());
    if (scene_->configure(rml.getURDF(), rml.getSRDF() ? rml.getSRDF() : boost::shared_ptr<srdf::Model>(new srdf::Model())))
    {
      scene_const_ = scene_;
      configureDefaultCollisionMatrix();
      configureDefaultPadding();
      configureDefaultJointLimits();
      scene_->getCollisionRobot()->setPadding(default_robot_padd_);
      scene_->getCollisionRobot()->setScale(default_robot_scale_);
    }
    else
    {
      ROS_ERROR("Configuration of planning scene failed");
      scene_.reset();
    }
  }
  
  if (scene_ && scene_->getKinematicModel())
  {
    // load the kinematics solvers
    if (!kinematics_loader_)
      kinematics_loader_.reset(new kinematics_plugin_loader::KinematicsPluginLoader());
    kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_loader_->getLoaderFunction();
    const std::vector<std::string> &groups = kinematics_loader_->getKnownGroups();
    std::map<std::string, planning_models::KinematicModel::SolverAllocatorFn> imap;
    for (std::size_t i = 0 ; i < groups.size() ; ++i)
      imap[groups[i]] =  kinematics_allocator;
    scene_->getKinematicModel()->setKinematicsAllocators(imap);
  }
  
  last_update_time_ = ros::Time::now();
  last_state_update_ = ros::WallTime::now();
  dt_state_update_ = 0.2;
}

void planning_scene_monitor::PlanningSceneMonitor::monitorDiffs(bool flag)
{
  if (scene_)
  {
    boost::mutex::scoped_lock slock(scene_update_mutex_);
    scene_->decoupleParent();
    if (flag)
      scene_.reset(new planning_scene::PlanningScene(scene_));
  }
}

void planning_scene_monitor::PlanningSceneMonitor::getMonitoredTopics(std::vector<std::string> &topics) const
{
  topics.clear();
  if (current_state_monitor_)
  {
    const std::string &t = current_state_monitor_->getMonitoredTopic();
    if (!t.empty())
      topics.push_back(t);
  }
  if (planning_scene_subscriber_)
    topics.push_back(planning_scene_subscriber_.getTopic());
  if (planning_scene_diff_subscriber_)
    topics.push_back(planning_scene_diff_subscriber_.getTopic());
  if (collision_object_subscriber_)
    topics.push_back(collision_object_subscriber_->getTopic());
  if (collision_map_subscriber_)
    topics.push_back(collision_map_subscriber_->getTopic());
  if (planning_scene_world_subscriber_)
    topics.push_back(planning_scene_world_subscriber_.getTopic());
}

void planning_scene_monitor::PlanningSceneMonitor::newPlanningSceneCallback(const moveit_msgs::PlanningSceneConstPtr &scene)
{
  if (scene_)
  {
    {
      boost::mutex::scoped_lock slock(scene_update_mutex_);
      scene_->setPlanningSceneMsg(*scene);
      last_update_time_ = ros::Time::now();
    }
    if (update_callback_)
      update_callback_();
  }
}

void planning_scene_monitor::PlanningSceneMonitor::newPlanningSceneDiffCallback(const moveit_msgs::PlanningSceneConstPtr &scene)
{
  if (scene_)
  {
    {
      boost::mutex::scoped_lock slock(scene_update_mutex_);
      scene_->setPlanningSceneDiffMsg(*scene);
      last_update_time_ = ros::Time::now();
    }
    if (update_callback_)
      update_callback_();
  }
}

void planning_scene_monitor::PlanningSceneMonitor::newPlanningSceneWorldCallback(const moveit_msgs::PlanningSceneWorldConstPtr &world)
{
  if (scene_)
  {
    {
      boost::mutex::scoped_lock slock(scene_update_mutex_);
      scene_->processCollisionMapMsg(world->collision_map);
      for (std::size_t i = 0 ; i < world->collision_objects.size() ; ++i)
        scene_->processCollisionObjectMsg(world->collision_objects[i]);
      last_update_time_ = ros::Time::now();
    }
    if (update_callback_)
      update_callback_();
  }
}

void planning_scene_monitor::PlanningSceneMonitor::collisionObjectCallback(const moveit_msgs::CollisionObjectConstPtr &obj)
{
  if (scene_)
  {
    {
      boost::mutex::scoped_lock slock(scene_update_mutex_);
      scene_->processCollisionObjectMsg(*obj);
      last_update_time_ = ros::Time::now();
    }
    if (update_callback_)
      update_callback_();
  }
}

void planning_scene_monitor::PlanningSceneMonitor::attachObjectCallback(const moveit_msgs::AttachedCollisionObjectConstPtr &obj)
{
  if (scene_)
  {
    {
      boost::mutex::scoped_lock slock(scene_update_mutex_);
      scene_->processAttachedCollisionObjectMsg(*obj);
      last_update_time_ = ros::Time::now();
    }
    if (update_callback_)
      update_callback_();
  }
}

void planning_scene_monitor::PlanningSceneMonitor::collisionMapCallback(const moveit_msgs::CollisionMapConstPtr &map)
{
  if (scene_)
  {
    {
      boost::mutex::scoped_lock slock(scene_update_mutex_);
      scene_->processCollisionMapMsg(*map);
      last_update_time_ = ros::Time::now();
    }
    if (update_callback_)
      update_callback_();
  }
}

void planning_scene_monitor::PlanningSceneMonitor::lockScene(void)
{
  scene_update_mutex_.lock();
}

void planning_scene_monitor::PlanningSceneMonitor::unlockScene(void)
{
  scene_update_mutex_.unlock();
}

void planning_scene_monitor::PlanningSceneMonitor::startSceneMonitor(const std::string &scene_topic, const std::string &scene_diff_topic)
{
  stopSceneMonitor();
  
  ROS_INFO("Starting scene monitor");
  // listen for planning scene updates; these messages include transforms, so no need for filters
  if (!scene_topic.empty())
    planning_scene_subscriber_ = root_nh_.subscribe(scene_topic, 2, &PlanningSceneMonitor::newPlanningSceneCallback, this);
  if (!scene_diff_topic.empty())
    planning_scene_diff_subscriber_ = root_nh_.subscribe(scene_diff_topic, 100, &PlanningSceneMonitor::newPlanningSceneDiffCallback, this);
  ROS_INFO("Listening to '%s' and '%s'", scene_topic.c_str(), scene_diff_topic.c_str());
}

void planning_scene_monitor::PlanningSceneMonitor::stopSceneMonitor(void)
{
  if (planning_scene_diff_subscriber_ || planning_scene_subscriber_)
  {
    ROS_INFO("Stopping scene monitor");
    planning_scene_diff_subscriber_.shutdown();
    planning_scene_subscriber_.shutdown();
  }
}

void planning_scene_monitor::PlanningSceneMonitor::startWorldGeometryMonitor(const std::string &collision_objects_topic,
                                                                             const std::string &collision_map_topic,
                                                                             const std::string &planning_scene_world_topic)
{
  stopWorldGeometryMonitor();
  ROS_INFO("Starting world geometry monitor");
  
  // listen for world geometry updates using message filters
  if (!collision_objects_topic.empty())
  {
    collision_object_subscriber_ = new message_filters::Subscriber<moveit_msgs::CollisionObject>(root_nh_, collision_objects_topic, 1024);
    collision_object_filter_ = new tf::MessageFilter<moveit_msgs::CollisionObject>(*collision_object_subscriber_, *tf_, scene_->getPlanningFrame(), 1024);
    collision_object_filter_->registerCallback(boost::bind(&PlanningSceneMonitor::collisionObjectCallback, this, _1));
    ROS_INFO("Listening to '%s' using message notifier with target frame '%s'", collision_objects_topic.c_str(), collision_object_filter_->getTargetFramesString().c_str());
  }
  
  if (!collision_map_topic.empty())
  {
    // listen to collision map using filters
    collision_map_subscriber_ = new message_filters::Subscriber<moveit_msgs::CollisionMap>(root_nh_, collision_map_topic, 2);
    collision_map_filter_ = new tf::MessageFilter<moveit_msgs::CollisionMap>(*collision_map_subscriber_, *tf_, scene_->getPlanningFrame(), 2);
    collision_map_filter_->registerCallback(boost::bind(&PlanningSceneMonitor::collisionMapCallback, this, _1));
    ROS_INFO("Listening to '%s' using message notifier with target frame '%s'", collision_map_topic.c_str(), collision_map_filter_->getTargetFramesString().c_str());
  }
  
  if (!planning_scene_world_topic.empty())
  {
    planning_scene_world_subscriber_ = root_nh_.subscribe(planning_scene_world_topic, 1, &PlanningSceneMonitor::newPlanningSceneWorldCallback, this);
    ROS_INFO("Listening to '%s' for planning scene world geometry", planning_scene_world_topic.c_str());
  }
}

void planning_scene_monitor::PlanningSceneMonitor::stopWorldGeometryMonitor(void)
{
  if (collision_object_subscriber_ || collision_object_filter_ ||
      collision_map_subscriber_ || collision_map_filter_)
  {
    ROS_INFO("Stopping world geometry monitor");
    delete collision_object_filter_;
    delete collision_object_subscriber_;
    delete collision_map_filter_;
    delete collision_map_subscriber_;
    collision_object_subscriber_ = NULL;
    collision_object_filter_ = NULL;
    collision_map_subscriber_ = NULL;
    collision_map_filter_ = NULL;
    planning_scene_world_subscriber_.shutdown();
  }
  else
    if (planning_scene_world_subscriber_)
    {
      ROS_INFO("Stopping world geometry monitor");
      planning_scene_world_subscriber_.shutdown();
    }
}

void planning_scene_monitor::PlanningSceneMonitor::startStateMonitor(const std::string &joint_states_topic, const std::string &attached_objects_topic)
{
  stopStateMonitor();
  if (scene_)
  {
    if (!current_state_monitor_)
      current_state_monitor_.reset(new CurrentStateMonitor(scene_->getKinematicModel(), tf_));  
    current_state_monitor_->setBoundsError(bounds_error_);
    current_state_monitor_->setOnStateUpdateCallback(boost::bind(&PlanningSceneMonitor::onStateUpdate, this, _1));
    current_state_monitor_->startStateMonitor(joint_states_topic);
    
    if (!attached_objects_topic.empty())
    {
      // using regular message filter as there's no header
      attached_collision_object_subscriber_ = new message_filters::Subscriber<moveit_msgs::AttachedCollisionObject>(root_nh_, attached_objects_topic, 1024);
      attached_collision_object_subscriber_->registerCallback(boost::bind(&PlanningSceneMonitor::attachObjectCallback, this, _1));
      ROS_INFO("Listening to '%s' for attached collision objects", attached_objects_topic.c_str());
    }
  }
  else
    ROS_ERROR("Cannot monitor robot state because planning scene is not configured");
}

void planning_scene_monitor::PlanningSceneMonitor::stopStateMonitor(void)
{
  if (current_state_monitor_)
    current_state_monitor_->stopStateMonitor();
  if (attached_collision_object_subscriber_)
  {
    delete attached_collision_object_subscriber_;
    attached_collision_object_subscriber_ = NULL;
  }
}

void planning_scene_monitor::PlanningSceneMonitor::onStateUpdate(const sensor_msgs::JointStateConstPtr & /* joint_state */ )
{
  const ros::WallTime &n = ros::WallTime::now();
  const double t = (n - last_state_update_).toSec();
  if (t >= dt_state_update_)
  {
    last_state_update_ = n;
    updateSceneWithCurrentState();
  }
}

void planning_scene_monitor::PlanningSceneMonitor::setStateUpdateFrequency(double hz)
{
  if (hz > std::numeric_limits<double>::epsilon())
    dt_state_update_ = 1.0 / hz;
  else
    dt_state_update_ = 0.0;
  ROS_INFO("Updating internal planning scene state at most every %lf seconds", dt_state_update_);
}

void planning_scene_monitor::PlanningSceneMonitor::setStateUpdateBoundsError(double error)
{
  bounds_error_ = error;
  if (current_state_monitor_)
    current_state_monitor_->setBoundsError(error);
}

void planning_scene_monitor::PlanningSceneMonitor::updateSceneWithCurrentState(void)
{
  if (current_state_monitor_)
  {
    std::vector<std::string> missing;
    if (!current_state_monitor_->haveCompleteState(missing)) {
      std::string missing_str = boost::algorithm::join(missing, ", ");
      ROS_WARN("The complete state of the robot is not yet known.  Missing %s", missing_str.c_str());
    }
    
    {
      boost::mutex::scoped_lock slock(scene_update_mutex_);
      const std::map<std::string, double> &v = current_state_monitor_->getCurrentStateValues();
      scene_->getCurrentState().setStateValues(v);
      last_update_time_ = ros::Time::now();
    }
    if (update_callback_)
      update_callback_();
  }
  else
    ROS_ERROR("State monitor is not active. Unable to set the planning scene state");
}

void planning_scene_monitor::PlanningSceneMonitor::setUpdateCallback(const boost::function<void()> &fn)
{
  update_callback_ = fn;
}

void planning_scene_monitor::PlanningSceneMonitor::updateFrameTransforms(void)
{
  if (!tf_ || !scene_)
    return;
  std::vector<geometry_msgs::TransformStamped> transforms;
  const std::string &target = scene_->getPlanningFrame();
  
  std::vector<std::string> all_frame_names;
  tf_->getFrameStrings(all_frame_names);
  for (std::size_t i = 0 ; i < all_frame_names.size() ; ++i)
  {
    if (!all_frame_names[i].empty() && all_frame_names[i][0] == '/')
      all_frame_names[i].erase(all_frame_names[i].begin());
    
    if (all_frame_names[i] == target || scene_->getKinematicModel()->hasLinkModel(all_frame_names[i]))
      continue;
    
    ros::Time stamp;
    std::string err_string;
    if (tf_->getLatestCommonTime(target, all_frame_names[i], stamp, &err_string) != tf::NO_ERROR)
    {
      ROS_WARN_STREAM("No transform available between frame '" << all_frame_names[i] << "' and planning frame '" <<
                      target << "' (" << err_string << ")");
      continue;
    }
    
    tf::StampedTransform t;
    try
    {
      tf_->lookupTransform(target, all_frame_names[i], stamp, t);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN_STREAM("Unable to transform object from frame '" << all_frame_names[i] << "' to planning frame '" <<
                      target << "' (" << ex.what() << ")");
      continue;
    }
    geometry_msgs::TransformStamped f;
    f.header.frame_id = all_frame_names[i];
    f.child_frame_id = target;
    f.transform.translation.x = t.getOrigin().x();
    f.transform.translation.y = t.getOrigin().y();
    f.transform.translation.z = t.getOrigin().z();
    const tf::Quaternion &q = t.getRotation();
    f.transform.rotation.x = q.x();
    f.transform.rotation.y = q.y();
    f.transform.rotation.z = q.z();
    f.transform.rotation.w = q.w();
    transforms.push_back(f);
  }
  {
    boost::mutex::scoped_lock slock(scene_update_mutex_);
    scene_->getTransforms()->setTransforms(transforms);
    last_update_time_ = ros::Time::now();
  }
  if (update_callback_)
    update_callback_();
}

void planning_scene_monitor::PlanningSceneMonitor::configureDefaultCollisionMatrix(void)
{
  if (!scene_)
    return;
  
  collision_detection::AllowedCollisionMatrix &acm = scene_->getAllowedCollisionMatrix();
  
  // no collisions allowed by default
  acm.setEntry(scene_->getKinematicModel()->getLinkModelNamesWithCollisionGeometry(),
               scene_->getKinematicModel()->getLinkModelNamesWithCollisionGeometry(), false);
  
  // allow collisions for pairs that have been disabled
  const std::vector<std::pair<std::string, std::string> >&dc = scene_->getSrdfModel()->getDisabledCollisions();
  for (std::size_t i = 0 ; i < dc.size() ; ++i)
    acm.setEntry(dc[i].first, dc[i].second, true);
  
  // read overriding values from the param server
  
  // first we do default collision operations
  if (!nh_.hasParam(robot_description_ + "_planning/default_collision_operations"))
    ROS_DEBUG("No additional default collision operations specified");
  else
  {
    ROS_DEBUG("Reading additional default collision operations");
    
    XmlRpc::XmlRpcValue coll_ops;
    nh_.getParam(robot_description_ + "_planning/default_collision_operations", coll_ops);
    
    if (coll_ops.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN("default_collision_operations is not an array");
      return;
    }
    
    if (coll_ops.size() == 0)
    {
      ROS_WARN("No collision operations in default collision operations");
      return;
    }
    
    for (int i = 0 ; i < coll_ops.size() ; ++i)
    {
      if (!coll_ops[i].hasMember("object1") || !coll_ops[i].hasMember("object2") || !coll_ops[i].hasMember("operation"))
      {
        ROS_WARN("All collision operations must have two objects and an operation");
        continue;
      }
      acm.setEntry(std::string(coll_ops[i]["object1"]), std::string(coll_ops[i]["object2"]), std::string(coll_ops[i]["operation"]) == "disable");
    }
  }
}

void planning_scene_monitor::PlanningSceneMonitor::configureDefaultPadding(void)
{
  nh_.param(robot_description_ + "_planning/default_robot_padding", default_robot_padd_, 0.0);
  nh_.param(robot_description_ + "_planning/default_robot_scale", default_robot_scale_, 1.0);
  nh_.param(robot_description_ + "_planning/default_object_padding", default_object_padd_, 0.0);
  nh_.param(robot_description_ + "_planning/default_attached_padding", default_attached_padd_, 0.0);
}

void planning_scene_monitor::PlanningSceneMonitor::configureDefaultJointLimits(void)
{
  for(unsigned int i = 0; i < scene_->getKinematicModel()->getJointModels().size(); ++i) 
  {
    std::vector<moveit_msgs::JointLimits> jlim = scene_->getKinematicModel()->getJointModels()[i]->getVariableLimits();
    for(unsigned int j = 0; j < jlim.size(); j++) {
      std::string prefix = robot_description_+"_planning/joint_limits/"+jlim[j].joint_name+"/";
      ROS_DEBUG_STREAM("Joint " << jlim[j].joint_name << " before vel " << jlim[j].max_velocity);
      bool has_vel = jlim[j].has_velocity_limits;
      nh_.param(prefix+"has_velocity_limits", has_vel, has_vel);
      jlim[j].has_velocity_limits = has_vel;
      nh_.param(prefix+"max_velocity", jlim[j].max_velocity, jlim[j].max_velocity);
      bool has_accel = jlim[j].has_acceleration_limits;
      nh_.param(prefix+"has_acceleration_limits", has_accel, has_accel); 
      jlim[j].has_acceleration_limits = has_accel;
      nh_.param(prefix+"max_acceleration", jlim[j].max_acceleration, jlim[j].max_acceleration);
      ROS_DEBUG_STREAM("Joint " << jlim[j].joint_name << " after vel " << jlim[j].max_velocity);
      ROS_DEBUG_STREAM("Joint " << jlim[j].joint_name << " accel " << jlim[j].max_acceleration);
    }
    individual_joint_limits_map_[scene_->getKinematicModel()->getJointModels()[i]->getName()] = jlim;
  }
  planning_models::KinematicModelPtr kmodel = scene_->getKinematicModel();
  const std::map<std::string, planning_models::KinematicModel::JointModelGroup*>& jmgm = scene_->getKinematicModel()->getJointModelGroupMap();
  for(std::map<std::string, planning_models::KinematicModel::JointModelGroup*>::const_iterator it = jmgm.begin(); it != jmgm.end(); ++it) 
  {
    for(unsigned int i = 0; i < it->second->getJointModelNames().size(); i++)
    {
      group_joint_limits_map_[it->first].insert(group_joint_limits_map_[it->first].end(),
                                                individual_joint_limits_map_[it->second->getJointModelNames()[i]].begin(),
                                                individual_joint_limits_map_[it->second->getJointModelNames()[i]].end());
    }
    // forward this information to the planning group
    if (kmodel)
      kmodel->getJointModelGroup(it->first)->setAdditionalLimits(group_joint_limits_map_[it->first]);
  }
}
