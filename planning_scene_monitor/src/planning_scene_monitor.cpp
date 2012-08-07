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

/* Author: Ioan Sucan */

#include "planning_scene_monitor/planning_scene_monitor.h"
#include <planning_models_loader/kinematic_model_loader.h>

#include <dynamic_reconfigure/server.h>
#include "planning_scene_monitor/PlanningSceneMonitorDynamicReconfigureConfig.h"

namespace planning_scene_monitor
{

class PlanningSceneMonitor::DynamicReconfigureImpl
{ 
public:
  
  DynamicReconfigureImpl(PlanningSceneMonitor *owner) : owner_(owner)
  {
    dynamic_reconfigure_server_.setCallback(boost::bind(&DynamicReconfigureImpl::dynamicReconfigureCallback, this, _1, _2));
  }
  
private:
  
  void dynamicReconfigureCallback(PlanningSceneMonitorDynamicReconfigureConfig &config, uint32_t level)
  {
    if (config.groups.scene_publisher.publish_planning_scene)
      owner_->startPublishingPlanningScene();
    else
      owner_->stopPublishingPlanningScene();
    owner_->setPlanningScenePublishingFrequency(config.groups.scene_publisher.publish_planning_scene_hz);
  }
  
  PlanningSceneMonitor *owner_;
  dynamic_reconfigure::Server<planning_scene_monitor::PlanningSceneMonitorDynamicReconfigureConfig> dynamic_reconfigure_server_;
};

}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const std::string &robot_description, const boost::shared_ptr<tf::Transformer> &tf) :
  nh_("~"), tf_(tf)
{  
  kinematics_loader_.reset(new planning_models_loader::KinematicModelLoader(robot_description));
  initialize(planning_scene::PlanningScenePtr());
}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const planning_scene::PlanningScenePtr &scene, const std::string &robot_description, const boost::shared_ptr<tf::Transformer> &tf) :
  nh_("~"), tf_(tf)
{
  kinematics_loader_.reset(new planning_models_loader::KinematicModelLoader(robot_description));
  initialize(scene);
}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const planning_models_loader::KinematicModelLoaderPtr &kml, const boost::shared_ptr<tf::Transformer> &tf) :
  nh_("~"), tf_(tf), kinematics_loader_(kml)
{
  initialize(planning_scene::PlanningScenePtr());
}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const planning_scene::PlanningScenePtr &scene, const planning_models_loader::KinematicModelLoaderPtr &kml, const boost::shared_ptr<tf::Transformer> &tf) :
  nh_("~"), tf_(tf), kinematics_loader_(kml)
{
  initialize(scene);
}

planning_scene_monitor::PlanningSceneMonitor::~PlanningSceneMonitor(void)
{
  stopPublishingPlanningScene();
  stopStateMonitor();
  stopWorldGeometryMonitor();
  stopSceneMonitor();
}

void planning_scene_monitor::PlanningSceneMonitor::initialize(const planning_scene::PlanningScenePtr &scene)
{
  bounds_error_ = std::numeric_limits<double>::epsilon();
  
  robot_description_ = kinematics_loader_->getRobotDescription();
  if (kinematics_loader_->getModel())
  {
    scene_ = scene ? scene : planning_scene::PlanningScenePtr(new planning_scene::PlanningScene());
    if (scene_->isConfigured() || scene_->configure(kinematics_loader_->getURDF(), kinematics_loader_->getSRDF() ? kinematics_loader_->getSRDF() : boost::shared_ptr<srdf::Model>(new srdf::Model()), kinematics_loader_->getModel()))
    {
      scene_const_ = scene_;
      configureCollisionMatrix(scene_);
      configureDefaultPadding();
      
      scene_->getCollisionRobot()->setPadding(default_robot_padd_);
      scene_->getCollisionRobot()->setScale(default_robot_scale_);
    }
    else
    {
      ROS_ERROR("Configuration of planning scene failed");
      scene_.reset();
    }
  }
  else
  {
    ROS_ERROR("Kinematic model not loaded");
  }
  occupancy_map_monitor::OccupancyMapMonitor::Options opt;
  opt.map_frame = scene_->getPlanningFrame();
  octomap_monitor_.reset(new occupancy_map_monitor::OccupancyMapMonitor(opt, tf_));
  octomap_monitor_->setUpdateCallback(boost::bind(&PlanningSceneMonitor::octomapUpdateCallback, this));
  
  publish_planning_scene_frequency_ = 2.0;
  new_scene_update_ = false;
  
  last_update_time_ = ros::Time::now();
  last_state_update_ = ros::WallTime::now();
  dt_state_update_ = 0.1;

  reconfigure_impl_.reset(new DynamicReconfigureImpl(this));
}

void planning_scene_monitor::PlanningSceneMonitor::monitorDiffs(bool flag)
{
  if (scene_)
  {
    boost::mutex::scoped_lock slock(scene_update_mutex_);
    scene_->decoupleParent();
    if (flag)
    {
      parent_scene_ = scene_;
      scene_ = parent_scene_->diff();
    }
    else
    { 
      parent_scene_.reset();
      if (publish_planning_scene_)
      {
        ROS_WARN("Diff monitoring was stopped while publishing planning scene diffs. Stopping planning scene diff publisher");
        stopPublishingPlanningScene();
      }
    }
  }
}

void planning_scene_monitor::PlanningSceneMonitor::stopPublishingPlanningScene(void)
{ 
  if (publish_planning_scene_)
  {
    boost::scoped_ptr<boost::thread> copy;
    copy.swap(publish_planning_scene_);
    new_scene_update_condition_.notify_all();
    copy->join();
    monitorDiffs(false);
    planning_scene_publisher_.shutdown(); 
    ROS_INFO("Stopped publishing maintained planning scene.");
  }
}

void planning_scene_monitor::PlanningSceneMonitor::startPublishingPlanningScene(const std::string &planning_scene_topic)
{
  if (!publish_planning_scene_)
  {
    planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>(planning_scene_topic, 100, false);
    ROS_INFO("Publishing maintained planning scene on '%s'", planning_scene_topic.c_str());
    monitorDiffs(true);
    publish_planning_scene_.reset(new boost::thread(boost::bind(&PlanningSceneMonitor::scenePublishingThread, this)));
  }
}

void planning_scene_monitor::PlanningSceneMonitor::scenePublishingThread(void)
{
  moveit_msgs::PlanningScene msg;
  scene_->getPlanningSceneMsg(msg);
  planning_scene_publisher_.publish(msg);
  
  bool have_diff = false;
  do 
  {
    have_diff = false;
    ros::Rate rate(publish_planning_scene_frequency_);
    {
      boost::unique_lock<boost::mutex> ulock(scene_update_mutex_);
      while (!new_scene_update_ && publish_planning_scene_)
        new_scene_update_condition_.wait(ulock);
      if (new_scene_update_)
      {
        rate.reset();
        scene_->getPlanningSceneDiffMsg(msg);  
        scene_->pushDiffs(parent_scene_);
        scene_->clearDiffs();
        
        new_scene_update_ = false;
        have_diff = true;
      }
    }
    if (have_diff)
    {
      planning_scene_publisher_.publish(msg);
      rate.sleep();
    }
  }
  while (have_diff && publish_planning_scene_);
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
  if (collision_object_subscriber_)
    topics.push_back(collision_object_subscriber_->getTopic());
  if (collision_map_subscriber_)
    topics.push_back(collision_map_subscriber_->getTopic());
  if (planning_scene_world_subscriber_)
    topics.push_back(planning_scene_world_subscriber_.getTopic());
}

void planning_scene_monitor::PlanningSceneMonitor::processSceneUpdateEvent(SceneUpdateType update_type)
{
  if (update_callback_)
    update_callback_(update_type);
  new_scene_update_ = true;
  new_scene_update_condition_.notify_all();
}

void planning_scene_monitor::PlanningSceneMonitor::newPlanningSceneCallback(const moveit_msgs::PlanningSceneConstPtr &scene)
{
  if (scene_)
  {
    {
      boost::mutex::scoped_lock slock(scene_update_mutex_);  
      last_update_time_ = ros::Time::now();
      scene_->usePlanningSceneMsg(*scene);
    }
    processSceneUpdateEvent(UPDATE_SCENE);
  }
}

void planning_scene_monitor::PlanningSceneMonitor::newPlanningSceneWorldCallback(const moveit_msgs::PlanningSceneWorldConstPtr &world)
{
  if (scene_)
  {
    updateFrameTransforms();
    {
      boost::mutex::scoped_lock slock(scene_update_mutex_);
      last_update_time_ = ros::Time::now();
      scene_->processCollisionMapMsg(world->collision_map);
      for (std::size_t i = 0 ; i < world->collision_objects.size() ; ++i)
        scene_->processCollisionObjectMsg(world->collision_objects[i]);
    }  
    processSceneUpdateEvent(UPDATE_GEOMETRY);
  }
}

void planning_scene_monitor::PlanningSceneMonitor::collisionObjectCallback(const moveit_msgs::CollisionObjectConstPtr &obj)
{
  if (scene_)
  {
    {
      boost::mutex::scoped_lock slock(scene_update_mutex_);
      last_update_time_ = ros::Time::now();
      scene_->processCollisionObjectMsg(*obj);
    }
    processSceneUpdateEvent(UPDATE_GEOMETRY);
  }
}

void planning_scene_monitor::PlanningSceneMonitor::attachObjectCallback(const moveit_msgs::AttachedCollisionObjectConstPtr &obj)
{
  if (scene_)
  {    
    updateFrameTransforms();
    {
      boost::mutex::scoped_lock slock(scene_update_mutex_);
      last_update_time_ = ros::Time::now();
      scene_->processAttachedCollisionObjectMsg(*obj);
    }
    processSceneUpdateEvent(UPDATE_GEOMETRY);
  }
}

void planning_scene_monitor::PlanningSceneMonitor::collisionMapCallback(const moveit_msgs::CollisionMapConstPtr &map)
{
  if (scene_)
  {  
    updateFrameTransforms();
    {
      boost::mutex::scoped_lock slock(scene_update_mutex_);
      last_update_time_ = ros::Time::now();
      scene_->processCollisionMapMsg(*map);
    }
    processSceneUpdateEvent(UPDATE_GEOMETRY);
  }
}

void planning_scene_monitor::PlanningSceneMonitor::lockScene(void)
{
  scene_update_mutex_.lock();
  octomap_monitor_->lockOcTree();
}

void planning_scene_monitor::PlanningSceneMonitor::unlockScene(void)
{
  scene_update_mutex_.unlock();
  octomap_monitor_->unlockOcTree();
}

void planning_scene_monitor::PlanningSceneMonitor::startSceneMonitor(const std::string &scene_topic)
{
  stopSceneMonitor();
  
  ROS_INFO("Starting scene monitor");
  // listen for planning scene updates; these messages include transforms, so no need for filters
  if (!scene_topic.empty())
  {
    planning_scene_subscriber_ = root_nh_.subscribe(scene_topic, 100, &PlanningSceneMonitor::newPlanningSceneCallback, this);
    ROS_INFO("Listening to '%s'", scene_topic.c_str());
  }
}

void planning_scene_monitor::PlanningSceneMonitor::stopSceneMonitor(void)
{
  if (planning_scene_subscriber_)
  {
    ROS_INFO("Stopping scene monitor");
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
    collision_object_subscriber_.reset(new message_filters::Subscriber<moveit_msgs::CollisionObject>(root_nh_, collision_objects_topic, 1024));
    if (tf_)
    {
      collision_object_filter_ .reset(new tf::MessageFilter<moveit_msgs::CollisionObject>(*collision_object_subscriber_, *tf_, scene_->getPlanningFrame(), 1024));
      collision_object_filter_->registerCallback(boost::bind(&PlanningSceneMonitor::collisionObjectCallback, this, _1));
      ROS_INFO("Listening to '%s' using message notifier with target frame '%s'", collision_objects_topic.c_str(), collision_object_filter_->getTargetFramesString().c_str());
    }
    else
    {
      collision_object_subscriber_->registerCallback(boost::bind(&PlanningSceneMonitor::collisionObjectCallback, this, _1));
      ROS_INFO("Listening to '%s'", collision_objects_topic.c_str());
    }
  }
  
  if (!collision_map_topic.empty())
  {
    // listen to collision map using filters
    collision_map_subscriber_.reset(new message_filters::Subscriber<moveit_msgs::CollisionMap>(root_nh_, collision_map_topic, 2));
    if (tf_)
    {
      collision_map_filter_.reset(new tf::MessageFilter<moveit_msgs::CollisionMap>(*collision_map_subscriber_, *tf_, scene_->getPlanningFrame(), 2));
      collision_map_filter_->registerCallback(boost::bind(&PlanningSceneMonitor::collisionMapCallback, this, _1));
      ROS_INFO("Listening to '%s' using message notifier with target frame '%s'", collision_map_topic.c_str(), collision_map_filter_->getTargetFramesString().c_str());
    }
    else
    {
      collision_map_subscriber_->registerCallback(boost::bind(&PlanningSceneMonitor::collisionMapCallback, this, _1)); 
      ROS_INFO("Listening to '%s'", collision_map_topic.c_str());
    }
  }
  
  if (!planning_scene_world_topic.empty())
  {
    planning_scene_world_subscriber_ = root_nh_.subscribe(planning_scene_world_topic, 1, &PlanningSceneMonitor::newPlanningSceneWorldCallback, this);
    ROS_INFO("Listening to '%s' for planning scene world geometry", planning_scene_world_topic.c_str());
  }

  octomap_monitor_->startMonitor();
}

void planning_scene_monitor::PlanningSceneMonitor::stopWorldGeometryMonitor(void)
{
  if (collision_object_subscriber_ || collision_object_filter_ ||
      collision_map_subscriber_ || collision_map_filter_)
  {
    ROS_INFO("Stopping world geometry monitor");
    collision_object_filter_.reset();
    collision_object_subscriber_.reset();
    collision_map_filter_.reset();
    collision_map_subscriber_.reset();
    planning_scene_world_subscriber_.shutdown();
  }
  else
    if (planning_scene_world_subscriber_)
    {
      ROS_INFO("Stopping world geometry monitor");
      planning_scene_world_subscriber_.shutdown();
    } 
  octomap_monitor_->stopMonitor();
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
      attached_collision_object_subscriber_ = root_nh_.subscribe(attached_objects_topic, 1024, &PlanningSceneMonitor::attachObjectCallback, this);
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
    attached_collision_object_subscriber_.shutdown();
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

void planning_scene_monitor::PlanningSceneMonitor::octomapUpdateCallback(void)
{
  updateFrameTransforms();
  {
    boost::mutex::scoped_lock slock(scene_update_mutex_);
    last_update_time_ = ros::Time::now();
    octomap_monitor_->lockOcTree();
    try
    {
      scene_->processOctomapPtr(octomap_monitor_->getOcTreePtr(), Eigen::Affine3d::Identity());
      octomap_monitor_->unlockOcTree();
    }
    catch(...)
    {
      octomap_monitor_->unlockOcTree(); // unlock and rethrow
      throw;
    }    
  }
  processSceneUpdateEvent(UPDATE_GEOMETRY);
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
    if (!current_state_monitor_->haveCompleteState(missing))
    {
      std::string missing_str = boost::algorithm::join(missing, ", ");
      ROS_WARN("The complete state of the robot is not yet known.  Missing %s", missing_str.c_str());
    }
    
    {
      boost::mutex::scoped_lock slock(scene_update_mutex_);
      const std::map<std::string, double> &v = current_state_monitor_->getCurrentStateValues();
      scene_->getCurrentState().setStateValues(v);
      last_update_time_ = ros::Time::now();
    }
    processSceneUpdateEvent(UPDATE_STATE);
  }
  else
    ROS_ERROR("State monitor is not active. Unable to set the planning scene state");
}

void planning_scene_monitor::PlanningSceneMonitor::setUpdateCallback(const boost::function<void(SceneUpdateType)> &fn)
{
  update_callback_ = fn;
}

void planning_scene_monitor::PlanningSceneMonitor::setPlanningScenePublishingFrequency(double hz)
{
  publish_planning_scene_frequency_ = hz;
}

void planning_scene_monitor::PlanningSceneMonitor::getUpdatedFrameTransforms(const planning_models::KinematicModelConstPtr &kmodel, std::vector<geometry_msgs::TransformStamped> &transforms)
{
  if (!tf_)
    return;
  const std::string &target = kmodel->getModelFrame();
  
  std::vector<std::string> all_frame_names;
  tf_->getFrameStrings(all_frame_names);
  for (std::size_t i = 0 ; i < all_frame_names.size() ; ++i)
  {
    if (!all_frame_names[i].empty() && all_frame_names[i][0] == '/')
      all_frame_names[i].erase(all_frame_names[i].begin());
    
    if (all_frame_names[i] == target || kmodel->hasLinkModel(all_frame_names[i]))
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
}

void planning_scene_monitor::PlanningSceneMonitor::updateFrameTransforms(void)
{
  if (!tf_)
    return;
  
  if (scene_)
  {
    std::vector<geometry_msgs::TransformStamped> transforms;
    getUpdatedFrameTransforms(scene_->getKinematicModel(), transforms);
    {
      boost::mutex::scoped_lock slock(scene_update_mutex_);
      scene_->getTransforms()->setTransforms(transforms);
      last_update_time_ = ros::Time::now();
    }
    processSceneUpdateEvent(UPDATE_TRANSFORMS);
  }
} 

void planning_scene_monitor::PlanningSceneMonitor::configureCollisionMatrix(const planning_scene::PlanningScenePtr &scene)
{
  if (!scene)
    return;
  collision_detection::AllowedCollisionMatrix &acm = scene->getAllowedCollisionMatrix();
  
  // no collisions allowed by default
  acm.setEntry(scene->getKinematicModel()->getLinkModelNamesWithCollisionGeometry(),
               scene->getKinematicModel()->getLinkModelNamesWithCollisionGeometry(), false);
  
  // allow collisions for pairs that have been disabled
  const std::vector<srdf::Model::DisabledCollision> &dc = scene->getSrdfModel()->getDisabledCollisionPairs();
  for (std::size_t i = 0 ; i < dc.size() ; ++i)
    acm.setEntry(dc[i].link1_, dc[i].link2_, true);
  
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
