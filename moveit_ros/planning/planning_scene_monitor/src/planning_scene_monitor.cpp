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

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_models_loader/kinematic_model_loader.h>

#include <dynamic_reconfigure/server.h>
#include <moveit_ros_planning/PlanningSceneMonitorDynamicReconfigureConfig.h>

namespace planning_scene_monitor
{

using namespace moveit_ros_planning;

class PlanningSceneMonitor::DynamicReconfigureImpl
{ 
public:
  
  DynamicReconfigureImpl(PlanningSceneMonitor *owner) : owner_(owner),
							dynamic_reconfigure_server_(ros::NodeHandle(decideNamespace(owner->getName())))
  {
    dynamic_reconfigure_server_.setCallback(boost::bind(&DynamicReconfigureImpl::dynamicReconfigureCallback, this, _1, _2));
  }
  
private:

  // make sure we do not advertise the same service multiple times, in case we use multiple PlanningSceneMonitor instances in a process
  static std::string decideNamespace(const std::string &name)
  {
    std::string ns = "~/" + name; 
    std::replace(ns.begin(), ns.end(), ' ', '_');
    std::transform(ns.begin(), ns.end(), ns.begin(), ::tolower);
    if (ros::service::exists(ns + "/set_parameters", false))
    {
      unsigned int c = 1;
      while (ros::service::exists(ns + boost::lexical_cast<std::string>(c) + "/set_parameters", false))
        c++;
      ns += boost::lexical_cast<std::string>(c);
    }
    return ns;
  }
  
  void dynamicReconfigureCallback(PlanningSceneMonitorDynamicReconfigureConfig &config, uint32_t level)
  {   
    PlanningSceneMonitor::SceneUpdateType event = PlanningSceneMonitor::UPDATE_NONE;
    if (config.publish_geometry_updates)
      event = (PlanningSceneMonitor::SceneUpdateType) ((int)event | (int)PlanningSceneMonitor::UPDATE_GEOMETRY);
    if (config.publish_state_updates)
      event = (PlanningSceneMonitor::SceneUpdateType) ((int)event | (int)PlanningSceneMonitor::UPDATE_STATE);
    if (config.publish_transforms_updates)   
      event = (PlanningSceneMonitor::SceneUpdateType) ((int)event | (int)PlanningSceneMonitor::UPDATE_TRANSFORMS);
    if (config.publish_planning_scene)
    {
      owner_->setPlanningScenePublishingFrequency(config.publish_planning_scene_hz);
      owner_->startPublishingPlanningScene(event);
    }
    else
      owner_->stopPublishingPlanningScene();
  }
  
  PlanningSceneMonitor *owner_;
  dynamic_reconfigure::Server<planning_scene_monitor::PlanningSceneMonitorDynamicReconfigureConfig> dynamic_reconfigure_server_;
};

}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const std::string &robot_description, const boost::shared_ptr<tf::Transformer> &tf, const std::string &name) :
  nh_("~"), tf_(tf), monitor_name_(name)
{  
  kinematics_loader_.reset(new planning_models_loader::KinematicModelLoader(robot_description));
  initialize(planning_scene::PlanningScenePtr());
}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const planning_scene::PlanningScenePtr &scene, const std::string &robot_description,
                                                                   const boost::shared_ptr<tf::Transformer> &tf, const std::string &name) :
  nh_("~"), tf_(tf), monitor_name_(name)
{
  kinematics_loader_.reset(new planning_models_loader::KinematicModelLoader(robot_description));
  initialize(scene);
}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const planning_models_loader::KinematicModelLoaderPtr &kml,
                                                                   const boost::shared_ptr<tf::Transformer> &tf, const std::string &name) :
  nh_("~"), tf_(tf), kinematics_loader_(kml), monitor_name_(name)
{
  initialize(planning_scene::PlanningScenePtr());
}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const planning_scene::PlanningScenePtr &scene, const planning_models_loader::KinematicModelLoaderPtr &kml,
                                                                   const boost::shared_ptr<tf::Transformer> &tf, const std::string &name) :
  nh_("~"), tf_(tf), kinematics_loader_(kml), monitor_name_(name)
{
  initialize(scene);
}

planning_scene_monitor::PlanningSceneMonitor::~PlanningSceneMonitor(void)
{
  stopPublishingPlanningScene();
  stopStateMonitor();
  stopWorldGeometryMonitor();
  stopSceneMonitor();
  delete reconfigure_impl_;
}

void planning_scene_monitor::PlanningSceneMonitor::initialize(const planning_scene::PlanningScenePtr &scene)
{
  bounds_error_ = std::numeric_limits<double>::epsilon();
  if (monitor_name_.empty())
    monitor_name_ = "planning_scene_monitor";
  robot_description_ = kinematics_loader_->getRobotDescription();
  if (kinematics_loader_->getModel())
  {
    scene_ = scene ? scene : planning_scene::PlanningScenePtr(new planning_scene::PlanningScene());
    if (scene_->isConfigured() || scene_->configure(kinematics_loader_->getURDF(), kinematics_loader_->getSRDF() ? kinematics_loader_->getSRDF() :
                                                    boost::shared_ptr<srdf::Model>(new srdf::Model()), kinematics_loader_->getModel()))
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
  
  publish_planning_scene_frequency_ = 2.0;
  new_scene_update_ = UPDATE_NONE;
  
  last_update_time_ = ros::Time::now();
  last_state_update_ = ros::WallTime::now();
  dt_state_update_ = 0.1;

  reconfigure_impl_ = new DynamicReconfigureImpl(this); 
}

void planning_scene_monitor::PlanningSceneMonitor::monitorDiffs(bool flag)
{
  if (scene_)
  {
    if (flag)
    {
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      if (scene_)
      {
        scene_->decoupleParent();
        parent_scene_ = scene_;
        scene_ = parent_scene_->diff();
        scene_const_ = scene_;
      }
    }
    else
    { 
      if (publish_planning_scene_)
      {
        ROS_WARN("Diff monitoring was stopped while publishing planning scene diffs. Stopping planning scene diff publisher");
        stopPublishingPlanningScene();
      }
      {
        boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
        if (scene_)
        {
          scene_->decoupleParent();
          parent_scene_.reset();
          // remove the '+' added by .diff() at the end of the scene name
          if (!scene_->getName().empty())
          {
            if (scene_->getName()[scene_->getName().length() - 1] == '+')
              scene_->setName(scene_->getName().substr(0, scene_->getName().length() - 1));
          }
        }
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

void planning_scene_monitor::PlanningSceneMonitor::startPublishingPlanningScene(SceneUpdateType update_type, const std::string &planning_scene_topic)
{
  publish_update_types_ = update_type;
  if (!publish_planning_scene_ && scene_)
  {
    planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>(planning_scene_topic, 100, false);
    ROS_INFO("Publishing maintained planning scene on '%s'", planning_scene_topic.c_str());
    monitorDiffs(true);
    publish_planning_scene_.reset(new boost::thread(boost::bind(&PlanningSceneMonitor::scenePublishingThread, this)));
  }
}

void planning_scene_monitor::PlanningSceneMonitor::scenePublishingThread(void)
{
  ROS_DEBUG("Started scene publishing thread ...");

  // publish the full planning scene 
  moveit_msgs::PlanningScene msg;
  scene_->getPlanningSceneMsg(msg);
  planning_scene_publisher_.publish(msg);
  ROS_DEBUG("Published the full planning scene: '%s'", msg.name.c_str());
  
  bool have_diff = false;
  bool have_full = false;
  do 
  {
    have_diff = false;
    have_full = false;
    ros::Rate rate(publish_planning_scene_frequency_);
    {
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      while (new_scene_update_ == UPDATE_NONE && publish_planning_scene_)
        new_scene_update_condition_.wait(ulock);
      if (new_scene_update_ != UPDATE_NONE)
      {
        if (new_scene_update_ == UPDATE_SCENE)
        {
          rate.reset();
          scene_->pushDiffs(parent_scene_);
          scene_->clearDiffs();
          scene_->getPlanningSceneMsg(msg);
          have_full = true;
        }
        else
          if (publish_update_types_ & new_scene_update_)
          {
            rate.reset();
            scene_->getPlanningSceneDiffMsg(msg);
            scene_->pushDiffs(parent_scene_);
            scene_->clearDiffs();
            have_diff = true;
          }
        new_scene_update_ = UPDATE_NONE;
      }
    }
    if (have_diff)
    {
      planning_scene_publisher_.publish(msg);
      //ROS_DEBUG("Published planning scene diff: '%s'", msg.name.c_str());
      rate.sleep(); 
    } 
    else
      if (have_full)
      {
        planning_scene_publisher_.publish(msg);
        ROS_DEBUG("Published complete planning scene: '%s'", msg.name.c_str());
        rate.sleep(); 
      }
  }
  while (publish_planning_scene_);
}

const kinematic_model::KinematicModelConstPtr& planning_scene_monitor::PlanningSceneMonitor::getKinematicModel(void) const
{
  if (scene_)
    return scene_->getKinematicModel();
  static const kinematic_model::KinematicModelConstPtr empty;
  return empty;
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
  for (std::size_t i = 0 ; i < update_callbacks_.size() ; ++i)
    update_callbacks_[i](update_type);
  new_scene_update_ = (SceneUpdateType) ((int)new_scene_update_ | (int)update_type);
  new_scene_update_condition_.notify_all();
}

void planning_scene_monitor::PlanningSceneMonitor::newPlanningSceneCallback(const moveit_msgs::PlanningSceneConstPtr &scene)
{
  if (scene_)
  {
    SceneUpdateType upd = UPDATE_SCENE;
    std::string old_scene_name;
    {
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      last_update_time_ = ros::Time::now();
      old_scene_name = scene_->getName();
      scene_->usePlanningSceneMsg(*scene);

      // if we just reset the scene completely but we were maintaining diffs, we need to fix that
      if (!scene->is_diff && parent_scene_)
      {
        // the scene is now decoupled from the parent, since we just reset it
        parent_scene_ = scene_;
        scene_ = parent_scene_->diff();
        scene_const_ = scene_;
      }
    }
    // if we have a diff, try to more accuratelly determine the update type
    if (scene->is_diff)
    {
      bool no_other_scene_upd = (scene->name.empty() || scene->name == old_scene_name) &&
        scene->allowed_collision_matrix.entry_names.empty() && scene->link_padding.empty() && scene->link_scale.empty();
      if (no_other_scene_upd)
      {
        upd = UPDATE_NONE;
        if (!planning_scene::PlanningScene::isEmpty(scene->world))
          upd = (SceneUpdateType) ((int)upd | (int)UPDATE_GEOMETRY);
        
        if (!scene->fixed_frame_transforms.empty())
          upd = (SceneUpdateType) ((int)upd | (int)UPDATE_TRANSFORMS);
        
        if (!planning_scene::PlanningScene::isEmpty(scene->robot_state))
          upd = (SceneUpdateType) ((int)upd | (int)UPDATE_STATE);
      }
    }
    processSceneUpdateEvent(upd);
  }
}

void planning_scene_monitor::PlanningSceneMonitor::newPlanningSceneWorldCallback(const moveit_msgs::PlanningSceneWorldConstPtr &world)
{
  if (scene_)
  {
    updateFrameTransforms();
    {
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      last_update_time_ = ros::Time::now();  
      scene_->getCollisionWorld()->clearObjects();
      scene_->processPlanningSceneWorldMsg(*world);
    }  
    processSceneUpdateEvent(UPDATE_SCENE);
  }
}

void planning_scene_monitor::PlanningSceneMonitor::collisionObjectCallback(const moveit_msgs::CollisionObjectConstPtr &obj)
{
  if (scene_)
  {
    {
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
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
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
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
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      last_update_time_ = ros::Time::now();
      scene_->processCollisionMapMsg(*map);
    }
    processSceneUpdateEvent(UPDATE_GEOMETRY);
  }
}

void planning_scene_monitor::PlanningSceneMonitor::lockSceneRead(void)
{
  scene_update_mutex_.lock_shared();
  if (octomap_monitor_)
    octomap_monitor_->lockOcTreeRead();
}

void planning_scene_monitor::PlanningSceneMonitor::unlockSceneRead(void)
{
  scene_update_mutex_.unlock_shared();  
  if (octomap_monitor_)
    octomap_monitor_->unlockOcTreeRead();
}

void planning_scene_monitor::PlanningSceneMonitor::lockSceneWrite(void)
{
  scene_update_mutex_.lock();
  if (octomap_monitor_)
    octomap_monitor_->lockOcTreeWrite();
}

void planning_scene_monitor::PlanningSceneMonitor::unlockSceneWrite(void)
{
  scene_update_mutex_.unlock();
  if (octomap_monitor_)
    octomap_monitor_->unlockOcTreeWrite();
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
  
  if (!octomap_monitor_)
  {
    occupancy_map_monitor::OccupancyMapMonitor::Options opt;
    opt.map_frame = scene_->getPlanningFrame();
    octomap_monitor_.reset(new occupancy_map_monitor::OccupancyMapMonitor(opt, tf_));
    octomap_monitor_->setUpdateCallback(boost::bind(&PlanningSceneMonitor::octomapUpdateCallback, this));
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
  if (octomap_monitor_)
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
    boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
    last_update_time_ = ros::Time::now();
    octomap_monitor_->lockOcTreeWrite();
    try
    {
      scene_->processOctomapPtr(octomap_monitor_->getOcTreePtr(), Eigen::Affine3d::Identity());
      octomap_monitor_->unlockOcTreeWrite();
    }
    catch(...)
    {
      octomap_monitor_->unlockOcTreeWrite(); // unlock and rethrow
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
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      const std::map<std::string, double> &v = current_state_monitor_->getCurrentStateValues();
      scene_->getCurrentState().setStateValues(v);
      last_update_time_ = ros::Time::now();
    }
    processSceneUpdateEvent(UPDATE_STATE);
  }
  else
    ROS_ERROR("State monitor is not active. Unable to set the planning scene state");
}

void planning_scene_monitor::PlanningSceneMonitor::addUpdateCallback(const boost::function<void(SceneUpdateType)> &fn)
{
  if (fn)
    update_callbacks_.push_back(fn);
}

void planning_scene_monitor::PlanningSceneMonitor::clearUpdateCallbacks(void)
{
  update_callbacks_.clear();
}

void planning_scene_monitor::PlanningSceneMonitor::setPlanningScenePublishingFrequency(double hz)
{
  publish_planning_scene_frequency_ = hz;
  ROS_DEBUG("Maximum frquency for publishing a planning scene is now %lf Hz", publish_planning_scene_frequency_);
}

void planning_scene_monitor::PlanningSceneMonitor::getUpdatedFrameTransforms(const kinematic_model::KinematicModelConstPtr &kmodel, std::vector<geometry_msgs::TransformStamped> &transforms)
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
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
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
  const std::vector<srdf::Model::DisabledCollision> &dc = scene->getKinematicModel()->getSRDF()->getDisabledCollisionPairs();
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
