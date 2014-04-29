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

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/exceptions/exceptions.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <dynamic_reconfigure/server.h>
#include <moveit_ros_planning/PlanningSceneMonitorDynamicReconfigureConfig.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit/profiler/profiler.h>

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
  dynamic_reconfigure::Server<PlanningSceneMonitorDynamicReconfigureConfig> dynamic_reconfigure_server_;
};

}

const std::string planning_scene_monitor::PlanningSceneMonitor::DEFAULT_JOINT_STATES_TOPIC = "joint_states";
const std::string planning_scene_monitor::PlanningSceneMonitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC = "attached_collision_object";
const std::string planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC = "collision_object";
const std::string planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC = "planning_scene_world";
const std::string planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC = "planning_scene";
const std::string planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE = "/get_planning_scene";
const std::string planning_scene_monitor::PlanningSceneMonitor::MONITORED_PLANNING_SCENE_TOPIC = "monitored_planning_scene";

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const std::string &robot_description, const boost::shared_ptr<tf::Transformer> &tf, const std::string &name) :
  monitor_name_(name), nh_("~"), tf_(tf)
{
  rm_loader_.reset(new robot_model_loader::RobotModelLoader(robot_description));
  initialize(planning_scene::PlanningScenePtr());
}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const planning_scene::PlanningScenePtr &scene, const std::string &robot_description,
                                                                   const boost::shared_ptr<tf::Transformer> &tf, const std::string &name) :
   monitor_name_(name), nh_("~"), tf_(tf)
{
  rm_loader_.reset(new robot_model_loader::RobotModelLoader(robot_description));
  initialize(scene);
}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const robot_model_loader::RobotModelLoaderPtr &rm_loader,
                                                                   const boost::shared_ptr<tf::Transformer> &tf, const std::string &name) :
  monitor_name_(name), nh_("~"), tf_(tf), rm_loader_(rm_loader)
{
  initialize(planning_scene::PlanningScenePtr());
}

planning_scene_monitor::PlanningSceneMonitor::PlanningSceneMonitor(const planning_scene::PlanningScenePtr &scene, const robot_model_loader::RobotModelLoaderPtr &rm_loader,
                                                                   const boost::shared_ptr<tf::Transformer> &tf, const std::string &name) :
  monitor_name_(name), nh_("~"), tf_(tf), rm_loader_(rm_loader)
{
  initialize(scene);
}

planning_scene_monitor::PlanningSceneMonitor::~PlanningSceneMonitor()
{
  if (scene_)
  {
    scene_->setCollisionObjectUpdateCallback(collision_detection::World::ObserverCallbackFn());
    scene_->setAttachedBodyUpdateCallback(robot_state::AttachedBodyCallback());
  }
  stopPublishingPlanningScene();
  stopStateMonitor();
  stopWorldGeometryMonitor();
  stopSceneMonitor();
  delete reconfigure_impl_;
  current_state_monitor_.reset();
  scene_const_.reset();
  scene_.reset();
  parent_scene_.reset();
  robot_model_.reset();
  rm_loader_.reset();
}

void planning_scene_monitor::PlanningSceneMonitor::initialize(const planning_scene::PlanningScenePtr &scene)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("PlanningSceneMonitor::initialize");

  if (monitor_name_.empty())
    monitor_name_ = "planning_scene_monitor";
  robot_description_ = rm_loader_->getRobotDescription();
  if (rm_loader_->getModel())
  {
    robot_model_ = rm_loader_->getModel();
    scene_ = scene;
    scene_const_ = scene_;
    if (!scene_)
    {
      try
      {
        scene_.reset(new planning_scene::PlanningScene(rm_loader_->getModel()));
        scene_const_ = scene_;
        configureCollisionMatrix(scene_);
        configureDefaultPadding();

        scene_->getCollisionRobotNonConst()->setPadding(default_robot_padd_);
        scene_->getCollisionRobotNonConst()->setScale(default_robot_scale_);
        for(std::map<std::string, double>::iterator it=default_robot_link_padd_.begin(); it != default_robot_link_padd_.end(); ++it) {
            scene_->getCollisionRobotNonConst()->setLinkPadding(it->first, it->second);
        }
        for(std::map<std::string, double>::iterator it=default_robot_link_scale_.begin(); it != default_robot_link_scale_.end(); ++it) {
            scene_->getCollisionRobotNonConst()->setLinkScale(it->first, it->second);
        }
        scene_->propogateRobotPadding();
      }
      catch (moveit::ConstructException &e)
      {
        ROS_ERROR("Configuration of planning scene failed");
        scene_.reset();
        scene_const_ = scene_;
      }
    }
    if (scene_)
    {
      scene_->setAttachedBodyUpdateCallback(boost::bind(&PlanningSceneMonitor::currentStateAttachedBodyUpdateCallback, this, _1, _2));
      scene_->setCollisionObjectUpdateCallback(boost::bind(&PlanningSceneMonitor::currentWorldObjectUpdateCallback, this, _1, _2));
    }
  }
  else
  {
    ROS_ERROR("Robot model not loaded");
  }

  publish_planning_scene_frequency_ = 2.0;
  new_scene_update_ = UPDATE_NONE;

  last_update_time_ = ros::Time::now();
  last_state_update_ = ros::WallTime::now();
  dt_state_update_ = ros::WallDuration(0.1);
  state_update_pending_ = false;
  state_update_timer_ = nh_.createWallTimer(dt_state_update_,
                                            &PlanningSceneMonitor::stateUpdateTimerCallback,
                                            this,
                                            false,    // not a oneshot timer
                                            false);   // do not start the timer yet

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
        scene_->setAttachedBodyUpdateCallback(robot_state::AttachedBodyCallback());
        scene_->setCollisionObjectUpdateCallback(collision_detection::World::ObserverCallbackFn());
        scene_->decoupleParent();
        parent_scene_ = scene_;
        scene_ = parent_scene_->diff();
        scene_const_ = scene_;
        scene_->setAttachedBodyUpdateCallback(boost::bind(&PlanningSceneMonitor::currentStateAttachedBodyUpdateCallback, this, _1, _2));
        scene_->setCollisionObjectUpdateCallback(boost::bind(&PlanningSceneMonitor::currentWorldObjectUpdateCallback, this, _1, _2));
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

void planning_scene_monitor::PlanningSceneMonitor::stopPublishingPlanningScene()
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

void planning_scene_monitor::PlanningSceneMonitor::scenePublishingThread()
{
  ROS_DEBUG("Started scene publishing thread ...");

  // publish the full planning scene
  moveit_msgs::PlanningScene msg;
  scene_->getPlanningSceneMsg(msg);
  planning_scene_publisher_.publish(msg);
  ROS_DEBUG("Published the full planning scene: '%s'", msg.name.c_str());

  do
  {
    bool publish_msg = false;
    bool is_full = false;
    ros::Rate rate(publish_planning_scene_frequency_);
    {
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      while (new_scene_update_ == UPDATE_NONE && publish_planning_scene_)
        new_scene_update_condition_.wait(ulock);
      if (new_scene_update_ != UPDATE_NONE)
      {
        if ((publish_update_types_ & new_scene_update_) ||
            new_scene_update_ == UPDATE_SCENE)
        {
          if (new_scene_update_ == UPDATE_SCENE)
            is_full = true;
          else
            scene_->getPlanningSceneDiffMsg(msg);
          boost::recursive_mutex::scoped_lock prevent_shape_cache_updates(shape_handles_lock_); // we don't want the transform cache to update while we are potentially changing attached bodies
          scene_->setAttachedBodyUpdateCallback(robot_state::AttachedBodyCallback());
          scene_->setCollisionObjectUpdateCallback(collision_detection::World::ObserverCallbackFn());
          scene_->pushDiffs(parent_scene_);
          scene_->clearDiffs();
          scene_->setAttachedBodyUpdateCallback(boost::bind(&PlanningSceneMonitor::currentStateAttachedBodyUpdateCallback, this, _1, _2));
          scene_->setCollisionObjectUpdateCallback(boost::bind(&PlanningSceneMonitor::currentWorldObjectUpdateCallback, this, _1, _2));
          if (octomap_monitor_)
          {
            excludeAttachedBodiesFromOctree(); // in case updates have happened to the attached bodies, put them in
            excludeWorldObjectsFromOctree(); // in case updates have happened to the attached bodies, put them in
          }
          if (is_full)
            scene_->getPlanningSceneMsg(msg);
          publish_msg = true;
        }
        new_scene_update_ = UPDATE_NONE;
      }
    }
    if (publish_msg)
    {
      rate.reset();
      planning_scene_publisher_.publish(msg);
      if (is_full)
        ROS_DEBUG("Published full planning scene: '%s'", msg.name.c_str());
      rate.sleep();
    }
  }
  while (publish_planning_scene_);
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
  if (planning_scene_world_subscriber_)
    topics.push_back(planning_scene_world_subscriber_.getTopic());
}

namespace
{
bool sceneIsParentOf(const planning_scene::PlanningSceneConstPtr &scene, const planning_scene::PlanningScene *possible_parent)
{
  if (scene && scene.get() == possible_parent)
    return true;
  if (scene)
    return sceneIsParentOf(scene->getParent(), possible_parent);
  return false;
}
}

bool planning_scene_monitor::PlanningSceneMonitor::updatesScene(const planning_scene::PlanningScenePtr &scene) const
{
  return sceneIsParentOf(scene_const_, scene.get());
}

bool planning_scene_monitor::PlanningSceneMonitor::updatesScene(const planning_scene::PlanningSceneConstPtr &scene) const
{
  return sceneIsParentOf(scene_const_, scene.get());
}

void planning_scene_monitor::PlanningSceneMonitor::triggerSceneUpdateEvent(SceneUpdateType update_type)
{
  // do not modify update functions while we are calling them
  boost::recursive_mutex::scoped_lock lock(update_lock_);

  for (std::size_t i = 0 ; i < update_callbacks_.size() ; ++i)
    update_callbacks_[i](update_type);
  new_scene_update_ = (SceneUpdateType) ((int)new_scene_update_ | (int)update_type);
  new_scene_update_condition_.notify_all();
}

bool planning_scene_monitor::PlanningSceneMonitor::requestPlanningSceneState(const std::string& service_name)
{
  ros::ServiceClient client = nh_.serviceClient<moveit_msgs::GetPlanningScene>(service_name);
  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components = 
      srv.request.components.SCENE_SETTINGS |
      srv.request.components.ROBOT_STATE |
      srv.request.components.ROBOT_STATE_ATTACHED_OBJECTS |
      srv.request.components.WORLD_OBJECT_NAMES |
      srv.request.components.WORLD_OBJECT_GEOMETRY |
      srv.request.components.OCTOMAP |
      srv.request.components.TRANSFORMS |
      srv.request.components.ALLOWED_COLLISION_MATRIX |
      srv.request.components.LINK_PADDING_AND_SCALING |
      srv.request.components.OBJECT_COLORS;

  // Make sure client is connected to server
  if (!client.exists())
  {
    ROS_DEBUG_STREAM("Waiting for service `" << service_name << "` to exist.");
    client.waitForExistence(ros::Duration(5.0));
  }

  if (client.call(srv))
  {
    newPlanningSceneMessage(srv.response.scene);
  }
  else
  {
    ROS_WARN("Failed to call service %s, have you launched move_group? at %s:%d",
      service_name.c_str(),
      __FILE__,
      __LINE__);
    return false;
  }
  return true;
}

void planning_scene_monitor::PlanningSceneMonitor::newPlanningSceneCallback(const moveit_msgs::PlanningSceneConstPtr &scene)
{
  newPlanningSceneMessage(*scene);
}

void planning_scene_monitor::PlanningSceneMonitor::newPlanningSceneMessage(const moveit_msgs::PlanningScene& scene)
{
  if (scene_)
  {
    SceneUpdateType upd = UPDATE_SCENE;
    std::string old_scene_name;
    {
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      boost::recursive_mutex::scoped_lock prevent_shape_cache_updates(shape_handles_lock_); // we don't want the transform cache to update while we are potentially changing attached bodies

      last_update_time_ = ros::Time::now();
      old_scene_name = scene_->getName();
      scene_->usePlanningSceneMsg(scene);
      if (octomap_monitor_)
      {
        if (!scene.is_diff && scene.world.octomap.octomap.data.empty())
        {
          octomap_monitor_->getOcTreePtr()->lockWrite();
          octomap_monitor_->getOcTreePtr()->clear();
          octomap_monitor_->getOcTreePtr()->unlockWrite();
        }
      }
      robot_model_ = scene_->getRobotModel();

      // if we just reset the scene completely but we were maintaining diffs, we need to fix that
      if (!scene.is_diff && parent_scene_)
      {
        // the scene is now decoupled from the parent, since we just reset it
        scene_->setAttachedBodyUpdateCallback(robot_state::AttachedBodyCallback());
        scene_->setCollisionObjectUpdateCallback(collision_detection::World::ObserverCallbackFn());
        parent_scene_ = scene_;
        scene_ = parent_scene_->diff();
        scene_const_ = scene_;
        scene_->setAttachedBodyUpdateCallback(boost::bind(&PlanningSceneMonitor::currentStateAttachedBodyUpdateCallback, this, _1, _2));
        scene_->setCollisionObjectUpdateCallback(boost::bind(&PlanningSceneMonitor::currentWorldObjectUpdateCallback, this, _1, _2));
      }
      if (octomap_monitor_)
      {
        excludeAttachedBodiesFromOctree(); // in case updates have happened to the attached bodies, put them in
        excludeWorldObjectsFromOctree(); // in case updates have happened to the attached bodies, put them in
      }
    }

    // if we have a diff, try to more accuratelly determine the update type
    if (scene.is_diff)
    {
      bool no_other_scene_upd = (scene.name.empty() || scene.name == old_scene_name) &&
        scene.allowed_collision_matrix.entry_names.empty() && scene.link_padding.empty() && scene.link_scale.empty();
      if (no_other_scene_upd)
      {
        upd = UPDATE_NONE;
        if (!planning_scene::PlanningScene::isEmpty(scene.world))
          upd = (SceneUpdateType) ((int)upd | (int)UPDATE_GEOMETRY);

        if (!scene.fixed_frame_transforms.empty())
          upd = (SceneUpdateType) ((int)upd | (int)UPDATE_TRANSFORMS);

        if (!planning_scene::PlanningScene::isEmpty(scene.robot_state))
        {
          upd = (SceneUpdateType) ((int)upd | (int)UPDATE_STATE);
          if (!scene.robot_state.attached_collision_objects.empty() || scene.robot_state.is_diff == false)
            upd = (SceneUpdateType) ((int)upd | (int)UPDATE_GEOMETRY);
        }
      }
    }
    triggerSceneUpdateEvent(upd);
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
      scene_->getWorldNonConst()->clearObjects();
      scene_->processPlanningSceneWorldMsg(*world);
      if (octomap_monitor_)
      {
        if (world->octomap.octomap.data.empty())
        {
          octomap_monitor_->getOcTreePtr()->lockWrite();
          octomap_monitor_->getOcTreePtr()->clear();
          octomap_monitor_->getOcTreePtr()->unlockWrite();
        }
      }
    }
    triggerSceneUpdateEvent(UPDATE_SCENE);
  }
}

void planning_scene_monitor::PlanningSceneMonitor::collisionObjectFailTFCallback(const moveit_msgs::CollisionObjectConstPtr &obj, tf::filter_failure_reasons::FilterFailureReason reason)
{
  // if we just want to remove objects, the frame does not matter
  if (reason == tf::filter_failure_reasons::EmptyFrameID && obj->operation == moveit_msgs::CollisionObject::REMOVE)
    collisionObjectCallback(obj);
}

void planning_scene_monitor::PlanningSceneMonitor::collisionObjectCallback(const moveit_msgs::CollisionObjectConstPtr &obj)
{
  if (scene_)
  {
    updateFrameTransforms();
    {
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      last_update_time_ = ros::Time::now();
      scene_->processCollisionObjectMsg(*obj);
    }
    triggerSceneUpdateEvent(UPDATE_GEOMETRY);
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
    triggerSceneUpdateEvent(UPDATE_GEOMETRY);
  }
}

void planning_scene_monitor::PlanningSceneMonitor::excludeRobotLinksFromOctree()
{
  boost::recursive_mutex::scoped_lock _(shape_handles_lock_);

  includeRobotLinksInOctree();
  const std::vector<const robot_model::LinkModel*> &links = getRobotModel()->getLinkModelsWithCollisionGeometry();
  for (std::size_t i = 0 ; i < links.size() ; ++i)
  {
    std::vector<shapes::ShapeConstPtr> shapes = links[i]->getShapes(); // copy shared ptrs on purpuse
    for (std::size_t j = 0 ; j < shapes.size() ; ++j)
    {
      // merge mesh vertices up to 0.1 mm apart
      if (shapes[j]->type == shapes::MESH)
      {
        shapes::Mesh *m = static_cast<shapes::Mesh*>(shapes[j]->clone());
        m->mergeVertices(1e-4);
        shapes[j].reset(m);
      }
      
      occupancy_map_monitor::ShapeHandle h = octomap_monitor_->excludeShape(shapes[j]);
      if (h)
        link_shape_handles_[links[i]].push_back(std::make_pair(h, j));
    }
  }
}

void planning_scene_monitor::PlanningSceneMonitor::includeRobotLinksInOctree()
{
  if (!octomap_monitor_)
    return;

  boost::recursive_mutex::scoped_lock _(shape_handles_lock_);

  for (LinkShapeHandles::iterator it = link_shape_handles_.begin() ; it != link_shape_handles_.end() ; ++it)
    for (std::size_t i = 0 ; i < it->second.size() ; ++i)
      octomap_monitor_->forgetShape(it->second[i].first);
  link_shape_handles_.clear();
}

void planning_scene_monitor::PlanningSceneMonitor::includeAttachedBodiesInOctree()
{
  boost::recursive_mutex::scoped_lock _(shape_handles_lock_);

  // clear information about any attached body, without refering to the AttachedBody* ptr (could be invalid)
  for (AttachedBodyShapeHandles::iterator it = attached_body_shape_handles_.begin() ; it != attached_body_shape_handles_.end() ; ++it)
    for (std::size_t k = 0 ; k < it->second.size() ; ++k)
      octomap_monitor_->forgetShape(it->second[k].first);
  attached_body_shape_handles_.clear();
}

void planning_scene_monitor::PlanningSceneMonitor::excludeAttachedBodiesFromOctree()
{
  boost::recursive_mutex::scoped_lock _(shape_handles_lock_);

  includeAttachedBodiesInOctree();
  // add attached objects again
  std::vector<const robot_state::AttachedBody*> ab;
  scene_->getCurrentState().getAttachedBodies(ab);
  for (std::size_t i = 0 ; i < ab.size() ; ++i)
    excludeAttachedBodyFromOctree(ab[i]);
}

void planning_scene_monitor::PlanningSceneMonitor::includeWorldObjectsInOctree()
{
  boost::recursive_mutex::scoped_lock _(shape_handles_lock_);

  // clear information about any attached object
  for (CollisionBodyShapeHandles::iterator it = collision_body_shape_handles_.begin() ; it != collision_body_shape_handles_.end() ; ++it)
    for (std::size_t k = 0 ; k < it->second.size() ; ++k)
      octomap_monitor_->forgetShape(it->second[k].first);
  collision_body_shape_handles_.clear();
}

void planning_scene_monitor::PlanningSceneMonitor::excludeWorldObjectsFromOctree()
{
  boost::recursive_mutex::scoped_lock _(shape_handles_lock_);

  includeWorldObjectsInOctree();
  for (collision_detection::World::const_iterator it = scene_->getWorld()->begin(); it != scene_->getWorld()->end() ; ++it)
    excludeWorldObjectFromOctree(it->second);
}

void planning_scene_monitor::PlanningSceneMonitor::excludeAttachedBodyFromOctree(const robot_state::AttachedBody *attached_body)
{
  boost::recursive_mutex::scoped_lock _(shape_handles_lock_);
  bool found = false;
  for (std::size_t i = 0 ; i < attached_body->getShapes().size() ; ++i)
  {
    if (attached_body->getShapes()[i]->type == shapes::PLANE || attached_body->getShapes()[i]->type == shapes::OCTREE)
      continue;
    occupancy_map_monitor::ShapeHandle h = octomap_monitor_->excludeShape(attached_body->getShapes()[i]);
    if (h)
    {
      found = true;
      attached_body_shape_handles_[attached_body].push_back(std::make_pair(h, i));
    }
  }
  if (found)
    ROS_DEBUG("Excluding attached body '%s' from monitored octomap", attached_body->getName().c_str());
}

void planning_scene_monitor::PlanningSceneMonitor::includeAttachedBodyInOctree(const robot_state::AttachedBody *attached_body)
{
  boost::recursive_mutex::scoped_lock _(shape_handles_lock_);

  AttachedBodyShapeHandles::iterator it = attached_body_shape_handles_.find(attached_body);
  if (it != attached_body_shape_handles_.end())
  {
    for (std::size_t k = 0 ; k < it->second.size() ; ++k)
      octomap_monitor_->forgetShape(it->second[k].first);
    ROS_DEBUG("Including attached body '%s' in monitored octomap", attached_body->getName().c_str());
    attached_body_shape_handles_.erase(it);
  }
}

void planning_scene_monitor::PlanningSceneMonitor::excludeWorldObjectFromOctree(const collision_detection::World::ObjectConstPtr &obj)
{
  boost::recursive_mutex::scoped_lock _(shape_handles_lock_);

  bool found = false;
  for (std::size_t i = 0 ; i < obj->shapes_.size() ; ++i)
  {
    if (obj->shapes_[i]->type == shapes::PLANE || obj->shapes_[i]->type == shapes::OCTREE)
      continue;
    occupancy_map_monitor::ShapeHandle h = octomap_monitor_->excludeShape(obj->shapes_[i]);
    if (h)
    {
      collision_body_shape_handles_[obj->id_].push_back(std::make_pair(h, &obj->shape_poses_[i]));
      found = true;
    }
  }
  if (found)
    ROS_DEBUG("Excluding collision object '%s' from monitored octomap", obj->id_.c_str());
}

void planning_scene_monitor::PlanningSceneMonitor::includeWorldObjectInOctree(const collision_detection::World::ObjectConstPtr &obj)
{
  boost::recursive_mutex::scoped_lock _(shape_handles_lock_);

  CollisionBodyShapeHandles::iterator it = collision_body_shape_handles_.find(obj->id_);
  if (it != collision_body_shape_handles_.end())
  {
    for (std::size_t k = 0 ; k < it->second.size() ; ++k)
      octomap_monitor_->forgetShape(it->second[k].first);
    ROS_DEBUG("Including collision object '%s' in monitored octomap", obj->id_.c_str());
    collision_body_shape_handles_.erase(it);
  }
}

void planning_scene_monitor::PlanningSceneMonitor::currentStateAttachedBodyUpdateCallback(robot_state::AttachedBody *attached_body, bool just_attached)
{
  if (!octomap_monitor_)
    return;

  if (just_attached)
    excludeAttachedBodyFromOctree(attached_body);
  else
    includeAttachedBodyInOctree(attached_body);
}

void planning_scene_monitor::PlanningSceneMonitor::currentWorldObjectUpdateCallback(const collision_detection::World::ObjectConstPtr &obj, collision_detection::World::Action action)
{
  if (!octomap_monitor_)
    return;
  if (obj->id_ == planning_scene::PlanningScene::OCTOMAP_NS)
    return;

  if (action & collision_detection::World::CREATE)
    excludeWorldObjectFromOctree(obj);
  else
    if (action & collision_detection::World::DESTROY)
      includeWorldObjectInOctree(obj);
    else
    {
      excludeWorldObjectFromOctree(obj);
      includeWorldObjectInOctree(obj);
    }
}

void planning_scene_monitor::PlanningSceneMonitor::lockSceneRead()
{
  scene_update_mutex_.lock_shared();
  if (octomap_monitor_)
    octomap_monitor_->getOcTreePtr()->lockRead();
}

void planning_scene_monitor::PlanningSceneMonitor::unlockSceneRead()
{
  scene_update_mutex_.unlock_shared();
  if (octomap_monitor_)
    octomap_monitor_->getOcTreePtr()->unlockRead();
}

void planning_scene_monitor::PlanningSceneMonitor::lockSceneWrite()
{
  scene_update_mutex_.lock();
  if (octomap_monitor_)
    octomap_monitor_->getOcTreePtr()->lockWrite();
}

void planning_scene_monitor::PlanningSceneMonitor::unlockSceneWrite()
{
  scene_update_mutex_.unlock();
  if (octomap_monitor_)
    octomap_monitor_->getOcTreePtr()->unlockWrite();
}

void planning_scene_monitor::PlanningSceneMonitor::startSceneMonitor(const std::string &scene_topic)
{
  stopSceneMonitor();

  ROS_INFO("Starting scene monitor");
  // listen for planning scene updates; these messages include transforms, so no need for filters
  if (!scene_topic.empty())
  {
    planning_scene_subscriber_ = root_nh_.subscribe(scene_topic, 100, &PlanningSceneMonitor::newPlanningSceneCallback, this);
    ROS_INFO("Listening to '%s'", root_nh_.resolveName(scene_topic).c_str());
  }
}

void planning_scene_monitor::PlanningSceneMonitor::stopSceneMonitor()
{
  if (planning_scene_subscriber_)
  {
    ROS_INFO("Stopping scene monitor");
    planning_scene_subscriber_.shutdown();
  }
}

bool planning_scene_monitor::PlanningSceneMonitor::getShapeTransformCache(const std::string &target_frame, const ros::Time &target_time,
                                                                          occupancy_map_monitor::ShapeTransformCache &cache) const
{
  if (!tf_)
    return false;
  try
  {
    boost::recursive_mutex::scoped_lock _(shape_handles_lock_);

    for (LinkShapeHandles::const_iterator it = link_shape_handles_.begin() ; it != link_shape_handles_.end() ; ++it)
    {
      tf::StampedTransform tr;
      tf_->lookupTransform(target_frame, it->first->getName(), target_time, tr);
      Eigen::Affine3d ttr;
      tf::transformTFToEigen(tr, ttr);
      for (std::size_t j = 0 ; j < it->second.size() ; ++j)
        cache[it->second[j].first] = ttr * it->first->getCollisionOriginTransforms()[it->second[j].second];
    }
    for (AttachedBodyShapeHandles::const_iterator it = attached_body_shape_handles_.begin() ; it != attached_body_shape_handles_.end() ; ++it)
    {
      tf::StampedTransform tr;
      tf_->lookupTransform(target_frame, it->first->getAttachedLinkName(), target_time, tr);
      Eigen::Affine3d transform;
      tf::transformTFToEigen(tr, transform);
      for (std::size_t k = 0 ; k < it->second.size() ; ++k)
        cache[it->second[k].first] = transform * it->first->getFixedTransforms()[it->second[k].second];
    }
    {
      tf::StampedTransform tr;
      tf_->lookupTransform(target_frame, scene_->getPlanningFrame(), target_time, tr);
      Eigen::Affine3d transform;
      tf::transformTFToEigen(tr, transform);
      for (CollisionBodyShapeHandles::const_iterator it = collision_body_shape_handles_.begin() ; it != collision_body_shape_handles_.end() ; ++it)
        for (std::size_t k = 0 ; k < it->second.size() ; ++k)
          cache[it->second[k].first] = transform * (*it->second[k].second);
    }
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_THROTTLE(1, "Transform error: %s", ex.what());
    return false;
  }
  return true;
}

void planning_scene_monitor::PlanningSceneMonitor::startWorldGeometryMonitor(const std::string &collision_objects_topic,
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
      collision_object_filter_->registerFailureCallback(boost::bind(&PlanningSceneMonitor::collisionObjectFailTFCallback, this, _1, _2));
      ROS_INFO("Listening to '%s' using message notifier with target frame '%s'", root_nh_.resolveName(collision_objects_topic).c_str(), collision_object_filter_->getTargetFramesString().c_str());
    }
    else
    {
      collision_object_subscriber_->registerCallback(boost::bind(&PlanningSceneMonitor::collisionObjectCallback, this, _1));
      ROS_INFO("Listening to '%s'", root_nh_.resolveName(collision_objects_topic).c_str());
    }
  }

  if (!planning_scene_world_topic.empty())
  {
    planning_scene_world_subscriber_ = root_nh_.subscribe(planning_scene_world_topic, 1, &PlanningSceneMonitor::newPlanningSceneWorldCallback, this);
    ROS_INFO("Listening to '%s' for planning scene world geometry", root_nh_.resolveName(planning_scene_world_topic).c_str());
  }
  if (!octomap_monitor_)
  {
    octomap_monitor_.reset(new occupancy_map_monitor::OccupancyMapMonitor(tf_, scene_->getPlanningFrame()));
    excludeRobotLinksFromOctree();
    excludeAttachedBodiesFromOctree();
    excludeWorldObjectsFromOctree();

    octomap_monitor_->setTransformCacheCallback(boost::bind(&PlanningSceneMonitor::getShapeTransformCache, this, _1, _2, _3));
    octomap_monitor_->setUpdateCallback(boost::bind(&PlanningSceneMonitor::octomapUpdateCallback, this));
  }
  octomap_monitor_->startMonitor();
}

void planning_scene_monitor::PlanningSceneMonitor::stopWorldGeometryMonitor()
{
  if (collision_object_subscriber_ || collision_object_filter_)
  {
    ROS_INFO("Stopping world geometry monitor");
    collision_object_filter_.reset();
    collision_object_subscriber_.reset();
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
      current_state_monitor_.reset(new CurrentStateMonitor(getRobotModel(), tf_));
    current_state_monitor_->addUpdateCallback(boost::bind(&PlanningSceneMonitor::onStateUpdate, this, _1));
    current_state_monitor_->startStateMonitor(joint_states_topic);

    {
      boost::mutex::scoped_lock lock(state_pending_mutex_);
      if (!dt_state_update_.isZero())
        state_update_timer_.start();
    }

    if (!attached_objects_topic.empty())
    {
      // using regular message filter as there's no header
      attached_collision_object_subscriber_ = root_nh_.subscribe(attached_objects_topic, 1024, &PlanningSceneMonitor::attachObjectCallback, this);
      ROS_INFO("Listening to '%s' for attached collision objects", root_nh_.resolveName(attached_objects_topic).c_str());
    }
  }
  else
    ROS_ERROR("Cannot monitor robot state because planning scene is not configured");
}

void planning_scene_monitor::PlanningSceneMonitor::stopStateMonitor()
{
  if (current_state_monitor_)
    current_state_monitor_->stopStateMonitor();
  if (attached_collision_object_subscriber_)
    attached_collision_object_subscriber_.shutdown();

  // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
  state_update_timer_.stop();
  {
    boost::mutex::scoped_lock lock(state_pending_mutex_);
    state_update_pending_ = false;
  }
}

void planning_scene_monitor::PlanningSceneMonitor::onStateUpdate(const sensor_msgs::JointStateConstPtr & /* joint_state */ )
{
  const ros::WallTime &n = ros::WallTime::now();
  ros::WallDuration dt = n - last_state_update_;

  bool update = false;
  {
    boost::mutex::scoped_lock lock(state_pending_mutex_);

    if (dt < dt_state_update_)
    {
      state_update_pending_ = true;
    }
    else
    {
      state_update_pending_ = false;
      last_state_update_ = n;
      update = true;
    }
  }

  // run the state update with state_pending_mutex_ unlocked
  if (update)
    updateSceneWithCurrentState();
}

void planning_scene_monitor::PlanningSceneMonitor::stateUpdateTimerCallback(const ros::WallTimerEvent& event)
{
  if (state_update_pending_)
  {
    bool update = false;

    const ros::WallTime &n = ros::WallTime::now();
    ros::WallDuration dt = n - last_state_update_;

    {
      // lock for access to dt_state_update_ and state_update_pending_
      boost::mutex::scoped_lock lock(state_pending_mutex_);
      if (state_update_pending_ && dt >= dt_state_update_)
      {
        state_update_pending_ = false;
        last_state_update_ = ros::WallTime::now();
        update = true;
      }
    }

    // run the state update with state_pending_mutex_ unlocked
    if (update)
      updateSceneWithCurrentState();
  }
}

void planning_scene_monitor::PlanningSceneMonitor::octomapUpdateCallback()
{
  updateFrameTransforms();
  {
    boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
    last_update_time_ = ros::Time::now();
    octomap_monitor_->getOcTreePtr()->lockRead();
    try
    {
      scene_->processOctomapPtr(octomap_monitor_->getOcTreePtr(), Eigen::Affine3d::Identity());
      octomap_monitor_->getOcTreePtr()->unlockRead();
    }
    catch(...)
    {
      octomap_monitor_->getOcTreePtr()->unlockRead(); // unlock and rethrow
      throw;
    }
  }
  triggerSceneUpdateEvent(UPDATE_GEOMETRY);
}

void planning_scene_monitor::PlanningSceneMonitor::setStateUpdateFrequency(double hz)
{
  bool update = false;
  if (hz > std::numeric_limits<double>::epsilon())
  {
    boost::mutex::scoped_lock lock(state_pending_mutex_);
    dt_state_update_.fromSec(1.0 / hz);
    state_update_timer_.setPeriod(dt_state_update_);
    state_update_timer_.start();
  }
  else
  {
    // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
    state_update_timer_.stop(); 
    boost::mutex::scoped_lock lock(state_pending_mutex_);
    dt_state_update_ = ros::WallDuration(0,0);
    if (state_update_pending_)
      update = true;
  }
  ROS_INFO("Updating internal planning scene state at most every %lf seconds", dt_state_update_.toSec());

  if (update)
    updateSceneWithCurrentState();
}

void planning_scene_monitor::PlanningSceneMonitor::updateSceneWithCurrentState()
{
  if (current_state_monitor_)
  {
    std::vector<std::string> missing;
    if (!current_state_monitor_->haveCompleteState(missing) && (ros::Time::now() - current_state_monitor_->getMonitorStartTime()).toSec() > 1.0)
    {
      std::string missing_str = boost::algorithm::join(missing, ", ");
      ROS_WARN_THROTTLE(1, "The complete state of the robot is not yet known.  Missing %s", missing_str.c_str());
    }

    {
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      current_state_monitor_->setToCurrentState(scene_->getCurrentStateNonConst());
      last_update_time_ = ros::Time::now();
      scene_->getCurrentStateNonConst().update(); // compute all transforms 
    }
    triggerSceneUpdateEvent(UPDATE_STATE);
  }
  else
    ROS_ERROR_THROTTLE(1, "State monitor is not active. Unable to set the planning scene state");
}

void planning_scene_monitor::PlanningSceneMonitor::addUpdateCallback(const boost::function<void(SceneUpdateType)> &fn)
{
  boost::recursive_mutex::scoped_lock lock(update_lock_);
  if (fn)
    update_callbacks_.push_back(fn);
}

void planning_scene_monitor::PlanningSceneMonitor::clearUpdateCallbacks()
{
  boost::recursive_mutex::scoped_lock lock(update_lock_);
  update_callbacks_.clear();
}

void planning_scene_monitor::PlanningSceneMonitor::setPlanningScenePublishingFrequency(double hz)
{
  publish_planning_scene_frequency_ = hz;
  ROS_DEBUG("Maximum frquency for publishing a planning scene is now %lf Hz", publish_planning_scene_frequency_);
}

void planning_scene_monitor::PlanningSceneMonitor::getUpdatedFrameTransforms(std::vector<geometry_msgs::TransformStamped> &transforms)
{
  const std::string &target = getRobotModel()->getModelFrame();

  std::vector<std::string> all_frame_names;
  tf_->getFrameStrings(all_frame_names);
  for (std::size_t i = 0 ; i < all_frame_names.size() ; ++i)
  {
    const std::string &frame_no_slash = (!all_frame_names[i].empty() && all_frame_names[i][0] == '/') ? all_frame_names[i].substr(1) : all_frame_names[i];
    const std::string &frame_with_slash = (!all_frame_names[i].empty() && all_frame_names[i][0] != '/') ? '/' + all_frame_names[i] : all_frame_names[i];

    if (frame_with_slash == target || getRobotModel()->hasLinkModel(frame_no_slash))
      continue;

    ros::Time stamp(0);
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
    f.header.frame_id = frame_with_slash;
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

void planning_scene_monitor::PlanningSceneMonitor::updateFrameTransforms()
{
  if (!tf_)
    return;

  if (scene_)
  {
    std::vector<geometry_msgs::TransformStamped> transforms;
    getUpdatedFrameTransforms(transforms);
    {
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      scene_->getTransformsNonConst().setTransforms(transforms);
      last_update_time_ = ros::Time::now();
    }
    triggerSceneUpdateEvent(UPDATE_TRANSFORMS);
  }
}

void planning_scene_monitor::PlanningSceneMonitor::publishDebugInformation(bool flag)
{
  if (octomap_monitor_)
    octomap_monitor_->publishDebugInformation(flag);
}

void planning_scene_monitor::PlanningSceneMonitor::configureCollisionMatrix(const planning_scene::PlanningScenePtr &scene)
{
  if (!scene || robot_description_.empty())
    return;
  collision_detection::AllowedCollisionMatrix &acm = scene->getAllowedCollisionMatrixNonConst();

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

void planning_scene_monitor::PlanningSceneMonitor::configureDefaultPadding()
{
  if (robot_description_.empty())
  {
    default_robot_padd_ = 0.0;
    default_robot_scale_ = 1.0;
    default_object_padd_ = 0.0;
    default_attached_padd_ = 0.0;
    return;
  }
  nh_.param(robot_description_ + "_planning/default_robot_padding", default_robot_padd_, 0.0);
  nh_.param(robot_description_ + "_planning/default_robot_scale", default_robot_scale_, 1.0);
  nh_.param(robot_description_ + "_planning/default_object_padding", default_object_padd_, 0.0);
  nh_.param(robot_description_ + "_planning/default_attached_padding", default_attached_padd_, 0.0);
  nh_.param(robot_description_ + "_planning/default_robot_link_padding", default_robot_link_padd_, std::map<std::string, double>());
  nh_.param(robot_description_ + "_planning/default_robot_link_scale", default_robot_link_scale_, std::map<std::string, double>());
}
