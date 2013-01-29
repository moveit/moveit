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

#ifndef MOVEIT_PLANNING_SCENE_MONITOR_PLANNING_SCENE_MONITOR_
#define MOVEIT_PLANNING_SCENE_MONITOR_PLANNING_SCENE_MONITOR_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_models_loader/kinematic_model_loader.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <boost/noncopyable.hpp>
#include <boost/thread/shared_mutex.hpp>

namespace planning_scene_monitor
{

/**
 * @brief PlanningSceneMonitor
 * Subscribes to the topic \e planning_scene */
class PlanningSceneMonitor : private boost::noncopyable
{
public:
  
  enum SceneUpdateType
    {
      /** \brief No update */
      UPDATE_NONE = 0,

      /** \brief The state in the monitored scene was updated */
      UPDATE_STATE = 1,

      /** \brief The maintained set of fixed transforms in the monitored scene was updated */
      UPDATE_TRANSFORMS = 2,

      /** \brief The geometry of the scene was updated. This includes receiving new maps, collision objects, attached objects, scene geometry, etc. */
      UPDATE_GEOMETRY = 4,

      /** \brief The entire scene was updated */
      UPDATE_SCENE = 8 + UPDATE_STATE + UPDATE_TRANSFORMS + UPDATE_GEOMETRY
    };
  
    
  /** @brief Constructor
   *  @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   *  @param tf A pointer to a tf::Transformer
   *  @param name A name identifying this planning scene monitor
   */
  PlanningSceneMonitor(const std::string &robot_description,
                       const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>(),
                       const std::string &name = "");

  /** @brief Constructor
   *  @param kml A pointer to a kinematic model loader
   *  @param tf A pointer to a tf::Transformer
   *  @param name A name identifying this planning scene monitor
   */
  PlanningSceneMonitor(const planning_models_loader::KinematicModelLoaderPtr &kml,
                       const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>(),
                       const std::string &name = "");

  /** @brief Constructor
   *  @param scene The scene instance to maintain up to date with monitored information
   *  @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   *  @param tf A pointer to a tf::Transformer
   *  @param name A name identifying this planning scene monitor
   */
  PlanningSceneMonitor(const planning_scene::PlanningScenePtr &scene, const std::string &robot_description, 
                       const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>(),
                       const std::string &name = "");

  /** @brief Constructor
   *  @param scene The scene instance to maintain up to date with monitored information
   *  @param kml A pointer to a kinematic model loader
   *  @param tf A pointer to a tf::Transformer
   *  @param name A name identifying this planning scene monitor
   */
  PlanningSceneMonitor(const planning_scene::PlanningScenePtr &scene, 
                       const planning_models_loader::KinematicModelLoaderPtr &kml, 
                       const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>(),
                       const std::string &name = "");

  ~PlanningSceneMonitor(void);
  
  /** \brief Get the name of this monitor */
  const std::string& getName(void) const
  {
    return monitor_name_;
  }
  
  /** \brief Get the user kinematic model loader */
  const planning_models_loader::KinematicModelLoaderPtr& getKinematicModelLoader(void) const
  {
    return kinematics_loader_;
  }
  
  const kinematic_model::KinematicModelConstPtr& getKinematicModel(void) const;
  
  /** @brief Get the planning scene
   *  @return An instance of the planning scene*/
  const planning_scene::PlanningScenePtr& getPlanningScene(void)
  {
    return scene_;
  }
  
  /** @brief Get the planning scene
   *  @return An instance of the planning scene*/
  const planning_scene::PlanningSceneConstPtr& getPlanningScene(void) const
  {
    return scene_const_;
  }

  /** @brief Return true if the scene \e scene can be updated directly
      or indirectly by this monitor. This function will return true if
      the pointer of the scene is the same as the one maintained,
      or if a parent of the scene is the one maintained. */
  bool updatesScene(const planning_scene::PlanningSceneConstPtr &scene) const;

  /** @brief Return true if the scene \e scene can be updated directly
      or indirectly by this monitor. This function will return true if
      the pointer of the scene is the same as the one maintained,
      or if a parent of the scene is the one maintained. */
  bool updatesScene(const planning_scene::PlanningScenePtr &scene) const;
  
  /** @brief Get the stored robot description
   *  @return An instance of the stored robot description*/
  const std::string& getRobotDescription(void) const
  {
    return robot_description_;
  }

  /// Get the default robot padding
  double getDefaultRobotPadding(void) const
  {
    return default_robot_padd_;
  }
  
  /// Get the default robot scaling
  double getDefaultRobotScale(void) const
  {
    return default_robot_scale_;
  }
    
  /// Get the default object padding
  double getDefaultObjectPadding(void) const
  {
    return default_object_padd_;
  }
  
  /// Get the default attached padding
  double getDefaultAttachedObjectPadding(void) const
  {
    return default_attached_padd_;
  }
  
  /** @brief Get the instance of the TF client that was passed to the constructor of this class. */
  const boost::shared_ptr<tf::Transformer>& getTFClient(void) const
  {
    return tf_;
  }
  
  /** \brief By default, the maintained planning scene does not reason about diffs. When the flag passed in is true, the maintained
      scene starts counting diffs. Future updates to the planning scene will be stored as diffs and can be retrieved as
      such. Setting the flag to false restores the default behaviour. Maintaining diffs is automatically enabled when
      publishing planning scenes. */
  void monitorDiffs(bool flag);

  /** \brief Start publishing the maintained planning scene. The first message set out is a complete planning scene. 
      Diffs are sent afterwards on updates specified by the \e event bitmask. For UPDATE_SCENE, the full scene is always sent. */
  void startPublishingPlanningScene(SceneUpdateType event, const std::string &planning_scene_topic = "monitored_planning_scene");

  /** \brief Stop publishing the maintained planning scene. */
  void stopPublishingPlanningScene(void);

  /** \brief Set the maximum frequency at which planning scenes are being published */
  void setPlanningScenePublishingFrequency(double hz);
  
  /** \brief Get the maximum frequency at which planning scenes are published (Hz) */
  double getPlanningScenePublishingFrequency(void) const
  {
    return publish_planning_scene_frequency_;
  }
  
  /** @brief Get the stored instance of the stored current state monitor
   *  @return An instance of the stored current state monitor*/
  const CurrentStateMonitorPtr& getStateMonitor(void) const
  {
    return current_state_monitor_;
  }

  /** @brief Update the transforms for the frames that are not part of the kinematic model using tf.
   *  Examples of these frames are the "map" and "odom_combined" transforms. This function is automatically called when data that uses transforms is received.
   *  However, this function should also be called before starting a planning request, for example.
   */
  void updateFrameTransforms(void);

  /** @brief Start the current state monitor
      @param joint_states_topic the topic to listen to for joint states
      @param attached_objects_topic the topic to listen to for attached collision objects */
  void startStateMonitor(const std::string &joint_states_topic = "joint_states", const std::string &attached_objects_topic = "attached_collision_object");

  /** @brief Stop the state monitor*/
  void stopStateMonitor(void);

  /** @brief Update the scene using the monitored state. This function is automatically called when an update to the current state is received (if startStateMonitor() has been called).
      The updates are throttled to a maximum update frequency however, which is set by setStateUpdateFrequency(). */
  void updateSceneWithCurrentState(void);

  /** @brief Update the scene using the monitored state at a specified frequency, in Hz. This function has an effect only when updates from the CurrentStateMonitor are received at a higher frequency.
      In that case, the updates are throttled down, so that they do not exceed a maximum update frequency specified here.
      @param hz the update frequency. By default this is 10Hz. */
  void setStateUpdateFrequency(double hz);

  /** @brief Get the maximum frequency (Hz) at which the current state of the planning scene is updated.*/
  double getStateUpdateFrequency(void);
  
  /** @brief Sometimes the state reported by the robot is outside bounds (outside safety limits). This parameter specifies the accepted error in bounds.
      If the read value is within \e error distance to the accepted safety limit bounds, the value is actually assumed to be the value of the bound itself,
      instead of being slightly outside bounds. By default this value is machine epsilon. */
  void setStateUpdateBoundsError(double error);

  /** @brief Get the error that is considered acceptable for a state to be outside bounds. */
  double getStateUpdateBoundsError(void) const
  {
    return bounds_error_;
  }  
  
  /** @brief Start the scene monitor
   *  @param scene_topic The name of the planning scene topic
   */
  void startSceneMonitor(const std::string &scene_topic = "planning_scene");

  /** @brief Stop the scene monitor*/
  void stopSceneMonitor(void);

  /** @brief Start listening for objects in the world, the collision map and attached collision objects. Additionally, this function starts the OccupancyMapMonitor as well.
   *  @param collision_objects_topic The topic on which to listen for collision objects
   *  @param collision_map_topic The topic on which to listen for the collision map
   *  @param planning_scene_world_topic The topic to listen to for world scene geometry */
  void startWorldGeometryMonitor(const std::string &collision_objects_topic = "collision_object",
                                 const std::string &collision_map_topic = "collision_map",
                                 const std::string &planning_scene_world_topic = "planning_scene_world");

  /** @brief Stop the world geometry monitor */
  void stopWorldGeometryMonitor(void);

  /** @brief Add a function to be called when an update to the scene is received */
  void addUpdateCallback(const boost::function<void(SceneUpdateType)> &fn);

  /** @brief Clear the functions to be called when an update to the scene is received */
  void clearUpdateCallbacks(void);
  
  /** @brief Get the topic names that the monitor is listening to */
  void getMonitoredTopics(std::vector<std::string> &topics) const;
  
  /** \brief Return the time when the last update was made to the planning scene (by the monitor) */
  const ros::Time& getLastUpdateTime(void) const
  {
    return last_update_time_;
  }

  /** \brief Lock the scene for reading (multiple threads can lock for reading at the same time) */
  void lockSceneRead(void);

  /** \brief Unlock the scene from reading (multiple threads can lock for reading at the same time) */
  void unlockSceneRead(void);

  /** \brief Lock the scene for writing (only one thread can lock for writing and no other thread can lock for reading) */
  void lockSceneWrite(void);

  /** \brief Lock the scene from writing (only one thread can lock for writing and no other thread can lock for reading) */
  void unlockSceneWrite(void);

protected:
  
  /** @brief Initialize the planning scene monitor
   *  @param scene The scene instance to fill with data (an instance is allocated if the one passed in is not allocated) */
  void initialize(const planning_scene::PlanningScenePtr &scene);

  /** @brief Configure the collision matrix for a particular scene */
  void configureCollisionMatrix(const planning_scene::PlanningScenePtr &scene);
  
  /** @brief Configure the default padding*/
  void configureDefaultPadding(void);

  /** @brief Callback for a new planning scene msg*/
  void newPlanningSceneCallback(const moveit_msgs::PlanningSceneConstPtr &scene);

  /** @brief Callback for a new collision object msg*/
  void collisionObjectCallback(const moveit_msgs::CollisionObjectConstPtr &obj);

  /** @brief Callback for a new planning scene world*/
  void newPlanningSceneWorldCallback(const moveit_msgs::PlanningSceneWorldConstPtr &world);

  /** @brief Callback for a new collision map*/
  void collisionMapCallback(const moveit_msgs::CollisionMapConstPtr &map);

  /** @brief Callback for octomap updates */
  void octomapUpdateCallback(void);
  
  /** @brief Callback for a new attached object msg*/
  void attachObjectCallback(const moveit_msgs::AttachedCollisionObjectConstPtr &obj);
  
  void getUpdatedFrameTransforms(const kinematic_model::KinematicModelConstPtr &kmodel, std::vector<geometry_msgs::TransformStamped> &transforms);
  
  /// The name of this scene monitor
  std::string                           monitor_name_;
  
  planning_scene::PlanningScenePtr      scene_;
  planning_scene::PlanningSceneConstPtr scene_const_;
  planning_scene::PlanningScenePtr      parent_scene_; /// if diffs are monitored, this is the pointer to the parent scene
  boost::shared_mutex                   scene_update_mutex_; /// mutex for stored scene

  ros::NodeHandle                       nh_;
  ros::NodeHandle                       root_nh_;
  boost::shared_ptr<tf::Transformer>    tf_;
  std::string                           robot_description_;

  /// default robot padding
  double                                default_robot_padd_;
  /// default robot scaling
  double                                default_robot_scale_;
  /// default object padding
  double                                default_object_padd_;
  /// default attached padding
  double                                default_attached_padd_;
  
  // variables for planning scene publishing
  ros::Publisher                        planning_scene_publisher_;
  boost::scoped_ptr<boost::thread>      publish_planning_scene_;
  double                                publish_planning_scene_frequency_;
  SceneUpdateType                       publish_update_types_;
  SceneUpdateType                       new_scene_update_;
  boost::condition_variable_any         new_scene_update_condition_;
  
  // subscribe to various sources of data
  ros::Subscriber                       planning_scene_subscriber_;
  ros::Subscriber                       planning_scene_world_subscriber_;

  boost::scoped_ptr<message_filters::Subscriber<moveit_msgs::CollisionObject> > collision_object_subscriber_; 
  boost::scoped_ptr<tf::MessageFilter<moveit_msgs::CollisionObject> >           collision_object_filter_;
  
  boost::scoped_ptr<message_filters::Subscriber<moveit_msgs::CollisionMap> >    collision_map_subscriber_;
  boost::scoped_ptr<tf::MessageFilter<moveit_msgs::CollisionMap> >              collision_map_filter_;

  // include the octomap monitor as well
  boost::scoped_ptr<occupancy_map_monitor::OccupancyMapMonitor> octomap_monitor_;
  
  ros::Subscriber                       attached_collision_object_subscriber_;
  
  CurrentStateMonitorPtr                current_state_monitor_;
  ros::Time                             last_update_time_; /// Last time the state was updated
  std::vector<boost::function<void(SceneUpdateType)> > update_callbacks_;

  /// the planning scene state is updated at a maximum specified frequency,
  /// and this timestamp is used to implement that functionality
  ros::WallTime                         last_state_update_;

  /// the amount of time to wait in between updates to the robot state (in seconds)
  double                                dt_state_update_; 

  /// the error accepted when the state is reported as outside of bounds;
  double                                bounds_error_;

  planning_models_loader::KinematicModelLoaderPtr kinematics_loader_;

private:

  /** @brief This function is called every time there is a change to the planning scene */
  void processSceneUpdateEvent(SceneUpdateType update_type);
  
  /** @brief */
  void scenePublishingThread(void);
  
  void onStateUpdate(const sensor_msgs::JointStateConstPtr &joint_state);

  class DynamicReconfigureImpl;
  DynamicReconfigureImpl *reconfigure_impl_;
};

typedef boost::shared_ptr<PlanningSceneMonitor> PlanningSceneMonitorPtr;
typedef boost::shared_ptr<const PlanningSceneMonitor> PlanningSceneMonitorConstPtr;

/** \brief This is a convenience class for obtaining access to an instance of a locked PlanningScene */
class LockedPlanningSceneRO
{
public:

  LockedPlanningSceneRO(const PlanningSceneMonitorPtr &planning_scene_monitor) :
    planning_scene_monitor_(planning_scene_monitor)
  {
    initialize(true);
  }
  
  const PlanningSceneMonitorPtr& getPlanningSceneMonitor(void)
  {
    return planning_scene_monitor_;
  }
  
  operator bool() const
  {
    return planning_scene_monitor_ && planning_scene_monitor_->getPlanningScene();
  }

  operator const planning_scene::PlanningSceneConstPtr&() const
  {
    return const_cast<const PlanningSceneMonitor*>(planning_scene_monitor_.get())->getPlanningScene();
  }

  const planning_scene::PlanningSceneConstPtr& operator->() const
  {
    return const_cast<const PlanningSceneMonitor*>(planning_scene_monitor_.get())->getPlanningScene();
  }

protected:

  LockedPlanningSceneRO(const PlanningSceneMonitorPtr &planning_scene_monitor, bool read_only) :
    planning_scene_monitor_(planning_scene_monitor)
  {
    initialize(read_only);
  }
  
  void initialize(bool read_only)
  {
    if (planning_scene_monitor_)
      lock_.reset(new SingleUnlock(planning_scene_monitor_.get(), read_only));
  }
  
  // we use this struct so that lock/unlock are called only once 
  // even if the LockedPlanningScene instance is copied around
  struct SingleUnlock
  {
    SingleUnlock(PlanningSceneMonitor *planning_scene_monitor, bool read_only) :
      planning_scene_monitor_(planning_scene_monitor), read_only_(read_only)
    {
      if (read_only)
        planning_scene_monitor_->lockSceneRead();
      else
        planning_scene_monitor_->lockSceneWrite();
    }
    ~SingleUnlock(void)
    { 
      if (read_only_)
        planning_scene_monitor_->unlockSceneRead();
      else
        planning_scene_monitor_->unlockSceneWrite();
    }
    PlanningSceneMonitor *planning_scene_monitor_;
    bool read_only_;
  };
  
  PlanningSceneMonitorPtr planning_scene_monitor_;
  boost::shared_ptr<SingleUnlock> lock_;
};

class LockedPlanningSceneRW : public LockedPlanningSceneRO
{
public:
  
  LockedPlanningSceneRW(const PlanningSceneMonitorPtr &planning_scene_monitor) :
    LockedPlanningSceneRO(planning_scene_monitor, false)
  {
  }
  
  operator const planning_scene::PlanningScenePtr&()
  {
    return planning_scene_monitor_->getPlanningScene();
  }

  const planning_scene::PlanningScenePtr& operator->()
  {
    return planning_scene_monitor_->getPlanningScene();
  }
};

}

#endif
