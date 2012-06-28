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
#include <planning_scene/planning_scene.h>
#include <planning_models_loader/kinematic_model_loader.h>
#include "planning_scene_monitor/current_state_monitor.h"
#include <boost/thread.hpp>

namespace planning_scene_monitor
{
/**
 * @class PlanningSceneMonitor
 * Subscribes to the topic \e planning_scene */
class PlanningSceneMonitor
{
public:

  /** @brief Constructor
   *  @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   */
  PlanningSceneMonitor(const std::string &robot_description);

  /** @brief Constructor
   *  @param kml A pointer to a kinematic model loader
   */
  PlanningSceneMonitor(const planning_models_loader::KinematicModelLoaderPtr &kml);


  /** @brief Constructor
   *  @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   *  @param tf A pointer to a tf::Transformer
   */
  PlanningSceneMonitor(const std::string &robot_description, const boost::shared_ptr<tf::Transformer> &tf);

  /** @brief Constructor
   *  @param kml A pointer to a kinematic model loader
   *  @param tf A pointer to a tf::Transformer
   */
  PlanningSceneMonitor(const planning_models_loader::KinematicModelLoaderPtr &kml, 
                       const boost::shared_ptr<tf::Transformer> &tf);

  /** @brief Constructor
   *  @param parent The parent planning scene with respect to which the diffs are to be maintained
   *  @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   *  @param tf A pointer to a tf::Transformer
   */
  PlanningSceneMonitor(const planning_scene::PlanningSceneConstPtr &parent, const std::string &robot_description, 
                       const boost::shared_ptr<tf::Transformer> &tf);

  /** @brief Constructor
   *  @param parent The parent planning scene with respect to which the diffs are to be maintained
   *  @param kml A pointer to a kinematic model loader
   *  @param tf A pointer to a tf::Transformer
   */
  PlanningSceneMonitor(const planning_scene::PlanningSceneConstPtr &parent, 
                       const planning_models_loader::KinematicModelLoaderPtr &kml, 
                       const boost::shared_ptr<tf::Transformer> &tf);

  ~PlanningSceneMonitor(void);

  /** \brief Get the user kinematic model loader */
  const planning_models_loader::KinematicModelLoaderPtr& getKinematicModelLoader(void) const
  {
    return kinematics_loader_;
  }
  
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

  /** @brief Get the stored robot description
   *  @return An instance of the stored robot description*/
  const std::string& getRobotDescription(void) const
  {
    return robot_description_;
  }

  /** \brief By default, the maintained planning scene does not reason about diffs. When the flag passed in is true, the maintained
      scene starts counting diffs. Future updates to the planning scene will be stored as diffs and can be retrieved as
      such. Setting the flag to false restores the default behaviour. Maintaining diffs is automatically enabled when
      publishing planning scenes. */
  void monitorDiffs(bool flag);

  /** \brief Start publishing the maintained planning scene. The first message set out is a complete planning scene. Diffs are sent afterwards. */
  void startPublishingPlanningScene(const std::string &planning_scene_topic = "monitored_planning_scene");

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
   *  Examples of these frames are the "map" and "odom_combined" transforms.
   */
  void updateFrameTransforms(void);

  /** @brief Start the current state monitor
      @param joint_states_topic the topic to listen to for joint states
      @param attached_objects_topic the topic to listen to for attached collision objects */
  void startStateMonitor(const std::string &joint_states_topic = "joint_states", const std::string &attached_objects_topic = "attached_collision_object");

  /** @brief Stop the state monitor*/
  void stopStateMonitor(void);

  /** @brief Update the scene using the monitored state*/
  void updateSceneWithCurrentState(void);

  /** @brief Update the scene using the monitored state at a specified frequency, in Hz
      @param hz the update frequency */
  void setStateUpdateFrequency(double hz);

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

  /** @brief Start listening for objects in the world, the collision map and attached collision objects
   *  @param collision_objects_topic The topic on which to listen for collision objects
   *  @param collision_map_topic The topic on which to listen for the collision map
   *  @param planning_scene_world_topic The topic to listen to for world scene geometry */
  void startWorldGeometryMonitor(const std::string &collision_objects_topic = "collision_object",
                                 const std::string &collision_map_topic = "collision_map",
                                 const std::string &planning_scene_world_topic = "planning_scene_world");

  /** @brief Stop the world geometry monitor*/
  void stopWorldGeometryMonitor(void);

  /** @brief Set the function to be called when an update to the scene is received */
  void setUpdateCallback(const boost::function<void()> &fn);
  
  /** @brief Get the topic names that the monitor is listening to */
  void getMonitoredTopics(std::vector<std::string> &topics) const;
  
  /** \brief Return the time when the last update was made to the planning scene (by the monitor) */
  const ros::Time& getLastUpdateTime(void) const
  {
    return last_update_time_;
  }

  /** \brief Lock the scene */
  void lockScene(void);

  /** \brief Unlock the scene */
  void unlockScene(void);

protected:

  /** @brief Initialize the planning scene monitor
   *  @param parent The parent planning scene with respect to which the diffs are to be maintained
   *  @param robot_description The name of the ROS parameter that contains the URDF */
  void initialize(const planning_scene::PlanningSceneConstPtr &parent, const std::string &robot_description);

  /** @brief Initialize the planning scene monitor
   *  @param parent The parent planning scene with respect to which the diffs are to be maintained */
  void initialize(const planning_scene::PlanningSceneConstPtr &parent);

  /** @brief Configure the default collision matrix*/
  void configureDefaultCollisionMatrix(void);

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

  /** @brief Callback for a new attached object msg*/
  void attachObjectCallback(const moveit_msgs::AttachedCollisionObjectConstPtr &obj);

  planning_scene::PlanningScenePtr      scene_; /// internally stored planning scene

  planning_scene::PlanningSceneConstPtr scene_const_; /// internally stored

  planning_scene::PlanningScenePtr      parent_scene_; /// if diffs are monitored, this is the pointer to the parent scene

  boost::mutex                          scene_update_mutex_; /// mutex for stored scene


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
  
  ros::Publisher                        planning_scene_publisher_;
  boost::scoped_ptr<boost::thread>      publish_planning_scene_;
  double                                publish_planning_scene_frequency_;
  bool                                  new_scene_update_;
  boost::condition_variable             new_scene_update_condition_;
  
  ros::Subscriber                       planning_scene_subscriber_;
  ros::Subscriber                       planning_scene_world_subscriber_;

  message_filters::Subscriber<moveit_msgs::CollisionObject> *collision_object_subscriber_;
  tf::MessageFilter<moveit_msgs::CollisionObject>           *collision_object_filter_;

  message_filters::Subscriber<moveit_msgs::CollisionMap>    *collision_map_subscriber_;
  tf::MessageFilter<moveit_msgs::CollisionMap>              *collision_map_filter_;

  ros::Subscriber                       attached_collision_object_subscriber_;
  
  CurrentStateMonitorPtr                current_state_monitor_;
  ros::Time                             last_update_time_; /// Last time the state was updated
  boost::function<void()>               update_callback_;

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
  void processSceneUpdateEvent(void);
  
  /** @brief */
  void scenePublishingThread(void);
  
  void onStateUpdate(const sensor_msgs::JointStateConstPtr &joint_state);

  class DynamicReconfigureImpl;
  boost::scoped_ptr<DynamicReconfigureImpl> reconfigure_impl_;
};

typedef boost::shared_ptr<PlanningSceneMonitor> PlanningSceneMonitorPtr;
typedef boost::shared_ptr<const PlanningSceneMonitor> PlanningSceneMonitorConstPtr;
}

#endif
