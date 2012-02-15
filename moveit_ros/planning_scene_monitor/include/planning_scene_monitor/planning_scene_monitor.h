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

#ifndef PLANNING_SCENE_MONITOR_PLANNING_SCENE_MONITOR_
#define PLANNING_SCENE_MONITOR_PLANNING_SCENE_MONITOR_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <planning_scene/planning_scene.h>
#include "planning_scene_monitor/robot_model_loader.h"
#include "planning_scene_monitor/current_state_monitor.h"

namespace planning_scene_monitor
{
/**
 * @class PlanningSceneMonitor
 * Subscribes to two topics \e planning_scene and \e planning_scene_diff*/
class PlanningSceneMonitor
{
public:
  /** @brief Constructor
   *  @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   */
  PlanningSceneMonitor(const std::string &robot_description);

  /** @brief Constructor
   *  @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   *  @param tf A pointer to a tf::Transformer
   */
  PlanningSceneMonitor(const std::string &robot_description, tf::Transformer *tf);

  /** @brief Constructor
   *  @param parent The parent planning scene with respect to which the diffs are to be maintained
   *  @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   *  @param tf A pointer to a tf::Transformer
   */
  PlanningSceneMonitor(const planning_scene::PlanningSceneConstPtr &parent, const std::string &robot_description, tf::Transformer *tf);
  ~PlanningSceneMonitor(void);

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

  void monitorDiffs(bool flag);

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

  /** @brief Start the scene monitor
   *  @param scene_topic The name of the planning scene topic
   *  @param planning_scene_diff The name of the planning scene diff topic
   */
  void startSceneMonitor(const std::string &scene_topic = "planning_scene",
                         const std::string &scene_diff_topic = "planning_scene_diff");

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
   *  @param robot_description The name of the ROS parameter that contains the URDF (in string format)*/
  void initialize(const planning_scene::PlanningSceneConstPtr &parent, const std::string &robot_description);

  /** @brief Configure the default collision matrix*/
  void configureDefaultCollisionMatrix(void);

  /** @brief Configure the default padding*/
  void configureDefaultPadding(void);

  /** @brief Configure the default joint limits*/
  void configureDefaultJointLimits(void);

  /** @brief Callback for a new planning scene msg*/
  void newPlanningSceneCallback(const moveit_msgs::PlanningSceneConstPtr &scene);

  /** @brief Callback for a new planning scene diff msg*/
  void newPlanningSceneDiffCallback(const moveit_msgs::PlanningSceneConstPtr &scene);

  /** @brief Callback for a new collision object msg*/
  void collisionObjectCallback(const moveit_msgs::CollisionObjectConstPtr &obj);

  /** @brief Callback for a new planning scene world*/
  void newPlanningSceneWorldCallback(const moveit_msgs::PlanningSceneWorldConstPtr &world);

  /** @brief Callback for a new collision map*/
  void collisionMapCallback(const moveit_msgs::CollisionMapConstPtr &map);

  /** @brief Callback for a new attached object msg*/
  void attachObjectCallback(const moveit_msgs::AttachedCollisionObjectConstPtr &obj);

  void onStateUpdate(const sensor_msgs::JointStateConstPtr &joint_state);

  planning_scene::PlanningScenePtr      scene_; /// internally stored planning scene

  planning_scene::PlanningSceneConstPtr scene_const_; /// internally stored
  boost::mutex                          scene_update_mutex_; /// mutex for stored scene

  ros::NodeHandle                       nh_;
  ros::NodeHandle                       root_nh_;
  tf::Transformer                      *tf_;
  std::string                           robot_description_;
  double                                default_robot_padd_; /// default robot padding
  double                                default_robot_scale_; /// default robot scaling
  double                                default_object_padd_; /// default object padding
  double                                default_attached_padd_; /// default attached padding

  ros::Subscriber                       planning_scene_subscriber_;
  ros::Subscriber                       planning_scene_diff_subscriber_;

  ros::Subscriber                       planning_scene_world_subscriber_;

  message_filters::Subscriber<moveit_msgs::CollisionObject> *collision_object_subscriber_;
  tf::MessageFilter<moveit_msgs::CollisionObject> *collision_object_filter_;
  message_filters::Subscriber<moveit_msgs::CollisionMap> *collision_map_subscriber_;
  tf::MessageFilter<moveit_msgs::CollisionMap> *collision_map_filter_;

  message_filters::Subscriber<moveit_msgs::AttachedCollisionObject> *attached_collision_object_subscriber_;

  CurrentStateMonitorPtr                current_state_monitor_;
  ros::Time                             last_update_time_; /// Last time the state was updated

  boost::function<void()>               update_callback_;

  /// the planning scene state is updated at a maximum specified frequency,
  /// and this timestamp is used to implement that functionality
  ros::WallTime                         last_state_update_;

  /// the amount of time to wait in between updates to the robot state (in seconds)
  double                                dt_state_update_;

  std::map<std::string, std::vector<moveit_msgs::JointLimits > > individual_joint_limits_map_;
  std::map<std::string, std::vector<moveit_msgs::JointLimits> > group_joint_limits_map_;

};

typedef boost::shared_ptr<PlanningSceneMonitor> PlanningSceneMonitorPtr;
typedef boost::shared_ptr<const PlanningSceneMonitor> PlanningSceneMonitorConstPtr;
}

#endif
