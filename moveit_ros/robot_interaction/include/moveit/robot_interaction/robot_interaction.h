/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Adam Leeper */

#ifndef MOVEIT_ROBOT_INTERACTION_ROBOT_INTERACTION_
#define MOVEIT_ROBOT_INTERACTION_ROBOT_INTERACTION_

#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <geometry_msgs/PoseArray.h>
#include <interactive_markers/menu_handler.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_interaction/interaction.h>
#include <boost/function.hpp>
#include <boost/thread.hpp>

// This is needed for legacy code that includes robot_interaction.h but not
// interaction_handler.h
#include <moveit/robot_interaction/interaction_handler.h>

namespace interactive_markers
{
class InteractiveMarkerServer;
}

namespace robot_interaction
{

class InteractionHandler;
typedef boost::shared_ptr<InteractionHandler> InteractionHandlerPtr;

class KinematicOptionsMap;
typedef boost::shared_ptr<KinematicOptionsMap> KinematicOptionsMapPtr;


// Manage interactive markers for controlling a robot state.
//
// The RobotInteraction class manages one or more InteractionHandler objects
// each of which maintains a set of interactive markers for manipulating one
// group of one RobotState.
//
// The group being manipulated is common to all InteractionHandler objects
// contained in a RobotInteraction instance.
class RobotInteraction
{
public:
  // DEPRECATED TYPES.  For backwards compatibility.  Avoid using these.
  typedef ::robot_interaction::InteractionHandler InteractionHandler;
  typedef ::robot_interaction::InteractionHandlerPtr InteractionHandlerPtr;
  typedef ::robot_interaction::EndEffectorInteraction EndEffector;
  typedef ::robot_interaction::JointInteraction Joint;
  typedef ::robot_interaction::GenericInteraction Generic;
public:

  /// The topic name on which the internal Interactive Marker Server operates
  static const std::string INTERACTIVE_MARKER_TOPIC;


  RobotInteraction(const robot_model::RobotModelConstPtr &kmodel,
                   const std::string &ns = "");
  virtual ~RobotInteraction();

  const std::string& getServerTopic(void) const
  {
    return topic_;
  }

  /// add an interaction.
  /// An interaction is a marker that can be used to manipulate the robot
  /// state. 
  /// This function does not add any markers.  To add markers for all
  /// active interactions call addInteractiveMarkers().
  /// construct - a callback to construct the marker.  See comment on
  ///              InteractiveMarkerConstructorFn above.
  /// update - Called when the robot state changes.  Updates the marker pose.
  ///              Optional.  See comment on InteractiveMarkerUpdateFn above.
  /// process - called when the marker moves.  Updates the robot state.  See
  ///              comment on ProcessFeedbackFn above.
  void addActiveComponent(
        const InteractiveMarkerConstructorFn &construct,
        const ProcessFeedbackFn &process,
        const InteractiveMarkerUpdateFn &update = InteractiveMarkerUpdateFn(),
        const std::string &name = "");

  /// Adds an interaction for:
  ///  - each end effector in the group that can be controller by IK
  ///  - each floating joint
  ///  - each planar joint
  /// If no end effector exists in the robot then adds an interaction for
  /// the last link in the chain.
  /// This function does not add any markers.  To add markers for all
  /// active interactions call addInteractiveMarkers().
  void decideActiveComponents(const std::string &group);
  void decideActiveComponents(const std::string &group,
                              InteractionStyle::InteractionStyle style);

  /// remove all interactions.
  /// Also removes all markers.
  void clear();

  /// Add interactive markers for all active interactions.
  /// This adds markers just to the one handler.  If there are multiple handlers
  /// call this for each handler for which you want markers.
  /// The markers are not actually added until you call
  /// publishInteractiveMarkers().
  void addInteractiveMarkers(
        const ::robot_interaction::InteractionHandlerPtr &handler,
        const double marker_scale = 0.0);

  // Update pose of all interactive markers to match the handler's RobotState.
  // Call this when the handler's RobotState changes.
  void updateInteractiveMarkers(
        const ::robot_interaction::InteractionHandlerPtr &handler);

  // True if markers are being shown for this handler.
  bool showingMarkers(
        const ::robot_interaction::InteractionHandlerPtr &handler);

  // Display all markers that have been added.
  // This is needed after calls to addInteractiveMarkers() to publish the
  // resulting markers so they get displayed.  This call is not needed after
  // calling updateInteractiveMarkers() which publishes the results itself.
  void publishInteractiveMarkers();

  // Clear all interactive markers.
  // This removes all interactive markers but does not affect which
  // interactions are active.  After this a call to publishInteractiveMarkers()
  // is needed to actually remove the markers from the display.
  void clearInteractiveMarkers();

  const robot_model::RobotModelConstPtr& getRobotModel() const { return robot_model_; }

  // Get the kinematic options map.
  // Use this to set kinematic options (defaults or per-group).
  KinematicOptionsMapPtr getKinematicOptionsMap() { return kinematic_options_map_; }
  
  // enable/disable subscription of the topics to move interactive marker
  void toggleMoveInteractiveMarkerTopic(bool enable);
private:
  // called by decideActiveComponents(); add markers for end effectors
  void decideActiveEndEffectors(const std::string &group);
  void decideActiveEndEffectors(const std::string &group,
                                InteractionStyle::InteractionStyle style);

  // called by decideActiveComponents(); add markers for planar and floating
  // joints
  void decideActiveJoints(const std::string &group);

  void moveInteractiveMarker(const std::string name,
                             const geometry_msgs::PoseStampedConstPtr& msg);
  // register the name of the topic and marker name to move
  // interactive marker from other ROS nodes
  void registerMoveInteractiveMarkerTopic(
    const std::string marker_name, const std::string& name);
  // return the diameter of the sphere that certainly can enclose the AABB of
  // the links in this group
  double computeGroupMarkerSize(const std::string &group);
  void computeMarkerPose(
        const ::robot_interaction::InteractionHandlerPtr &handler,
        const EndEffectorInteraction &eef,
        const robot_state::RobotState &robot_state,
        geometry_msgs::Pose &pose,
        geometry_msgs::Pose &control_to_eef_tf) const;

  void addEndEffectorMarkers(
        const ::robot_interaction::InteractionHandlerPtr &handler,
        const EndEffectorInteraction& eef,
        visualization_msgs::InteractiveMarker& im,
        bool position = true,
        bool orientation = true);
  void addEndEffectorMarkers(
        const ::robot_interaction::InteractionHandlerPtr &handler,
        const EndEffectorInteraction& eef,
        const geometry_msgs::Pose& offset,
        visualization_msgs::InteractiveMarker& im,
        bool position = true,
        bool orientation = true);
  void processInteractiveMarkerFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void subscribeMoveInteractiveMarker(const std::string marker_name, const std::string& name);
  void processingThread();
  void clearInteractiveMarkersUnsafe();

  boost::scoped_ptr<boost::thread> processing_thread_;
  bool run_processing_thread_;

  boost::condition_variable new_feedback_condition_;
  std::map<std::string, visualization_msgs::InteractiveMarkerFeedbackConstPtr> feedback_map_;

  robot_model::RobotModelConstPtr robot_model_;

  std::vector<EndEffectorInteraction> active_eef_;
  std::vector<JointInteraction> active_vj_;
  std::vector<GenericInteraction> active_generic_;

  std::map<std::string, ::robot_interaction::InteractionHandlerPtr> handlers_;
  std::map<std::string, std::size_t> shown_markers_;

  // This mutex is locked every time markers are read or updated;
  // This includes the active_* arrays and shown_markers_
  // Please note that this mutex *MUST NOT* be locked while operations
  // on the interative marker server are called because the server
  // also locks internally and we could othewrise end up with a problem
  // of Thread 1: Lock A,         Lock B, Unlock B, Unloack A
  //    Thread 2:         Lock B, Lock A
  // => deadlock
  boost::mutex marker_access_lock_;

  interactive_markers::InteractiveMarkerServer *int_marker_server_;
  // ros subscribers to move the interactive markers by other ros nodes
  std::vector<ros::Subscriber> int_marker_move_subscribers_;
  // the array of the names of the topics which need to be subscribed
  // to move the interactive markers by other ROS nodes
  std::vector<std::string> int_maker_move_topics_;
  // the array of the marker names in the same order to int_maker_move_topics_
  std::vector<std::string> int_marker_names_;
  
  std::string topic_;

  // options for doing IK
  // Locking is done internally
  KinematicOptionsMapPtr kinematic_options_map_;

public:
  // DEPRECATED.  This is included for backwards compatibility.
  // These classes/enums used to be subclasses of RobotInteraction.  This
  // allows client code to continue to work as if they are.

  /// DEPRECATED. Instead use
  /// robot_interaction::InteractionStyle::InteractionStyle in interaction.h
  enum EndEffectorInteractionStyle
  {
    EEF_POSITION_ARROWS = InteractionStyle::POSITION_ARROWS,
    EEF_ORIENTATION_CIRCLES = InteractionStyle::ORIENTATION_CIRCLES,
    EEF_POSITION_SPHERE = InteractionStyle::POSITION_SPHERE,
    EEF_ORIENTATION_SPHERE = InteractionStyle::ORIENTATION_SPHERE,
    EEF_POSITION_EEF = InteractionStyle::POSITION_EEF,
    EEF_ORIENTATION_EEF = InteractionStyle::ORIENTATION_EEF,
    EEF_FIXED = InteractionStyle::FIXED,
    EEF_POSITION = InteractionStyle::POSITION,
    EEF_ORIENTATION = InteractionStyle::ORIENTATION,
    EEF_6DOF = InteractionStyle::SIX_DOF,
    EEF_6DOF_SPHERE = InteractionStyle::SIX_DOF_SPHERE,
    EEF_POSITION_NOSPHERE = InteractionStyle::POSITION_NOSPHERE,
    EEF_ORIENTATION_NOSPHERE = InteractionStyle::ORIENTATION_NOSPHERE,
    EEF_6DOF_NOSPHERE = InteractionStyle::SIX_DOF_NOSPHERE
  };
  // DEPRECATED.  Use InteractionStyle::InteractionStyle version.
  void decideActiveComponents(const std::string &group,
                              EndEffectorInteractionStyle style);
  // DEPRECATED.  Use InteractionStyle::InteractionStyle version.
  void decideActiveEndEffectors(const std::string &group,
                                EndEffectorInteractionStyle style);
  // DEPRECATED
  const std::vector<EndEffectorInteraction>& getActiveEndEffectors() const
  {
    return active_eef_;
  }
  // DEPRECATED
  const std::vector<JointInteraction>& getActiveJoints() const
  {
    return active_vj_;
  }
  // DEPRECATED
  static bool updateState(
            robot_state::RobotState &state,
            const EndEffectorInteraction &eef,
            const geometry_msgs::Pose &pose,
            unsigned int attempts,
            double ik_timeout,
            const robot_state::GroupStateValidityCallbackFn &validity_callback =
                                robot_state::GroupStateValidityCallbackFn(),
            const kinematics::KinematicsQueryOptions &kinematics_query_options =
                                kinematics::KinematicsQueryOptions());
};

typedef boost::shared_ptr<RobotInteraction> RobotInteractionPtr;
typedef boost::shared_ptr<const RobotInteraction> RobotInteractionConstPtr;


}

#endif
