/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Ioan Sucan */

#ifndef MOVEIT_ROBOT_INTERACTION_ROBOT_INTERACTION_
#define MOVEIT_ROBOT_INTERACTION_ROBOT_INTERACTION_

#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <moveit/robot_state/robot_state.h>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <tf/tf.h>

namespace interactive_markers
{
class InteractiveMarkerServer;
}

namespace robot_interaction
{
  
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
  
  /// The topic name on which the internal Interactive Marker Server operates
  static const std::string INTERACTIVE_MARKER_TOPIC;
  
  /// Representation of an interaction via an end-effector
  struct EndEffector
  {
    /// The name of the group that sustains the end-effector (usually an arm)
    std::string parent_group;

    /// The name of the link in the parent group that connects to the end-effector
    std::string parent_link;

    /// The name of the group that defines the group joints
    std::string eef_group;
    
    /// The size of the end effector group (diameter of enclosing sphere)
    double size;
  };

  /// Representation of an interaction via a joint.
  struct Joint
  { 
    /// The link in the robot model this joint is a parent of
    std::string connecting_link;
    
    /// The name of the joint
    std::string joint_name;

    /// The type of joint disguised as the number of DOF it has.  3=PLANAR in X/Y; 6=FLOATING
    unsigned int dof;

    /// The size of the connecting link  (diameter of enclosing sphere)
    double size;
  };
  
  /// When using generic markers, a means to construct the marker is needed: this callback.
  /// The callback should set up the passed in marker according to the passed in robot state. 
  /// Return true on success.  Return false on failure or if the marker should not be added
  /// and displayed.
  typedef boost::function<bool(const robot_state::RobotState&, visualization_msgs::InteractiveMarker&)> InteractiveMarkerConstructorFn;

  /// When using generic markers, this callback is called when the interactive marker changes and sends feedback.
  /// Callback should update the robot state that was passed in according to the new position of the marker. Return true if the update was successful.
  /// Return false if the state was not successfully updated.
  typedef boost::function<bool(robot_state::RobotState&, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)> ProcessFeedbackFn;

  /// When using generic markers, this callback is called when the robot state changes. Callback should calculate a new
  /// pose for the marker based on the passed in robot state.
  /// Return true if the pose was modified, false if no update is to be issued (pose is unchanged).
  typedef boost::function<bool(const robot_state::RobotState&, geometry_msgs::Pose&)> InteractiveMarkerUpdateFn;

  /// Representation of a generic interaction.  Displays one interactive marker.
  struct Generic
  {
    InteractiveMarkerConstructorFn construct_marker; // see comment on typedef above
    ProcessFeedbackFn process_feedback;              // see comment on typedef above
    InteractiveMarkerUpdateFn update_pose;           // see comment on typedef above
    std::string marker_name_suffix; // automatically generated suffix added to name of markers
  };
  
  class InteractionHandler;

  /// This function is called by the InteractionHandler::handle* functions, when changes are made to the internal robot state the handler maintains.
  /// The handler passes its own pointer as argument to the callback, as well as a boolean flag that indicates wheher the error state changed --
  /// whether updates to the robot state performed in the InteractionHandler::handle* functions have switched from failing to succeeding or the other way around.
  typedef boost::function<void(InteractionHandler*, bool)> InteractionHandlerCallbackFn;
  
  /// Manage interactive markers to control a RobotState.
  ///
  /// Each instance maintains one or more interactive markers to control various joints in one group of one RobotState.
  /// The group being controlled is maintained by the RobotInteraction object that contains this InteractionHandler object.
  /// All InteractionHandler objects in the same RobotInteraction are controlling the same group.
  class InteractionHandler
  {
  public:

    InteractionHandler(const std::string &name,
                       const robot_state::RobotState &kstate,
                       const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>());
    InteractionHandler(const std::string &name,
                       const robot_model::RobotModelConstPtr &kmodel,
                       const boost::shared_ptr<tf::Transformer> &tf = boost::shared_ptr<tf::Transformer>());
    
    virtual ~InteractionHandler()
    {
    }
    
    const std::string& getName() const
    {
      return name_;
    }
    
    robot_state::RobotStateConstPtr getState() const;    
    void setState(const robot_state::RobotState& kstate);
    
    void setUpdateCallback(const InteractionHandlerCallbackFn &callback)
    {
      update_callback_ = callback;
    }    
    
    const InteractionHandlerCallbackFn& getUpdateCallback() const
    {
      return update_callback_;
    }    

    void setStateValidityCallback(const robot_state::StateValidityCallbackFn &callback)
    {
      state_validity_callback_fn_ = callback;
    }
    
    const robot_state::StateValidityCallbackFn& getStateValidityCallback() const
    {
      return state_validity_callback_fn_;
    }
    
    void setIKTimeout(double timeout)
    {
      ik_timeout_ = timeout;
    }
    
    double getIKTimeout() const
    {
      return ik_timeout_;
    }
    
    void setIKAttempts(unsigned int attempts)
    {
      ik_attempts_ = attempts;
    }
    
    unsigned int getIKAttempts() const
    {
      return ik_attempts_;
    }
        
    void setMeshesVisible(bool visible)
    {
      display_meshes_ = visible;
    }

    bool getMeshesVisible() const
    {
      return display_meshes_;
    }

    void setControlsVisible(bool visible)
    {
      display_controls_ = visible;
    }

    bool getControlsVisible() const
    {
      return display_controls_;
    }

    void setPoseOffset(const EndEffector& eef, const geometry_msgs::Pose& m);

    void clearPoseOffset(const RobotInteraction::EndEffector& eef);

    void clearPoseOffsets();

    bool getPoseOffset(const RobotInteraction::EndEffector& eef, geometry_msgs::Pose& m);
    bool getPoseOffset(const RobotInteraction::Joint& vj, geometry_msgs::Pose& m);
    
    /** \brief Get the last interactive_marker command pose for the end-effector
     * @param The end-effector in question.
     * @param A PoseStamped message containing the last (offset-adjusted) pose commanded for the end-effector.
     * @return True if a pose for that end-effector was found, false otherwise.
     */
    bool getLastEndEffectorMarkerPose(const RobotInteraction::EndEffector& eef, geometry_msgs::PoseStamped& pose);
    bool getLastJointMarkerPose(const RobotInteraction::Joint& vj, geometry_msgs::PoseStamped& pose);

    void clearLastEndEffectorMarkerPose(const RobotInteraction::EndEffector& eef);
    void clearLastJointMarkerPose(const RobotInteraction::Joint& vj);
    void clearLastMarkerPoses();

    /** \brief Update the internal state maintained by the handler using
     * information from the received feedback message. */
    virtual void handleEndEffector(const RobotInteraction::EndEffector &eef,
                                   const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    /** \brief Update the internal state maintained by the handler using
     * information from the received feedback message. */
    virtual void handleJoint(const RobotInteraction::Joint &vj,
                             const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    /** \brief Update the internal state maintained by the handler using
     * information from the received feedback message. */
    virtual void handleGeneric(const RobotInteraction::Generic &g,
                               const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    
    virtual bool inError(const RobotInteraction::EndEffector& eef) const;
    virtual bool inError(const RobotInteraction::Joint& vj) const;
    virtual bool inError(const RobotInteraction::Generic& g) const;

    void clearError(void);
    
  protected:

    bool transformFeedbackPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
                               const geometry_msgs::Pose &offset,
                               geometry_msgs::PoseStamped &tpose);

    robot_state::RobotStatePtr getUniqueStateAccess();
    void setStateToAccess(robot_state::RobotStatePtr &state);
    
    std::string name_;
    std::string planning_frame_;
    robot_state::RobotStatePtr kstate_;
    boost::shared_ptr<tf::Transformer> tf_;
    std::set<std::string> error_state_;
    std::map<std::string, geometry_msgs::PoseStamped> pose_map_;
    std::map<std::string, geometry_msgs::Pose> offset_map_;

    // Called when the RobotState maintained by the handler changes.  The caller may, for example, redraw the robot at the new state.
    // handler is the handler that changed.
    // error_state_changed is true if an end effector's error state may have changed.
    boost::function<void(InteractionHandler* handler, bool error_state_changed)> update_callback_;

    robot_state::StateValidityCallbackFn state_validity_callback_fn_;
    double ik_timeout_;
    unsigned int ik_attempts_;
    bool display_meshes_;
    bool display_controls_;
    
  private:
    
    mutable boost::mutex state_lock_;
    mutable boost::condition_variable state_available_condition_;
    boost::mutex pose_map_lock_;
    boost::mutex offset_map_lock_;

    void setup();
  };

  typedef boost::shared_ptr<InteractionHandler> InteractionHandlerPtr;
  typedef boost::shared_ptr<const InteractionHandler> InteractionHandlerConstPtr;
  
  RobotInteraction(const robot_model::RobotModelConstPtr &kmodel, const std::string &ns = "");
  virtual ~RobotInteraction();
  
  const std::string& getServerTopic(void) const
  {
    return topic_;
  }
  
  // add an interactive marker.
  // construct - a callback to construct the marker.  See comment on InteractiveMarkerConstructorFn above.
  // update - Called when the robot state changes.  Updates the marker pose.  Optional.  See comment on InteractiveMarkerUpdateFn above.
  // process - called when the marker moves.  Updates the robot state.  See comment on ProcessFeedbackFn above.
  void addActiveComponent(const InteractiveMarkerConstructorFn &construct,
                          const ProcessFeedbackFn &process,
                          const InteractiveMarkerUpdateFn &update = InteractiveMarkerUpdateFn(),
                          const std::string &name = "");

  // Adds an interactive marker for:
  //  - each end effector in the group that can be controller by IK
  //  - each floating joint
  //  - each planar joint
  // If no end effector exists in the robot then adds an interactive marker for the last link in the chain.
  void decideActiveComponents(const std::string &group);

  /// called by decideActiveComponents(); add markers for end effectors
  void decideActiveEndEffectors(const std::string &group);

  /// called by decideActiveComponents(); add markers for planar and floating joints
  void decideActiveJoints(const std::string &group);
  
  // remove all interactive markers.
  void clear();
  
  enum EefInteractionStyle { EEF_6DOF, EEF_POSITION, EEF_ORIENTATION };

  void addInteractiveMarkers(const InteractionHandlerPtr &handler, const double marker_scale = 0.0, EefInteractionStyle style = EEF_6DOF);
  void updateInteractiveMarkers(const InteractionHandlerPtr &handler);
  bool showingMarkers(const InteractionHandlerPtr &handler);

  void publishInteractiveMarkers();
  void clearInteractiveMarkers();
  
  const std::vector<EndEffector>& getActiveEndEffectors() const
  {
    return active_eef_;
  }

  const std::vector<Joint>& getActiveJoints() const
  {
    return active_vj_;
  }
  
  static bool updateState(robot_state::RobotState &state, const EndEffector &eef, const geometry_msgs::Pose &pose,
                          unsigned int attempts, double ik_timeout, const robot_state::StateValidityCallbackFn &validity_callback = robot_state::StateValidityCallbackFn());
  static bool updateState(robot_state::RobotState &state, const Joint &vj, const geometry_msgs::Pose &pose);

private:
  
  // return the diameter of the sphere that certainly can enclose the AABB of the links in this group
  double computeGroupMarkerSize(const std::string &group);  
  void computeMarkerPose(const InteractionHandlerPtr &handler, const EndEffector &eef, const robot_state::RobotState &robot_state,
                         geometry_msgs::Pose &pose, geometry_msgs::Pose &control_to_eef_tf) const;
  
  void addEndEffectorMarkers(const InteractionHandlerPtr &handler, const EndEffector& eef, visualization_msgs::InteractiveMarker& im);
  void addEndEffectorMarkers(const InteractionHandlerPtr &handler, const EndEffector& eef, const geometry_msgs::Pose& offset, visualization_msgs::InteractiveMarker& im);
  void processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void processingThread();
  void clearInteractiveMarkersUnsafe();
  
  boost::scoped_ptr<boost::thread> processing_thread_;
  bool run_processing_thread_;

  boost::condition_variable new_feedback_condition_;
  std::map<std::string, visualization_msgs::InteractiveMarkerFeedbackConstPtr> feedback_map_;

  robot_model::RobotModelConstPtr robot_model_;
  
  std::vector<EndEffector> active_eef_;
  std::vector<Joint> active_vj_;
  std::vector<Generic> active_generic_;
  
  std::map<std::string, InteractionHandlerPtr> handlers_;
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
  std::string topic_;
};

typedef boost::shared_ptr<RobotInteraction> RobotInteractionPtr;
typedef boost::shared_ptr<const RobotInteraction> RobotInteractionConstPtr;

}

#endif
