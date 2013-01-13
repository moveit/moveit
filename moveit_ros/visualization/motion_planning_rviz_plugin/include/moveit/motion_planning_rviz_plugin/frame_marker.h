/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Mario Prats */

#ifndef MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_FRAME_MARKER
#define MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_FRAME_MARKER

#include <rviz/display_context.h>
#include <rviz/default_plugin/interactive_markers/interactive_marker.h>

#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/kinematic_state/kinematic_state.h>


#include <string>


namespace moveit_rviz_plugin
{

class MotionPlanningDisplay;

/** Base class for interactive markers displaying a coordinate system
 * A basic coordinate system is composed its three axis and a small sphere at the origin
 * The frame can be selected/unselected. When selected, controls for positioning are displayed
 */
class FrameMarker
{
public:
  visualization_msgs::InteractiveMarker imarker_msg;
  boost::shared_ptr<rviz::InteractiveMarker> imarker;

  FrameMarker() {}
  /** Constructor
   * @param parent_node the Ogre node that will hold this interactive marker
   * @param context the display context
   * @param name the name for the interactive marker
   * @param frame_id the frame_id the interactive marker is given relative to
   * @param pose the desired pose of the coordinate system
   * @param scale the scale of the coordinate system
   * @param color the color of the sphere drawn at the origin of the frame
   * @param is_selected whether this frame marker is selected or not by default. When selected, controls are displayed
   * @param visible_x, visible_y, visible_z define the visibility of each axis
   */
  FrameMarker(Ogre::SceneNode *parent_node, rviz::DisplayContext *context, const std::string &name,
              const std::string &frame_id, const geometry_msgs::Pose &pose, double scale, const std_msgs::ColorRGBA &color,
              bool is_selected = false, bool visible_x = true, bool visible_y = true, bool visible_z = true);

  FrameMarker(Ogre::SceneNode *parent_node, rviz::DisplayContext *context, const std::string &name,
              const std::string &frame_id, const geometry_msgs::Pose &pose, double scale, const float color[4],
              bool is_selected = false, bool visible_x = true, bool visible_y = true, bool visible_z = true);

  virtual void updateMarker(void)
  {
    imarker->processMessage(imarker_msg);
  }

  virtual void hide(void);
  virtual void show(MotionPlanningDisplay *pdisplay, rviz::DisplayContext *context);

  virtual void setAxisVisibility(bool x, bool y, bool z)
  {
    visible_x_ = x;
    visible_y_ = y;
    visible_z_ = z;
    rebuild();
  }

  virtual void setColor(float r, float g, float b, float a);

  virtual void getPosition(geometry_msgs::Point &position);
  virtual void getOrientation(geometry_msgs::Quaternion &orientation);

  virtual void select(void);
  virtual void unselect(void);

  bool isSelected(void)
  {
    return selected_;
  }

  bool isVisible()
  {
    return (imarker);
  }

  virtual ~FrameMarker()
  {}

protected:
  virtual void buildFrom(const std::string &name, const std::string &frame_id, const geometry_msgs::Pose &pose, double scale, const std_msgs::ColorRGBA &color);
  virtual void rebuild();

  Ogre::SceneNode *parent_node_;
  rviz::DisplayContext* context_;

  bool selected_;
  bool visible_x_, visible_y_, visible_z_;
  std_msgs::ColorRGBA color_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
};


/** A special FrameMarker that displays the robot end-effector mesh when selected.
 * It also allows defining different states (rechable, not-reachable, in-collision, not-tested and processing)
 * and will switch the mesh color according to these.
 */
class GripperMarker : public FrameMarker
{
public:
  typedef enum
  {
    NOT_TESTED, PROCESSING, REACHABLE, NOT_REACHABLE, IN_COLLISION
  } GripperMarkerState;

  GripperMarker(): display_gripper_mesh_(true) {}

  /** Constructor
   * @param kinematic_state the kinematic state of the robot
   * @param parent_node the Ogre node that will hold this interactive marker
   * @param context the display context
   * @param name the name for the interactive marker
   * @param frame_id the frame_id the interactive marker is given relative to
   * @param eef the robot_interaction end effector description
   * @param pose the desired pose of the coordinate system
   * @param scale the scale of the coordinate system
   * @param state the default state of the GripperMarker
   * @param is_selected whether this frame marker is selected or not by default. When selected, controls are displayed
   * @param visible_x, visible_y, visible_z define the visibility of each axis
   */
  GripperMarker(const kinematic_state::KinematicStatePtr& kinematic_state, Ogre::SceneNode *parent_node, rviz::DisplayContext *context, const std::string &name,
                const std::string &frame_id, const robot_interaction::RobotInteraction::EndEffector &eef, const geometry_msgs::Pose &pose, double scale,
                const GripperMarkerState &state, bool is_selected = false, bool visible_x = true, bool visible_y = true, bool visible_z = true);

  virtual void select(bool display_gripper_mesh = true);
  virtual void setState(const GripperMarkerState &state)
  {
    if (state != state_)
    {
      state_ = state;
      const float *color = stateToColor(state);
      setColor(color[0], color[1], color[2], color[3]);
    }
  }

protected:
  static const float GOAL_NOT_TESTED_COLOR[4];
  static const float GOAL_PROCESSING_COLOR[4];
  static const float GOAL_NOT_REACHABLE_COLOR[4];
  static const float GOAL_REACHABLE_COLOR[4];
  static const float GOAL_COLLISION_COLOR[4];

  virtual void buildFrom(const std::string &name, const std::string &frame_id, const geometry_msgs::Pose &pose, double scale, const std_msgs::ColorRGBA &color);

  const float *stateToColor(const GripperMarkerState &state);

  const kinematic_state::KinematicStatePtr kinematic_state_;
  robot_interaction::RobotInteraction::EndEffector eef_;

  bool display_gripper_mesh_;
  GripperMarkerState state_;
};

} //namespace
#endif
