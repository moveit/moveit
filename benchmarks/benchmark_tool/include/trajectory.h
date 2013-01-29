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
 *
 * Author: Mario Prats
 */

#ifndef BT_TRAJECTORY
#define BT_TRAJECTORY

#include <frame_marker.h>

namespace benchmark_tool
{

/* Implements functionality to create and manage straight and arc trajectories with interactive markers
 */
class Trajectory: public QObject
{
  Q_OBJECT

public Q_SLOTS:
  void trajectoryMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);
  void handMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);
  void startMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);
  void endMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);

public:
  static const int TRAJECTORY_SET_START_POSE = 1;
  static const int TRAJECTORY_SET_END_POSE = 2;
  static const int TRAJECTORY_EDIT_CONTROL_FRAME = 3;
  static const int TRAJECTORY_FIX_CONTROL_FRAME = 4;

  static const std::string TRAJECTORY_SET_START_POSE_STRING;
  static const std::string TRAJECTORY_SET_END_POSE_STRING;
  static const std::string TRAJECTORY_EDIT_CONTROL_FRAME_STRING;
  static const std::string TRAJECTORY_FIX_CONTROL_FRAME_STRING;

  Trajectory(const kinematic_state::KinematicState& kinematic_state, Ogre::SceneNode *parent_node, rviz::DisplayContext *context, const std::string &name,
             const std::string &frame_id, const robot_interaction::RobotInteraction::EndEffector &eef, const geometry_msgs::Pose &pose, double scale,
             const GripperMarker::GripperMarkerState &state, bool is_selected = true, bool visible_x = true, bool visible_y = true, bool visible_z = true);

  void setControlMarker(const GripperMarkerPtr control_marker);

  GripperMarkerPtr control_marker;
  GripperMarkerPtr hand_marker;
  GripperMarkerPtr start_marker;
  GripperMarkerPtr end_marker;

  void hide()
  {
    if (control_marker)
      control_marker->hide();
    if (hand_marker)
      hand_marker->hide();
    if (start_marker)
      start_marker->hide();
    if (end_marker)
      end_marker->hide();
  }

  void show(Ogre::SceneNode *scene_node, rviz::DisplayContext *context)
  {
    if (control_marker)
      control_marker->show(scene_node, context);
    if (hand_marker)
      hand_marker->show(scene_node, context);
    if (start_marker)
      start_marker->show(scene_node, context);
    if (end_marker)
      end_marker->show(scene_node, context);
  }

  ~Trajectory() {}

protected:
  void createControlMarker(const kinematic_state::KinematicState& kinematic_state, Ogre::SceneNode *parent_node, rviz::DisplayContext *context, const std::string &name,
                         const std::string &frame_id, const robot_interaction::RobotInteraction::EndEffector &eef, const geometry_msgs::Pose &pose, double scale,
                         const GripperMarker::GripperMarkerState &state, bool is_selected = true, bool visible_x = true, bool visible_y = true, bool visible_z = true);

  void createHandMarker();
  void createStartMarker();
  void createEndMarker();

  void connectControlMarker();
  void connectHandMarker();
  void connectStartMarker();
  void connectEndMarker();

private:
  Eigen::Affine3d hand_marker_start_pose;
  Eigen::Affine3d control_marker_start_pose;

  bool dragging_;

  typedef enum {CONTROL_MARKER_FLOATING, CONTROL_MARKER_FIXED} ControlMarkerModeType;
  ControlMarkerModeType control_marker_mode_;
};

typedef boost::shared_ptr<Trajectory> TrajectoryPtr;

}

#endif
