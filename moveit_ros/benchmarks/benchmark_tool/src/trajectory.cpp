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

#include <trajectory.h>
#include <job_processing.h>

#include <eigen_conversions/eigen_msg.h>

#include <boost/math/constants/constants.hpp>

#include <QWidget>

namespace benchmark_tool
{

const std::string Trajectory::TRAJECTORY_SET_START_POSE_STRING = "Set start pose";
const std::string Trajectory::TRAJECTORY_SET_END_POSE_STRING = "Set end pose";
const std::string Trajectory::TRAJECTORY_EDIT_CONTROL_FRAME_STRING = "Edit control frame";
const std::string Trajectory::TRAJECTORY_FIX_CONTROL_FRAME_STRING = "Fix control frame";

Trajectory::Trajectory(const kinematic_state::KinematicState& kinematic_state, Ogre::SceneNode *parent_node, rviz::DisplayContext *context, const std::string &name,
                       const std::string &frame_id, const robot_interaction::RobotInteraction::EndEffector &eef, const geometry_msgs::Pose &pose, double scale,
                       const GripperMarker::GripperMarkerState &state, bool is_selected, bool visible_x, bool visible_y, bool visible_z):
                       dragging_(false), control_marker_mode_(CONTROL_MARKER_FIXED)
{
  createControlMarker(kinematic_state, parent_node, context, name, frame_id, eef, pose, scale, state, is_selected, visible_x, visible_y, visible_z);
  createHandMarker();
}

void Trajectory::createControlMarker(const kinematic_state::KinematicState& kinematic_state, Ogre::SceneNode *parent_node, rviz::DisplayContext *context, const std::string &name,
                       const std::string &frame_id, const robot_interaction::RobotInteraction::EndEffector &eef, const geometry_msgs::Pose &pose, double scale,
                       const GripperMarker::GripperMarkerState &state, bool is_selected, bool visible_x, bool visible_y, bool visible_z)
{
  GripperMarkerPtr control( new GripperMarker(kinematic_state, parent_node, context, name, frame_id, eef, pose, scale, state));
  control->select(true);
  std::vector<visualization_msgs::MenuEntry> menu_entries;
  visualization_msgs::MenuEntry m;
  m.id = TRAJECTORY_SET_START_POSE;
  m.command_type = m.FEEDBACK;
  m.parent_id = 0;
  m.title = TRAJECTORY_SET_START_POSE_STRING;
  menu_entries.push_back(m);
  m.id = TRAJECTORY_SET_END_POSE;
  m.command_type = m.FEEDBACK;
  m.parent_id = 0;
  m.title = TRAJECTORY_SET_END_POSE_STRING;
  menu_entries.push_back(m);
  m.id = TRAJECTORY_EDIT_CONTROL_FRAME;
  m.command_type = m.FEEDBACK;
  m.parent_id = 0;
  m.title = TRAJECTORY_EDIT_CONTROL_FRAME_STRING;
  menu_entries.push_back(m);
  control->setMenu(menu_entries);
  control->select(false);

  control_marker = control;
  connectControlMarker();
}

void Trajectory::createHandMarker()
{
  if (control_marker)
  {
    hand_marker = GripperMarkerPtr(new GripperMarker(*control_marker));
    hand_marker->unselect(true);
    connectHandMarker();
  }
  else
  {
    ROS_ERROR("Main trajectory marker does not exist");
  }
}

void Trajectory::createStartMarker()
{
  if (control_marker)
  {
    start_marker = GripperMarkerPtr(new GripperMarker(*hand_marker));
    start_marker->unselect(true);
    start_marker->setColor(0.0, 0.9, 0.0, 0.9);
    start_marker->showDescription("Start");
    connectStartMarker();
  }
  else
  {
    ROS_ERROR("Main trajectory marker does not exist");
  }
}

void Trajectory::createEndMarker()
{
  if (control_marker)
  {
    end_marker = GripperMarkerPtr(new GripperMarker(*hand_marker));
    end_marker->unselect(true);
    end_marker->setColor(0.0, 0.9, 0.0, 0.4);
    end_marker->showDescription("End");
    connectEndMarker();
  }
  else
  {
    ROS_ERROR("Main trajectory marker does not exist");
  }
}

void Trajectory::connectControlMarker()
{
  control_marker->connect(this, SLOT( trajectoryMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &)));
}

void Trajectory::connectHandMarker()
{
  hand_marker->connect( this, SLOT( handMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &) ));
}

void Trajectory::connectStartMarker()
{
  start_marker->connect(this, SLOT( startMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &) ));
}
void Trajectory::connectEndMarker()
{
  end_marker->connect(this, SLOT( endMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &) ));
}

void Trajectory::trajectoryMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback)
{
  if (feedback.event_type == feedback.MENU_SELECT)
  {
    if (feedback.menu_entry_id == TRAJECTORY_SET_START_POSE)
    {
      //Create start marker
      JobProcessing::addMainLoopJob(boost::bind(&benchmark_tool::Trajectory::createStartMarker, this));
    }
    else if (feedback.menu_entry_id == TRAJECTORY_SET_END_POSE)
    {
      //Create end marker
      JobProcessing::addMainLoopJob(boost::bind(&benchmark_tool::Trajectory::createEndMarker, this));
    }
    else if (feedback.menu_entry_id == TRAJECTORY_EDIT_CONTROL_FRAME)
    {
      control_marker_mode_ = CONTROL_MARKER_FLOATING;

      std::vector<visualization_msgs::MenuEntry> menu_entries;
      visualization_msgs::MenuEntry m;
      m.id = TRAJECTORY_SET_START_POSE;
      m.command_type = m.FEEDBACK;
      m.parent_id = 0;
      m.title = TRAJECTORY_SET_START_POSE_STRING;
      menu_entries.push_back(m);
      m.id = TRAJECTORY_SET_END_POSE;
      m.command_type = m.FEEDBACK;
      m.parent_id = 0;
      m.title = TRAJECTORY_SET_END_POSE_STRING;
      menu_entries.push_back(m);
      m.id = TRAJECTORY_FIX_CONTROL_FRAME;
      m.command_type = m.FEEDBACK;
      m.parent_id = 0;
      m.title = TRAJECTORY_FIX_CONTROL_FRAME_STRING;
      menu_entries.push_back(m);

      JobProcessing::addMainLoopJob(boost::bind(&benchmark_tool::GripperMarker::setMenu, control_marker, menu_entries));
      JobProcessing::addMainLoopJob(boost::bind(&benchmark_tool::Trajectory::connectControlMarker, this));
    }
    else if (feedback.menu_entry_id == TRAJECTORY_FIX_CONTROL_FRAME)
    {
      control_marker_mode_ = CONTROL_MARKER_FIXED;

      std::vector<visualization_msgs::MenuEntry> menu_entries;
      visualization_msgs::MenuEntry m;
      m.id = TRAJECTORY_SET_START_POSE;
      m.command_type = m.FEEDBACK;
      m.parent_id = 0;
      m.title = TRAJECTORY_SET_START_POSE_STRING;
      menu_entries.push_back(m);
      m.id = TRAJECTORY_SET_END_POSE;
      m.command_type = m.FEEDBACK;
      m.parent_id = 0;
      m.title = TRAJECTORY_SET_END_POSE_STRING;
      menu_entries.push_back(m);
      m.id = TRAJECTORY_EDIT_CONTROL_FRAME;
      m.command_type = m.FEEDBACK;
      m.parent_id = 0;
      m.title = TRAJECTORY_EDIT_CONTROL_FRAME_STRING;
      menu_entries.push_back(m);

      JobProcessing::addMainLoopJob(boost::bind(&benchmark_tool::GripperMarker::setMenu, control_marker, menu_entries));
      JobProcessing::addMainLoopJob(boost::bind(&benchmark_tool::Trajectory::connectControlMarker, this));
    }
  }
  else if (feedback.event_type == feedback.MOUSE_DOWN && control_marker_mode_ == CONTROL_MARKER_FIXED)
  {
    //Store current hand pose
    hand_marker_start_pose = Eigen::Affine3d(Eigen::Quaterniond(hand_marker->imarker->getOrientation().w, hand_marker->imarker->getOrientation().x,
                                              hand_marker->imarker->getOrientation().y, hand_marker->imarker->getOrientation().z));
    hand_marker_start_pose(0,3) = hand_marker->imarker->getPosition().x;
    hand_marker_start_pose(1,3) = hand_marker->imarker->getPosition().y;
    hand_marker_start_pose(2,3) = hand_marker->imarker->getPosition().z;

    control_marker_start_pose = Eigen::Affine3d(Eigen::Quaterniond(control_marker->imarker->getOrientation().w, control_marker->imarker->getOrientation().x,
                                                 control_marker->imarker->getOrientation().y, control_marker->imarker->getOrientation().z));
    control_marker_start_pose(0,3) = control_marker->imarker->getPosition().x;
    control_marker_start_pose(1,3) = control_marker->imarker->getPosition().y;
    control_marker_start_pose(2,3) = control_marker->imarker->getPosition().z;
    dragging_=true;
  }
  else if (feedback.event_type == feedback.POSE_UPDATE && dragging_ && control_marker_mode_ == CONTROL_MARKER_FIXED)
  {
    //Compute displacement from stored pose, and apply to the rest of selected markers
    Eigen::Affine3d current_pose_eigen;
    tf::poseMsgToEigen(feedback.pose, current_pose_eigen);

    Eigen::Affine3d current_wrt_initial = control_marker_start_pose.inverse() * current_pose_eigen;

    visualization_msgs::InteractiveMarkerPose impose;
    Eigen::Affine3d newpose = control_marker_start_pose * current_wrt_initial * control_marker_start_pose.inverse() * hand_marker_start_pose;
    tf::poseEigenToMsg(newpose, impose.pose);

    hand_marker->imarker->setPose(Ogre::Vector3(impose.pose.position.x, impose.pose.position.y, impose.pose.position.z),
                                  Ogre::Quaternion(impose.pose.orientation.w, impose.pose.orientation.x, impose.pose.orientation.y, impose.pose.orientation.z), "");
  }
  else if (feedback.event_type == feedback.MOUSE_UP)
  {
    dragging_ = false;
  }
}

void Trajectory::startMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback)
{
}

void Trajectory::endMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback)
{
}

void Trajectory::handMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback)
{
}

} //namespace
