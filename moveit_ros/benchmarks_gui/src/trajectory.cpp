/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Mario Prats */

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

Trajectory::Trajectory(const robot_state::RobotState& robot_state, Ogre::SceneNode *parent_node, rviz::DisplayContext *context, const std::string &name,
                       const std::string &frame_id, const robot_interaction::RobotInteraction::EndEffector &eef, const geometry_msgs::Pose &pose, double scale,
                       const GripperMarker::GripperMarkerState &state, unsigned int nwaypoints, bool is_selected, bool visible_x, bool visible_y, bool visible_z):
                       dragging_(false), nwaypoints_(nwaypoints), control_marker_mode_(CONTROL_MARKER_FIXED)
{
  createControlMarker(robot_state, parent_node, context, name, frame_id, eef, pose, scale, state, is_selected, visible_x, visible_y, visible_z);
  createHandMarker();
}

void Trajectory::createControlMarker(const robot_state::RobotState& robot_state, Ogre::SceneNode *parent_node, rviz::DisplayContext *context, const std::string &name,
                       const std::string &frame_id, const robot_interaction::RobotInteraction::EndEffector &eef, const geometry_msgs::Pose &pose, double scale,
                       const GripperMarker::GripperMarkerState &state, bool is_selected, bool visible_x, bool visible_y, bool visible_z)
{
  GripperMarkerPtr control( new GripperMarker(robot_state, parent_node, context, name, frame_id, eef, pose, scale, state));
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
    start_marker->setColor(0.0, 0.9, 0.0, 1.0);
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
    end_marker->setColor(0.0, 0.9, 0.0, 0.5);
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

void Trajectory::rebuildWayPointMarkers()
{
  waypoint_markers.clear();
  if (start_marker && end_marker)
  {
    Eigen::Vector3d start_t(control_marker_start_pose.translation());
    Eigen::Vector3d end_t(control_marker_end_pose.translation());

    Eigen::Quaterniond start_r(control_marker_start_pose.rotation());
    Eigen::Quaterniond end_r(control_marker_end_pose.rotation());

    Eigen::Affine3d wMh;
    hand_marker->getPose(wMh);

    Eigen::Affine3d wMt;
    control_marker->getPose(wMt);

    Eigen::Affine3d tMh = wMt.inverse() * wMh;

    unsigned int nfragments = nwaypoints_ + 1;
    for (std::size_t i = 0; i <= nfragments; ++i)
    {
      Eigen::Vector3d waypoint_t = ((nfragments - i) * start_t + i * end_t) / nfragments;
      Eigen::Quaterniond waypoint_r = start_r.slerp( (double)i / (double)nfragments, end_r );
      Eigen::Affine3d wMwpt(waypoint_r);
      wMwpt.translation() = waypoint_t;

      Eigen::Affine3d wMwph = wMwpt * tMh;

      GripperMarkerPtr waypoint(new GripperMarker(*hand_marker));
      std::stringstream name;
      name << hand_marker->getName() << "_wp" << i;
      waypoint->setName(name.str());
      waypoint->unselect(true);
      waypoint->setColor(0.0, 0.9, 0.0, 1 - (double)i / (double)nfragments);
      Eigen::Quaterniond rotation(wMwph.rotation());
      waypoint->imarker->setPose(Ogre::Vector3(wMwph(0,3), wMwph(1,3), wMwph(2,3)),
                                 Ogre::Quaternion(rotation.w(), rotation.x(), rotation.y(), rotation.z()), "");
      waypoint_markers.push_back(waypoint);
    }
  }
}

void Trajectory::trajectoryMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback)
{
  if (feedback.event_type == feedback.MENU_SELECT)
  {
    if (feedback.menu_entry_id == TRAJECTORY_SET_START_POSE)
    {
      if (control_marker_mode_ == CONTROL_MARKER_FLOATING)
      {
        //If the control marker is not fixed, fix it
        fixControlFrame();
      }

      control_marker->getPose(control_marker_start_pose);

      //Create start marker
      JobProcessing::addMainLoopJob(boost::bind(&benchmark_tool::Trajectory::createStartMarker, this));
      JobProcessing::addMainLoopJob(boost::bind(&benchmark_tool::Trajectory::rebuildWayPointMarkers, this));
    }
    else if (feedback.menu_entry_id == TRAJECTORY_SET_END_POSE)
    {
      if (control_marker_mode_ == CONTROL_MARKER_FLOATING)
      {
        //If the control marker is not fixed, fix it
        fixControlFrame();
      }

      control_marker->getPose(control_marker_end_pose);

      //Create end marker
      JobProcessing::addMainLoopJob(boost::bind(&benchmark_tool::Trajectory::createEndMarker, this));
      JobProcessing::addMainLoopJob(boost::bind(&benchmark_tool::Trajectory::rebuildWayPointMarkers, this));
    }
    else if (feedback.menu_entry_id == TRAJECTORY_EDIT_CONTROL_FRAME)
    {
      editControlFrame();
    }
    else if (feedback.menu_entry_id == TRAJECTORY_FIX_CONTROL_FRAME)
    {
      fixControlFrame();
    }
  }
  else if (feedback.event_type == feedback.MOUSE_DOWN && control_marker_mode_ == CONTROL_MARKER_FIXED)
  {
    //Store current hand pose
    hand_marker->getPose(hand_marker_start_pose);
    control_marker->getPose(control_marker_drag_start_pose);

    dragging_=true;
  }
  else if (feedback.event_type == feedback.POSE_UPDATE && dragging_ && control_marker_mode_ == CONTROL_MARKER_FIXED)
  {
    //Compute displacement from stored pose, and apply to the rest of selected markers
    Eigen::Affine3d current_pose_eigen;
    tf::poseMsgToEigen(feedback.pose, current_pose_eigen);

    Eigen::Affine3d current_wrt_initial = control_marker_drag_start_pose.inverse() * current_pose_eigen;

    visualization_msgs::InteractiveMarkerPose impose;
    Eigen::Affine3d newpose = control_marker_drag_start_pose * current_wrt_initial * control_marker_drag_start_pose.inverse() * hand_marker_start_pose;
    tf::poseEigenToMsg(newpose, impose.pose);

    hand_marker->imarker->setPose(Ogre::Vector3(impose.pose.position.x, impose.pose.position.y, impose.pose.position.z),
                                  Ogre::Quaternion(impose.pose.orientation.w, impose.pose.orientation.x, impose.pose.orientation.y, impose.pose.orientation.z), "");
  }
  else if (feedback.event_type == feedback.MOUSE_UP)
  {
    dragging_ = false;
  }
}

void Trajectory::editControlFrame()
{
  start_marker.reset();
  end_marker.reset();
  waypoint_markers.clear();

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

void Trajectory::fixControlFrame()
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
