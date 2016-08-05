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

#include <frame_marker.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>

#include <interactive_markers/tools.h>

#include <eigen_conversions/eigen_msg.h>

#include <boost/math/constants/constants.hpp>

namespace benchmark_tool
{

const float GripperMarker::GOAL_NOT_TESTED_COLOR[4] = { 0.75, 0.75, 0.75, 1.0};
const float GripperMarker::GOAL_PROCESSING_COLOR[4] = { 0.9, 0.9, 0.9, 1.0};
const float GripperMarker::GOAL_NOT_REACHABLE_COLOR[4] = { 1.0, 0.0, 0.0, 1.0};
const float GripperMarker::GOAL_REACHABLE_COLOR[4] = { 0.0, 1.0, 0.0, 1.0};
const float GripperMarker::GOAL_COLLISION_COLOR[4] = { 1.0, 1.0, 0.0, 1.0};

FrameMarker::FrameMarker(Ogre::SceneNode *parent_node, rviz::DisplayContext *context, const std::string &name,
            const std::string &frame_id, const geometry_msgs::Pose &pose, double scale, const std_msgs::ColorRGBA &color,
            bool is_selected, bool visible_x, bool visible_y, bool visible_z):
              parent_node_(parent_node),
              context_(context),
              selected_(is_selected),
              visible_x_(visible_x),
              visible_y_(visible_y),
              visible_z_(visible_z),
              color_(color),
              receiver_(NULL),
              receiver_method_(NULL)
{
  buildFrom(name, frame_id, pose, scale, color);
}

FrameMarker::FrameMarker(Ogre::SceneNode *parent_node, rviz::DisplayContext *context, const std::string &name,
            const std::string &frame_id, const geometry_msgs::Pose &pose, double scale, const float color[4],
            bool is_selected, bool visible_x, bool visible_y, bool visible_z):
              parent_node_(parent_node),
              context_(context),
              selected_(is_selected),
              visible_x_(visible_x),
              visible_y_(visible_y),
              visible_z_(visible_z),
              receiver_(NULL),
              receiver_method_(NULL)
{
  color_.r = color[0];
  color_.g = color[1];
  color_.b = color[2];
  color_.a = color[3];
  buildFrom(name, frame_id, pose, scale, color_);
}

void FrameMarker::hide(void)
{
  if ( isVisible() )
  {
    position_ = imarker->getPosition();
    orientation_ = imarker->getOrientation();
    imarker.reset();
  }
}

void FrameMarker::show(Ogre::SceneNode *scene_node, rviz::DisplayContext *context)
{
  if ( ! isVisible() )
  {
    rebuild();
  }
}

void FrameMarker::showDescription(const std::string &description)
{
  imarker_msg.description = description;

  if (isVisible())
  {
    Ogre::Vector3 position = imarker->getPosition();
    Ogre::Quaternion orientation = imarker->getOrientation();
    updateMarker();
    imarker->setPose(position, orientation, "");
  }
  imarker->setShowDescription(true);
}

void FrameMarker::hideDescription()
{
  imarker->setShowDescription(false);
}

void FrameMarker::getPosition(geometry_msgs::Point &position)
{
  if (imarker)
  {
    position.x = imarker->getPosition().x;
    position.y = imarker->getPosition().y;
    position.z = imarker->getPosition().z;
  }
  else
  {
    position.x = position_.x;
    position.y = position_.y;
    position.z = position_.z;
  }
}

void FrameMarker::getOrientation(geometry_msgs::Quaternion &orientation)
{
  if (imarker)
  {
    orientation.x = imarker->getOrientation().x;
    orientation.y = imarker->getOrientation().y;
    orientation.z = imarker->getOrientation().z;
    orientation.w = imarker->getOrientation().w;
  }
  else
  {
    orientation.x = orientation_.x;
    orientation.y = orientation_.y;
    orientation.z = orientation_.z;
    orientation.w = orientation_.w;
  }
}

void FrameMarker::getPose(Eigen::Affine3d &pose)
{
  pose = Eigen::Affine3d(Eigen::Quaterniond(imarker->getOrientation().w, imarker->getOrientation().x,
                                            imarker->getOrientation().y, imarker->getOrientation().z));
  pose.translation() = Eigen::Vector3d(imarker->getPosition().x,
                                       imarker->getPosition().y,
                                       imarker->getPosition().z);
}

void FrameMarker::setPose(Eigen::Affine3d &pose)
{
  Eigen::Quaterniond q(pose.rotation());
  imarker->setPose(Ogre::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z()),
                   Ogre::Quaternion(q.w(), q.x(), q.y(), q.z()), "");
}

void FrameMarker::setColor(float r, float g, float b, float a)
{
  //Update marker color
  color_.r = r;
  color_.g = g;
  color_.b = b;
  color_.a = a;

  for (unsigned int c = 0; c < imarker_msg.controls.size(); ++c)
  {
    for (unsigned int m = 0; m < imarker_msg.controls[c].markers.size(); ++m)
    {
      if (imarker_msg.controls[c].markers[m].type == visualization_msgs::Marker::MESH_RESOURCE ||
          imarker_msg.controls[c].markers[m].type == visualization_msgs::Marker::SPHERE)
      {
        imarker_msg.controls[c].markers[m].color.r = r;
        imarker_msg.controls[c].markers[m].color.g = g;
        imarker_msg.controls[c].markers[m].color.b = b;
        imarker_msg.controls[c].markers[m].color.a = a;
      }
    }
  }

  if (isVisible())
  {
    Ogre::Vector3 position = imarker->getPosition();
    Ogre::Quaternion orientation = imarker->getOrientation();
    updateMarker();
    imarker->setPose(position, orientation, "");
  }
}

void FrameMarker::select(void)
{
  selected_ = true;
  rebuild();
}

void FrameMarker::unselect(void)
{
  selected_ = false;
  rebuild();
}

void FrameMarker::rebuild()
{
  geometry_msgs::Pose pose;
  getPosition(pose.position);
  getOrientation(pose.orientation);
  buildFrom(imarker_msg.name, imarker_msg.header.frame_id, pose, imarker_msg.scale, color_);
}

void FrameMarker::buildFrom(const std::string &name, const std::string &frame_id, const geometry_msgs::Pose &pose, double scale, const std_msgs::ColorRGBA &color)
{
  color_ = color;

  visualization_msgs::InteractiveMarker int_marker;
  if (isSelected())
  {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.pose = pose;
    int_marker = robot_interaction::make6DOFMarker(name, pose_stamped, scale);
  }
  else
  {
    int_marker.scale = scale;
    int_marker.name = name;
    int_marker.pose = pose;
  }
  int_marker.header.frame_id = frame_id;
  int_marker.header.stamp = ros::Time::now();

  int_marker.menu_entries = menu_entries_;

  visualization_msgs::InteractiveMarkerControl m_control;
  m_control.always_visible = true;
  m_control.interaction_mode = m_control.BUTTON;

  //Display a frame marker with an sphere in the origin
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::SPHERE;
  m.scale.x = 0.1 * scale;
  m.scale.y = 0.1 * scale;
  m.scale.z = 0.1 * scale;
  m.ns = "goal_pose_marker";
  m.action = visualization_msgs::Marker::ADD;
  m.color = color;
  m_control.markers.push_back(m);

  m.type = visualization_msgs::Marker::ARROW;
  m.scale.x = 0.3 * scale;
  m.scale.y = 0.1 * m.scale.x;
  m.scale.z = 0.1 * m.scale.x;
  m.ns = "goal_pose_marker";
  m.action = visualization_msgs::Marker::ADD;

  //X axis
  if (visible_x_)
  {
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.color.a = 1.0f;
    m_control.markers.push_back(m);
  }

  //Y axis
  if (visible_y_)
  {
    tf::Quaternion imq;
    imq = tf::createQuaternionFromRPY(0, 0, boost::math::constants::pi<double>() / 2.0);
    tf::quaternionTFToMsg(imq, m.pose.orientation);
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.color.a = 1.0f;
    m_control.markers.push_back(m);
  }

  //Z axis
  if (visible_z_) {
    tf::Quaternion imq;
    imq = tf::createQuaternionFromRPY(0, -boost::math::constants::pi<double>() / 2.0, 0);
    tf::quaternionTFToMsg(imq, m.pose.orientation);
    m.color.r = 0.0f;
    m.color.g = 0.0f;
    m.color.b = 1.0f;
    m.color.a = 1.0f;
    m_control.markers.push_back(m);
  }
  int_marker.controls.push_back(m_control);

  imarker.reset(new rviz::InteractiveMarker(parent_node_, context_ ));
  interactive_markers::autoComplete(int_marker);
  imarker->processMessage(int_marker);
  imarker->setShowVisualAids(true);
  imarker->setShowAxes(false);
  imarker->setShowDescription(false);

  //reconnect if it was previously connected
  if (receiver_ && receiver_method_)
  {
    connect(receiver_, receiver_method_);
  }

  imarker_msg = int_marker;
}


GripperMarker::GripperMarker(const robot_state::RobotState& robot_state, Ogre::SceneNode *parent_node, rviz::DisplayContext *context, const std::string &name,
                             const std::string &frame_id, const robot_interaction::RobotInteraction::EndEffector &eef, const geometry_msgs::Pose &pose, double scale,
                             const GripperMarkerState &state, bool is_selected, bool visible_x, bool visible_y, bool visible_z):
                             FrameMarker(parent_node, context, name, frame_id, pose, scale, stateToColor(state), is_selected, visible_x, visible_y, visible_z),
                             eef_(eef),
                             display_gripper_mesh_(false),
                             state_(state)
{
  robot_state_ = &robot_state;
}

void GripperMarker::select(bool display_gripper_mesh)
{
  display_gripper_mesh_ = display_gripper_mesh;
  FrameMarker::select();
}

void GripperMarker::unselect(bool display_gripper_mesh)
{
  display_gripper_mesh_ = display_gripper_mesh;
  FrameMarker::unselect();
}

void GripperMarker::buildFrom(const std::string &name, const std::string &frame_id, const geometry_msgs::Pose &pose, double scale, const std_msgs::ColorRGBA &color)
{
  color_ = color;

  visualization_msgs::InteractiveMarker int_marker;
  if (isSelected())
  {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    int_marker = robot_interaction::make6DOFMarker(name, pose_stamped, scale);
  }
  else
  {
    int_marker.scale = scale;
    int_marker.name = name;
    int_marker.pose = pose;
  }
  int_marker.header.frame_id = frame_id;
  int_marker.header.stamp = ros::Time::now();

  int_marker.menu_entries = menu_entries_;

  visualization_msgs::InteractiveMarkerControl m_control;
  m_control.always_visible = true;
  m_control.interaction_mode = m_control.BUTTON;

  if (display_gripper_mesh_)
  {
    //If selected and gripper_mesh enabled, display the actual end effector mesh
    const robot_state::JointModelGroup *joint_model_group = robot_state_->getJointModelGroup(eef_.eef_group);
    const std::vector<std::string> &link_names = joint_model_group->getLinkModelNames();

    std_msgs::ColorRGBA marker_color;
    marker_color = color;
    visualization_msgs::MarkerArray marker_array;
    robot_state_->getRobotMarkers(marker_array, link_names, marker_color, "goal_pose_marker", ros::Duration());

    for (std::size_t i = 0 ; i < marker_array.markers.size() ; ++i)
    {
      marker_array.markers[i].header = int_marker.header;
      m_control.markers.push_back(marker_array.markers[i]);
    }

    Eigen::Affine3d tip_pose = robot_state_->getGlobalLinkTransform(eef_.parent_link);
    tf::poseEigenToMsg(tip_pose, int_marker.pose);
  }
  else
  {
    //Display a frame marker with an sphere in the origin
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::SPHERE;
    m.scale.x = 0.1 * scale;
    m.scale.y = 0.1 * scale;
    m.scale.z = 0.1 * scale;
    m.ns = "goal_pose_marker";
    m.action = visualization_msgs::Marker::ADD;
    m.color = color;
    m_control.markers.push_back(m);

    m.type = visualization_msgs::Marker::ARROW;
    m.scale.x = 0.3 * scale;
    m.scale.y = 0.1 * m.scale.x;
    m.scale.z = 0.1 * m.scale.x;
    m.ns = "goal_pose_marker";
    m.action = visualization_msgs::Marker::ADD;

    //X axis
    if (visible_x_)
    {
      m.color.r = 1.0f;
      m.color.g = 0.0f;
      m.color.b = 0.0f;
      m.color.a = 1.0f;
      m_control.markers.push_back(m);
    }

    //Y axis
    if (visible_y_)
    {
      tf::Quaternion imq;
      imq = tf::createQuaternionFromRPY(0, 0, boost::math::constants::pi<double>() / 2.0);
      tf::quaternionTFToMsg(imq, m.pose.orientation);
      m.color.r = 0.0f;
      m.color.g = 1.0f;
      m.color.b = 0.0f;
      m.color.a = 1.0f;
      m_control.markers.push_back(m);
    }

    //Z axis
    if (visible_z_) {
      tf::Quaternion imq;
      imq = tf::createQuaternionFromRPY(0, -boost::math::constants::pi<double>() / 2.0, 0);
      tf::quaternionTFToMsg(imq, m.pose.orientation);
      m.color.r = 0.0f;
      m.color.g = 0.0f;
      m.color.b = 1.0f;
      m.color.a = 1.0f;
      m_control.markers.push_back(m);
    }
  }

  int_marker.controls.push_back(m_control);

  imarker.reset(new rviz::InteractiveMarker(parent_node_, context_ ));
  interactive_markers::autoComplete(int_marker);
  imarker->processMessage(int_marker);
  imarker->setShowAxes(false);
  imarker->setShowVisualAids(true);
  imarker->setShowDescription(false);
  imarker->setPose(Ogre::Vector3(pose.position.x, pose.position.y, pose.position.z),
                    Ogre::Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z), "");

  //reconnect if it was previously connected
  if (receiver_ && receiver_method_)
  {
    connect(receiver_, receiver_method_);
  }

  imarker_msg = int_marker;
}

const float *GripperMarker::stateToColor(const GripperMarkerState &state)
{
  const float *color = 0;
  if (state == NOT_TESTED)
    color = GOAL_NOT_TESTED_COLOR;
  else if (state == PROCESSING)
    color = GOAL_PROCESSING_COLOR;
  else if (state == REACHABLE)
    color = GOAL_REACHABLE_COLOR;
  else if (state == NOT_REACHABLE)
    color = GOAL_NOT_REACHABLE_COLOR;
  else if (state == IN_COLLISION)
    color = GOAL_COLLISION_COLOR;
  else
    color = GOAL_NOT_TESTED_COLOR;

  return color;
}
}
