/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Mario Prats, Ioan Sucan */

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematic_state/conversions.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>

#include <interactive_markers/tools.h>

#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>

#include <eigen_conversions/eigen_msg.h>

#include <QMessageBox>
#include <QInputDialog>

#include "ui_motion_planning_rviz_plugin_frame.h"

#include <boost/math/constants/constants.hpp>

namespace moveit_rviz_plugin
{

static const float GOAL_NOT_TESTED_COLOR[4] = { 0.75, 0.75, 0.75, 1.0};
static const float GOAL_PROCESSING_COLOR[4] = { 0.9, 0.9, 0.9, 1.0};
static const float GOAL_NOT_REACHABLE_COLOR[4] = { 1.0, 0.0, 0.0, 1.0};
static const float GOAL_REACHABLE_COLOR[4] = { 0.0, 1.0, 0.0, 1.0};
static const float GOAL_COLLISION_COLOR[4] = { 1.0, 1.0, 0.0, 1.0};

MotionPlanningFrame::MsgMarkerPair MotionPlanningFrame::make6DOFEndEffectorMarker(const std::string& name,
                                                                                  const robot_interaction::RobotInteraction::EndEffector &eef,
                                                                                  const geometry_msgs::Pose &pose,
                                                                                  double scale,
                                                                                  bool selected)
{
  visualization_msgs::InteractiveMarker int_marker;
  geometry_msgs::PoseStamped tip_pose_msg;
  Eigen::Affine3d tip_pose = planning_display_->getQueryGoalState()->getLinkState(eef.parent_link)->getGlobalLinkTransform();
  tf::poseEigenToMsg(tip_pose, tip_pose_msg.pose);
  if (selected)
  {
    int_marker = robot_interaction::make6DOFMarker(name, tip_pose_msg, scale);
  }
  else
  {
    int_marker.scale = scale;
    int_marker.name = name;
    int_marker.pose = tip_pose_msg.pose;
  }
  int_marker.header.frame_id = planning_display_->getKinematicModel()->getModelFrame();

  visualization_msgs::InteractiveMarkerControl m_control;
  m_control.always_visible = true;
  m_control.interaction_mode = m_control.BUTTON;
  if (selected && ui_->goal_poses_list->selectedItems().size() == 1)
  {
    //If selected (and only one selected), display the actual end effector mesh
    const kinematic_state::JointStateGroup *joint_state_group = planning_display_->getQueryGoalState()->getJointStateGroup(eef.eef_group);
    const kinematic_state::KinematicState *kinematic_state = joint_state_group->getKinematicState();

    const kinematic_model::JointModelGroup *joint_model_group = joint_state_group->getJointModelGroup();
    const std::vector<std::string> &link_names = joint_model_group->getLinkModelNames();

    std_msgs::ColorRGBA marker_color;
    marker_color.r = GOAL_NOT_TESTED_COLOR[0];
    marker_color.g = GOAL_NOT_TESTED_COLOR[1];
    marker_color.b = GOAL_NOT_TESTED_COLOR[2];
    marker_color.a = GOAL_NOT_TESTED_COLOR[3];
    visualization_msgs::MarkerArray marker_array;
    kinematic_state->getRobotMarkers(marker_color, "goal_pose_marker", ros::Duration(), marker_array, link_names);

    for (std::size_t i = 0 ; i < marker_array.markers.size() ; ++i)
    {
      marker_array.markers[i].header = int_marker.header;
      m_control.markers.push_back(marker_array.markers[i]);
    }
  }
  else
  {
    //If not selected, display a frame marker with an sphere in the origin
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::SPHERE;
    m.scale.x = 0.1 * scale;
    m.scale.y = 0.1 * scale;
    m.scale.z = 0.1 * scale;
    m.ns = "goal_pose_marker";
    m.action = visualization_msgs::Marker::ADD;
    m.color.r = GOAL_NOT_TESTED_COLOR[0];
    m.color.g = GOAL_NOT_TESTED_COLOR[1];
    m.color.b = GOAL_NOT_TESTED_COLOR[2];
    m.color.a = GOAL_NOT_TESTED_COLOR[3];
    m_control.markers.push_back(m);

    m.type = visualization_msgs::Marker::ARROW;
    m.scale.x = 0.3 * scale;
    m.scale.y = 0.1 * m.scale.x;
    m.scale.z = 0.1 * m.scale.x;
    m.ns = "goal_pose_marker";
    m.action = visualization_msgs::Marker::ADD;

    if (ui_->show_x_checkbox->isChecked())
    {
      m.color.r = 1.0f;
      m.color.g = 0.0f;
      m.color.b = 0.0f;
      m.color.a = 1.0f;
      m_control.markers.push_back(m);
    }

    //Y axis
    if (ui_->show_y_checkbox->isChecked())
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
    if (ui_->show_z_checkbox->isChecked()) {
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

  rviz::InteractiveMarker* imarker = new rviz::InteractiveMarker(planning_display_->getSceneNode(), context_ );
  interactive_markers::autoComplete(int_marker);
  imarker->processMessage(int_marker);
  imarker->setShowAxes(false);
  imarker->setShowDescription(false);
  imarker->setPose(Ogre::Vector3(pose.position.x, pose.position.y, pose.position.z),
                   Ogre::Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z), "");

  return MsgMarkerPair(int_marker, boost::shared_ptr<rviz::InteractiveMarker>(imarker));
}

void MotionPlanningFrame::createGoalPoseButtonClicked(void)
{
  std::stringstream ss;

  {
    const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
    if ( ! ps || planning_display_->getRobotInteraction()->getActiveEndEffectors().empty() )
      return;
    else
      ss << ps->getName().c_str() << "_pose_" << std::setfill('0') << std::setw(4) << goal_poses_.size();
  }

  bool ok = false;
  QString text = QInputDialog::getText(this, tr("Choose a name"),
                                       tr("Goal pose name:"), QLineEdit::Normal,
                                       QString(ss.str().c_str()), &ok);

  std::string name;
  if (ok)
  {
    if ( ! text.isEmpty() )
    {
      name = text.toStdString();
      if (goal_poses_.find(name) != goal_poses_.end())
        QMessageBox::warning(this, "Name already exists", QString("The name '").append(name.c_str()).
                             append("' already exists. Not creating goal."));
      else
      {
        //Create the new goal pose at the current eef pose, and attach an interactive marker to it
        Eigen::Affine3d tip_pose = planning_display_->getQueryGoalState()->getLinkState(planning_display_->getRobotInteraction()->getActiveEndEffectors()[0].parent_link)->getGlobalLinkTransform();
        geometry_msgs::Pose marker_pose;
        tf::poseEigenToMsg(tip_pose, marker_pose);
        static const float marker_scale = 0.35;
        MsgMarkerPair imarker = make6DOFEndEffectorMarker(name, planning_display_->getRobotInteraction()->getActiveEndEffectors()[0], marker_pose, marker_scale, true);

        goal_poses_.insert(GoalPosePair(name,  GoalPoseMarker(imarker.second, imarker.first, true)));

        // Connect signals
        connect( imarker.second.get(), SIGNAL( userFeedback(visualization_msgs::InteractiveMarkerFeedback &)), this, SLOT( goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback &) ));

        //If connected to a databse, store the constraint
        if (constraints_storage_)
        {
          moveit_msgs::Constraints c;
          c.name = name;

          shape_msgs::SolidPrimitive sp;
          sp.type = sp.BOX;
          sp.dimensions.resize(3, std::numeric_limits<float>::epsilon() * 10.0);

          moveit_msgs::PositionConstraint pc;
          pc.constraint_region.primitives.push_back(sp);
          geometry_msgs::Pose posemsg;
          posemsg.position.x = imarker.second->getPosition().x;
          posemsg.position.y = imarker.second->getPosition().y;
          posemsg.position.z = imarker.second->getPosition().z;
          posemsg.orientation.x = 0.0;
          posemsg.orientation.y = 0.0;
          posemsg.orientation.z = 0.0;
          posemsg.orientation.w = 1.0;
          pc.constraint_region.primitive_poses.push_back(posemsg);
          pc.weight = 1.0;
          c.position_constraints.push_back(pc);

          moveit_msgs::OrientationConstraint oc;
          oc.orientation.x = imarker.second->getOrientation().x;
          oc.orientation.y = imarker.second->getOrientation().y;
          oc.orientation.z = imarker.second->getOrientation().z;
          oc.orientation.w = imarker.second->getOrientation().w;
          oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance =
            oc.absolute_z_axis_tolerance = std::numeric_limits<float>::epsilon() * 10.0;
          oc.weight = 1.0;
          c.orientation_constraints.push_back(oc);

          try
          {
            constraints_storage_->addConstraints(c);
          }
          catch (std::runtime_error &ex)
          {
            ROS_ERROR("Cannot save constraint on database: %s", ex.what());
          }
        }
      }
    }
    else
      QMessageBox::warning(this, "Goal not created", "Cannot use an empty name for a new goal pose.");
  }

  populateGoalPosesList();
}

void MotionPlanningFrame::removeSelectedGoalsButtonClicked(void)
{
  QList<QListWidgetItem*> found_items = ui_->goal_poses_list->selectedItems();
  for ( unsigned int i = 0 ; i < found_items.size() ; i++ )
  {
    goal_poses_.erase(found_items[i]->text().toStdString());
  }
  populateGoalPosesList();
}

void MotionPlanningFrame::removeAllGoalsButtonClicked(void)
{
  goal_poses_.clear();
  populateGoalPosesList();
}

void MotionPlanningFrame::loadGoalsFromDBButtonClicked(void)
{
  //Get all the constraints from the database, convert to goal pose markers
  if (constraints_storage_ && !planning_display_->getRobotInteraction()->getActiveEndEffectors().empty())
  {
    //First clear the current list
    removeAllGoalsButtonClicked();

    std::vector<std::string> names;
    try
    {
      constraints_storage_->getKnownConstraints(ui_->load_poses_filter_text->text().toStdString(), names);
    }
    catch (std::runtime_error &ex)
    {
      QMessageBox::warning(this, "Cannot query the database", QString("Wrongly formatted regular expression for goal poses: ").append(ex.what()));
      return;
    }

    for (unsigned int i = 0 ; i < names.size() ; i++)
    {
      //Create a goal pose marker
      moveit_warehouse::ConstraintsWithMetadata c;
      bool got_constraint = false;
      try
      {
        got_constraint = constraints_storage_->getConstraints(c, names[i]);
      }
      catch (std::runtime_error &ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      if (!got_constraint)
        continue;

      if ( c->position_constraints.size() > 0 && c->position_constraints[0].constraint_region.primitive_poses.size() > 0 && c->orientation_constraints.size() > 0 )
      {
        //Overwrite if exists. TODO: Ask the user before overwriting? copy the existing one with another name before?
        if ( goal_poses_.find(c->name) != goal_poses_.end() )
        {
          goal_poses_.erase(c->name);
        }
        geometry_msgs::Pose shape_pose;
        shape_pose.position = c->position_constraints[0].constraint_region.primitive_poses[0].position;
        shape_pose.orientation = c->orientation_constraints[0].orientation;

        static const float marker_scale = 0.35;
        MsgMarkerPair imarker = make6DOFEndEffectorMarker(c->name, planning_display_->getRobotInteraction()->getActiveEndEffectors()[0], shape_pose, marker_scale);

        goal_poses_.insert(GoalPosePair(c->name, GoalPoseMarker(imarker.second, imarker.first)));

        // Connect signals
        connect( imarker.second.get(), SIGNAL( userFeedback(visualization_msgs::InteractiveMarkerFeedback &)), this, SLOT( goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback &) ));
      }
    }
    populateGoalPosesList();
   }
  else
  {
    if (!constraints_storage_)
      QMessageBox::warning(this, "Warning", "Not connected to a database.");
  }
}

void MotionPlanningFrame::saveGoalsOnDBButtonClicked(void)
{
  if (constraints_storage_)
  {
    //Convert all goal pose markers into constraints and store them
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
    {
      moveit_msgs::Constraints c;
      c.name = it->first;

      shape_msgs::SolidPrimitive sp;
      sp.type = sp.BOX;
      sp.dimensions.resize(3, std::numeric_limits<float>::epsilon() * 10.0);

      moveit_msgs::PositionConstraint pc;
      pc.constraint_region.primitives.push_back(sp);
      geometry_msgs::Pose posemsg;
      it->second.getPosition(posemsg.position);
      posemsg.orientation.x = 0.0;
      posemsg.orientation.y = 0.0;
      posemsg.orientation.z = 0.0;
      posemsg.orientation.w = 1.0;
      pc.constraint_region.primitive_poses.push_back(posemsg);
      pc.weight = 1.0;
      c.position_constraints.push_back(pc);

      moveit_msgs::OrientationConstraint oc;
      it->second.getOrientation(oc.orientation);
      oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance =
        oc.absolute_z_axis_tolerance = std::numeric_limits<float>::epsilon() * 10.0;
      oc.weight = 1.0;
      c.orientation_constraints.push_back(oc);

      try
      {
        constraints_storage_->addConstraints(c);
      }
      catch (std::runtime_error &ex)
      {
        ROS_ERROR("Cannot save constraint: %s", ex.what());
      }
    }
  }
  else
  {
    QMessageBox::warning(this, "Warning", "Not connected to a database.");
  }
}

void MotionPlanningFrame::deleteGoalsOnDBButtonClicked(void)
{
  if (constraints_storage_)
  {
    //Warn the user
    QMessageBox msgBox;
    msgBox.setText("All the selected items will be removed from the database");
    msgBox.setInformativeText("Do you want to continue?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::No);
    int ret = msgBox.exec();

    switch (ret)
    {
      case QMessageBox::Yes:
      {
        //Go through the list of goal poses, and delete those selected
        QList<QListWidgetItem*> found_items = ui_->goal_poses_list->selectedItems();
        for ( std::size_t i = 0 ; i < found_items.size() ; i++ )
        {
          try
          {
            constraints_storage_->removeConstraints(found_items[i]->text().toStdString());
            removeSelectedGoalsButtonClicked();
          }
          catch (std::runtime_error &ex)
          {
            ROS_ERROR("%s", ex.what());
          }
        }
        break;
      }
    }
  }
}


void MotionPlanningFrame::loadStatesFromDBButtonClicked(void)
{
  //Get all the start states from the database
  if (robot_state_storage_)
  {
    //First clear the current list
    removeAllStatesButtonClicked();

    std::vector<std::string> names;
    try
    {
      robot_state_storage_->getKnownRobotStates(ui_->load_states_filter_text->text().toStdString(), names);
    }
    catch (std::runtime_error &ex)
    {
      QMessageBox::warning(this, "Cannot query the database", QString("Wrongly formatted regular expression for goal poses: ").append(ex.what()));
      return;
    }

    for ( unsigned int i = 0 ; i < names.size() ; i++ )
    {
      moveit_warehouse::RobotStateWithMetadata rs;
      bool got_state = false;
      try
      {
        got_state = robot_state_storage_->getRobotState(rs, names[i]);
      }
      catch(std::runtime_error &ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      if (!got_state)
        continue;

      //Overwrite if exists.
      if (start_states_.find(names[i]) != start_states_.end())
      {
        start_states_.erase(names[i]);
      }

      //Store the current start state
      start_states_.insert(StartStatePair(names[i],  StartState(*rs)));
    }
    populateStartStatesList();
  }
  else
  {
    QMessageBox::warning(this, "Warning", "Not connected to a database.");
  }
}

void MotionPlanningFrame::saveStatesOnDBButtonClicked(void)
{
  if (robot_state_storage_)
  {
    //Store all start states
    for (StartStateMap::iterator it = start_states_.begin(); it != start_states_.end(); ++it)
      try
      {
        robot_state_storage_->addRobotState(it->second.state_msg, it->first);
      }
      catch (std::runtime_error &ex)
      {
        ROS_ERROR("Cannot save robot state: %s", ex.what());
      }
  }
  else
  {
    QMessageBox::warning(this, "Warning", "Not connected to a database.");
  }
}

void MotionPlanningFrame::deleteStatesOnDBButtonClicked(void)
{
  if (robot_state_storage_)
  {
    //Warn the user
    QMessageBox msgBox;
    msgBox.setText("All the selected items will be removed from the database");
    msgBox.setInformativeText("Do you want to continue?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::No);
    int ret = msgBox.exec();

    switch (ret)
    {
      case QMessageBox::Yes:
      {
        for (unsigned int i = 0; i < ui_->start_states_list->count() ; ++i)
        {
          QListWidgetItem *item = ui_->start_states_list->item(i);
          if ( item->isSelected() )
          {
            try
            {
              robot_state_storage_->removeRobotState(item->text().toStdString());
              removeSelectedStatesButtonClicked();
            }
            catch (std::runtime_error &ex)
            {
              ROS_ERROR("%s", ex.what());
            }
          }
        }
        break;
      }
    }
  }
}

void MotionPlanningFrame::visibleAxisChanged(int state)
{
  if ( ! planning_display_->getRobotInteraction()->getActiveEndEffectors().empty() )
  {
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
    {
      if (it->second.isVisible())
      {
        Ogre::Vector3 position = it->second.imarker->getPosition();
        Ogre::Quaternion orientation = it->second.imarker->getOrientation();

        MsgMarkerPair marker_pair;
        marker_pair = make6DOFEndEffectorMarker(it->first,
                                                planning_display_->getRobotInteraction()->getActiveEndEffectors()[0],
                                                it->second.imarker_msg.pose, it->second.imarker_msg.scale, it->second.selected);

        connect( marker_pair.second.get(), SIGNAL( userFeedback(visualization_msgs::InteractiveMarkerFeedback &)), this, SLOT( goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback &) ));

        marker_pair.second->setPose(position, orientation, "");
        it->second.imarker_msg = marker_pair.first;
        it->second.imarker = marker_pair.second;
      }
    }
  }
}

void MotionPlanningFrame::populateGoalPosesList(void)
{
  ui_->goal_poses_list->clear();
  for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
  {
    QListWidgetItem *item = new QListWidgetItem(QString(it->first.c_str()));
    ui_->goal_poses_list->addItem(item);
    if (! it->second.isVisible())
    {
      item->setBackground(QBrush(Qt::Dense4Pattern));
    }
    else if (it->second.selected)
    {
      //If selected, highlight in the list
      item->setSelected(true);
    }
  }
}

void MotionPlanningFrame::GoalPoseMarker::hide(void)
{
  if (imarker)
  {
    position_ = imarker->getPosition();
    orientation_ = imarker->getOrientation();
  }
  imarker.reset();
}

void MotionPlanningFrame::GoalPoseMarker::show(MotionPlanningDisplay *pdisplay, rviz::DisplayContext *context)
{
  imarker.reset(new rviz::InteractiveMarker(pdisplay->getSceneNode(), context ));
  updateMarker();
  imarker->setShowAxes(false);
  imarker->setShowDescription(false);
  imarker->setPose(position_, orientation_, "");
}

void MotionPlanningFrame::GoalPoseMarker::getPosition(geometry_msgs::Point &position)
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
    position.y = position_.x;
    position.z = position_.z;
  }
}

void MotionPlanningFrame::GoalPoseMarker::getOrientation(geometry_msgs::Quaternion &orientation)
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

void MotionPlanningFrame::switchGoalVisibilityButtonClicked(void)
{
  QList<QListWidgetItem*> selection = ui_->goal_poses_list->selectedItems();
  for (std::size_t i = 0; i < selection.size() ; ++i)
  {
    std::string name = selection[i]->text().toStdString();
    if (goal_poses_[name].isVisible())
    {
      //Set invisible
      goal_poses_[name].hide();
      selection[i]->setBackground(QBrush(Qt::Dense4Pattern));
    }
    else
    {
      //Set visible
      goal_poses_[name].show(planning_display_, context_);
      selection[i]->setBackground(QBrush(Qt::NoBrush));
    }
  }
}

void MotionPlanningFrame::goalPoseSelectionChanged(void)
{
  for (unsigned int i = 0; i < ui_->goal_poses_list->count() ; ++i)
  {
    QListWidgetItem *item = ui_->goal_poses_list->item(i);
    std::string name = item->text().toStdString();
    if ( goal_poses_.find(name) != goal_poses_.end() &&
        ( (item->isSelected() && ! goal_poses_[name].selected )
            || ( ! item->isSelected() && goal_poses_[name].selected )))
      switchGoalPoseMarkerSelection(name);
  }
}

void MotionPlanningFrame::goalPoseDoubleClicked(QListWidgetItem * item)
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeGoalPoseDoubleClicked, this, item));
}

void MotionPlanningFrame::updateMarkerColorFromName(const std::string & name, float r, float g, float b, float a)
{
  //Update marker color
  for (unsigned int c = 0; c < goal_poses_[name].imarker_msg.controls.size(); ++c)
  {
    for (unsigned int m = 0; m < goal_poses_[name].imarker_msg.controls[c].markers.size(); ++m)
    {
      if (goal_poses_[name].imarker_msg.controls[c].markers[m].type == visualization_msgs::Marker::MESH_RESOURCE ||
          goal_poses_[name].imarker_msg.controls[c].markers[m].type == visualization_msgs::Marker::SPHERE)
      {
        goal_poses_[name].imarker_msg.controls[c].markers[m].color.r = r;
        goal_poses_[name].imarker_msg.controls[c].markers[m].color.g = g;
        goal_poses_[name].imarker_msg.controls[c].markers[m].color.b = b;
        goal_poses_[name].imarker_msg.controls[c].markers[m].color.a = a;
      }
    }
  }

  if (goal_poses_[name].isVisible())
  {
    Ogre::Vector3 position = goal_poses_[name].imarker->getPosition();
    Ogre::Quaternion orientation = goal_poses_[name].imarker->getOrientation();
    goal_poses_[name].updateMarker();
    goal_poses_[name].imarker->setPose(position, orientation, "");
  }
}

void MotionPlanningFrame::computeGoalPoseDoubleClicked(QListWidgetItem * item)
{
  if ( !planning_display_->getRobotInteraction() || planning_display_->getRobotInteraction()->getActiveEndEffectors().empty() || !planning_display_->getQueryGoalState() )
    return;

  std::string item_text = item->text().toStdString();

  //Switch the marker color to processing color while processing
  goal_poses_[item_text].reachable = GoalPoseMarker::PROCESSING;
  planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::updateMarkerColorFromName, this, item_text,
                                                GOAL_PROCESSING_COLOR[0], GOAL_PROCESSING_COLOR[1], GOAL_PROCESSING_COLOR[2], GOAL_NOT_TESTED_COLOR[3]));

  checkIfGoalReachable(planning_display_->getQueryGoalState(), item_text);
  planning_display_->updateQueryGoalState();
}

/* Receives feedback from the interactive marker attached to a goal pose */
void MotionPlanningFrame::goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback)
{
  static Eigen::Affine3d initial_pose_eigen;

  if (feedback.event_type == feedback.BUTTON_CLICK)
  {
    //Unselect all but the clicked one. Needs to be in order, first unselect, then select.
    QListWidgetItem *item = 0;
    for (unsigned int i = 0; i < ui_->goal_poses_list->count(); ++i)
    {
      item = ui_->goal_poses_list->item(i);
      if ( item->text().toStdString() != feedback.marker_name)
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::selectItemJob, this, item, false));
    }

    for (unsigned int i = 0; i < ui_->goal_poses_list->count(); ++i)
    {
      item = ui_->goal_poses_list->item(i);
      if (item->text().toStdString() == feedback.marker_name)
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::selectItemJob, this, item, true));
    }
  }
  else if (feedback.event_type == feedback.MOUSE_DOWN)
  {
    //Store current poses
    goals_initial_pose_.clear();
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
    {
      if (it->second.selected && it->second.isVisible())
      {
        Eigen::Affine3d pose(Eigen::Quaterniond(it->second.imarker->getOrientation().w, it->second.imarker->getOrientation().x,
                                                it->second.imarker->getOrientation().y, it->second.imarker->getOrientation().z));
        pose(0,3) = it->second.imarker->getPosition().x;
        pose(1,3) = it->second.imarker->getPosition().y;
        pose(2,3) = it->second.imarker->getPosition().z;
        goals_initial_pose_.insert(std::pair<std::string, Eigen::Affine3d>(it->second.imarker->getName(), pose));

        if (it->second.imarker->getName() == feedback.marker_name)
          initial_pose_eigen = pose;
      }
    }
    goal_pose_dragging_=true;
  }
  else if (feedback.event_type == feedback.POSE_UPDATE && goal_pose_dragging_)
  {
    //Compute displacement from stored pose, and apply to the rest of selected markers
    Eigen::Affine3d current_pose_eigen;
    tf::poseMsgToEigen(feedback.pose, current_pose_eigen);

    Eigen::Affine3d current_wrt_initial = initial_pose_eigen.inverse() * current_pose_eigen;

    //Display the pose in the ui
    Eigen::Vector3d v = current_pose_eigen.linear().eulerAngles(0,1,2);
    context_->getWindowManager()->setStatus(QString().sprintf(
        "Position: %.2f %.2f %.2f   Orientation: %.2f %.2f %.2f",
        current_pose_eigen(0,3),
        current_pose_eigen(1,3),
        current_pose_eigen(2,3),
        v(0) * 180.0 / boost::math::constants::pi<double>(),
        v(1) * 180.0 / boost::math::constants::pi<double>(),
        v(2) * 180.0 / boost::math::constants::pi<double>()));

    //Update the rest of selected markers
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end() ; ++it)
    {
      if (it->second.isVisible() && it->second.imarker->getName() != feedback.marker_name && it->second.selected)
      {
        visualization_msgs::InteractiveMarkerPose impose;

        Eigen::Affine3d newpose = initial_pose_eigen * current_wrt_initial * initial_pose_eigen.inverse() * goals_initial_pose_[it->second.imarker->getName()];
        tf::poseEigenToMsg(newpose, impose.pose);
        impose.header.frame_id = it->second.imarker->getReferenceFrame();

        it->second.imarker->processMessage(impose);
      }
    }
  } else if (feedback.event_type == feedback.MOUSE_UP)
  {
    goal_pose_dragging_ = false;
    planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::checkIfGoalInCollision, this, feedback.marker_name));
  }
}

void MotionPlanningFrame::checkGoalsReachable(void)
{
  kinematic_state::KinematicStatePtr tmp(new kinematic_state::KinematicState(*planning_display_->getQueryGoalState()));
  for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
    planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::checkIfGoalReachable, this, tmp, it->first));
}

void MotionPlanningFrame::checkGoalsInCollision(void)
{
  kinematic_state::KinematicStatePtr tmp(new kinematic_state::KinematicState(*planning_display_->getQueryGoalState()));
  for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
    planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::checkIfGoalInCollision, this, tmp, it->first));
}

void MotionPlanningFrame::checkIfGoalReachable(const kinematic_state::KinematicStatePtr &work_state, const std::string &goal_name)
{
  if ( ! goal_poses_[goal_name].isVisible())
    return;

  const boost::shared_ptr<rviz::InteractiveMarker> &imarker = goal_poses_[goal_name].imarker;

  geometry_msgs::Pose current_pose;
  current_pose.position.x = imarker->getPosition().x;
  current_pose.position.y = imarker->getPosition().y;
  current_pose.position.z = imarker->getPosition().z;
  current_pose.orientation.x = imarker->getOrientation().x;
  current_pose.orientation.y = imarker->getOrientation().y;
  current_pose.orientation.z = imarker->getOrientation().z;
  current_pose.orientation.w = imarker->getOrientation().w;

  // Call to IK
  bool feasible = planning_display_->getRobotInteraction()->updateState(*work_state,
                                                                        planning_display_->getRobotInteraction()->getActiveEndEffectors()[0],
                                                                        current_pose,
                                                                        planning_display_->getQueryGoalStateHandler()->getIKAttempts(),
                                                                        planning_display_->getQueryGoalStateHandler()->getIKTimeout());
  if (feasible)
  {
    //Switch the marker color to reachable
    goal_poses_[goal_name].reachable = GoalPoseMarker::REACHABLE;
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::updateMarkerColorFromName, this, goal_name,
                                                  GOAL_REACHABLE_COLOR[0], GOAL_REACHABLE_COLOR[1], GOAL_REACHABLE_COLOR[2], GOAL_REACHABLE_COLOR[3]));
  }
  else
  {
    //Switch the marker color to not-reachable
    goal_poses_[goal_name].reachable = GoalPoseMarker::NOT_REACHABLE;
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::updateMarkerColorFromName, this, goal_name,
                                                  GOAL_NOT_REACHABLE_COLOR[0], GOAL_NOT_REACHABLE_COLOR[1], GOAL_NOT_REACHABLE_COLOR[2], GOAL_NOT_REACHABLE_COLOR[3]));
  }
}

void MotionPlanningFrame::checkIfGoalInCollision(const kinematic_state::KinematicStatePtr &work_state, const std::string & goal_name)
{
  // Check if the end-effector is in collision at the current pose
  if ( ! goal_poses_[goal_name].isVisible())
    return;

  const robot_interaction::RobotInteraction::EndEffector &eef = planning_display_->getRobotInteraction()->getActiveEndEffectors()[0];

  const boost::shared_ptr<rviz::InteractiveMarker> &im = goal_poses_[goal_name].imarker;
  Eigen::Affine3d marker_pose_eigen = Eigen::Translation3d(im->getPosition().x, im->getPosition().y, im->getPosition().z) *
    Eigen::Quaterniond(im->getOrientation().w, im->getOrientation().x, im->getOrientation().y, im->getOrientation().z);

  work_state->updateStateWithLinkAt(eef.parent_link, marker_pose_eigen);
  bool in_collision = planning_display_->getPlanningSceneRO()->isStateColliding(*work_state, eef.eef_group);

  if ( in_collision )
  {
    //End effector in collision. Set the color accordingly
    if (goal_poses_[goal_name].reachable != GoalPoseMarker::IN_COLLISION)
    {
      goal_poses_[goal_name].reachable = GoalPoseMarker::IN_COLLISION;
      planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::updateMarkerColorFromName, this, goal_name,
                                                    GOAL_COLLISION_COLOR[0], GOAL_COLLISION_COLOR[1], GOAL_COLLISION_COLOR[2], GOAL_COLLISION_COLOR[3]));
    }
  }
  else
  {
    //Not in collision
    if (goal_poses_[goal_name].reachable == GoalPoseMarker::IN_COLLISION)
    {
      goal_poses_[goal_name].reachable = GoalPoseMarker::NOT_TESTED;
      planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::updateMarkerColorFromName, this, goal_name,
                                                    GOAL_NOT_TESTED_COLOR[0], GOAL_NOT_TESTED_COLOR[1], GOAL_NOT_TESTED_COLOR[2], GOAL_NOT_TESTED_COLOR[3]));
    }
  }
}

void MotionPlanningFrame::checkIfGoalInCollision(const std::string &goal_name)
{
  kinematic_state::KinematicStatePtr tmp(new kinematic_state::KinematicState(*planning_display_->getQueryGoalState()));
  checkIfGoalInCollision(tmp, goal_name);
}

void MotionPlanningFrame::switchGoalPoseMarkerSelection(const std::string &marker_name)
{
  if (planning_display_->getRobotInteraction()->getActiveEndEffectors().empty() || ! goal_poses_[marker_name].isVisible())
    return;

  geometry_msgs::Pose marker_pose;
  marker_pose.position.x = goal_poses_[marker_name].imarker->getPosition().x;
  marker_pose.position.y = goal_poses_[marker_name].imarker->getPosition().y;
  marker_pose.position.z = goal_poses_[marker_name].imarker->getPosition().z;
  marker_pose.orientation.x = goal_poses_[marker_name].imarker->getOrientation().x;
  marker_pose.orientation.y = goal_poses_[marker_name].imarker->getOrientation().y;
  marker_pose.orientation.z = goal_poses_[marker_name].imarker->getOrientation().z;
  marker_pose.orientation.w = goal_poses_[marker_name].imarker->getOrientation().w;
  static const double marker_scale = 0.35;

  MsgMarkerPair imarker;
  if (goal_poses_[marker_name].selected)
  {
    //If selected, unselect
    goal_poses_[marker_name].selected = false;
    setItemSelectionInList(marker_name, false, ui_->goal_poses_list);
    imarker = make6DOFEndEffectorMarker(goal_poses_[marker_name].imarker->getName(),
                                        planning_display_->getRobotInteraction()->getActiveEndEffectors()[0], marker_pose, marker_scale, false);
  }
  else
  {
    //If unselected, select
    goal_poses_[marker_name].selected = true;
    setItemSelectionInList(marker_name, true, ui_->goal_poses_list);
    imarker = make6DOFEndEffectorMarker(goal_poses_[marker_name].imarker->getName(),
                                        planning_display_->getRobotInteraction()->getActiveEndEffectors()[0], marker_pose, marker_scale, true);
  }

  goal_poses_[marker_name].imarker_msg = imarker.first;
  goal_poses_[marker_name].imarker = imarker.second;

  if (goal_poses_[marker_name].reachable == GoalPoseMarker::NOT_TESTED)
    updateMarkerColorFromName(marker_name, GOAL_NOT_TESTED_COLOR[0], GOAL_NOT_TESTED_COLOR[1], GOAL_NOT_TESTED_COLOR[2], GOAL_NOT_TESTED_COLOR[3]);
  else if (goal_poses_[marker_name].reachable == GoalPoseMarker::PROCESSING)
    updateMarkerColorFromName(marker_name, GOAL_PROCESSING_COLOR[0], GOAL_PROCESSING_COLOR[1], GOAL_PROCESSING_COLOR[2], GOAL_PROCESSING_COLOR[3]);
  else if (goal_poses_[marker_name].reachable == GoalPoseMarker::REACHABLE)
    updateMarkerColorFromName(marker_name, GOAL_REACHABLE_COLOR[0], GOAL_REACHABLE_COLOR[1], GOAL_REACHABLE_COLOR[2], GOAL_REACHABLE_COLOR[3]);
  else if (goal_poses_[marker_name].reachable == GoalPoseMarker::NOT_REACHABLE)
    updateMarkerColorFromName(marker_name, GOAL_NOT_REACHABLE_COLOR[0], GOAL_NOT_REACHABLE_COLOR[1], GOAL_NOT_REACHABLE_COLOR[2], GOAL_NOT_REACHABLE_COLOR[3]);
  else if (goal_poses_[marker_name].reachable == GoalPoseMarker::IN_COLLISION)
    updateMarkerColorFromName(marker_name, GOAL_COLLISION_COLOR[0], GOAL_COLLISION_COLOR[1], GOAL_COLLISION_COLOR[2], GOAL_COLLISION_COLOR[3]);

  // Connect signals
  connect( imarker.second.get(), SIGNAL( userFeedback(visualization_msgs::InteractiveMarkerFeedback &)), this, SLOT( goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback &) ));
}

void MotionPlanningFrame::copySelectedGoalPoses(void)
{
  QList<QListWidgetItem *> sel = ui_->goal_poses_list->selectedItems();
  if (sel.empty() || planning_display_->getRobotInteraction()->getActiveEndEffectors().empty())
    return;

  std::string scene_name;
  {
    const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
    if (!ps)
      return;
    else
      scene_name = ps->getName();
  }

  for (int i = 0 ; i < sel.size() ; ++i)
  {
    std::string name = sel[i]->text().toStdString();
    if (! goal_poses_[name].isVisible())
      continue;

    std::stringstream ss;
    ss << scene_name.c_str() << "_pose_" << std::setfill('0') << std::setw(4) << goal_poses_.size();

    Eigen::Affine3d tip_pose = planning_display_->getQueryGoalState()->getLinkState(planning_display_->getRobotInteraction()->getActiveEndEffectors()[0].parent_link)->getGlobalLinkTransform();
    geometry_msgs::Pose marker_pose;
    marker_pose.position.x = goal_poses_[name].imarker->getPosition().x;
    marker_pose.position.y = goal_poses_[name].imarker->getPosition().y;
    marker_pose.position.z = goal_poses_[name].imarker->getPosition().z;
    marker_pose.orientation.x = goal_poses_[name].imarker->getOrientation().x;
    marker_pose.orientation.y = goal_poses_[name].imarker->getOrientation().y;
    marker_pose.orientation.z = goal_poses_[name].imarker->getOrientation().z;
    marker_pose.orientation.w = goal_poses_[name].imarker->getOrientation().w;

    static const float marker_scale = 0.35;
    MsgMarkerPair imarker = make6DOFEndEffectorMarker(ss.str(),
                                                      planning_display_->getRobotInteraction()->getActiveEndEffectors()[0], marker_pose, marker_scale, true);

    goal_poses_.insert(GoalPosePair(ss.str(), GoalPoseMarker(imarker.second, imarker.first, true)));

    // Connect signals
    connect( imarker.second.get(), SIGNAL( userFeedback(visualization_msgs::InteractiveMarkerFeedback &)), this, SLOT( goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback &) ));

    //Unselect the marker source of the copy
    switchGoalPoseMarkerSelection(name);
  }

  planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateGoalPosesList, this));
}

void MotionPlanningFrame::saveStartStateButtonClicked(void)
{
  bool ok = false;

  std::stringstream ss;
  ss << planning_display_->getKinematicModel()->getName().c_str() << "_state_" << std::setfill('0') << std::setw(4) << start_states_.size();

  QString text = QInputDialog::getText(this, tr("Choose a name"),
                                       tr("Start state name:"), QLineEdit::Normal,
                                       QString(ss.str().c_str()), &ok);

  std::string name;
  if (ok)
  {
    if (!text.isEmpty())
    {
      name = text.toStdString();
      if (start_states_.find(name) != start_states_.end())
        QMessageBox::warning(this, "Name already exists", QString("The name '").append(name.c_str()).
                             append("' already exists. Not creating state."));
      else
      {
        //Store the current start state
        moveit_msgs::RobotState msg;
        kinematic_state::kinematicStateToRobotState(*planning_display_->getQueryStartState(), msg);
        start_states_.insert(StartStatePair(name,  StartState(msg)));

        //Save to the database if connected
        if (robot_state_storage_)
        {
          try
          {
              robot_state_storage_->addRobotState(msg, name);
          }
          catch (std::runtime_error &ex)
          {
            ROS_ERROR("Cannot save robot state on the database: %s", ex.what());
          }
        }
      }
    }
    else
      QMessageBox::warning(this, "Start state not saved", "Cannot use an empty name for a new start state.");
  }
  populateStartStatesList();
}

void MotionPlanningFrame::removeSelectedStatesButtonClicked(void)
{
  QList<QListWidgetItem*> found_items = ui_->start_states_list->selectedItems();
  for (unsigned int i = 0; i < found_items.size(); i++)
  {
    start_states_.erase(found_items[i]->text().toStdString());
  }
  populateStartStatesList();
}

void MotionPlanningFrame::removeAllStatesButtonClicked(void)
{
  start_states_.clear();
  populateStartStatesList();
}

void MotionPlanningFrame::populateStartStatesList(void)
{
  ui_->start_states_list->clear();
  for (StartStateMap::iterator it = start_states_.begin(); it != start_states_.end(); ++it)
  {
    QListWidgetItem *item = new QListWidgetItem(QString(it->first.c_str()));
    ui_->start_states_list->addItem(item);
    if (it->second.selected)
    {
      //If selected, highlight in the list
      item->setSelected(true);
    }
  }
}

void MotionPlanningFrame::startStateItemDoubleClicked(QListWidgetItem * item)
{
  //If a start state item is double clicked, apply it to the start query
  kinematic_state::KinematicStatePtr ks(new kinematic_state::KinematicState(*planning_display_->getQueryStartState()));
  kinematic_state::robotStateToKinematicState(start_states_[item->text().toStdString()].state_msg, *ks);
  planning_display_->setQueryStartState(ks);
}

}
