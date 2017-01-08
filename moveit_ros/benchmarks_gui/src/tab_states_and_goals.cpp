/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Mario Prats, Ioan Sucan */

#include <main_window.h>
#include <job_processing.h>
#include <benchmark_processing_thread.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/benchmarks/benchmark_execution.h>
#include <moveit/planning_interface/planning_interface.h>

#include <pluginlib/class_loader.h>

#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>

#include <eigen_conversions/eigen_msg.h>

#include <QMessageBox>
#include <QInputDialog>
#include <QFileDialog>
#include <QProgressDialog>

#include <boost/math/constants/constants.hpp>

namespace benchmark_tool
{
void MainWindow::createGoalAtPose(const std::string& name, const Eigen::Affine3d& pose)
{
  goals_initial_pose_.insert(std::pair<std::string, Eigen::Affine3d>(name, pose));

  geometry_msgs::Pose marker_pose;
  tf::poseEigenToMsg(pose * goal_offset_, marker_pose);
  static const float marker_scale = 0.15;

  GripperMarkerPtr goal_pose(new GripperMarker(
      scene_display_->getPlanningSceneRO()->getCurrentState(), scene_display_->getSceneNode(), visualization_manager_,
      name, scene_display_->getRobotModel()->getModelFrame(), robot_interaction_->getActiveEndEffectors()[0],
      marker_pose, marker_scale, GripperMarker::NOT_TESTED));
  goal_poses_.insert(GoalPosePair(name, goal_pose));

  // Connect signals
  goal_pose->connect(this, SLOT(goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback&)));

  // If connected to a database, save all the goals back to the database
  if (constraints_storage_)
  {
    saveGoalsToDB();
  }
}

void MainWindow::saveGoalsToDB()
{
  if (constraints_storage_)
  {
    // Convert all goal pose markers into constraints and store them
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
    {
      moveit_msgs::Constraints constraints;
      constraints.name = it->first;

      shape_msgs::SolidPrimitive sp;
      sp.type = sp.BOX;
      sp.dimensions.resize(3, std::numeric_limits<float>::epsilon() * 10.0);

      moveit_msgs::PositionConstraint pc;
      pc.header.frame_id = scene_display_->getRobotModel()->getModelFrame();
      pc.link_name = robot_interaction_->getActiveEndEffectors()[0].parent_link;  // TODO this is hacky i think
      pc.constraint_region.primitives.push_back(sp);
      geometry_msgs::Pose posemsg;
      it->second->getPosition(posemsg.position);
      posemsg.orientation.x = 0.0;
      posemsg.orientation.y = 0.0;
      posemsg.orientation.z = 0.0;
      posemsg.orientation.w = 1.0;
      pc.constraint_region.primitive_poses.push_back(posemsg);
      pc.weight = 1.0;
      constraints.position_constraints.push_back(pc);

      moveit_msgs::OrientationConstraint oc;
      oc.header.frame_id = scene_display_->getRobotModel()->getModelFrame();
      oc.link_name = robot_interaction_->getActiveEndEffectors()[0].parent_link;  // TODO this is hacky i think
      it->second->getOrientation(oc.orientation);
      oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance =
          std::numeric_limits<float>::epsilon() * 10.0;
      oc.weight = 1.0;
      constraints.orientation_constraints.push_back(oc);

      try
      {
        constraints_storage_->addConstraints(constraints);
      }
      catch (std::runtime_error& ex)
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

void MainWindow::createGoalPoseButtonClicked(void)
{
  std::stringstream ss;

  {
    const planning_scene_monitor::LockedPlanningSceneRO& ps = scene_display_->getPlanningSceneRO();
    if (!ps || robot_interaction_->getActiveEndEffectors().empty())
    {
      if (!ps)
        ROS_ERROR("Not planning scene");
      if (robot_interaction_->getActiveEndEffectors().empty())
        ROS_ERROR("No end effector");
      return;
    }
    else
      ss << ps->getName().c_str() << "_pose_" << std::setfill('0') << std::setw(4) << goal_poses_.size();
  }

  bool ok = false;
  QString text = QInputDialog::getText(this, tr("Choose a name"), tr("Goal pose name:"), QLineEdit::Normal,
                                       QString(ss.str().c_str()), &ok);

  std::string name;
  if (ok)
  {
    if (!text.isEmpty())
    {
      name = text.toStdString();
      if (goal_poses_.find(name) != goal_poses_.end())
        QMessageBox::warning(this, "Name already exists",
                             QString("The name '").append(name.c_str()).append("' already exists. Not creating goal."));
      else
      {
        // Create the new goal pose at the current eef pose, and attach an interactive marker to it
        scene_display_->getPlanningSceneRW()->getCurrentStateNonConst().update();
        Eigen::Affine3d tip_pose = scene_display_->getPlanningSceneRO()->getCurrentState().getGlobalLinkTransform(
            robot_interaction_->getActiveEndEffectors()[0].parent_link);
        createGoalAtPose(name, tip_pose);
      }
    }
    else
      QMessageBox::warning(this, "Goal not created", "Cannot use an empty name for a new goal pose.");
  }

  populateGoalPosesList();
}

void MainWindow::showBBoxGoalsDialog()
{
  std::string goals_base_name;
  {
    const planning_scene_monitor::LockedPlanningSceneRO& ps = scene_display_->getPlanningSceneRO();
    if (!ps || robot_interaction_->getActiveEndEffectors().empty())
    {
      if (!ps)
        ROS_ERROR("No planning scene");
      if (robot_interaction_->getActiveEndEffectors().empty())
        ROS_ERROR("No end effector");
      return;
    }
    goals_base_name = ps->getName() + "_pose_";
  }

  bbox_dialog_ui_.base_name_text->setText(QString(goals_base_name.c_str()));

  bbox_dialog_->exec();
}

void MainWindow::createBBoxGoalsButtonClicked(void)
{
  bbox_dialog_->close();

  double minx = bbox_dialog_ui_.center_x_text->text().toDouble() - bbox_dialog_ui_.size_x_text->text().toDouble() / 2.0;
  double maxx = bbox_dialog_ui_.center_x_text->text().toDouble() + bbox_dialog_ui_.size_x_text->text().toDouble() / 2.0;
  double stepx = (maxx - minx) / std::max<double>(bbox_dialog_ui_.ngoals_x_text->text().toDouble() - 1, 1);
  double miny = bbox_dialog_ui_.center_y_text->text().toDouble() - bbox_dialog_ui_.size_y_text->text().toDouble() / 2.0;
  double maxy = bbox_dialog_ui_.center_y_text->text().toDouble() + bbox_dialog_ui_.size_y_text->text().toDouble() / 2.0;
  double stepy = (maxy - miny) / std::max<double>(bbox_dialog_ui_.ngoals_y_text->text().toDouble() - 1, 1);
  double minz = bbox_dialog_ui_.center_z_text->text().toDouble() - bbox_dialog_ui_.size_z_text->text().toDouble() / 2.0;
  double maxz = bbox_dialog_ui_.center_z_text->text().toDouble() + bbox_dialog_ui_.size_z_text->text().toDouble() / 2.0;
  double stepz = (maxz - minz) / std::max<double>(bbox_dialog_ui_.ngoals_z_text->text().toDouble() - 1, 1);

  Eigen::Affine3d goal_pose;
  goal_pose.setIdentity();
  for (std::size_t x = 0; x < bbox_dialog_ui_.ngoals_x_text->text().toShort(); ++x)
    for (std::size_t y = 0; y < bbox_dialog_ui_.ngoals_y_text->text().toShort(); ++y)
      for (std::size_t z = 0; z < bbox_dialog_ui_.ngoals_z_text->text().toShort(); ++z)
      {
        goal_pose(0, 3) = minx + x * stepx;
        goal_pose(1, 3) = miny + y * stepy;
        goal_pose(2, 3) = minz + z * stepz;

        std::stringstream ss;
        ss << bbox_dialog_ui_.base_name_text->text().toStdString() << std::setfill('0') << std::setw(4)
           << goal_poses_.size();

        createGoalAtPose(ss.str(), goal_pose);
      }

  populateGoalPosesList();
}

void MainWindow::removeSelectedGoalsButtonClicked(void)
{
  QList<QListWidgetItem*> found_items = ui_.goal_poses_list->selectedItems();
  for (unsigned int i = 0; i < found_items.size(); i++)
  {
    goal_poses_.erase(found_items[i]->text().toStdString());
  }
  populateGoalPosesList();
}

void MainWindow::removeAllGoalsButtonClicked(void)
{
  goal_poses_.clear();
  goals_initial_pose_.clear();
  populateGoalPosesList();
}

void MainWindow::loadGoalsFromDBButtonClicked(void)
{
  // Get all the constraints from the database, convert to goal pose markers
  if (constraints_storage_ && !robot_interaction_->getActiveEndEffectors().empty())
  {
    // First clear the current list
    removeAllGoalsButtonClicked();

    std::vector<std::string> names;
    try
    {
      constraints_storage_->getKnownConstraints(ui_.load_poses_filter_text->text().toStdString(), names);
    }
    catch (std::runtime_error& ex)
    {
      QMessageBox::warning(this, "Cannot query the database",
                           QString("Wrongly formatted regular expression for goal poses: ").append(ex.what()));
      return;
    }

    for (unsigned int i = 0; i < names.size(); i++)
    {
      // Create a goal pose marker
      moveit_warehouse::ConstraintsWithMetadata c;
      bool got_constraint = false;
      try
      {
        got_constraint = constraints_storage_->getConstraints(c, names[i]);
      }
      catch (std::runtime_error& ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      if (!got_constraint)
        continue;

      if (c->position_constraints.size() > 0 &&
          c->position_constraints[0].constraint_region.primitive_poses.size() > 0 &&
          c->orientation_constraints.size() > 0)
      {
        // Overwrite if exists. TODO: Ask the user before overwriting? copy the existing one with another name before?
        if (goal_poses_.find(c->name) != goal_poses_.end())
        {
          goal_poses_.erase(c->name);
        }
        geometry_msgs::Pose shape_pose;
        shape_pose.position = c->position_constraints[0].constraint_region.primitive_poses[0].position;
        shape_pose.orientation = c->orientation_constraints[0].orientation;

        Eigen::Affine3d shape_pose_eigen;
        tf::poseMsgToEigen(shape_pose, shape_pose_eigen);
        goals_initial_pose_.insert(std::pair<std::string, Eigen::Affine3d>(c->name, shape_pose_eigen));

        tf::poseEigenToMsg(shape_pose_eigen * goal_offset_, shape_pose);

        static const float marker_scale = 0.15;
        GripperMarkerPtr goal_pose(new GripperMarker(
            scene_display_->getPlanningSceneRO()->getCurrentState(), scene_display_->getSceneNode(),
            visualization_manager_, c->name, scene_display_->getRobotModel()->getModelFrame(),
            robot_interaction_->getActiveEndEffectors()[0], shape_pose, marker_scale, GripperMarker::NOT_TESTED, false,
            ui_.show_x_checkbox->isChecked(), ui_.show_y_checkbox->isChecked(), ui_.show_z_checkbox->isChecked()));
        // Connect signals
        goal_pose->connect(this, SLOT(goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback&)));

        goal_poses_.insert(GoalPosePair(c->name, goal_pose));
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

void MainWindow::saveGoalsOnDBButtonClicked(void)
{
  saveGoalsToDB();
}

void MainWindow::deleteGoalsOnDBButtonClicked(void)
{
  if (constraints_storage_)
  {
    // Warn the user
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
        // Go through the list of goal poses, and delete those selected
        QList<QListWidgetItem*> found_items = ui_.goal_poses_list->selectedItems();
        for (std::size_t i = 0; i < found_items.size(); i++)
        {
          try
          {
            constraints_storage_->removeConstraints(found_items[i]->text().toStdString());
          }
          catch (std::runtime_error& ex)
          {
            ROS_ERROR("%s", ex.what());
          }
        }
        break;
      }
    }
  }
  removeSelectedGoalsButtonClicked();
}

void MainWindow::loadStatesFromDBButtonClicked(void)
{
  // Get all the start states from the database
  if (robot_state_storage_)
  {
    // First clear the current list
    removeAllStatesButtonClicked();

    std::vector<std::string> names;
    try
    {
      robot_state_storage_->getKnownRobotStates(ui_.load_states_filter_text->text().toStdString(), names);
    }
    catch (std::runtime_error& ex)
    {
      QMessageBox::warning(this, "Cannot query the database",
                           QString("Wrongly formatted regular expression for goal poses: ").append(ex.what()));
      return;
    }

    for (unsigned int i = 0; i < names.size(); i++)
    {
      moveit_warehouse::RobotStateWithMetadata rs;
      bool got_state = false;
      try
      {
        got_state = robot_state_storage_->getRobotState(rs, names[i]);
      }
      catch (std::runtime_error& ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      if (!got_state)
        continue;

      // Overwrite if exists.
      if (start_states_.find(names[i]) != start_states_.end())
      {
        start_states_.erase(names[i]);
      }

      // Store the current start state
      start_states_.insert(StartStatePair(names[i], StartStatePtr(new StartState(*rs))));
    }
    populateStartStatesList();
  }
  else
  {
    QMessageBox::warning(this, "Warning", "Not connected to a database.");
  }
}

void MainWindow::saveStatesOnDBButtonClicked(void)
{
  if (robot_state_storage_)
  {
    // Store all start states
    for (StartStateMap::iterator it = start_states_.begin(); it != start_states_.end(); ++it)
      try
      {
        robot_state_storage_->addRobotState(it->second->state_msg, it->first);
      }
      catch (std::runtime_error& ex)
      {
        ROS_ERROR("Cannot save robot state: %s", ex.what());
      }
  }
  else
  {
    QMessageBox::warning(this, "Warning", "Not connected to a database.");
  }
}

void MainWindow::deleteStatesOnDBButtonClicked(void)
{
  if (robot_state_storage_)
  {
    // Warn the user
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
        QList<QListWidgetItem*> found_items = ui_.start_states_list->selectedItems();
        for (unsigned int i = 0; i < found_items.size(); ++i)
        {
          try
          {
            robot_state_storage_->removeRobotState(found_items[i]->text().toStdString());
          }
          catch (std::runtime_error& ex)
          {
            ROS_ERROR("%s", ex.what());
          }
        }
        break;
      }
    }
  }
  removeSelectedStatesButtonClicked();
}

void MainWindow::visibleAxisChanged(int state)
{
  if (!robot_interaction_->getActiveEndEffectors().empty())
  {
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
    {
      if (it->second->isVisible())
      {
        it->second->setAxisVisibility(ui_.show_x_checkbox->isChecked(), ui_.show_y_checkbox->isChecked(),
                                      ui_.show_z_checkbox->isChecked());
      }
    }
  }
}

void MainWindow::populateGoalPosesList(void)
{
  ui_.goal_poses_list->clear();
  for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
  {
    QListWidgetItem* item = new QListWidgetItem(QString(it->first.c_str()));
    ui_.goal_poses_list->addItem(item);
    if (!it->second->isVisible())
    {
      item->setBackground(QBrush(Qt::Dense4Pattern));
    }
    else if (it->second->isSelected())
    {
      // If selected, highlight in the list
      item->setSelected(true);
    }
  }
}

void MainWindow::switchGoalVisibilityButtonClicked(void)
{
  QList<QListWidgetItem*> selection = ui_.goal_poses_list->selectedItems();
  for (std::size_t i = 0; i < selection.size(); ++i)
  {
    std::string name = selection[i]->text().toStdString();
    if (goal_poses_[name]->isVisible())
    {
      // Set invisible
      goal_poses_[name]->hide();
      selection[i]->setBackground(QBrush(Qt::Dense4Pattern));
    }
    else
    {
      // Set visible
      goal_poses_[name]->show(scene_display_->getSceneNode(), visualization_manager_);
      selection[i]->setBackground(QBrush(Qt::NoBrush));
    }
  }
}

void MainWindow::goalPoseSelectionChanged(void)
{
  for (unsigned int i = 0; i < ui_.goal_poses_list->count(); ++i)
  {
    QListWidgetItem* item = ui_.goal_poses_list->item(i);
    std::string name = item->text().toStdString();
    if (goal_poses_.find(name) != goal_poses_.end() && ((item->isSelected() && !goal_poses_[name]->isSelected()) ||
                                                        (!item->isSelected() && goal_poses_[name]->isSelected())))
      switchGoalPoseMarkerSelection(name);
  }
}

void MainWindow::goalPoseDoubleClicked(QListWidgetItem* item)
{
  JobProcessing::addBackgroundJob(boost::bind(&MainWindow::computeGoalPoseDoubleClicked, this, item));
}

void MainWindow::computeGoalPoseDoubleClicked(QListWidgetItem* item)
{
  if (!robot_interaction_ || robot_interaction_->getActiveEndEffectors().empty())
    return;

  std::string item_text = item->text().toStdString();

  // Switch the marker color to processing color while processing
  JobProcessing::addMainLoopJob(
      boost::bind(&MainWindow::updateGoalMarkerStateFromName, this, item_text, GripperMarker::PROCESSING));

  checkIfGoalReachable(item_text, true);
}

/* Receives feedback from the interactive marker attached to a goal pose */
void MainWindow::goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback& feedback)
{
  if (feedback.event_type == feedback.BUTTON_CLICK)
  {
    // Unselect all but the clicked one. Needs to be in order, first unselect, then select.
    QListWidgetItem* item = 0;
    for (unsigned int i = 0; i < ui_.goal_poses_list->count(); ++i)
    {
      item = ui_.goal_poses_list->item(i);
      if (item->text().toStdString() != feedback.marker_name)
        JobProcessing::addMainLoopJob(boost::bind(&MainWindow::selectItemJob, this, item, false));
    }

    for (unsigned int i = 0; i < ui_.goal_poses_list->count(); ++i)
    {
      item = ui_.goal_poses_list->item(i);
      if (item->text().toStdString() == feedback.marker_name)
        JobProcessing::addMainLoopJob(boost::bind(&MainWindow::selectItemJob, this, item, true));
    }
  }
  else if (feedback.event_type == feedback.MOUSE_DOWN)
  {
    // First check to see if this mouse_down is happening on an
    // already-selected goal pose.  A mouse_down event is sent even if
    // the gesture turns out to be nothing but a click.
    bool this_goal_already_selected = false;
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
    {
      if (it->second->isSelected() && it->second->isVisible() && it->second->imarker->getName() == feedback.marker_name)
      {
        this_goal_already_selected = true;
      }
    }

    // If this is a mouse-down on an already-selected goal pose, we
    // want to initialize dragging all selected goal poses at the same
    // time.
    if (this_goal_already_selected)
    {
      // Store current poses
      goals_dragging_initial_pose_.clear();
      for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
      {
        if (it->second->isSelected() && it->second->isVisible())
        {
          Eigen::Affine3d pose(
              Eigen::Quaterniond(it->second->imarker->getOrientation().w, it->second->imarker->getOrientation().x,
                                 it->second->imarker->getOrientation().y, it->second->imarker->getOrientation().z));
          pose(0, 3) = it->second->imarker->getPosition().x;
          pose(1, 3) = it->second->imarker->getPosition().y;
          pose(2, 3) = it->second->imarker->getPosition().z;
          goals_dragging_initial_pose_.insert(
              std::pair<std::string, Eigen::Affine3d>(it->second->imarker->getName(), pose));

          if (it->second->imarker->getName() == feedback.marker_name)
            drag_initial_pose_ = pose;
        }
      }
      goal_pose_dragging_ = true;
    }
  }
  else if (feedback.event_type == feedback.POSE_UPDATE && goal_pose_dragging_)
  {
    // Compute displacement from stored pose, and apply to the rest of selected markers
    Eigen::Affine3d current_pose_eigen;
    tf::poseMsgToEigen(feedback.pose, current_pose_eigen);

    Eigen::Affine3d current_wrt_initial = drag_initial_pose_.inverse() * current_pose_eigen;

    // Display the pose in the ui
    Eigen::Vector3d v = current_pose_eigen.linear().eulerAngles(0, 1, 2);
    setStatusFromBackground(STATUS_INFO, QString().sprintf("%.2f %.2f %.2f    %.2f %.2f %.2f", current_pose_eigen(0, 3),
                                                           current_pose_eigen(1, 3), current_pose_eigen(2, 3),
                                                           v(0) * 180.0 / boost::math::constants::pi<double>(),
                                                           v(1) * 180.0 / boost::math::constants::pi<double>(),
                                                           v(2) * 180.0 / boost::math::constants::pi<double>()));

    // Update the rest of selected markers
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
    {
      if (it->second->isVisible() && it->second->imarker->getName() != feedback.marker_name && it->second->isSelected())
      {
        visualization_msgs::InteractiveMarkerPose impose;

        Eigen::Affine3d newpose = drag_initial_pose_ * current_wrt_initial * drag_initial_pose_.inverse() *
                                  goals_dragging_initial_pose_[it->second->imarker->getName()];
        tf::poseEigenToMsg(newpose, impose.pose);

        it->second->imarker->setPose(
            Ogre::Vector3(impose.pose.position.x, impose.pose.position.y, impose.pose.position.z),
            Ogre::Quaternion(impose.pose.orientation.w, impose.pose.orientation.x, impose.pose.orientation.y,
                             impose.pose.orientation.z),
            "");
      }
    }
  }
  else if (feedback.event_type == feedback.MOUSE_UP)
  {
    goal_pose_dragging_ = false;
    JobProcessing::addBackgroundJob(boost::bind(&MainWindow::checkIfGoalInCollision, this, feedback.marker_name));
  }
}

void MainWindow::checkGoalsReachable(void)
{
  for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
    JobProcessing::addBackgroundJob(boost::bind(&MainWindow::checkIfGoalReachable, this, it->first, false));
}

void MainWindow::checkGoalsInCollision(void)
{
  for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
    JobProcessing::addBackgroundJob(boost::bind(&MainWindow::checkIfGoalInCollision, this, it->first));
}

void MainWindow::checkIfGoalReachable(const std::string& goal_name, bool update_if_reachable)
{
  if (goal_poses_.find(goal_name) == goal_poses_.end())
    return;

  if (!goal_poses_[goal_name]->isVisible())
    return;

  const boost::shared_ptr<rviz::InteractiveMarker>& imarker = goal_poses_[goal_name]->imarker;

  geometry_msgs::Pose current_pose_msg;
  current_pose_msg.position.x = imarker->getPosition().x;
  current_pose_msg.position.y = imarker->getPosition().y;
  current_pose_msg.position.z = imarker->getPosition().z;
  current_pose_msg.orientation.x = imarker->getOrientation().x;
  current_pose_msg.orientation.y = imarker->getOrientation().y;
  current_pose_msg.orientation.z = imarker->getOrientation().z;
  current_pose_msg.orientation.w = imarker->getOrientation().w;

  // Call to IK
  setStatusFromBackground(STATUS_INFO, "Computing inverse kinematics...");
  robot_state::RobotState ks(scene_display_->getPlanningSceneRO()->getCurrentState());
  static const int ik_attempts = 5;
  static const float ik_timeout = 0.2;
  bool feasible = robot_interaction_->updateState(ks, robot_interaction_->getActiveEndEffectors()[0], current_pose_msg,
                                                  ik_attempts, ik_timeout);
  if (feasible)
  {
    setStatusFromBackground(STATUS_INFO, "Updating state...");
    if (update_if_reachable)
    {
      scene_display_->getPlanningSceneRW()->setCurrentState(ks);
      scene_display_->queueRenderSceneGeometry();
    }

    setStatusFromBackground(STATUS_INFO, "Updating marker...");
    // Switch the marker color to reachable
    JobProcessing::addMainLoopJob(
        boost::bind(&MainWindow::updateGoalMarkerStateFromName, this, goal_name, GripperMarker::REACHABLE));
  }
  else
  {
    // Switch the marker color to not-reachable
    JobProcessing::addMainLoopJob(
        boost::bind(&MainWindow::updateGoalMarkerStateFromName, this, goal_name, GripperMarker::NOT_REACHABLE));
  }
  setStatusFromBackground(STATUS_INFO, "");
}

bool MainWindow::isGroupCollidingWithWorld(robot_state::RobotState& robot_state, const std::string& group_name)
{
  collision_detection::AllowedCollisionMatrix acm(scene_display_->getPlanningSceneRO()->getAllowedCollisionMatrix());
  // get link names in group_name
  const std::vector<std::string>& group_link_names =
      scene_display_->getRobotModel()->getJointModelGroup(group_name)->getLinkModelNamesWithCollisionGeometry();

  // Create a set of links which is all links minus links in the group.
  const std::vector<std::string>& all_links = scene_display_->getRobotModel()->getLinkModelNames();
  std::set<std::string> link_set(all_links.begin(), all_links.end());
  for (size_t i = 0; i < group_link_names.size(); i++)
  {
    link_set.erase(group_link_names[i]);
  }

  // for each link name in the set,
  for (std::set<std::string>::const_iterator it = link_set.begin(); it != link_set.end(); it++)
  {
    // allow collisions with link.
    acm.setEntry(*it, true);
  }

  // call checkCollision();
  collision_detection::CollisionRequest req;
  req.verbose = false;
  collision_detection::CollisionResult res;
  scene_display_->getPlanningSceneRO()->checkCollision(req, res, robot_state, acm);

  // return result boolean.
  return res.collision;
}

void MainWindow::checkIfGoalInCollision(const std::string& goal_name)
{
  if (goal_poses_.find(goal_name) == goal_poses_.end())
    return;

  // Check if the end-effector is in collision at the current pose
  if (!goal_poses_[goal_name]->isVisible())
    return;

  const robot_interaction::RobotInteraction::EndEffector& eef = robot_interaction_->getActiveEndEffectors()[0];

  const boost::shared_ptr<rviz::InteractiveMarker>& im = goal_poses_[goal_name]->imarker;
  Eigen::Affine3d marker_pose_eigen;
  goal_poses_[goal_name]->getPose(marker_pose_eigen);

  robot_state::RobotState ks(scene_display_->getPlanningSceneRO()->getCurrentState());
  ks.updateStateWithLinkAt(eef.parent_link, marker_pose_eigen);
  bool in_collision = isGroupCollidingWithWorld(ks, eef.eef_group);

  if (in_collision)
  {
    JobProcessing::addMainLoopJob(
        boost::bind(&MainWindow::updateGoalMarkerStateFromName, this, goal_name, GripperMarker::IN_COLLISION));
  }
  else
  {
    JobProcessing::addMainLoopJob(
        boost::bind(&MainWindow::updateGoalMarkerStateFromName, this, goal_name, GripperMarker::NOT_TESTED));
  }
}

void MainWindow::switchGoalPoseMarkerSelection(const std::string& marker_name)
{
  if (robot_interaction_->getActiveEndEffectors().empty() || !goal_poses_[marker_name]->isVisible())
    return;

  if (goal_poses_[marker_name]->isSelected())
  {
    // If selected, unselect
    goal_poses_[marker_name]->unselect();
    setItemSelectionInList(marker_name, false, ui_.goal_poses_list);
  }
  else
  {
    // If unselected, select. Only display the gripper mesh for one
    if (ui_.goal_poses_list->selectedItems().size() == 1)
      goal_poses_[marker_name]->select(true);
    else
      goal_poses_[marker_name]->select(false);
    setItemSelectionInList(marker_name, true, ui_.goal_poses_list);
  }
}

void MainWindow::copySelectedGoalPoses(void)
{
  QList<QListWidgetItem*> sel = ui_.goal_poses_list->selectedItems();
  if (sel.empty() || robot_interaction_->getActiveEndEffectors().empty())
    return;

  std::string scene_name;
  {
    const planning_scene_monitor::LockedPlanningSceneRO& ps = scene_display_->getPlanningSceneRO();
    if (!ps)
      return;
    else
      scene_name = ps->getName();
  }

  for (int i = 0; i < sel.size(); ++i)
  {
    std::string name = sel[i]->text().toStdString();
    if (!goal_poses_[name]->isVisible())
      continue;

    std::stringstream ss;
    ss << scene_name.c_str() << "_pose_" << std::setfill('0') << std::setw(4) << goal_poses_.size();

    scene_display_->getPlanningSceneRW()->getCurrentStateNonConst().update();
    Eigen::Affine3d tip_pose = scene_display_->getPlanningSceneRO()->getCurrentState().getGlobalLinkTransform(
        robot_interaction_->getActiveEndEffectors()[0].parent_link);
    geometry_msgs::Pose marker_pose;
    marker_pose.position.x = goal_poses_[name]->imarker->getPosition().x;
    marker_pose.position.y = goal_poses_[name]->imarker->getPosition().y;
    marker_pose.position.z = goal_poses_[name]->imarker->getPosition().z;
    marker_pose.orientation.x = goal_poses_[name]->imarker->getOrientation().x;
    marker_pose.orientation.y = goal_poses_[name]->imarker->getOrientation().y;
    marker_pose.orientation.z = goal_poses_[name]->imarker->getOrientation().z;
    marker_pose.orientation.w = goal_poses_[name]->imarker->getOrientation().w;

    static const float marker_scale = 0.15;
    GripperMarkerPtr goal_pose(new GripperMarker(
        scene_display_->getPlanningSceneRO()->getCurrentState(), scene_display_->getSceneNode(), visualization_manager_,
        ss.str(), scene_display_->getRobotModel()->getModelFrame(), robot_interaction_->getActiveEndEffectors()[0],
        marker_pose, marker_scale, GripperMarker::NOT_TESTED, true));

    goal_poses_.insert(GoalPosePair(ss.str(), goal_pose));

    // Connect signals
    goal_pose->connect(this, SLOT(goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback&)));

    // Unselect the marker source of the copy
    switchGoalPoseMarkerSelection(name);
  }

  JobProcessing::addMainLoopJob(boost::bind(&MainWindow::populateGoalPosesList, this));
}

void MainWindow::saveStartStateButtonClicked(void)
{
  bool ok = false;

  std::stringstream ss;
  ss << scene_display_->getRobotModel()->getName().c_str() << "_state_" << std::setfill('0') << std::setw(4)
     << start_states_.size();

  QString text = QInputDialog::getText(this, tr("Choose a name"), tr("Start state name:"), QLineEdit::Normal,
                                       QString(ss.str().c_str()), &ok);

  std::string name;
  if (ok)
  {
    if (!text.isEmpty())
    {
      name = text.toStdString();
      if (start_states_.find(name) != start_states_.end())
        QMessageBox::warning(
            this, "Name already exists",
            QString("The name '").append(name.c_str()).append("' already exists. Not creating state."));
      else
      {
        // Store the current start state
        moveit_msgs::RobotState msg;
        robot_state::robotStateToRobotStateMsg(scene_display_->getPlanningSceneRO()->getCurrentState(), msg);
        start_states_.insert(StartStatePair(name, StartStatePtr(new StartState(msg))));

        // Save to the database if connected
        if (robot_state_storage_)
        {
          try
          {
            robot_state_storage_->addRobotState(msg, name);
          }
          catch (std::runtime_error& ex)
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

void MainWindow::removeSelectedStatesButtonClicked(void)
{
  QList<QListWidgetItem*> found_items = ui_.start_states_list->selectedItems();
  for (unsigned int i = 0; i < found_items.size(); i++)
  {
    start_states_.erase(found_items[i]->text().toStdString());
  }
  populateStartStatesList();
}

void MainWindow::removeAllStatesButtonClicked(void)
{
  start_states_.clear();
  populateStartStatesList();
}

void MainWindow::populateStartStatesList(void)
{
  ui_.start_states_list->clear();
  for (StartStateMap::iterator it = start_states_.begin(); it != start_states_.end(); ++it)
  {
    QListWidgetItem* item = new QListWidgetItem(QString(it->first.c_str()));
    ui_.start_states_list->addItem(item);
    if (it->second->selected)
    {
      // If selected, highlight in the list
      item->setSelected(true);
    }
  }
}

void MainWindow::startStateItemDoubleClicked(QListWidgetItem* item)
{
  scene_display_->getPlanningSceneRW()->setCurrentState(start_states_[item->text().toStdString()]->state_msg);
  scene_display_->queueRenderSceneGeometry();
}

void MainWindow::runBenchmark(void)
{
  if (!robot_interaction_ || robot_interaction_->getActiveEndEffectors().size() == 0)
  {
    QMessageBox::warning(this, "Error", "Please make sure a planning group with an end-effector is active");
    return;
  }

  run_benchmark_ui_.benchmark_goal_text->setText(ui_.load_poses_filter_text->text());
  run_benchmark_ui_.benchmark_start_state_text->setText(ui_.load_states_filter_text->text());

  // Load available planners
  boost::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
  std::map<std::string, planning_interface::PlannerManagerPtr> planner_interfaces;
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }

  if (planner_plugin_loader)
  {
    // load the planning plugins
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    for (std::size_t i = 0; i < classes.size(); ++i)
    {
      ROS_DEBUG("Attempting to load and configure %s", classes[i].c_str());
      try
      {
        planning_interface::PlannerManagerPtr p = planner_plugin_loader->createInstance(classes[i]);
        p->initialize(scene_display_->getPlanningSceneRO()->getRobotModel(), "");
        planner_interfaces[classes[i]] = p;
      }
      catch (pluginlib::PluginlibException& ex)
      {
        ROS_ERROR_STREAM("Exception while loading planner '" << classes[i] << "': " << ex.what());
      }
    }

    // Get the list of planning interfaces and planning algorithms
    if (!planner_interfaces.empty())
    {
      std::stringstream interfaces_ss, algorithms_ss;
      for (std::map<std::string, planning_interface::PlannerManagerPtr>::const_iterator it = planner_interfaces.begin();
           it != planner_interfaces.end(); ++it)
      {
        interfaces_ss << it->first << " ";

        std::vector<std::string> known;
        it->second->getPlanningAlgorithms(known);
        for (std::size_t i = 0; i < known.size(); ++i)
          algorithms_ss << known[i] << " ";
      }
      ROS_DEBUG("Available planner instances: %s. Available algorithms: %s", interfaces_ss.str().c_str(),
                algorithms_ss.str().c_str());
      run_benchmark_ui_.planning_interfaces_text->setText(QString(interfaces_ss.str().c_str()));
      run_benchmark_ui_.planning_algorithms_text->setText(QString(algorithms_ss.str().c_str()));
    }
  }

  run_benchmark_dialog_->show();
}

void MainWindow::benchmarkFolderButtonClicked(void)
{
  QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home",
                                                  QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  run_benchmark_ui_.benchmark_output_folder_text->setText(dir);
}

void MainWindow::cancelBenchmarkButtonClicked(void)
{
  run_benchmark_dialog_->close();
}

bool MainWindow::saveBenchmarkConfigButtonClicked(void)
{
  if (run_benchmark_ui_.benchmark_output_folder_text->text().isEmpty())
  {
    QMessageBox::warning(this, "Missing data", "Must specify an output folder");
    return false;
  }

  QString outfilename = run_benchmark_ui_.benchmark_output_folder_text->text().append("/config.cfg");
  std::ofstream outfile(outfilename.toUtf8());
  if (outfile)
  {
    outfile << "[scene]" << std::endl;
    outfile << "group=" << ui_.planning_group_combo->currentText().toStdString() << std::endl;
    outfile << "default_constrained_link=" << robot_interaction_->getActiveEndEffectors()[0].parent_link << std::endl;
    outfile << "planning_frame=" << scene_display_->getPlanningSceneMonitor()->getRobotModel()->getModelFrame()
            << std::endl;
    outfile << "name=" << scene_display_->getPlanningSceneRO()->getName() << std::endl;

    outfile << "timeout=" << run_benchmark_ui_.timeout_spin->value() << std::endl;
    outfile << "runs=" << run_benchmark_ui_.number_of_runs_spin->value() << std::endl;
    outfile << "output=" << run_benchmark_ui_.benchmark_output_folder_text->text().toStdString() << "/"
            << scene_display_->getPlanningSceneMonitor()->getRobotModel()->getName() << "_"
            << scene_display_->getPlanningSceneRO()->getName() << "_" << ros::Time::now() << std::endl;
    outfile << "start=" << run_benchmark_ui_.benchmark_start_state_text->text().toStdString() << std::endl;
    outfile << "query=" << std::endl;
    outfile << "goal=" << run_benchmark_ui_.benchmark_goal_text->text().toStdString() << std::endl;
    outfile << "goal_offset_roll=" << ui_.goal_offset_roll->value() << std::endl;
    outfile << "goal_offset_pitch=" << ui_.goal_offset_pitch->value() << std::endl;
    outfile << "goal_offset_yaw=" << ui_.goal_offset_yaw->value() << std::endl << std::endl;

    outfile << "[plugin]" << std::endl;
    outfile << "name=" << run_benchmark_ui_.planning_interfaces_text->text().toStdString() << std::endl;
    outfile << "planners=" << run_benchmark_ui_.planning_algorithms_text->text().toStdString() << std::endl;

    outfile.close();

    run_benchmark_dialog_->close();
  }
  else
  {
    QMessageBox::warning(this, "Error", QString("Cannot open file ").append(outfilename).append(" for writing"));
    return false;
  }

  return true;
}

void MainWindow::runBenchmarkButtonClicked(void)
{
  if (!saveBenchmarkConfigButtonClicked())
    return;

  QMessageBox msgBox;
  msgBox.setText("This will run the benchmark pipeline. The process will take several minutes during which you will "
                 "not be able to interact with the interface. You can follow the progress in the console output");
  msgBox.setInformativeText("Do you want to continue?");
  msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
  msgBox.setDefaultButton(QMessageBox::Yes);
  int ret = msgBox.exec();

  switch (ret)
  {
    case QMessageBox::Yes:
    {
      // Set up db
      warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase();
      conn->setParams(database_host_, database_port_, 10.0);
      if (!conn->connect())
      {
        QMessageBox::warning(this, "Error", QString("Unable to connect to Database"));
        break;
      }
      QString outfilename = run_benchmark_ui_.benchmark_output_folder_text->text().append("/config.cfg");
      moveit_benchmarks::BenchmarkType btype = 0;
      moveit_benchmarks::BenchmarkExecution be(scene_display_->getPlanningSceneMonitor()->getPlanningScene(), conn);
      if (run_benchmark_ui_.benchmark_include_planners_checkbox->isChecked())
        btype += moveit_benchmarks::BENCHMARK_PLANNERS;
      if (run_benchmark_ui_.benchmark_check_reachability_checkbox->isChecked())
        btype += moveit_benchmarks::BENCHMARK_GOAL_EXISTANCE;

      if (be.readOptions(outfilename.toStdString()))
      {
        std::stringstream ss;
        be.printOptions(ss);
        ROS_INFO_STREAM("Calling benchmark with options:" << std::endl << ss.str() << std::endl);

        BenchmarkProcessingThread benchmark_thread(be, btype, this);
        benchmark_thread.startAndShow();

        if (benchmark_thread.isRunning())
        {
          benchmark_thread.terminate();
          benchmark_thread.wait();
          QMessageBox::warning(this, "", "Benchmark computation canceled");
        }
        else
        {
          QMessageBox::information(
              this, "Benchmark computation finished",
              QString("The results were logged into '").append(run_benchmark_ui_.benchmark_output_folder_text->text()));
        }
      }

      break;
    }
  }
}

void MainWindow::loadBenchmarkResults(void)
{
  // Select a log file of the set.
  QString path =
      QFileDialog::getOpenFileName(this, tr("Select a log file in the set"), tr(""), tr("Log files (*.log)"));
  if (!path.isEmpty())
  {
    JobProcessing::addBackgroundJob(boost::bind(&MainWindow::computeLoadBenchmarkResults, this, path.toStdString()));
  }
}

void MainWindow::computeLoadBenchmarkResults(const std::string& file)
{
  std::string logid, basename, logid_text;
  try
  {
    logid = file.substr(file.find_last_of(".", file.length() - 5) + 1,
                        file.find_last_of(".") - file.find_last_of(".", file.length() - 5) - 1);
    basename = file.substr(0, file.find_last_of(".", file.length() - 5));
    logid_text = logid.substr(logid.find_first_of("_"), logid.length() - logid.find_first_of("_"));
  }
  catch (...)
  {
    ROS_ERROR("Invalid benchmark log file. Cannot load results.");
    return;
  }

  int count = 1;
  bool more_files = true;
  // Parse all the log files in the set
  while (more_files)
  {
    std::ifstream ifile;
    std::stringstream file_to_load;
    file_to_load << basename << "." << count << logid_text << ".log";

    ifile.open(file_to_load.str().c_str());
    if (ifile.good())
    {
      // Parse results
      char text_line[512];
      static std::streamsize text_line_length = 512;
      bool valid_file = false;
      ifile.getline(text_line, text_line_length);

      // Check if the file is valid. The first line must contain "Experiment <scene>". There must be a "total_time"
      // line, followed by the results
      if (ifile.good() && strstr(text_line, "Experiment") != NULL && strlen(text_line) > 11 &&
          strncmp(&text_line[11], scene_display_->getPlanningSceneRO()->getName().c_str(),
                  scene_display_->getPlanningSceneRO()->getName().length()) == 0)
      {
        while (ifile.good())
        {
          if (strstr(text_line, "total_time REAL") != NULL)
          {
            valid_file = true;
            break;
          }

          ifile.getline(text_line, text_line_length);
        }
      }
      else
      {
        ROS_ERROR("Not a valid log file, or a different planning scene loaded");
      }

      if (valid_file)
      {
        if (basename.find(".trajectory") != std::string::npos)
        {
          // Trajectory results may contain multiple goals
          int nwaypoint = 0;
          while (ifile.getline(text_line, text_line_length) && ifile.good() && strlen(text_line) > 6)
          {
            // This should be a reachability results line composed of reachable, collision-free and time fields (bool;
            // bool; float)
            try
            {
              bool reachable = boost::lexical_cast<bool>(text_line[0]);
              bool collision_free = boost::lexical_cast<bool>(text_line[3]);

              // Update colors accordingly
              if (count <= trajectories_.size())
              {
                std::string trajectory_name = ui_.trajectory_list->item(count - 1)->text().toStdString();
                if (nwaypoint < trajectories_[trajectory_name]->waypoint_markers.size())
                {
                  if (reachable)
                  {
                    if (collision_free)
                    {
                      // Reachable and collision-free
                      JobProcessing::addMainLoopJob(boost::bind(
                          &MainWindow::updateMarkerState, this,
                          trajectories_[trajectory_name]->waypoint_markers[nwaypoint], GripperMarker::REACHABLE));
                    }
                    else
                    {
                      // Reachable, but in collision
                      JobProcessing::addMainLoopJob(boost::bind(
                          &MainWindow::updateMarkerState, this,
                          trajectories_[trajectory_name]->waypoint_markers[nwaypoint], GripperMarker::IN_COLLISION));
                    }
                  }
                  else
                  {
                    // Not reachable
                    JobProcessing::addMainLoopJob(boost::bind(
                        &MainWindow::updateMarkerState, this,
                        trajectories_[trajectory_name]->waypoint_markers[nwaypoint], GripperMarker::NOT_REACHABLE));
                  }
                }
              }
              nwaypoint++;
            }
            catch (...)
            {
              ROS_ERROR("Error parsing the log file");
            }
          }
        }
        else
        {
          ifile.getline(text_line, text_line_length);
          if (ifile.good() && strlen(text_line) > 6)
          {
            // This should be the reachability results line composed of reachable, collision-free and time fields (bool;
            // bool; float)
            try
            {
              bool reachable = boost::lexical_cast<bool>(text_line[0]);
              bool collision_free = boost::lexical_cast<bool>(text_line[3]);

              // Update colors accordingly
              if (count <= goal_poses_.size())
              {
                std::string goal_name = ui_.goal_poses_list->item(count - 1)->text().toStdString();
                if (reachable)
                {
                  if (collision_free)
                  {
                    // Reachable and collision-free
                    JobProcessing::addMainLoopJob(boost::bind(&MainWindow::updateGoalMarkerStateFromName, this,
                                                              goal_name, GripperMarker::REACHABLE));
                  }
                  else
                  {
                    // Reachable, but in collision
                    JobProcessing::addMainLoopJob(boost::bind(&MainWindow::updateGoalMarkerStateFromName, this,
                                                              goal_name, GripperMarker::IN_COLLISION));
                  }
                }
                else
                {
                  // Not reachable
                  JobProcessing::addMainLoopJob(boost::bind(&MainWindow::updateGoalMarkerStateFromName, this, goal_name,
                                                            GripperMarker::NOT_REACHABLE));
                }
              }
            }
            catch (...)
            {
              ROS_ERROR("Error parsing the log file");
            }
          }
        }
      }
      else
      {
        ROS_ERROR("Invalid benchmark log file. Cannot load results.");
      }

      ifile.close();
    }
    else
    {
      more_files = false;
    }
    count++;
  }
}

void MainWindow::updateGoalMarkerStateFromName(const std::string& name, const GripperMarker::GripperMarkerState& state)
{
  if (goal_poses_.find(name) != goal_poses_.end())
    goal_poses_[name]->setState(state);
}

void MainWindow::updateMarkerState(GripperMarkerPtr marker, const GripperMarker::GripperMarkerState& state)
{
  marker->setState(state);
}

void MainWindow::goalOffsetChanged()
{
  goal_offset_.setIdentity();
  goal_offset_ = Eigen::AngleAxisd(ui_.goal_offset_roll->value() * boost::math::constants::pi<double>() / 180.0,
                                   Eigen::Vector3d::UnitX()) *
                 Eigen::AngleAxisd(ui_.goal_offset_pitch->value() * boost::math::constants::pi<double>() / 180.0,
                                   Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(ui_.goal_offset_yaw->value() * boost::math::constants::pi<double>() / 180.0,
                                   Eigen::Vector3d::UnitZ());

  if (!robot_interaction_->getActiveEndEffectors().empty())
  {
    Eigen::Affine3d current_pose;
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
    {
      current_pose = goals_initial_pose_[it->first] * goal_offset_;
      it->second->setPose(current_pose);
    }
  }
}

}  // namespace
