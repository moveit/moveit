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
 *
 * Author: Mario Prats, Ioan Sucan
 */

#include <main_window.h>
#include <trajectory.h>
#include <job_processing.h>
#include <ui_utils.h>

#include <eigen_conversions/eigen_msg.h>

#include <QMessageBox>
#include <QInputDialog>

#include <boost/math/constants/constants.hpp>

namespace benchmark_tool
{

void MainWindow::createTrajectoryButtonClicked(void)
{
  std::stringstream ss;

  {
    const planning_scene_monitor::LockedPlanningSceneRO &ps = scene_display_->getPlanningSceneRO();
    if ( ! ps || robot_interaction_->getActiveEndEffectors().empty() )
    {
      if ( ! ps ) ROS_ERROR("Not planning scene");
      if ( robot_interaction_->getActiveEndEffectors().empty() ) ROS_ERROR("No end effector");
      return;
    }
    else
      ss << ps->getName().c_str() << "_trajectory_" << std::setfill('0') << std::setw(4) << trajectories_.size();
  }

  bool ok = false;
  QString text = QInputDialog::getText(this, tr("Choose a name"),
                                       tr("Trajectory name:"), QLineEdit::Normal,
                                       QString(ss.str().c_str()), &ok);

  std::string name;
  if (ok)
  {
    if ( ! text.isEmpty() )
    {
      name = text.toStdString();
      if (goal_poses_.find(name) != goal_poses_.end())
        QMessageBox::warning(this, "Name already exists", QString("The name '").append(name.c_str()).
                             append("' already exists. Not creating trajectory."));
      else
      {
        //Create the new trajectory starting point at the current eef pose, and attach an interactive marker to it
        Eigen::Affine3d tip_pose = scene_display_->getPlanningSceneRO()->getCurrentState().getLinkState(robot_interaction_->getActiveEndEffectors()[0].parent_link)->getGlobalLinkTransform();
        geometry_msgs::Pose marker_pose;
        tf::poseEigenToMsg(tip_pose, marker_pose);
        static const float marker_scale = 0.15;

        TrajectoryPtr trajectory_marker( new Trajectory(scene_display_->getPlanningSceneRO()->getCurrentState(), scene_display_->getSceneNode(), visualization_manager_,
                                                              name, scene_display_->getRobotModel()->getModelFrame(),
                                                              robot_interaction_->getActiveEndEffectors()[0], marker_pose, marker_scale, GripperMarker::NOT_TESTED,
                                                              ui_.trajectory_nwaypoints_spin->value()));

        trajectories_.insert(TrajectoryPair(name,  trajectory_marker));
      }
    }
    else
      QMessageBox::warning(this, "Goal not created", "Cannot use an empty name for a new goal pose.");
  }

  populateTrajectoriesList();
  selectLastItemInList(ui_.trajectory_list);
}

void MainWindow::populateTrajectoriesList(void)
{
  bool old_signal_state = ui_.trajectory_list->blockSignals(true);
  ui_.trajectory_list->clear();
  for (TrajectoryMap::iterator it = trajectories_.begin(); it != trajectories_.end(); ++it)
  {
    QListWidgetItem *item = new QListWidgetItem(QString(it->first.c_str()));
    ui_.trajectory_list->addItem(item);
  }
  ui_.trajectory_list->blockSignals(old_signal_state);
}

void MainWindow::trajectorySelectionChanged(void)
{
  for (unsigned int i = 0; i < ui_.trajectory_list->count() ; ++i)
  {
    QListWidgetItem *item = ui_.trajectory_list->item(i);
    std::string name = item->text().toStdString();
    if ( trajectories_.find(name) != trajectories_.end() && item->isSelected())
    {
      trajectories_[name]->show(scene_display_->getSceneNode(), visualization_manager_);
    }
    else
    {
      trajectories_[name]->hide();
    }
  }
}

void MainWindow::removeTrajectoryButtonClicked(void)
{
  if (ui_.trajectory_list->currentItem())
  {
    //Warn the user
    QMessageBox msgBox;
    msgBox.setText("The selected trajectory will be removed from the database");
    msgBox.setInformativeText("Do you want to continue?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::No);
    int ret = msgBox.exec();

    switch (ret)
    {
      case QMessageBox::Yes:
      {
        //Go through the list of trajectories, and delete those selected
        QList<QListWidgetItem*> found_items = ui_.trajectory_list->selectedItems();
        for ( std::size_t i = 0 ; i < found_items.size() ; i++ )
        {
          try
          {
            trajectory_constraints_storage_->removeTrajectoryConstraints(found_items[i]->text().toStdString());
          }
          catch (std::runtime_error &ex)
          {
            ROS_ERROR("%s", ex.what());
          }
          trajectories_.erase(found_items[i]->text().toStdString());
        }
        populateTrajectoriesList();
      }
      break;
    }
  }
}

void MainWindow::loadTrajectoriesFromDBButtonClicked(void)
{
  //Get all the trajectory constraints from the database
  if (trajectory_constraints_storage_ && !robot_interaction_->getActiveEndEffectors().empty())
  {
    //First clear the current list
    trajectories_.clear();

    std::vector<std::string> names;
    try
    {
      trajectory_constraints_storage_->getKnownTrajectoryConstraints(ui_.trajectories_filter_text->text().toStdString(), names);
    }
    catch (std::runtime_error &ex)
    {
      QMessageBox::warning(this, "Cannot query the database", QString("Wrongly formatted regular expression for trajectory contraints: ").append(ex.what()));
      return;
    }

    for (unsigned int i = 0 ; i < names.size() ; i++)
    {
      //Create a trajectory constraint
      moveit_warehouse::TrajectoryConstraintsWithMetadata tc;
      bool got_constraint = false;
      try
      {
        got_constraint = trajectory_constraints_storage_->getTrajectoryConstraints(tc, names[i]);
      }
      catch (std::runtime_error &ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      if (!got_constraint)
        continue;

      if (tc->constraints.size() > 0 && tc->constraints[0].position_constraints[0].constraint_region.primitive_poses.size() > 0 && tc->constraints[0].orientation_constraints.size() > 0)
      {
        geometry_msgs::Pose shape_pose;
        shape_pose.position = tc->constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position;
        shape_pose.orientation = tc->constraints[0].orientation_constraints[0].orientation;
        static const float marker_scale = 0.15;
        TrajectoryPtr trajectory_marker( new Trajectory(scene_display_->getPlanningSceneRO()->getCurrentState(), scene_display_->getSceneNode(), visualization_manager_,
                                                        names[i], scene_display_->getRobotModel()->getModelFrame(),
                                                        robot_interaction_->getActiveEndEffectors()[0], shape_pose, marker_scale, GripperMarker::NOT_TESTED,
                                                        ui_.trajectory_nwaypoints_spin->value()));

        if ( trajectories_.find(names[i]) != trajectories_.end() )
        {
          trajectories_.erase(names[i]);
        }
        trajectories_.insert(TrajectoryPair(names[i], trajectory_marker));

        for (std::size_t c = 0; c < tc->constraints.size(); ++c)
        {
          if (tc->constraints[c].position_constraints.size() > 0 && tc->constraints[c].position_constraints[0].constraint_region.primitive_poses.size() > 0 && tc->constraints[c].orientation_constraints.size() > 0 )
          {
            shape_pose.position = tc->constraints[c].position_constraints[0].constraint_region.primitive_poses[0].position;
            shape_pose.orientation = tc->constraints[c].orientation_constraints[0].orientation;

            static const float marker_scale = 0.15;
            GripperMarkerPtr waypoint_marker( new GripperMarker(scene_display_->getPlanningSceneRO()->getCurrentState(), scene_display_->getSceneNode(), visualization_manager_,
                                                                names[i], scene_display_->getRobotModel()->getModelFrame(),
                                                                robot_interaction_->getActiveEndEffectors()[0], shape_pose, marker_scale, GripperMarker::NOT_TESTED));
            waypoint_marker->unselect(true);
            waypoint_marker->setColor(0.0, 0.9, 0.0, 1 - (double)c / (double)tc->constraints.size());
            trajectory_marker->waypoint_markers.push_back(waypoint_marker);
          }
        }

        if (trajectory_marker->waypoint_markers.size() > 0)
        {
          trajectory_marker->start_marker = GripperMarkerPtr(new GripperMarker(*trajectory_marker->waypoint_markers.front()));
          trajectory_marker->end_marker = GripperMarkerPtr(new GripperMarker(*trajectory_marker->waypoint_markers.back()));
          trajectory_marker->waypoint_markers.front()->getPose(trajectory_marker->control_marker_start_pose);
          trajectory_marker->waypoint_markers.back()->getPose(trajectory_marker->control_marker_end_pose);
        }
        populateTrajectoriesList();
        selectFirstItemInList(ui_.trajectory_list);
      }
    }
   }
  else
  {
    if (!trajectory_constraints_storage_)
      QMessageBox::warning(this, "Warning", "Not connected to a database.");
  }
}

void MainWindow::saveTrajectoriesOnDBButtonClicked(void)
{
  if (trajectory_constraints_storage_)
  {
    //Convert all goal trajectory markers into constraints and store them
    for (TrajectoryMap::iterator it = trajectories_.begin(); it != trajectories_.end(); ++it)
    {
      moveit_msgs::TrajectoryConstraints tc;

      for (std::size_t w = 0; w < it->second->waypoint_markers.size(); ++w)
      {
        moveit_msgs::Constraints c;
        c.name = it->second->waypoint_markers[w]->getName();

        shape_msgs::SolidPrimitive sp;
        sp.type = sp.BOX;
        sp.dimensions.resize(3, std::numeric_limits<float>::epsilon() * 10.0);

        moveit_msgs::PositionConstraint pc;
        pc.constraint_region.primitives.push_back(sp);
        geometry_msgs::Pose posemsg;
        it->second->waypoint_markers[w]->getPosition(posemsg.position);
        posemsg.orientation.x = 0.0;
        posemsg.orientation.y = 0.0;
        posemsg.orientation.z = 0.0;
        posemsg.orientation.w = 1.0;
        pc.constraint_region.primitive_poses.push_back(posemsg);
        pc.weight = 1.0;
        c.position_constraints.push_back(pc);

        moveit_msgs::OrientationConstraint oc;
        it->second->waypoint_markers[w]->getOrientation(oc.orientation);
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance =
            oc.absolute_z_axis_tolerance = std::numeric_limits<float>::epsilon() * 10.0;
        oc.weight = 1.0;
        c.orientation_constraints.push_back(oc);
        tc.constraints.push_back(c);
      }

      try
      {
        trajectory_constraints_storage_->addTrajectoryConstraints(tc, it->first);
      }
      catch (std::runtime_error &ex)
      {
        ROS_ERROR("Cannot save trajectory constraint: %s", ex.what());
      }
    }
  }
  else
  {
    QMessageBox::warning(this, "Warning", "Not connected to a database.");
  }
}

void MainWindow::trajectoryNWaypointsChanged(int n)
{
  if (ui_.trajectory_list->currentItem())
  {
    TrajectoryMap::iterator it = trajectories_.find(ui_.trajectory_list->currentItem()->text().toStdString());
    it->second->setNumberOfWaypoints(n);
  }
}

void MainWindow::trajectoryExecuteButtonClicked()
{
  if (ui_.trajectory_list->currentItem())
  {
    TrajectoryMap::iterator it = trajectories_.find(ui_.trajectory_list->currentItem()->text().toStdString());
    EigenSTL::vector_Affine3d waypoint_poses;
    for (std::size_t w = 1; w < it->second->waypoint_markers.size(); ++w )
    {
      Eigen::Affine3d pose;
      it->second->waypoint_markers[w]->getPose(pose);
      waypoint_poses.push_back(pose);
    }

    if (waypoint_poses.size() > 0)
    {
      robot_state::JointStateGroup *jsg = scene_display_->getPlanningSceneRW()->getCurrentStateNonConst().getJointStateGroup(ui_.planning_group_combo->currentText().toStdString());

      std::vector<boost::shared_ptr<robot_state::RobotState> > traj;
      double completed = jsg->computeCartesianPath(traj, robot_interaction_->getActiveEndEffectors()[0].parent_link, waypoint_poses, true, 0.04, 0.0);

      ROS_INFO_STREAM("Trajectory completion percentage " << completed);
      JobProcessing::addBackgroundJob(boost::bind(&MainWindow::animateTrajectory, this, traj));
    }
  }
}

void MainWindow::animateTrajectory(const std::vector<boost::shared_ptr<robot_state::RobotState> > &traj)
{
  for (std::size_t i = 0; i < traj.size(); ++i)
  {
    scene_display_->getPlanningSceneRW()->setCurrentState(*traj[i]);
    scene_display_->queueRenderSceneGeometry();
    usleep(100000);
  }
}

} // namespace
