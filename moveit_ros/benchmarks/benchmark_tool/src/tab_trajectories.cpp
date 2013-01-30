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
                                                              name, scene_display_->getKinematicModel()->getModelFrame(),
                                                              robot_interaction_->getActiveEndEffectors()[0], marker_pose, marker_scale, GripperMarker::NOT_TESTED));

        trajectories_.insert(TrajectoryPair(name,  trajectory_marker));
      }
    }
    else
      QMessageBox::warning(this, "Goal not created", "Cannot use an empty name for a new goal pose.");
  }

  populateTrajectoriesList();
}

void MainWindow::populateTrajectoriesList(void)
{
  ui_.trajectory_list->clear();
  for (TrajectoryMap::iterator it = trajectories_.begin(); it != trajectories_.end(); ++it)
  {
    QListWidgetItem *item = new QListWidgetItem(QString(it->first.c_str()));
    ui_.trajectory_list->addItem(item);
  }
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

} // namespace
