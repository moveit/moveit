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

/* Author: Ioan Sucan, Mario Prats */

#include <moveit/warehouse/planning_scene_storage.h>

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>

#include <interactive_markers/tools.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/window_manager_interface.h>

#include <tf2_eigen/tf2_eigen.h>
#include <geometric_shapes/shape_operations.h>

#include <QMessageBox>
#include <QInputDialog>
#include <QFileDialog>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{
void MotionPlanningFrame::importFileButtonClicked()
{
  QString path = QFileDialog::getOpenFileName(this, tr("Import Object"));
  if (!path.isEmpty())
    importResource("file://" + path.toStdString());
}

void MotionPlanningFrame::importUrlButtonClicked()
{
  bool ok = false;
  QString url = QInputDialog::getText(this, tr("Import Object"), tr("URL for file to import:"), QLineEdit::Normal,
                                      QString("http://"), &ok);
  if (ok && !url.isEmpty())
    importResource(url.toStdString());
}

void MotionPlanningFrame::clearSceneButtonClicked()
{
  planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
  if (ps)
  {
    ps->getWorldNonConst()->clearObjects();
    ps->getCurrentStateNonConst().clearAttachedBodies();
    moveit_msgs::PlanningScene msg;
    ps->getPlanningSceneMsg(msg);
    planning_scene_publisher_.publish(msg);
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateCollisionObjectsList, this));
    planning_display_->queueRenderSceneGeometry();
  }
}

void MotionPlanningFrame::sceneScaleChanged(int value)
{
  if (scaled_object_)
  {
    planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
    if (ps)
    {
      if (ps->getWorld()->hasObject(scaled_object_->id_))
      {
        ps->getWorldNonConst()->removeObject(scaled_object_->id_);
        for (std::size_t i = 0; i < scaled_object_->shapes_.size(); ++i)
        {
          shapes::Shape* s = scaled_object_->shapes_[i]->clone();
          s->scale((double)value / 100.0);
          ps->getWorldNonConst()->addToObject(scaled_object_->id_, shapes::ShapeConstPtr(s),
                                              scaled_object_->shape_poses_[i]);
        }
        planning_display_->queueRenderSceneGeometry();
      }
      else
        scaled_object_.reset();
    }
    else
      scaled_object_.reset();
  }
}

void MotionPlanningFrame::sceneScaleStartChange()
{
  QList<QListWidgetItem*> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
    return;
  if (planning_display_->getPlanningSceneMonitor() && sel[0]->checkState() == Qt::Unchecked)
  {
    planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
    if (ps)
    {
      scaled_object_ = ps->getWorld()->getObject(sel[0]->text().toStdString());
    }
  }
}

void MotionPlanningFrame::sceneScaleEndChange()
{
  scaled_object_.reset();
  ui_->scene_scale->setSliderPosition(100);
}

void MotionPlanningFrame::removeObjectButtonClicked()
{
  QList<QListWidgetItem*> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
    return;
  planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
  if (ps)
  {
    for (int i = 0; i < sel.count(); ++i)
      if (sel[i]->checkState() == Qt::Unchecked)
        ps->getWorldNonConst()->removeObject(sel[i]->text().toStdString());
      else
        ps->getCurrentStateNonConst().clearAttachedBody(sel[i]->text().toStdString());
    scene_marker_.reset();
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateCollisionObjectsList, this));
    planning_display_->queueRenderSceneGeometry();
  }
}

static QString decideStatusText(const collision_detection::CollisionWorld::ObjectConstPtr& obj)
{
  QString status_text = "'" + QString::fromStdString(obj->id_) + "' is a collision object with ";
  if (obj->shapes_.empty())
    status_text += "no geometry";
  else
  {
    std::vector<QString> shape_names;
    for (std::size_t i = 0; i < obj->shapes_.size(); ++i)
      shape_names.push_back(QString::fromStdString(shapes::shapeStringName(obj->shapes_[i].get())));
    if (shape_names.size() == 1)
      status_text += "one " + shape_names[0];
    else
    {
      status_text += QString::fromStdString(boost::lexical_cast<std::string>(shape_names.size())) + " shapes:";
      for (std::size_t i = 0; i < shape_names.size(); ++i)
        status_text += " " + shape_names[i];
    }
  }
  return status_text;
}

static QString decideStatusText(const robot_state::AttachedBody* attached_body)
{
  QString status_text = "'" + QString::fromStdString(attached_body->getName()) + "' is attached to '" +
                        QString::fromStdString(attached_body->getAttachedLinkName()) + "'";
  return status_text;
}

void MotionPlanningFrame::selectedCollisionObjectChanged()
{
  QList<QListWidgetItem*> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
  {
    bool oldState = ui_->object_x->blockSignals(true);
    ui_->object_x->setValue(0.0);
    ui_->object_x->blockSignals(oldState);

    oldState = ui_->object_y->blockSignals(true);
    ui_->object_y->setValue(0.0);
    ui_->object_y->blockSignals(oldState);

    oldState = ui_->object_z->blockSignals(true);
    ui_->object_z->setValue(0.0);
    ui_->object_z->blockSignals(oldState);

    oldState = ui_->object_rx->blockSignals(true);
    ui_->object_rx->setValue(0.0);
    ui_->object_rx->blockSignals(oldState);

    oldState = ui_->object_ry->blockSignals(true);
    ui_->object_ry->setValue(0.0);
    ui_->object_ry->blockSignals(oldState);

    oldState = ui_->object_rz->blockSignals(true);
    ui_->object_rz->setValue(0.0);
    ui_->object_rz->blockSignals(oldState);

    ui_->object_status->setText("");
    scene_marker_.reset();
    ui_->scene_scale->setEnabled(false);
  }
  else if (planning_display_->getPlanningSceneMonitor())
  {
    // if this is a CollisionWorld element
    if (sel[0]->checkState() == Qt::Unchecked)
    {
      ui_->scene_scale->setEnabled(true);
      bool update_scene_marker = false;
      Eigen::Affine3d obj_pose;
      {
        const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
        const collision_detection::CollisionWorld::ObjectConstPtr& obj =
            ps->getWorld()->getObject(sel[0]->text().toStdString());
        if (obj)
        {
          ui_->object_status->setText(decideStatusText(obj));

          if (obj->shapes_.size() == 1)
          {
            obj_pose = obj->shape_poses_[0];
            Eigen::Vector3d xyz = obj_pose.rotation().eulerAngles(0, 1, 2);
            update_scene_marker = true;  // do the marker update outside locked scope to avoid deadlock

            bool oldState = ui_->object_x->blockSignals(true);
            ui_->object_x->setValue(obj_pose.translation()[0]);
            ui_->object_x->blockSignals(oldState);

            oldState = ui_->object_y->blockSignals(true);
            ui_->object_y->setValue(obj_pose.translation()[1]);
            ui_->object_y->blockSignals(oldState);

            oldState = ui_->object_z->blockSignals(true);
            ui_->object_z->setValue(obj_pose.translation()[2]);
            ui_->object_z->blockSignals(oldState);

            oldState = ui_->object_rx->blockSignals(true);
            ui_->object_rx->setValue(xyz[0]);
            ui_->object_rx->blockSignals(oldState);

            oldState = ui_->object_ry->blockSignals(true);
            ui_->object_ry->setValue(xyz[1]);
            ui_->object_ry->blockSignals(oldState);

            oldState = ui_->object_rz->blockSignals(true);
            ui_->object_rz->setValue(xyz[2]);
            ui_->object_rz->blockSignals(oldState);
          }
        }
        else
          ui_->object_status->setText("ERROR: '" + sel[0]->text() + "' should be a collision object but it is not");
      }
      if (update_scene_marker && ui_->tabWidget->tabText(ui_->tabWidget->currentIndex()).toStdString() == TAB_OBJECTS)
      {
        createSceneInteractiveMarker();
      }
    }
    else
    {
      ui_->scene_scale->setEnabled(false);
      // if it is an attached object
      scene_marker_.reset();
      const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
      const robot_state::AttachedBody* attached_body =
          ps->getCurrentState().getAttachedBody(sel[0]->text().toStdString());
      if (attached_body)
        ui_->object_status->setText(decideStatusText(attached_body));
      else
        ui_->object_status->setText("ERROR: '" + sel[0]->text() + "' should be an attached object but it is not");
    }
  }
}

void MotionPlanningFrame::objectPoseValueChanged(double /* value */)
{
  updateCollisionObjectPose(true);
}

void MotionPlanningFrame::updateCollisionObjectPose(bool update_marker_position)
{
  QList<QListWidgetItem*> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
    return;
  planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
  if (ps)
  {
    collision_detection::CollisionWorld::ObjectConstPtr obj = ps->getWorld()->getObject(sel[0]->text().toStdString());
    if (obj && obj->shapes_.size() == 1)
    {
      Eigen::Affine3d p;
      p.translation()[0] = ui_->object_x->value();
      p.translation()[1] = ui_->object_y->value();
      p.translation()[2] = ui_->object_z->value();

      p = Eigen::Translation3d(p.translation()) *
          (Eigen::AngleAxisd(ui_->object_rx->value(), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(ui_->object_ry->value(), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(ui_->object_rz->value(), Eigen::Vector3d::UnitZ()));

      ps->getWorldNonConst()->moveShapeInObject(obj->id_, obj->shapes_[0], p);
      planning_display_->queueRenderSceneGeometry();

      // Update the interactive marker pose to match the manually introduced one
      if (update_marker_position && scene_marker_)
      {
        Eigen::Quaterniond eq(p.rotation());
        scene_marker_->setPose(Ogre::Vector3(ui_->object_x->value(), ui_->object_y->value(), ui_->object_z->value()),
                               Ogre::Quaternion(eq.w(), eq.x(), eq.y(), eq.z()), "");
      }
    }
  }
}

void MotionPlanningFrame::collisionObjectChanged(QListWidgetItem* item)
{
  if (item->type() < (int)known_collision_objects_.size() && planning_display_->getPlanningSceneMonitor())
  {
    // if we have a name change
    if (known_collision_objects_[item->type()].first != item->text().toStdString())
      renameCollisionObject(item);
    else
    {
      bool checked = item->checkState() == Qt::Checked;
      if (known_collision_objects_[item->type()].second != checked)
        attachDetachCollisionObject(item);
    }
  }
}

/* Receives feedback from the interactive marker and updates the shape pose in the world accordingly */
void MotionPlanningFrame::imProcessFeedback(visualization_msgs::InteractiveMarkerFeedback& feedback)
{
  bool oldState = ui_->object_x->blockSignals(true);
  ui_->object_x->setValue(feedback.pose.position.x);
  ui_->object_x->blockSignals(oldState);

  oldState = ui_->object_y->blockSignals(true);
  ui_->object_y->setValue(feedback.pose.position.y);
  ui_->object_y->blockSignals(oldState);

  oldState = ui_->object_z->blockSignals(true);
  ui_->object_z->setValue(feedback.pose.position.z);
  ui_->object_z->blockSignals(oldState);

  Eigen::Quaterniond q;
  tf2::fromMsg(feedback.pose.orientation, q);
  Eigen::Vector3d xyz = q.matrix().eulerAngles(0, 1, 2);

  oldState = ui_->object_rx->blockSignals(true);
  ui_->object_rx->setValue(xyz[0]);
  ui_->object_rx->blockSignals(oldState);

  oldState = ui_->object_ry->blockSignals(true);
  ui_->object_ry->setValue(xyz[1]);
  ui_->object_ry->blockSignals(oldState);

  oldState = ui_->object_rz->blockSignals(true);
  ui_->object_rz->setValue(xyz[2]);
  ui_->object_rz->blockSignals(oldState);

  updateCollisionObjectPose(false);
}

void MotionPlanningFrame::copySelectedCollisionObject()
{
  QList<QListWidgetItem*> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
    return;

  planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
  if (!ps)
    return;

  for (int i = 0; i < sel.size(); ++i)
  {
    std::string name = sel[i]->text().toStdString();
    collision_detection::CollisionWorld::ObjectConstPtr obj = ps->getWorld()->getObject(name);
    if (!obj)
      continue;

    // find a name for the copy
    name = "Copy of " + name;
    if (ps->getWorld()->hasObject(name))
    {
      name += " ";
      unsigned int n = 1;
      while (ps->getWorld()->hasObject(name + boost::lexical_cast<std::string>(n)))
        n++;
      name += boost::lexical_cast<std::string>(n);
    }
    ps->getWorldNonConst()->addToObject(name, obj->shapes_, obj->shape_poses_);
    ROS_DEBUG("Copied collision object to '%s'", name.c_str());
  }
  planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateCollisionObjectsList, this));
}

void MotionPlanningFrame::computeSaveSceneButtonClicked()
{
  if (planning_scene_storage_)
  {
    moveit_msgs::PlanningScene msg;
    planning_display_->getPlanningSceneRO()->getPlanningSceneMsg(msg);
    try
    {
      planning_scene_storage_->removePlanningScene(msg.name);
      planning_scene_storage_->addPlanningScene(msg);
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populatePlanningSceneTreeView, this));
  }
}

void MotionPlanningFrame::computeSaveQueryButtonClicked(const std::string& scene, const std::string& query_name)
{
  moveit_msgs::MotionPlanRequest mreq;
  constructPlanningRequest(mreq);
  if (planning_scene_storage_)
  {
    try
    {
      if (!query_name.empty())
        planning_scene_storage_->removePlanningQuery(scene, query_name);
      planning_scene_storage_->addPlanningQuery(mreq, scene, query_name);
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populatePlanningSceneTreeView, this));
  }
}

void MotionPlanningFrame::computeDeleteSceneButtonClicked()
{
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem*> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem* s = sel.front();
      if (s->type() == ITEM_TYPE_SCENE)
      {
        std::string scene = s->text(0).toStdString();
        try
        {
          planning_scene_storage_->removePlanningScene(scene);
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
      }
      else
      {
        // if we selected a query name, then we overwrite that query
        std::string scene = s->parent()->text(0).toStdString();
        try
        {
          planning_scene_storage_->removePlanningScene(scene);
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
      }
      planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populatePlanningSceneTreeView, this));
    }
  }
}

void MotionPlanningFrame::computeDeleteQueryButtonClicked()
{
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem*> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem* s = sel.front();
      if (s->type() == ITEM_TYPE_QUERY)
      {
        std::string scene = s->parent()->text(0).toStdString();
        std::string query_name = s->text(0).toStdString();
        try
        {
          planning_scene_storage_->removePlanningQuery(scene, query_name);
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
        planning_display_->addMainLoopJob(
            boost::bind(&MotionPlanningFrame::computeDeleteQueryButtonClickedHelper, this, s));
      }
    }
  }
}

void MotionPlanningFrame::computeDeleteQueryButtonClickedHelper(QTreeWidgetItem* s)
{
  ui_->planning_scene_tree->setUpdatesEnabled(false);
  s->parent()->removeChild(s);
  ui_->planning_scene_tree->setUpdatesEnabled(true);
}

void MotionPlanningFrame::checkPlanningSceneTreeEnabledButtons()
{
  QList<QTreeWidgetItem*> sel = ui_->planning_scene_tree->selectedItems();
  if (sel.empty())
  {
    ui_->load_scene_button->setEnabled(false);
    ui_->load_query_button->setEnabled(false);
    ui_->save_query_button->setEnabled(false);
    ui_->delete_scene_button->setEnabled(false);
  }
  else
  {
    ui_->save_query_button->setEnabled(true);

    QTreeWidgetItem* s = sel.front();

    // if the item is a PlanningScene
    if (s->type() == ITEM_TYPE_SCENE)
    {
      ui_->load_scene_button->setEnabled(true);
      ui_->load_query_button->setEnabled(false);
      ui_->delete_scene_button->setEnabled(true);
      ui_->delete_query_button->setEnabled(false);
      ui_->save_query_button->setEnabled(true);
    }
    else
    {
      // if the item is a query
      ui_->load_scene_button->setEnabled(false);
      ui_->load_query_button->setEnabled(true);
      ui_->delete_scene_button->setEnabled(false);
      ui_->delete_query_button->setEnabled(true);
    }
  }
}

void MotionPlanningFrame::computeLoadSceneButtonClicked()
{
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem*> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem* s = sel.front();
      if (s->type() == ITEM_TYPE_SCENE)
      {
        std::string scene = s->text(0).toStdString();
        ROS_DEBUG("Attempting to load scene '%s'", scene.c_str());
        moveit_warehouse::PlanningSceneWithMetadata scene_m;
        bool got_ps = false;
        try
        {
          got_ps = planning_scene_storage_->getPlanningScene(scene_m, scene);
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("%s", ex.what());
        }

        if (got_ps)
        {
          ROS_INFO("Loaded scene '%s'", scene.c_str());
          if (planning_display_->getPlanningSceneMonitor())
          {
            if (scene_m->robot_model_name != planning_display_->getRobotModel()->getName())
            {
              ROS_INFO("Scene '%s' was saved for robot '%s' but we are using robot '%s'. Using scene geometry only",
                       scene.c_str(), scene_m->robot_model_name.c_str(),
                       planning_display_->getRobotModel()->getName().c_str());
              planning_scene_world_publisher_.publish(scene_m->world);
              // publish the parts that are not in the world
              moveit_msgs::PlanningScene diff;
              diff.is_diff = true;
              diff.name = scene_m->name;
              planning_scene_publisher_.publish(diff);
            }
            else
              planning_scene_publisher_.publish(static_cast<const moveit_msgs::PlanningScene&>(*scene_m));
          }
          else
            planning_scene_publisher_.publish(static_cast<const moveit_msgs::PlanningScene&>(*scene_m));
        }
        else
          ROS_WARN("Failed to load scene '%s'. Has the message format changed since the scene was saved?",
                   scene.c_str());
      }
    }
  }
}

void MotionPlanningFrame::computeLoadQueryButtonClicked()
{
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem*> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem* s = sel.front();
      if (s->type() == ITEM_TYPE_QUERY)
      {
        std::string scene = s->parent()->text(0).toStdString();
        std::string query_name = s->text(0).toStdString();
        moveit_warehouse::MotionPlanRequestWithMetadata mp;
        bool got_q = false;
        try
        {
          got_q = planning_scene_storage_->getPlanningQuery(mp, scene, query_name);
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("%s", ex.what());
        }

        if (got_q)
        {
          robot_state::RobotStatePtr start_state(new robot_state::RobotState(*planning_display_->getQueryStartState()));
          robot_state::robotStateMsgToRobotState(planning_display_->getPlanningSceneRO()->getTransforms(),
                                                 mp->start_state, *start_state);
          planning_display_->setQueryStartState(*start_state);

          robot_state::RobotStatePtr goal_state(new robot_state::RobotState(*planning_display_->getQueryGoalState()));
          for (std::size_t i = 0; i < mp->goal_constraints.size(); ++i)
            if (mp->goal_constraints[i].joint_constraints.size() > 0)
            {
              std::map<std::string, double> vals;
              for (std::size_t j = 0; j < mp->goal_constraints[i].joint_constraints.size(); ++j)
                vals[mp->goal_constraints[i].joint_constraints[j].joint_name] =
                    mp->goal_constraints[i].joint_constraints[j].position;
              goal_state->setVariablePositions(vals);
              break;
            }
          planning_display_->setQueryGoalState(*goal_state);
        }
        else
          ROS_ERROR("Failed to load planning query '%s'. Has the message format changed since the query was saved?",
                    query_name.c_str());
      }
    }
  }
}

void MotionPlanningFrame::addObject(const collision_detection::WorldPtr& world, const std::string& id,
                                    const shapes::ShapeConstPtr& shape, const Eigen::Affine3d& pose)
{
  world->addToObject(id, shape, pose);

  planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateCollisionObjectsList, this));

  // Automatically select the inserted object so that its IM is displayed
  planning_display_->addMainLoopJob(
      boost::bind(&MotionPlanningFrame::setItemSelectionInList, this, id, true, ui_->collision_objects_list));

  planning_display_->queueRenderSceneGeometry();
}

void MotionPlanningFrame::createSceneInteractiveMarker()
{
  QList<QListWidgetItem*> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
    return;

  const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
  if (!ps)
    return;

  const collision_detection::CollisionWorld::ObjectConstPtr& obj =
      ps->getWorld()->getObject(sel[0]->text().toStdString());
  if (obj && obj->shapes_.size() == 1)
  {
    Eigen::Quaterniond eq(obj->shape_poses_[0].rotation());
    geometry_msgs::PoseStamped shape_pose;
    shape_pose.pose.position.x = obj->shape_poses_[0].translation()[0];
    shape_pose.pose.position.y = obj->shape_poses_[0].translation()[1];
    shape_pose.pose.position.z = obj->shape_poses_[0].translation()[2];
    shape_pose.pose.orientation.x = eq.x();
    shape_pose.pose.orientation.y = eq.y();
    shape_pose.pose.orientation.z = eq.z();
    shape_pose.pose.orientation.w = eq.w();

    // create an interactive marker for moving the shape in the world
    visualization_msgs::InteractiveMarker int_marker =
        robot_interaction::make6DOFMarker(std::string("marker_") + sel[0]->text().toStdString(), shape_pose, 1.0);
    int_marker.header.frame_id = planning_display_->getRobotModel()->getModelFrame();
    int_marker.description = sel[0]->text().toStdString();

    rviz::InteractiveMarker* imarker = new rviz::InteractiveMarker(planning_display_->getSceneNode(), context_);
    interactive_markers::autoComplete(int_marker);
    imarker->processMessage(int_marker);
    imarker->setShowAxes(false);
    scene_marker_.reset(imarker);

    // Connect signals
    connect(imarker, SIGNAL(userFeedback(visualization_msgs::InteractiveMarkerFeedback&)), this,
            SLOT(imProcessFeedback(visualization_msgs::InteractiveMarkerFeedback&)));
  }
  else
  {
    scene_marker_.reset();
  }
}

void MotionPlanningFrame::renameCollisionObject(QListWidgetItem* item)
{
  long unsigned int version = known_collision_objects_version_;
  if (item->text().isEmpty())
  {
    QMessageBox::warning(this, "Invalid object name", "Cannot set an empty object name.");
    if (version == known_collision_objects_version_)
      item->setText(QString::fromStdString(known_collision_objects_[item->type()].first));
    return;
  }

  std::string item_text = item->text().toStdString();
  bool already_exists = planning_display_->getPlanningSceneRO()->getWorld()->hasObject(item_text);
  if (!already_exists)
    already_exists = planning_display_->getPlanningSceneRO()->getCurrentState().hasAttachedBody(item_text);
  if (already_exists)
  {
    QMessageBox::warning(this, "Duplicate object name",
                         QString("The name '")
                             .append(item->text())
                             .append("' already exists. Not renaming object ")
                             .append((known_collision_objects_[item->type()].first.c_str())));
    if (version == known_collision_objects_version_)
      item->setText(QString::fromStdString(known_collision_objects_[item->type()].first));
    return;
  }

  if (item->checkState() == Qt::Unchecked)
  {
    planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
    collision_detection::CollisionWorld::ObjectConstPtr obj =
        ps->getWorld()->getObject(known_collision_objects_[item->type()].first);
    if (obj)
    {
      known_collision_objects_[item->type()].first = item_text;
      ps->getWorldNonConst()->removeObject(obj->id_);
      ps->getWorldNonConst()->addToObject(known_collision_objects_[item->type()].first, obj->shapes_,
                                          obj->shape_poses_);
      if (scene_marker_)
      {
        scene_marker_.reset();
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::createSceneInteractiveMarker, this));
      }
    }
  }
  else
  {
    // rename attached body
    planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
    robot_state::RobotState& cs = ps->getCurrentStateNonConst();
    const robot_state::AttachedBody* ab = cs.getAttachedBody(known_collision_objects_[item->type()].first);
    if (ab)
    {
      known_collision_objects_[item->type()].first = item_text;
      robot_state::AttachedBody* new_ab = new robot_state::AttachedBody(
          ab->getAttachedLink(), known_collision_objects_[item->type()].first, ab->getShapes(),
          ab->getFixedTransforms(), ab->getTouchLinks(), ab->getDetachPosture());
      cs.clearAttachedBody(ab->getName());
      cs.attachBody(new_ab);
    }
  }
}

void MotionPlanningFrame::attachDetachCollisionObject(QListWidgetItem* item)
{
  long unsigned int version = known_collision_objects_version_;
  bool checked = item->checkState() == Qt::Checked;
  std::pair<std::string, bool> data = known_collision_objects_[item->type()];
  moveit_msgs::AttachedCollisionObject aco;

  if (checked)  // we need to attach a known collision object
  {
    QStringList links;
    const std::vector<std::string>& links_std = planning_display_->getRobotModel()->getLinkModelNames();
    for (std::size_t i = 0; i < links_std.size(); ++i)
      links.append(QString::fromStdString(links_std[i]));
    bool ok = false;
    QString response =
        QInputDialog::getItem(this, tr("Select Link Name"), tr("Choose the link to attach to:"), links, 0, false, &ok);
    if (!ok)
    {
      if (version == known_collision_objects_version_)
        item->setCheckState(Qt::Unchecked);
      return;
    }
    aco.link_name = response.toStdString();
    aco.object.id = data.first;
    aco.object.operation = moveit_msgs::CollisionObject::ADD;
  }
  else  // we need to detach an attached object
  {
    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
    const robot_state::AttachedBody* attached_body = ps->getCurrentState().getAttachedBody(data.first);
    if (attached_body)
    {
      aco.link_name = attached_body->getAttachedLinkName();
      aco.object.id = attached_body->getName();
      aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
    }
  }
  {
    planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
    // we loop through the list in case updates were received since the start of the function
    for (std::size_t i = 0; i < known_collision_objects_.size(); ++i)
      if (known_collision_objects_[i].first == data.first)
      {
        known_collision_objects_[i].second = checked;
        break;
      }
    ps->processAttachedCollisionObjectMsg(aco);
  }

  selectedCollisionObjectChanged();
  planning_display_->queueRenderSceneGeometry();
}

void MotionPlanningFrame::populateCollisionObjectsList()
{
  ui_->collision_objects_list->setUpdatesEnabled(false);
  bool oldState = ui_->collision_objects_list->blockSignals(true);

  {
    QList<QListWidgetItem*> sel = ui_->collision_objects_list->selectedItems();
    std::set<std::string> to_select;
    for (int i = 0; i < sel.size(); ++i)
      to_select.insert(sel[i]->text().toStdString());
    ui_->collision_objects_list->clear();
    known_collision_objects_.clear();
    known_collision_objects_version_++;

    planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
    if (ps)
    {
      const std::vector<std::string>& collision_object_names = ps->getWorld()->getObjectIds();
      for (std::size_t i = 0; i < collision_object_names.size(); ++i)
      {
        if (collision_object_names[i] == planning_scene::PlanningScene::OCTOMAP_NS)
          continue;

        QListWidgetItem* item =
            new QListWidgetItem(QString::fromStdString(collision_object_names[i]), ui_->collision_objects_list, (int)i);
        item->setFlags(item->flags() | Qt::ItemIsEditable);
        item->setToolTip(item->text());
        item->setCheckState(Qt::Unchecked);
        if (to_select.find(collision_object_names[i]) != to_select.end())
          item->setSelected(true);
        ui_->collision_objects_list->addItem(item);
        known_collision_objects_.push_back(std::make_pair(collision_object_names[i], false));
      }

      const robot_state::RobotState& cs = ps->getCurrentState();
      std::vector<const robot_state::AttachedBody*> attached_bodies;
      cs.getAttachedBodies(attached_bodies);
      for (std::size_t i = 0; i < attached_bodies.size(); ++i)
      {
        QListWidgetItem* item =
            new QListWidgetItem(QString::fromStdString(attached_bodies[i]->getName()), ui_->collision_objects_list,
                                (int)(i + collision_object_names.size()));
        item->setFlags(item->flags() | Qt::ItemIsEditable);
        item->setToolTip(item->text());
        item->setCheckState(Qt::Checked);
        if (to_select.find(attached_bodies[i]->getName()) != to_select.end())
          item->setSelected(true);
        ui_->collision_objects_list->addItem(item);
        known_collision_objects_.push_back(std::make_pair(attached_bodies[i]->getName(), true));
      }
    }
  }

  ui_->collision_objects_list->blockSignals(oldState);
  ui_->collision_objects_list->setUpdatesEnabled(true);
  selectedCollisionObjectChanged();
}

void MotionPlanningFrame::exportAsTextButtonClicked()
{
  QString path =
      QFileDialog::getSaveFileName(this, tr("Export Scene Geometry"), tr(""), tr("Scene Geometry (*.scene)"));
  if (!path.isEmpty())
    planning_display_->addBackgroundJob(
        boost::bind(&MotionPlanningFrame::computeExportAsText, this, path.toStdString()), "export as text");
}

void MotionPlanningFrame::computeExportAsText(const std::string& path)
{
  planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
  if (ps)
  {
    std::string p = (path.length() < 7 || path.substr(path.length() - 6) != ".scene") ? path + ".scene" : path;
    std::ofstream fout(p.c_str());
    if (fout.good())
    {
      ps->saveGeometryToStream(fout);
      fout.close();
      ROS_INFO("Saved current scene geometry to '%s'", p.c_str());
    }
    else
      ROS_WARN("Unable to save current scene geometry to '%s'", p.c_str());
  }
}

void MotionPlanningFrame::computeImportFromText(const std::string& path)
{
  planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
  if (ps)
  {
    std::ifstream fin(path.c_str());
    if (ps->loadGeometryFromStream(fin))
    {
      ROS_INFO("Loaded scene geometry from '%s'", path.c_str());
      planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateCollisionObjectsList, this));
      planning_display_->queueRenderSceneGeometry();
    }
    else
    {
      QMessageBox::warning(nullptr, "Loading scene geometry", "Failed to load scene geometry.\n"
                                                              "See console output for more details.");
    }
  }
}

void MotionPlanningFrame::importFromTextButtonClicked()
{
  QString path =
      QFileDialog::getOpenFileName(this, tr("Import Scene Geometry"), tr(""), tr("Scene Geometry (*.scene)"));
  if (!path.isEmpty())
    planning_display_->addBackgroundJob(
        boost::bind(&MotionPlanningFrame::computeImportFromText, this, path.toStdString()), "import from text");
}
}
