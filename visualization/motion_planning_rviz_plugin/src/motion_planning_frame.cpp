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

/* Author: Ioan Sucan */

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

#include <geometric_shapes/shape_operations.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>

#include <QMessageBox>
#include <QInputDialog>
#include <QShortcut>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{

MotionPlanningFrame::MotionPlanningFrame(MotionPlanningDisplay *pdisplay, rviz::DisplayContext *context, QWidget *parent) :
  QWidget(parent),
  planning_display_(pdisplay),
  context_(context),
  ui_(new Ui::MotionPlanningUI()),
  goal_pose_dragging_(false)
{
  // set up the GUI
  ui_->setupUi(this);
  
  // connect bottons to actions; each action usually registers the function pointer for the actual computation,
  // to keep the GUI more responsive (using the background job processing)
  connect( ui_->plan_button, SIGNAL( clicked() ), this, SLOT( planButtonClicked() ));
  connect( ui_->execute_button, SIGNAL( clicked() ), this, SLOT( executeButtonClicked() ));
  connect( ui_->plan_and_execute_button, SIGNAL( clicked() ), this, SLOT( planAndExecuteButtonClicked() ));
  connect( ui_->set_random_states_button, SIGNAL( clicked() ), this, SLOT( randomStatesButtonClicked() ));
  connect( ui_->set_start_to_current_button, SIGNAL( clicked() ), this, SLOT( setStartToCurrentButtonClicked() ));
  connect( ui_->set_goal_to_current_button, SIGNAL( clicked() ), this, SLOT( setGoalToCurrentButtonClicked() ));
  connect( ui_->database_connect_button, SIGNAL( clicked() ), this, SLOT( databaseConnectButtonClicked() ));
  connect( ui_->save_scene_button, SIGNAL( clicked() ), this, SLOT( saveSceneButtonClicked() ));
  connect( ui_->save_query_button, SIGNAL( clicked() ), this, SLOT( saveQueryButtonClicked() ));
  connect( ui_->delete_scene_button, SIGNAL( clicked() ), this, SLOT( deleteSceneButtonClicked() ));
  connect( ui_->delete_query_button, SIGNAL( clicked() ), this, SLOT( deleteQueryButtonClicked() ));
  connect( ui_->planning_scene_tree, SIGNAL( itemSelectionChanged() ), this, SLOT( planningSceneItemClicked() ));
  connect( ui_->load_scene_button, SIGNAL( clicked() ), this, SLOT( loadSceneButtonClicked() ));
  connect( ui_->load_query_button, SIGNAL( clicked() ), this, SLOT( loadQueryButtonClicked() ));
  connect( ui_->allow_looking, SIGNAL( toggled(bool) ), this, SLOT( allowLookingToggled(bool) ));
  connect( ui_->allow_replanning, SIGNAL( toggled(bool) ), this, SLOT( allowReplanningToggled(bool) ));
  connect( ui_->planning_algorithm_combo_box, SIGNAL( currentIndexChanged ( int ) ), this, SLOT( planningAlgorithmIndexChanged( int ) ));
  connect( ui_->import_file_button, SIGNAL( clicked() ), this, SLOT( importFileButtonClicked() ));
  connect( ui_->import_url_button, SIGNAL( clicked() ), this, SLOT( importUrlButtonClicked() ));
  connect( ui_->clear_scene_button, SIGNAL( clicked() ), this, SLOT( clearSceneButtonClicked() ));
  connect( ui_->scene_scale, SIGNAL( valueChanged(int) ), this, SLOT( sceneScaleChanged(int) ));
  connect( ui_->scene_scale, SIGNAL( sliderPressed() ), this, SLOT( sceneScaleStartChange() ));
  connect( ui_->scene_scale, SIGNAL( sliderReleased() ), this, SLOT( sceneScaleEndChange() ));
  connect( ui_->remove_object_button, SIGNAL( clicked() ), this, SLOT( removeObjectButtonClicked() ));
  connect( ui_->object_x, SIGNAL( valueChanged(double) ), this, SLOT( objectPoseValueChanged(double) ));
  connect( ui_->object_y, SIGNAL( valueChanged(double) ), this, SLOT( objectPoseValueChanged(double) ));
  connect( ui_->object_z, SIGNAL( valueChanged(double) ), this, SLOT( objectPoseValueChanged(double) ));
  connect( ui_->object_rx, SIGNAL( valueChanged(double) ), this, SLOT( objectPoseValueChanged(double) ));
  connect( ui_->object_ry, SIGNAL( valueChanged(double) ), this, SLOT( objectPoseValueChanged(double) ));
  connect( ui_->object_rz, SIGNAL( valueChanged(double) ), this, SLOT( objectPoseValueChanged(double) ));
  connect( ui_->publish_current_scene_button, SIGNAL( clicked() ), this, SLOT( publishSceneButtonClicked() ));
  connect( ui_->collision_objects_list, SIGNAL( itemSelectionChanged() ), this, SLOT( selectedCollisionObjectChanged() ));
  connect( ui_->collision_objects_list, SIGNAL( itemChanged( QListWidgetItem * ) ), this, SLOT( collisionObjectChanged( QListWidgetItem * ) ));
  connect( ui_->path_constraints_combo_box, SIGNAL( currentIndexChanged ( int ) ), this, SLOT( pathConstraintsIndexChanged( int ) ));
  connect( ui_->planning_scene_tree, SIGNAL( itemChanged( QTreeWidgetItem *, int ) ), this, SLOT( warehouseItemNameChanged( QTreeWidgetItem *, int ) ));
  connect( ui_->reset_db_button, SIGNAL( clicked() ), this, SLOT( resetDbButtonClicked() ));
  connect( ui_->export_scene_text_button, SIGNAL( clicked() ), this, SLOT( exportAsTextButtonClicked() ));
  connect( ui_->import_scene_text_button, SIGNAL( clicked() ), this, SLOT( importFromTextButtonClicked() ));

  connect( ui_->tabWidget, SIGNAL( currentChanged ( int ) ), this, SLOT( tabChanged( int ) ));

  QShortcut *copy_object_shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_C), ui_->collision_objects_list);
  connect(copy_object_shortcut, SIGNAL( activated() ), this, SLOT( copySelectedCollisionObject() ) );

  //Goal poses
  connect( ui_->goal_poses_add_button, SIGNAL( clicked() ), this, SLOT( createGoalPoseButtonClicked() ));
  connect( ui_->goal_poses_remove_button, SIGNAL( clicked() ), this, SLOT( deleteGoalsOnDBButtonClicked() ));
  connect( ui_->load_poses_filter_text, SIGNAL( returnPressed() ), this, SLOT( loadGoalsFromDBButtonClicked() ));
  connect( ui_->goal_poses_open_button, SIGNAL( clicked() ), this, SLOT( loadGoalsFromDBButtonClicked() ));
  connect( ui_->goal_poses_save_button, SIGNAL( clicked() ), this, SLOT( saveGoalsOnDBButtonClicked() ));
  connect( ui_->goal_switch_visibility_button, SIGNAL( clicked() ), this, SLOT( switchGoalVisibilityButtonClicked() ));
  connect( ui_->goal_poses_list, SIGNAL( itemSelectionChanged() ), this, SLOT( goalPoseSelectionChanged() ));
  connect( ui_->goal_poses_list, SIGNAL( itemDoubleClicked(QListWidgetItem *) ), this, SLOT( goalPoseDoubleClicked(QListWidgetItem *) ));
  connect( ui_->show_x_checkbox, SIGNAL( stateChanged( int ) ), this, SLOT( visibleAxisChanged( int ) ));
  connect( ui_->show_y_checkbox, SIGNAL( stateChanged( int ) ), this, SLOT( visibleAxisChanged( int ) ));
  connect( ui_->show_z_checkbox, SIGNAL( stateChanged( int ) ), this, SLOT( visibleAxisChanged( int ) ));
  connect( ui_->check_goal_collisions_button, SIGNAL( clicked( ) ), this, SLOT( checkGoalsInCollision( ) ));
  connect( ui_->check_goal_reachability_button, SIGNAL( clicked( ) ), this, SLOT( checkGoalsReachable( ) ));

  QShortcut *copy_goals_shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_C), ui_->goal_poses_list);
  connect(copy_goals_shortcut, SIGNAL( activated() ), this, SLOT( copySelectedGoalPoses() ) );

  //Goal poses icons
  ui_->goal_poses_open_button->setIcon(QIcon::fromTheme("document-open", QApplication::style()->standardIcon(QStyle::SP_DirOpenIcon)));
  ui_->goal_poses_add_button->setIcon(QIcon::fromTheme("list-add", QApplication::style()->standardIcon(QStyle::SP_FileDialogNewFolder)));
  ui_->goal_poses_remove_button->setIcon(QIcon::fromTheme("list-remove", QApplication::style()->standardIcon(QStyle::SP_DialogDiscardButton)));
  ui_->goal_poses_save_button->setIcon(QIcon::fromTheme("document-save", QApplication::style()->standardIcon(QStyle::SP_DriveFDIcon)));
  ui_->goal_switch_visibility_button->setIcon(QApplication::style()->standardIcon(QStyle::SP_DialogDiscardButton));

  //Start states
  connect( ui_->start_states_add_button, SIGNAL( clicked() ), this, SLOT( saveStartStateButtonClicked() ));
  connect( ui_->start_states_remove_button, SIGNAL( clicked() ), this, SLOT( deleteStatesOnDBButtonClicked() ));
  connect( ui_->load_states_filter_text, SIGNAL( returnPressed() ), this, SLOT( loadStatesFromDBButtonClicked() ));
  connect( ui_->start_states_open_button, SIGNAL( clicked() ), this, SLOT( loadStatesFromDBButtonClicked() ));
  connect( ui_->start_states_save_button, SIGNAL( clicked() ), this, SLOT( saveStatesOnDBButtonClicked() ));
  connect( ui_->start_states_list, SIGNAL( itemDoubleClicked(QListWidgetItem*) ), this, SLOT( startStateItemDoubleClicked(QListWidgetItem*) ));

  //Start states icons
  ui_->start_states_open_button->setIcon(QIcon::fromTheme("document-open", QApplication::style()->standardIcon(QStyle::SP_DirOpenIcon)));
  ui_->start_states_add_button->setIcon(QIcon::fromTheme("list-add", QApplication::style()->standardIcon(QStyle::SP_FileDialogNewFolder)));
  ui_->start_states_remove_button->setIcon(QIcon::fromTheme("list-remove", QApplication::style()->standardIcon(QStyle::SP_DialogDiscardButton)));
  ui_->start_states_save_button->setIcon(QIcon::fromTheme("document-save", QApplication::style()->standardIcon(QStyle::SP_DriveFDIcon)));

  ui_->reset_db_button->hide();
  ui_->background_job_progress->hide(); 
  ui_->background_job_progress->setMaximum(0);

  ui_->tabWidget->setCurrentIndex(0);
  
  known_collision_objects_version_ = 0;
  
  planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  planning_scene_world_publisher_ = nh_.advertise<moveit_msgs::PlanningSceneWorld>("planning_scene_world", 1);
}

MotionPlanningFrame::~MotionPlanningFrame(void)
{
}

void MotionPlanningFrame::selectItemJob(QListWidgetItem *item, bool flag)
{
  item->setSelected(flag);
}

void MotionPlanningFrame::setItemSelectionInList(const std::string &item_name, bool selection, QListWidget *list) 
{
  QList<QListWidgetItem*> found_items = list->findItems(QString(item_name.c_str()), Qt::MatchExactly);
  for (std::size_t i = 0 ; i < found_items.size(); ++i)
    found_items[i]->setSelected(selection);
}

void MotionPlanningFrame::changePlanningGroupHelper(void)
{ 
  if (!planning_display_->getPlanningSceneMonitor())
    return;
  
  const kinematic_model::KinematicModelConstPtr &kmodel = planning_display_->getKinematicModel();
  std::string group = planning_display_->getCurrentPlanningGroup(); 

  if (!group.empty() && kmodel)
  {
    if (move_group_ && move_group_->getName() == group)
      return;
    move_group_interface::MoveGroup::Options opt(group);
    opt.kinematic_model_ = kmodel;
    opt.robot_description_.clear();
    try
    {
      move_group_.reset(new move_group_interface::MoveGroup(opt, context_->getFrameManager()->getTFClientPtr(), ros::Duration(5, 0)));
      move_group_construction_time_ = ros::WallTime::now();
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    if (move_group_)
    {
      move_group_->allowLooking(ui_->allow_looking->isChecked());
      move_group_->allowReplanning(ui_->allow_replanning->isChecked());
      moveit_msgs::PlannerInterfaceDescription desc;
      if (move_group_->getInterfaceDescription(desc))
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populatePlannersList, this, desc));
      planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::populateConstraintsList, this));
    }
  } 
}

void MotionPlanningFrame::changePlanningGroup(void)
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::changePlanningGroupHelper, this));
}

void MotionPlanningFrame::sceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  if (update_type & planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY)
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateCollisionObjectsList, this));
}

void MotionPlanningFrame::importResource(const std::string &path)
{
  if (planning_display_->getPlanningSceneMonitor())
  {
    shapes::Mesh *mesh = shapes::createMeshFromResource(path);
    if (mesh)
    {
      std::size_t slash = path.find_last_of("/\\");
      std::string name = path.substr(slash + 1);
      shapes::ShapeConstPtr shape(mesh);
      Eigen::Affine3d pose;
      pose.setIdentity();
      
      if (planning_display_->getPlanningSceneRO()->getCurrentState().hasAttachedBody(name))
      {
        QMessageBox::warning(this, QString("Duplicate names"),
                             QString("An attached object named '").append(name.c_str()).append("' already exists. Please rename the attached object before importing."));
        return;
      }
      
      //If the object already exists, ask the user whether to overwrite or rename
      if (planning_display_->getPlanningSceneRO()->getCollisionWorld()->hasObject(name))
      {
        QMessageBox msgBox;
        msgBox.setText("There exists another object with the same name.");
        msgBox.setInformativeText("Would you like to overwrite it?");
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::No);
        int ret = msgBox.exec();
        
        switch (ret)
        {
          case QMessageBox::Yes:
            // Overwrite was clicked
            {
              planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
              if (ps)
              {
                ps->getCollisionWorld()->removeObject(name);
                addObject(ps->getCollisionWorld(), name, shape, pose);
              }
            }
            break;
          case QMessageBox::No:
          {
            // Don't overwrite was clicked. Ask for another name
            bool ok = false;
            QString text = QInputDialog::getText(this, tr("Choose a new name"),
                                                 tr("Import the new object under the name:"), QLineEdit::Normal,
                                                 QString::fromStdString(name + "-" + boost::lexical_cast<std::string>
                                                                        (planning_display_->getPlanningSceneRO()->getCollisionWorld()->getObjectsCount())), &ok);
            if (ok)
            {
              if (!text.isEmpty())
              {
                name = text.toStdString();
                planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
                if (ps)
                {
                  if (ps->getCollisionWorld()->hasObject(name))
                    QMessageBox::warning(this, "Name already exists", QString("The name '").append(name.c_str()).
                                         append("' already exists. Not importing object."));
                  else
                    addObject(ps->getCollisionWorld(), name, shape, pose);
                }
              }
              else
                QMessageBox::warning(this, "Object not imported", "Cannot use an empty name for an imported object");
            }
            break;
          }
          default:
            //Pressed cancel, do nothing
            break;
        }
      }
      else
      {
        planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
        if (ps)
          addObject(ps->getCollisionWorld(), name, shape, pose);  
      }
    }
    else
    {
      QMessageBox::warning(this, QString("Import error"), QString("Unable to import scene"));
      return;
    }
  }
}

void MotionPlanningFrame::enable(void)
{
  ui_->planning_algorithm_combo_box->clear();  
  ui_->library_label->setText("NO PLANNING LIBRARY LOADED");
  ui_->library_label->setStyleSheet("QLabel { color : red; font: bold }");
  ui_->object_status->setText("");

  changePlanningGroup();

  for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end() ; ++it)
    if (it->second.isVisible())
      it->second.imarker->setShowAxes(false);

  // activate the frame
  show();
}

void MotionPlanningFrame::disable(void)
{
  move_group_.reset();
  hide();
}

void MotionPlanningFrame::tabChanged(int index)
{
  if (scene_marker_ && index != 2)
    scene_marker_.reset();
  else
    if (index == 2)
      selectedCollisionObjectChanged();

  //Create goals when entering the goals tab. Destroy them when exiting that tab
  if (index != 4)
  {
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it !=  goal_poses_.end(); it++)
      it->second.hide();
  }
  else
    if (index == 4)
      for (unsigned int i = 0; i < ui_->goal_poses_list->count() ; ++i)
      {
        QListWidgetItem *item = ui_->goal_poses_list->item(i);
        item->setBackground(QBrush(Qt::NoBrush));
        goal_poses_[item->text().toStdString()].show(planning_display_, context_);
      }
}

void MotionPlanningFrame::updateSceneMarkers(float wall_dt, float ros_dt)
{
  if (scene_marker_)
    scene_marker_->update(wall_dt);
}

void MotionPlanningFrame::updateGoalPoseMarkers(float wall_dt, float ros_dt)
{
  if (goal_pose_dragging_)
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end() ; ++it)
      if (it->second.isVisible())
        it->second.imarker->update(wall_dt);
}

} // namespace
