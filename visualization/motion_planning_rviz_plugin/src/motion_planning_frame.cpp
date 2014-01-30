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
  first_time_(true)
{
  // set up the GUI
  ui_->setupUi(this);

  // connect bottons to actions; each action usually registers the function pointer for the actual computation,
  // to keep the GUI more responsive (using the background job processing)
  connect( ui_->plan_button, SIGNAL( clicked() ), this, SLOT( planButtonClicked() ));
  connect( ui_->execute_button, SIGNAL( clicked() ), this, SLOT( executeButtonClicked() ));
  connect( ui_->plan_and_execute_button, SIGNAL( clicked() ), this, SLOT( planAndExecuteButtonClicked() ));
  connect( ui_->use_start_state_button, SIGNAL( clicked() ), this, SLOT( useStartStateButtonClicked() ));
  connect( ui_->use_goal_state_button, SIGNAL( clicked() ), this, SLOT( useGoalStateButtonClicked() ));
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
  connect( ui_->load_state_button, SIGNAL( clicked() ), this, SLOT( loadStateButtonClicked() ));
  connect( ui_->save_start_state_button, SIGNAL( clicked() ), this, SLOT( saveStartStateButtonClicked() ));
  connect( ui_->save_goal_state_button, SIGNAL( clicked() ), this, SLOT( saveGoalStateButtonClicked() ));
  connect( ui_->set_as_start_state_button, SIGNAL( clicked() ), this, SLOT( setAsStartStateButtonClicked() ));
  connect( ui_->set_as_goal_state_button, SIGNAL( clicked() ), this, SLOT( setAsGoalStateButtonClicked() ));
  connect( ui_->remove_state_button, SIGNAL( clicked() ), this, SLOT( removeStateButtonClicked() ));
  connect( ui_->clear_states_button, SIGNAL( clicked() ), this, SLOT( clearStatesButtonClicked() ));
  connect( ui_->approximate_ik, SIGNAL( stateChanged(int) ), this, SLOT( approximateIKChanged(int) ));


  connect( ui_->detect_objects_button, SIGNAL( clicked() ), this, SLOT( detectObjectsButtonClicked() ));
  connect( ui_->pick_button, SIGNAL( clicked() ), this, SLOT( pickObjectButtonClicked() ));
  connect( ui_->place_button, SIGNAL( clicked() ), this, SLOT( placeObjectButtonClicked() ));
  connect( ui_->detected_objects_list, SIGNAL( itemSelectionChanged() ), this, SLOT( selectedDetectedObjectChanged() ));
  connect( ui_->detected_objects_list, SIGNAL( itemChanged( QListWidgetItem * ) ), this, SLOT( detectedObjectChanged( QListWidgetItem * ) ));
  connect( ui_->support_surfaces_list, SIGNAL( itemSelectionChanged() ), this, SLOT( selectedSupportSurfaceChanged() ));

  connect( ui_->tabWidget, SIGNAL( currentChanged ( int ) ), this, SLOT( tabChanged( int ) ));

  QShortcut *copy_object_shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_C), ui_->collision_objects_list);
  connect(copy_object_shortcut, SIGNAL( activated() ), this, SLOT( copySelectedCollisionObject() ) );

  ui_->reset_db_button->hide();
  ui_->background_job_progress->hide();
  ui_->background_job_progress->setMaximum(0);

  ui_->tabWidget->setCurrentIndex(0);

  known_collision_objects_version_ = 0;

  planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  planning_scene_world_publisher_ = nh_.advertise<moveit_msgs::PlanningSceneWorld>("planning_scene_world", 1);
 
  //  object_recognition_trigger_publisher_ = nh_.advertise<std_msgs::Bool>("recognize_objects_switch", 1);
  object_recognition_client_.reset(new actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction>(OBJECT_RECOGNITION_ACTION, false));
  object_recognition_subscriber_ = nh_.subscribe("recognized_object_array", 1, &MotionPlanningFrame::listenDetectedObjects, this);

  if(object_recognition_client_)
  {
    try
    {
      waitForAction(object_recognition_client_, nh_, ros::Duration(3.0), OBJECT_RECOGNITION_ACTION); 
    }
    catch(std::runtime_error &ex)
    {
      //      ROS_ERROR("Object recognition action: %s", ex.what());      
      object_recognition_client_.reset();      
    }          
  }  
  try  
  {
    planning_scene_interface_.reset(new moveit::planning_interface::PlanningSceneInterface());
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  try
  {
    const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
    if(ps)
    {
      semantic_world_.reset(new moveit::semantic_world::SemanticWorld(ps));
    }
    else
      semantic_world_.reset();    
    if(semantic_world_)
    {
      semantic_world_->addTableCallback(boost::bind(&MotionPlanningFrame::updateTables, this));    
    }  
  } 
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  ros::NodeHandle nh;
  plan_subscriber_ = nh.subscribe("/rviz/moveit/plan", 1, &MotionPlanningFrame::remotePlanCallback, this);
  execute_subscriber_ = nh.subscribe("/rviz/moveit/execute", 1, &MotionPlanningFrame::remoteExecuteCallback, this);
  update_start_state_subscriber_ = nh.subscribe("/rviz/moveit/update_start_state",1,
                                                &MotionPlanningFrame::remoteUpdateStartStateCallback, this);
  update_goal_state_subscriber_ = nh.subscribe("/rviz/moveit/update_goal_state",1,
                                               &MotionPlanningFrame::remoteUpdateGoalStateCallback, this);
}

MotionPlanningFrame::~MotionPlanningFrame()
{
}

void MotionPlanningFrame::approximateIKChanged(int state)
{
  planning_display_->useApproximateIK(state == Qt::Checked);
}

void MotionPlanningFrame::setItemSelectionInList(const std::string &item_name, bool selection, QListWidget *list)
{
  QList<QListWidgetItem*> found_items = list->findItems(QString(item_name.c_str()), Qt::MatchExactly);
  for (std::size_t i = 0 ; i < found_items.size(); ++i)
    found_items[i]->setSelected(selection);
}

void MotionPlanningFrame::fillStateSelectionOptions()
{
  ui_->start_state_selection->clear();
  ui_->goal_state_selection->clear();

  if (!planning_display_->getPlanningSceneMonitor())
    return;

  const robot_model::RobotModelConstPtr &kmodel = planning_display_->getRobotModel();
  std::string group = planning_display_->getCurrentPlanningGroup();
  if (group.empty())
    return;
  const robot_model::JointModelGroup *jmg = kmodel->getJointModelGroup(group);
  if (jmg)
  {
    ui_->start_state_selection->addItem(QString("<random valid>"));
    ui_->start_state_selection->addItem(QString("<random>"));
    ui_->start_state_selection->addItem(QString("<current>"));
    ui_->start_state_selection->addItem(QString("<same as goal>"));

    ui_->goal_state_selection->addItem(QString("<random valid>"));
    ui_->goal_state_selection->addItem(QString("<random>"));
    ui_->goal_state_selection->addItem(QString("<current>"));
    ui_->goal_state_selection->addItem(QString("<same as start>"));

    const std::vector<std::string> &known_states = jmg->getDefaultStateNames();
    if (!known_states.empty())
    {
      ui_->start_state_selection->insertSeparator(ui_->start_state_selection->count());
      ui_->goal_state_selection->insertSeparator(ui_->goal_state_selection->count());
      for (std::size_t i = 0 ; i < known_states.size() ; ++i)
      {
        ui_->start_state_selection->addItem(QString::fromStdString(known_states[i]));
        ui_->goal_state_selection->addItem(QString::fromStdString(known_states[i]));
      }
    }
    ui_->start_state_selection->setCurrentIndex(2); // default to 'current'
    ui_->goal_state_selection->setCurrentIndex(0); // default to 'random valid'
  }
}

void MotionPlanningFrame::changePlanningGroupHelper()
{
  if (!planning_display_->getPlanningSceneMonitor())
    return;

  planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::fillStateSelectionOptions, this));
  planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateConstraintsList, this, std::vector<std::string>()));

  const robot_model::RobotModelConstPtr &kmodel = planning_display_->getRobotModel();
  std::string group = planning_display_->getCurrentPlanningGroup();

  if (!group.empty() && kmodel)
  {
    if (move_group_ && move_group_->getName() == group)
      return;
    ROS_INFO("Constructing new MoveGroup connection for group '%s'", group.c_str());
    moveit::planning_interface::MoveGroup::Options opt(group);
    opt.robot_model_ = kmodel;
    opt.robot_description_.clear();
    try
    {
      move_group_.reset(new moveit::planning_interface::MoveGroup(opt, context_->getFrameManager()->getTFClientPtr(), ros::Duration(30, 0)));
      if (planning_scene_storage_)
        move_group_->setConstraintsDatabase(ui_->database_host->text().toStdString(), ui_->database_port->value());
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
      planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::populateConstraintsList, this), "populateConstraintsList");

      if (first_time_)
      {
        first_time_ = false;
        const planning_scene_monitor::LockedPlanningSceneRO &ps = planning_display_->getPlanningSceneRO();
        if (ps)
        {
          planning_display_->setQueryStartState(ps->getCurrentState());
          planning_display_->setQueryGoalState(ps->getCurrentState());
        }
      }
    }
  }
}

void MotionPlanningFrame::changePlanningGroup()
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::changePlanningGroupHelper, this), "Frame::changePlanningGroup");
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
      if (planning_display_->getPlanningSceneRO()->getWorld()->hasObject(name))
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
                ps->getWorldNonConst()->removeObject(name);
                addObject(ps->getWorldNonConst(), name, shape, pose);
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
                                                                        (planning_display_->getPlanningSceneRO()->getWorld()->size())), &ok);
            if (ok)
            {
              if (!text.isEmpty())
              {
                name = text.toStdString();
                planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
                if (ps)
                {
                  if (ps->getWorld()->hasObject(name))
                    QMessageBox::warning(this, "Name already exists", QString("The name '").append(name.c_str()).
                                         append("' already exists. Not importing object."));
                  else
                    addObject(ps->getWorldNonConst(), name, shape, pose);
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
          addObject(ps->getWorldNonConst(), name, shape, pose);
      }
    }
    else
    {
      QMessageBox::warning(this, QString("Import error"), QString("Unable to import scene"));
      return;
    }
  }
}

void MotionPlanningFrame::enable()
{
  ui_->planning_algorithm_combo_box->clear();
  ui_->library_label->setText("NO PLANNING LIBRARY LOADED");
  ui_->library_label->setStyleSheet("QLabel { color : red; font: bold }");
  ui_->object_status->setText("");

  // activate the frame
  show();
}

void MotionPlanningFrame::disable()
{
  move_group_.reset();
  hide();
}

void MotionPlanningFrame::tabChanged(int index)
{
  if (scene_marker_ && ui_->tabWidget->tabText(index) != "Scene Objects")
    scene_marker_.reset();
  else
    if (ui_->tabWidget->tabText(index) == "Scene Objects")
      selectedCollisionObjectChanged();
}

void MotionPlanningFrame::updateSceneMarkers(float wall_dt, float ros_dt)
{
  if (scene_marker_)
    scene_marker_->update(wall_dt);
}

} // namespace
