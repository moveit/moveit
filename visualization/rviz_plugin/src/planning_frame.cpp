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

#include <moveit/rviz_plugin/planning_frame.h>
#include <moveit/rviz_plugin/planning_display.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include "ui_moveit_rviz_plugin_frame.h"
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematic_state/conversions.h>
#include <moveit/warehouse/planning_scene_storage.h>
#include <geometric_shapes/shape_operations.h>
#include <interactive_markers/tools.h>
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>

#include <moveit/robot_interaction/interactive_marker_helpers.h>

moveit_rviz_plugin::PlanningFrame::PlanningFrame(PlanningDisplay *pdisplay, rviz::DisplayContext *context, QWidget *parent) :
  QWidget(parent),
  planning_display_(pdisplay),
  context_(context),
  ui_(new Ui::MotionPlanningFrame()),
  scene_marker_(NULL)
{
  // set up the GUI
  ui_->setupUi(this);

  // connect bottons to actions; each action actually only registers the function pointer for the actual computation,
  // to keep the GUI more responsive
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
  connect( ui_->import_scene_button, SIGNAL( clicked() ), this, SLOT( importSceneButtonClicked() ));
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
  connect( ui_->collision_objects_list, SIGNAL( itemChanged( QListWidgetItem * ) ), this, SLOT( collisionObjectNameChanged( QListWidgetItem * ) ));
  connect( ui_->path_constraints_combo_box, SIGNAL( currentIndexChanged ( int ) ), this, SLOT( pathConstraintsIndexChanged( int ) ));
  connect( ui_->tabWidget, SIGNAL( currentChanged ( int ) ), this, SLOT( tabChanged( int ) ));

  ui_->tabWidget->setCurrentIndex(0); 
  planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  planning_scene_world_publisher_ = nh_.advertise<moveit_msgs::PlanningSceneWorld>("planning_scene_world", 1);
}

moveit_rviz_plugin::PlanningFrame::~PlanningFrame(void)
{
}

void moveit_rviz_plugin::PlanningFrame::changePlanningGroupHelper(void)
{ 
  if (!planning_display_->getPlanningSceneMonitor())
    return;
  const kinematic_model::KinematicModelConstPtr &kmodel = planning_display_->getPlanningSceneMonitor()->getKinematicModel();
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
        planning_display_->addMainLoopJob(boost::bind(&PlanningFrame::populatePlannersList, this, desc));
      planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::populateConstraintsList, this));
    }
  }
}

void moveit_rviz_plugin::PlanningFrame::changePlanningGroup(void)
{
  planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::changePlanningGroupHelper, this));
}

void moveit_rviz_plugin::PlanningFrame::publishSceneButtonClicked(void)
{
  if (planning_display_->getPlanningSceneMonitor())
  {
    moveit_msgs::PlanningScene msg;
    planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getPlanningSceneMsg(msg);
    planning_scene_publisher_.publish(msg);
  }
}

void moveit_rviz_plugin::PlanningFrame::sceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  if (update_type & planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY)
    planning_display_->addMainLoopJob(boost::bind(&PlanningFrame::populateCollisionObjectsList, this));
}

void moveit_rviz_plugin::PlanningFrame::populateCollisionObjectsList(void)
{
  ui_->collision_objects_list->setUpdatesEnabled(false);
  ui_->collision_objects_list->clear();
  collision_object_names_.clear();
  
  if (planning_display_->getPlanningSceneMonitor())
  {
    collision_detection::CollisionWorldPtr world = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getCollisionWorld();
    collision_object_names_ = world->getObjectIds();
    for (std::size_t i = 0 ; i < collision_object_names_.size() ; ++i)
    {
      QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(collision_object_names_[i]),
                                                  ui_->collision_objects_list, (int)i);
      item->setFlags(item->flags() | Qt::ItemIsEditable);
      ui_->collision_objects_list->addItem(item);
    }
  }
  ui_->collision_objects_list->setUpdatesEnabled(true);
}

/* Receives feedback from the interactive marker and updates the shape pose in the world accordingly */
void  moveit_rviz_plugin::PlanningFrame::imProcessFeedback(
    visualization_msgs::InteractiveMarkerFeedback &feedback )
{
  ui_->object_x->setValue(feedback.pose.position.x);
  ui_->object_y->setValue(feedback.pose.position.y);
  ui_->object_z->setValue(feedback.pose.position.z);
  tf::Quaternion q(feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w);

  double yaw, pitch, roll;
  tf::Matrix3x3(q).getEulerYPR(yaw,pitch,roll);
  ui_->object_rx->setValue(roll);
  ui_->object_ry->setValue(pitch);
  ui_->object_rz->setValue(yaw);
}

void moveit_rviz_plugin::PlanningFrame::createSceneInteractiveMarker() {
  QList<QListWidgetItem *> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
      return;
  if (scene_marker_==NULL)
  {
    // create an interactive marker for moving the shape in the world
    visualization_msgs::InteractiveMarker int_marker=robot_interaction::make6DOFMarker(std::string("marker_")+sel[0]->text().toStdString(), geometry_msgs::PoseStamped(), 1.0);
    int_marker.header.frame_id = context_->getFrameManager()->getFixedFrame();
    int_marker.description = sel[0]->text().toStdString();

    //Store the interactive marker in a map, access through the name on the shape
    rviz::InteractiveMarker* imarker=new rviz::InteractiveMarker(planning_display_->getSceneNode(), context_ );
    interactive_markers::autoComplete(int_marker);
    imarker->processMessage(int_marker);
    imarker->setShowAxes(false);
    scene_marker_=imarker;

    //Connect signals
    connect( imarker, SIGNAL( userFeedback(visualization_msgs::InteractiveMarkerFeedback &)), this, SLOT( imProcessFeedback(visualization_msgs::InteractiveMarkerFeedback &) ));
  }
}

void moveit_rviz_plugin::PlanningFrame::importSceneButtonClicked(void)
{
  std::string path = QFileDialog::getOpenFileName(this, "Import Scene").toStdString();
  std::string name;

  if (!path.empty() && planning_display_->getPlanningSceneMonitor())
  {
    path = "file://" + path;
    shapes::Mesh *mesh = shapes::createMeshFromResource(path);
    if (mesh)
    {
      std::size_t slash = path.find_last_of("/");
      name = path.substr(slash + 1);
      shapes::ShapeConstPtr shape(mesh);
      Eigen::Affine3d pose;
      pose.setIdentity();
      collision_detection::CollisionWorldPtr world = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getCollisionWorld();

      //If the object already exists, ask the user whether to overwrite or rename
      if (world->hasObject(name))
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
            world->removeObject(name);
            addObject(world, name, shape, pose);
            break;
          case QMessageBox::No:
          {
            // Don't overwrite was clicked. Ask for another name
            bool ok;
            QString text = QInputDialog::getText(this, tr("Choose a new name"),
                                                 tr("New object name:"), QLineEdit::Normal,
                                                 QString::fromStdString(name + "-" + boost::lexical_cast<std::string>(world->getObjectsCount())), &ok);
            if (ok)
            {
              if (!text.isEmpty())
              {
                name = text.toStdString();
                if (world->hasObject(name))
                  QMessageBox::warning(this, "Name already exists", QString("The name '").append(name.c_str()).
                                       append("' already exists. Not importing object."));
                else
                  addObject(world, name, shape, pose);
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
        addObject(world, name, shape, pose);
      }
    }
  }
}

void moveit_rviz_plugin::PlanningFrame::addObject(const collision_detection::CollisionWorldPtr &world, const std::string &id,
                                                  const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose)
{
  world->addToObject(id, shape, pose);
  populateCollisionObjectsList();
  planning_display_->queueRenderSceneGeometry();
}

void moveit_rviz_plugin::PlanningFrame::removeObjectButtonClicked(void)
{
  QList<QListWidgetItem *> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
    return;
  if (planning_display_->getPlanningSceneMonitor())
  {    
    collision_detection::CollisionWorldPtr world = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getCollisionWorld();
    for (int i = 0 ; i < sel.count() ; ++i)
      world->removeObject(sel[i]->text().toStdString());
    populateCollisionObjectsList();
    planning_display_->queueRenderSceneGeometry(); 
  }
}

void moveit_rviz_plugin::PlanningFrame::collisionObjectNameChanged(QListWidgetItem *item)
{
  if (item->type() < (int)collision_object_names_.size() && 
      collision_object_names_[item->type()] != item->text().toStdString() && 
      planning_display_->getPlanningSceneMonitor())
  {  
    collision_detection::CollisionWorldPtr world = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getCollisionWorld();
    collision_detection::CollisionWorld::ObjectConstPtr obj = world->getObject(collision_object_names_[item->type()]);
    if (obj)
    {
      collision_object_names_[item->type()] = item->text().toStdString();
      world->removeObject(obj->id_);
      world->addToObject(collision_object_names_[item->type()], obj->shapes_, obj->shape_poses_);
    }
  }
}

void moveit_rviz_plugin::PlanningFrame::selectedCollisionObjectChanged(void)
{
  QList<QListWidgetItem *> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
  {
    ui_->object_x->setValue(0.0);
    ui_->object_y->setValue(0.0);
    ui_->object_z->setValue(0.0);
    ui_->object_rx->setValue(0.0);
    ui_->object_ry->setValue(0.0);
    ui_->object_rz->setValue(0.0);
  }
  else
  {
    if (planning_display_->getPlanningSceneMonitor())
    {    
      collision_detection::CollisionWorldPtr world = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getCollisionWorld();
      collision_detection::CollisionWorld::ObjectConstPtr obj = world->getObject(sel[0]->text().toStdString());
      if (obj && obj->shapes_.size() == 1)
      {
        ui_->object_x->setValue(obj->shape_poses_[0].translation()[0]);
        ui_->object_y->setValue(obj->shape_poses_[0].translation()[1]);
        ui_->object_z->setValue(obj->shape_poses_[0].translation()[2]);
        Eigen::Vector3d xyz = obj->shape_poses_[0].rotation().eulerAngles(0, 1, 2);
        ui_->object_rx->setValue(xyz[0]);
        ui_->object_ry->setValue(xyz[1]);
        ui_->object_rz->setValue(xyz[2]);
      }

      if (!scene_marker_) createSceneInteractiveMarker();
      if (scene_marker_) {
        Eigen::Quaterniond eq(obj->shape_poses_[0].rotation());
        //Update the IM pose to match the current selected object
        scene_marker_->setPose(Ogre::Vector3(obj->shape_poses_[0].translation()[0], obj->shape_poses_[0].translation()[1], obj->shape_poses_[0].translation()[2]),
                               Ogre::Quaternion(eq.w(), eq.x(), eq.y(), eq.z()), "");
      }
    }
  }
}

void moveit_rviz_plugin::PlanningFrame::clearSceneButtonClicked(void)
{    
  if (planning_display_->getPlanningSceneMonitor())
  {
    planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getCollisionWorld()->clearObjects(); 
    populateCollisionObjectsList();
    planning_display_->queueRenderSceneGeometry(); 
  }
}

void moveit_rviz_plugin::PlanningFrame::objectPoseValueChanged(double value)
{
  QList<QListWidgetItem *> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
    return;
  if (planning_display_->getPlanningSceneMonitor())
  {
    collision_detection::CollisionWorldPtr world = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getCollisionWorld();
    collision_detection::CollisionWorld::ObjectConstPtr obj = world->getObject(sel[0]->text().toStdString());
    if (obj && obj->shapes_.size() == 1)
    {
      Eigen::Affine3d p;
      p.translation()[0] = ui_->object_x->value();
      p.translation()[1] = ui_->object_y->value();
      p.translation()[2] = ui_->object_z->value();

      p = Eigen::Translation3d(p.translation()) *
          Eigen::AngleAxisd(ui_->object_rz->value(), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(ui_->object_ry->value(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(ui_->object_rx->value(), Eigen::Vector3d::UnitX());

      world->moveShapeInObject(obj->id_, obj->shapes_[0], p);  
      planning_display_->queueRenderSceneGeometry(); 

      //Update the interactive marker pose to match the manually introduced one
      if (scene_marker_) {
        Eigen::Quaterniond eq(p.rotation());
        scene_marker_->setPose(Ogre::Vector3(ui_->object_x->value(), ui_->object_y->value(), ui_->object_z->value()),
                               Ogre::Quaternion(eq.w(), eq.x(), eq.y(), eq.z()), "");
      }
    }
  }
}

void moveit_rviz_plugin::PlanningFrame::sceneScaleStartChange(void)
{
  QList<QListWidgetItem *> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
    return;
  if (planning_display_->getPlanningSceneMonitor())
  {
    collision_detection::CollisionWorldPtr world = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getCollisionWorld();
    scaled_object_ = world->getObject(sel[0]->text().toStdString());
  }
}

void moveit_rviz_plugin::PlanningFrame::sceneScaleEndChange(void)
{
  scaled_object_.reset();
  ui_->scene_scale->setSliderPosition(100);
}

void moveit_rviz_plugin::PlanningFrame::sceneScaleChanged(int value)
{
  if (scaled_object_ && planning_display_->getPlanningSceneMonitor())
  {
    collision_detection::CollisionWorldPtr world = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getCollisionWorld();
    if (world->hasObject(scaled_object_->id_))
    {
      world->removeObject(scaled_object_->id_);
      for (std::size_t i = 0 ; i < scaled_object_->shapes_.size() ; ++i)
      {
        shapes::Shape *s = scaled_object_->shapes_[i]->clone();
        s->scale((double)value / 100.0);
        world->addToObject(scaled_object_->id_, shapes::ShapeConstPtr(s), scaled_object_->shape_poses_[i]);
      }
      planning_display_->queueRenderSceneGeometry(); 
    }
    else
      scaled_object_.reset();
  }
}

void moveit_rviz_plugin::PlanningFrame::populateConstraintsList(void)
{
  if (move_group_)
  {
    // add some artificial wait time (but in the background) for the constraints DB to connect
    double dt = (ros::WallTime::now() - move_group_construction_time_).toSec();
    if (dt < 0.2)
      ros::WallDuration(0.1).sleep();
    planning_display_->addMainLoopJob(boost::bind(&PlanningFrame::populateConstraintsList, this, move_group_->getKnownConstraints()));
  }
}

void moveit_rviz_plugin::PlanningFrame::populateConstraintsList(const std::vector<std::string> &constr)
{
  ui_->path_constraints_combo_box->clear();     
  ui_->path_constraints_combo_box->addItem("None");
  for (std::size_t i = 0 ; i < constr.size() ; ++i)
    ui_->path_constraints_combo_box->addItem(QString::fromStdString(constr[i]));
}

void moveit_rviz_plugin::PlanningFrame::populatePlannersList(const moveit_msgs::PlannerInterfaceDescription &desc)
{ 
  std::string group = planning_display_->getCurrentPlanningGroup();
  ui_->planning_algorithm_combo_box->clear();  

  // set the label for the planning library
  ui_->library_label->setText(QString::fromStdString(desc.name));
  ui_->library_label->setStyleSheet("QLabel { color : green; font: bold }");
  
  bool found_group = false;
  // the name of a planner is either "GROUP[planner_id]" or "planner_id"
  if (!group.empty())
    for (std::size_t i = 0 ; i < desc.planner_ids.size() ; ++i)
      if (desc.planner_ids[i] == group)
        found_group = true;
      else
        if (desc.planner_ids[i].substr(0, group.length()) == group)
        {
          std::string id = desc.planner_ids[i].substr(group.length());
          if (id.size() > 2)
          {
            id.resize(id.length() - 1);
            ui_->planning_algorithm_combo_box->addItem(QString::fromStdString(id.substr(1)));
          }
        }
  if (ui_->planning_algorithm_combo_box->count() == 0 && !found_group)
    for (std::size_t i = 0 ; i < desc.planner_ids.size() ; ++i)
      ui_->planning_algorithm_combo_box->addItem(QString::fromStdString(desc.planner_ids[i]));  
  ui_->planning_algorithm_combo_box->insertItem(0, "<unspecified>");
  ui_->planning_algorithm_combo_box->setCurrentIndex(0);
}

void moveit_rviz_plugin::PlanningFrame::enable(void)
{
  ui_->planning_algorithm_combo_box->clear();  
  ui_->library_label->setText("NO PLANNING LIBRARY LOADED");
  ui_->library_label->setStyleSheet("QLabel { color : red; font: bold }");
  
  changePlanningGroup();
  
  // activate the frame
  show();
}

void moveit_rviz_plugin::PlanningFrame::disable(void)
{
  move_group_.reset();
  hide();
}

void moveit_rviz_plugin::PlanningFrame::allowLookingToggled(bool checked)
{
  if (move_group_)
    move_group_->allowLooking(checked);
}

void moveit_rviz_plugin::PlanningFrame::allowReplanningToggled(bool checked)
{
  if (move_group_)
    move_group_->allowReplanning(checked);
}

void moveit_rviz_plugin::PlanningFrame::pathConstraintsIndexChanged(int index)
{
  if (move_group_)
  {
    if (index > 0)
      move_group_->setPathConstraints(ui_->path_constraints_combo_box->itemText(index).toStdString());
    else
      move_group_->clearPathConstraints();
  }
}

void moveit_rviz_plugin::PlanningFrame::tabChanged(int index)
{
  if(scene_marker_!=NULL) {
    delete scene_marker_;
    scene_marker_=NULL;
  }
}

void moveit_rviz_plugin::PlanningFrame::planningAlgorithmIndexChanged(int index)
{
  if (move_group_)
  {
    if (index > 0)
      move_group_->setPlannerId(ui_->planning_algorithm_combo_box->itemText(index).toStdString());
    else
      move_group_->setPlannerId("");
  }
}

void moveit_rviz_plugin::PlanningFrame::constructPlanningRequest(moveit_msgs::MotionPlanRequest &mreq)
{
  mreq.group_name = planning_display_->getCurrentPlanningGroup();
  mreq.num_planning_attempts = 1;
  mreq.allowed_planning_time = ros::Duration(ui_->planning_time->value());
  kinematic_state::kinematicStateToRobotState(*planning_display_->getQueryStartState(), mreq.start_state);
  mreq.workspace_parameters.min_corner.x = ui_->wcenter_x->value() - ui_->wsize_x->value() / 2.0;
  mreq.workspace_parameters.min_corner.y = ui_->wcenter_y->value() - ui_->wsize_y->value() / 2.0;
  mreq.workspace_parameters.min_corner.z = ui_->wcenter_z->value() - ui_->wsize_z->value() / 2.0;
  mreq.workspace_parameters.max_corner.x = ui_->wcenter_x->value() + ui_->wsize_x->value() / 2.0;
  mreq.workspace_parameters.max_corner.y = ui_->wcenter_y->value() + ui_->wsize_y->value() / 2.0;
  mreq.workspace_parameters.max_corner.z = ui_->wcenter_z->value() + ui_->wsize_z->value() / 2.0;
  const kinematic_state::JointStateGroup *jsg = planning_display_->getQueryGoalState()->getJointStateGroup(mreq.group_name);
  if (jsg)
  {
    mreq.goal_constraints.resize(1);
    mreq.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(jsg);
  }
}

void moveit_rviz_plugin::PlanningFrame::planButtonClicked(void)
{
  planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::computePlanButtonClicked, this));
}

void moveit_rviz_plugin::PlanningFrame::executeButtonClicked(void)
{
  ui_->execute_button->setEnabled(false);
  planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::computeExecuteButtonClicked, this));
}

void moveit_rviz_plugin::PlanningFrame::planAndExecuteButtonClicked(void)
{
  ui_->plan_and_execute_button->setEnabled(false);
  ui_->execute_button->setEnabled(false);
  planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::computePlanAndExecuteButtonClicked, this));
}

void moveit_rviz_plugin::PlanningFrame::randomStatesButtonClicked(void)
{
  planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::computeRandomStatesButtonClicked, this));
}

void moveit_rviz_plugin::PlanningFrame::setStartToCurrentButtonClicked(void)
{ 
  planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::computeSetStartToCurrentButtonClicked, this));
}

void moveit_rviz_plugin::PlanningFrame::setGoalToCurrentButtonClicked(void)
{
  planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::computeSetGoalToCurrentButtonClicked, this));
}

void moveit_rviz_plugin::PlanningFrame::databaseConnectButtonClicked(void)
{   
  planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::computeDatabaseConnectButtonClicked, this));
}

void moveit_rviz_plugin::PlanningFrame::saveSceneButtonClicked(void)
{ 
  if (planning_scene_storage_)
  {
    const std::string &name = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getName();
    if (name.empty() || planning_scene_storage_->hasPlanningScene(name))
    {
      boost::scoped_ptr<QMessageBox> q;
      if (name.empty())
        q.reset(new QMessageBox(QMessageBox::Question, "Change Planning Scene Name",
                                QString("The name for the planning scene should not be empty. Would you like to rename the planning scene?'"),
                                QMessageBox::Cancel,
                                this));
      else
        q.reset(new QMessageBox(QMessageBox::Question, "Confirm Planning Scene Overwrite",
                                QString("A planning scene named '")
                                .append( name.c_str() )
                                .append( "' already exists. Do you wish to overwrite that scene?"),
                                QMessageBox::Yes | QMessageBox::No,
                                this));
      boost::scoped_ptr<QPushButton> rename(q->addButton("&Rename", QMessageBox::AcceptRole));
      if (q->exec() != QMessageBox::Yes)
      {
        if (q->clickedButton() == rename.get())
        {
          bool ok = false;
          QString new_name = QInputDialog::getText(this, "Rename Planning Scene", "New name for the planning scene:", QLineEdit::Normal, QString::fromStdString(name), &ok);
          if (ok)
          {
            planning_display_->getPlanningSceneMonitor()->getPlanningScene()->setName(new_name.toStdString());
            planning_display_->subProp("Planning Scene")->subProp("Scene Name")->setValue(new_name);
            saveSceneButtonClicked();
          }
          return;
        }
        return;
      }
    }
    
    planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::computeSaveSceneButtonClicked, this));
  }
}

void moveit_rviz_plugin::PlanningFrame::loadSceneButtonClicked(void)
{   
  planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::computeLoadSceneButtonClicked, this));
}

void moveit_rviz_plugin::PlanningFrame::loadQueryButtonClicked(void)
{   
  planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::computeLoadQueryButtonClicked, this));
}

void moveit_rviz_plugin::PlanningFrame::saveQueryButtonClicked(void)
{   
  planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::computeSaveQueryButtonClicked, this));
}

void moveit_rviz_plugin::PlanningFrame::deleteSceneButtonClicked(void)
{   
  planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::computeDeleteSceneButtonClicked, this));
}

void moveit_rviz_plugin::PlanningFrame::deleteQueryButtonClicked(void)
{   
  planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::computeDeleteQueryButtonClicked, this));
}

void moveit_rviz_plugin::PlanningFrame::checkPlanningSceneTreeEnabledButtons(void)
{
  QList<QTreeWidgetItem *> sel = ui_->planning_scene_tree->selectedItems();
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
    
    QTreeWidgetItem *s = sel.front();
    
    // if the item is a PlanningScene
    if (s->type() == 1)
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

void moveit_rviz_plugin::PlanningFrame::planningSceneItemClicked(void)
{
  checkPlanningSceneTreeEnabledButtons();
}

void moveit_rviz_plugin::PlanningFrame::configureForPlanning(void)
{
  move_group_->setStartState(*planning_display_->getQueryStartState());
  move_group_->setJointValueTarget(*planning_display_->getQueryGoalState());
  move_group_->setPlanningTime(ui_->planning_time->value());
  move_group_->setWorkspace(ui_->wcenter_x->value() - ui_->wsize_x->value() / 2.0,
                            ui_->wcenter_y->value() - ui_->wsize_y->value() / 2.0,
                            ui_->wcenter_z->value() - ui_->wsize_z->value() / 2.0,
                            ui_->wcenter_x->value() + ui_->wsize_x->value() / 2.0,
                            ui_->wcenter_y->value() + ui_->wsize_y->value() / 2.0,
                            ui_->wcenter_z->value() + ui_->wsize_z->value() / 2.0);
}

void moveit_rviz_plugin::PlanningFrame::updateSceneMarkers(float wall_dt, float ros_dt)
{
  if (scene_marker_!=NULL)
    scene_marker_->update(wall_dt);
}

void moveit_rviz_plugin::PlanningFrame::computePlanButtonClicked(void)
{
  if (!move_group_)
    return;
  configureForPlanning();
  current_plan_.reset(new move_group_interface::MoveGroup::Plan());
  if (move_group_->plan(*current_plan_))
    ui_->execute_button->setEnabled(true);
  else
    current_plan_.reset();
}

void moveit_rviz_plugin::PlanningFrame::computeExecuteButtonClicked(void)
{
  if (move_group_ && current_plan_)
    move_group_->execute(*current_plan_);
}

void moveit_rviz_plugin::PlanningFrame::computePlanAndExecuteButtonClicked(void)
{    
  if (!move_group_)
    return;
  configureForPlanning();
  move_group_->move();
  ui_->plan_and_execute_button->setEnabled(true);
}

void moveit_rviz_plugin::PlanningFrame::computeSetStartToCurrentButtonClicked(void)
{  
  if (!move_group_)
    return;
  kinematic_state::KinematicStatePtr s = move_group_->getCurrentState();
  if (s)
    planning_display_->setQueryStartState(s);
}

void moveit_rviz_plugin::PlanningFrame::computeSetGoalToCurrentButtonClicked(void)
{ 
  if (!move_group_)
    return;
  kinematic_state::KinematicStatePtr s = move_group_->getCurrentState();
  if (s)
    planning_display_->setQueryGoalState(s);
}

void moveit_rviz_plugin::PlanningFrame::computeRandomStatesButtonClicked(void)
{
  std::string group_name = planning_display_->getCurrentPlanningGroup();

  if (planning_display_->getQueryStartState())
  {
    kinematic_state::KinematicStatePtr start(new kinematic_state::KinematicState(*planning_display_->getQueryStartState()));
    kinematic_state::JointStateGroup *jsg = start->getJointStateGroup(group_name);
    if (jsg)
      jsg->setToRandomValues();
    planning_display_->setQueryStartState(start);
  }

  if (planning_display_->getQueryGoalState())
  {
    kinematic_state::KinematicStatePtr goal(new kinematic_state::KinematicState(*planning_display_->getQueryGoalState()));
    kinematic_state::JointStateGroup *jsg = goal->getJointStateGroup(group_name);
    if (jsg)
      jsg->setToRandomValues();
    planning_display_->setQueryGoalState(goal);
  }
}

void moveit_rviz_plugin::PlanningFrame::populatePlanningSceneTreeView(void)
{  
  boost::shared_ptr<moveit_warehouse::PlanningSceneStorage> planning_scene_storage = planning_scene_storage_;
  if (!planning_scene_storage)
    return;
  
  ui_->planning_scene_tree->setUpdatesEnabled(false);

  // remember which items were expanded
  std::set<std::string> expanded;
  for (int i = 0 ; i < ui_->planning_scene_tree->topLevelItemCount() ; ++i)
  {
    QTreeWidgetItem *it = ui_->planning_scene_tree->topLevelItem(i);
    if (it->isExpanded())
      expanded.insert(it->text(0).toStdString());
  }
  
  ui_->planning_scene_tree->clear();
  std::vector<std::string> names;
  planning_scene_storage->getPlanningSceneNames(names);
  
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    std::vector<moveit_warehouse::MotionPlanRequestWithMetadata> planning_queries;
    std::vector<std::string> query_names;
    planning_scene_storage->getPlanningQueries(planning_queries, query_names, names[i]);
    QTreeWidgetItem *item = new QTreeWidgetItem(ui_->planning_scene_tree, QStringList(QString::fromStdString(names[i])), 1);
    for (std::size_t j = 0 ; j < query_names.size() ; ++j)
      item->addChild(new QTreeWidgetItem(item, QStringList(QString::fromStdString(query_names[j])), 2));
    ui_->planning_scene_tree->insertTopLevelItem(ui_->planning_scene_tree->topLevelItemCount(), item);
    if (expanded.find(names[i]) != expanded.end())
      ui_->planning_scene_tree->expandItem(item);
  }
  ui_->planning_scene_tree->sortItems(0, Qt::AscendingOrder);
  ui_->planning_scene_tree->setUpdatesEnabled(true); 
  checkPlanningSceneTreeEnabledButtons();
}

void moveit_rviz_plugin::PlanningFrame::computeDatabaseConnectButtonClickedHelper(int mode)
{  
  if (mode == 1)
  {
    ui_->planning_scene_tree->setUpdatesEnabled(false); 
    ui_->planning_scene_tree->clear();
    ui_->planning_scene_tree->setUpdatesEnabled(true); 
    
    ui_->database_connect_button->setUpdatesEnabled(false); 
    ui_->database_connect_button->setText(QString::fromStdString("Connect")); 
    ui_->database_connect_button->setStyleSheet("QPushButton { color : green }");
    ui_->database_connect_button->setUpdatesEnabled(true); 
    
    ui_->load_scene_button->setEnabled(false);
    ui_->load_query_button->setEnabled(false);
    ui_->save_query_button->setEnabled(false);
    ui_->save_scene_button->setEnabled(false);
    ui_->delete_query_button->setEnabled(false);
    ui_->delete_scene_button->setEnabled(false);
  }
  else  
  if (mode == 2)
  {   
    ui_->database_connect_button->setUpdatesEnabled(false); 
    ui_->database_connect_button->setText(QString::fromStdString("Connecting ..."));
    ui_->database_connect_button->setUpdatesEnabled(true); 
  }
  else  
  if (mode == 3)
  {   
    ui_->database_connect_button->setUpdatesEnabled(false); 
    ui_->database_connect_button->setText(QString::fromStdString("Connect"));   
    ui_->database_connect_button->setStyleSheet("QPushButton { color : green }");
    ui_->database_connect_button->setUpdatesEnabled(true); 
  }
  else
  if (mode == 4)
  {
    ui_->database_connect_button->setUpdatesEnabled(false); 
    ui_->database_connect_button->setText(QString::fromStdString("Disconnect"));
    ui_->database_connect_button->setStyleSheet("QPushButton { color : red }");
    ui_->database_connect_button->setUpdatesEnabled(true); 
    ui_->save_scene_button->setEnabled(true);
    populatePlanningSceneTreeView();
  }
}

void moveit_rviz_plugin::PlanningFrame::computeDatabaseConnectButtonClicked(void)
{
  if (planning_scene_storage_)
  {
    planning_scene_storage_.reset();
    planning_display_->addMainLoopJob(boost::bind(&PlanningFrame::computeDatabaseConnectButtonClickedHelper, this, 1));
  }
  else
  {      
    planning_display_->addMainLoopJob(boost::bind(&PlanningFrame::computeDatabaseConnectButtonClickedHelper, this, 2));
    try
    {
      planning_scene_storage_.reset(new moveit_warehouse::PlanningSceneStorage(ui_->database_host->text().toStdString(),
                                                                               ui_->database_port->value(), 5.0));
    }
    catch(std::runtime_error &ex)
    { 
      planning_display_->addMainLoopJob(boost::bind(&PlanningFrame::computeDatabaseConnectButtonClickedHelper, this, 3));
      ROS_ERROR("%s", ex.what());  
      return;
    }     
    planning_display_->addMainLoopJob(boost::bind(&PlanningFrame::computeDatabaseConnectButtonClickedHelper, this, 4));
  }
}

void moveit_rviz_plugin::PlanningFrame::computeSaveSceneButtonClicked(void)
{
  if (planning_scene_storage_)
  {
    moveit_msgs::PlanningScene msg;
    planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getPlanningSceneMsg(msg);
    planning_scene_storage_->removePlanningScene(msg.name);
    planning_scene_storage_->addPlanningScene(msg);
    planning_display_->addMainLoopJob(boost::bind(&PlanningFrame::populatePlanningSceneTreeView, this));
  }
}

void moveit_rviz_plugin::PlanningFrame::computeLoadSceneButtonClicked(void)
{
  if (planning_scene_storage_)
  { 
    QList<QTreeWidgetItem *> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem *s = sel.front();
      if (s->type() == 1)
      {
        std::string scene = s->text(0).toStdString();
        ROS_DEBUG("Attempting to load scene '%s'", scene.c_str());
        moveit_warehouse::PlanningSceneWithMetadata scene_m;
        if (planning_scene_storage_->getPlanningScene(scene_m, scene))
        {
          ROS_DEBUG("Loaded scene '%s'", scene.c_str());
          if (planning_display_->getPlanningSceneMonitor())
          {
            if (scene_m->robot_model_name != planning_display_->getPlanningSceneMonitor()->getKinematicModel()->getName())
            {
              ROS_INFO("Scene '%s' was saved for robot '%s' but we are using robot '%s'. Using scene geometry only",
                       scene.c_str(), scene_m->robot_model_name.c_str(),
                       planning_display_->getPlanningSceneMonitor()->getKinematicModel()->getName().c_str());
              planning_scene_world_publisher_.publish(scene_m->world);
            }
            else
              planning_scene_publisher_.publish(static_cast<const moveit_msgs::PlanningScene&>(*scene_m));
          }
          else
            planning_scene_publisher_.publish(static_cast<const moveit_msgs::PlanningScene&>(*scene_m));
        }
        else
          ROS_WARN("Failed to load scene '%s'. Has the message format changed since the scene was saved?", scene.c_str());
      }
    }
  }
}

void moveit_rviz_plugin::PlanningFrame::computeLoadQueryButtonClicked(void)
{
  if (planning_scene_storage_)
  { 
    QList<QTreeWidgetItem *> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem *s = sel.front();
      if (s->type() == 2)
      {
        std::string scene = s->parent()->text(0).toStdString();
        std::string query_name = s->text(0).toStdString();
        moveit_warehouse::MotionPlanRequestWithMetadata mp;
        if (planning_scene_storage_->getPlanningQuery(mp, scene, query_name))
        {
          kinematic_state::KinematicStatePtr start_state(new kinematic_state::KinematicState(*planning_display_->getQueryStartState()));
          kinematic_state::robotStateToKinematicState(*planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getTransforms(),
                                                      mp->start_state, *start_state);
          planning_display_->setQueryStartState(start_state);
          
          kinematic_state::KinematicStatePtr goal_state(new kinematic_state::KinematicState(*planning_display_->getQueryGoalState()));
          for (std::size_t i = 0 ; i < mp->goal_constraints.size() ; ++i)
            if (mp->goal_constraints[i].joint_constraints.size() > 0)
            {
              std::map<std::string, double> vals;
              for (std::size_t j = 0 ; j < mp->goal_constraints[i].joint_constraints.size() ; ++j)
                vals[mp->goal_constraints[i].joint_constraints[j].joint_name] = mp->goal_constraints[i].joint_constraints[j].position;
              goal_state->setStateValues(vals);
              break;
            }
          planning_display_->setQueryGoalState(goal_state);
        }
      }
    }
  }
}

void moveit_rviz_plugin::PlanningFrame::computeSaveQueryButtonClicked(void)
{
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem *> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem *s = sel.front();
      moveit_msgs::MotionPlanRequest mreq;
      constructPlanningRequest(mreq);

      // if we have selected a PlanningScene, add the query as a new one, under that planning scene
      if (s->type() == 1)
      {
        std::string scene = s->text(0).toStdString();
        planning_scene_storage_->addPlanningRequest(mreq, scene);
      }
      else
      {
        // if we selected a query name, then we overwrite that query
        std::string scene = s->parent()->text(0).toStdString();
        std::string query_name = s->text(0).toStdString();
        planning_scene_storage_->addPlanningRequest(mreq, scene, query_name);
      }
      planning_display_->addMainLoopJob(boost::bind(&PlanningFrame::populatePlanningSceneTreeView, this));
    }
  }
}

void moveit_rviz_plugin::PlanningFrame::computeDeleteSceneButtonClicked(void)
{
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem *> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem *s = sel.front();
      if (s->type() == 1)
      {
        std::string scene = s->text(0).toStdString();
        planning_scene_storage_->removePlanningScene(scene);
      }
      else
      {
        // if we selected a query name, then we overwrite that query
        std::string scene = s->parent()->text(0).toStdString();
        planning_scene_storage_->removePlanningScene(scene);  
      }
      planning_display_->addMainLoopJob(boost::bind(&PlanningFrame::populatePlanningSceneTreeView, this));
    }
  }
}

void moveit_rviz_plugin::PlanningFrame::computeDeleteQueryButtonClickedHelper(QTreeWidgetItem *s)
{     
  ui_->planning_scene_tree->setUpdatesEnabled(false);
  s->parent()->removeChild(s);  
  ui_->planning_scene_tree->setUpdatesEnabled(true);
}

void moveit_rviz_plugin::PlanningFrame::computeDeleteQueryButtonClicked(void)
{
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem *> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem *s = sel.front();
      if (s->type() == 2)
      {
        std::string scene = s->parent()->text(0).toStdString();
        std::string query_name = s->text(0).toStdString();
        planning_scene_storage_->removePlanningQuery(scene, query_name); 
        planning_display_->addMainLoopJob(boost::bind(&PlanningFrame::computeDeleteQueryButtonClickedHelper, this, s));
      }
    }
  }
}
