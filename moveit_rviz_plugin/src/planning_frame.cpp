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

#include "moveit_rviz_plugin/planning_frame.h"
#include "moveit_rviz_plugin/planning_display.h"
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include "ui_moveit_rviz_plugin_frame.h"
#include <kinematic_constraints/utils.h>
#include <planning_models/conversions.h>
#include <geometric_shapes/shape_operations.h>
#include <QFileDialog>

moveit_rviz_plugin::PlanningFrame::PlanningFrame(PlanningDisplay *pdisplay, rviz::DisplayContext *context, QWidget *parent) :
  QWidget(parent),
  planning_display_(pdisplay),
  context_(context),
  ui_(new Ui::MotionPlanningFrame())
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
  connect( ui_->object_x, SIGNAL( valueChanged(double) ), this, SLOT( objectXValueChanged(double) ));
  connect( ui_->object_y, SIGNAL( valueChanged(double) ), this, SLOT( objectYValueChanged(double) ));
  connect( ui_->object_z, SIGNAL( valueChanged(double) ), this, SLOT( objectZValueChanged(double) ));
  connect( ui_->object_rx, SIGNAL( valueChanged(double) ), this, SLOT( objectRXValueChanged(double) ));
  connect( ui_->object_ry, SIGNAL( valueChanged(double) ), this, SLOT( objectRYValueChanged(double) ));
  connect( ui_->object_rz, SIGNAL( valueChanged(double) ), this, SLOT( objectRZValueChanged(double) ));
  connect( ui_->publish_current_scene_button, SIGNAL( clicked() ), this, SLOT( publishSceneButtonClicked() ));
  connect( ui_->collision_objects_list, SIGNAL( itemSelectionChanged() ), this, SLOT( selectedCollisionObjectChanged() ));
  connect( ui_->collision_objects_list, SIGNAL( itemChanged( QListWidgetItem * ) ), this, SLOT( collisionObjectNameChanged( QListWidgetItem * ) ));
  
  ui_->tabWidget->setCurrentIndex(0);
}

moveit_rviz_plugin::PlanningFrame::~PlanningFrame(void)
{
}

void moveit_rviz_plugin::PlanningFrame::changePlanningGroupHelper(void)
{ 
  if (!planning_display_->getPlanningSceneMonitor())
    return;
  const planning_models::KinematicModelConstPtr &kmodel = planning_display_->getPlanningSceneMonitor()->getKinematicModel();
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
    std::string topic = planning_display_->subProp("Planning Scene")->subProp("Planning Scene Topic")->getValue().toString().toStdString();
    if (!topic.empty())
    {
      planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>(topic, 1);
      moveit_msgs::PlanningScene msg;
      planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getPlanningSceneMsg(msg);
      planning_scene_publisher_.publish(msg);
    }
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

void moveit_rviz_plugin::PlanningFrame::importSceneButtonClicked(void)
{ 
  std::string path = QFileDialog::getOpenFileName(this, "Import Scene").toStdString();
  if (!path.empty() && planning_display_->getPlanningSceneMonitor())
  {
    path = "file://" + path;
    shapes::Mesh *mesh = shapes::createMeshFromResource(path);
    if (mesh)
    {
      std::size_t slash = path.find_last_of("/");
      std::string name = path.substr(slash + 1);
      shapes::ShapeConstPtr shape(mesh);
      Eigen::Affine3d pose;
      pose.setIdentity();
      collision_detection::CollisionWorldPtr world = planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getCollisionWorld();
      world->removeObject(name);
      world->addToObject(name, shape, pose);
      populateCollisionObjectsList();
      planning_display_->queueRenderSceneGeometry();
    }
  }
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

void moveit_rviz_plugin::PlanningFrame::objectXValueChanged(double value)
{
  objectPoseValueChanged(0, value);
}

void moveit_rviz_plugin::PlanningFrame::objectYValueChanged(double value)
{
  objectPoseValueChanged(1, value);
}

void moveit_rviz_plugin::PlanningFrame::objectZValueChanged(double value)
{
  objectPoseValueChanged(2, value);
}

void moveit_rviz_plugin::PlanningFrame::objectRXValueChanged(double value)
{
  objectPoseValueChanged(3, value);
}

void moveit_rviz_plugin::PlanningFrame::objectRYValueChanged(double value)
{
  objectPoseValueChanged(4, value);
}

void moveit_rviz_plugin::PlanningFrame::objectRZValueChanged(double value)
{
  objectPoseValueChanged(5, value);
}

void moveit_rviz_plugin::PlanningFrame::objectPoseValueChanged(int index, double value)
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
      Eigen::Affine3d p = obj->shape_poses_[0];
      if (index < 3)
        p.translation()[index] = value;
      else
      {
        Eigen::Vector3d xyz = obj->shape_poses_[0].rotation().eulerAngles(0, 1, 2);
        xyz[index - 3] = value;
        p = Eigen::Translation3d(p.translation()) * 
          Eigen::AngleAxisd(xyz[0], Eigen::Vector3d::UnitX()) * 
          Eigen::AngleAxisd(xyz[1], Eigen::Vector3d::UnitY()) * 
          Eigen::AngleAxisd(xyz[2], Eigen::Vector3d::UnitZ());
      }
      world->moveShapeInObject(obj->id_, obj->shapes_[0], p);  
      planning_display_->queueRenderSceneGeometry(); 
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

void moveit_rviz_plugin::PlanningFrame::populatePlannersList(const moveit_msgs::PlannerInterfaceDescription &desc)
{ 
  std::string group = planning_display_->getCurrentPlanningGroup();

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

void moveit_rviz_plugin::PlanningFrame::planningAlgorithmIndexChanged(int index)
{
  if (move_group_ && index >= 0)
    move_group_->setPlannerId(ui_->planning_algorithm_combo_box->itemText(index).toStdString());
}

void moveit_rviz_plugin::PlanningFrame::constructPlanningRequest(moveit_msgs::MotionPlanRequest &mreq)
{
  mreq.group_name = planning_display_->getCurrentPlanningGroup();
  mreq.num_planning_attempts = 1;
  mreq.allowed_planning_time = ros::Duration(5.0);
  planning_models::kinematicStateToRobotState(*planning_display_->getQueryStartState(), mreq.start_state);
  
  const planning_models::KinematicState::JointStateGroup *jsg = planning_display_->getQueryGoalState()->getJointStateGroup(mreq.group_name);
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
  planning_display_->addBackgroundJob(boost::bind(&PlanningFrame::computeSaveSceneButtonClicked, this));
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

void moveit_rviz_plugin::PlanningFrame::computePlanButtonClicked(void)
{
  if (!move_group_)
    return;
  
  move_group_->setStartState(*planning_display_->getQueryStartState());
  move_group_->setJointValueTarget(*planning_display_->getQueryGoalState());
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
  move_group_->setStartState(*planning_display_->getQueryStartState());
  move_group_->setJointValueTarget(*planning_display_->getQueryGoalState());
  move_group_->move();
  ui_->plan_and_execute_button->setEnabled(true);
}

void moveit_rviz_plugin::PlanningFrame::computeSetStartToCurrentButtonClicked(void)
{  
  if (!move_group_)
    return;
  planning_models::KinematicStatePtr s = move_group_->getCurrentState();
  if (s)
    planning_display_->setQueryStartState(s);
}

void moveit_rviz_plugin::PlanningFrame::computeSetGoalToCurrentButtonClicked(void)
{ 
  if (!move_group_)
    return;
  planning_models::KinematicStatePtr s = move_group_->getCurrentState();
  if (s)
    planning_display_->setQueryGoalState(s);
}

void moveit_rviz_plugin::PlanningFrame::computeRandomStatesButtonClicked(void)
{
  std::string group_name = planning_display_->getCurrentPlanningGroup();
  
  planning_models::KinematicStatePtr start(new planning_models::KinematicState(*planning_display_->getQueryStartState()));
  
  planning_models::KinematicState::JointStateGroup *jsg = start->getJointStateGroup(group_name);
  if (jsg)
    jsg->setToRandomValues();
  
  planning_models::KinematicStatePtr goal(new planning_models::KinematicState(*planning_display_->getQueryGoalState()));
  jsg = goal->getJointStateGroup(group_name);
  if (jsg)
    jsg->setToRandomValues();
  
  planning_display_->setQueryStartState(start);
  planning_display_->setQueryGoalState(goal);
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
        moveit_warehouse::PlanningSceneWithMetadata scene_m;
        if (planning_scene_storage_->getPlanningScene(scene_m, scene))
        {
          std::string topic = planning_display_->subProp("Planning Scene")->subProp("Planning Scene Topic")->getValue().toString().toStdString();
          if (!topic.empty())
          {
            planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>(topic, 1);
            planning_scene_publisher_.publish(static_cast<const moveit_msgs::PlanningScene&>(*scene_m));
          }
        }
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
          planning_models::KinematicStatePtr start_state(new planning_models::KinematicState(*planning_display_->getQueryStartState()));
          planning_models::robotStateToKinematicState(*planning_display_->getPlanningSceneMonitor()->getPlanningScene()->getTransforms(),
                                                      mp->start_state, *start_state);
          planning_display_->setQueryStartState(start_state);
          
          planning_models::KinematicStatePtr goal_state(new planning_models::KinematicState(*planning_display_->getQueryGoalState()));
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
