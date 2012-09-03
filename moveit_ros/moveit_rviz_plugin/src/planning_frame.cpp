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
#include "ui_moveit_rviz_plugin_frame.h"
#include <kinematic_constraints/utils.h>
#include <planning_models/conversions.h>

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

  ui_->tabWidget->setCurrentIndex(0);
  
  // spin a thread that will process user events
  run_processing_thread_ = true;
  processing_thread_.reset(new boost::thread(boost::bind(&PlanningFrame::processingThread, this)));
}

moveit_rviz_plugin::PlanningFrame::~PlanningFrame(void)
{
  run_processing_thread_ = false; 
  new_action_condition_.notify_all();
  processing_thread_->join();
}

void moveit_rviz_plugin::PlanningFrame::enable(const planning_scene_monitor::PlanningSceneMonitorPtr &planning_scene_monitor)
{  
  if (planning_scene_monitor)
  {
    plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor));
    plan_execution_->getPlanningPipeline().displayComputedMotionPlans(false);
    plan_execution_->getPlanningPipeline().checkSolutionPaths(true); 
    plan_execution_->displayCostSources(true);

    const planning_interface::PlannerPtr &planner_interface = plan_execution_->getPlanningPipeline().getPlannerInterface();
    if (planner_interface)
    {
      ui_->library_label->setText(QString::fromStdString(planner_interface->getDescription()));
      std::vector<std::string> algs;
      planner_interface->getPlanningAlgorithms(algs);
      ui_->planning_algorithm_combo_box->clear();
      for (std::size_t i = 0 ; i < algs.size() ; ++i)
      {
        ui_->planning_algorithm_combo_box->addItem(QString::fromStdString(algs[i]));
      }
    }
  }
  show();
}

void moveit_rviz_plugin::PlanningFrame::disable(void)
{
  plan_execution_.reset();
  hide();
}

void moveit_rviz_plugin::PlanningFrame::processingThread(void)
{
  boost::unique_lock<boost::mutex> ulock(action_lock_);

  while (run_processing_thread_)
  {
    while (actions_.empty() && run_processing_thread_)
      new_action_condition_.wait(ulock);
    
    while (!actions_.empty())
    {
      boost::function<void(void)> fn = actions_.front();
      actions_.pop_front();
      
      // make sure we are unlocked while we process the event
      action_lock_.unlock();
      try
      {
        fn();
      }
      catch(...)
      {
        ROS_ERROR("Exception caught while processing event");
      }
      action_lock_.lock();
    }
  }
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
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.push_back(boost::bind(&PlanningFrame::computePlanButtonClicked, this));
  new_action_condition_.notify_all();
}

void moveit_rviz_plugin::PlanningFrame::executeButtonClicked(void)
{
  ui_->execute_button->setEnabled(false);
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.push_back(boost::bind(&PlanningFrame::computeExecuteButtonClicked, this));
  new_action_condition_.notify_all();
}

void moveit_rviz_plugin::PlanningFrame::planAndExecuteButtonClicked(void)
{
  ui_->plan_and_execute_button->setEnabled(false);
  ui_->execute_button->setEnabled(false);
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.push_back(boost::bind(&PlanningFrame::computePlanAndExecuteButtonClicked, this));
  new_action_condition_.notify_all();
}

void moveit_rviz_plugin::PlanningFrame::randomStatesButtonClicked(void)
{
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.push_back(boost::bind(&PlanningFrame::computeRandomStatesButtonClicked, this));
  new_action_condition_.notify_all();
}

void moveit_rviz_plugin::PlanningFrame::setStartToCurrentButtonClicked(void)
{
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.push_back(boost::bind(&PlanningFrame::computeSetStartToCurrentButtonClicked, this));
  new_action_condition_.notify_all();
}

void moveit_rviz_plugin::PlanningFrame::setGoalToCurrentButtonClicked(void)
{
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.push_back(boost::bind(&PlanningFrame::computeSetGoalToCurrentButtonClicked, this));
  new_action_condition_.notify_all();
}

void moveit_rviz_plugin::PlanningFrame::databaseConnectButtonClicked(void)
{   
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.push_back(boost::bind(&PlanningFrame::computeDatabaseConnectButtonClicked, this));
  new_action_condition_.notify_all();
}

void moveit_rviz_plugin::PlanningFrame::saveSceneButtonClicked(void)
{   
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.push_back(boost::bind(&PlanningFrame::computeSaveSceneButtonClicked, this));
  new_action_condition_.notify_all();
}

void moveit_rviz_plugin::PlanningFrame::loadSceneButtonClicked(void)
{   
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.push_back(boost::bind(&PlanningFrame::computeLoadSceneButtonClicked, this));
  new_action_condition_.notify_all();
}

void moveit_rviz_plugin::PlanningFrame::loadQueryButtonClicked(void)
{   
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.push_back(boost::bind(&PlanningFrame::computeLoadQueryButtonClicked, this));
  new_action_condition_.notify_all();
}

void moveit_rviz_plugin::PlanningFrame::saveQueryButtonClicked(void)
{   
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.push_back(boost::bind(&PlanningFrame::computeSaveQueryButtonClicked, this));
  new_action_condition_.notify_all();
}

void moveit_rviz_plugin::PlanningFrame::deleteSceneButtonClicked(void)
{   
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.push_back(boost::bind(&PlanningFrame::computeDeleteSceneButtonClicked, this));
  new_action_condition_.notify_all();
}

void moveit_rviz_plugin::PlanningFrame::deleteQueryButtonClicked(void)
{   
  boost::mutex::scoped_lock slock(action_lock_);
  actions_.push_back(boost::bind(&PlanningFrame::computeDeleteQueryButtonClicked, this));
  new_action_condition_.notify_all();
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
  moveit_msgs::MotionPlanRequest mreq;
  constructPlanningRequest(mreq);
  plan_execution_->planOnly(mreq);
  const plan_execution::PlanExecution::Result &res = plan_execution_->getLastResult();
  if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    planning_models::KinematicStatePtr start_state(new planning_models::KinematicState(plan_execution_->getPlanningSceneMonitor()->getPlanningScene()->getCurrentState()));
    planning_models::robotStateToKinematicState(*plan_execution_->getPlanningSceneMonitor()->getPlanningScene()->getTransforms(), res.trajectory_start_, *start_state);
    planning_display_->displayRobotTrajectory(start_state, res.planned_trajectory_states_);
    ui_->execute_button->setEnabled(true);
  }
}

void moveit_rviz_plugin::PlanningFrame::computeExecuteButtonClicked(void)
{
  const plan_execution::PlanExecution::Result &res = plan_execution_->getLastResult();
  plan_execution_->getTrajectoryExecutionManager().push(res.planned_trajectory_);
  plan_execution_->getTrajectoryExecutionManager().execute();
}

void moveit_rviz_plugin::PlanningFrame::computePlanAndExecuteButtonClickedDisplayHelper(void)
{ 
  const plan_execution::PlanExecution::Result &res = plan_execution_->getLastResult();
  if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    planning_models::KinematicStatePtr start_state(new planning_models::KinematicState(plan_execution_->getPlanningSceneMonitor()->getPlanningScene()->getCurrentState()));
    planning_models::robotStateToKinematicState(*plan_execution_->getPlanningSceneMonitor()->getPlanningScene()->getTransforms(), res.trajectory_start_, *start_state);
    planning_display_->displayRobotTrajectory(start_state, res.planned_trajectory_states_);
  } 
}

void moveit_rviz_plugin::PlanningFrame::computePlanAndExecuteButtonClicked(void)
{    
  moveit_msgs::MotionPlanRequest mreq;
  constructPlanningRequest(mreq);
  plan_execution::PlanExecution::Options opt;
  opt.replan_ = ui_->allow_replanning->isChecked();
  opt.look_around_ = ui_->allow_looking->isChecked();
  opt.beforeExecutionCallback_ = boost::bind(&PlanningFrame::computePlanAndExecuteButtonClickedDisplayHelper, this);
  plan_execution_->planAndExecute(mreq, opt);
  ui_->plan_and_execute_button->setEnabled(true);
}

void moveit_rviz_plugin::PlanningFrame::computeSetStartToCurrentButtonClicked(void)
{    
  planning_models::KinematicStatePtr current_state(new planning_models::KinematicState(plan_execution_->getPlanningSceneMonitor()->getPlanningScene()->getCurrentState()));
  planning_display_->setQueryStartState(current_state);
}

void moveit_rviz_plugin::PlanningFrame::computeSetGoalToCurrentButtonClicked(void)
{    
  planning_models::KinematicStatePtr current_state(new planning_models::KinematicState(plan_execution_->getPlanningSceneMonitor()->getPlanningScene()->getCurrentState()));
  planning_display_->setQueryGoalState(current_state);
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
  planning_scene_storage_->getPlanningSceneNames(names);
  
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    std::vector<moveit_warehouse::MotionPlanRequestWithMetadata> planning_queries;
    std::vector<std::string> query_names;
    planning_scene_storage_->getPlanningQueries(planning_queries, query_names, names[i]);
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

void moveit_rviz_plugin::PlanningFrame::computeDatabaseConnectButtonClicked(void)
{
  if (planning_scene_storage_)
  {
    ui_->planning_scene_tree->setUpdatesEnabled(false); 
    ui_->planning_scene_tree->clear();
    ui_->planning_scene_tree->setUpdatesEnabled(true); 

    ui_->database_connect_button->setUpdatesEnabled(false); 
    ui_->database_connect_button->setText(QString::fromStdString("Connect"));
    ui_->database_connect_button->setUpdatesEnabled(true); 

    planning_scene_storage_.reset();
    ui_->load_scene_button->setEnabled(false);
    ui_->load_query_button->setEnabled(false);
    ui_->save_query_button->setEnabled(false);
    ui_->save_scene_button->setEnabled(false);
    ui_->delete_query_button->setEnabled(false);
    ui_->delete_scene_button->setEnabled(false);
  }
  else
  {      
    ui_->database_connect_button->setUpdatesEnabled(false); 
    ui_->database_connect_button->setText(QString::fromStdString("Connecting ..."));
    ui_->database_connect_button->setUpdatesEnabled(true); 
    try
    {
      planning_scene_storage_.reset(new moveit_warehouse::PlanningSceneStorage(ui_->database_host->text().toStdString(),
                                                                               ui_->database_port->value(), 5.0));
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("%s", ex.what());  
      ui_->database_connect_button->setUpdatesEnabled(false); 
      ui_->database_connect_button->setText(QString::fromStdString("Connect"));
      ui_->database_connect_button->setUpdatesEnabled(true); 
      return;
    }
    ui_->database_connect_button->setUpdatesEnabled(false); 
    ui_->database_connect_button->setText(QString::fromStdString("Disconnect"));
    ui_->database_connect_button->setUpdatesEnabled(true); 
    populatePlanningSceneTreeView();
    ui_->save_scene_button->setEnabled(true);
  }
}

void moveit_rviz_plugin::PlanningFrame::computeSaveSceneButtonClicked(void)
{
  if (planning_scene_storage_)
  {
    moveit_msgs::PlanningScene msg;
    plan_execution_->getPlanningSceneMonitor()->getPlanningScene()->getPlanningSceneMsg(msg);
    planning_scene_storage_->addPlanningScene(msg); 
    populatePlanningSceneTreeView();
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
          const std::string& topic = planning_display_->getPlanningSceneTopic();
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
          // do stuff with mp
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
      populatePlanningSceneTreeView();
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
      populatePlanningSceneTreeView();
    }
  }
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
        ui_->planning_scene_tree->setUpdatesEnabled(false);
        s->parent()->removeChild(s);  
        ui_->planning_scene_tree->setUpdatesEnabled(true);
      }
    }
  }
}
