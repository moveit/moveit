/*
 * Copyright (c) 2013, Willow Garage, Inc.
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
#include <ui_utils.h>

#include <QShortcut>
#include <QFileDialog>

#include <assert.h>
#include <string>
#include <rviz/tool_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/display_factory.h>
#include <rviz/selection/selection_manager.h>

namespace benchmark_tool
{

const char * MainWindow::ROBOT_DESCRIPTION_PARAM = "robot_description";
const char * MainWindow::ROBOT_DESCRIPTION_SEMANTIC_PARAM = "robot_description_semantic";
const unsigned int MainWindow::DEFAULT_WAREHOUSE_PORT = 33830;

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    goal_pose_dragging_(false)
{
  setWindowTitle("Benchmark Tool");

  goal_offset_.setIdentity();

  ui_.setupUi(this);
  settings_.reset(new QSettings("WillowGarage", "Benchmark Tool"));
  QVariant previous_database_names = settings_->value("database_name", QStringList());
  ui_.db_combo->addItems(previous_database_names.toStringList());

  robot_loader_dialog_ = new QDialog(0,0);
  load_robot_ui_.setupUi(robot_loader_dialog_);

  //Rviz render panel
  render_panel_ = new rviz::RenderPanel();
  ui_.render_widget->addWidget(render_panel_);
  ui_.splitter->setStretchFactor(1, 4);

  visualization_manager_ = new rviz::VisualizationManager(render_panel_);
  render_panel_->initialize(visualization_manager_->getSceneManager(), visualization_manager_);

  visualization_manager_->initialize();
  visualization_manager_->startUpdate();

  //Grid display
  visualization_manager_->createDisplay("rviz/Grid", "Grid", true);

  //Planning scene display
  scene_display_ = new moveit_rviz_plugin::PlanningSceneDisplay();
  scene_display_->setName("Planning Scene");
  scene_display_->subProp("Robot Description")->setValue(ROBOT_DESCRIPTION_PARAM);
  scene_display_->subProp("Scene Geometry")->subProp("Scene Alpha")->setValue(1.0);
  visualization_manager_->addDisplay( scene_display_, true );

  //Interactive Marker display
  int_marker_display_ = visualization_manager_->getDisplayFactory()->make("rviz/InteractiveMarkers");
  int_marker_display_->initialize(visualization_manager_);
  int_marker_display_->setEnabled(true);
  int_marker_display_->subProp("Update Topic")->setValue(QString::fromStdString(robot_interaction::RobotInteraction::INTERACTIVE_MARKER_TOPIC + "/update"));

  if ( scene_display_ )
  {
    configure();
    if (ui_.planning_group_combo->count() > 0)
    {
      planningGroupChanged(ui_.planning_group_combo->currentText());
    }

    rviz::Tool * interact_tool = visualization_manager_->getToolManager()->addTool("rviz/Interact");
    if (interact_tool)
    {
      visualization_manager_->getToolManager()->setCurrentTool(interact_tool);
      interact_tool->activate();
    }
    visualization_manager_->getSelectionManager()->enableInteraction(true);

    //Setup ui: colors and icons
    ui_.db_connect_button->setStyleSheet("QPushButton { color : red }");

    ui_.goal_poses_open_button->setIcon(QIcon::fromTheme("document-open", QApplication::style()->standardIcon(QStyle::SP_DirOpenIcon)));
    ui_.goal_poses_add_button->setIcon(QIcon::fromTheme("list-add", QApplication::style()->standardIcon(QStyle::SP_FileDialogNewFolder)));
    ui_.goal_poses_remove_button->setIcon(QIcon::fromTheme("list-remove", QApplication::style()->standardIcon(QStyle::SP_DialogDiscardButton)));
    ui_.goal_poses_save_button->setIcon(QIcon::fromTheme("document-save", QApplication::style()->standardIcon(QStyle::SP_DriveFDIcon)));
    ui_.goal_switch_visibility_button->setIcon(QApplication::style()->standardIcon(QStyle::SP_DialogDiscardButton));

    ui_.start_states_open_button->setIcon(QIcon::fromTheme("document-open", QApplication::style()->standardIcon(QStyle::SP_DirOpenIcon)));
    ui_.start_states_add_button->setIcon(QIcon::fromTheme("list-add", QApplication::style()->standardIcon(QStyle::SP_FileDialogNewFolder)));
    ui_.start_states_remove_button->setIcon(QIcon::fromTheme("list-remove", QApplication::style()->standardIcon(QStyle::SP_DialogDiscardButton)));
    ui_.start_states_save_button->setIcon(QIcon::fromTheme("document-save", QApplication::style()->standardIcon(QStyle::SP_DriveFDIcon)));

    ui_.trajectory_open_button->setIcon(QIcon::fromTheme("document-open", QApplication::style()->standardIcon(QStyle::SP_DirOpenIcon)));
    ui_.trajectory_add_button->setIcon(QIcon::fromTheme("list-add", QApplication::style()->standardIcon(QStyle::SP_FileDialogNewFolder)));
    ui_.trajectory_remove_button->setIcon(QIcon::fromTheme("list-remove", QApplication::style()->standardIcon(QStyle::SP_DialogDiscardButton)));
    ui_.trajectory_save_button->setIcon(QIcon::fromTheme("document-save", QApplication::style()->standardIcon(QStyle::SP_DriveFDIcon)));

    //Connect signals and slots
    connect(ui_.actionExit, SIGNAL( triggered(bool) ), this, SLOT( exitActionTriggered(bool) ));
    connect(ui_.actionOpen, SIGNAL( triggered(bool) ), this, SLOT( openActionTriggered(bool) ));
    connect(ui_.planning_group_combo, SIGNAL( currentIndexChanged ( const QString & ) ), this, SLOT( planningGroupChanged( const QString & ) ));
    connect(ui_.db_connect_button, SIGNAL( clicked() ), this, SLOT( dbConnectButtonClicked() ));
    connect(ui_.load_scene_button, SIGNAL( clicked() ), this, SLOT( loadSceneButtonClicked() ));
    connect(ui_.planning_scene_list, SIGNAL( itemDoubleClicked (QListWidgetItem *) ), this, SLOT( loadSceneButtonClicked(QListWidgetItem *) ));
    connect(ui_.robot_interaction_button, SIGNAL( clicked() ), this, SLOT( robotInteractionButtonClicked() ));

    //Goal poses
    connect( ui_.goal_poses_add_button, SIGNAL( clicked() ), this, SLOT( createGoalPoseButtonClicked() ));
    connect( ui_.goal_poses_remove_button, SIGNAL( clicked() ), this, SLOT( deleteGoalsOnDBButtonClicked() ));
    connect( ui_.load_poses_filter_text, SIGNAL( returnPressed() ), this, SLOT( loadGoalsFromDBButtonClicked() ));
    connect( ui_.goal_poses_open_button, SIGNAL( clicked() ), this, SLOT( loadGoalsFromDBButtonClicked() ));
    connect( ui_.goal_poses_save_button, SIGNAL( clicked() ), this, SLOT( saveGoalsOnDBButtonClicked() ));
    connect( ui_.goal_switch_visibility_button, SIGNAL( clicked() ), this, SLOT( switchGoalVisibilityButtonClicked() ));
    connect( ui_.goal_poses_list, SIGNAL( itemSelectionChanged() ), this, SLOT( goalPoseSelectionChanged() ));
    connect( ui_.goal_poses_list, SIGNAL( itemDoubleClicked(QListWidgetItem *) ), this, SLOT( goalPoseDoubleClicked(QListWidgetItem *) ));
    connect( ui_.show_x_checkbox, SIGNAL( stateChanged( int ) ), this, SLOT( visibleAxisChanged( int ) ));
    connect( ui_.show_y_checkbox, SIGNAL( stateChanged( int ) ), this, SLOT( visibleAxisChanged( int ) ));
    connect( ui_.show_z_checkbox, SIGNAL( stateChanged( int ) ), this, SLOT( visibleAxisChanged( int ) ));
    connect( ui_.goal_offset_roll, SIGNAL( editingFinished ( )), this, SLOT( goalOffsetChanged( ) ));
    connect( ui_.goal_offset_pitch, SIGNAL( editingFinished ( )), this, SLOT( goalOffsetChanged( ) ));
    connect( ui_.goal_offset_yaw, SIGNAL( editingFinished ( )), this, SLOT( goalOffsetChanged( ) ));

    connect( ui_.check_goal_collisions_button, SIGNAL( clicked( ) ), this, SLOT( checkGoalsInCollision( ) ));
    connect( ui_.check_goal_reachability_button, SIGNAL( clicked( ) ), this, SLOT( checkGoalsReachable( ) ));
    connect( ui_.run_benchmark_button, SIGNAL( clicked( ) ), this, SLOT( runBenchmark( ) ));
    connect( ui_.load_results_button, SIGNAL( clicked( ) ), this, SLOT( loadBenchmarkResults( ) ));

    connect( ui_.start_states_add_button, SIGNAL( clicked() ), this, SLOT( saveStartStateButtonClicked() ));
    connect( ui_.start_states_remove_button, SIGNAL( clicked() ), this, SLOT( deleteStatesOnDBButtonClicked() ));
    connect( ui_.load_states_filter_text, SIGNAL( returnPressed() ), this, SLOT( loadStatesFromDBButtonClicked() ));
    connect( ui_.start_states_open_button, SIGNAL( clicked() ), this, SLOT( loadStatesFromDBButtonClicked() ));
    connect( ui_.start_states_save_button, SIGNAL( clicked() ), this, SLOT( saveStatesOnDBButtonClicked() ));
    connect( ui_.start_states_list, SIGNAL( itemDoubleClicked(QListWidgetItem*) ), this, SLOT( startStateItemDoubleClicked(QListWidgetItem*) ));

    QShortcut *copy_goals_shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_C), ui_.goal_poses_list);
    connect(copy_goals_shortcut, SIGNAL( activated() ), this, SLOT( copySelectedGoalPoses() ) );

    //Trajectories
    connect( ui_.trajectory_add_button, SIGNAL( clicked() ), this, SLOT( createTrajectoryButtonClicked() ));
    connect( ui_.trajectory_remove_button, SIGNAL( clicked() ), this, SLOT( removeTrajectoryButtonClicked() ));
    connect( ui_.trajectory_open_button, SIGNAL( clicked() ), this, SLOT( loadTrajectoriesFromDBButtonClicked() ));
    connect( ui_.trajectory_save_button, SIGNAL( clicked() ), this, SLOT( saveTrajectoriesOnDBButtonClicked() ));
    connect( ui_.trajectory_list, SIGNAL( itemSelectionChanged() ), this, SLOT( trajectorySelectionChanged() ));
    connect( ui_.trajectory_nwaypoints_spin, SIGNAL( valueChanged ( int )), this, SLOT( trajectoryNWaypointsChanged( int ) ));
    connect( ui_.trajectory_execute_button, SIGNAL( clicked() ), this, SLOT( trajectoryExecuteButtonClicked() ));

    //Start a QTimer for handling main loop jobs
    main_loop_jobs_timer_.reset(new QTimer(this));
    connect(main_loop_jobs_timer_.get(), SIGNAL( timeout() ), this, SLOT( MainLoop() ));
    main_loop_jobs_timer_->start(1000 / MAIN_LOOP_RATE);
  }
  else
  {
    ROS_ERROR("Cannot load robot. Is the robot_description parameter set?");
    exit(0);
  }
}

MainWindow::~MainWindow()
{
}

void MainWindow::exitActionTriggered(bool)
{
  QApplication::quit();
}

void MainWindow::openActionTriggered(bool)
{
  QString urdf_path = QFileDialog::getOpenFileName(this, tr("Select a robot description file"), tr(""), tr("URDF files (*.urdf)"));

  if (!urdf_path.isEmpty())
  {
    QString srdf_path = QFileDialog::getOpenFileName(this, tr("Select a semantic robot description file"), tr(""), tr("SRDF files (*.srdf)"));

    if (!srdf_path.isEmpty())
    {
      loadNewRobot(urdf_path.toStdString(), srdf_path.toStdString());
    }
  }
}

void MainWindow::loadNewRobot(const std::string &urdf_path, const std::string &srdf_path)
{
  load_robot_ui_.status_label->setText("");
  load_robot_ui_.load_progress_bar->setValue(0);

  robot_loader_dialog_->show();

  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

  //Load urdf
  boost::filesystem::path boost_urdf_path(urdf_path);
  load_robot_ui_.status_label->setText(QString::fromStdString("Loading urdf " + boost_urdf_path.string()));
  load_robot_ui_.load_progress_bar->setValue(10);
  if (boost::filesystem::exists(boost_urdf_path))
  {
    std::ifstream urdf_input_stream(boost_urdf_path.string().c_str());
    std::stringstream urdf_sstr;
    urdf_sstr << urdf_input_stream.rdbuf();
    ros::param::set(ROBOT_DESCRIPTION_PARAM, urdf_sstr.str());
  }
  else
  {
    ROS_ERROR("Cannot load URDF file");
  }

  //Load srdf
  boost::filesystem::path boost_srdf_path(srdf_path);
  load_robot_ui_.status_label->setText(QString::fromStdString("Loading srdf " + boost_srdf_path.string()));
  load_robot_ui_.load_progress_bar->setValue(20);
  if (boost::filesystem::exists(boost_srdf_path))
  {
    std::ifstream srdf_input_stream(boost_srdf_path.string().c_str());
    std::stringstream srdf_sstr;
    srdf_sstr << srdf_input_stream.rdbuf();
    ros::param::set(ROBOT_DESCRIPTION_SEMANTIC_PARAM, srdf_sstr.str());
  }
  else
  {
    ROS_ERROR("Cannot load SRDF file");
  }

  //Load kinematics.yaml, joint_limits.yaml and ompl_planning.yaml
  //TODO: Can we assume kinematics.yaml to be in the same folder as the srdf?
  boost::filesystem::path kinematics_file = boost::filesystem::operator/(boost_srdf_path.branch_path(), "kinematics.yaml");
  load_robot_ui_.status_label->setText(QString::fromStdString("Loading " + kinematics_file.string()));
  load_robot_ui_.load_progress_bar->setValue(30);
  if (boost::filesystem::exists( kinematics_file ) && boost::filesystem::is_regular_file( kinematics_file ))
  {
    if (system(("rosparam load " + kinematics_file.string()).c_str()) < 0)
    {
      ROS_ERROR("Couldn't load kinematics.yaml file");
    }
  }

  boost::filesystem::path joint_limits_file = boost::filesystem::operator/(boost_srdf_path.branch_path(), "joint_limits.yaml");
  load_robot_ui_.status_label->setText(QString::fromStdString("Loading " + joint_limits_file.string()));
  load_robot_ui_.load_progress_bar->setValue(40);
  if (boost::filesystem::exists( joint_limits_file ) && boost::filesystem::is_regular_file( joint_limits_file ))
  {
    if (system(("rosparam load " + joint_limits_file.string()).c_str()) < 0)
    {
      ROS_ERROR("Couldn't load joint_limits.yaml file");
    }
  }

  boost::filesystem::path ompl_planning_file = boost::filesystem::operator/(boost_srdf_path.branch_path(), "ompl_planning.yaml");
  load_robot_ui_.status_label->setText(QString::fromStdString("Loading " + ompl_planning_file.string()));
  load_robot_ui_.load_progress_bar->setValue(50);
  if (boost::filesystem::exists( ompl_planning_file ) && boost::filesystem::is_regular_file( ompl_planning_file ))
  {
    if (system(("rosparam load " + ompl_planning_file.string()).c_str()) < 0)
    {
      ROS_ERROR("Couldn't load ompl_planning.yaml file");
    }
  }

  load_robot_ui_.status_label->setText(QString("Resetting scene display... "));
  load_robot_ui_.load_progress_bar->setValue(60);
  std::string old_scene_name;
  if (scene_display_->getPlanningSceneRO())
    old_scene_name = scene_display_->getPlanningSceneRO()->getName();
  scene_display_->reset();

  if (configure())
  {
    //Reload the scene geometry if one scene was already loaded
    load_robot_ui_.status_label->setText(QString("Reloading scene... "));
    load_robot_ui_.load_progress_bar->setValue(90);
    QList<QListWidgetItem *> found_items = ui_.planning_scene_list->findItems(QString::fromStdString(old_scene_name), Qt::MatchExactly);
    if (found_items.size() > 0)
    {
      found_items[0]->setSelected(true);
      loadSceneButtonClicked();
    }

    //Reload the goals
    load_robot_ui_.status_label->setText(QString("Reloading goals... "));
    load_robot_ui_.load_progress_bar->setValue(95);
    if (ui_.goal_poses_list->count() > 0)
    {
      loadGoalsFromDBButtonClicked();
    }
    load_robot_ui_.status_label->setText(QString(""));
    load_robot_ui_.load_progress_bar->setValue(100);
  }
  robot_loader_dialog_->close();
  load_robot_ui_.load_progress_bar->setValue(0);
  QApplication::restoreOverrideCursor();
}

bool MainWindow::configure()
{
  if ( ! scene_display_->getPlanningSceneMonitor() || ! scene_display_->getPlanningSceneMonitor()->getRobotModel() )
  {
    ROS_ERROR("Cannot load robot");
    ui_.robot_interaction_button->setEnabled(false);
    ui_.load_scene_button->setEnabled(false);
    ui_.load_results_button->setEnabled(false);
    ui_.check_goal_collisions_button->setEnabled(false);
    ui_.check_goal_reachability_button->setEnabled(false);
    ui_.db_connect_button->setEnabled(false);
    ui_.goal_poses_add_button->setEnabled(false);
    ui_.goal_poses_remove_button->setEnabled(false);
    ui_.load_poses_filter_text->setEnabled(false);
    ui_.goal_poses_open_button->setEnabled(false);
    ui_.goal_poses_save_button->setEnabled(false);
    ui_.goal_switch_visibility_button->setEnabled(false);
    ui_.show_x_checkbox->setEnabled(false);
    ui_.show_y_checkbox->setEnabled(false);
    ui_.show_z_checkbox->setEnabled(false);
    ui_.goal_offset_roll->setEnabled(false);
    ui_.goal_offset_pitch->setEnabled(false);
    ui_.goal_offset_yaw->setEnabled(false);
    ui_.start_states_add_button->setEnabled(false);
    ui_.start_states_remove_button->setEnabled(false);
    ui_.start_states_open_button->setEnabled(false);
    ui_.start_states_save_button->setEnabled(false);
    ui_.run_benchmark_button->setEnabled(false);

    ui_.trajectory_nwaypoints_spin->setEnabled(false);
    ui_.trajectories_filter_text->setEnabled(false);
    ui_.trajectory_add_button->setEnabled(false);
    ui_.trajectory_remove_button->setEnabled(false);
    ui_.trajectory_save_button->setEnabled(false);
    ui_.trajectory_open_button->setEnabled(false);
    ui_.trajectory_execute_button->setEnabled(false);
    return false;
  }

  ui_.robot_interaction_button->setEnabled(true);
  ui_.load_scene_button->setEnabled(true);
  ui_.load_results_button->setEnabled(true);
  ui_.check_goal_collisions_button->setEnabled(true);
  ui_.check_goal_reachability_button->setEnabled(true);
  ui_.db_connect_button->setEnabled(true);
  ui_.goal_poses_add_button->setEnabled(true);
  ui_.goal_poses_remove_button->setEnabled(true);
  ui_.load_poses_filter_text->setEnabled(true);
  ui_.goal_poses_open_button->setEnabled(true);
  ui_.goal_poses_save_button->setEnabled(true);
  ui_.goal_switch_visibility_button->setEnabled(true);
  ui_.show_x_checkbox->setEnabled(true);
  ui_.show_y_checkbox->setEnabled(true);
  ui_.show_z_checkbox->setEnabled(true);
  ui_.goal_offset_roll->setEnabled(true);
  ui_.goal_offset_pitch->setEnabled(true);
  ui_.goal_offset_yaw->setEnabled(true);
  ui_.start_states_add_button->setEnabled(true);
  ui_.start_states_remove_button->setEnabled(true);
  ui_.start_states_open_button->setEnabled(true);
  ui_.start_states_save_button->setEnabled(true);

  ui_.trajectory_nwaypoints_spin->setEnabled(true);
  ui_.trajectories_filter_text->setEnabled(true);
  ui_.trajectory_add_button->setEnabled(true);
  ui_.trajectory_remove_button->setEnabled(true);
  ui_.trajectory_save_button->setEnabled(true);
  ui_.trajectory_open_button->setEnabled(true);
  ui_.trajectory_execute_button->setEnabled(true);

  //Set the fixed frame to the model frame
  load_robot_ui_.status_label->setText(QString("Setting fixed frame..."));
  load_robot_ui_.load_progress_bar->setValue(65);
  visualization_manager_->setFixedFrame(QString(scene_display_->getPlanningSceneMonitor()->getRobotModel()->getModelFrame().c_str()));
  int_marker_display_->setFixedFrame(QString::fromStdString(scene_display_->getPlanningSceneMonitor()->getRobotModel()->getModelFrame()));

  //robot interaction
  load_robot_ui_.status_label->setText(QString("Resetting robot interaction..."));
  load_robot_ui_.load_progress_bar->setValue(75);
  robot_interaction_.reset(new robot_interaction::RobotInteraction(scene_display_->getPlanningSceneMonitor()->getRobotModel()));

  //Configure robot-dependent ui elements
  ui_.load_states_filter_text->setText(QString::fromStdString(scene_display_->getPlanningSceneMonitor()->getRobotModel()->getName() + ".*"));

  //Get the list of planning groups and fill in the combo box
  load_robot_ui_.status_label->setText(QString("Updating planning groups..."));
  load_robot_ui_.load_progress_bar->setValue(85);
  std::vector<std::string> group_names = scene_display_->getPlanningSceneMonitor()->getRobotModel()->getJointModelGroupNames();
  ui_.planning_group_combo->clear();
  for (std::size_t i = 0; i < group_names.size(); i++)
  {
    ui_.planning_group_combo->addItem(QString(group_names[i].c_str()));
  }

  if (group_names.size() > 0)
    planningGroupChanged(QString(group_names[0].c_str()));

  return true;
}

void MainWindow::planningGroupChanged(const QString &text)
{
  if (robot_interaction_ && ! text.isEmpty())
  {
    robot_interaction_->decideActiveComponents(text.toStdString());
    if (robot_interaction_->getActiveEndEffectors().size() == 0)
      ROS_WARN_STREAM("No end-effectors defined for robot " << scene_display_->getPlanningSceneMonitor()->getRobotModel()->getName() << " and group " << text.toStdString());
    else
    {
      //Update the kinematic state associated to the goals
      for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); ++it)
      {
        it->second->setRobotState(scene_display_->getPlanningSceneRO()->getCurrentState());
        it->second->setEndEffector(robot_interaction_->getActiveEndEffectors()[0]);

        if (query_goal_state_)
        {
          robotInteractionButtonClicked();
          robotInteractionButtonClicked();
        }
      }
    }
  }
}

void MainWindow::robotInteractionButtonClicked()
{
  if (query_goal_state_ && robot_interaction_)
  {
    robot_interaction_->clearInteractiveMarkers();
    query_goal_state_.reset();
  }
  else if (robot_interaction_ && scene_display_)
  {
    query_goal_state_.reset(new robot_interaction::RobotInteraction::InteractionHandler("goal", scene_display_->getPlanningSceneRO()->getCurrentState(), scene_display_->getPlanningSceneMonitor()->getTFClient()));
    query_goal_state_->setUpdateCallback(boost::bind(&MainWindow::scheduleStateUpdate, this));
    query_goal_state_->setStateValidityCallback(boost::bind(&MainWindow::isIKSolutionCollisionFree, this, _1, _2));
    robot_interaction_->addInteractiveMarkers(query_goal_state_);
  }
  else
  {
    ROS_WARN("robot interaction not initialized");
  }
  robot_interaction_->publishInteractiveMarkers();
}

bool MainWindow::isIKSolutionCollisionFree(robot_state::JointStateGroup *group, const std::vector<double> &ik_solution)
{
  if (scene_display_)
  {
    group->setVariableValues(ik_solution);
    return !scene_display_->getPlanningSceneRO()->isStateColliding(*group->getRobotState(), group->getName());
  }
  else
    return true;
}

void MainWindow::scheduleStateUpdate()
{
  JobProcessing::addBackgroundJob(boost::bind(&MainWindow::scheduleStateUpdateBackgroundJob, this));
}

void MainWindow::scheduleStateUpdateBackgroundJob()
{
  scene_display_->getPlanningSceneRW()->setCurrentState(*query_goal_state_->getState());
  scene_display_->queueRenderSceneGeometry();
}

void MainWindow::dbConnectButtonClicked()
{
  JobProcessing::addBackgroundJob(boost::bind(&MainWindow::dbConnectButtonClickedBackgroundJob, this));
}

void MainWindow::dbConnectButtonClickedBackgroundJob()
{
  if (planning_scene_storage_)
  {
    planning_scene_storage_.reset();
    robot_state_storage_.reset();
    constraints_storage_.reset();
    trajectory_constraints_storage_.reset();
    ui_.planning_scene_list->clear();

    JobProcessing::addMainLoopJob(boost::bind(&setButtonState, ui_.db_connect_button, false, "Disconnected", "QPushButton { color : red }"));
  }
  else
  {
    int port = -1;
    QStringList host_port;

    host_port = ui_.db_combo->currentText().split(":");
    if (host_port.size() > 1)
    {
      try
      {
        port = boost::lexical_cast<int>(host_port[1].toStdString());
      }
      catch (...)
      {
      }
    }
    else
      port = DEFAULT_WAREHOUSE_PORT;

    if (port > 0 && ! host_port[0].isEmpty())
    {
      //Store into settings
      QVariant previous_database_names = settings_->value("database_name", QStringList());
      QStringList name_list = previous_database_names.toStringList();
      name_list.removeAll(ui_.db_combo->currentText());
      name_list.push_front(ui_.db_combo->currentText());
      static const short db_names_history_size = 5;
      if (name_list.size() > db_names_history_size)
        name_list.pop_back();
      settings_->setValue("database_name", name_list);
      ui_.db_combo->clear();
      ui_.db_combo->addItems(name_list);

      //Connect
      JobProcessing::addMainLoopJob(boost::bind(&setButtonState, ui_.db_connect_button, true, "Connecting...", "QPushButton { color : yellow }"));
      try
      {
        planning_scene_storage_.reset(new moveit_warehouse::PlanningSceneStorage(host_port[0].toStdString(),
                                                                                 port, 5.0));
        robot_state_storage_.reset(new moveit_warehouse::RobotStateStorage(host_port[0].toStdString(),
                                                                           port, 5.0));
        constraints_storage_.reset(new moveit_warehouse::ConstraintsStorage(host_port[0].toStdString(),
                                                                            port, 5.0));
        trajectory_constraints_storage_.reset(new moveit_warehouse::TrajectoryConstraintsStorage(host_port[0].toStdString(),
                                                                            port, 5.0));
        JobProcessing::addMainLoopJob(boost::bind(&setButtonState, ui_.db_connect_button, true, "Getting data...", "QPushButton { color : yellow }"));

        //Get all the scenes
        populatePlanningSceneList();

        JobProcessing::addMainLoopJob(boost::bind(&setButtonState, ui_.db_connect_button, true, "Connected", "QPushButton { color : green }"));
      }
      catch(std::runtime_error &ex)
      {
        ROS_ERROR("%s", ex.what());
        JobProcessing::addMainLoopJob(boost::bind(&setButtonState, ui_.db_connect_button, false, "Disconnected", "QPushButton { color : red }"));
        JobProcessing::addMainLoopJob(boost::bind(&showCriticalMessage, this, "Error", ex.what()));
        return;
      }
    }
    else
    {
      ROS_ERROR("Warehouse server must be introduced as host:port (eg. server.domain.com:33830)");
      JobProcessing::addMainLoopJob(boost::bind(&setButtonState, ui_.db_connect_button, false, "Disconnected", "QPushButton { color : red }"));
      JobProcessing::addMainLoopJob(boost::bind(&showCriticalMessage, this, "Error", "Warehouse server must be introduced as host:port (eg. server.domain.com:33830)"));
    }
  }
}

void MainWindow::populatePlanningSceneList(void)
{
  ui_.planning_scene_list->setUpdatesEnabled(false);

  ui_.planning_scene_list->clear();
  ui_.planning_scene_list->setSortingEnabled(true);
  ui_.planning_scene_list->sortItems(Qt::AscendingOrder);

  std::vector<std::string> names;
  planning_scene_storage_->getPlanningSceneNames(names);
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    ui_.planning_scene_list->addItem(names[i].c_str());
  }

  ui_.planning_scene_list->setUpdatesEnabled(true);
}

void MainWindow::loadSceneButtonClicked(QListWidgetItem *item)
{
  loadSceneButtonClicked();
}

void MainWindow::loadSceneButtonClicked(void)
{
  if (ui_.planning_scene_list->currentItem() && ui_.planning_scene_list->currentItem()->text().toStdString() != scene_display_->getPlanningSceneRO()->getName())
  {
    ui_.goal_poses_list->clear();
    ui_.trajectory_list->clear();
    goal_poses_.clear();
    trajectories_.clear();
    JobProcessing::addBackgroundJob(boost::bind(&MainWindow::loadSceneButtonClickedBackgroundJob, this));
  }
}

void MainWindow::loadSceneButtonClickedBackgroundJob(void)
{
  if (planning_scene_storage_)
  {
    QList<QListWidgetItem *> sel = ui_.planning_scene_list->selectedItems();
    if ( ! sel.empty())
    {
      QListWidgetItem *s = sel.front();
      std::string scene = s->text().toStdString();

      setStatusFromBackground(STATUS_INFO, QString::fromStdString("Attempting to load scene" + scene + "..."));

      moveit_warehouse::PlanningSceneWithMetadata scene_m;
      moveit_msgs::PlanningScene scene_msg;
      bool got_ps = false;
      try
      {
        got_ps = planning_scene_storage_->getPlanningScene(scene_m, scene);
      }
      catch (std::runtime_error &ex)
      {
        ROS_ERROR("%s", ex.what());
      }

      if (got_ps)
      {
        ROS_DEBUG("Loaded scene '%s'", scene.c_str());
        setStatusFromBackground(STATUS_INFO, QString::fromStdString("Rendering Scene..."));

        //Update the planning scene
        planning_scene_monitor::LockedPlanningSceneRW ps = scene_display_->getPlanningSceneRW();
        if (ps)
        {
          scene_msg = *scene_m;
          ps->getWorldNonConst()->clearObjects();
          for (std::size_t i = 0 ; i < scene_msg.object_colors.size() ; ++i)
            ps->setObjectColor(scene_msg.object_colors[i].id, scene_msg.object_colors[i].color);
          ps->processPlanningSceneWorldMsg(scene_msg.world);
          ps->setName(scene_msg.name);
          scene_display_->queueRenderSceneGeometry();
        }

        //Configure ui elements
        ui_.load_poses_filter_text->setText(QString::fromStdString(scene + ".*"));
        ui_.trajectories_filter_text->setText(QString::fromStdString(scene + ".*"));
        setStatusFromBackground(STATUS_INFO, QString::fromStdString(""));
      }
      else
      {
        ROS_WARN("Failed to load scene '%s'. Has the message format changed since the scene was saved?", scene.c_str());
      }
    }
  }
}


void MainWindow::updateGoalPoseMarkers(float wall_dt, float ros_dt)
{
  if (goal_pose_dragging_) {
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end() ; ++it)
      if (it->second->isVisible())
        it->second->imarker->update(wall_dt);
  }

  for (TrajectoryMap::iterator it = trajectories_.begin(); it != trajectories_.end() ; ++it)
    if (it->second->control_marker->isVisible())
    {
      it->second->control_marker->imarker->update(wall_dt);
      if (it->second->start_marker)
        it->second->start_marker->imarker->update(wall_dt);
      if (it->second->end_marker)
        it->second->end_marker->imarker->update(wall_dt);
    }
}

void MainWindow::MainLoop()
{
  int_marker_display_->update(1.0 / MAIN_LOOP_RATE, 1.0 / MAIN_LOOP_RATE);
  updateGoalPoseMarkers(1.0 / MAIN_LOOP_RATE, 1.0 / MAIN_LOOP_RATE);

  JobProcessing::executeMainLoopJobs();
}

void MainWindow::selectItemJob(QListWidgetItem *item, bool flag)
{
  item->setSelected(flag);
}

void MainWindow::setItemSelectionInList(const std::string &item_name, bool selection, QListWidget *list)
{
  QList<QListWidgetItem*> found_items = list->findItems(QString(item_name.c_str()), Qt::MatchExactly);
  for (std::size_t i = 0 ; i < found_items.size(); ++i)
    found_items[i]->setSelected(selection);
}

}
