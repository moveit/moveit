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
 * Author: Mario Prats
 */

#include <main_window.h>
#include <ui_utils.h>

#include <rviz/frame_manager.h>

namespace benchmark_tool
{

const char * MainWindow::ROBOT_DESCRIPTION_PARAM = "robot_description";
const unsigned int MainWindow::DEFAULT_WAREHOUSE_PORT = 33830;

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent)
{
  setWindowTitle("Benchmark Tool");

  ui_.setupUi(this);

  //Rviz render panel
  render_panel_ = new rviz::RenderPanel();
  ui_.render_widget->addWidget(render_panel_);

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

  if ( scene_display_ && scene_display_->getPlanningSceneRO())
  {
    kinematic_state_.reset(new kinematic_state::KinematicState(scene_display_->getPlanningSceneRO()->getCurrentState()));

    //Set the fixed frame to the model frame
    visualization_manager_->setFixedFrame(QString(scene_display_->getPlanningSceneMonitor()->getKinematicModel()->getModelFrame().c_str()));

    //Get the list of planning groups and fill in the combo box
    std::vector<std::string> group_names = scene_display_->getPlanningSceneMonitor()->getKinematicModel()->getJointModelGroupNames();
    for (std::size_t i = 0; i < group_names.size(); i++)
    {
      ui_.planning_group_combo->addItem(QString(group_names[i].c_str()));
    }

    //Setup ui
    ui_.db_connect_button->setStyleSheet("QPushButton { color : red }");

    //Connect signals and slots
    connect(ui_.planning_group_combo, SIGNAL( currentIndexChanged ( const QString & ) ), this, SLOT( planningGroupChanged( const QString & ) ));
    connect(ui_.db_connect_button, SIGNAL( clicked() ), this, SLOT( dbConnectButtonClicked() ));
    connect(ui_.load_scene_button, SIGNAL( clicked() ), this, SLOT( loadSceneButtonClicked() ));

    //Start a QTimer for handling main loop jobs
    main_loop_jobs_timer_.reset(new QTimer(this));
    connect(main_loop_jobs_timer_.get(), SIGNAL( timeout() ), this, SLOT( executeMainLoopJobs() ));
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
  kinematic_state_.reset();
  delete scene_display_;
  delete visualization_manager_;
  delete render_panel_;
}

void MainWindow::planningGroupChanged(const QString &text)
{
  ROS_DEBUG_STREAM("Planning group changed to " << text.toStdString());
}

void MainWindow::dbConnectButtonClicked()
{
  addBackgroundJob(boost::bind(&MainWindow::dbConnectButtonClickedBackgroundJob, this));
}

void MainWindow::dbConnectButtonClickedBackgroundJob()
{
  if (planning_scene_storage_)
  {
    planning_scene_storage_.reset();
    robot_state_storage_.reset();
    constraints_storage_.reset();
    ui_.planning_scene_list->clear();

    addMainLoopJob(boost::bind(&setButtonTextAndColor, ui_.db_connect_button, "Disconnected", "QPushButton { color : red }"));
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
      addMainLoopJob(boost::bind(&setButtonTextAndColor, ui_.db_connect_button, "Connecting...", "QPushButton { color : yellow }"));
      try
      {
        planning_scene_storage_.reset(new moveit_warehouse::PlanningSceneStorage(host_port[0].toStdString(),
                                                                                 port, 5.0));
        robot_state_storage_.reset(new moveit_warehouse::RobotStateStorage(host_port[0].toStdString(),
                                                                           port, 5.0));
        constraints_storage_.reset(new moveit_warehouse::ConstraintsStorage(host_port[0].toStdString(),
                                                                            port, 5.0));
        addMainLoopJob(boost::bind(&setButtonTextAndColor, ui_.db_connect_button, "Getting data...", "QPushButton { color : yellow }"));

        //Get all the scenes
        populatePlanningSceneList();

        addMainLoopJob(boost::bind(&setButtonTextAndColor, ui_.db_connect_button, "Connected", "QPushButton { color : green }"));
      }
      catch(std::runtime_error &ex)
      {
        ROS_ERROR("%s", ex.what());
        addMainLoopJob(boost::bind(&setButtonTextAndColor, ui_.db_connect_button, "Disconnected", "QPushButton { color : red }"));
        addMainLoopJob(boost::bind(&showCriticalMessage, this, "Error", ex.what()));
        return;
      }
    }
    else
    {
      ROS_ERROR("Malformed url. Warehouse host must be introduced as host:port");
      addMainLoopJob(boost::bind(&setButtonTextAndColor, ui_.db_connect_button, "Disconnected", "QPushButton { color : red }"));
      addMainLoopJob(boost::bind(&showCriticalMessage, this, "Error", "Malformed url. Warehouse host must be introduced as host:port"));
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

void MainWindow::loadSceneButtonClicked(void)
{
  addBackgroundJob(boost::bind(&MainWindow::loadSceneButtonClickedBackgroundJob, this));
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
        ps->setPlanningSceneMsg(*scene_m.get());
        scene_display_->queueRenderSceneGeometry();
        setStatusFromBackground(STATUS_INFO, QString::fromStdString(""));
      }
      else
      {
        ROS_WARN("Failed to load scene '%s'. Has the message format changed since the scene was saved?", scene.c_str());
      }
    }
  }
}

void MainWindow::addBackgroundJob(const boost::function<void(void)> &job)
{
  background_process_.addJob(job);
}

void MainWindow::addMainLoopJob(const boost::function<void(void)> &job)
{
  boost::mutex::scoped_lock slock(main_loop_jobs_lock_);
  main_loop_jobs_.push_back(job);
}

void MainWindow::executeMainLoopJobs()
{
  main_loop_jobs_lock_.lock();
  while (!main_loop_jobs_.empty())
  {
    boost::function<void(void)> fn = main_loop_jobs_.front();
    main_loop_jobs_.pop_front();
    main_loop_jobs_lock_.unlock();
    try
    {
      fn();
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Exception caught executing main loop job: %s", ex.what());
    }
    catch(...)
    {
      ROS_ERROR("Exception caught executing main loop job");
    }
    main_loop_jobs_lock_.lock();
  }
  main_loop_jobs_lock_.unlock();
}

}
