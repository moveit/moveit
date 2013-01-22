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

#ifndef BT_MAIN_WINDOW_
#define BT_MAIN_WINDOW_

#include <QtGui/QMainWindow>
#include <QTimer>
#include "ui_main_window.h"

#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>

#include <moveit/planning_scene_rviz_plugin/kinematic_state_visualization.h>
#include <moveit/planning_scene_rviz_plugin/planning_scene_display.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/motion_planning_rviz_plugin/background_processing.h>
#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>

namespace benchmark_tool
{

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
  MainWindow(int argc, char **argv, QWidget *parent = 0);

  // pass the execution of this function call to a separate thread that runs in the background
  void addBackgroundJob(const boost::function<void(void)> &job);

  // queue the execution of this function for the next time the main update() loop gets called
  void addMainLoopJob(const boost::function<void(void)> &job);

  ~MainWindow();
public Q_SLOTS:

  void planningGroupChanged(const QString &text);
  void dbConnectButtonClicked();
  void dbConnectButtonClickedBackgroundJob();

  void loadSceneButtonClicked(void);

  //main loop processing
  void executeMainLoopJobs();


private:
  const static char * ROBOT_DESCRIPTION_PARAM;
  const static unsigned int DEFAULT_WAREHOUSE_PORT;

  Ui::MainWindow ui_;

  kinematic_state::KinematicStatePtr kinematic_state_;

  //rviz
  rviz::RenderPanel *render_panel_;
  rviz::VisualizationManager *visualization_manager_;
  moveit_rviz_plugin::PlanningSceneDisplay *scene_display_;

  //Warehouse
  boost::shared_ptr<moveit_warehouse::PlanningSceneStorage> planning_scene_storage_;
  boost::shared_ptr<moveit_warehouse::ConstraintsStorage> constraints_storage_;
  boost::shared_ptr<moveit_warehouse::RobotStateStorage> robot_state_storage_;

  void populatePlanningSceneList(void);

  //Background processing
  moveit_rviz_plugin::BackgroundProcessing background_process_;
  void loadSceneButtonClickedBackgroundJob(void);

  //Foreground processing
  const static unsigned int MAIN_LOOP_RATE = 20; //calls to executeMainLoopJobs per second
  std::deque<boost::function<void(void)> > main_loop_jobs_;
  boost::mutex main_loop_jobs_lock_;
  boost::shared_ptr<QTimer> main_loop_jobs_timer_;

  //Status and logging
  typedef enum {STATUS_WARN, STATUS_ERROR, STATUS_INFO} StatusType;
  void setStatus(StatusType st, const QString &text)
  {
    if (st == STATUS_WARN)
    {
      ROS_WARN_STREAM(text.toStdString());
      ui_.status_label->setText(text);
    }
    else if (st == STATUS_ERROR)
    {
      ROS_ERROR_STREAM(text.toStdString());
      ui_.status_label->setText(text);
    }
    else if (st == STATUS_INFO)
    {
      ui_.status_label->setText(text);
    }
  }

  void setStatusFromBackground(StatusType st, const QString &text)
  {
    addMainLoopJob(boost::bind(&MainWindow::setStatus, this, st, text));
  }

};

}

#endif
