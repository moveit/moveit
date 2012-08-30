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

#ifndef MOVEIT_RVIZ_PLUGIN_PLANNING_FRAME_
#define MOVEIT_RVIZ_PLUGIN_PLANNING_FRAME_

#include <QWidget>
#include <plan_execution/plan_execution.h>
#include <moveit_warehouse/warehouse.h>
#include <deque>
#include <boost/thread.hpp>

namespace rviz
{
class DisplayContext;
class FrameManager;
}

namespace Ui
{
class MotionPlanningFrame;
}

namespace moveit_rviz_plugin
{
class PlanningDisplay;

class PlanningFrame : public QWidget
{
Q_OBJECT

public:
  PlanningFrame(PlanningDisplay *pdisplay, rviz::DisplayContext *context, QWidget *parent = 0);
  ~PlanningFrame(void);

  void enable(const planning_scene_monitor::PlanningSceneMonitorPtr &planning_scene_monitor);
  void disable(void);
  
protected:

  void constructPlanningRequest(moveit_msgs::MotionPlanRequest &mreq);
  
  PlanningDisplay *planning_display_;  
  rviz::DisplayContext* context_;
  Ui::MotionPlanningFrame *ui_;
  
  boost::scoped_ptr<plan_execution::PlanExecution> plan_execution_;
  boost::scoped_ptr<moveit_warehouse::PlanningSceneStorage> planning_scene_storage_;

private Q_SLOTS:

  void planButtonClicked(void);  
  void executeButtonClicked(void);
  void planAndExecuteButtonClicked(void);
  void randomStatesButtonClicked(void);
  void setStartToCurrentButtonClicked(void);
  void setGoalToCurrentButtonClicked(void);
  void databaseConnectButtonClicked(void);
  void saveSceneButtonClicked(void);
  void planningSceneItemClicked(void);
  void saveQueryButtonClicked(void);
  void deleteSceneButtonClicked(void);
  void deleteQueryButtonClicked(void);

private:

  void computePlanButtonClicked(void);  
  void computeExecuteButtonClicked(void);
  void computePlanAndExecuteButtonClicked(void); 
  void computePlanAndExecuteButtonClickedDisplayHelper(void);
  void computeRandomStatesButtonClicked(void);
  void computeSetStartToCurrentButtonClicked(void);
  void computeSetGoalToCurrentButtonClicked(void);
  void populatePlanningSceneTreeView(void);
  void computeDatabaseConnectButtonClicked(void);
  void computeSaveSceneButtonClicked(void);
  void computeSaveQueryButtonClicked(void);
  void computeDeleteSceneButtonClicked(void);
  void computeDeleteQueryButtonClicked(void);
  void checkPlanningSceneTreeEnabledButtons(void);
  
  boost::scoped_ptr<boost::thread> processing_thread_;
  bool run_processing_thread_;
  
  boost::mutex action_lock_;
  boost::condition_variable new_action_condition_;
  std::deque<boost::function<void(void)> > actions_;

  void processingThread();
};
  
}

#endif

