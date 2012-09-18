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
#include <QTreeWidgetItem>
#include <move_group_interface/move_group.h>
#include <moveit_warehouse/warehouse.h>
#include <planning_scene_monitor/planning_scene_monitor.h>

namespace rviz
{
class DisplayContext;
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

  void changePlanningGroup(void);
  void enable(void);
  void disable(void);
  void sceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);
  
protected:

  void constructPlanningRequest(moveit_msgs::MotionPlanRequest &mreq);
  
  PlanningDisplay *planning_display_;  
  rviz::DisplayContext* context_;
  Ui::MotionPlanningFrame *ui_;
  
  boost::shared_ptr<move_group_interface::MoveGroup> move_group_;
  boost::shared_ptr<move_group_interface::MoveGroup::Plan> current_plan_;
  boost::shared_ptr<moveit_warehouse::PlanningSceneStorage> planning_scene_storage_;

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
  void loadSceneButtonClicked(void);
  void loadQueryButtonClicked(void);
  void allowReplanningToggled(bool checked);
  void allowLookingToggled(bool checked);
  void planningAlgorithmIndexChanged(int index);
  void importSceneButtonClicked(void);
  void clearSceneButtonClicked(void);
  void sceneScaleChanged(int value);
  void sceneScaleStartChange(void);
  void sceneScaleEndChange(void);
  void removeObjectButtonClicked(void);
  void selectedCollisionObjectChanged(void);
  void objectXValueChanged(double v);
  void objectYValueChanged(double v);
  void objectZValueChanged(double v);
  void objectRXValueChanged(double v);
  void objectRYValueChanged(double v);
  void objectRZValueChanged(double v);
    
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
  void computeDatabaseConnectButtonClickedHelper(int mode);
  void computeSaveSceneButtonClicked(void);
  void computeSaveQueryButtonClicked(void);
  void computeDeleteSceneButtonClicked(void);
  void computeDeleteQueryButtonClicked(void);
  void computeDeleteQueryButtonClickedHelper(QTreeWidgetItem *s);
  void checkPlanningSceneTreeEnabledButtons(void);
  void computeLoadSceneButtonClicked(void);
  void computeLoadQueryButtonClicked(void);
  void populatePlannersList(const moveit_msgs::PlannerInterfaceDescription &desc);
  void changePlanningGroupHelper(void);
  void populateCollisionObjectsList(void);
  void objectPoseValueChanged(int index, double value);
  
  ros::NodeHandle nh_;
  ros::Publisher planning_scene_publisher_;
  
  collision_detection::CollisionWorld::ObjectConstPtr scaled_object_;
};

}

#endif

