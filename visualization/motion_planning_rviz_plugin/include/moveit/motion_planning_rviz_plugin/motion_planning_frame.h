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

#ifndef MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MOTION_PLANNING_FRAME_
#define MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_MOTION_PLANNING_FRAME_

#include <QWidget>
#include <QTreeWidgetItem>
#include <QListWidgetItem>

#ifndef Q_MOC_RUN
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_interaction/robot_interaction.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rviz/default_plugin/interactive_markers/interactive_marker.h>
#endif

#include <moveit_msgs/MotionPlanRequest.h>
#include <map>
#include <string>

namespace rviz
{
class DisplayContext;
}

namespace Ui
{
class MotionPlanningUI;
}

namespace moveit_warehouse
{
class PlanningSceneStorage;
class ConstraintsStorage;
class RobotStateStorage;
}

namespace moveit_rviz_plugin
{
class MotionPlanningDisplay;
  
class MotionPlanningFrame : public QWidget
{
  friend class MotionPlanningDisplay;
  Q_OBJECT
  
public:
  MotionPlanningFrame(MotionPlanningDisplay *pdisplay, rviz::DisplayContext *context, QWidget *parent = 0);
  ~MotionPlanningFrame();

  void changePlanningGroup();
  void enable();
  void disable();
  void sceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);

protected:
  static const int ITEM_TYPE_SCENE = 1;
  static const int ITEM_TYPE_QUERY = 2;

  void constructPlanningRequest(moveit_msgs::MotionPlanRequest &mreq);
  
  void updateSceneMarkers(float wall_dt, float ros_dt);

  MotionPlanningDisplay *planning_display_;  
  rviz::DisplayContext* context_;
  Ui::MotionPlanningUI *ui_;
  
  boost::shared_ptr<move_group_interface::MoveGroup> move_group_;
  ros::WallTime move_group_construction_time_;

  boost::shared_ptr<move_group_interface::MoveGroup::Plan> current_plan_;
  boost::shared_ptr<moveit_warehouse::PlanningSceneStorage> planning_scene_storage_;
  boost::shared_ptr<moveit_warehouse::ConstraintsStorage> constraints_storage_;
  boost::shared_ptr<moveit_warehouse::RobotStateStorage> robot_state_storage_;

  boost::shared_ptr<rviz::InteractiveMarker> scene_marker_;

  typedef std::map<std::string, moveit_msgs::RobotState> RobotStateMap;
  typedef std::pair<std::string, moveit_msgs::RobotState> RobotStatePair;
  RobotStateMap robot_states_;
 
private Q_SLOTS:

  //Context tab
  void databaseConnectButtonClicked();
  void publishSceneButtonClicked();
  void planningAlgorithmIndexChanged(int index);
  void resetDbButtonClicked();

  //Planning tab
  void planButtonClicked();  
  void executeButtonClicked();
  void planAndExecuteButtonClicked();
  void allowReplanningToggled(bool checked);
  void allowLookingToggled(bool checked);
  void pathConstraintsIndexChanged(int index);
  void useStartStateButtonClicked();
  void useGoalStateButtonClicked();  

  //Scene Objects tab
  void importFileButtonClicked();
  void importUrlButtonClicked();
  void clearSceneButtonClicked();
  void sceneScaleChanged(int value);
  void sceneScaleStartChange();
  void sceneScaleEndChange();
  void removeObjectButtonClicked();
  void selectedCollisionObjectChanged();
  void objectPoseValueChanged(double value);
  void collisionObjectChanged(QListWidgetItem *item);
  void imProcessFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);
  void copySelectedCollisionObject();
  void exportAsTextButtonClicked();
  void importFromTextButtonClicked();
  
  //Stored scenes tab
  void saveSceneButtonClicked();
  void planningSceneItemClicked();
  void saveQueryButtonClicked();
  void deleteSceneButtonClicked();
  void deleteQueryButtonClicked();
  void loadSceneButtonClicked();
  void loadQueryButtonClicked();
  void warehouseItemNameChanged(QTreeWidgetItem *item, int column);

  //States tab
  void loadStateButtonClicked();
  void saveStartStateButtonClicked();
  void saveGoalStateButtonClicked();
  void removeStateButtonClicked();
  void clearStatesButtonClicked();
  void setAsStartStateButtonClicked();
  void setAsGoalStateButtonClicked();

  //General
  void tabChanged(int index);
  
private:

  //Context tab
  void computeDatabaseConnectButtonClicked();
  void computeDatabaseConnectButtonClickedHelper(int mode);
  void computeResetDbButtonClicked(const std::string &db);
  void populatePlannersList(const moveit_msgs::PlannerInterfaceDescription &desc);

  //Planning tab
  void computePlanButtonClicked();  
  void computeExecuteButtonClicked();
  void computePlanAndExecuteButtonClicked(); 
  void computePlanAndExecuteButtonClickedDisplayHelper();
  void populateConstraintsList();
  void populateConstraintsList(const std::vector<std::string> &constr);
  void configureForPlanning();
  void configureWorkspace();  
  void updateQueryStateHelper(robot_state::RobotState &state, const std::string &v);
  void fillStateSelectionOptions();

  //Scene objects tab
  void addObject(const collision_detection::WorldPtr &world, const std::string &id,
                 const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose);
  void updateCollisionObjectPose(bool update_marker_position);
  void createSceneInteractiveMarker();
  void renameCollisionObject(QListWidgetItem *item);
  void attachDetachCollisionObject(QListWidgetItem *item);
  void populateCollisionObjectsList();
  void computeImportFromText(const std::string &path);
  void computeExportAsText(const std::string &path);
    
  //Stored scenes tab
  void computeSaveSceneButtonClicked();
  void computeSaveQueryButtonClicked(const std::string &scene, const std::string &query_name);
  void computeLoadSceneButtonClicked();
  void computeLoadQueryButtonClicked();
  void populatePlanningSceneTreeView();
  void computeDeleteSceneButtonClicked();
  void computeDeleteQueryButtonClicked();
  void computeDeleteQueryButtonClickedHelper(QTreeWidgetItem *s);
  void checkPlanningSceneTreeEnabledButtons();


  //States tab
  void saveRobotStateButtonClicked(const robot_state::RobotState &state);
  void populateRobotStatesList();

  //General
  void changePlanningGroupHelper();
  void importResource(const std::string &path);
  

  /* Selects or unselects a item in a list by the item name */
  void setItemSelectionInList(const std::string &item_name, bool selection, QListWidget *list);
  
  ros::NodeHandle nh_;
  ros::Publisher planning_scene_publisher_;
  ros::Publisher planning_scene_world_publisher_;

  collision_detection::CollisionWorld::ObjectConstPtr scaled_object_;
  
  std::vector< std::pair<std::string, bool> > known_collision_objects_;
  long unsigned int known_collision_objects_version_;
  bool first_time_;
  
};

}

#endif

