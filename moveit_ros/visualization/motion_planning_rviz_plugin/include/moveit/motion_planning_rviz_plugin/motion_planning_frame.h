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
  ~MotionPlanningFrame(void);

  void changePlanningGroup(void);
  void enable(void);
  void disable(void);
  void sceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);

protected:
  static const int ITEM_TYPE_SCENE = 1;
  static const int ITEM_TYPE_QUERY = 2;

  void constructPlanningRequest(moveit_msgs::MotionPlanRequest &mreq);
  
  void updateSceneMarkers(float wall_dt, float ros_dt);
  void updateGoalPoseMarkers(float wall_dt, float ros_dt);

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
 
  class GoalPoseMarker
  {
  public:
    visualization_msgs::InteractiveMarker imarker_msg;
    boost::shared_ptr<rviz::InteractiveMarker> imarker;
    bool selected;
    enum
      {
        NOT_TESTED, PROCESSING, REACHABLE, NOT_REACHABLE, IN_COLLISION
      } reachable;
    
    GoalPoseMarker(void): selected(false) {}
    GoalPoseMarker(const boost::shared_ptr<rviz::InteractiveMarker> &marker, const visualization_msgs::InteractiveMarker &msg):
      imarker(marker), imarker_msg(msg), selected(false), reachable(GoalPoseMarker::NOT_TESTED) {}
    GoalPoseMarker(const boost::shared_ptr<rviz::InteractiveMarker> &marker, const visualization_msgs::InteractiveMarker &msg, bool is_selected):
      imarker(marker), imarker_msg(msg), selected(is_selected), reachable(GoalPoseMarker::NOT_TESTED) {}
    
    void updateMarker(void)
    {
      imarker->processMessage(imarker_msg);
    }

    void hide(void);
    void show(MotionPlanningDisplay *pdisplay, rviz::DisplayContext *context);
    void getPosition(geometry_msgs::Point &position);
    void getOrientation(geometry_msgs::Quaternion &orientation);

    bool isVisible()
    {
      return (imarker);
    }

  private:
    Ogre::Vector3 position_;
    Ogre::Quaternion orientation_;
  };
  
  typedef std::map<std::string, GoalPoseMarker> GoalPoseMap;
  typedef std::pair<std::string, GoalPoseMarker> GoalPosePair;
  GoalPoseMap goal_poses_;
          
  class StartState
  {
  public:
    moveit_msgs::RobotState state_msg;
    bool selected;
    
    StartState(): selected(false) {}
    StartState(const moveit_msgs::RobotState &state): state_msg(state), selected(false) {}
    StartState(const moveit_msgs::RobotState &state, bool is_selected): state_msg(state), selected(is_selected) {}
  };
  
  typedef std::map<std::string, StartState> StartStateMap;
  typedef std::pair<std::string, StartState> StartStatePair;
  StartStateMap start_states_;
                             
private Q_SLOTS:

  //Context tab
  void databaseConnectButtonClicked(void);
  void publishSceneButtonClicked(void);
  void planningAlgorithmIndexChanged(int index);
  void resetDbButtonClicked(void);

  //Planning tab
  void planButtonClicked(void);  
  void executeButtonClicked(void);
  void planAndExecuteButtonClicked(void);
  void randomStatesButtonClicked(void);
  void setStartToCurrentButtonClicked(void);
  void setGoalToCurrentButtonClicked(void);
  void allowReplanningToggled(bool checked);
  void allowLookingToggled(bool checked);
  void pathConstraintsIndexChanged(int index);

  //Scene Objects tab
  void importFileButtonClicked(void);
  void importUrlButtonClicked(void);
  void clearSceneButtonClicked(void);
  void sceneScaleChanged(int value);
  void sceneScaleStartChange(void);
  void sceneScaleEndChange(void);
  void removeObjectButtonClicked(void);
  void selectedCollisionObjectChanged(void);
  void objectPoseValueChanged(double value);
  void collisionObjectChanged(QListWidgetItem *item);
  void imProcessFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);
  void copySelectedCollisionObject(void);
  
  //Stored scenes tab
  void saveSceneButtonClicked(void);
  void planningSceneItemClicked(void);
  void saveQueryButtonClicked(void);
  void deleteSceneButtonClicked(void);
  void deleteQueryButtonClicked(void);
  void loadSceneButtonClicked(void);
  void loadQueryButtonClicked(void);
  void warehouseItemNameChanged(QTreeWidgetItem *item, int column);

  //Stored queries tab
  void goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);
  void createGoalPoseButtonClicked(void);
  void removeSelectedGoalsButtonClicked(void);
  void removeAllGoalsButtonClicked(void);
  void goalPoseSelectionChanged(void);
  void switchGoalVisibilityButtonClicked(void);
  void goalPoseDoubleClicked(QListWidgetItem *item);
  void copySelectedGoalPoses(void);
  void visibleAxisChanged(int state);
  void checkGoalsInCollision(void);
  void checkGoalsReachable(void);

  void saveStartStateButtonClicked(void);
  void removeSelectedStatesButtonClicked(void);
  void removeAllStatesButtonClicked(void);
  void startStateItemDoubleClicked(QListWidgetItem * item);

  void loadGoalsFromDBButtonClicked(void);
  void saveGoalsOnDBButtonClicked(void);
  void deleteGoalsOnDBButtonClicked(void);
  void loadStatesFromDBButtonClicked(void);
  void saveStatesOnDBButtonClicked(void);
  void deleteStatesOnDBButtonClicked(void);

  //General
  void tabChanged(int index);
  
private:

  //Context tab
  void computeDatabaseConnectButtonClicked(void);
  void computeDatabaseConnectButtonClickedHelper(int mode);
  void computeResetDbButtonClicked(const std::string &db);
  void populatePlannersList(const moveit_msgs::PlannerInterfaceDescription &desc);

  //Planning tab
  void computePlanButtonClicked(void);  
  void computeExecuteButtonClicked(void);
  void computePlanAndExecuteButtonClicked(void); 
  void computePlanAndExecuteButtonClickedDisplayHelper(void);
  void computeSetStartToCurrentButtonClicked(void);
  void computeSetGoalToCurrentButtonClicked(void);
  void computeRandomStatesButtonClicked(void);
  void populateConstraintsList(void);
  void populateConstraintsList(const std::vector<std::string> &constr);
  void configureForPlanning(void);

  //Scene objects tab
  void computeSaveSceneButtonClicked(void);
  void computeSaveQueryButtonClicked(const std::string &scene, const std::string &query_name);
  void computeDeleteSceneButtonClicked(void);
  void computeDeleteQueryButtonClicked(void);
  void computeDeleteQueryButtonClickedHelper(QTreeWidgetItem *s);
  void checkPlanningSceneTreeEnabledButtons(void);
  void computeLoadSceneButtonClicked(void);
  void computeLoadQueryButtonClicked(void);
  void addObject(const collision_detection::CollisionWorldPtr &world, const std::string &id,
                 const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose);
  void createSceneInteractiveMarker(void);
  void renameCollisionObject(QListWidgetItem *item);
  void attachDetachCollisionObject(QListWidgetItem *item);
  void populateCollisionObjectsList(void);

  //Stored scenes tab
  void populatePlanningSceneTreeView(void);

  //Stored queries tab
  void populateGoalPosesList();
  void populateStartStatesList();
  void computeGoalPoseDoubleClicked(QListWidgetItem * item);
  void switchGoalPoseMarkerSelection(const std::string &marker_name);
  typedef std::pair<visualization_msgs::InteractiveMarker, boost::shared_ptr<rviz::InteractiveMarker> > MsgMarkerPair;
  MsgMarkerPair make6DOFEndEffectorMarker(const std::string& name,
                                             const robot_interaction::RobotInteraction::EndEffector &eef,
                                             const geometry_msgs::Pose &pose,
                                             double scale,
                                             bool selected = false);
  void updateMarkerColorFromName(const std::string & name, float r, float g, float b, float a);
  void checkIfGoalInCollision(const std::string & goal_name);
  void checkIfGoalInCollision(const kinematic_state::KinematicStatePtr &work_state, const std::string & goal_name);
  void checkIfGoalReachable(const kinematic_state::KinematicStatePtr &work_state, const std::string &goal_name);

  //General
  void changePlanningGroupHelper(void);
  void importResource(const std::string &path);

  /** Selects or unselects a item in a list by the item name */
  void setItemSelectionInList(const std::string &item_name, bool selection, QListWidget *list);
  void selectItemJob(QListWidgetItem *item, bool flag);

  
  ros::NodeHandle nh_;
  ros::Publisher planning_scene_publisher_;
  ros::Publisher planning_scene_world_publisher_;

  collision_detection::CollisionWorld::ObjectConstPtr scaled_object_;
  
  std::vector< std::pair<std::string, bool> > known_collision_objects_;
  long unsigned int known_collision_objects_version_;
  
  EigenSTL::map_string_Affine3d goals_initial_pose_;
};

}

#endif

