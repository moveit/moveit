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

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematic_state/conversions.h>
#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>

#include <geometric_shapes/shape_operations.h>
#include <interactive_markers/tools.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>

#include <eigen_conversions/eigen_msg.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <QShortcut>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{

static const int ITEM_TYPE_SCENE = 1;
static const int ITEM_TYPE_QUERY = 2;

MotionPlanningFrame::MotionPlanningFrame(MotionPlanningDisplay *pdisplay, rviz::DisplayContext *context, QWidget *parent) :
  QWidget(parent),
  planning_display_(pdisplay),
  context_(context),
  ui_(new Ui::MotionPlanningUI())
{
  // set up the GUI
  ui_->setupUi(this);
  
  // connect bottons to actions; each action usually registers the function pointer for the actual computation,
  // to keep the GUI more responsive (using the background job processing)
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
  connect( ui_->import_file_button, SIGNAL( clicked() ), this, SLOT( importFileButtonClicked() ));
  connect( ui_->import_url_button, SIGNAL( clicked() ), this, SLOT( importUrlButtonClicked() ));
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
  connect( ui_->collision_objects_list, SIGNAL( itemChanged( QListWidgetItem * ) ), this, SLOT( collisionObjectChanged( QListWidgetItem * ) ));
  connect( ui_->path_constraints_combo_box, SIGNAL( currentIndexChanged ( int ) ), this, SLOT( pathConstraintsIndexChanged( int ) ));
  connect( ui_->planning_scene_tree, SIGNAL( itemChanged( QTreeWidgetItem *, int ) ), this, SLOT( warehouseItemNameChanged( QTreeWidgetItem *, int ) ));

  connect( ui_->tabWidget, SIGNAL( currentChanged ( int ) ), this, SLOT( tabChanged( int ) ));

  QShortcut *copy_object_shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_C), ui_->collision_objects_list);
  connect(copy_object_shortcut, SIGNAL( activated() ), this, SLOT( copySelectedCollisionObject() ) );

  //Goal poses
  connect( ui_->new_goal_pose_button, SIGNAL( clicked() ), this, SLOT( createGoalPoseButtonClicked() ));
  connect( ui_->remove_goal_pose_button, SIGNAL( clicked() ), this, SLOT( removeSelectedGoalsButtonClicked() ));
  connect( ui_->load_from_db_button, SIGNAL( clicked() ), this, SLOT( loadFromDBButtonClicked() ));
  connect( ui_->save_on_db_button, SIGNAL( clicked() ), this, SLOT( saveOnDBButtonClicked() ));
  connect( ui_->delete_on_db_button, SIGNAL( clicked() ), this, SLOT( deleteOnDBButtonClicked() ));
  connect( ui_->goal_poses_list, SIGNAL( itemSelectionChanged() ), this, SLOT( goalPoseSelectionChanged() ));
  connect( ui_->goal_poses_list, SIGNAL( itemDoubleClicked(QListWidgetItem *) ), this, SLOT( goalPoseDoubleClicked(QListWidgetItem *) ));
  
  QShortcut *copy_goals_shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_C), ui_->goal_poses_list);
  connect(copy_goals_shortcut, SIGNAL( activated() ), this, SLOT( copySelectedGoalPoses() ) );

  //Start states
  connect( ui_->save_start_state_button, SIGNAL( clicked() ), this, SLOT( saveStartStateButtonClicked() ));
  connect( ui_->remove_start_state_button, SIGNAL( clicked() ), this, SLOT( removeSelectedStatesButtonClicked() ));
  connect( ui_->start_states_list, SIGNAL( itemDoubleClicked(QListWidgetItem*) ), this, SLOT( startStateItemDoubleClicked(QListWidgetItem*) ));

  ui_->tabWidget->setCurrentIndex(0); 
  planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  planning_scene_world_publisher_ = nh_.advertise<moveit_msgs::PlanningSceneWorld>("planning_scene_world", 1);
}

MotionPlanningFrame::~MotionPlanningFrame(void)
{
}

void MotionPlanningFrame::createGoalPoseButtonClicked(void) 
{
  planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
  if ( ! ps || planning_display_->getRobotInteraction()->getActiveEndEffectors().empty() )
    return;

  bool ok = false;
  std::stringstream ss;
  ss << ps->getName().c_str() << "_pose_" << std::setfill('0') << std::setw(4) << goal_poses_.size();
  
  QString text = QInputDialog::getText(this, tr("Choose a name"),
                                       tr("Goal pose name:"), QLineEdit::Normal,
                                       QString(ss.str().c_str()), &ok);

  std::string name;
  if (ok)
  {
    if ( ! text.isEmpty() )
    {
      name = text.toStdString();
      if (goal_poses_.find(name) != goal_poses_.end())
        QMessageBox::warning(this, "Name already exists", QString("The name '").append(name.c_str()).
                             append("' already exists. Not creating goal."));
      else 
      {
        //Create the new goal pose at the current eef pose, and attach an interactive marker to it
        Eigen::Affine3d tip_pose = planning_display_->getQueryGoalState()->getLinkState(planning_display_->getRobotInteraction()->getActiveEndEffectors()[0].parent_link)->getGlobalLinkTransform();
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = planning_display_->getKinematicModel()->getModelFrame();
        static const float marker_scale = 0.35;
        int_marker.scale = marker_scale;
        tf::poseEigenToMsg(tip_pose, int_marker.pose);

        int_marker.name = name;

        robot_interaction::addArrowMarker(int_marker);

        rviz::InteractiveMarker* imarker = new rviz::InteractiveMarker(planning_display_->getSceneNode(), context_ );
        interactive_markers::autoComplete(int_marker);
        imarker->processMessage(int_marker);
        imarker->setShowAxes(false);			  
        imarker->setShowDescription(false);

        goal_poses_.insert(GoalPosePair(name,  GoalPoseMarker(boost::shared_ptr<rviz::InteractiveMarker>(imarker))));

        // Connect signals
        connect( imarker, SIGNAL( userFeedback(visualization_msgs::InteractiveMarkerFeedback &)), this, SLOT( goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback &) ));
      }
    }
    else
      QMessageBox::warning(this, "Goal not created", "Cannot use an empty name for a new goal pose.");
  }

  populateGoalPosesList();
}

void MotionPlanningFrame::removeSelectedGoalsButtonClicked(void)
{
  QList<QListWidgetItem*> found_items = ui_->goal_poses_list->selectedItems();
  for ( unsigned int i = 0 ; i < found_items.size() ; i++ )
  {
    goal_poses_.erase(found_items[i]->text().toStdString());
  }
  populateGoalPosesList();
}

void MotionPlanningFrame::loadFromDBButtonClicked(void) 
{
  
  //Get all the constraints from the database, convert to goal pose markers
  if (constraints_storage_ && robot_state_storage_) 
  {
    std::vector<std::string> names;
    try 
    {
      constraints_storage_->getKnownConstraints(ui_->load_goals_filter_text->text().toStdString(), names);
    }
    catch (...)
    {
      QMessageBox::warning(this, "Cannot query the database", "Wrongly formatted regular expression for goal poses.");
      return;
    }

    for (unsigned int i = 0 ; i < names.size() ; i++)
    {
      //Create a goal pose marker
      moveit_warehouse::ConstraintsWithMetadata c;
      constraints_storage_->getConstraints(c, names[i]);

      if ( c->position_constraints.size() > 0 && c->position_constraints[0].constraint_region.primitive_poses.size() > 0 && c->orientation_constraints.size() > 0 )
      {
        //Overwrite if exists. TODO: Ask the user before overwriting? copy the existing one with another name before?
        if ( goal_poses_.find(c->name) != goal_poses_.end() )
        {
          goal_poses_.erase(c->name);
        }
        geometry_msgs::Pose shape_pose;
        shape_pose.position = c->position_constraints[0].constraint_region.primitive_poses[0].position;
        shape_pose.orientation = c->orientation_constraints[0].orientation;

        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = planning_display_->getKinematicModel()->getModelFrame();
        static const float marker_scale = 0.35;
        int_marker.scale = marker_scale;
        int_marker.pose = shape_pose;
        int_marker.name = c->name;

        robot_interaction::addArrowMarker(int_marker);

        rviz::InteractiveMarker* imarker = new rviz::InteractiveMarker(planning_display_->getSceneNode(), context_ );
        interactive_markers::autoComplete(int_marker);
        imarker->processMessage(int_marker);
        imarker->setShowAxes(false);			  
        imarker->setShowDescription(false);

        goal_poses_.insert(GoalPosePair(c->name, GoalPoseMarker(boost::shared_ptr<rviz::InteractiveMarker>(imarker))));

        // Connect signals
        connect( imarker, SIGNAL( userFeedback(visualization_msgs::InteractiveMarkerFeedback &)), this, SLOT( goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback &) ));
      }
    }
    populateGoalPosesList();
    
    //Now get all start states from the database
    names.clear();
    try 
    {
      robot_state_storage_->getKnownRobotStates(ui_->load_states_filter_text->text().toStdString(), names);
    }
    catch (...)
    {
      QMessageBox::warning(this, "Cannot query the database", "Wrongly formatted regular expression for start states.");
      return;
    }

    for ( unsigned int i = 0 ; i < names.size() ; i++ )
    {
      moveit_warehouse::RobotStateWithMetadata rs;
      if ( robot_state_storage_->getRobotState(rs, names[i]) )
      {
        //Overwrite if exists.
        if (start_states_.find(names[i]) != start_states_.end())
        {
          start_states_.erase(names[i]);
        }
      }
             
      //Store the current start state
      start_states_.insert(StartStatePair(names[i],  StartState(*rs)));        
    }
    populateStartStatesList();
  } 
  else 
  {
    QMessageBox::warning(this, "Warning", "Not connected to a database.");
  }
}

void MotionPlanningFrame::saveOnDBButtonClicked(void)
{  
  if (constraints_storage_ && robot_state_storage_) 
  {
    //Convert all goal pose markers into constraints and store them
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); it++)
    {
      moveit_msgs::Constraints c;
      c.name = it->second.imarker->getName();

      shape_msgs::SolidPrimitive sp;
      sp.type = sp.BOX;
      sp.dimensions.resize(3, std::numeric_limits<float>::epsilon() * 10.0);

      moveit_msgs::PositionConstraint pc;
      pc.constraint_region.primitives.push_back(sp);
      geometry_msgs::Pose posemsg;
      posemsg.position.x = it->second.imarker->getPosition().x;
      posemsg.position.y = it->second.imarker->getPosition().y;
      posemsg.position.z = it->second.imarker->getPosition().z;
      posemsg.orientation.x = 0.0;
      posemsg.orientation.y = 0.0;
      posemsg.orientation.z = 0.0;
      posemsg.orientation.w = 1.0;
      pc.constraint_region.primitive_poses.push_back(posemsg);
      pc.weight = 1.0;
      c.position_constraints.push_back(pc);

      moveit_msgs::OrientationConstraint oc;
      oc.orientation.x = it->second.imarker->getOrientation().x;
      oc.orientation.y = it->second.imarker->getOrientation().y;
      oc.orientation.z = it->second.imarker->getOrientation().z;
      oc.orientation.w = it->second.imarker->getOrientation().w;
      oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = 
        oc.absolute_z_axis_tolerance = std::numeric_limits<float>::epsilon() * 10.0;
      oc.weight = 1.0;
      c.orientation_constraints.push_back(oc);
      
      constraints_storage_->addConstraints(c);
    }
    
    //Store all start states
    for (StartStateMap::iterator it = start_states_.begin(); it != start_states_.end(); it++)
    {
      robot_state_storage_->addRobotState(it->second.state_msg, it->first);
    }
  } 
  else
  {
    QMessageBox::warning(this, "Warning", "Not connected to a database.");
  }
  
}

void MotionPlanningFrame::deleteOnDBButtonClicked(void) 
{
  //Go through the list of goal poses, and delete those selected
  if (constraints_storage_ && robot_state_storage_) 
  {
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); it++) 
    {
      if (it->second.selected) 
      {
        constraints_storage_->removeConstraints(it->second.imarker->getName());        
      }
    }
    
    removeSelectedGoalsButtonClicked();
    
    for (StartStateMap::iterator it = start_states_.begin(); it != start_states_.end(); it++) 
    {
      if (it->second.selected) 
      {
        robot_state_storage_->removeRobotState(it->first);        
      }
    }
    
    removeSelectedStatesButtonClicked();
  }
  else
  {
    QMessageBox::warning(this, "Warning", "Not connected to a database.");
  }
}

void MotionPlanningFrame::populateGoalPosesList(void) 
{
  ui_->goal_poses_list->clear();
  for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); it++) 
  {
    QListWidgetItem *item = new QListWidgetItem(QString(it->first.c_str()));
    ui_->goal_poses_list->addItem(item);
    if (it->second.selected) 
    {
      //If selected, highlight in the list
      item->setSelected(true);
    }
  }
}

void MotionPlanningFrame::goalPoseSelectionChanged()
{
  for (unsigned int i = 0; i < ui_->goal_poses_list->count() ; ++i)
  {
    QListWidgetItem *item = ui_->goal_poses_list->item(i);
    std::string name = item->text().toStdString();
    if ( goal_poses_.find(name) != goal_poses_.end() &&
        ( (item->isSelected() && ! goal_poses_[name].selected )
            || ( ! item->isSelected() && goal_poses_[name].selected )))
      switchGoalPoseMarkerSelection(name);
  }
}

void MotionPlanningFrame::goalPoseDoubleClicked(QListWidgetItem * item)
{
  if ( planning_display_->getRobotInteraction()->getActiveEndEffectors().empty() || ! planning_display_->getQueryGoalState() )
    return;
  
  // Call to IK  
  geometry_msgs::Pose current_pose;
  const boost::shared_ptr<rviz::InteractiveMarker> &imarker = goal_poses_[item->text().toStdString()].imarker;
  current_pose.position.x = imarker->getPosition().x;
  current_pose.position.y = imarker->getPosition().y;
  current_pose.position.z = imarker->getPosition().z;
  current_pose.orientation.x = imarker->getOrientation().x;
  current_pose.orientation.y = imarker->getOrientation().y;
  current_pose.orientation.z = imarker->getOrientation().z;
  current_pose.orientation.w = imarker->getOrientation().w;

  static const float timeout = 1.0;
  static const unsigned int attempts = 1.0;

  bool feasible = planning_display_->getRobotInteraction()->updateState(*planning_display_->getQueryGoalState(),
                                                                        planning_display_->getRobotInteraction()->getActiveEndEffectors()[0], current_pose, timeout, attempts);
  if (feasible)
  {
    planning_display_->updateQueryGoalState();
  }
  else
  {
    QMessageBox::warning(this, "Goal not reachable" , "Could not find a solution to the inverse kinematics");
  }
}

/* Receives feedback from the interactive marker attached to a goal pose */
void MotionPlanningFrame::goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback)
{ 
  static Eigen::Affine3d initial_pose_eigen;
  static bool dragging = false;
  
  if (feedback.event_type == feedback.BUTTON_CLICK) 
  {
    //Unselect all but the clicked one
    for (unsigned int i = 0; i < ui_->goal_poses_list->count(); ++i)
    {
      QListWidgetItem *item = ui_->goal_poses_list->item(i);
      if (item->text().toStdString() == feedback.marker_name)
        item->setSelected(true);
      else
        item->setSelected(false);
    }
  } 
  else if (feedback.event_type == feedback.MOUSE_DOWN) 
  {
    //Store current poses
    goals_initial_pose_.clear();    
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); it++) 
    {
      Eigen::Affine3d pose(Eigen::Quaterniond(it->second.imarker->getOrientation().w, it->second.imarker->getOrientation().x, it->second.imarker->getOrientation().y, it->second.imarker->getOrientation().z));
      pose(0,3) = it->second.imarker->getPosition().x;
      pose(1,3) = it->second.imarker->getPosition().y;
      pose(2,3) = it->second.imarker->getPosition().z;
      goals_initial_pose_.insert(std::pair<std::string, Eigen::Affine3d>(it->second.imarker->getName(), pose));

      if (it->second.imarker->getName() == feedback.marker_name) 
      {
        initial_pose_eigen=pose;
      }
    }
    dragging=true;
  } 
  else if (feedback.event_type == feedback.POSE_UPDATE && dragging) 
  {
    //Compute displacement from stored pose, and apply to the rest of selected markers
    Eigen::Affine3d current_pose_eigen;
    tf::poseMsgToEigen(feedback.pose, current_pose_eigen);

    Eigen::Affine3d current_wrt_initial = initial_pose_eigen.inverse() * current_pose_eigen;

    //Update the rest of selected markers    
    for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end(); it++) 
    {
      if (it->second.imarker->getName() != feedback.marker_name && it->second.selected) 
      {
        visualization_msgs::InteractiveMarkerPose impose;

        Eigen::Affine3d newpose = initial_pose_eigen * current_wrt_initial * initial_pose_eigen.inverse() * goals_initial_pose_[it->second.imarker->getName()];
        tf::poseEigenToMsg(newpose, impose.pose);
        impose.header.frame_id = it->second.imarker->getReferenceFrame();

        it->second.imarker->processMessage(impose);
      }
    }
  } else if (feedback.event_type == feedback.MOUSE_UP) 
  {
    dragging=false;
  }
}

void MotionPlanningFrame::switchGoalPoseMarkerSelection(const std::string &marker_name) 
{
  geometry_msgs::PoseStamped current_pose;
  current_pose.pose.position.x=goal_poses_[marker_name].imarker->getPosition().x;
  current_pose.pose.position.y=goal_poses_[marker_name].imarker->getPosition().y;
  current_pose.pose.position.z=goal_poses_[marker_name].imarker->getPosition().z;
  current_pose.pose.orientation.x=goal_poses_[marker_name].imarker->getOrientation().x;
  current_pose.pose.orientation.y=goal_poses_[marker_name].imarker->getOrientation().y;
  current_pose.pose.orientation.z=goal_poses_[marker_name].imarker->getOrientation().z;
  current_pose.pose.orientation.w=goal_poses_[marker_name].imarker->getOrientation().w;

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.name = goal_poses_[marker_name].imarker->getName();
  if (goal_poses_[marker_name].selected) 
  {
    //If selected, unselect
    int_marker.pose = current_pose.pose;
    goal_poses_[marker_name].selected = false;
    setItemSelectionInList(marker_name, false, ui_->goal_poses_list);
  } 
  else 
  {
    //If unselected, select
    int_marker = robot_interaction::make6DOFMarker(goal_poses_[marker_name].imarker->getName(), current_pose, 1.0);
    goal_poses_[marker_name].selected = true;
    setItemSelectionInList(marker_name, true, ui_->goal_poses_list);
  }
  int_marker.header.frame_id = goal_poses_[marker_name].imarker->getReferenceFrame();
  static const float marker_scale = 0.35;
  int_marker.scale = marker_scale;
  robot_interaction::addArrowMarker(int_marker);
  interactive_markers::autoComplete(int_marker);
  goal_poses_[marker_name].imarker->processMessage(int_marker);
}

void MotionPlanningFrame::setItemSelectionInList(const std::string &item_name, bool selection, QListWidget *list) 
{
  QList<QListWidgetItem*> found_items = list->findItems(QString(item_name.c_str()), Qt::MatchExactly);
  for (unsigned int i = 0 ; i < found_items.size(); ++i)
    found_items[i]->setSelected(selection);
}

void MotionPlanningFrame::copySelectedGoalPoses(void)
{
  QList<QListWidgetItem *> sel = ui_->goal_poses_list->selectedItems();
  if (sel.empty())
    return;

  planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
  if (!ps)
    return;

  for (int i = 0 ; i < sel.size() ; ++i)
  {
    std::string name = sel[i]->text().toStdString();
    std::stringstream ss;
    ss << ps->getName().c_str() << "_pose_" << std::setfill('0') << std::setw(4) << goal_poses_.size();

    geometry_msgs::PoseStamped current_pose;
    current_pose.pose.position.x=goal_poses_[name].imarker->getPosition().x;
    current_pose.pose.position.y=goal_poses_[name].imarker->getPosition().y;
    current_pose.pose.position.z=goal_poses_[name].imarker->getPosition().z;
    current_pose.pose.orientation.x=goal_poses_[name].imarker->getOrientation().x;
    current_pose.pose.orientation.y=goal_poses_[name].imarker->getOrientation().y;
    current_pose.pose.orientation.z=goal_poses_[name].imarker->getOrientation().z;
    current_pose.pose.orientation.w=goal_poses_[name].imarker->getOrientation().w;

    visualization_msgs::InteractiveMarker int_marker;
    int_marker = robot_interaction::make6DOFMarker(ss.str(), current_pose, 1.0);

    int_marker.header.frame_id = ps->getKinematicModel()->getModelFrame();
    static const float marker_scale = 0.35;
    int_marker.scale = marker_scale;
    robot_interaction::addArrowMarker(int_marker);
    interactive_markers::autoComplete(int_marker);

    rviz::InteractiveMarker* imarker = new rviz::InteractiveMarker(planning_display_->getSceneNode(), context_ );
    imarker->processMessage(int_marker);
    imarker->setShowAxes(false);
    imarker->setShowDescription(false);

    goal_poses_.insert(GoalPosePair(ss.str(), GoalPoseMarker(boost::shared_ptr<rviz::InteractiveMarker>(imarker), true)));

    // Connect signals
    connect( imarker, SIGNAL( userFeedback(visualization_msgs::InteractiveMarkerFeedback &)), this, SLOT( goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback &) ));

    //Unselect the marker source of the copy
    switchGoalPoseMarkerSelection(name);
  }

  planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateGoalPosesList, this));
}

void MotionPlanningFrame::saveStartStateButtonClicked(void) 
{
  bool ok = false;                

  std::stringstream ss;
  ss << planning_display_->getPlanningScene()->getName().c_str() << "_state_" << std::setfill('0') << std::setw(4) << start_states_.size();

  QString text = QInputDialog::getText(this, tr("Choose a name"),
                                       tr("Start state name:"), QLineEdit::Normal,
                                       QString(ss.str().c_str()), &ok);

  std::string name;
  if (ok)
  {
    if (!text.isEmpty())
    {
      name = text.toStdString();
      if (start_states_.find(name) != start_states_.end())
        QMessageBox::warning(this, "Name already exists", QString("The name '").append(name.c_str()).
                             append("' already exists. Not creating state."));
      else 
      {
        //Store the current start state
        moveit_msgs::RobotState msg;
        kinematic_state::kinematicStateToRobotState(*planning_display_->getQueryStartState(), msg);
        start_states_.insert(StartStatePair(name,  StartState(msg)));        
      }
    }
    else
      QMessageBox::warning(this, "Start state not saved", "Cannot use an empty name for a new start state.");
  }
  populateStartStatesList();
}

void MotionPlanningFrame::removeSelectedStatesButtonClicked(void)
{
  QList<QListWidgetItem*> found_items = ui_->start_states_list->selectedItems();
  for (unsigned int i = 0; i < found_items.size(); i++)
  {
    start_states_.erase(found_items[i]->text().toStdString());    
  }
  populateStartStatesList();
}

void MotionPlanningFrame::populateStartStatesList(void) 
{
  ui_->start_states_list->clear();
  for (StartStateMap::iterator it = start_states_.begin(); it != start_states_.end(); it++) 
  {
    QListWidgetItem *item = new QListWidgetItem(QString(it->first.c_str()));
    ui_->start_states_list->addItem(item);
    if (it->second.selected) 
    {
      //If selected, highlight in the list
      item->setSelected(true);
    }
  }
}

void MotionPlanningFrame::startStateItemDoubleClicked(QListWidgetItem * item)
{       
  //If a start state item is double clicked, apply it to the start query
  kinematic_state::KinematicStatePtr ks(new kinematic_state::KinematicState(*planning_display_->getQueryStartState()));
  kinematic_state::robotStateToKinematicState(start_states_[item->text().toStdString()].state_msg, *ks);
  planning_display_->setQueryStartState(ks);
}

void MotionPlanningFrame::copySelectedCollisionObject(void)
{
  QList<QListWidgetItem *> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
    return;
  
  planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
  if (!ps)
    return;
  
  collision_detection::CollisionWorldPtr world = ps->getCollisionWorld();
  bool change = false;
  for (int i = 0 ; i < sel.size() ; ++i)
  {
    std::string name = sel[i]->text().toStdString();
    collision_detection::CollisionWorld::ObjectConstPtr obj = world->getObject(name);
    if (!obj)
      continue;

    // find a name for the copy
    name = "Copy of " + name;
    if (world->hasObject(name))
    {
      name += " ";
      unsigned int n = 1;
      while (world->hasObject(name + boost::lexical_cast<std::string>(n)))
        n++;
      name += boost::lexical_cast<std::string>(n);
    }
    world->addToObject(name, obj->shapes_, obj->shape_poses_);
    ROS_DEBUG("Copied collision object to '%s'", name.c_str());
    change = true;
  }

  if (change)
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateCollisionObjectsList, this));
}

void MotionPlanningFrame::changePlanningGroupHelper(void)
{ 
  if (!planning_display_->getPlanningSceneMonitor())
    return;
  
  const kinematic_model::KinematicModelConstPtr &kmodel = planning_display_->getKinematicModel();
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
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populatePlannersList, this, desc));
      planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::populateConstraintsList, this));
    }
  } 
}

void MotionPlanningFrame::changePlanningGroup(void)
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::changePlanningGroupHelper, this));
}

void MotionPlanningFrame::publishSceneButtonClicked(void)
{
  const planning_scene_monitor::LockedPlanningScene &ps = planning_display_->getPlanningScene();
  if (ps)
  {
    moveit_msgs::PlanningScene msg;
    ps->getPlanningSceneMsg(msg);
    planning_scene_publisher_.publish(msg);
  }
}

void MotionPlanningFrame::sceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  if (update_type & planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY)
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateCollisionObjectsList, this));
}

void MotionPlanningFrame::populateCollisionObjectsList(void)
{
  ui_->collision_objects_list->setUpdatesEnabled(false);
  bool oldState = ui_->collision_objects_list->blockSignals(true);  

  { 
    QList<QListWidgetItem *> sel = ui_->collision_objects_list->selectedItems();
    std::set<std::string> to_select;
    for (int i = 0 ; i < sel.size() ; ++i)
      to_select.insert(sel[i]->text().toStdString());
    ui_->collision_objects_list->clear();
    known_collision_objects_.clear();
    
    planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
    if (ps)
    {
      const collision_detection::CollisionWorldPtr &world = ps->getCollisionWorld();
      const std::vector<std::string> &collision_object_names = world->getObjectIds();
      for (std::size_t i = 0 ; i < collision_object_names.size() ; ++i)
      {
        QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(collision_object_names[i]),
                                                    ui_->collision_objects_list, (int)i);
        item->setFlags(item->flags() | Qt::ItemIsEditable);
        item->setToolTip(item->text());
        item->setCheckState(Qt::Unchecked);
        if (to_select.find(collision_object_names[i]) != to_select.end())
          item->setSelected(true);
        ui_->collision_objects_list->addItem(item);
        known_collision_objects_.push_back(std::make_pair(collision_object_names[i], false));
      }
      
      const kinematic_state::KinematicState &cs = ps->getCurrentState();
      std::vector<const kinematic_state::AttachedBody*> attached_bodies;
      cs.getAttachedBodies(attached_bodies);
      for (std::size_t i = 0 ; i < attached_bodies.size() ; ++i)
      {
        QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(attached_bodies[i]->getName()),
                                                    ui_->collision_objects_list, (int)(i + collision_object_names.size()));
        item->setFlags(item->flags() | Qt::ItemIsEditable);
        item->setToolTip(item->text());
        item->setCheckState(Qt::Checked);
        if (to_select.find(attached_bodies[i]->getName()) != to_select.end())
          item->setSelected(true);
        ui_->collision_objects_list->addItem(item);
        known_collision_objects_.push_back(std::make_pair(attached_bodies[i]->getName(), true));
      }
    }
  }
  
  ui_->collision_objects_list->blockSignals(oldState);
  ui_->collision_objects_list->setUpdatesEnabled(true);
  selectedCollisionObjectChanged();
}

/* Receives feedback from the interactive marker and updates the shape pose in the world accordingly */
void  MotionPlanningFrame::imProcessFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback)
{
  ui_->object_x->setValue(feedback.pose.position.x);
  ui_->object_y->setValue(feedback.pose.position.y);
  ui_->object_z->setValue(feedback.pose.position.z);

  Eigen::Quaterniond q;
  tf::quaternionMsgToEigen(feedback.pose.orientation, q);
  Eigen::Vector3d xyz = q.matrix().eulerAngles(0, 1, 2);
  
  ui_->object_rx->setValue(xyz[0]);
  ui_->object_ry->setValue(xyz[1]);
  ui_->object_rz->setValue(xyz[2]);
}

void MotionPlanningFrame::createSceneInteractiveMarker(void)
{
  QList<QListWidgetItem *> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
    return;

  const planning_scene_monitor::LockedPlanningScene &ps = planning_display_->getPlanningScene();
  if (!ps)
    return;
  
  const collision_detection::CollisionWorldConstPtr &world = ps->getCollisionWorld();
  const collision_detection::CollisionWorld::ObjectConstPtr &obj = world->getObject(sel[0]->text().toStdString());
  if (!scene_marker_ && obj && obj->shapes_.size() == 1)
  {
    Eigen::Quaterniond eq(obj->shape_poses_[0].rotation());
    geometry_msgs::PoseStamped shape_pose;
    shape_pose.pose.position.x = obj->shape_poses_[0].translation()[0];
    shape_pose.pose.position.y = obj->shape_poses_[0].translation()[1];
    shape_pose.pose.position.z = obj->shape_poses_[0].translation()[2];
    shape_pose.pose.orientation.x = eq.x();
    shape_pose.pose.orientation.y = eq.y();
    shape_pose.pose.orientation.z = eq.z();
    shape_pose.pose.orientation.w = eq.w();

    // create an interactive marker for moving the shape in the world
    visualization_msgs::InteractiveMarker int_marker = robot_interaction::make6DOFMarker(std::string("marker_") + sel[0]->text().toStdString(), shape_pose, 1.0);
    int_marker.header.frame_id = context_->getFrameManager()->getFixedFrame();
    int_marker.description = sel[0]->text().toStdString();

    rviz::InteractiveMarker* imarker = new rviz::InteractiveMarker(planning_display_->getSceneNode(), context_ );
    interactive_markers::autoComplete(int_marker);
    imarker->processMessage(int_marker);
    imarker->setShowAxes(false);
    scene_marker_.reset(imarker);

    // Connect signals
    connect( imarker, SIGNAL( userFeedback(visualization_msgs::InteractiveMarkerFeedback &)), this,
             SLOT( imProcessFeedback(visualization_msgs::InteractiveMarkerFeedback &) ));
  }
}

void MotionPlanningFrame::importUrlButtonClicked(void)
{
  bool ok = false;
  QString url = QInputDialog::getText(this, tr("Import Scene"),
                                      tr("URL for file to import:"), QLineEdit::Normal,
                                      QString("http://"), &ok);
  if (ok && !url.isEmpty())
    importResource(url.toStdString());
}

void MotionPlanningFrame::importFileButtonClicked(void)
{ 
  QString path = QFileDialog::getOpenFileName(this, "Import Scene");
  if (!path.isEmpty())
    importResource("file://" + path.toStdString());
}

void MotionPlanningFrame::importResource(const std::string &path)
{
  if (planning_display_->getPlanningSceneMonitor())
  {
    shapes::Mesh *mesh = shapes::createMeshFromResource(path);
    if (mesh)
    {
      std::size_t slash = path.find_last_of("/\\");
      std::string name = path.substr(slash + 1);
      shapes::ShapeConstPtr shape(mesh);
      Eigen::Affine3d pose;
      pose.setIdentity();
      
      if (planning_display_->getPlanningScene()->getCurrentState().hasAttachedBody(name))
      {
        QMessageBox::warning(this, QString("Duplicate names"),
                             QString("An attached object named '").append(name.c_str()).append("' already exists. Please rename the attached object before importing."));
        return;
      }
      
      //If the object already exists, ask the user whether to overwrite or rename
      if (planning_display_->getPlanningScene()->getCollisionWorld()->hasObject(name))
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
            {
              planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
              if (ps)
              {
                ps->getCollisionWorld()->removeObject(name);
                addObject(ps->getCollisionWorld(), name, shape, pose);
              }
            }
            break;
          case QMessageBox::No:
          {
            // Don't overwrite was clicked. Ask for another name
            bool ok = false;
            QString text = QInputDialog::getText(this, tr("Choose a new name"),
                                                 tr("Import the new object under the name:"), QLineEdit::Normal,
                                                 QString::fromStdString(name + "-" + boost::lexical_cast<std::string>
                                                                        (planning_display_->getPlanningScene()->getCollisionWorld()->getObjectsCount())), &ok);
            if (ok)
            {
              if (!text.isEmpty())
              {
                name = text.toStdString();
                planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
                if (ps)
                {
                  if (ps->getCollisionWorld()->hasObject(name))
                    QMessageBox::warning(this, "Name already exists", QString("The name '").append(name.c_str()).
                                         append("' already exists. Not importing object."));
                  else
                    addObject(ps->getCollisionWorld(), name, shape, pose);
                }
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
        planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
        if (ps)
          addObject(ps->getCollisionWorld(), name, shape, pose);  
      }
    }
    else
    {
      QMessageBox::warning(this, QString("Import error"), QString("Unable to import scene"));
      return;
    }
  }
}

void MotionPlanningFrame::addObject(const collision_detection::CollisionWorldPtr &world, const std::string &id,
                                    const shapes::ShapeConstPtr &shape, const Eigen::Affine3d &pose)
{
  world->addToObject(id, shape, pose);
  
  planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateCollisionObjectsList, this)); 

  // Automatically select the inserted object so that its IM is displayed
  planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::setItemSelectionInList, this, id, true, ui_->collision_objects_list));
  
  planning_display_->queueRenderSceneGeometry();
}

void MotionPlanningFrame::removeObjectButtonClicked(void)
{
  QList<QListWidgetItem *> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
    return;
  planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
  if (ps)
  {    
    collision_detection::CollisionWorldPtr world = ps->getCollisionWorld();
    for (int i = 0 ; i < sel.count() ; ++i)
      if (sel[i]->checkState() == Qt::Unchecked)
        world->removeObject(sel[i]->text().toStdString());
      else
        ps->getCurrentState().clearAttachedBody(sel[i]->text().toStdString());
    scene_marker_.reset();
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateCollisionObjectsList, this));
    planning_display_->queueRenderSceneGeometry(); 
  }
} 

void MotionPlanningFrame::warehouseItemNameChanged(QTreeWidgetItem *item, int column)
{
  if (item->text(column) == item->toolTip(column) || item->toolTip(column).length() == 0)
    return;
  boost::shared_ptr<moveit_warehouse::PlanningSceneStorage> planning_scene_storage = planning_scene_storage_;
  if (!planning_scene_storage)
    return;

  if (item->type() == ITEM_TYPE_SCENE)
  {
    std::string new_name = item->text(column).toStdString();

    if (planning_scene_storage->hasPlanningScene(new_name))
    {
      planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populatePlanningSceneTreeView, this));
      QMessageBox::warning(this, "Scene not renamed", QString("The scene name '").append(item->text(column)).append("' already exists"));
      return;
    }
    else
    {
      std::string old_name = item->toolTip(column).toStdString();
      planning_scene_storage->renamePlanningScene(old_name, new_name);
      item->setToolTip(column, item->text(column));
    }
  }
  else
  {     
    std::string scene = item->parent()->text(0).toStdString();
    std::string new_name = item->text(column).toStdString();
    if (planning_scene_storage->hasPlanningQuery(scene, new_name))
    {
      planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populatePlanningSceneTreeView, this));
      QMessageBox::warning(this, "Query not renamed", QString("The query name '").append(item->text(column)).
                           append("' already exists for scene ").append(item->parent()->text(0)));
      return;
    }
    else
    {
      std::string old_name = item->toolTip(column).toStdString();
      planning_scene_storage->renamePlanningQuery(scene, old_name, new_name);
      item->setToolTip(column, item->text(column));
    }
  }
}

void MotionPlanningFrame::renameCollisionObject(QListWidgetItem *item)
{
  if (item->text().isEmpty())
  {
    QMessageBox::warning(this, "Invalid object name", "Cannot set an empty object name.");
    item->setText(QString::fromStdString(known_collision_objects_[item->type()].first));
    return;
  }
  
  if (planning_display_->getPlanningScene()->getCollisionWorld()->hasObject(item->text().toStdString()) ||
      planning_display_->getPlanningScene()->getCurrentState().hasAttachedBody(item->text().toStdString()))
  { 
    QMessageBox::warning(this, "Duplicate object name", QString("The name '").append(item->text()).
                         append("' already exists. Not renaming object ").append((known_collision_objects_[item->type()].first.c_str())));
    item->setText(QString::fromStdString(known_collision_objects_[item->type()].first));
    return;
  }

  if (item->checkState() == Qt::Unchecked)
  {
    planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
    const collision_detection::CollisionWorldPtr &world = ps->getCollisionWorld();
    collision_detection::CollisionWorld::ObjectConstPtr obj = world->getObject(known_collision_objects_[item->type()].first);
    if (obj)
    {
      known_collision_objects_[item->type()].first = item->text().toStdString();
      world->removeObject(obj->id_);
      world->addToObject(known_collision_objects_[item->type()].first, obj->shapes_, obj->shape_poses_);
      if (scene_marker_)
      {
        scene_marker_.reset();
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::createSceneInteractiveMarker, this));
      }
    }
  }
  else
  {
    // rename attached body
    planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
    kinematic_state::KinematicState &cs = ps->getCurrentState();
    const kinematic_state::AttachedBody *ab = cs.getAttachedBody(known_collision_objects_[item->type()].first);
    if (ab)
    {
      known_collision_objects_[item->type()].first = item->text().toStdString();
      std::vector<std::string> touch_links(ab->getTouchLinks().begin(), ab->getTouchLinks().end());
      kinematic_state::AttachedBody *new_ab = new kinematic_state::AttachedBody(cs.getLinkState(ab->getAttachedLinkName()),
                                                                                known_collision_objects_[item->type()].first,
                                                                                ab->getShapes(), ab->getFixedTransforms(),
                                                                                touch_links);
      cs.clearAttachedBody(ab->getName());
      cs.attachBody(new_ab);
    }
  }
}

void MotionPlanningFrame::attachDetachCollisionObject(QListWidgetItem *item)
{
  bool checked = item->checkState() == Qt::Checked;
  moveit_msgs::AttachedCollisionObject aco;

  if (checked) // we need to attach a known collision object
  {
    QStringList links;
    const std::vector<std::string> &links_std = planning_display_->getKinematicModel()->getLinkModelNames();
    for (std::size_t i = 0 ; i < links_std.size() ; ++i)
      links.append(QString::fromStdString(links_std[i]));
    bool ok = false;
    QString response = QInputDialog::getItem(this, tr("Select Link Name"), tr("Choose the link to attach to:"),
                                             links, 0, false, &ok);
    if (!ok)
    {
      item->setCheckState(Qt::Unchecked);
      return;
    }
    aco.link_name = response.toStdString();
    aco.object.id = known_collision_objects_[item->type()].first;
    aco.object.operation = moveit_msgs::CollisionObject::ADD;
  }
  else // we need to detach an attached object
  { 
    const planning_scene_monitor::LockedPlanningScene &ps = planning_display_->getPlanningScene();
    const kinematic_state::AttachedBody *attached_body = ps->getCurrentState().getAttachedBody(known_collision_objects_[item->type()].first);
    if (attached_body)
    {
      aco.link_name = attached_body->getAttachedLinkName();
      aco.object.id = attached_body->getName();
      aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
    }
  }
  {
    planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
    known_collision_objects_[item->type()].second = checked;
    ps->processAttachedCollisionObjectMsg(aco);
  }
  
  selectedCollisionObjectChanged();
  planning_display_->queueRenderSceneGeometry(); 
}

void MotionPlanningFrame::collisionObjectChanged(QListWidgetItem *item)
{
  if (item->type() < (int)known_collision_objects_.size() && 
      planning_display_->getPlanningSceneMonitor())
  {
    // if we have a name change
    if (known_collision_objects_[item->type()].first != item->text().toStdString())
      renameCollisionObject(item);
    else
    {
      bool checked = item->checkState() == Qt::Checked;
      if (known_collision_objects_[item->type()].second != checked)
        attachDetachCollisionObject(item);
    }
  }
}

static QString decideStatusText(const collision_detection::CollisionWorld::ObjectConstPtr &obj) 
{
  QString status_text = "'" + QString::fromStdString(obj->id_) + "' is a collision object with ";
  if (obj->shapes_.empty())
    status_text += "no geometry";
  else
  {
    std::vector<QString> shape_names;
    for (std::size_t i = 0 ; i < obj->shapes_.size() ; ++i)
      switch (obj->shapes_[i]->type)
      {
        case shapes::SPHERE:
          shape_names.push_back("sphere");
          break;     
        case shapes::CYLINDER:
          shape_names.push_back("cylinder");
          break;
        case shapes::CONE:
          shape_names.push_back("cone");
          break;
        case shapes::BOX:
          shape_names.push_back("box");
          break;
        case shapes::PLANE:
          shape_names.push_back("plane");
          break;
        case shapes::MESH:
          shape_names.push_back("mesh");
          break;
        case shapes::OCTREE:
          shape_names.push_back("octree");
          break;
        default: 
          shape_names.push_back("unknown");
          break;
      }
    if (shape_names.size() == 1)
      status_text += "one " + shape_names[0];
    else
    {
      status_text += QString::fromStdString(boost::lexical_cast<std::string>(shape_names.size())) + " shapes:";
      for (std::size_t i = 0 ; i < shape_names.size() ; ++i)
        status_text += " " + shape_names[i];
    }
  }
  return status_text;
}

static QString decideStatusText(const kinematic_state::AttachedBody *attached_body) 
{
  QString status_text = "'" + QString::fromStdString(attached_body->getName()) + "' is attached to '" +
      QString::fromStdString(attached_body->getAttachedLinkName()) + "'";
  return status_text;
}

void MotionPlanningFrame::selectedCollisionObjectChanged(void)
{
  QList<QListWidgetItem *> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
  {    
    bool oldState = ui_->object_x->blockSignals(true);
    ui_->object_x->setValue(0.0);
    ui_->object_x->blockSignals(oldState);

    oldState = ui_->object_y->blockSignals(true);
    ui_->object_y->setValue(0.0);
    ui_->object_y->blockSignals(oldState);

    oldState = ui_->object_z->blockSignals(true);
    ui_->object_z->setValue(0.0);
    ui_->object_z->blockSignals(oldState);

    oldState = ui_->object_rx->blockSignals(true);
    ui_->object_rx->setValue(0.0);
    ui_->object_rx->blockSignals(oldState);

    oldState = ui_->object_ry->blockSignals(true);
    ui_->object_ry->setValue(0.0);
    ui_->object_ry->blockSignals(oldState);

    oldState = ui_->object_rz->blockSignals(true);
    ui_->object_rz->setValue(0.0);
    ui_->object_rz->blockSignals(oldState);
    
    ui_->object_status->setText("");
    scene_marker_.reset();
    ui_->scene_scale->setEnabled(false);
  }
  else
    if (planning_display_->getPlanningSceneMonitor())
    {
      // if this is a CollisionWorld element
      if (sel[0]->checkState() == Qt::Unchecked)
      {
        ui_->scene_scale->setEnabled(true);
        bool update_scene_marker = false;
        Eigen::Affine3d obj_pose;
        {  
          const planning_scene_monitor::LockedPlanningScene &ps = planning_display_->getPlanningScene();
          const collision_detection::CollisionWorldConstPtr &world = ps->getCollisionWorld();
          const collision_detection::CollisionWorld::ObjectConstPtr &obj = world->getObject(sel[0]->text().toStdString()); 
          if (obj)
          {
            ui_->object_status->setText(decideStatusText(obj));
            
            if (obj->shapes_.size() == 1)
            {
              obj_pose = obj->shape_poses_[0];
              Eigen::Vector3d xyz = obj_pose.rotation().eulerAngles(0, 1, 2);
              update_scene_marker = true; // do the marker update to avoid deadlock
              
              bool oldState = ui_->object_x->blockSignals(true);
              ui_->object_x->setValue(obj_pose.translation()[0]);
              ui_->object_x->blockSignals(oldState);

              oldState = ui_->object_y->blockSignals(true);
              ui_->object_y->setValue(obj_pose.translation()[1]);
              ui_->object_y->blockSignals(oldState);

              oldState = ui_->object_z->blockSignals(true);
              ui_->object_z->setValue(obj_pose.translation()[2]);
              ui_->object_z->blockSignals(oldState);
              
              oldState = ui_->object_rx->blockSignals(true);
              ui_->object_rx->setValue(xyz[0]);
              ui_->object_rx->blockSignals(oldState);

              oldState = ui_->object_ry->blockSignals(true);
              ui_->object_ry->setValue(xyz[1]);
              ui_->object_ry->blockSignals(oldState);
              
              oldState = ui_->object_rz->blockSignals(true);
              ui_->object_rz->setValue(xyz[2]);
              ui_->object_rz->blockSignals(oldState);
            }
          }
          else
            ui_->object_status->setText("ERROR: '" + sel[0]->text() + "' should be a collision object but it is not");
        }
        if (update_scene_marker)
        {              
          if (!scene_marker_)
            createSceneInteractiveMarker();
          if (scene_marker_)
          {
            Eigen::Quaterniond eq(obj_pose.rotation());
            // Update the IM pose to match the current selected object
            scene_marker_->setPose(Ogre::Vector3(obj_pose.translation()[0], obj_pose.translation()[1], obj_pose.translation()[2]),
                                   Ogre::Quaternion(eq.w(), eq.x(), eq.y(), eq.z()), "");
          }
        }
      }
      else
      {        
        ui_->scene_scale->setEnabled(false);
        // if it is an attached object
        scene_marker_.reset();
        const planning_scene_monitor::LockedPlanningScene &ps = planning_display_->getPlanningScene();
        const kinematic_state::AttachedBody *attached_body = ps->getCurrentState().getAttachedBody(sel[0]->text().toStdString());
        if (attached_body)
          ui_->object_status->setText(decideStatusText(attached_body));
        else
          ui_->object_status->setText("ERROR: '" + sel[0]->text() + "' should be an attached object but it is not");        
      }
    }
}

void MotionPlanningFrame::clearSceneButtonClicked(void)
{    
  planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
  if (ps)
  {
    ps->getCollisionWorld()->clearObjects();
    ps->getCurrentState().clearAttachedBodies();
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateCollisionObjectsList, this));
    planning_display_->queueRenderSceneGeometry();
  }
}

void MotionPlanningFrame::objectPoseValueChanged(double value)
{
  QList<QListWidgetItem *> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
    return;
  planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
  if (ps)
  {
    const collision_detection::CollisionWorldPtr &world = ps->getCollisionWorld();
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
      if (scene_marker_)
      {
        Eigen::Quaterniond eq(p.rotation());
        scene_marker_->setPose(Ogre::Vector3(ui_->object_x->value(), ui_->object_y->value(), ui_->object_z->value()),
                               Ogre::Quaternion(eq.w(), eq.x(), eq.y(), eq.z()), "");
      }
    }
  }
}

void MotionPlanningFrame::sceneScaleStartChange(void)
{
  QList<QListWidgetItem *> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
    return;
  if (planning_display_->getPlanningSceneMonitor() && sel[0]->checkState() == Qt::Unchecked)
  {
    planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
    if (ps)
    {
      const collision_detection::CollisionWorldPtr &world = ps->getCollisionWorld();
      scaled_object_ = world->getObject(sel[0]->text().toStdString());
    }
  }
}

void MotionPlanningFrame::sceneScaleEndChange(void)
{
  scaled_object_.reset();
  ui_->scene_scale->setSliderPosition(100);
}

void MotionPlanningFrame::sceneScaleChanged(int value)
{
  if (scaled_object_)
  {
    planning_scene_monitor::LockedPlanningScene ps = planning_display_->getPlanningScene();
    if (ps)
    {
      collision_detection::CollisionWorldPtr world = ps->getCollisionWorld();
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
    else
      scaled_object_.reset();
  }
}

void MotionPlanningFrame::populateConstraintsList(void)
{
  if (move_group_)
  {
    // add some artificial wait time (but in the background) for the constraints DB to connect
    double dt = (ros::WallTime::now() - move_group_construction_time_).toSec();
    if (dt < 0.2)
      ros::WallDuration(0.1).sleep();
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populateConstraintsList, this, move_group_->getKnownConstraints()));
  }
}

void MotionPlanningFrame::populateConstraintsList(const std::vector<std::string> &constr)
{
  ui_->path_constraints_combo_box->clear();     
  ui_->path_constraints_combo_box->addItem("None");
  for (std::size_t i = 0 ; i < constr.size() ; ++i)
    ui_->path_constraints_combo_box->addItem(QString::fromStdString(constr[i]));
}

void MotionPlanningFrame::populatePlannersList(const moveit_msgs::PlannerInterfaceDescription &desc)
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

void MotionPlanningFrame::enable(void)
{
  ui_->planning_algorithm_combo_box->clear();  
  ui_->library_label->setText("NO PLANNING LIBRARY LOADED");
  ui_->library_label->setStyleSheet("QLabel { color : red; font: bold }");
  ui_->object_status->setText("");

  changePlanningGroup();

  // activate the frame
  show();
}

void MotionPlanningFrame::disable(void)
{
  move_group_.reset();
  hide();
}

void MotionPlanningFrame::allowLookingToggled(bool checked)
{
  if (move_group_)
    move_group_->allowLooking(checked);
}

void MotionPlanningFrame::allowReplanningToggled(bool checked)
{
  if (move_group_)
    move_group_->allowReplanning(checked);
}

void MotionPlanningFrame::pathConstraintsIndexChanged(int index)
{
  if (move_group_)
  {
    if (index > 0)
      move_group_->setPathConstraints(ui_->path_constraints_combo_box->itemText(index).toStdString());
    else
      move_group_->clearPathConstraints();
  }
}

void MotionPlanningFrame::tabChanged(int index)
{
  if (scene_marker_ && index != 3)
    scene_marker_.reset();
  else
    if (index == 3)
      selectedCollisionObjectChanged();
}

void MotionPlanningFrame::planningAlgorithmIndexChanged(int index)
{
  if (move_group_)
  {
    if (index > 0)
      move_group_->setPlannerId(ui_->planning_algorithm_combo_box->itemText(index).toStdString());
    else
      move_group_->setPlannerId("");
  }
}

void MotionPlanningFrame::constructPlanningRequest(moveit_msgs::MotionPlanRequest &mreq)
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

void MotionPlanningFrame::planButtonClicked(void)
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computePlanButtonClicked, this));
}

void MotionPlanningFrame::executeButtonClicked(void)
{
  ui_->execute_button->setEnabled(false);
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeExecuteButtonClicked, this));
}

void MotionPlanningFrame::planAndExecuteButtonClicked(void)
{
  ui_->plan_and_execute_button->setEnabled(false);
  ui_->execute_button->setEnabled(false);
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computePlanAndExecuteButtonClicked, this));
}

void MotionPlanningFrame::randomStatesButtonClicked(void)
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeRandomStatesButtonClicked, this));
}

void MotionPlanningFrame::setStartToCurrentButtonClicked(void)
{ 
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeSetStartToCurrentButtonClicked, this));
}

void MotionPlanningFrame::setGoalToCurrentButtonClicked(void)
{
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeSetGoalToCurrentButtonClicked, this));
}

void MotionPlanningFrame::databaseConnectButtonClicked(void)
{   
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeDatabaseConnectButtonClicked, this));
}

void MotionPlanningFrame::saveSceneButtonClicked(void)
{ 
  if (planning_scene_storage_)
  {
    const std::string &name = planning_display_->getPlanningScene()->getName();
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
            planning_display_->getPlanningScene()->setName(new_name.toStdString());
            planning_display_->subProp("Planning Scene")->subProp("Scene Name")->setValue(new_name);
            saveSceneButtonClicked();
          }
          return;
        }
        return;
      }
    }

    planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeSaveSceneButtonClicked, this));
  }
}

void MotionPlanningFrame::loadSceneButtonClicked(void)
{   
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeLoadSceneButtonClicked, this));
}

void MotionPlanningFrame::loadQueryButtonClicked(void)
{   
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeLoadQueryButtonClicked, this));
}

void MotionPlanningFrame::saveQueryButtonClicked(void)
{   
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem *> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem *s = sel.front();

      // if we have selected a PlanningScene, add the query as a new one, under that planning scene
      if (s->type() == ITEM_TYPE_SCENE)
      {
        std::string scene = s->text(0).toStdString();
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeSaveQueryButtonClicked, this, scene, ""));
      }
      else
      {
        // if we selected a query name, then we overwrite that query
        std::string scene = s->parent()->text(0).toStdString();
        std::string query_name = s->text(0).toStdString();

        while (query_name.empty() || planning_scene_storage_->hasPlanningQuery(scene, query_name))
        {
          boost::scoped_ptr<QMessageBox> q;
          if (query_name.empty())
            q.reset(new QMessageBox(QMessageBox::Question, "Change Planning Query Name",
                                    QString("The name for the planning query should not be empty. Would you like to rename the planning query?'"),
                                    QMessageBox::Cancel,
                                    this));
          else
            q.reset(new QMessageBox(QMessageBox::Question, "Confirm Planning Query Overwrite",
                                    QString("A planning query named '")
                                    .append( query_name.c_str() )
                                    .append( "' already exists. Do you wish to overwrite that query?"),
                                    QMessageBox::Yes | QMessageBox::No,
                                    this));
          boost::scoped_ptr<QPushButton> rename(q->addButton("&Rename", QMessageBox::AcceptRole)); 
          if (q->exec() == QMessageBox::Yes)
            break;
          else
          {
            if (q->clickedButton() == rename.get())
            {
              bool ok = false;
              QString new_name = QInputDialog::getText(this, "Rename Planning Query", "New name for the planning query:", QLineEdit::Normal, QString::fromStdString(query_name), &ok);
              if (ok)
                query_name = new_name.toStdString();
              else
                return;
            }
            else
              return;
          }
        }
        planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeSaveQueryButtonClicked, this, scene, query_name));
      }
    }
  }
}

void MotionPlanningFrame::deleteSceneButtonClicked(void)
{   
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeDeleteSceneButtonClicked, this));
}

void MotionPlanningFrame::deleteQueryButtonClicked(void)
{   
  planning_display_->addBackgroundJob(boost::bind(&MotionPlanningFrame::computeDeleteQueryButtonClicked, this));
}

void MotionPlanningFrame::checkPlanningSceneTreeEnabledButtons(void)
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
    if (s->type() == ITEM_TYPE_SCENE)
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

void MotionPlanningFrame::planningSceneItemClicked(void)
{
  checkPlanningSceneTreeEnabledButtons();
}

void MotionPlanningFrame::configureForPlanning(void)
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

void MotionPlanningFrame::updateSceneMarkers(float wall_dt, float ros_dt)
{
  if (scene_marker_)
    scene_marker_->update(wall_dt);
}

void MotionPlanningFrame::updateGoalPoseMarkers(float wall_dt, float ros_dt)
{
  for (GoalPoseMap::iterator it = goal_poses_.begin(); it != goal_poses_.end() ; ++it)
    it->second.imarker->update(wall_dt);
}

void MotionPlanningFrame::computePlanButtonClicked(void)
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

void MotionPlanningFrame::computeExecuteButtonClicked(void)
{
  if (move_group_ && current_plan_)
    move_group_->execute(*current_plan_);
}

void MotionPlanningFrame::computePlanAndExecuteButtonClicked(void)
{    
  if (!move_group_)
    return;
  configureForPlanning();
  move_group_->move();
  ui_->plan_and_execute_button->setEnabled(true);
}

void MotionPlanningFrame::computeSetStartToCurrentButtonClicked(void)
{  
  const planning_scene_monitor::LockedPlanningScene &ps = planning_display_->getPlanningScene();
  if (ps)
  {
    const kinematic_state::KinematicState &s = ps->getCurrentState();
    planning_display_->setQueryStartState(kinematic_state::KinematicStatePtr(new kinematic_state::KinematicState(s)));
  }
}

void MotionPlanningFrame::computeSetGoalToCurrentButtonClicked(void)
{ 
  const planning_scene_monitor::LockedPlanningScene &ps = planning_display_->getPlanningScene();
  if (ps)
  {
    const kinematic_state::KinematicState &s = ps->getCurrentState();
    planning_display_->setQueryGoalState(kinematic_state::KinematicStatePtr(new kinematic_state::KinematicState(s)));
  }
}

void MotionPlanningFrame::computeRandomStatesButtonClicked(void)
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

void MotionPlanningFrame::populatePlanningSceneTreeView(void)
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
    QTreeWidgetItem *item = new QTreeWidgetItem(ui_->planning_scene_tree, QStringList(QString::fromStdString(names[i])), ITEM_TYPE_SCENE);
    item->setFlags(item->flags() | Qt::ItemIsEditable);
    item->setToolTip(0, item->text(0)); // we use the tool tip as a backup of the old name when renaming
    for (std::size_t j = 0 ; j < query_names.size() ; ++j)
    {
      QTreeWidgetItem *subitem = new QTreeWidgetItem(item, QStringList(QString::fromStdString(query_names[j])), ITEM_TYPE_QUERY);
      subitem->setFlags(subitem->flags() | Qt::ItemIsEditable);
      subitem->setToolTip(0, subitem->text(0));
      item->addChild(subitem);
    }

    ui_->planning_scene_tree->insertTopLevelItem(ui_->planning_scene_tree->topLevelItemCount(), item);
    if (expanded.find(names[i]) != expanded.end())
      ui_->planning_scene_tree->expandItem(item);
  }
  ui_->planning_scene_tree->sortItems(0, Qt::AscendingOrder);
  ui_->planning_scene_tree->setUpdatesEnabled(true); 
  checkPlanningSceneTreeEnabledButtons();
}

void MotionPlanningFrame::computeDatabaseConnectButtonClickedHelper(int mode)
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

void MotionPlanningFrame::computeDatabaseConnectButtonClicked(void)
{
  if (planning_scene_storage_)
  {
    planning_scene_storage_.reset();
    robot_state_storage_.reset();
    constraints_storage_.reset();
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::computeDatabaseConnectButtonClickedHelper, this, 1));
  }
  else
  {      
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::computeDatabaseConnectButtonClickedHelper, this, 2));
    try
    {
      planning_scene_storage_.reset(new moveit_warehouse::PlanningSceneStorage(ui_->database_host->text().toStdString(),
                                                                               ui_->database_port->value(), 5.0));
      robot_state_storage_.reset(new moveit_warehouse::RobotStateStorage(ui_->database_host->text().toStdString(),
                                                                         ui_->database_port->value(), 5.0));
      constraints_storage_.reset(new moveit_warehouse::ConstraintsStorage(ui_->database_host->text().toStdString(),
                                                                          ui_->database_port->value(), 5.0));
    }
    catch(std::runtime_error &ex)
    { 
      planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::computeDatabaseConnectButtonClickedHelper, this, 3));
      ROS_ERROR("%s", ex.what());  
      return;
    }     
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::computeDatabaseConnectButtonClickedHelper, this, 4));
  }
}

void MotionPlanningFrame::computeSaveSceneButtonClicked(void)
{
  if (planning_scene_storage_)
  {
    moveit_msgs::PlanningScene msg;
    planning_display_->getPlanningScene()->getPlanningSceneMsg(msg);
    planning_scene_storage_->removePlanningScene(msg.name);
    planning_scene_storage_->addPlanningScene(msg);
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populatePlanningSceneTreeView, this));
  }
}

void MotionPlanningFrame::computeLoadSceneButtonClicked(void)
{
  if (planning_scene_storage_)
  { 
    QList<QTreeWidgetItem *> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem *s = sel.front();
      if (s->type() == ITEM_TYPE_SCENE)
      {
        std::string scene = s->text(0).toStdString();
        ROS_DEBUG("Attempting to load scene '%s'", scene.c_str());
        moveit_warehouse::PlanningSceneWithMetadata scene_m;
        if (planning_scene_storage_->getPlanningScene(scene_m, scene))
        {
          ROS_INFO("Loaded scene '%s'", scene.c_str());
          if (planning_display_->getPlanningSceneMonitor())
          {
            if (scene_m->robot_model_name != planning_display_->getKinematicModel()->getName())
            {
              ROS_INFO("Scene '%s' was saved for robot '%s' but we are using robot '%s'. Using scene geometry only",
                       scene.c_str(), scene_m->robot_model_name.c_str(),
                       planning_display_->getKinematicModel()->getName().c_str());
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

void MotionPlanningFrame::computeLoadQueryButtonClicked(void)
{
  if (planning_scene_storage_)
  { 
    QList<QTreeWidgetItem *> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem *s = sel.front();
      if (s->type() == ITEM_TYPE_QUERY)
      {
        std::string scene = s->parent()->text(0).toStdString();
        std::string query_name = s->text(0).toStdString();
        moveit_warehouse::MotionPlanRequestWithMetadata mp;
        if (planning_scene_storage_->getPlanningQuery(mp, scene, query_name))
        {
          kinematic_state::KinematicStatePtr start_state(new kinematic_state::KinematicState(*planning_display_->getQueryStartState()));
          kinematic_state::robotStateToKinematicState(*planning_display_->getPlanningScene()->getTransforms(), mp->start_state, *start_state);
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

void MotionPlanningFrame::computeSaveQueryButtonClicked(const std::string &scene, const std::string &query_name)
{
  moveit_msgs::MotionPlanRequest mreq;
  constructPlanningRequest(mreq);
  if (planning_scene_storage_)
  {
    if (!query_name.empty())
      planning_scene_storage_->removePlanningQuery(scene, query_name);
    planning_scene_storage_->addPlanningQuery(mreq, scene, query_name);
    planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populatePlanningSceneTreeView, this));
  }
}

void MotionPlanningFrame::computeDeleteSceneButtonClicked(void)
{
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem *> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem *s = sel.front();
      if (s->type() == ITEM_TYPE_SCENE)
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
      planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::populatePlanningSceneTreeView, this));
    }
  }
}

void MotionPlanningFrame::computeDeleteQueryButtonClickedHelper(QTreeWidgetItem *s)
{     
  ui_->planning_scene_tree->setUpdatesEnabled(false);
  s->parent()->removeChild(s);  
  ui_->planning_scene_tree->setUpdatesEnabled(true);
}

void MotionPlanningFrame::computeDeleteQueryButtonClicked(void)
{
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem *> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem *s = sel.front();
      if (s->type() == ITEM_TYPE_QUERY)
      {
        std::string scene = s->parent()->text(0).toStdString();
        std::string query_name = s->text(0).toStdString();
        planning_scene_storage_->removePlanningQuery(scene, query_name); 
        planning_display_->addMainLoopJob(boost::bind(&MotionPlanningFrame::computeDeleteQueryButtonClickedHelper, this, s));
      }
    }
  }
}

} // namespace
