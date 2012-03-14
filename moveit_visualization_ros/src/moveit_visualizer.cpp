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

#include <QApplication>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMenuBar>

#include <moveit_visualization_ros/moveit_visualizer.h>
#include <moveit_visualization_ros/primitive_object_addition_dialog.h>
#include <moveit_visualization_ros/mesh_object_addition_dialog.h>

static const std::string VIS_TOPIC_NAME = "planning_components_visualization";

namespace moveit_visualization_ros {

MoveItVisualizer::MoveItVisualizer() : first_update_(false) {

  ros::NodeHandle nh;
  ros::NodeHandle loc_nh("~");

  bool monitor_robot_state = false;
  loc_nh.param("monitor_robot_state", monitor_robot_state, false);

  interactive_marker_server_.reset(new interactive_markers::InteractiveMarkerServer("interactive_kinematics_visualization", "", false));

  boost::shared_ptr<tf::TransformListener> transformer;
  if(!monitor_robot_state) {
    ROS_INFO_STREAM("Starting publisher thread");
    joint_state_publisher_.reset(new KinematicStateJointStatePublisher());
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    boost::thread publisher_thread(boost::bind(&MoveItVisualizer::publisherFunction, this));
  } else {
    transformer.reset(new tf::TransformListener());
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description", transformer.get()));
    planning_scene_monitor_->startStateMonitor();
  }

  bool allow_trajectory_execution = false;
  if(monitor_robot_state) {
    loc_nh.param("allow_trajectory_execution", allow_trajectory_execution, false);
    if(allow_trajectory_execution) {
      trajectory_execution_monitor_.reset(new trajectory_execution_ros::TrajectoryExecutionMonitorRos(planning_scene_monitor_->getPlanningScene()->getKinematicModel()));
    }
  }

  vis_marker_publisher_ = nh.advertise<visualization_msgs::Marker> (VIS_TOPIC_NAME, 128);
  vis_marker_array_publisher_ = nh.advertise<visualization_msgs::MarkerArray> (VIS_TOPIC_NAME + "_array", 128);

  std_msgs::ColorRGBA col;
  col.r = col.g = col.b = .5;
  col.a = 1.0;

  std_msgs::ColorRGBA good_color;
  good_color.a = 1.0;    
  good_color.g = 1.0;    
  
  std_msgs::ColorRGBA bad_color;
  bad_color.a = 1.0;    
  bad_color.r = 1.0;    

  kinematics_plugin_loader_.reset(new kinematics_plugin_loader::KinematicsPluginLoader());

  pv_.reset(new PlanningVisualizationQtWrapper(planning_scene_monitor_->getPlanningScene(),
                                               planning_scene_monitor_->getGroupJointLimitsMap(),
                                               interactive_marker_server_,
                                               kinematics_plugin_loader_,
                                               vis_marker_array_publisher_));
  
  iov_.reset(new InteractiveObjectVisualizationQtWrapper(planning_scene_monitor_->getPlanningScene(),
                                                         interactive_marker_server_,
                                                         col));
  
  iov_->setUpdateCallback(boost::bind(&MoveItVisualizer::updatePlanningScene, this, _1));

  if(monitor_robot_state) {
    pv_->addMenuEntry("Reset start state", boost::bind(&MoveItVisualizer::updateToCurrentState, this));
    if(allow_trajectory_execution) {
      pv_->setAllStartChainModes(true);
      pv_->addMenuEntry("Execute last trajectory", boost::bind(&MoveItVisualizer::executeLastTrajectory, this));
    }
  }
  pv_->hideAllGroups();

  //must go before
  Ogre::LogManager* log_manager = new Ogre::LogManager();
  log_manager->createLog( "Ogre.log", false, false, false );

  rviz_frame_ = new rviz::VisualizationPanel;
  
  QList<int> sizes;
  sizes.push_back(0);
  sizes.push_back(1000);

  rviz_frame_->setSizes(sizes);

  //kind of hacky way to do this - this just turns on interactive move
  //given the way that the vis manager is creating tools
  rviz_frame_->getManager()->setCurrentTool(rviz_frame_->getManager()->getTool(1));

  //std::string config_name = ros::package::getPath("pr2_arm_navigation")+"/config/pr2_planning_components_config.config";
  std::string display_config_name = ros::package::getPath("pr2_arm_navigation")+"/config/pr2_planning_components_display_config.config";
  
  //frame->loadGeneralConfig(config_name);
  rviz_frame_->loadDisplayConfig(display_config_name);

  main_window_ = new QWidget;
  main_window_->resize(1500,1000);
  //InteractiveObjectVisualizationWidget* iov_widget = new InteractiveObjectVisualizationWidget(main_window_);

  PrimitiveObjectAdditionDialog* primitive_object_dialog = new PrimitiveObjectAdditionDialog(main_window_);
  MeshObjectAdditionDialog* mesh_object_dialog = new MeshObjectAdditionDialog(main_window_);

  QHBoxLayout* main_layout = new QHBoxLayout;
  QMenuBar* menu_bar = new QMenuBar(main_window_);
  PlanningSceneFileMenu* planning_scene_file_menu = new PlanningSceneFileMenu(menu_bar);
  QObject::connect(iov_.get(),
                   SIGNAL(updatePlanningSceneSignal(planning_scene::PlanningSceneConstPtr)),
                   planning_scene_file_menu,
                   SLOT(updatePlanningSceneSignalled(planning_scene::PlanningSceneConstPtr)));
  QObject::connect(planning_scene_file_menu->getDatabaseDialog(),
                   SIGNAL(planningSceneLoaded(moveit_msgs::PlanningScenePtr)),
                   iov_.get(),
                   SLOT(loadPlanningSceneSignalled(moveit_msgs::PlanningScenePtr)));
  menu_bar->addMenu(planning_scene_file_menu);

  planning_group_selection_menu_ = new PlanningGroupSelectionMenu(menu_bar);
  QObject::connect(planning_group_selection_menu_, 
                   SIGNAL(groupSelected(const QString&)),
                   pv_.get(),
                   SLOT(newGroupSelected(const QString&)));
  planning_group_selection_menu_->init(planning_scene_monitor_->getPlanningScene()->getSrdfModel());
  menu_bar->addMenu(planning_group_selection_menu_);

  QMenu* coll_object_menu = menu_bar->addMenu("Collision Objects");

  QAction* show_primitive_objects_dialog = coll_object_menu->addAction("Add Primitive Collision Object");
  QObject::connect(show_primitive_objects_dialog, SIGNAL(triggered()), primitive_object_dialog, SLOT(show()));
  QAction* show_mesh_objects_dialog = coll_object_menu->addAction("Add Mesh Collision Object");
  QObject::connect(show_mesh_objects_dialog, SIGNAL(triggered()), mesh_object_dialog, SLOT(show()));

  main_layout->setMenuBar(menu_bar);
  
  //main_layout->addWidget(iov_widget);
  main_layout->addWidget(rviz_frame_);

  main_window_->setLayout(main_layout);

  //QObject::connect(iov_widget, SIGNAL(addCubeRequested()), iov_.get(), SLOT(addCubeSignalled()));
  QObject::connect(primitive_object_dialog, 
                   SIGNAL(addCollisionObjectRequested(const moveit_msgs::CollisionObject&, const QColor&)), 
                   iov_.get(), 
                   SLOT(addCollisionObjectSignalled(const moveit_msgs::CollisionObject&, const QColor&)));

  QObject::connect(mesh_object_dialog, 
                   SIGNAL(addCollisionObjectRequested(const moveit_msgs::CollisionObject&, const QColor&)), 
                   iov_.get(), 
                   SLOT(addCollisionObjectSignalled(const moveit_msgs::CollisionObject&, const QColor&)));


  main_window_->show();

  planning_scene_monitor_->setUpdateCallback(boost::bind(&MoveItVisualizer::updateSceneCallback, this));
}

MoveItVisualizer::~MoveItVisualizer() {
  iov_.reset();
  pv_.reset();
  if(trajectory_execution_monitor_) {
    trajectory_execution_monitor_->restoreOriginalControllers();
    trajectory_execution_monitor_.reset();
  }
  planning_scene_monitor_.reset();
  delete rviz_frame_;
}

void MoveItVisualizer::publisherFunction() { 
  ros::WallRate r(10.0);

  while(ros::ok())
  {
    joint_state_publisher_->broadcastRootTransform(planning_scene_monitor_->getPlanningScene()->getCurrentState());
    joint_state_publisher_->publishKinematicState(planning_scene_monitor_->getPlanningScene()->getCurrentState());
    r.sleep();
  }
}

void MoveItVisualizer::updatePlanningScene(planning_scene::PlanningSceneConstPtr planning_scene) {
  current_diff_ = planning_scene;
  pv_->updatePlanningScene(planning_scene);
}

void MoveItVisualizer::updateSceneCallback() {
  if(first_update_) {
    ROS_INFO_STREAM("Got update scene callback");
    pv_->resetAllStartAndGoalStates();
    first_update_ = false;
  }
}

bool MoveItVisualizer::doneWithExecution() {
  ROS_INFO_STREAM("Done");
  return true;
} 
void MoveItVisualizer::executeLastTrajectory() {
  std::string group_name;
  trajectory_msgs::JointTrajectory traj;
  if(pv_->getLastTrajectory(group_name, traj)) {
    trajectory_execution::TrajectoryExecutionRequest ter;
    ter.group_name_ = group_name;
    
    ter.trajectory_ = traj;
    ROS_DEBUG_STREAM("Attempting to execute trajectory for group name " << group_name); 

    std::vector<trajectory_execution::TrajectoryExecutionRequest> ter_reqs;
    ter_reqs.push_back(ter);

    trajectory_execution_monitor_->executeTrajectories(ter_reqs,
                                                       boost::bind(&MoveItVisualizer::doneWithExecution, this));
  }
}

void MoveItVisualizer::updateToCurrentState() {
  iov_->updateCurrentState(planning_scene_monitor_->getPlanningScene()->getCurrentState());
  pv_->resetAllStartStates();
}

}
