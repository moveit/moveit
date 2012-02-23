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

#include "rviz/visualization_panel.h"
#include "rviz/visualization_manager.h"

#include <ros/ros.h>
#include <moveit_visualization_ros/interactive_object_visualization_qt_wrapper.h>
#include <moveit_visualization_ros/interactive_object_visualization_widget.h>
#include <moveit_visualization_ros/primitive_object_addition_dialog.h>
#include <moveit_visualization_ros/planning_group_selection_menu.h>
#include <moveit_visualization_ros/planning_scene_file_menu.h>
#include <moveit_visualization_ros/planning_visualization_qt_wrapper.h>
#include <moveit_visualization_ros/kinematic_state_joint_state_publisher.h>
#include <OGRE/OgreLogManager.h>
#include <trajectory_execution_ros/trajectory_execution_monitor_ros.h>

using namespace moveit_visualization_ros;

static const std::string VIS_TOPIC_NAME = "planning_components_visualization";

boost::shared_ptr<KinematicStateJointStatePublisher> joint_state_publisher_;
boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
boost::shared_ptr<PlanningVisualizationQtWrapper> pv_;
boost::shared_ptr<InteractiveObjectVisualizationQtWrapper> iov_;
boost::shared_ptr<trajectory_execution::TrajectoryExecutionMonitor> trajectory_execution_monitor_;

bool first_update = true;

void sigHandler(int x) {
  ROS_INFO_STREAM("Getting sig handler");
  iov_.reset();
  pv_.reset();
  trajectory_execution_monitor_->restoreOriginalControllers();
  planning_scene_monitor_.reset();
  exit(0);
}

void publisher_function() {
  ros::WallRate r(10.0);

  while(ros::ok())
  {
    joint_state_publisher_->broadcastRootTransform(planning_scene_monitor_->getPlanningScene()->getCurrentState());
    joint_state_publisher_->publishKinematicState(planning_scene_monitor_->getPlanningScene()->getCurrentState());
    r.sleep();
  }
}

void updateCallback(planning_scene::PlanningSceneConstPtr planning_scene) {
  pv_->updatePlanningScene(planning_scene);
}

void addCubeCallback() {
  iov_->addCube();
}

void addCylinderCallback() {
  iov_->addCylinder();
}

void addSphereCallback() {
  iov_->addSphere();
}

void updateSceneCallback() {
  if(first_update) {
    ROS_INFO_STREAM("Got update scene callback");
    pv_->resetAllStartAndGoalStates();
    first_update = false;
  }
}

bool doneWithExecution() {
  ROS_INFO_STREAM("Done");
  return true;
} 
void executeLastTrajectory() {
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
                                                       boost::bind(&doneWithExecution));
  }
}

void updateToCurrentState() {
  iov_->updateCurrentState(planning_scene_monitor_->getPlanningScene()->getCurrentState());
  pv_->resetAllStartStates();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_components_visualizer_full", ros::init_options::NoSigintHandler);

  // CAN'T SPIN AS RVIZ ALREADY IS!!!!
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  signal(SIGINT, sigHandler);
  signal(SIGTERM, sigHandler);

  ros::NodeHandle nh;
  ros::NodeHandle loc_nh("~");

  bool monitor_robot_state = false;
  loc_nh.param("monitor_robot_state", monitor_robot_state, false);

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  interactive_marker_server_.reset(new interactive_markers::InteractiveMarkerServer("interactive_kinematics_visualization", "", false));

  boost::shared_ptr<tf::TransformListener> transformer;
  if(!monitor_robot_state) {
    ROS_INFO_STREAM("Starting publisher thread");
    joint_state_publisher_.reset(new KinematicStateJointStatePublisher());
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    boost::thread publisher_thread(boost::bind(&publisher_function));
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

  ros::Publisher vis_marker_array_publisher;
  ros::Publisher vis_marker_publisher;

  vis_marker_publisher = nh.advertise<visualization_msgs::Marker> (VIS_TOPIC_NAME, 128);
  vis_marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray> (VIS_TOPIC_NAME + "_array", 128);

  std_msgs::ColorRGBA col;
  col.r = col.g = col.b = .5;
  col.a = 1.0;

  std_msgs::ColorRGBA good_color;
  good_color.a = 1.0;    
  good_color.g = 1.0;    
  
  std_msgs::ColorRGBA bad_color;
  bad_color.a = 1.0;    
  bad_color.r = 1.0;    

  boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader> 
    kinematics_plugin_loader(new kinematics_plugin_loader::KinematicsPluginLoader());

  pv_.reset(new PlanningVisualizationQtWrapper(planning_scene_monitor_->getPlanningScene(),
                                               planning_scene_monitor_->getGroupJointLimitsMap(),
                                               interactive_marker_server_,
                                               kinematics_plugin_loader,
                                               vis_marker_array_publisher));
  
  iov_.reset(new InteractiveObjectVisualizationQtWrapper(planning_scene_monitor_->getPlanningScene(),
                                                         interactive_marker_server_,
                                                         col));
  
  iov_->setUpdateCallback(boost::bind(&updateCallback, _1));

  // pv_->addMenuEntry("Add Cube", boost::bind(&addCubeCallback));
  // pv_->addMenuEntry("Add Cylinder", boost::bind(&addCylinderCallback));
  // pv_->addMenuEntry("Add Sphere", boost::bind(&addSphereCallback));
  if(monitor_robot_state) {
    pv_->addMenuEntry("Reset start state", boost::bind(&updateToCurrentState));
    if(allow_trajectory_execution) {
      pv_->setAllStartChainModes(true);
      pv_->addMenuEntry("Execute last trajectory", boost::bind(&executeLastTrajectory));
    }
  }
  pv_->hideAllGroups();

  QApplication app( argc, argv );
  
  //must go before
  Ogre::LogManager* log_manager = new Ogre::LogManager();
  log_manager->createLog( "Ogre.log", false, false, false );

  rviz::VisualizationPanel* frame = new rviz::VisualizationPanel;
  
  QList<int> sizes;
  sizes.push_back(0);
  sizes.push_back(1000);

  frame->setSizes(sizes);

  //kind of hacky way to do this - this just turns on interactive move
  //given the way that the vis manager is creating tools
  frame->getManager()->setCurrentTool(frame->getManager()->getTool(1));

  //std::string config_name = ros::package::getPath("pr2_arm_navigation")+"/config/pr2_planning_components_config.config";
  std::string display_config_name = ros::package::getPath("pr2_arm_navigation")+"/config/pr2_planning_components_display_config.config";

  //frame->loadGeneralConfig(config_name);
  frame->loadDisplayConfig(display_config_name);

  QWidget* main_window = new QWidget;
  main_window->resize(1500,1000);
  InteractiveObjectVisualizationWidget* iov_widget = new InteractiveObjectVisualizationWidget(main_window);

  PrimitiveObjectAdditionDialog* primitive_object_dialog = new PrimitiveObjectAdditionDialog(main_window);

  QHBoxLayout* main_layout = new QHBoxLayout;
  QMenuBar* menu_bar = new QMenuBar(main_window);
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

  PlanningGroupSelectionMenu* planning_group_selection_menu = new PlanningGroupSelectionMenu(menu_bar);
  QObject::connect(planning_group_selection_menu, 
                   SIGNAL(groupSelected(const QString&)),
                   pv_.get(),
                   SLOT(newGroupSelected(const QString&)));
  planning_group_selection_menu->init(planning_scene_monitor_->getPlanningScene()->getSrdfModel());
  menu_bar->addMenu(planning_group_selection_menu);

  QMenu* coll_object_menu = menu_bar->addMenu("Collision Objects");

  QAction* show_primitive_objects_dialog = coll_object_menu->addAction("Add Primitive Collision Object");
  QObject::connect(show_primitive_objects_dialog, SIGNAL(triggered()), primitive_object_dialog, SLOT(show()));
  main_layout->setMenuBar(menu_bar);
  
  main_layout->addWidget(iov_widget);
  main_layout->addWidget(frame);

  main_window->setLayout(main_layout);

  QObject::connect(iov_widget, SIGNAL(addCubeRequested()), iov_.get(), SLOT(addCubeSignalled()));
  QObject::connect(primitive_object_dialog, 
                   SIGNAL(addCollisionObjectRequested(const moveit_msgs::CollisionObject&, const QColor&)), 
                   iov_.get(), 
                   SLOT(addCollisionObjectSignalled(const moveit_msgs::CollisionObject&, const QColor&)));

  main_window->show();

  planning_scene_monitor_->setUpdateCallback(boost::bind(&updateSceneCallback));

  app.exec();

  delete main_window;
}
