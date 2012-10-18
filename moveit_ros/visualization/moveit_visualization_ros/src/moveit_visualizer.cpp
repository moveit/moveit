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
#include <collision_distance_field_ros/hybrid_collision_robot_ros.h>
#include <collision_distance_field/hybrid_collision_world.h>
// Rviz
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/tool_manager.h>
#include <rviz/default_plugin/view_controllers/orbit_view_controller.h>
#include <rviz/default_plugin/marker_display.h>
#include <rviz/default_plugin/interactive_marker_display.h>


static const std::string VIS_TOPIC_NAME = "planning_components_visualization";

namespace moveit_visualization_ros {

MoveItVisualizer::MoveItVisualizer() :
    first_update_(true),
    allow_trajectory_execution_(false),
    execution_succeeded_(false),
    stop_cycle_requested_(false)
{
    ros::NodeHandle nh;
    ros::NodeHandle loc_nh("~");

    bool monitor_robot_state = false;
    loc_nh.param("monitor_robot_state", monitor_robot_state, false);

    interactive_marker_server_.reset(new interactive_markers::InteractiveMarkerServer("interactive_kinematics_visualization", "", false));
    kinematic_model_loader_.reset(new planning_models_loader::KinematicModelLoader("robot_description"));
    tf_broadcaster_.reset(new tf::TransformBroadcaster);

    if(!monitor_robot_state) {
        ROS_INFO_STREAM("Starting publisher thread");
        joint_state_publisher_.reset(new KinematicStateJointStatePublisher());
        planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(kinematic_model_loader_));
        boost::thread publisher_thread(boost::bind(&MoveItVisualizer::publisherFunction, this, true));
    } else {
        transformer_.reset(new tf::TransformListener());
        planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(kinematic_model_loader_, transformer_));
        joint_state_publisher_.reset(new KinematicStateJointStatePublisher());
        bool publish_root_transform = false;
        loc_nh.param("publish_root_transform", publish_root_transform, false);
        if(publish_root_transform) {
            boost::thread publisher_thread(boost::bind(&MoveItVisualizer::publisherFunction, this, false));
        }
        planning_scene_monitor_->startStateMonitor();
    }

    if(monitor_robot_state) {
        loc_nh.param("allow_trajectory_execution", allow_trajectory_execution_, false);
        if(allow_trajectory_execution_) {
            bool manage_controllers= false;
            loc_nh.param("manage_controllers", manage_controllers, true);
            trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(planning_scene_monitor_->getPlanningScene()->getKinematicModel(), manage_controllers));
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

    boost::shared_ptr<planning_pipeline::PlanningPipeline> move_group_pipeline(new planning_pipeline::PlanningPipeline(planning_scene_monitor_->getPlanningScene()->getKinematicModel()));
    move_group_pipeline->displayComputedMotionPlans(false);
    move_group_pipeline->publishReceivedRequests(false);

    pv_.reset(new PlanningVisualizationQtWrapper(planning_scene_monitor_->getPlanningScene(),
                                                 move_group_pipeline,
                                                 interactive_marker_server_,
                                                 kinematic_model_loader_,
                                                 vis_marker_array_publisher_,
                                                 tf_broadcaster_));

    iov_.reset(new InteractiveObjectVisualizationQtWrapper(planning_scene_monitor_->getPlanningScene(),
                                                           interactive_marker_server_,
                                                           col));

    iov_->setUpdateCallback(boost::bind(&MoveItVisualizer::updatePlanningScene, this, _1));

    if(monitor_robot_state) {
        pv_->addMenuEntry("Reset start state", boost::bind(&MoveItVisualizer::updateToCurrentState, this));
        if(allow_trajectory_execution_) {
            pv_->setAllStartChainModes(true);
            pv_->addMenuEntry("Execute last trajectory", boost::bind(&MoveItVisualizer::executeLastTrajectory, this));
            pv_->addMenuEntry("Cycle last trajectory", boost::bind(&MoveItVisualizer::startCycle, this));
            pv_->addMenuEntry("Stop cycle", boost::bind(&MoveItVisualizer::stopCycle, this));
        }
    }
    //pv_->setAllStartVisibility(true);
    //pv_->setAllStartInteractionModes(false);
    pv_->hideAllGroups();


    // Load Rviz
    QWidget* rviz_frame_ = loadRviz();


    main_window_ = new QWidget;
    main_window_->resize(1500,1000);
    main_window_->setWindowTitle("MoveIt Visualizer"); // set window title

    //InteractiveObjectVisualizationWidget* iov_widget = new InteractiveObjectVisualizationWidget(main_window_);

    PrimitiveObjectAdditionDialog* primitive_object_dialog = new PrimitiveObjectAdditionDialog(main_window_);
    MeshObjectAdditionDialog* mesh_object_dialog = new MeshObjectAdditionDialog(main_window_);
    attach_object_addition_dialog_ = new AttachObjectAdditionDialog(main_window_,
                                                                    planning_scene_monitor_->getPlanningScene()->getKinematicModel());

    QHBoxLayout* main_layout = new QHBoxLayout;

    // Create Menu Bar
    QMenuBar* menu_bar = new QMenuBar(main_window_);

    // File Menu
    PlanningSceneFileMenu* planning_scene_file_menu = new PlanningSceneFileMenu(menu_bar);
    QObject::connect(iov_.get(),
                     SIGNAL(updatePlanningSceneSignal(planning_scene::PlanningSceneConstPtr)),
                     planning_scene_file_menu,
                     SLOT(updatePlanningSceneSignalled(planning_scene::PlanningSceneConstPtr)));
    QObject::connect(planning_scene_file_menu->getDatabaseDialog(),
                     SIGNAL(planningSceneLoaded(moveit_msgs::PlanningScenePtr)),
                     iov_.get(),
                     SLOT(loadPlanningSceneSignalled(moveit_msgs::PlanningScenePtr)));

    // Planner menu
    planner_selection_menu_ = new PlannerSelectionMenu(menu_bar);
    QObject::connect(planner_selection_menu_,
                     SIGNAL(plannerSelected(const QString&)),
                     pv_.get(),
                     SLOT(newPlannerSelected(const QString&)));

    // Planning Group Menu
    planning_group_selection_menu_ = new PlanningGroupSelectionMenu(menu_bar);
    // Connect group selection to main program
    QObject::connect(planning_group_selection_menu_,
                     SIGNAL(groupSelected(const QString&)),
                     pv_.get(),
                     SLOT(newGroupSelected(const QString&)));
    // Connect group selection to planner selection
    QObject::connect(planning_group_selection_menu_,
                     SIGNAL(groupSelected(const QString&)),
                     planner_selection_menu_,
                     SLOT(newGroupSelected(const QString&)));
    planning_group_selection_menu_->init(planning_scene_monitor_->getPlanningScene()->getKinematicModel()->getSRDF());

    // Initialize planner menu after planning group menu so that we know what planning group to inialize to
    planner_selection_menu_->init(planning_scene_monitor_->getPlanningScene(), pv_->getCurrentGroup());

    // Add menus in order
    menu_bar->addMenu(planning_scene_file_menu);
    menu_bar->addMenu(planning_group_selection_menu_);
    menu_bar->addMenu(planner_selection_menu_);

    // Collision Object Menu
    coll_object_menu_ = menu_bar->addMenu("Collision Objects");
    QAction* show_primitive_objects_dialog = coll_object_menu_->addAction("Add Primitive Collision Object");
    QObject::connect(show_primitive_objects_dialog, SIGNAL(triggered()), primitive_object_dialog, SLOT(show()));
    QAction* show_mesh_objects_dialog = coll_object_menu_->addAction("Add Mesh Collision Object");
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
    //stuff for handling attached objects
    iov_->addMenuEntry("Attach object",
                       boost::bind(&MoveItVisualizer::attachObject, this, _1));
    QObject::connect(attach_object_addition_dialog_,
                     SIGNAL(attachCollisionObjectRequested(const std::string&,
                                                           const std::string&,
                                                           const std::vector<std::string>&)),
                     iov_.get(),
                     SLOT(attachCollisionObjectSignalled(const std::string&,
                                                         const std::string&,
                                                         const std::vector<std::string>&)));
    main_window_->show();

    planning_scene_monitor_->addUpdateCallback(boost::bind(&MoveItVisualizer::updateSceneCallback, this));
}

MoveItVisualizer::~MoveItVisualizer() {
    iov_.reset();
    pv_.reset();
    if(trajectory_execution_manager_) {
        // TODO: not sure if i need to restore controllers here -binney
        trajectory_execution_manager_.reset();
    }
    planning_scene_monitor_.reset();
    //delete rviz_frame_;
}

QWidget* MoveItVisualizer::loadRviz()
{
    // Create rviz frame
    rviz_render_panel_ = new rviz::RenderPanel();
    //  rviz_render_panel_->setMinimumWidth( 200 );
    rviz_render_panel_->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );

    rviz_manager_ = new rviz::VisualizationManager( rviz_render_panel_ );
    rviz_render_panel_->initialize( rviz_manager_->getSceneManager(), rviz_manager_ );
    rviz_manager_->initialize();
    rviz_manager_->startUpdate();

    // Turn on interactive mode
    /*
      me:  is there a better way than referring to the "Interact" tool by index 1?
      is that guaranteed to be in that order?
      Dave:  it is there by default but depends if you load config from a file
      me:  what is the better way?
      Dave:  you can loop over them and ask them their name
      Dave:  or you might be able to... setCurrentTool( new InteractionTool()) not sure if that works or not
      you'd prolly need to call initialize( context_ ) on the tool
    */
    rviz_manager_->getToolManager()->setCurrentTool( rviz_manager_->getToolManager()->getTool(1) );

    // Set the fixed and target frame
    rviz_manager_->setFixedFrame( QString::fromStdString( planning_scene_monitor_->getPlanningScene()->getPlanningFrame() ) );

    // Add the robot model
    rviz_manager_->createDisplay("rviz/RobotModel", "Robot Model", true);

    // Add a marker display
    rviz::MarkerDisplay* marker_display = new rviz::MarkerDisplay();
    marker_display->setName( "Markers" );
    marker_display->subProp( "Marker Topic" )->setValue(QString::fromStdString(VIS_TOPIC_NAME));
    //  marker_display->setMarkerTopic(VIS_TOPIC_NAME);
    rviz_manager_->addDisplay(marker_display, true);

    // Add interactive marker display
    rviz::InteractiveMarkerDisplay* interactive_marker_display = new rviz::InteractiveMarkerDisplay();
    interactive_marker_display->setName( "Interactive Markers" );
    //  interactive_marker_display->setMarkerUpdateTopic("interactive_kinematics_visualization/update");
    rviz_manager_->addDisplay(interactive_marker_display, true);
    interactive_marker_display->subProp( "Update Topic" )->setValue("interactive_kinematics_visualization/update");

    // Zoom into robot
    rviz::ViewController* view = rviz_manager_->getViewManager()->getCurrent();
    view->subProp( "Distance" )->setValue( 4.0f );

    return rviz_render_panel_;
}

void MoveItVisualizer::publisherFunction(bool joint_states) {
    ros::WallRate r(10.0);

    while(ros::ok())
    {
        joint_state_publisher_->broadcastRootTransform(planning_scene_monitor_->getPlanningScene()->getCurrentState());
        if(joint_states) {
            joint_state_publisher_->publishKinematicState(planning_scene_monitor_->getPlanningScene()->getCurrentState());
        }
        r.sleep();
    }
}

void MoveItVisualizer::updatePlanningScene(planning_scene::PlanningSceneConstPtr planning_scene) {
    current_diff_ = planning_scene;
    pv_->updatePlanningScene(planning_scene);
}

void MoveItVisualizer::updateSceneCallback() {
    if(first_update_) {
        pv_->resetAllStartAndGoalStates();
        first_update_ = false;
    }
    if(allow_trajectory_execution_) {
        updateToCurrentState();
    }
}

bool MoveItVisualizer::doneWithExecution(const moveit_controller_manager::ExecutionStatus& ex_status) {
    ROS_INFO_STREAM("Done");
    boost::lock_guard<boost::mutex> lock(trajectory_execution_mutex_);
    execution_succeeded_ = (bool)ex_status;
    trajectory_execution_finished_.notify_all();
    return true;
}
void MoveItVisualizer::executeLastTrajectory() {
    std::string group_name;
    trajectory_msgs::JointTrajectory traj;
    if(pv_->getLastTrajectory(group_name, traj)) {
        trajectory_execution_manager_->push(traj);
        trajectory_execution_manager_->execute(boost::bind(&MoveItVisualizer::doneWithExecution, this, _1));
    }
}

void MoveItVisualizer::startCycle() {
    std::string group_name;
    trajectory_msgs::JointTrajectory traj;
    if(pv_->getLastTrajectory(group_name, traj) &&
       pv_->cycleOk()) {
        ROS_INFO_STREAM("Just before cycle");
        cycle_thread_.reset(new boost::thread(boost::bind(&MoveItVisualizer::cycleLastTrajectory, this)));
        ROS_INFO_STREAM("Just after cycle");
    }
    ROS_INFO_STREAM("Starting cycle");
}

void MoveItVisualizer::cycleLastTrajectory() {
    std::string group_name;
    trajectory_msgs::JointTrajectory traj;
    if(pv_->getLastTrajectory(group_name, traj)) {
        while(ros::ok()) {
            trajectory_execution_manager_->push(traj);
            trajectory_execution_manager_->execute(boost::bind(&MoveItVisualizer::doneWithExecution, this, _1));

            boost::unique_lock<boost::mutex> lock(trajectory_execution_mutex_);
            trajectory_execution_finished_.wait(lock);
            if(!execution_succeeded_) {
                ROS_WARN_STREAM("Stopping cycle due to failure");
                break;
            } else if(stop_cycle_requested_) {
                stop_cycle_requested_ = false;
                ROS_WARN_STREAM("Stopping cycle due to request");
                break;
            }
        }
    }
}


void MoveItVisualizer::stopCycle() {
    stop_cycle_requested_ = true;
}

void MoveItVisualizer::attachObject(const std::string& name) {
    attach_object_addition_dialog_->attachObject(name);
}

void MoveItVisualizer::updateToCurrentState() {
    iov_->updateCurrentState(planning_scene_monitor_->getPlanningScene()->getCurrentState());
    pv_->resetAllStartStates();
}

}
