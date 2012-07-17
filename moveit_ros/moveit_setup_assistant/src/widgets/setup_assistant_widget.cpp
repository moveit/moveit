/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman */

// SA
#include "setup_screen_widget.h" // a base class for screens in the setup assistant
#include "setup_assistant_widget.h"
// Qt
#include <QStackedLayout>
#include <QListWidget>
#include <QListWidgetItem>
#include <QDebug>
#include <QFont>
#include <QLabel>
#include <QPushButton>
#include <QCloseEvent>
#include <QMessageBox>
#include <pluginlib/class_loader.h> // for loading all avail kinematic planners
// Rviz
#include <rviz/default_plugin/marker_display.h>
#include <rviz/default_plugin/interactive_marker_display.h>
#include <rviz/display_wrapper.h>
#include <rviz/view_controllers/orbit_view_controller.h>
#include <moveit_rviz_plugin/planning_display.h>
// ROS
#include <ros/master.h> // for checking if roscore is started

namespace moveit_setup_assistant
{

// ******************************************************************************************
// Outer User Interface for MoveIt Configuration Assistant
// ******************************************************************************************
SetupAssistantWidget::SetupAssistantWidget( QWidget *parent, boost::program_options::variables_map args )
  : QWidget( parent )
{
  // Create timer to ping ROS ----------------------------------------
  //  QTimer *update_timer = new QTimer( this );
  //  connect( update_timer, SIGNAL( timeout() ), this, SLOT( updateTimer() ));
  //  update_timer->start( 250 );
  
  // Create object to hold all moveit configuration data
  config_data_.reset( new MoveItConfigData() );

  // Set debug mode flag if necessary
  if (args.count("debug"))
    config_data_->debug_ = true;

  // Basic widget container -----------------------------------------
  QHBoxLayout *layout = new QHBoxLayout();
  layout->setAlignment( Qt::AlignTop );

  // Create main content stack for various screens
  main_content_ = new QStackedLayout();

  // Wrap main_content_ with a widget
  right_frame_ = new QWidget( this );
  right_frame_->setLayout( main_content_ );

  // Screens --------------------------------------------------------

  // Start Screen
  ssw_ = new StartScreenWidget( this, config_data_ );
  connect( ssw_, SIGNAL( readyToProgress() ), this, SLOT( progressPastStartScreen() ) );
  connect( ssw_, SIGNAL( loadRviz() ), this, SLOT( loadRviz() ) );
  main_content_->addWidget(ssw_);

  // Pass command arg values to start screen
  if (args.count("urdf_pkg"))
    ssw_->urdf_file_->robot_desc_pkg_field_->setText( args["urdf_pkg"].as<std::string>().c_str() );
  if (args.count("urdf_path"))
    ssw_->urdf_file_->relative_urdf_path_field_->setText( args["urdf_path"].as<std::string>().c_str() );
  if (args.count("config_pkg"))
  {
    ssw_->stack_path_->setPath( args["config_pkg"].as<std::string>() );

    // Show this part of screen
    ssw_->select_mode_->btn_exist_->click();
  }
  // Add Navigation Buttons (but do not load widgets yet except start screen)
  nav_name_list_ << "Start";
  nav_name_list_ << "Self-Collisions";
  nav_name_list_ << "Planning Groups";
  nav_name_list_ << "Robot Poses";
  nav_name_list_ << "End Effectors";
  nav_name_list_ << "Virtual Joints";
  nav_name_list_ << "Configuration Files";

  // Navigation Left Pane --------------------------------------------------
  navs_view_ = new NavigationWidget( this );
  navs_view_->setNavs(nav_name_list_);
  navs_view_->setDisabled( true );
  navs_view_->setSelected( 0 ); // start screen

  // Rviz View Right Pane ---------------------------------------------------
  rviz_container_ = new QWidget( this );
  rviz_container_->hide(); // do not show until after the start screen

  // Split screen -----------------------------------------------------
  splitter_ = new QSplitter( Qt::Horizontal, this );
  splitter_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  splitter_->addWidget( navs_view_ );
  splitter_->addWidget( right_frame_ );  
  splitter_->addWidget( rviz_container_ );
  splitter_->setHandleWidth( 6 );
  //splitter_->setCollapsible( 0, false ); // don't let navigation collapse
  layout->addWidget( splitter_ );

  // Add event for switching between screens -------------------------
  connect( navs_view_, SIGNAL(clicked(const QModelIndex&)), this, SLOT(navigationClicked(const QModelIndex&)) );

  
  // Final Layout Setup ---------------------------------------------
  this->setLayout(layout);
  
  // Title
  this->setWindowTitle("MoveIt Setup Assistant"); // title of window

  // Show screen before message
  QApplication::processEvents();

  // Check that ROS Core is running
  bool do_check = true;
  while( do_check )
  {
    if( ! ros::master::check() )
    {
      // roscore is not running
      QMessageBox::warning( this, "ROS Error", 
                            "ROS Core does not appear to be started. Be sure to run the command 'roscore' at command line before using this application.");    
    }
    else
    {
      do_check = false;
    }
  }

}

// ******************************************************************************************
// Change screens of Setup Assistant
// ******************************************************************************************
void SetupAssistantWidget::navigationClicked( const QModelIndex& index )
{
  // Convert QModelIndex to int
  moveToScreen( index.row() );
}

// ******************************************************************************************
// Change screens
// ******************************************************************************************
void SetupAssistantWidget::moveToScreen( const int index )
{
  // Use this static variable to prevent double clicks on navigation from slowing down system
  static int current_index = 0;

  if( current_index != index )
  {
    current_index = index;

    // Show Rviz if appropriate
    /*if( index != 0 )
      {
      rviz_container_->show();
      }
      else
      {
      rviz_container_->hide();
      // hide the start screen image so that is doesn't mess up the rviz column resizing      
      }*/
    rviz_container_->show();

    // Change screens
    main_content_->setCurrentIndex( index );

    // Send the focus given command to the screen widget
    SetupScreenWidget *ssw = qobject_cast< SetupScreenWidget* >( main_content_->widget( index ) );
    ssw->focusGiven();  

    // Change navigation selected option
    navs_view_->setSelected( index ); // Select first item in list
  }
}

// ******************************************************************************************
// Loads other windows, enables navigation and goes to screen 2
// ******************************************************************************************
void SetupAssistantWidget::progressPastStartScreen()
{
  // Load all widgets ------------------------------------------------
  
  // Self-Collisions
  cdcw_ = new DefaultCollisionsWidget( this, config_data_);
  main_content_->addWidget(cdcw_);

  // Planning Groups
  pgw_ = new PlanningGroupsWidget( this, config_data_ );
  main_content_->addWidget(pgw_);

  // Robot Poses
  rpw_ = new RobotPosesWidget( this, config_data_ );
  main_content_->addWidget(rpw_);

  // End Effectors
  efw_ = new EndEffectorsWidget( this, config_data_ );
  main_content_->addWidget(efw_);  

  // Virtual Joints
  vjw_ = new VirtualJointsWidget( this, config_data_ );
  main_content_->addWidget(vjw_);  

  // Configuration Files
  cfw_ = new ConfigurationFilesWidget( this, config_data_ );
  main_content_->addWidget(cfw_);  

  // Enable all nav buttons -------------------------------------------
  for( int i = 0; i < nav_name_list_.count(); ++i)
  {
    navs_view_->setEnabled( i, true );
  }

  // Go to next screen
  //moveToScreen( 2 );

  // Enable navigation
  navs_view_->setDisabled( false );

  // Replace logo with Rviz screen
  rviz_container_->show();
}

// ******************************************************************************************
// Ping ROS on internval
// ******************************************************************************************
void SetupAssistantWidget::updateTimer()
{
  ros::spinOnce(); // keep ROS node alive
}

// ******************************************************************************************
// Load Rviz once we have a robot description ready
// ******************************************************************************************
void SetupAssistantWidget::loadRviz()
{
  // Create rviz frame
  rviz_frame_ = new rviz::VisualizationPanel();
  //rviz_frame_->setMinimumWidth( 800 );

  // Turn on interactive mode
  // EGJ: kind of hacky way to do this, given the way that the vis manager is creating tools
  //rviz_frame_->getManager()->setCurrentTool(rviz_frame_->getManager()->getTool(1));

  // Sizes for QSplitter - allows the left pane to be hidden
  QList<int> sizes;
  sizes.push_back(0);
  sizes.push_back(1000);
  rviz_frame_->setSizes(sizes); 

  // Set the fixed and target frame 
  rviz_frame_->getManager()->setFixedFrame( config_data_->getPlanningScene()->getPlanningFrame() );
  rviz_frame_->getManager()->setTargetFrame( config_data_->getPlanningScene()->getPlanningFrame() );

  // Add Motion Planning Plugin to Rviz
  rviz::DisplayWrapper* display_wrapper = rviz_frame_->getManager()->
    createDisplay( "moveit_rviz_plugin/MotionPlanning","Motion Planning", true );
  
  // Get Motion Planning Display Reference
  moveit_rviz_plugin::PlanningDisplay* planning_display = 
    dynamic_cast<moveit_rviz_plugin::PlanningDisplay*>( display_wrapper->getDisplay() );
  
  // Turn off planned path
  planning_display->setVisualVisible( false );

  // Set the topic on which the moveit_msgs::PlanningScene messages are recieved
  planning_display->setPlanningSceneTopic( MOVEIT_PLANNING_SCENE );

  // Set robot description
  planning_display->setRobotDescription( ROBOT_DESCRIPTION );

  // Set the Orbit View
  rviz::OrbitViewController* orbit_view = 
    dynamic_cast<rviz::OrbitViewController*>(rviz_frame_->getManager()->getCurrentViewController());

  if(orbit_view == NULL) 
  {
    ROS_WARN_STREAM("Current view controller not orbit");
  } 
  else 
  {
    orbit_view->zoom(14.0);
  }
  
  // Add RobotModel Display to Rviz
  //rviz_frame_->getManager()->createDisplay("rviz/RobotModel", "Robot Model", true);
  /*
  // Add Marker Display to Rviz 
  rviz::DisplayWrapper* marker_display = rviz_frame_->getManager()->createDisplay("rviz/Marker", "Markers", true);
  // Get pointer to created marker display 
  rviz::MarkerDisplay* md = dynamic_cast<rviz::MarkerDisplay*>(marker_display->getDisplay());
  1  // Set Marker Topic Name
  md->setMarkerTopic(VIS_TOPIC_NAME);
  // Add Interactive Marker Display to Rviz
  rviz::DisplayWrapper* interactive_marker_display = rviz_frame_->getManager()->
  createDisplay("rviz/InteractiveMarker", "Interactive Markers", true);
  // Get pointer to created interactive marker
  rviz::InteractiveMarkerDisplay* imd = dynamic_cast<rviz::InteractiveMarkerDisplay*>(interactive_marker_display->getDisplay());
  // Set Interactive Marker Name
  imd->setMarkerUpdateTopic("interactive_kinematics_visualization/update");
  */

  // Add Rviz to Planning Groups Widget
  QVBoxLayout *rviz_layout = new QVBoxLayout();
  rviz_layout->addWidget( rviz_frame_ );
  rviz_container_->setLayout( rviz_layout );

}

// ******************************************************************************************
// Show/hide Rviz Frame
// ******************************************************************************************
void SetupAssistantWidget::showRviz( bool show )
{
  QList<int> sizes;
  sizes.push_back(0);
  sizes.push_back(1000);
  rviz_frame_->setSizes(sizes); 
}

// ******************************************************************************************
// Qt close event function for reminding user to save
// ******************************************************************************************
void SetupAssistantWidget::closeEvent( QCloseEvent * event )
{
  // Only prompt to close if not in debug mode 
  if( !config_data_->debug_ )
  {
    if( QMessageBox::question( this, "Exit Setup Assistant", 
                               QString("Are you sure you want to exit the MoveIt Setup Assistant?"),
                               QMessageBox::Ok | QMessageBox::Cancel) 
        == QMessageBox::Cancel )
    {
      event->ignore();
      return;
    }
  }

  // Shutdown app
  event->accept();
}

// ******************************************************************************************
// Qt Error Handling 
// ******************************************************************************************
bool SetupAssistantWidget::notify( QObject * reciever, QEvent * event )
{
  /*  try
      {
      return QApplication::notify( reciever, event );
      }
      catch( std::Exception& event)
      {*/
  QMessageBox::critical( this, "Error", "An error occurred and was caught by Qt notify event handler.", QMessageBox::Ok);

  /*  }
      catch(...)
      {
      QMessageBox::warning(0, "An unexpected error occurred", "This is likely a bug.");
      }*/
  return false; 
}



} // namespace
