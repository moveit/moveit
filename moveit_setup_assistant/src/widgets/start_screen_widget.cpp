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

// Qt
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QString>
#include <QApplication>
#include <QFont>
#include <QFileDialog>
// ROS
#include <ros/ros.h>
#include <ros/package.h> // for getting file path for loadng images
// SA
#include "header_widget.h" // title and instructions
#include "start_screen_widget.h"
// C
#include <fstream>  // for reading in urdf
#include <streambuf>
// Boost
#include <boost/algorithm/string.hpp> // for trimming whitespace from user input
#include <boost/filesystem.hpp>  // for reading folders/files
#include <boost/algorithm/string.hpp> // for string find and replace in paths

namespace moveit_setup_assistant
{

// Boost file system
namespace fs = boost::filesystem; 

// ******************************************************************************************
// Start screen user interface for MoveIt Configuration Assistant
// ******************************************************************************************
StartScreenWidget::StartScreenWidget( QWidget* parent, moveit_setup_assistant::MoveItConfigDataPtr config_data )
  :  SetupScreenWidget( parent ), config_data_( config_data )
{
  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout( this );
  // Horizontal layout splitter 
  QHBoxLayout *hlayout = new QHBoxLayout( );
  // Left side of screen
  QVBoxLayout *left_layout = new QVBoxLayout( );
  // Right side of screen
  QVBoxLayout *right_layout = new QVBoxLayout( );

  // Top Label Area ---------------------------------------------------
  HeaderWidget *header = new HeaderWidget( "MoveIt Setup Assistant",
                                           "Welcome to the MoveIt Setup Assistant! These tools will assist you in creating a planning configuration for your robot. This includes generating a Semantic Robot Description Format (SRDF) file, kinematics configuration file and OMPL planning configuration file. It also involves creating launch files for move groups, OMPL planner, planning contexts and the planning warehouse.",
                                           this);
  layout->addWidget( header );

  // Select Mode Area -------------------------------------------------
  select_mode_ = new SelectModeWidget( this );
  connect( select_mode_->btn_new_, SIGNAL( clicked() ), this, SLOT( showNewOptions() ) );
  connect( select_mode_->btn_exist_, SIGNAL( clicked() ), this, SLOT( showExistingOptions() ) );
  left_layout->addWidget( select_mode_ );

  // Path Box Area ----------------------------------------------------
  
  // Stack Path Dialog
  stack_path_ = new LoadPathWidget("Load MoveIt Configuration Package Path", 
                                   "Specify the package name or path of an existing MoveIt configuration package to be edited for your robot. Example package name: <i>pr2_moveit_config</i>",
                                   true, this); // is directory
  stack_path_->hide(); // user needs to select option before this is shown
  left_layout->addWidget( stack_path_ );

  // URDF File Dialog
  /*urdf_file_ = new LoadPathWidget("Load a URDF or COLLADA Robot Model",
    "Specify the location of an existing Unified Robot Description Format or COLLADA file for your robot. It will load the robot model to the parameter server for you. \nNote: an XACRO URDF must first be converted to a regular XML URDF before opening here. To convert a file run the following command: 'rosrun xacro xacro.py model.xacro > model.urdf'.", 
    false, true, this); // no directory, load only
    urdf_file_->hide(); // user needs to select option before this is shown
    left_layout->addWidget( urdf_file_ );*/

  // URDF Package and File Dialog
  urdf_file_ = new LoadURDFWidget( this );
  urdf_file_->hide();
  left_layout->addWidget( urdf_file_ );

  // SRDF File Dialog
  /*srdf_file_ = new LoadPathWidget("Load a SRDF File (optional)",
    "Specify the location for an existing Semantic Robot Description Format (SRDF) file for your robot, if one exists. It will be copied into the generated MoveIt configuration package. If left blank this setup assistant will create the file for you.",
    false, false, this); // no directory, save
    srdf_file_->hide(); // user needs to select option before this is shown
    left_layout->addWidget( srdf_file_ ); */
  
  // Load settings box ---------------------------------------------
  QHBoxLayout *load_files_layout = new QHBoxLayout();

  progress_bar_ = new QProgressBar( this );
  progress_bar_->setMaximum(100);
  progress_bar_->setMinimum(0);
  progress_bar_->hide();
  load_files_layout->addWidget( progress_bar_ );
  //load_files_layout->setContentsMargins( 20, 30, 20, 30 );

  btn_load_ = new QPushButton("&Load Files", this);
  btn_load_->setMinimumWidth(180);
  btn_load_->setMinimumHeight(40);
  btn_load_->hide();
  load_files_layout->addWidget( btn_load_ );  
  load_files_layout->setAlignment( btn_load_, Qt::AlignRight );  
  connect( btn_load_, SIGNAL( clicked() ), this, SLOT( loadFilesClick() ) );

  // Next step instructions
  next_label_ = new QLabel( this );
  QFont next_label_font( "Arial", 12, QFont::Bold );
  next_label_->setFont(next_label_font);
  next_label_->setWordWrap(true);
  next_label_->hide(); // only show once the files have been loaded. 
  next_label_->setText( "Configuration files loaded successfully.\nMove to next screen using the left navigation pane." );
  
  // Right Image Area ----------------------------------------------
  right_image_ = new QImage();
  const std::string image_path = "./resources/MoveIt_Setup_Asst_xSm.png";
  if(chdir(config_data_->setup_assistant_path_.c_str()) != 0)
  {
    ROS_ERROR("FAILED TO CHANGE PACKAGE TO moveit_setup_assistant");
  }
  if(!right_image_->load( image_path.c_str() ) )
  {
    ROS_ERROR_STREAM("FAILED TO LOAD " << image_path );
  }
  right_image_label_ = new QLabel( this );
  right_image_label_->setPixmap(QPixmap::fromImage( *right_image_));
  right_image_label_->setMinimumHeight(384);  // size of right_image_label_
  //right_image_label_->setMinimumWidth(450);
  right_layout->addWidget(right_image_label_);
  right_layout->setAlignment(right_image_label_, Qt::AlignRight | Qt::AlignTop);

  // Final Layout Setup ---------------------------------------------
  // Alignment
  layout->setAlignment( Qt::AlignTop );
  hlayout->setAlignment( Qt::AlignTop );
  left_layout->setAlignment( Qt::AlignTop );
  right_layout->setAlignment( Qt::AlignTop );

  // Stretch
  left_layout->setSpacing( 30 );
  //hlayout->setContentsMargins( 0, 20, 0, 0);

  // Attach Layouts
  hlayout->addLayout( left_layout );
  hlayout->addLayout( right_layout );
  layout->addLayout( hlayout );

  // Verticle Spacer
  QWidget *vspacer = new QWidget( this );
  vspacer->setSizePolicy( QSizePolicy::Preferred, QSizePolicy::Expanding );
  layout->addWidget( vspacer );
  
  // Attach bottom layout
  layout->addWidget( next_label_);
  layout->addLayout( load_files_layout );
  layout->setAlignment( next_label_, Qt::AlignCenter );  

  this->setLayout(layout);
  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);  


  // Debug mode:
  if( config_data_->debug_ && false )
  {
    select_mode_->btn_new_->click();

    QTimer *update_timer = new QTimer( this );
    update_timer->setSingleShot( true ); // only run once
    connect( update_timer, SIGNAL( timeout() ), btn_load_, SLOT( click() ));
    update_timer->start( 100 );  
  }
}

// ******************************************************************************************
// Destructor
// ******************************************************************************************
StartScreenWidget::~StartScreenWidget()
{

  delete right_image_; // does not have a parent passed to it
}

// ******************************************************************************************
// Show options for creating a new configuration package
// ******************************************************************************************
void StartScreenWidget::showNewOptions()
{
  // Do GUI stuff  
  select_mode_->btn_exist_->setFlat( false );
  select_mode_->btn_new_->setFlat( true );
  urdf_file_->show();
  //  srdf_file_->show();
  stack_path_->hide();
  btn_load_->show();

  // Remember choice
  create_new_package_ = true;
}

// ******************************************************************************************
// Show options for editing an existing configuration package
// ******************************************************************************************
void StartScreenWidget::showExistingOptions()
{
  // Do GUI stuff
  select_mode_->btn_exist_->setFlat( true );
  select_mode_->btn_new_->setFlat( false );
  urdf_file_->hide();
  //srdf_file_->hide();
  stack_path_->show();
  btn_load_->show();

  // Remember choice
  create_new_package_ = false;
}

// ******************************************************************************************
// Load files to parameter server - CLICK
// ******************************************************************************************
void StartScreenWidget::loadFilesClick()
{
  // Disable start screen GUI components from being changed
  urdf_file_->setDisabled(true);
  //srdf_file_->setDisabled(true);
  stack_path_->setDisabled(true);
  select_mode_->setDisabled(true);
  btn_load_->setDisabled(true);
  progress_bar_->show();

  bool result;

  // Decide if this is a new config package, or loading an old one
  if( create_new_package_ )
  {
    result = loadNewFiles();
  }
  else
  {
    result = loadExistingFiles();
  }

  // Check if there was a failure loading files
  if( !result )
  {
    // Renable components
    urdf_file_->setDisabled(false);
    //srdf_file_->setDisabled(false);
    stack_path_->setDisabled(false);
    select_mode_->setDisabled(false);
    btn_load_->setDisabled(false);
    progress_bar_->hide();
  }
  else
  {
    // Hide the logo image so that other screens can resize the rviz thing properly
    right_image_label_->hide();
  }

}

// ******************************************************************************************
// Load exisiting package files
// ******************************************************************************************
bool StartScreenWidget::loadExistingFiles()
{
  // Progress Indicator
  progress_bar_->setValue( 10 );
  QApplication::processEvents();

  // Get the package path
  if( !createFullPackagePath() )
    return false; // error occured

  // Path of .setup_assistant file
  const std::string setup_assistant_file = config_data_->config_pkg_path_ + ".setup_assistant";

  // Check if the old package is a setup assistant package. If it is not, quit
  if( ! fs::is_regular_file( setup_assistant_file ) )
  {
    QMessageBox::warning( this, "Incorrect Directory/Package", 
                          QString("The chosen package location exists but was not previously created using this MoveIt Setup Assistant. If this is a mistake, replace the missing file: ")
                          .append( setup_assistant_file.c_str() ) );
    return false;
  }

  // Get setup assistant data
  if( !config_data_->inputSetupAssistantYAML( setup_assistant_file ) )
  {
    QMessageBox::warning( this, "Setup Assistant File Error",
                          QString("Unable to correctly parse the setup assistant configuration file: " )
                          .append( setup_assistant_file.c_str() ) );
    return false;
  }

  // Progress Indicator
  progress_bar_->setValue( 30 );
  QApplication::processEvents();

  // Get the URDF path using the loaded .setup_assistant data and check it
  if( !createFullURDFPath() )
    return false; // error occured

  // Load the URDF
  if( !loadURDFFile( config_data_->urdf_path_ ) )
    return false; // error occured

  // Get the SRDF path using the loaded .setup_assistant data and check it
  if( !createFullSRDFPath( config_data_->config_pkg_path_ ) )
    return false; // error occured

  // Progress Indicator
  progress_bar_->setValue( 50 );
  QApplication::processEvents();

  // Load the SRDF
  if( !loadSRDFFile( config_data_->srdf_path_ ) )
    return false; // error occured

  // Progress Indicator
  progress_bar_->setValue( 60 );
  QApplication::processEvents();

  // Load kinematics yaml file if available --------------------------------------------------
  const std::string kinematics_yaml_path = config_data_->config_pkg_path_ + "config/kinematics.yaml"; 
  
  if( !config_data_->inputKinematicsYAML( kinematics_yaml_path ) )
  {
    QMessageBox::critical( this, "Error Loading Files", 
                           QString("Failed to parse kinematics yaml file at location ").append( kinematics_yaml_path.c_str() ) );
    return false;
  }

  // DONE LOADING --------------------------------------------------------------------------

  // Call a function that enables navigation
  Q_EMIT readyToProgress();

  // Progress Indicator
  progress_bar_->setValue( 70 );
  QApplication::processEvents();

  // Load Rviz
  Q_EMIT loadRviz(); 

  // Progress Indicator
  progress_bar_->setValue( 100 );
  QApplication::processEvents();

  next_label_->show(); // only show once the files have been loaded

  ROS_INFO( "Loading Setup Assistant Complete" );
  return true; // success!
}

// ******************************************************************************************
// Load chosen files for creating new package
// ******************************************************************************************
bool StartScreenWidget::loadNewFiles()
{
  // Get URDF file names and load to moveit_config_data_
  config_data_->urdf_path_ = urdf_file_->getURDFPath();

  // Check that an error did not occur
  if( config_data_->urdf_path_.empty() )
    return false;
  
  // Copy other data
  config_data_->urdf_pkg_name_ = urdf_file_->robot_desc_pkg_field_->text().toStdString();
  config_data_->urdf_pkg_relative_path_ = urdf_file_->relative_urdf_path_field_->text().toStdString();

  // Progress Indicator
  progress_bar_->setValue( 20 );
  QApplication::processEvents();

  // Load the URDF to the parameter server and check that it is correct format
  if( !loadURDFFile( config_data_->urdf_path_ ) )
    return false; // error occurred

  // Progress Indicator
  progress_bar_->setValue( 50 );
  QApplication::processEvents();

  // Create blank SRDF file
  const std::string blank_srdf = 
    "<?xml version='1.0'?><robot name='" + config_data_->urdf_model_->getName() + "'></robot>";

  // Load a blank SRDF file to the parameter server
  if( !setSRDFFile( blank_srdf ))
  {
    QMessageBox::warning( this, "Error Loading Files", "Failure loading blank SRDF file." );
    return false;
  }

  // Progress Indicator
  progress_bar_->setValue( 60 );
  QApplication::processEvents();

  // DONE LOADING --------------------------------------------------------------------------

  // Call a function that enables navigation
  Q_EMIT readyToProgress();

  // Progress Indicator
  progress_bar_->setValue( 70 );
  QApplication::processEvents();

  // Load Rviz
  Q_EMIT loadRviz(); 

  // Progress Indicator
  progress_bar_->setValue( 100 );
  QApplication::processEvents();

  next_label_->show(); // only show once the files have been loaded

  ROS_INFO( "Loading Setup Assistant Complete" );
  return true; // success!
}

// ******************************************************************************************
// Load URDF File to Parameter Server
// ******************************************************************************************
bool StartScreenWidget::loadURDFFile( const std::string& urdf_file_path )
{
  // check that URDF can be loaded
  std::ifstream urdf_stream( urdf_file_path.c_str() );
  if( !urdf_stream.good() ) // File not found
  {
    QMessageBox::warning( this, "Error Loading Files", QString( "URDF/COLLADA file not found: " ).append( urdf_file_path.c_str() ) );
    return false;
  }
      
  // Load the file to a string using an efficient memory allocation technique
  std::string urdf_string;
  urdf_stream.seekg(0, std::ios::end);   
  urdf_string.reserve(urdf_stream.tellg());
  urdf_stream.seekg(0, std::ios::beg);
  urdf_string.assign( (std::istreambuf_iterator<char>(urdf_stream)), std::istreambuf_iterator<char>() );  
  urdf_stream.close();

  // Verify that file is in correct format / not an XACRO by loading into robot model
  if( !config_data_->urdf_model_->initString( urdf_string ) )
  {
    QMessageBox::warning( this, "Error Loading Files", 
                          "URDF/COLLADA file is not a valid robot model. Is the URDF still in XACRO format?" );
    return false;
  }

  ROS_INFO_STREAM( "Loaded " << config_data_->urdf_model_->getName() << " robot model." );

  // Load the robot model to the parameter server
  ros::NodeHandle nh;
  while (!nh.ok())
  {
    ROS_WARN("Waiting for node handle");
    sleep(1);
    ros::spinOnce(); 
  }

  ROS_INFO("Setting Param Server with Robot Description");
  nh.setParam("/robot_description", urdf_string); // TODO: fix the rosparam issue with large strings

  return true;
}

// ******************************************************************************************
// Load SRDF File to Parameter Server
// ******************************************************************************************
bool StartScreenWidget::loadSRDFFile( const std::string& srdf_file_path )
{
  // check that SRDF can be loaded
  std::ifstream srdf_stream( srdf_file_path.c_str() );
  if( !srdf_stream.good() ) // File not found
  {
    QMessageBox::warning( this, "Error Loading Files", QString( "SRDF file not found: " ).append( config_data_->srdf_path_.c_str() ) );
    return false;
  }
      
  // Load the file to a string using an efficient memory allocation technique
  std::string srdf_string;
  srdf_stream.seekg(0, std::ios::end);   
  srdf_string.reserve(srdf_stream.tellg());
  srdf_stream.seekg(0, std::ios::beg);
  srdf_string.assign( (std::istreambuf_iterator<char>(srdf_stream)), std::istreambuf_iterator<char>() );  
  srdf_stream.close();

  // Put on param server
  return setSRDFFile( srdf_string );
}

// ******************************************************************************************
// Put SRDF File on Parameter Server
// ******************************************************************************************
bool StartScreenWidget::setSRDFFile( const std::string& srdf_string )
{
  // Verify that file is in correct format / not an XACRO by loading into robot model
  if( !config_data_->srdf_->initString( *config_data_->urdf_model_, srdf_string ) )
  {
    QMessageBox::warning( this, "Error Loading Files", 
                          "SRDF file not a valid semantic robot description model." );
    return false;
  }
  ROS_INFO_STREAM( "Robot semantic model successfully loaded." );

  // Load to param server
  ros::NodeHandle nh;
  while (!nh.ok())
  {
    ROS_WARN("Waiting for node handle");
    sleep(1);
    ros::spinOnce(); 
  }

  ROS_INFO("Setting Param Server with Robot Semantic Description");
  nh.setParam("/robot_description_semantic", srdf_string);

  return true;
}

// ******************************************************************************************
// Make the full URDF path using the loaded .setup_assistant data
// ******************************************************************************************
bool StartScreenWidget::createFullURDFPath()
{
  // Check that ROS can find the package
  const std::string robot_desc_pkg_path = ros::package::getPath( config_data_->urdf_pkg_name_ ) + "/";

  if( robot_desc_pkg_path.empty() )
  {
    QMessageBox::warning( this, "Error Loading Files", QString("ROS was unable to find the package name '")
                          .append( config_data_->urdf_pkg_name_.c_str() )
                          .append("'. Verify this package is inside your ROS workspace and is a proper ROS package.") );
    return false;
  }

  // Append the relative URDF url path
  config_data_->urdf_path_ = robot_desc_pkg_path + config_data_->urdf_pkg_relative_path_;

  // Trim whitespace from user input
  boost::trim( config_data_->urdf_path_ );

  // Check that this file exits
  if( ! fs::is_regular_file( config_data_->urdf_path_ ) )
  {
    QMessageBox::warning( this, "Error Loading Files",
                          QString("Unable to locate the URDF file in package. File: " )
                          .append( config_data_->urdf_path_.c_str() ) );
    return false;
  }

  return true; // success
}

// ******************************************************************************************
// Make the full SRDF path using the loaded .setup_assistant data
// ******************************************************************************************
bool StartScreenWidget::createFullSRDFPath( const std::string& package_path )
{
  // Append the relative SRDF url path
  config_data_->srdf_path_ = package_path + config_data_->srdf_pkg_relative_path_;

  // Check that this file exits
  if( ! fs::is_regular_file( config_data_->srdf_path_ ) )
  {
    QMessageBox::warning( this, "Error Loading Files",
                          QString("Unable to locate the SRDF file: " )
                          .append( config_data_->srdf_path_.c_str() ) );
    return false;
  }

  return true; // success
}

// ******************************************************************************************
// Get the full package path for editing an existing package
// ******************************************************************************************
bool StartScreenWidget::createFullPackagePath()
{
  // Get package path
  std::string package_path_input = stack_path_->getPath();
  std::string full_package_path;

  // Trim whitespace from user input
  boost::trim( package_path_input );

  // check that a folder is provided
  if( package_path_input.empty() )
  {
    QMessageBox::warning( this, "Error Loading Files", "Please specify a configuration package path to load." );
    return false;
  }

  // Decide if this is a package name or a full path ----------------------------------------------

  // check that the folder exists
  if( !fs::is_directory( package_path_input ) )
  {
    // does not exist, check if its a package
    full_package_path = ros::package::getPath( package_path_input ) + "/";    

    // check that the folder exists
    if( !fs::is_directory( full_package_path ) )
    {
      // error
      QMessageBox::critical( this, "Error Loading Files", "The specified path is not a directory or is not accessable" );
      return false;
    }
  }
  else
  {
    // they inputted a full path
    full_package_path = package_path_input;

    // Add a slash at end of path name if it does not currently have one
    size_t slash_pos = full_package_path.find_last_of("/\\");

    // Make sure the last slash is the last character
    if( slash_pos != full_package_path.size() - 1 )
    {
      // Add the slash at end
      full_package_path.append("/"); // TODO: not Windows ready
    
      // Update the textbox
      stack_path_->setPath( full_package_path );
    }

  }

  config_data_->config_pkg_path_ = full_package_path;

  return true;
}


// ******************************************************************************************
// ******************************************************************************************
// Class for selecting which mode
// ******************************************************************************************
// ******************************************************************************************

// ******************************************************************************************
// Create the widget
// ******************************************************************************************
SelectModeWidget::SelectModeWidget( QWidget* parent )
  : QFrame(parent)
{
  // Set frame graphics
  setFrameShape(QFrame::StyledPanel);
  setFrameShadow(QFrame::Raised);
  setLineWidth(1);
  setMidLineWidth(0);

  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout(this);

  // Horizontal layout splitter
  QHBoxLayout *hlayout = new QHBoxLayout();
    
  // Widget Title
  QLabel * widget_title = new QLabel(this);
  widget_title->setText( "Choose mode:" );
  QFont widget_title_font( "Arial", 12, QFont::Bold );
  widget_title->setFont(widget_title_font);
  layout->addWidget( widget_title);
  layout->setAlignment( widget_title, Qt::AlignTop);

  // Widget Instructions
  QLabel * widget_instructions = new QLabel(this);
  widget_instructions->setText( "All settings for MoveIt are stored in a Moveit configuration package. Here you have the option to create a new configuration package from scractch, or load an existing one. Note: any changes to a MoveIt configuration package outside this setup assistant will likely be overwritten by this tool." );
  widget_instructions->setWordWrap(true);
  widget_instructions->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);  
  layout->addWidget( widget_instructions);
  layout->setAlignment( widget_instructions, Qt::AlignTop);    

  // New Button
  btn_new_ = new QPushButton(this);
  btn_new_->setText("Create &New MoveIt\nConfiguration Package");
  hlayout->addWidget( btn_new_ );

  // Exist Button
  btn_exist_ = new QPushButton(this);
  btn_exist_->setText("&Edit Existing MoveIt\nConfiguration Package");
  hlayout->addWidget( btn_exist_ );

  // Add horizontal layer to verticle layer
  layout->addLayout(hlayout);
  setLayout(layout);
}

// ******************************************************************************************
// ******************************************************************************************
// Class for selecting urdf package and file
// ******************************************************************************************
// ******************************************************************************************

// ******************************************************************************************
// Create the widget
// ******************************************************************************************
LoadURDFWidget::LoadURDFWidget( QWidget* parent )
  : QFrame(parent)
{
  // Set frame graphics
  setFrameShape(QFrame::StyledPanel);
  setFrameShadow(QFrame::Raised);
  setLineWidth(1);
  setMidLineWidth(0);

  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout(this);

  // Widget Title
  QLabel * widget_title = new QLabel(this);
  widget_title->setText( "Load a URDF or Collada Robot Model" );
  QFont widget_title_font( "Arial", 12, QFont::Bold );
  widget_title->setFont(widget_title_font);
  layout->addWidget( widget_title);
  layout->setAlignment( widget_title, Qt::AlignTop);

  // Widget Instructions
  QLabel * widget_instructions = new QLabel(this);
  widget_instructions->setText( "Provide the name of the 'robot description' package that contains your robot's URDF/Collada file. Then provide a package-relative path to a URDF/Collada file. This setup assistant will load the robot model to the parameter server for you. Note: an XACRO URDF must first be converted to a regular XML URDF before opening here. To convert a file run the following command: <i>rosrun xacro xacro.py model.xacro > model.urdf</i>." );
  widget_instructions->setWordWrap(true);
  widget_instructions->setTextFormat( Qt::RichText );
  layout->addWidget( widget_instructions);
  layout->setAlignment( widget_instructions, Qt::AlignTop);    

  // Robot Description Package Name ----------------------------------------------

  // Label
  QLabel * field1_label = new QLabel(this);
  field1_label->setText( "Package containing URDF:" );
  layout->addWidget( field1_label);

  // Layout 1
  QHBoxLayout * field1_layout = new QHBoxLayout();

  // Field
  robot_desc_pkg_field_ = new QLineEdit( this );
  robot_desc_pkg_field_->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
  field1_layout->addWidget( robot_desc_pkg_field_ );

  // Button
  QPushButton *btn_browse1 = new QPushButton(this);
  btn_browse1->setText("Browse");
  connect( btn_browse1, SIGNAL( clicked() ), this, SLOT( btn_folder_dialog() ) );
  field1_layout->addWidget( btn_browse1 );
  field1_layout->setAlignment( btn_browse1, Qt::AlignRight );

  // Add horizontal layer to verticle layer
  layout->addLayout( field1_layout );

  // Relative URDF path -----------------------------------------------------------

  // Label
  QLabel * field2_label = new QLabel(this);
  field2_label->setText( "Relative Path to URDF in Package:" );
  layout->addWidget( field2_label);

  // Layout 2
  QHBoxLayout * field2_layout = new QHBoxLayout();

  // Field
  relative_urdf_path_field_ = new QLineEdit( this );
  relative_urdf_path_field_->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
  field2_layout->addWidget( relative_urdf_path_field_ );
  //field2_layout->setAlignment( relative_urdf_path_field_, Qt::AlignLeft );

  // Button
  QPushButton *btn_browse2 = new QPushButton(this);
  btn_browse2->setText("Browse");
  connect( btn_browse2, SIGNAL( clicked() ), this, SLOT( btn_file_dialog() ) );
  field2_layout->addWidget( btn_browse2 );
  field2_layout->setAlignment( btn_browse2, Qt::AlignRight );

  // Add horizontal layer to verticle layer
  layout->addLayout( field2_layout );

  // Done
  setLayout(layout);

  // Init values
  robot_desc_pkg_path_ = "";
}

// ******************************************************************************************
// Load the folder dialog
// ******************************************************************************************
void LoadURDFWidget::btn_folder_dialog()
{
  // Decide where to start off the popup box
  std::string initial_path = robot_desc_pkg_path_;

  // Popup the folder dialog
  QString path = QFileDialog::getExistingDirectory(this, "Open Robot Description Folder", initial_path.c_str(), 
                                                   QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks); 

  // Check they did not press cancel
  if (path == NULL)
    return;

  // Confirm that the chosen folder is within the robot description package
  robot_desc_pkg_path_ = path.toStdString();

  // Remove the whole path except the lowest child folder ----------------------------------

  // Find the last slash
  int slash_pos = robot_desc_pkg_path_.find_last_of("/\\");

  // Take the substring after the slash
  std::string package_name = robot_desc_pkg_path_.substr( slash_pos+1, robot_desc_pkg_path_.size() );

  // Put modified name in text field
  robot_desc_pkg_field_->setText( package_name.c_str() );
}

// ******************************************************************************************
// Load the file dialog
// ******************************************************************************************
void LoadURDFWidget::btn_file_dialog()
{
  // Get the full path to the robot description package
  const std::string robot_desc_pkg_path = getURDFPackagePath();

  // Exit if error occured
  if( robot_desc_pkg_path.empty() )
    return;
  
  // Popup the file dialog
  QString start_path = robot_desc_pkg_path.c_str();
  QString path = QFileDialog::getOpenFileName(this, "Open File", start_path, "");

  // Check they did not press cancel
  if (path == NULL)
    return;

  // Confirm that the chosen file is within the robot description package
  std::string urdf_path = path.toStdString();
  if( urdf_path.find( robot_desc_pkg_path ) == std::string::npos )
  {
    // they have chosen a location outside the robot description path
    QMessageBox::warning( this, "Error Loading Files", QString("The chosen URDF path is outside the ROS package you chose for the robot description package. Original path: " )
                          .append( robot_desc_pkg_field_->text() ).append("'. URDF file path: '")
                          .append( path ) );
    return;
  }    

  // Remove preceeding string part
  boost::replace_all( urdf_path, robot_desc_pkg_path, "" );

  relative_urdf_path_field_->setText( urdf_path.c_str() );
}

// ******************************************************************************************
// Get the full URDF path
// ******************************************************************************************
const std::string LoadURDFWidget::getURDFPath()
{
  // Get the full path to the robot description package
  const std::string robot_desc_pkg_path = getURDFPackagePath();

  // Exit if error occured
  if( robot_desc_pkg_path.empty() )
    return "";

  // Check that a relative URL has been added
  if( !relative_urdf_path_field_->text().size() )
  {
    QMessageBox::warning( this, "Error Loading Files", "Please specify the package-relative path to the actual URDF/Collada file." );
    return "";
  }    

  // Check that the relative path does not have preceeding slash
  std::string relative_path = relative_urdf_path_field_->text().toStdString();

  if( relative_path.substr( 0, 1 ) == "/" ||
      relative_path.substr( 0, 1 ) == "/\\" )
  {
    relative_path = relative_path.substr( 1, relative_path.size() - 1 );
    relative_urdf_path_field_->setText( relative_path.c_str() );
  }

  // Append the relative URDF url path
  std::string full_urdf_file_path = robot_desc_pkg_path + relative_path;

  // Trim whitespace from user input
  boost::trim( full_urdf_file_path );

  // Check that this file exits
  if( ! fs::is_regular_file( full_urdf_file_path ) )
  {
    QMessageBox::warning( this, "Error Loading Files",
                          QString("Unable to locate the URDF file: " )
                          .append( full_urdf_file_path.c_str() ) );
    return "";
  }
  
  return full_urdf_file_path;
}


// ******************************************************************************************
// Get the full robot descriuption path
// ******************************************************************************************
const std::string LoadURDFWidget::getURDFPackagePath()
{
  // Check that a package has been provided
  if( !robot_desc_pkg_field_->text().size() )
  {
    QMessageBox::warning( this, "Error Loading Files", "Please first specify the ROS package containing the URDF/Collada file." );
    return "";
  }  

  const std::string robot_desc_pkg_name = robot_desc_pkg_field_->text().toStdString();
  
  // Check that there are no slashes
  if( robot_desc_pkg_name.find( "/" ) != std::string::npos )
  {
    // they have chosen a location outside the robot description path
    QMessageBox::warning( this, "Error Loading Files", "No slashes allowed in package name" );
    return "";
  }      

  // Check that ROS can find the package
  const std::string robot_desc_pkg_path = ros::package::getPath( robot_desc_pkg_name ) + "/";
  if( robot_desc_pkg_path.empty() )
  {
    QMessageBox::warning( this, "Error Loading Files", QString("ROS was unable to find the package name '")
                          .append( robot_desc_pkg_field_->text() ).append("'. Verify this package is inside your ROS workspace and is a proper ROS package.") );
    return "";
  }

  return robot_desc_pkg_path;
}


} // namespace
