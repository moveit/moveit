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

#include <QFileDialog>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QString>
#include <QFont>
#include <ros/ros.h>
#include <ros/package.h> // for getting file path for loading images
#include <ros/master.h> // for checking if roscore is started
#include <fstream>  // for reading in urdf
#include <streambuf>
#include <boost/algorithm/string.hpp> // for trimming whitespace from user input
#include "header_widget.h" // title and instructions
#include "start_screen_widget.h"


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
  //right_layout->setContentsMargins( 20, 0, 0, 0);

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
                                   "Specify the location for an existing MoveIt configuration package to be edited for your robot. Example package name: '~/ros/pr2_moveit_config'",
                                   true, this); // is directory
  stack_path_->hide(); // user needs to select option before this is shown
  left_layout->addWidget( stack_path_ );

  // URDF File Dialog
  urdf_file_ = new LoadPathWidget("Load a URDF or COLLADA Robot Model",
                                  "Specify the location of an existing Unified Robot Description Format or COLLADA file for your robot. It will load the robot model to the parameter server for you. \nNote: an XACRO URDF must first be converted to a regular XML URDF before opening here. To convert a file run the following command: 'rosrun xacro xacro.py model.xacro > model.urdf'.", 
                                  false, true, this); // no directory, load only
  urdf_file_->hide(); // user needs to select option before this is shown
  left_layout->addWidget( urdf_file_ );

  // SRDF File Dialog
  srdf_file_ = new LoadPathWidget("Load a SRDF File (optional)",
                                  "Specify the location for an existing Semantic Robot Description Format (SRDF) file for your robot, if one exists. It will be copied into the generated MoveIt configuration package. If left blank this setup assistant will create the file for you.",
                                  false, false, this); // no directory, save
  srdf_file_->hide(); // user needs to select option before this is shown
  left_layout->addWidget( srdf_file_ );
  
  // Load settings box ---------------------------------------------

  btn_load_ = new QPushButton("&Load Files", this);
  btn_load_->setMinimumWidth(180);
  btn_load_->setMinimumHeight(40);
  btn_load_->hide();
  left_layout->addWidget( btn_load_ );  
  left_layout->setAlignment( btn_load_, Qt::AlignRight );  
  connect( btn_load_, SIGNAL( clicked() ), this, SLOT( loadFiles() ) );
  
  // Right Image Area ----------------------------------------------
  QImage image;

  if(chdir(ros::package::getPath("moveit_setup_assistant").c_str()) != 0)
  {
    ROS_ERROR("FAILED TO CHANGE PACKAGE TO moveit_setup_assistant");
  }
  if(!image.load("./resources/MoveIt_Setup_Asst_Sm.png"))
  {
    ROS_ERROR("FAILED TO LOAD ./resources/wizard.png");
  }
  QLabel* imageLabel = new QLabel( this );
  imageLabel->setPixmap(QPixmap::fromImage(image));
  imageLabel->setMinimumHeight(493);  // size of imageLabel
  imageLabel->setMinimumWidth(450);
  right_layout->addWidget(imageLabel);
  right_layout->setAlignment(imageLabel, Qt::AlignRight | Qt::AlignTop);


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

  this->setLayout(layout);
  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);  

  // Development Only TODO REMOVE
  urdf_file_->setPath( "/u/dcoleman/ros/moveit/moveit_pr2/pr2_moveit_config/config/pr2.urdf" );
  srdf_file_->setPath( "/u/dcoleman/ros/moveit/moveit_pr2/pr2_moveit_tests/pr2_jacobian_tests/pr2.srdf" );
  select_mode_->btn_new_->click();


  if( true )
  {
    QTimer *update_timer = new QTimer( this );
    update_timer->setSingleShot( true ); // only run once
    connect( update_timer, SIGNAL( timeout() ), btn_load_, SLOT( click() ));
    update_timer->start( 200 );  
  }
}

// ******************************************************************************************
// Destructor
// ******************************************************************************************
StartScreenWidget::~StartScreenWidget()
{

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
  srdf_file_->show();
  stack_path_->hide();
  btn_load_->show();
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
  srdf_file_->hide();
  stack_path_->show();
  btn_load_->show();
}

// ******************************************************************************************
// Load files to parameter server
// ******************************************************************************************
void StartScreenWidget::loadFiles()
{
  std::string urdf_path = urdf_file_->getPath();

  // Trim whitespace from user input
  boost::trim( urdf_path );

  // check that a file is provided
  if( urdf_path == "" )
  {
    QMessageBox::critical( this, "Error Loading Files", "Please specify a URDF or COLLADA file to load" );
    return;
  }

  // check that URDF can be loaded
  std::ifstream urdf_stream( urdf_path.c_str() );
  if( !urdf_stream.good() ) // File not found
  {
    QMessageBox::critical( this, "Error Loading Files", "URDF/COLLADA file not found" );
    return;
  }
      
  // Load the file to a string using an efficient memory allocation technique
  std::string urdf_string;

  urdf_stream.seekg(0, std::ios::end);   
  urdf_string.reserve(urdf_stream.tellg());
  urdf_stream.seekg(0, std::ios::beg);
  urdf_string.assign((std::istreambuf_iterator<char>(urdf_stream)),
                     std::istreambuf_iterator<char>());  

  // Verify that file is in correct format / not an XACRO by loading into robot model
  urdf::Model robot_model;
  if( !robot_model.initString( urdf_string ) )
  {
    QMessageBox::critical( this, "Error Loading Files", 
                           "URDF/COLLADA file not a valid robot model. Is the URDF still in XACRO format?" );
    return;
  }
  else
  {
    ROS_INFO_STREAM( "Loaded " << robot_model.getName() << " robot model." );

    // Copy path to config data
    config_data_->urdf_path_ = urdf_string;
  }

  // Check that ROS Core is running
  if( ! ros::master::check() )
  {
    // roscore is not running
    QMessageBox::critical( this, "ROS Error", 
                           "ROS Core does not appear to be started. Be sure to run the command 'roscore' at command line before using this application.");
    return;
  }

  // Load the robot model to the parameter server
  ros::NodeHandle nh;
  ros::spinOnce();
  nh.setParam("/robot_description", urdf_string);
  ros::spinOnce();


  // SRDF -----------------------------------------------------
  std::string srdf_path = srdf_file_->getPath();

  // Trim whitespace from user input
  boost::trim( srdf_path );
    
  // check that a file is provided. if not, we don't bother with anything else
  if( srdf_path != "" )
  {
    // check that SRDF can be loaded
    std::ifstream srdf_stream( srdf_path.c_str() );
    if( !srdf_stream.good() ) // File not found
    {
      QMessageBox::critical( this, "Error Loading Files", 
                             "SRDF file not found. This file is optional, so leaving the textbox blank is also allowable" );
      return;
    }
      
    // Load the file to a string using an efficient memory allocation technique
    std::string srdf_string;

    srdf_stream.seekg(0, std::ios::end);   
    srdf_string.reserve(srdf_stream.tellg());
    srdf_stream.seekg(0, std::ios::beg);
    srdf_string.assign((std::istreambuf_iterator<char>(srdf_stream)),
                       std::istreambuf_iterator<char>());  

    // Verify that file is in correct format by loading into srdf parser
    if( !config_data_->srdf_->initString( robot_model, srdf_string ) )
    {
      QMessageBox::critical( this, "Error Loading Files", 
                             "SRDF file not a valid semantic robot description model." );
      return;
    }
    else
    {
      ROS_INFO_STREAM( "Robot semantic model successfully loaded." );
      
      // Copy path to config data
      config_data_->srdf_path_ = srdf_string;
    }

    // Load the robot model to the parameter server
    ros::NodeHandle nh;
    ros::spinOnce();
    nh.setParam("/robot_description_semantic", srdf_string);
    ros::spinOnce();
  }

  // Call a function that enables navigation and goes to screen 2
  Q_EMIT readyToProgress();

  // Disable start screen GUI components from being changed
  urdf_file_->setDisabled(true);
  srdf_file_->setDisabled(true);
  stack_path_->setDisabled(true);
  select_mode_->setDisabled(true);
  btn_load_->hide();
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
// Class for selecting files
// ******************************************************************************************
// ******************************************************************************************

// ******************************************************************************************
// Create the widget
// ******************************************************************************************
LoadPathWidget::LoadPathWidget( const std::string &title, const std::string &instructions, 
                                const bool dir_only, const bool load_only, QWidget* parent )
  : QFrame(parent), dir_only_(dir_only), load_only_(load_only)
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
  widget_title->setText( title.c_str() );
  QFont widget_title_font( "Arial", 12, QFont::Bold );
  widget_title->setFont(widget_title_font);
  layout->addWidget( widget_title);
  layout->setAlignment( widget_title, Qt::AlignTop);

  // Widget Instructions
  QLabel * widget_instructions = new QLabel(this);
  widget_instructions->setText( instructions.c_str() );
  widget_instructions->setWordWrap(true);
  layout->addWidget( widget_instructions);
  layout->setAlignment( widget_instructions, Qt::AlignTop);    

  // Line Edit
  path_box_ = new QLineEdit(this);
  hlayout->addWidget(path_box_);

  // Button
  button_ = new QPushButton(this);
  button_->setText("Browse");
  connect( button_, SIGNAL( clicked() ), this, SLOT( btn_file_dialog() ) );
  hlayout->addWidget(button_);

  // Add horizontal layer to verticle layer
  layout->addLayout(hlayout);

  setLayout(layout);
}

// ******************************************************************************************
// Load the file dialog
// ******************************************************************************************
void LoadPathWidget::btn_file_dialog()
{
  QString path;
  if( dir_only_ ) // only allow user to select a directory
  {
    path = QFileDialog::getExistingDirectory(this, "Open Package Directory", path_box_->text(),
                                             QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  }
  else // only allow user to select file
  {
    QString start_path;
    // smart load: open file dialog in location of stack directory
    if( path_box_->text() != "" )
    {
      start_path = path_box_->text();
    }
    else
    {
      // Temp pointers used for casting and accessing parent widget elements
      StartScreenWidget *my_parent = qobject_cast< StartScreenWidget* >( this->parentWidget() );
      LoadPathWidget *my_stack_path = qobject_cast< LoadPathWidget* >( my_parent->stack_path_ );
      LoadPathWidget *my_urdf_path = qobject_cast< LoadPathWidget* >( my_parent->urdf_file_ );

      // Check if the urdf file was already loaded
      if( my_urdf_path->path_box_->text() != "" ) // it has text
      {
        start_path = my_urdf_path->path_box_->text();
      }
      else
      {
        start_path = my_stack_path->path_box_->text();	
      }
    }
    if( load_only_ )
    {
      path = QFileDialog::getOpenFileName(this, "Open File", start_path, "");
    }
    else
    {
      path = QFileDialog::getSaveFileName(this, "Create/Load File", start_path, "" );
    }
  }
  
  // check they did not press cancel
  if (path != NULL)
    path_box_->setText( path );
}

const QString LoadPathWidget::getQPath()
{
  return path_box_->text();
}

const std::string LoadPathWidget::getPath()
{
  return getQPath().toStdString();
}

void LoadPathWidget::setPath( const QString &path )
{
  path_box_->setText( path );
}

