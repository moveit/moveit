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
#include <QVBoxLayout>
#include <QPushButton>
#include <QMessageBox>
#include <QApplication>
#include <QSplitter>
// ROS
#include "configuration_files_widget.h"
#include <srdfdom/model.h> // use their struct datastructures
#include <ros/ros.h>
// Boost
#include <boost/algorithm/string.hpp> // for trimming whitespace from user input
#include <boost/filesystem.hpp>  // for creating folders/files
// Read write files
#include <iostream> // For writing yaml and launch files
#include <fstream>

namespace moveit_setup_assistant
{

// Boost file system
namespace fs = boost::filesystem;

// ******************************************************************************************
// Outer User Interface for MoveIt Configuration Assistant
// ******************************************************************************************
ConfigurationFilesWidget::ConfigurationFilesWidget( QWidget *parent, moveit_setup_assistant::MoveItConfigDataPtr config_data )
  : SetupScreenWidget( parent ), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout();

  // Top Header Area ------------------------------------------------

  HeaderWidget *header = new HeaderWidget( "Generate Configuration Files",
                                           "Create or update the configuration files package needed to run your robot with MoveIt. Generated files highlighted orange indicate they were skipped.",
                                           this);
  layout->addWidget( header );

  // Path Widget ----------------------------------------------------

  // Stack Path Dialog
  stack_path_ = new LoadPathWidget("Configuration Package Save Path",
                                   "Specify the desired directory for the MoveIt configuration package to be generated. Overwriting an existing configuration package directory is acceptable. Example: <i>/u/robot/ros/pr2_moveit_config</i>",
                                   true, this); // is directory
  layout->addWidget( stack_path_ );

  // Pass the package path from start screen to configuration files screen
  stack_path_->setPath( config_data_->config_pkg_path_ );


  // Save buttons ---------------------------------------------------
  QHBoxLayout *hlayout1 = new QHBoxLayout();

  // Progress Bar
  progress_bar_ = new QProgressBar( this );
  progress_bar_->setMaximum(100);
  progress_bar_->setMinimum(0);
  hlayout1->addWidget(progress_bar_);
  hlayout1->setContentsMargins( 20, 30, 20, 30 );

  // Generate Package Button
  btn_save_ = new QPushButton("&Generate Package", this);
  btn_save_->setMinimumWidth(180);
  btn_save_->setMinimumHeight(40);
  connect( btn_save_, SIGNAL( clicked() ), this, SLOT( savePackage() ) );
  hlayout1->addWidget( btn_save_ );

  // Add Layout
  layout->addLayout( hlayout1 );

  // Generated Files List -------------------------------------------
  QLabel* generated_list = new QLabel( "Generated Files/Folders:", this );
  layout->addWidget( generated_list );

  QSplitter* splitter = new QSplitter( Qt::Horizontal, this );
  splitter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  // List Box
  action_list_ = new QListWidget( this );
  action_list_->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );
  connect( action_list_, SIGNAL( currentRowChanged(int) ), this, SLOT( changeActionDesc(int) ) );

  // Description
  action_label_ = new QLabel( this );
  action_label_->setFrameShape(QFrame::StyledPanel);
  action_label_->setFrameShadow(QFrame::Raised);
  action_label_->setLineWidth(1);
  action_label_->setMidLineWidth(0);
  action_label_->setWordWrap(true);
  action_label_->setSizePolicy( QSizePolicy::Preferred, QSizePolicy::Expanding );
  action_label_->setMinimumWidth( 100 );
  action_label_->setAlignment( Qt::AlignTop );
  action_label_->setOpenExternalLinks(true); // open with web browser

  // Add to splitter
  splitter->addWidget( action_list_ );
  splitter->addWidget( action_label_ );

  // Add Layout
  layout->addWidget( splitter );

  // Bottom row --------------------------------------------------

  QHBoxLayout *hlayout3 = new QHBoxLayout();

  // Success label
  success_label_ = new QLabel( this );
  QFont success_label_font( "Arial", 12, QFont::Bold );
  success_label_->setFont( success_label_font );
  success_label_->hide(); // only show once the files have been generated
  success_label_->setText(  "Configuration package generated successfully!" );
  hlayout3->addWidget( success_label_ );
  hlayout3->setAlignment( success_label_, Qt::AlignRight );

  // Exit button
  QPushButton *btn_exit = new QPushButton( "E&xit Setup Assistant", this );
  btn_exit->setMinimumWidth(180);
  connect( btn_exit, SIGNAL( clicked() ), this, SLOT( exitSetupAssistant() ) );
  hlayout3->addWidget( btn_exit );
  hlayout3->setAlignment( btn_exit, Qt::AlignRight );

  layout->addLayout( hlayout3 );

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);

}

// ******************************************************************************************
// Verify with user if certain screens have not been completed
// ******************************************************************************************
bool ConfigurationFilesWidget::checkDependencies()
{
  QStringList dependencies;

  // Check that at least 1 planning group exists
  if( ! config_data_->srdf_->groups_.size() )
  {
    dependencies << "No robot model planning groups have been created";
  }

  // Check that at least 1 link pair is disabled from collision checking
  if( ! config_data_->srdf_->disabled_collisions_.size() )
  {
    dependencies << "No self-collisions have been disabled";
  }

  // Check that there is at least 1 end effector added
  if( ! config_data_->srdf_->end_effectors_.size() )
  {
    dependencies << "No end effectors have been added";
  }

  // Check that there is at least 1 virtual joint added
  if( ! config_data_->srdf_->virtual_joints_.size() )
  {
    dependencies << "No virtual joints have been added";
  }

  // Display all accumumlated errors:
  if( dependencies.size() )
  {
    // Create a dependency message
    QString dep_message = "There are incomplete steps in this setup assistant. You probably want to complete the following steps before generating a MoveIt configuration package:<br /><ul>";

    for (int i = 0; i < dependencies.size(); ++i)
    {
      dep_message.append("<li>").append(dependencies.at(i)).append("</li>");
    }
    dep_message.append("</ul><br/>Press Ok to continue generating files.");

    if( QMessageBox::question( this, "Incomplete MoveIt Setup Assistant Steps", dep_message,
                               QMessageBox::Ok | QMessageBox::Cancel)
        == QMessageBox::Cancel )
    {
      return false; // abort
    }
  }

  return true;
}

// ******************************************************************************************
// A function for showing progress and user feedback about what happened
// ******************************************************************************************
void ConfigurationFilesWidget::displayAction( const QString title, const QString desc,
                                              bool skipped )
{
  action_num++;

  // Programmer error check (because it might be forgotten)
  if( action_num > action_num_total )
    QMessageBox::warning( this, "Programmer Error", "A simple programmer error has occured: increase action_num_total in file configuration_files_widget.h at least by one");

  // Calc percentage
  progress_bar_->setValue( double(action_num)/action_num_total*100 );

  // Create a formatted row
  QListWidgetItem *item = new QListWidgetItem( title, action_list_, 0 );

  if( skipped )
  {
    item->setForeground( QBrush(QColor(255, 135, 0)));
  }

  // Add actions to list
  action_list_->addItem( item );
  action_desc_.append( desc );

  // allow the progress bar to be shown
  QApplication::processEvents();
}

// ******************************************************************************************
// Display the selected action in the desc box
// ******************************************************************************************
void ConfigurationFilesWidget::changeActionDesc(int id)
{
  // Only allow event if list is not empty
  if( id >= 0 )
  {
    // Show the selected text
    action_label_->setText( action_desc_.at(id) );
  }
}

// ******************************************************************************************
// Save package using default path
// ******************************************************************************************
void ConfigurationFilesWidget::savePackage()
{
  // Feedback
  success_label_->hide();

  // Get path name
  std::string new_package_path = stack_path_->getPath();
  
  // Use these strings multiple times
  QString skipped_pkg_msg = QString("<b>This file was not generated because it would over-write a user customized file with a blank template file. If this is the desired action then first manually delete the file and then re-generate your package by clicking the 'Generate Package' button.</b><br /><br />");
  QString pkg_description;

  // Check that a valid stack package name has been given
  if( new_package_path.empty() )
  {
    QMessageBox::warning( this, "Error Generating", "No package path provided. Please choose a directory location to generate the MoveIt configuration files." );
    return;
  }

  // Check setup assist deps
  if( !checkDependencies() )
    return; // canceled

  // Check that all groups have components
  if( !noGroupsEmpty() )
    return; // not ready

  // Trim whitespace from user input
  boost::trim( new_package_path );

  // Get template package location ----------------------------------------------------------------------
  fs::path template_package_path = config_data_->setup_assistant_path_;
  template_package_path /= "templates";
  template_package_path /= "moveit_config_pkg_template";
  config_data_->template_package_path_ = template_package_path.make_preferred().native().c_str();

  if( !fs::is_directory( config_data_->template_package_path_ ) )
  {
    QMessageBox::critical( this, "Error Generating",
                           QString("Unable to find package template directory: ")
                           .append( config_data_->template_package_path_.c_str() ) );
    return;
  }

  // Get the package name ---------------------------------------------------------------------------------
  const std::string new_package_name = getPackageName( new_package_path );
  QString qnew_package_name = QString( new_package_name.c_str() ).append("/"); // for gui feedback

  const std::string setup_assistant_file = config_data_->appendPaths( new_package_path, ".setup_assistant" );

  // Reset the progress bar counter and GUI stuff
  action_num = 0;
  action_list_->clear();
  action_desc_.clear();

  // Reused variables for copying
  std::string file_name;
  std::string file_path;
  std::string template_path;

  // Make sure old package is correct package type and verify over write
  if( fs::is_directory( new_package_path ) && !fs::is_empty( new_package_path ) )
  {

    // Check if the old package is a setup assistant package. If it is not, quit
    if( ! fs::is_regular_file( setup_assistant_file ) )
    {
      QMessageBox::warning( this, "Incorrect Folder/Package",
                            QString("The chosen package location already exists but was not previously created using this MoveIt Setup Assistant. If this is a mistake, replace the missing file: ")
                            .append( setup_assistant_file.c_str() ) );
      return;
    }

    // Confirm overwrite
    if( QMessageBox::question( this, "Confirm Package Update",
                               QString("Are you sure you want to overwrite this existing package with updated configurations?<br /><i>")
                               .append( new_package_path.c_str() )
                               .append( "</i>" ),
                               QMessageBox::Ok | QMessageBox::Cancel)
        == QMessageBox::Cancel )
    {
      return; // abort
    }

  }
  else // this is a new package (but maybe the folder already exists)
  {
    // Create new directory ------------------------------------------------------------------
    try
    {
      fs::create_directory( new_package_path ) && !fs::is_directory( new_package_path );
    }
    catch( ... )
    {
      QMessageBox::critical( this, "Error Generating Files",
                             QString("Unable to create directory ").append( new_package_path.c_str() ) );
      return;
    }
    // Feedback
    displayAction( qnew_package_name,
                   "Package that contains all necessary configuration and launch files for MoveIt");

    // Copy package.xml ------------------------------------------------------------------
    // Note: we call the file package.xml.disabled so that it isn't automatically indexed by rosprofile
    // in the scenario where we want to disabled the setup_assistant by renaming its root package.xml
    file_name = "package.xml";
    template_path = config_data_->appendPaths( config_data_->template_package_path_, "package.xml.template" );
    file_path = config_data_->appendPaths( new_package_path, file_name );

    // Use generic template copy function
    if( !copyTemplate( template_path, file_path, new_package_name ) )
    {
      QMessageBox::critical( this, "Error Generating File",
                             QString("Failed to generate file ").append( file_path.c_str() ));
      return;
    }
    // Feedback
    displayAction( QString( file_name.c_str() ).prepend( qnew_package_name ),
                   "Required ROS package meta data file.");

    // Copy CMakeLists.txt ------------------------------------------------------------------
    file_name = "CMakeLists.txt";
    template_path = config_data_->appendPaths( config_data_->template_package_path_, file_name );
    file_path = config_data_->appendPaths( new_package_path, file_name );

    // Use generic template copy function
    if( !copyTemplate( template_path, file_path, new_package_name ) )
    {
      QMessageBox::critical( this, "Error Generating File",
                             QString("Failed to generate file ").append( file_path.c_str() ));
      return;
    }
    // Feedback
    displayAction( QString( file_name.c_str() ).prepend( qnew_package_name ),
                   "CMake build system.");
  }

  // Create config folder ---------------------------------------------------------------
  const std::string config_path = config_data_->appendPaths( new_package_path, "config" );
  QString qconfig_path = QString("config/").prepend( qnew_package_name );

  if( !fs::is_directory( config_path ) )
  {
    if ( !fs::create_directory( config_path ) )
    {
      QMessageBox::critical( this, "Error Generating Files",
                             QString("Unable to create directory ").append( config_path.c_str() ) );
      return;
    }
    // Feedback
    displayAction( qconfig_path,
                   "Subfolder containing necessary MoveIt configuration files");
  }

  // Create launch folder ---------------------------------------------------------------
  const std::string launch_path = config_data_->appendPaths( new_package_path, "launch" );
  QString qlaunch_path = QString("launch/").prepend( qnew_package_name );

  if( !fs::is_directory( launch_path ) )
  {
    if( !fs::create_directory( launch_path ) )
    {
      QMessageBox::critical( this, "Error Generating Files",
                             QString("Unable to create directory ").append( launch_path.c_str() ) );
      return;
    }
    // Feedback
    displayAction( qlaunch_path,
                   "Subfolder containing MoveIt launch files");
  }

  // -------------------------------------------------------------------------------------------------------------------
  // CONIG FILES -------------------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------------------------------

  // Create SRDF file -----------------------------------------------------------------
  const std::string srdf_file = config_data_->urdf_model_->getName() + ".srdf";
  const std::string srdf_path = config_data_->appendPaths( config_path, srdf_file );
  config_data_->srdf_pkg_relative_path_ = "config/" + srdf_file;

  if ( !config_data_->srdf_->writeSRDF( srdf_path ) )
  {
    QMessageBox::critical( this, "Error Generating Files",
                           QString("Failed to create an SRDF file at location ").append( srdf_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( srdf_file.c_str() ).prepend( qconfig_path ),
                 "SRDF (<a href='http://www.ros.org/wiki/srdf'>Semantic Robot Description Format</a>) is a representation of semantic information about robots. This format is intended to represent information about the robot that is not in the URDF file, but it is useful for a variety of applications. The intention is to include information that has a semantic aspect to it.");
  // Select first item in file list for user clue. This may or may not be the srdf file being highlighted
  action_list_->setCurrentRow( 0 );

  // Create OMPL Planning Config File --------------------------------------------------
  const std::string ompl_file = "ompl_planning.yaml";
  const std::string ompl_path = config_data_->appendPaths( config_path, ompl_file );

  if ( !config_data_->outputOMPLPlanningYAML( ompl_path ) )
  {
    QMessageBox::critical( this, "Error Generating Files",
                           QString("Failed to create ompl_planning.yaml file at location ")
                           .append( ompl_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( ompl_file.c_str() ).prepend( qconfig_path ),
                 "Configures the OMPL (<a href='http://ompl.kavrakilab.org/'>Open Motion Planning Library</a>) planning plugin. For every planning group defined in the SRDF, a number of planning configurations are specified (under planner_configs). Additionally, default settings for the state space to plan in for a particular group can be specified, such as the collision checking resolution. Each planning configuration specified for a group must be defined under the planner_configs tag. While defining a planner configuration, the only mandatory parameter is 'type', which is the name of the motion planner to be used. Any other planner-specific parameters can be defined but are optional.");

  // Create Kinematics Config File -----------------------------------------------------
  const std::string kinematics_file = "kinematics.yaml";
  const std::string kinematics_path = config_data_->appendPaths( config_path, kinematics_file );

  if ( !config_data_->outputKinematicsYAML( kinematics_path ) )
  {
    QMessageBox::critical( this, "Error Generating Files",
                           QString("Failed to create kinematics.yaml file at location ")
                           .append( kinematics_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( kinematics_file.c_str() ).prepend( qconfig_path ),
                 "Specifies which kinematic solver plugin to use for each planning group in the SRDF, as well as the kinematic solver search resolution.");

  // Create Joint Limits Config File -----------------------------------------------------
  const std::string joint_limits_file = "joint_limits.yaml";
  const std::string joint_limits_path = config_data_->appendPaths( config_path, joint_limits_file );

  if ( !config_data_->outputJointLimitsYAML( joint_limits_path ) )
  {
    QMessageBox::critical( this, "Error Generating Files",
                           QString("Failed to create joint_limits.yaml file at location ")
                           .append( joint_limits_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( joint_limits_file.c_str() ).prepend( qconfig_path ),
                 "Contains additional information about joints that appear in your planning groups that is not contained in the URDF, as well as allowing you to set maximum and minimum limits for velocity and acceleration than those contained in your URDF. This information is used by our trajectory filtering system to assign reasonable velocities and timing for the trajectory before it is passed to the robots controllers.");


  // -------------------------------------------------------------------------------------------------------------------
  // LAUNCH FILES ------------------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------------------------------
  const std::string template_launch_path = config_data_->appendPaths( config_data_->template_package_path_, "launch" );
  const std::string robot_name = config_data_->srdf_->robot_name_;

  // Create Move_Group Launch File  -----------------------------------------------------
  file_name = "move_group.launch";
  file_path = config_data_->appendPaths( launch_path, file_name );
  template_path = config_data_->appendPaths( template_launch_path, file_name );
  // Use generic template copy function
  if ( !copyTemplate( template_path, file_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", QString("Failed to create ").append( file_name.c_str() )
                           .append( " file at location " ).append( file_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ),
                 "Launches the move_group node that provides the MoveGroup action and other parameters <a href='http://moveit.ros.org/move_group.html'>MoveGroup action</a>");


  // Create Planning_Context Launch File  -----------------------------------------------------
  file_name = "planning_context.launch";
  file_path = config_data_->appendPaths( launch_path, file_name );
  template_path = config_data_->appendPaths( template_launch_path, file_name );
  // Use generic template copy function
  if ( !copyTemplate( template_path, file_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", QString("Failed to create ").append( file_name.c_str() )
                           .append( " file at location " ).append( file_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ),
                 "Loads settings for the ROS parameter server, required for running MoveIt. This includes the SRDF, joints_limits.yaml file, ompl_planning.yaml file, optionally the URDF, etc");


  // Create Moveit_Visualizer Launch File  -----------------------------------------------------
  file_name = "moveit_rviz.launch";
  file_path = config_data_->appendPaths( launch_path, file_name );
  template_path = config_data_->appendPaths( template_launch_path,  file_name );
  // Use generic template copy function
  if ( !copyTemplate( template_path, file_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", QString("Failed to create ").append( file_name.c_str() )
                           .append( " file at location " ).append( file_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ),
                 "Visualize in Rviz the robot's planning groups running with interactive markers that allow goal states to be set.");


  // Create Ompl_Planning_Pipeline Launch File  -----------------------------------------------------
  file_name = "ompl_planning_pipeline.launch";
  file_path = config_data_->appendPaths( launch_path, file_name );
  template_path = config_data_->appendPaths( template_launch_path, file_name );
  // Use generic template copy function
  if ( !copyTemplate( template_path, file_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", QString("Failed to create ").append( file_name.c_str() )
                           .append( " file at location " ).append( file_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ),
                 "Intended to be included in other launch files that require the OMPL planning plugin. Defines the proper plugin name on the parameter server and a default selection of planning request adapters.");


  // Create Planning_Pipeline Launch File  -----------------------------------------------------
  file_name = "planning_pipeline.launch";
  file_path = config_data_->appendPaths( launch_path, file_name );
  template_path = config_data_->appendPaths( template_launch_path, file_name );
  // Use generic template copy function
  if ( !copyTemplate( template_path, file_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", QString("Failed to create ").append( file_name.c_str() )
                           .append( " file at location " ).append( file_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ),
                 "Helper launch file that can choose between different planning pipelines to be loaded.");

  // Create warehouse_settings Launch File  -----------------------------------------------------
  file_name = "warehouse_settings.launch";
  file_path = config_data_->appendPaths( launch_path, file_name );
  template_path = config_data_->appendPaths( template_launch_path, file_name );
  // Use generic template copy function
  if ( !copyTemplate( template_path, file_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", QString("Failed to create ").append( file_name.c_str() )
                           .append( " file at location " ).append( file_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ),
                 "Helper launch file that specifies default settings for MongoDB.");


  // Create warehouse Launch File  -----------------------------------------------------
  file_name = "warehouse.launch";
  file_path = config_data_->appendPaths( launch_path, file_name );
  template_path = config_data_->appendPaths( template_launch_path, file_name );
  // Use generic template copy function
  if ( !copyTemplate( template_path, file_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", QString("Failed to create ").append( file_name.c_str() )
                           .append( " file at location " ).append( file_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ),
                 "Launch file for starting MongoDB.");

  // Create warehouse Launch File  -----------------------------------------------------
  file_name = "run_benchmark_server_ompl.launch";
  file_path = config_data_->appendPaths( launch_path, file_name );
  template_path = config_data_->appendPaths( template_launch_path, file_name );
  // Use generic template copy function
  if ( !copyTemplate( template_path, file_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", QString("Failed to create ").append( file_name.c_str() )
                           .append( " file at location " ).append( file_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ),
                 "Launch file for benchmarking OMPL planners");


  // Create Planning_Pipeline Launch File  -----------------------------------------------------
  file_name = "sensor_manager.launch";
  file_path = config_data_->appendPaths( launch_path, file_name );
  template_path = config_data_->appendPaths( template_launch_path, file_name );
  // Use generic template copy function
  if ( !copyTemplate( template_path, file_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", QString("Failed to create ").append( file_name.c_str() )
                           .append( " file at location " ).append( file_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ),
                 "Helper launch file that can choose between different sensor managers to be loaded.");

  // Create Moveit_Controller_Manager Launch File  -----------------------------------------------------
  file_name = robot_name + "_moveit_controller_manager.launch";
  file_path = config_data_->appendPaths( launch_path, file_name );
  template_path = config_data_->appendPaths( template_launch_path, "moveit_controller_manager.launch" );
  pkg_description = "Placeholder for settings specific to the MoveIt controller manager implemented for you robot.";

  // Do not overwrite file if it already exists - user might have customized it
  if( fs::is_regular_file( file_path ) )
  {
    // Skipped feedback
    displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ), pkg_description.prepend(skipped_pkg_msg), true );
  }
  else
  {
    // Use generic template copy function
    if ( !copyTemplate( template_path, file_path, new_package_name ) )
    {
      QMessageBox::critical( this, "Error Generating Files", QString("Failed to create ").append( file_name.c_str() )
                             .append( " file at location " ).append( file_path.c_str() ) );
      return;
    }
    // Feedback
    displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ), pkg_description );
  }

  // Create Moveit_Sensor_Manager Launch File  -----------------------------------------------------
  file_name = robot_name + "_moveit_sensor_manager.launch";
  file_path = config_data_->appendPaths( launch_path, file_name );
  template_path = config_data_->appendPaths( template_launch_path, "moveit_sensor_manager.launch" );
  pkg_description = "Placeholder for settings specific to the MoveIt sensor manager implemented for you robot.";

  // Do not overwrite file if it already exists - user might have customized it
  if( fs::is_regular_file( file_path ) )
  {
    // Skipped feedback
    displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ), pkg_description.prepend(skipped_pkg_msg), true );
  }
  else
  {
    // Use generic template copy function
    if ( !copyTemplate( template_path, file_path, new_package_name ) )
    {
      QMessageBox::critical( this, "Error Generating Files", QString("Failed to create ").append( file_name.c_str() )
                             .append( " file at location " ).append( file_path.c_str() ) );
      return;
    }
    // Feedback
    displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ), pkg_description );
  }

  // Create Trajectory_Execution Launch File  -----------------------------------------------------
  file_name = "trajectory_execution.launch";
  file_path = config_data_->appendPaths( launch_path, file_name );
  template_path = config_data_->appendPaths( template_launch_path, file_name );
  // Use generic template copy function
  if ( !copyTemplate( template_path, file_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", QString("Failed to create ").append( file_name.c_str() )
                           .append( " file at location " ).append( file_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ),
                 "Loads settings for the ROS parameter server required for executing trajectories using the trajectory_execution_manager::TrajectoryExecutionManager.");

  // Create Demo Launch File  -----------------------------------------------------
  file_name = "demo.launch";
  file_path = config_data_->appendPaths( launch_path, file_name );
  template_path = config_data_->appendPaths( template_launch_path, file_name );
  // Use generic template copy function
  if ( !copyTemplate( template_path, file_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", QString("Failed to create ").append( file_name.c_str() )
                           .append( " file at location " ).append( file_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ),
                 "Run a demo of MoveIt.");


  // Create Setup_Assistant Launch File  -----------------------------------------------------
  file_name = "setup_assistant.launch";
  file_path = config_data_->appendPaths( launch_path, file_name );
  template_path = config_data_->appendPaths( template_launch_path, "edit_configuration_package.launch" );
  // Use generic template copy function
  if ( !copyTemplate( template_path, file_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", QString("Failed to create ").append( file_name.c_str() )
                           .append( " file at location " ).append( file_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( file_name.c_str() ).prepend( qlaunch_path ),
                 "Launch file for easily re-starting the MoveIt Setup Assistant to edit this robot's generated configuration package.");


  // -------------------------------------------------------------------------------------------------------------------
  // OTHER FILES -------------------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------------------------------


  // Create setup assistant file --------------------------------------------------------
  const std::string hidden_file = ".setup_assistant";
  const std::string hidden_path = config_data_->appendPaths( new_package_path, hidden_file );

  if ( !config_data_->outputSetupAssistantFile( hidden_path ) )
  {
    QMessageBox::critical( this, "Error Generating Files",
                           QString("Failed to create an .setup_assistant file at location ").append( hidden_path.c_str() ) );
    return;
  }

  // Feedback
  displayAction( QString( hidden_file.c_str() ).prepend( qnew_package_name ),
                 "MoveIt Setup Assistant hidden settings file. You should not need to edit this file.");


  // Alert user it completed successfully --------------------------------------------------
  progress_bar_->setValue( 100 );
  success_label_->show();
}


// ******************************************************************************************
// Quit the program because we are done
// ******************************************************************************************
void ConfigurationFilesWidget::exitSetupAssistant()
{
  if( QMessageBox::question( this, "Exit Setup Assistant",
                             QString("Are you sure you want to exit the MoveIt Setup Assistant?"),
                             QMessageBox::Ok | QMessageBox::Cancel)
      == QMessageBox::Ok )
  {
    QApplication::quit();
  }
}

// ******************************************************************************************
// Get the last folder name in a directory path
// ******************************************************************************************
const std::string ConfigurationFilesWidget::getPackageName( std::string package_path )
{
  // Remove end slash if there is one
  if( !package_path.compare( package_path.size() - 1, 1, "/" ) ) // || package_path[ package_path.size() - 1 ] == "\\" )
  {
    package_path = package_path.substr( 0, package_path.size() - 1 );
  }

  // Get the last directory name
  std::string package_name;
  fs::path fs_package_path = package_path;

  package_name = fs_package_path.filename().c_str();

  // check for empty
  if( package_name.empty() )
    package_name = "unknown";

  return package_name;
}

// ******************************************************************************************
// Check that no group is empty (without links/joints/etc)
// ******************************************************************************************
bool ConfigurationFilesWidget::noGroupsEmpty()
{
  // Loop through all groups
  for( std::vector<srdf::Model::Group>::const_iterator group_it = config_data_->srdf_->groups_.begin();
       group_it != config_data_->srdf_->groups_.end();  ++group_it )
  {
    // Whenever 1 of the 4 component types are found, stop checking this group
    if( group_it->joints_.size() )
      continue;
    if( group_it->links_.size() )
      continue;
    if( group_it->chains_.size() )
      continue;
    if( group_it->subgroups_.size() )
      continue;

    // This group has no contents, bad
    QMessageBox::warning( this, "Empty Group",
                          QString("The planning group '").append( group_it->name_.c_str() )
                          .append("' is empty and has no subcomponents associated with it (joints/links/chains/subgroups). You must edit or remove this planning group before this configuration package can be saved.") );
    return false;
  }

  return true; // good
}

// ******************************************************************************************
// Copy a template from location <template_path> to location <output_path> and replace package name
// ******************************************************************************************
bool ConfigurationFilesWidget::copyTemplate( const std::string& template_path, const std::string& output_path,
                                             const std::string& new_package_name )
{
  // Error check file
  if( ! fs::is_regular_file( template_path ) )
  {
    ROS_ERROR_STREAM( "Unable to find template file " << template_path );
    return false;
  }

  // Load file
  std::ifstream template_stream( template_path.c_str() );
  if( !template_stream.good() ) // File not found
  {
    ROS_ERROR_STREAM( "Unable to load file " << template_path );
    return false;
  }

  // Load the file to a string using an efficient memory allocation technique
  std::string template_string;
  template_stream.seekg(0, std::ios::end);
  template_string.reserve(template_stream.tellg());
  template_stream.seekg(0, std::ios::beg);
  template_string.assign( (std::istreambuf_iterator<char>(template_stream)), std::istreambuf_iterator<char>() );
  template_stream.close();

  // Replace keywords in string ------------------------------------------------------------
  boost::replace_all( template_string, "[GENERATED_PACKAGE_NAME]", new_package_name );

  if (config_data_->urdf_pkg_name_.empty())
  {
    boost::replace_all( template_string, "[URDF_LOCATION]", config_data_->urdf_path_ );
  }
  else
  {
    boost::replace_all( template_string, "[URDF_LOCATION]", "$(find " + config_data_->urdf_pkg_name_ + ")/" + config_data_->urdf_pkg_relative_path_);
  }

  boost::replace_all( template_string, "[ROBOT_NAME]", config_data_->srdf_->robot_name_ );

  std::stringstream vjb;
  for (std::size_t i = 0 ; i < config_data_->srdf_->virtual_joints_.size(); ++i)
  {
    const srdf::Model::VirtualJoint &vj = config_data_->srdf_->virtual_joints_[i];
    if (vj.type_ != "fixed")
      vjb << "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\"virtual_joint_broadcaster_" << i << "\" args=\"0 0 0 0 0 0 " << vj.parent_frame_ << " " << vj.child_link_ << " 100\" />" << std::endl;
  }
  
  boost::replace_all ( template_string, "[VIRTUAL_JOINT_BROADCASTER]", vjb.str());
  
  // Save string to new location -----------------------------------------------------------
  std::ofstream output_stream( output_path.c_str(), std::ios_base::trunc );
  if( !output_stream.good() )
  {
    ROS_ERROR_STREAM( "Unable to open file for writing " << output_path );
    return false;
  }

  output_stream << template_string.c_str();
  output_stream.close();

  return true; // file created successfully
}



} // namespace

