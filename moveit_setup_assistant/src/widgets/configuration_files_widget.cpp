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
// ROS
#include "configuration_files_widget.h"
#include <srdfdom/model.h> // use their struct datastructures
// Boost
#include <boost/algorithm/string.hpp> // for trimming whitespace from user input
#include <boost/filesystem.hpp>  // for creating folders/files

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
                                           "Create the unary stack of configuration files needed to run your robot with MoveIt.",
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
  actions_box_ = new QGroupBox( "Generated Files/Folders:", this );
    
  QHBoxLayout *hlayout2 = new QHBoxLayout();

  // List Box
  action_list_ = new QListWidget( this );
  action_list_->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );
  connect( action_list_, SIGNAL( currentRowChanged(int) ), this, SLOT( changeActionDesc(int) ) );
  hlayout2->addWidget( action_list_ );

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
  hlayout2->addWidget( action_label_ );

  // Add Layout
  actions_box_->setLayout( hlayout2 );
  layout->addWidget( actions_box_ );

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

  // Check that there is at least 1 robot pose
  if( ! config_data_->srdf_->group_states_.size() )
  {
    dependencies << "No robot poses have been added";
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
void ConfigurationFilesWidget::displayAction( const QString title, const QString desc )
{
  action_num++;

  // Programmer error check (because it might be forgotten)
  if( action_num > action_num_total )
    QMessageBox::warning( this, "Programmer Error", "A simple programmer error has occured: increase action_num_total in file configuration_files_widget.h at least by one");

  // Calc percentage
  progress_bar_->setValue( double(action_num)/action_num_total*100 );
  
  // Add actions to list
  action_list_->addItem( title );
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

  // Add a slash at end of path name if it does not currently have one
  size_t slash_pos = new_package_path.find_last_of("/\\");

  // Make sure the last slash is the last character
  if( slash_pos != new_package_path.size() - 1 )
  {
    // Add the slash at end
    new_package_path.append("/"); // TODO: not Windows ready
    
    // Update the textbox
    stack_path_->setPath( new_package_path );
  }

  // Get template package location ----------------------------------------------------------------------
  config_data_->template_package_path_ = config_data_->setup_assistant_path_ + "/templates/moveit_config_pkg_template/";
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

  const std::string setup_assistant_file = new_package_path + ".setup_assistant";

  // Reset the progress bar counter and GUI stuff
  action_num = 0;
  action_list_->clear();
  action_desc_.clear();

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


    // Copy CMakeLists.txt ------------------------------------------------------------------
    std::string file_name = "CMakeLists.txt";
    std::string template_path = config_data_->template_package_path_ + file_name;
    std::string file_path = new_package_path + file_name;

    // Use generic template copy function
    if( !config_data_->copyTemplate( template_path, file_path, new_package_name ) )
    {
      QMessageBox::critical( this, "Error Generating File", 
                             QString("Failed to generate file ").append( file_path.c_str() ));
      return;
    }
    // Feedback
    displayAction( QString( file_name.c_str() ).prepend( qnew_package_name ),
                   "Package that contains all necessary configuration and launch files for MoveIt");

    // Copy Makefile ------------------------------------------------------------------
    file_name = "Makefile";
    template_path = config_data_->template_package_path_ + file_name;
    file_path = new_package_path + file_name;

    // Use generic template copy function
    if( !config_data_->copyTemplate( template_path, file_path, new_package_name ) )
    {
      QMessageBox::critical( this, "Error Generating File", 
                             QString("Failed to generate file ").append( file_path.c_str() ));
      return;
    }
    // Feedback
    displayAction( QString( file_name.c_str() ).prepend( qnew_package_name ),
                   "Package that contains all necessary configuration and launch files for MoveIt");

    // Copy manifest.xml ------------------------------------------------------------------
    file_name = "manifext.xml";
    template_path = config_data_->template_package_path_ + file_name;
    file_path = new_package_path + file_name;

    // Use generic template copy function
    if( !config_data_->copyTemplate( template_path, file_path, new_package_name ) )
    {
      QMessageBox::critical( this, "Error Generating File", 
                             QString("Failed to generate file ").append( file_path.c_str() ));
      return;
    }
    // Feedback
    displayAction( QString( file_name.c_str() ).prepend( qnew_package_name ),
                   "Package that contains all necessary configuration and launch files for MoveIt");

  }

  // Create config folder ---------------------------------------------------------------
  const std::string config_path = new_package_path + "config";
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
                   "Folder for storing configuration files");
  }

  // Create launch folder ---------------------------------------------------------------
  const std::string launch_path = new_package_path + "launch";
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
                   "Folder for storing launch files" );
  }

  // Create SRDF file -----------------------------------------------------------------
  const std::string srdf_file = config_data_->urdf_model_->getName() + ".srdf";
  const std::string srdf_path = config_path + "/" + srdf_file;
  config_data_->srdf_pkg_relative_path_ = "config/" + srdf_file;

  if ( !config_data_->srdf_->writeSRDF( srdf_path ) )
  {
    QMessageBox::critical( this, "Error Generating Files", 
                           QString("Failed to create an SRDF file at location ").append( srdf_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( srdf_file.c_str() ).prepend( qconfig_path ), 
                 "SRDF ( Robot Description Format) is a representation of semantic information about robots. This format is intended to represent information about the robot that is not in the URDF file, but it is useful for a variety of applications. The intention is to include information that has a semantic aspect to it. <a href='http://www.ros.org/wiki/srdf'>http://www.ros.org/wiki/srdf</a>");
  // Select first item in file list for user clue. This may or may not be the srdf file being highlighted
  action_list_->setCurrentRow( 0 );

  // Create OMPL Planning Config File --------------------------------------------------
  const std::string ompl_file = "ompl_planning.yaml";
  const std::string ompl_path = config_path + "/" + ompl_file;

  if ( !config_data_->outputOMPLPlanningYAML( ompl_path ) )
  {
    QMessageBox::critical( this, "Error Generating Files", 
                           QString("Failed to create ompl_planning.yaml file at location ").append( ompl_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( ompl_file.c_str() ).prepend( qconfig_path ), 
                 "Configures the OMPL planning node for your robot. In addition to some basic parameterization for OMPL it contains two groups per planning group - a <group_name> group and a <group_name>_cartesian group. The <group_name> group is configured to support configuration space planning for your groups - sampling in the joint space in order to reach joint goals. The <group_name>_cartesian group is configured to support task space planning for your robot. This is particularly useful as it allows you to specify path constraints on allowable paths, for instance to instruct the robot to keep its end effector upright while moving a glass. The task space planner likely will not successfully plan for your robot in a reasonable amount of time using the auto-configured nodes, as it requires robust and very fast inverse kinematics, generally from an analytic solver." );


  // Create Kinematics Config File -----------------------------------------------------
  const std::string kinematics_file = "kinematics.yaml";
  const std::string kinematics_path = config_path + "/" + kinematics_file;

  if ( !config_data_->outputKinematicsYAML( kinematics_path ) )
  {
    QMessageBox::critical( this, "Error Generating Files", 
                           QString("Failed to create kinematics.yaml file at location ").append( kinematics_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( kinematics_file.c_str() ).prepend( qconfig_path ), 
                 "Holds kinematic info" ); // TODO: description

  // Create Joint Limits Config File -----------------------------------------------------
  const std::string joint_limits_file = "joint_limits.yaml";
  const std::string joint_limits_path = config_path + "/" + joint_limits_file;

  if ( !config_data_->outputJointLimitsYAML( joint_limits_path ) )
  {
    QMessageBox::critical( this, "Error Generating Files", 
                           QString("Failed to create joint_limits.yaml file at location ").append( joint_limits_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( joint_limits_file.c_str() ).prepend( qconfig_path ), 
                 "This file contains additional information about joints that appear in your planning groups that is not contained in the URDF, as well as allowing you to set lower limits for velocity and acceleration than those contained in your URDF. As our planners are exclusively kinematic planners, this information is used by our trajectory filtering system to assign reasonable velocities and timing for the trajectory before it is passed to your controllers." );

  // Create Benchmark_Server Launch File  -----------------------------------------------------
  // TODO: Ioan needs to fix this stuff. Told me to disable it for now
  /*
    const std::string benchmark_server_file = "benchmark_server.launch";
    const std::string benchmark_server_path = launch_path + "/" + benchmark_server_file;

    if ( !config_data_->outputBenchmarkServerLaunch( benchmark_server_path ) )
    {
    QMessageBox::critical( this, "Error Generating Files", 
    QString("Failed to create benchmark_server.launch file at location ").append( benchmark_server_path.c_str() ) );
    return;
    }

    // Feedback
    displayAction( QString( benchmark_server_file.c_str() ).prepend( qlaunch_path ), 
    "Holds benchmark_server info" ); // TODO: description
  */

  // Create Move_Group Launch File  -----------------------------------------------------
  const std::string move_group_file = "move_group.launch";
  const std::string move_group_path = launch_path + "/" + move_group_file;

  if ( !config_data_->outputMoveGroupLaunch( move_group_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", 
                           QString("Failed to create move_group.launch file at location ").append( move_group_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( move_group_file.c_str() ).prepend( qlaunch_path ), 
                 "This launch file just launches all the individual move_<group_name> launch files. You should not need to alter it" ); // TODO: description

  // Create Ompl_Planner Launch File  -----------------------------------------------------
  const std::string ompl_planner_file = "ompl_planner.launch";
  const std::string ompl_planner_path = launch_path + "/" + ompl_planner_file;

  if ( !config_data_->outputOMPLPlannerLaunch( ompl_planner_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", 
                           QString("Failed to create ompl_planner.launch file at location ").append( ompl_planner_path.c_str() ) );
    return;
  }
  // Feedback
  displayAction( QString( ompl_planner_file.c_str() ).prepend( qlaunch_path ), 
                 "TODO" ); // TODO: description

  // Create Planning_Context Launch File  -----------------------------------------------------
  const std::string planning_context_file = "planning_context.launch";
  const std::string planning_context_path = launch_path + "/" + planning_context_file;

  if ( !config_data_->outputPlanningContextLaunch( planning_context_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", 
                           QString("Failed to create planning_context.launch file at location ").append( planning_context_path.c_str() ) );
    return;
  }

  // Feedback
  displayAction( QString( planning_context_file.c_str() ).prepend( qlaunch_path ), 
                 "TODO" ); // TODO: description

  // Create Warehouse Launch File  -----------------------------------------------------
  // TODO: Ioan needs to fix this stuff. Told me to disable it for now
  /*
    const std::string warehouse_file = "warehouse.launch";
    const std::string warehouse_path = launch_path + "/" + warehouse_file;

    if ( !config_data_->outputWarehouseLaunch( warehouse_path ) )
    {
    QMessageBox::critical( this, "Error Generating Files", 
    QString("Failed to create warehouse.launch file at location ").append( warehouse_path.c_str() ) );
    return;
    }

    // Feedback
    displayAction( QString( warehouse_file.c_str() ).prepend( qlaunch_path ), 
    "TODO" ); // TODO: description
  */

  // Create Warehouse_Settings Launch File  -----------------------------------------------------
  // TODO: Ioan needs to fix this stuff. Told me to disable it for now
  /*
    const std::string warehouse_settings_file = "warehouse_settings.launch";
    const std::string warehouse_settings_path = launch_path + "/" + warehouse_settings_file;

    if ( !config_data_->outputWarehouseSettingsLaunch( warehouse_settings_path ) )
    {
    QMessageBox::critical( this, "Error Generating Files", 
    QString("Failed to create warehouse_settings.launch file at location ").append( warehouse_settings_path.c_str() ) );
    return;
    }

    // Feedback
    displayAction( QString( warehouse_settings_file.c_str() ).prepend( qlaunch_path ), 
    "TODO" ); // TODO: description
  */

  // Create Moveit_Visualizer Launch File  -----------------------------------------------------
  const std::string moveit_visualizer_file = "moveit_visualizer.launch";
  const std::string moveit_visualizer_path = launch_path + "/" + moveit_visualizer_file;

  if ( !config_data_->outputMoveItVisualizerLaunch( moveit_visualizer_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", 
                           QString("Failed to create moveit_visualizer.launch file at location ").append( moveit_visualizer_path.c_str() ) );
    return;
  }

  // Feedback
  displayAction( QString( moveit_visualizer_file.c_str() ).prepend( qlaunch_path ), 
                 "TODO" ); // TODO: description

  // Create Setup Assistant Launch File  -----------------------------------------------------
  const std::string setup_assistant_launch_file = "setup_assistant.launch";
  const std::string setup_assistant_launch_path = launch_path + "/" + setup_assistant_launch_file;

  if ( !config_data_->outputSetupAssistantLaunch( setup_assistant_launch_path, new_package_name ) )
  {
    QMessageBox::critical( this, "Error Generating Files", 
                           QString("Failed to create setup_assistant.launch file at location ")
                           .append( setup_assistant_launch_path.c_str() ) );
    return;
  }

  // Feedback
  displayAction( QString( setup_assistant_launch_file.c_str() ).prepend( qlaunch_path ), 
                 "Launch file for easily re-starting the MoveIt Setup Assistant to edit this generated configuration package." );

  // Create setup assistant file --------------------------------------------------------
  const std::string hidden_file = ".setup_assistant";
  const std::string hidden_path = new_package_path + hidden_file;

  if ( !config_data_->outputSetupAssistantFile( hidden_path ) )
  {
    QMessageBox::critical( this, "Error Generating Files", 
                           QString("Failed to create an .setup_assistant file at location ").append( hidden_path.c_str() ) );
    return;
  }

  // Feedback
  displayAction( QString( hidden_file.c_str() ).prepend( qnew_package_name ), 
                 "MoveIt Setup Assistant hidden settings file. Do not edit." );
  

  // Alert user it completed successfully --------------------------------------------------
  progress_bar_->setValue( 100 );
  //QMessageBox::information( this, "Complete", "All MoveIt configuration files generated successfully!" );
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
  std::string package_name;

  size_t found_index = package_path.find_last_of("/\\");
  //std::cout << found_index << std::endl;

  // Check if last character is a slash
  if( found_index == package_path.size() - 1 ) // get second to last
  {
    std::string sub_path = package_path.substr( 0, package_path.size() - 1);
    //std::cout << "substring " << sub_path << std::endl;

    found_index = sub_path.find_last_of("/\\");
    package_name = sub_path.substr( found_index + 1, sub_path.size() -1 );
  }
  else // get substring only
  {
    package_name = package_path.substr( found_index + 1, package_path.size() - 1 );
  }

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

} // namespace


