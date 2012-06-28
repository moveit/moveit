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

#include <QVBoxLayout>
#include <QPushButton>
#include <QMessageBox>
#include <QApplication>
#include "configuration_files_widget.h"
#include <boost/filesystem.hpp>  // for creating folders/files

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
  stack_path_ = new LoadPathWidget("MoveIt Configuration Package Generation Path", 
                                   "Specify the desired directory for the MoveIt configuration package to be generated. Choosing an existing configuration package directory to overwrite is fine. <br/>Example: <i>~/ros/pr2_moveit_config</i>",
                                   true, this); // is directory
  layout->addWidget( stack_path_ );
  
  // Save buttons ---------------------------------------------------
  QHBoxLayout *hlayout = new QHBoxLayout();

  // Generate Package
  btn_save_ = new QPushButton("&Generate Package", this);
  //  btn_save_->setMinimumWidth(180);
  //  btn_save_->setMinimumHeight(40);
  connect( btn_save_, SIGNAL( clicked() ), this, SLOT( savePackage() ) );
  hlayout->addWidget( btn_save_ );  

  // Progress Bar
  progress_bar_ = new QProgressBar( this );
  progress_bar_->setMaximum(100);
  progress_bar_->setMinimum(0);
  hlayout->addWidget(progress_bar_);
  hlayout->setContentsMargins( 20, 50, 20, 50 );

  // Add Layout
  layout->addLayout( hlayout );

  // Generated Files List -------------------------------------------
  actions_box_ = new QGroupBox( "Generated Files:", this );
    
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
  action_label_->setMinimumWidth( 300 );
  action_label_->setAlignment( Qt::AlignTop );
  action_label_->setOpenExternalLinks(true); // open with web browser
  hlayout2->addWidget( action_label_ );

  // Add Layout
  actions_box_->setLayout( hlayout2 );
  layout->addWidget( actions_box_ );

  // Bottom button --------------------------------------------------
  QPushButton *btn_exit = new QPushButton( "E&xit Setup Assistant", this );
  btn_exit->setMinimumWidth(180);
  connect( btn_exit, SIGNAL( clicked() ), this, SLOT( exitSetupAssistant() ) );
  layout->addWidget( btn_exit );
  layout->setAlignment( btn_exit, Qt::AlignRight );  
  
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
  // Show the selected text
  action_label_->setText( action_desc_.at(id) );
}

// ******************************************************************************************
// Save package using default path
// ******************************************************************************************
void ConfigurationFilesWidget::savePackage()
{
  // Check setup assist deps
  if( !checkDependencies() )
    return; // canceled

  // File system
  namespace fs = boost::filesystem;
  
  const std::string stack_path = stack_path_->getPath();

  // Check that a valid stack package name has been given --------------------------------------------------
  if( stack_path.empty() )
  {
    QMessageBox::warning( this, "Error Generating", "No package path provided. Please choose a folder location to generate the MoveIt configuration files." );
    return;
  }

  // Get the package name ---------------------------------------------------------------------------------
  std::string package_name = getPackageName( stack_path );
  QString qpackage_name = QString( package_name.c_str() ).append("/"); // for gui feedback


  // Reset the progress bar counter and GUI stuff
  action_num = 0;
  action_list_->clear();
  action_desc_.clear();

  // Verify with user the desire to overwrite old package--------------------------------------------------
  if( fs::is_directory( stack_path ) )
  {
    if( QMessageBox::question( this, "Confirm Directory Overwrite", 
                               QString("Are you sure you want to permanently overwrite this existing folder?<br /><i>")
                               .append( stack_path.c_str() )
                               .append( "</i>" ),
                               QMessageBox::Ok | QMessageBox::Cancel) 
        == QMessageBox::Cancel )
    {
      return; // abort
    }

    // Now delete all contents in old directory
    if( !fs::remove_all( stack_path ) )
    {
      QMessageBox::critical( this, "Error Generating Files", 
                             QString("Unable to delete old directory ").append( stack_path.c_str() ) );
      return;
    }
  }

  // Create new directory
  if ( !fs::create_directory( stack_path ))
  {
    QMessageBox::critical( this, "Error Generating Files", 
                           QString("Unable to create directory ").append( stack_path.c_str() ) );
    return;
  }

  // Create config folder
  const std::string config_path = stack_path + "/config";
  QString qconfig_path = QString("config/").prepend( qpackage_name );

  if ( !fs::create_directory( config_path ))
  {
    QMessageBox::critical( this, "Error Generating Files", 
                           QString("Unable to create directory ").append( config_path.c_str() ) );
    return;
  }
  
  // Copy barebones package template --------------------------------------------------
  // TODO
  // try this: http://stackoverflow.com/questions/8593608/how-can-i-copy-a-directory-using-boost-filesystem

  displayAction( qpackage_name,
                 "Package that contains all necessary configuration and launch files for MoveIt");

  // Select first item in list
  action_list_->setCurrentRow( 0 );

  // Create SRDF file -----------------------------------------------------------------
  const std::string srdf_file = config_data_->urdf_model_.getName() + ".srdf";
  const std::string srdf_path = config_path + "/" + srdf_file;

  if ( !config_data_->srdf_->writeSRDF( srdf_path ) )
  {
    QMessageBox::critical( this, "Error Generating Files", 
                           QString("Failed to create an SRDF file at location ").append( srdf_path.c_str() ) );
    return;
  }

  // Show 
  displayAction( QString( srdf_file.c_str() ).prepend( qconfig_path ), 
                 "SRDF (Semantic Robot Description Format) is a representation of semantic information about robots. This format is intended to represent information about the robot that is not in the URDF file, but it is useful for a variety of applications. The intention is to include information that has a semantic aspect to it. <a href='http://www.ros.org/wiki/srdf'>http://www.ros.org/wiki/srdf</a>");


  // Create other configuration files --------------------------------------------------
  // TODO

  // Alert user it completed successfully --------------------------------------------------
  progress_bar_->setValue( 100 );
  QMessageBox::information( this, "Complete", "All MoveIt configuration files generated successfully!" );

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
const std::string ConfigurationFilesWidget::getPackageName( std::string stack_path )
{
  std::string package_name;

  size_t found_index = stack_path.find_last_of("/\\");
  //std::cout << found_index << std::endl;

  // Check if last character is a slash
  if( found_index == stack_path.size() - 1 ) // get second to last
  {
    std::string sub_path = stack_path.substr( 0, stack_path.size() - 1);
    //std::cout << "substring " << sub_path << std::endl;

    found_index = sub_path.find_last_of("/\\");
    package_name = sub_path.substr( found_index + 1, sub_path.size() -1 );
  }
  else // get substring only
  {
    package_name = stack_path.substr( found_index + 1, stack_path.size() - 1 );
  }

  // check for empty
  if( package_name.empty() )
    package_name = "unknown";

  return package_name;
}

// ******************************************************************************************
// 
// ******************************************************************************************

// ******************************************************************************************
// 
// ******************************************************************************************

// ******************************************************************************************
// 
// ******************************************************************************************



