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
n *   * Redistributions of source code must retain the above copyright
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

#include "setup_assistant_widget.h"
#include <QStackedLayout>
#include <QListWidget>
#include <QListWidgetItem>
#include <QDebug>
#include <QFont>
#include <QLabel>
#include <QPushButton>
#include <QCloseEvent>
#include <QMessageBox>
#include "setup_screen_widget.h" // a base class for screens in the setup assistant

namespace moveit_setup_assistant
{

// ******************************************************************************************
// Outer User Interface for MoveIt Configuration Assistant
// ******************************************************************************************
SetupAssistantWidget::SetupAssistantWidget( QWidget *parent, boost::program_options::variables_map args )
  : QWidget( parent )
{
  // Create timer to ping ROS ----------------------------------------
  /*QTimer *update_timer = new QTimer( this );
  connect( update_timer, SIGNAL( timeout() ), this, SLOT( updateTimer() ));
  update_timer->start( 250 );*/
  
  // Create object to hold all moveit configuration data
  config_data_.reset( new MoveItConfigData() );

  // Basic widget container -----------------------------------------
  QHBoxLayout *layout = new QHBoxLayout();
  layout->setAlignment( Qt::AlignTop );

  main_content_ = new QStackedLayout();

  // Screens --------------------------------------------------------

  // Start Screen
  ssw_ = new StartScreenWidget( this, config_data_ );
  connect( ssw_, SIGNAL( readyToProgress() ), this, SLOT( progressPastStartScreen() ) );
  main_content_->addWidget(ssw_);

  // Pass command arg values to start screen
  if (args.count("urdf"))
    ssw_->urdf_file_->setPath( args["urdf"].as<std::string>() );
  if (args.count("srdf"))
    ssw_->srdf_file_->setPath( args["srdf"].as<std::string>() );
  if (args.count("config_pkg"))
    ssw_->stack_path_->setPath( args["config_pkg"].as<std::string>() );

  // Add Navigation Buttons (but do not load widgets yet except start screen)
  nav_name_list_ << "Start";
  nav_name_list_ << "Planning Groups";
  nav_name_list_ << "Self-Collisions";
  nav_name_list_ << "Robot Poses";
  nav_name_list_ << "End Effectors";
  nav_name_list_ << "Configuration Files";

  // Navigation Left Pane --------------------------------------------------
  navs_view_ = new NavigationWidget( this );
  navs_view_->setNavs(nav_name_list_);
  navs_view_->setDisabled( true );
  navs_view_->setSelected( 0 ); // start screen
  
  // Wrap main_content_ with a widget
  right_frame_ = new QWidget( this );
  right_frame_->setLayout( main_content_ );

  // Split screen -----------------------------------------------------
  splitter_ = new QSplitter( Qt::Horizontal, this );
  splitter_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  splitter_->addWidget( navs_view_ );
  splitter_->addWidget( right_frame_ );  
  layout->addWidget( splitter_ );

  // Add event for switching between screens -------------------------
  connect( navs_view_, SIGNAL(clicked(const QModelIndex&)), this, SLOT(navigationClicked(const QModelIndex&)) );

  
  // Final Layout Setup ---------------------------------------------
  this->setLayout(layout);
  
  // Title
  this->setWindowTitle("MoveIt Setup Assistant"); // title of window
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
  // Change screens
  main_content_->setCurrentIndex( index );

  // Send the focus given command to the screen widget
  SetupScreenWidget *ssw = qobject_cast< SetupScreenWidget* >( main_content_->widget( index ) );
  ssw->focusGiven();  

  // Change navigation selected option
  navs_view_->setSelected( index ); // Select first item in list
}

// ******************************************************************************************
// Loads other windows, enables navigation and goes to screen 2
// ******************************************************************************************
void SetupAssistantWidget::progressPastStartScreen()
{
  // Load all widgets ------------------------------------------------

  // Planning Groups
  pgw_ = new PlanningGroupsWidget( this, config_data_ );
  main_content_->addWidget(pgw_);

  // Self-Collisions
  cdcw_ = new ComputeDefaultCollisionsWidget( this, config_data_);
  main_content_->addWidget(cdcw_);

  // Robot Poses
  rpw_ = new RobotPosesWidget( this, config_data_ );
  main_content_->addWidget(rpw_);

  // End Effectors
  efw_ = new EndEffectorsWidget( this, config_data_ );
  main_content_->addWidget(efw_);  

  // Configuration Files
  cfw_ = new ConfigurationFilesWidget( this, config_data_ );
  main_content_->addWidget(cfw_);  

  // Pass command arg values to config files screen
  cfw_->stack_path_->setPath( ssw_->stack_path_->getQPath() );


  // Enable all nav buttons -------------------------------------------
  for( int i = 0; i < nav_name_list_.count(); ++i)
  {
    navs_view_->setEnabled( i, true );
  }

  // Go to next screen
  moveToScreen( 1 );

  // Enable navigation
  navs_view_->setDisabled( false );
}

// ******************************************************************************************
// Ping ROS on internval
// ******************************************************************************************
void SetupAssistantWidget::updateTimer()
{
  ros::spinOnce(); // keep ROS alive
}

// ******************************************************************************************
// Qt close event function for reminding user to save
// ******************************************************************************************
void SetupAssistantWidget::closeEvent( QCloseEvent * event )
{
  if( QMessageBox::question( this, "Exit Setup Assistant", 
                             QString("Are you sure you want to exit the MoveIt Setup Assistant?"),
                             QMessageBox::Ok | QMessageBox::Cancel) 
      == QMessageBox::Cancel )
  {
      event->ignore();
      return;
  }

  event->accept();
}



}
