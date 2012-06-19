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

#include "setup_assistant_widget.h"

using namespace moveit_configuration_tools;

// ******************************************************************************************
// Outer User Interface for MoveIt Configuration Assistant
// ******************************************************************************************
SetupAssistantWidget::SetupAssistantWidget( QWidget *parent )
  : QWidget( parent )
{
  // Create timer to ping ROS ----------------------------------------
  QTimer *update_timer = new QTimer( this );
  connect( update_timer, SIGNAL( timeout() ), this, SLOT( updateTimer() ));
  update_timer->start( 1000 );
  
  // Create object to hold all moveit configuration data
  MoveItConfigDataPtr config_data( new MoveItConfigData() );

  // Basic widget container -----------------------------------------
  QHBoxLayout *layout = new QHBoxLayout( this );
  layout->setAlignment( Qt::AlignTop );


  // Screens --------------------------------------------------------

  // Start Screen
  StartScreenWidget *ssw = new StartScreenWidget( this, config_data );
  //ssw->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  connect( ssw, SIGNAL( readyToProgress() ), this, SLOT( progressPastStartScreen() ) );
  navs_ << NavScreen("Start", ssw, false);
  
  // Planning Groups
  PlanningGroupsWidget *pgw = new PlanningGroupsWidget( this, config_data );
  navs_ << NavScreen("Planning Groups", pgw, true);

  // Self-Collisions
  ComputeDefaultCollisionsWidget *cdcw = new ComputeDefaultCollisionsWidget( this, config_data);
  navs_ << NavScreen("Self-Collisions", cdcw, true);

  // Robot Poses
  RobotPosesWidget *rpw = new RobotPosesWidget( this, config_data );
  navs_ << NavScreen("Robot Poses", rpw, false);

  // End Effectors
  EndEffectorsWidget *efw = new EndEffectorsWidget( this, config_data );
  navs_ << NavScreen("End Effectors", efw, false);

  // Configuration Files
  ConfigurationFilesWidget *cfw = new ConfigurationFilesWidget( this, config_data );
  navs_ << NavScreen("Configuration Files", cfw, true);
  


  // Left side navigation -------------------------------------------
  navs_view_ = new NavigationWidget( this );
  navs_view_->setNavs(navs_);
  navs_view_->setDisabled( true );

  connect( navs_view_, SIGNAL(clicked(const QModelIndex&)), this, SLOT(navigationClicked(const QModelIndex&)) );

  // Initial right frame widget holder. Change these 2 lines if you want diff default screen
  right_frame_ = ssw;
  navs_view_->setSelected(0); // Select first item in list

  // Split screen
  splitter_ = new QSplitter( Qt::Horizontal, this );
  splitter_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  splitter_->addWidget( navs_view_ );
  splitter_->addWidget( right_frame_ );  
  layout->addWidget( splitter_ );

  //layout->addWidget( right_frame_ );
  //layout->addWidget( navs_view_ );

  // Final Layout Setup ---------------------------------------------
  this->setLayout(layout);

  // Title
  this->setWindowTitle("MoveIt Setup Assistant"); // title of window
}

void SetupAssistantWidget::moveToScreen( const int index )
{
  // Get a reference to the requested screen
  NavScreen next_screen = navs_.at( index );

  // Hide the widget currently in the right frame
  right_frame_->hide();  

  // Change widgets
  right_frame_ = next_screen.screen();

  // Insert widget into splitter
  splitter_->addWidget( right_frame_ );
  right_frame_->show();

  // Change navigation selected option
  navs_view_->setSelected( index ); // Select first item in list
}
// ******************************************************************************************
// Change screens of Setup Assistant
// ******************************************************************************************
void SetupAssistantWidget::navigationClicked( const QModelIndex& index )
{
  moveToScreen( index.row() );
}

// ******************************************************************************************
// Ping ROS on internval
// ******************************************************************************************
void SetupAssistantWidget::updateTimer()
{
  ros::spinOnce(); // keep ROS alive

}

// ******************************************************************************************
// Enables navigation and goes to screen 2
// ******************************************************************************************
void SetupAssistantWidget::progressPastStartScreen()
{
  // Enable all nav buttons
  /*for(int i = 0; i < navs_.size(); i ++)
  {
    navs_[i].setDisabled( false );
    navs_[i].
    }*/
  
  // Repaint - TODO
  navs_view_->hide();
  navs_view_->repaint();
  navs_view_->update();
  navs_view_->show();
  QCoreApplication::processEvents();

  // Enable navigation
  navs_view_->setDisabled( false );
  
  // Go to next screen
  moveToScreen( 1 );
}
