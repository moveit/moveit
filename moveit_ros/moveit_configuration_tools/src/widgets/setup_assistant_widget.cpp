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


// ******************************************************************************************
// Outer User Interface for MoveIt Configuration Assistant
// ******************************************************************************************
SetupAssistantWidget::SetupAssistantWidget()
{
  // Basic widget container
  QHBoxLayout *layout = new QHBoxLayout();
  layout->setAlignment( Qt::AlignTop );
  //layout->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  // Screens --------------------------------------------------------

  // Start Screen
  StartScreenWidget *ssw = new StartScreenWidget();
  ssw->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  navs_ << NavItem("Start", ssw);
  
  // Planning Groups
  QWidget *blank = new QWidget();
  navs_ << NavItem("Planning Groups", blank);

  // Self-Collisions
  ComputeDefaultCollisionsWidget *cdcw = new ComputeDefaultCollisionsWidget("TODO");
  navs_ << NavItem("Self-Collisions", cdcw);

  // Robot Poses
  QWidget *blank2 = new QWidget();
  navs_ << NavItem("Robot Poses", blank2);

  // Configuration Files
  QWidget *blank3 = new QWidget();
  navs_ << NavItem("Configuration Files", blank3);

  // Left side navigation -------------------------------------------
  navs_view_ = new NavigationWidget();
  navs_view_->setNavs(navs_);
  connect( navs_view_, SIGNAL(clicked(const QModelIndex&)), this, SLOT(navigation_clicked(const QModelIndex&)) );

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


void SetupAssistantWidget::navigation_clicked( const QModelIndex& index )
{
  NavItem this_nav = navs_.at( index.row() ); //(NavItem*) index.model();
  //qDebug() << this_nav.name();

  // Hide the widget currently in the right frame
  right_frame_->hide();  

  // Change widgets
  right_frame_ = this_nav.screen();

  // Insert widget into splitter
  splitter_->addWidget( right_frame_ );
  right_frame_->show();
}
