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
#include <QHBoxLayout>
#include <QMessageBox>
#include <QFormLayout>
#include <QString>
#include "group_edit_widget.h"
#include <pluginlib/class_loader.h> // for loading all avail kinematic planners

namespace moveit_setup_assistant
{

// ******************************************************************************************
// 
// ******************************************************************************************
GroupEditWidget::GroupEditWidget( QWidget *parent, moveit_setup_assistant::MoveItConfigDataPtr config_data )
  :  QWidget( parent ), config_data_( config_data )
{
  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout( );

  // Label ------------------------------------------------
  title_ = new QLabel( this ); // specify the title from the parent widget
  QFont group_title_font( "Arial", 12, QFont::Bold );
  title_->setFont(group_title_font);
  layout->addWidget( title_ );
  
  // Simple form -------------------------------------------
  QFormLayout *form_layout = new QFormLayout();
  form_layout->setContentsMargins( 0, 15, 0, 15 );

  // Name input
  group_name_field_ = new QLineEdit( this );
  group_name_field_->setMaximumWidth( 400 );
  form_layout->addRow( "Group Name:", group_name_field_ );

  // Kinematic solver
  kinematics_solver_field_ = new QComboBox( this );
  kinematics_solver_field_->setEditable( false );
  kinematics_solver_field_->setMaximumWidth( 400 );
  form_layout->addRow( "Kinematic Solver:", kinematics_solver_field_ );
  
  // resolution to use with solver
  kinematics_resolution_field_ = new QLineEdit( this );
  kinematics_resolution_field_->setMaximumWidth( 400 );
  form_layout->addRow( "Kin. Search Resolution:", kinematics_resolution_field_ );

  // resolution to use with solver
  kinematics_timeout_field_ = new QLineEdit( this );
  kinematics_timeout_field_->setMaximumWidth( 400 );
  form_layout->addRow( "Kin. Search Timeout (s):", kinematics_timeout_field_ );

  layout->addLayout( form_layout );
  layout->setAlignment( Qt::AlignTop );

  // New Group Options  ---------------------------------------------------------
  new_buttons_widget_ = new QWidget();
  QVBoxLayout *new_buttons_layout = new QVBoxLayout();

  QLabel *save_and_add = new QLabel( "Specify components for this planning group:", this );
  QFont save_and_add_font( "Arial", 10, QFont::Bold );
  save_and_add->setFont( save_and_add_font );
  new_buttons_layout->addWidget( save_and_add );

  // Save and add joints
  QPushButton *btn_save_joints = new QPushButton( "Save and Add Joints", this );
  btn_save_joints->setMaximumWidth( 280 );
  connect( btn_save_joints, SIGNAL(clicked()), this, SIGNAL( saveJoints() ) );
  new_buttons_layout->addWidget( btn_save_joints );

  // Save and add links
  QPushButton *btn_save_links = new QPushButton( "Save and Add Links", this );
  btn_save_links->setMaximumWidth( 280 );
  connect( btn_save_links, SIGNAL(clicked()), this, SIGNAL( saveLinks() ) );
  new_buttons_layout->addWidget( btn_save_links );

  // Save and add chain
  QPushButton *btn_save_chain = new QPushButton( "Save and Add Chain", this );
  btn_save_chain->setMaximumWidth( 280 );
  connect( btn_save_chain, SIGNAL(clicked()), this, SIGNAL( saveChain() ) );
  new_buttons_layout->addWidget( btn_save_chain );

  // Save and add subgroups
  QPushButton *btn_save_subgroups = new QPushButton( "Save and Add Subgroups", this );
  btn_save_subgroups->setMaximumWidth( 280 );
  connect( btn_save_subgroups, SIGNAL(clicked()), this, SIGNAL( saveSubgroups() ) );
  new_buttons_layout->addWidget( btn_save_subgroups );

  // Create widget and add to main layout
  new_buttons_widget_->setLayout( new_buttons_layout );
  layout->addWidget( new_buttons_widget_ );

  // Verticle Spacer -----------------------------------------------------
  QWidget *vspacer = new QWidget( this );
  vspacer->setSizePolicy( QSizePolicy::Preferred, QSizePolicy::Expanding );
  layout->addWidget( vspacer );
  
  // Bottom Controls ---------------------------------------------------------
  QHBoxLayout *controls_layout = new QHBoxLayout();

  // Delete
  btn_delete_ = new QPushButton( "&Delete Group", this );
  btn_delete_->setMaximumWidth( 200 );
  connect( btn_delete_, SIGNAL(clicked()), this, SIGNAL( deleteGroup() ) );
  controls_layout->addWidget( btn_delete_ );
  controls_layout->setAlignment(btn_delete_, Qt::AlignRight);

  // Horizontal Spacer
  QWidget *spacer = new QWidget( this );
  spacer->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
  controls_layout->addWidget( spacer );

  // Save
  btn_save_ = new QPushButton( "&Save", this );
  btn_save_->setMaximumWidth( 200 );
  connect( btn_save_, SIGNAL(clicked()), this, SIGNAL( save() ) );
  controls_layout->addWidget( btn_save_ );
  controls_layout->setAlignment(btn_save_, Qt::AlignRight);

  // Cancel
  QPushButton *btn_cancel = new QPushButton( "&Cancel", this );
  btn_cancel->setMaximumWidth( 200 );
  connect( btn_cancel, SIGNAL(clicked()), this, SIGNAL( cancelEditing() ) );
  controls_layout->addWidget( btn_cancel );
  controls_layout->setAlignment(btn_cancel, Qt::AlignRight);
  
  // Add layout
  layout->addLayout( controls_layout );

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
  
}

// ******************************************************************************************
// Set the link field with previous value
// ******************************************************************************************
void GroupEditWidget::setSelected( const std::string &group_name )
{
  group_name_field_->setText( QString( group_name.c_str() ) );
  
  // Load properties from moveit_config_data.cpp ----------------------------------------------

  // Load resolution
  double *resolution = &config_data_->group_meta_data_[ group_name ].kinematics_solver_search_resolution_;
  if( *resolution == 0 )
  {
    // Set default value
    *resolution = 0.005;
  }
  kinematics_resolution_field_->setText( QString::number( *resolution ) );

  // Load timeout
  double *timeout = &config_data_->group_meta_data_[ group_name ].kinematics_solver_timeout_;
  if( *timeout == 0 )
  {
    // Set default value
    *timeout = 0.05;
  }
  kinematics_timeout_field_->setText( QString::number( *timeout ) );

  // Set kin solver
  const std::string& kin_solver = config_data_->group_meta_data_[ group_name ].kinematics_solver_;

  // Set the kin solver combo box if its not empty
  if( !kin_solver.empty() )
  {
    int index = kinematics_solver_field_->findText( kin_solver.c_str() );
    if( index == -1 )
    {
      QMessageBox::warning( this, "Missing Kinematic Solvers", 
                            QString( "Unable to find the kinematic solver '").append( kin_solver.c_str() )
                            .append( "'. Trying running rosmake for this package. Until fixed, this setting will be lost the next time the MoveIt configuration files are generated" ));
      return;
    }
    else
    {
      kinematics_solver_field_->setCurrentIndex( index );
    }
  }

  // Set default
}

// ******************************************************************************************
// Populate the combo dropdown box with kinematic planners
// ******************************************************************************************
void GroupEditWidget::loadKinematicPlannersComboBox()
{
  // Only load this combo box once
  static bool hasLoaded = false;
  if( hasLoaded )
    return;
  hasLoaded = true;

  // Remove all old items
  kinematics_solver_field_->clear();

  // Add none option, the default
  kinematics_solver_field_->addItem( "None" );  

  // load all avail kin planners
  boost::scoped_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > loader;
  try
  {
    loader.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));
  }
  catch(pluginlib::PluginlibException& ex)
  { 
    QMessageBox::warning( this, "Missing Kinematic Solvers", "Exception while creating class loader for kinematic solver plugins");
    ROS_ERROR_STREAM( ex.what() );
    return;
  }  

  // Get classes
  const std::vector<std::string> &classes = loader->getDeclaredClasses();

  // Loop through all planners and add to combo box
  for( std::vector<std::string>::const_iterator plugin_it = classes.begin();
       plugin_it != classes.end(); ++plugin_it )
  {
    kinematics_solver_field_->addItem( plugin_it->c_str() );
  }



}

} // namespace
