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
  
  layout->addLayout( form_layout );
  layout->setAlignment( Qt::AlignTop );

  // Verticle Spacer
  QWidget *vspacer = new QWidget( this );
  vspacer->setSizePolicy( QSizePolicy::Preferred, QSizePolicy::Expanding );
  form_layout->addWidget( vspacer );
  
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
  QPushButton *btn_save = new QPushButton( "&Save", this );
  btn_save->setMaximumWidth( 200 );
  connect( btn_save, SIGNAL(clicked()), this, SIGNAL( doneEditing() ) );
  controls_layout->addWidget( btn_save );
  controls_layout->setAlignment(btn_save, Qt::AlignRight);

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
}


}
