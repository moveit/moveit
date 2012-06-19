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
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QFormLayout>
#include <QLineEdit>
#include <QString>
#include "joint_collection_widget.h"

// ******************************************************************************************
// Outer User Interface for MoveIt Configuration Assistant
// ******************************************************************************************
JointCollectionWidget::JointCollectionWidget( QWidget *parent, moveit_setup_assistant::MoveItConfigDataPtr config_data )
  :  QWidget( parent ), config_data_( config_data )
{
  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout( );

  // Label ------------------------------------------------
  QLabel *group_title = new QLabel( this );
  group_title->setText( "Add/Edit Joint Collection" );
  QFont group_title_font( "Arial", 12, QFont::Bold );
  group_title->setFont(group_title_font);
  layout->addWidget( group_title);
  
  // Simple form -------------------------------------------
  QFormLayout *form_layout = new QFormLayout();
  form_layout->setContentsMargins( 0, 15, 0, 15 );

  // Name input
  QLineEdit *name_input = new QLineEdit( this );
  name_input->setMaximumWidth( 400 );
  form_layout->addRow( "Joint Collection Name:", name_input );
  
  layout->addLayout( form_layout );
  layout->setAlignment( Qt::AlignTop );

  // Double selection lists -------------------------------
  
  QHBoxLayout *hlayout = new QHBoxLayout();
  
  // Left column -------------------------------------------
  QVBoxLayout *column1 = new QVBoxLayout();

  // Label
  QLabel *column1_label = new QLabel( "Available Joints", this );
  column1->addWidget( column1_label );

  // Table
  joint_table_ = new QTableWidget( this );
  column1->addWidget( joint_table_ );

  // Add layouts
  hlayout->addLayout( column1 );

  // Center column ------------------------------------------
  QVBoxLayout *column2 = new QVBoxLayout();
  column2->setSizeConstraint( QLayout::SetFixedSize ); // constraint it
  
  // Right Arrow Button
  QPushButton *btn_right = new QPushButton( ">", this);
  btn_right->setMaximumSize(30, 80);
  column2->addWidget( btn_right );
  
  // Left Arrow Button
  QPushButton *btn_left = new QPushButton( "<", this);
  btn_left->setMaximumSize(30, 80);
  column2->addWidget( btn_left );

  // Add layouts
  hlayout->addLayout( column2 );

  // Right column -------------------------------------------
  QVBoxLayout *column3 = new QVBoxLayout();

  // Label
  QLabel *column3_label = new QLabel( "Selected Joints", this );
  column3->addWidget( column3_label );

  // Table
  selected_joint_table_ = new QTableWidget( this );
  column3->addWidget( selected_joint_table_ );

  // Add layouts
  hlayout->addLayout( column3 );

  // End Double Selection List ---------------------------------
  layout->addLayout( hlayout );
  

  // Button controls -------------------------------------------
  QHBoxLayout *controls_layout = new QHBoxLayout();
  controls_layout->setContentsMargins( 0, 25, 0, 15 );

  // Delete
  QPushButton *btn_delete = new QPushButton( "Delete This Group", this );
  btn_delete->setMaximumWidth( 200 );
  controls_layout->addWidget( btn_delete );

  // Spacer
  QWidget *spacer = new QWidget( this );
  spacer->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
  controls_layout->addWidget( spacer );

  // Save
  QPushButton *btn_save = new QPushButton( "Save", this );
  btn_save->setMaximumWidth( 200 );
  controls_layout->addWidget( btn_save );
  controls_layout->setAlignment(btn_save, Qt::AlignRight);


  // Cancel
  QPushButton *btn_cancel = new QPushButton( "Cancel", this );
  btn_cancel->setMaximumWidth( 200 );
  controls_layout->addWidget( btn_cancel );
  controls_layout->setAlignment(btn_cancel, Qt::AlignRight);
  
  // Add layout
  layout->addLayout( controls_layout );

  //layout->addWidget( new QLabel( "TEST", this ) );
  // Finish Layout --------------------------------------------------

  this->setLayout(layout);
}

