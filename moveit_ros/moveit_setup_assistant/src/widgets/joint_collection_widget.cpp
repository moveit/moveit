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
#include <QMessageBox>
#include <QString>
#include "joint_collection_widget.h"
#include <planning_scene/planning_scene.h> // for getting the joints
#include <planning_scene_monitor/planning_scene_monitor.h> // for getting monitor

static const std::string ROBOT_DESCRIPTION="robot_description";

// ******************************************************************************************
// 
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
  name_input_ = new QLineEdit( this );
  name_input_->setMaximumWidth( 400 );
  form_layout->addRow( "Joint Collection Name:", name_input_ );
  
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
  joint_table_->setColumnCount(1);
  joint_table_->setSortingEnabled(true);
  column1->addWidget( joint_table_ );

  // Table headers
  QStringList joint_header_list;
  joint_header_list.append("Joint Names");
  joint_table_->setHorizontalHeaderLabels( joint_header_list );

  // Add layouts
  hlayout->addLayout( column1 );

  // Center column ------------------------------------------
  QVBoxLayout *column2 = new QVBoxLayout();
  column2->setSizeConstraint( QLayout::SetFixedSize ); // constraint it
  
  // Right Arrow Button
  QPushButton *btn_right = new QPushButton( ">", this);
  btn_right->setMaximumSize(30, 80);
  connect( btn_right, SIGNAL( clicked() ), this, SLOT( selectJointButtonClicked() ) );
  column2->addWidget( btn_right );
  
  // Left Arrow Button
  QPushButton *btn_left = new QPushButton( "<", this);
  btn_left->setMaximumSize(30, 80);
  connect( btn_left, SIGNAL( clicked() ), this, SLOT( deselectJointButtonClicked() ) );
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
  selected_joint_table_->setColumnCount(1);
  selected_joint_table_->setSortingEnabled(true);
  column3->addWidget( selected_joint_table_ );

  // Table Headers (use same)
  selected_joint_table_->setHorizontalHeaderLabels( joint_header_list );

  // Add layouts
  hlayout->addLayout( column3 );

  // End Double Selection List ---------------------------------
  layout->addLayout( hlayout );
  

  // Button controls -------------------------------------------
  QHBoxLayout *controls_layout = new QHBoxLayout();
  controls_layout->setContentsMargins( 0, 25, 0, 15 );

  // Delete
  QPushButton *btn_delete = new QPushButton( "&Delete This Group", this );
  connect( btn_delete, SIGNAL(clicked()), this, SLOT( deleteGroup() ) );
  btn_delete->setMaximumWidth( 200 );
  controls_layout->addWidget( btn_delete );

  // Spacer
  QWidget *spacer = new QWidget( this );
  spacer->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
  controls_layout->addWidget( spacer );

  // Save
  QPushButton *btn_save = new QPushButton( "&Save", this );
  btn_save->setMaximumWidth( 200 );
  connect( btn_save, SIGNAL(clicked()), this, SLOT( saveGroup() ) );
  controls_layout->addWidget( btn_save );
  controls_layout->setAlignment(btn_save, Qt::AlignRight);

  // Cancel
  QPushButton *btn_cancel = new QPushButton( "&Cancel", this );
  btn_cancel->setMaximumWidth( 200 );
  connect( btn_cancel, SIGNAL(clicked()), this, SLOT( quitScreen() ) );
  controls_layout->addWidget( btn_cancel );
  controls_layout->setAlignment(btn_cancel, Qt::AlignRight);
  
  // Add layout
  layout->addLayout( controls_layout );

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

void JointCollectionWidget::quitScreen()
{
  // discard changes and return to prev screen
  Q_EMIT( doneEditing() );
}

void JointCollectionWidget::saveGroup()
{
  // check that there are joints seleted
  if( !selected_joint_table_->rowCount() )
  {
    QMessageBox::critical( this, "Error Saving", "Please select some joints for the group before saving." );
    return;
  }

  // check that group has name
  std::string group_name = name_input_->text().toStdString();
  boost::trim( group_name );

  if( group_name == "" )
  {
    QMessageBox::critical( this, "Error Saving", "Please provide a name for the joint collection." );
    return;
  }
  
  // create new group
  srdf::Model::Group this_group;
  this_group.name_ = group_name;
  
  // add all joints to group
  for( int i = 0; i < selected_joint_table_->rowCount(); ++i )
  {
    this_group.joints_.push_back( selected_joint_table_->item(i,0)->text().toStdString() );
  }

  // save group to moveit_config_data
  config_data_->srdf_->groups_.push_back( this_group );

  // return to previous screen
  Q_EMIT( doneEditing() );
}

void JointCollectionWidget::deleteGroup()
{
  // Confirm user wants to delete group
  if( QMessageBox::question( this, "Delete Planning Group", 
                             "Are you sure you want to delete this joint collection?") == QMessageBox::Ok )
  {
    // delete group

    // return to prev screen
    Q_EMIT( doneEditing() );
  }
}

void JointCollectionWidget::loadJoints()
{
  std::cout << "LOADING JOINTS" << std::endl;

  // Load robot description
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);

  // Load scene
  const planning_scene::PlanningSceneConstPtr &scene = psm.getPlanningScene();

  // Get the names of the all joints
  const std::vector<std::string> joints = scene->getKinematicModel()->getJointModelNames();

  if( joints.size() == 0 )
  {
    ROS_ERROR("No joints found for robot model");
    return;
  }

  // Setup table
  joint_table_->setUpdatesEnabled(false); // prevent table from updating until we are completely done
  joint_table_->setDisabled(true); // make sure we disable it so that the cellChanged event is not called
  joint_table_->clearContents();

  // Intially set the table to be worst-case scenario of every joint
  joint_table_->setRowCount( joints.size() ); 

  // Loop through every joint
  int row = 0;
  for( std::vector<std::string>::const_iterator joint_it = joints.begin(); joint_it != joints.end(); ++joint_it )
  {
    //std::cout << *joint_it << std::endl;

    // Create row elements
    QTableWidgetItem* joint_name = new QTableWidgetItem( joint_it->c_str() );
    joint_name->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    // Add to table
    joint_table_->setItem( row, 0, joint_name );
    
    // Increment counter
    ++row;
  }

  // Reduce the table size to only the number of used rows
  joint_table_->setRowCount( row ); 

  // Reenable
  joint_table_->setUpdatesEnabled(true); // prevent table from updating until we are completely done
  joint_table_->setDisabled(false); // make sure we disable it so that the cellChanged event is not called

  // Resize both tables
  joint_table_->resizeColumnToContents(0);
  selected_joint_table_->setColumnWidth( 0, joint_table_->columnWidth( 0 ) );
}


void JointCollectionWidget::selectJointButtonClicked()
{
  // Get list of all selected joints
  QList<QTableWidgetItem*> selected = joint_table_->selectedItems();

  // Loop through all selected joints
  for(int i = 0; i < selected.size(); i++)
  {
    std::string name = selected[i]->text().toStdString();
    bool alreadyExists = false;
    int rowToAdd = 0;

    // Check if this selected joint is already in the selected joint table
    for(int r = 0; r < selected_joint_table_->rowCount(); r++)
    {
      QTableWidgetItem* item = selected_joint_table_->item(r, 0);

      if(item->text().toStdString() == name)
      {
        alreadyExists = true;
        break;
      }
      rowToAdd = r + 1;
    }

    // This joint needs to be added to the selected joint table
    if(!alreadyExists)
    {
      selected_joint_table_->setRowCount(selected_joint_table_->rowCount() + 1);
      QTableWidgetItem* newItem = new QTableWidgetItem(name.c_str());
      newItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      selected_joint_table_->setItem(rowToAdd, 0, newItem);
    }
  }

  //TODO?
  /*
    if(selected_joint_table_->rowCount() > 0) {
    first_joint_field_->setText("has");
    }
  */

}

void JointCollectionWidget::deselectJointButtonClicked()
{
  // Get list of joints to be removed from selected list
  QList<QTableWidgetItem*> deselected = selected_joint_table_->selectedItems();

  // loop through deselect list and remove
  for(int i = 0; i < deselected.size(); i++)
  {
    selected_joint_table_->removeRow(deselected[i]->row());
  }

  // TODO?
  /*
    if(selected_joint_table_->rowCount() == 0) {
    first_joint_field_->clear();
    }
  */
}

