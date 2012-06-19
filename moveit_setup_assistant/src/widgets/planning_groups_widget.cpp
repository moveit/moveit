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

#include "planning_groups_widget.h"
#include <boost/thread.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include "ros/ros.h"
#include "header_widget.h"


// ******************************************************************************************
// CLASS
// ******************************************************************************************
PlanningGroupsWidget::PlanningGroupsWidget( QWidget *parent, moveit_setup_assistant::MoveItConfigDataPtr config_data )
  : QWidget( parent ), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout();
  //QVBoxLayout *left_layout = new QVBoxLayout();
  QVBoxLayout *right_layout = new QVBoxLayout();
  //left_layout->setContentsMargins( 0, 0, 0, 0);

  // Top Label Area ------------------------------------------------

  HeaderWidget *header = new HeaderWidget( "Create Planning Groups",
                                           "Select planning groups for your robot based on kinematic chains or joint collections.",
                                           this);
  layout->addWidget( header );

  // Left Side ---------------------------------------------

  // Create left side widgets 
  //  groups_table_widget_ = new PlanningGroupsTableWidget( this, config_data_ );
  groups_table_widget_ = createContentsWidget();

  joints_widget_ = new JointCollectionWidget( this, config_data_ );

  // Combine into stack
  stacked_layout_ = new QStackedLayout( this );
  stacked_layout_->addWidget( groups_table_widget_ );
  stacked_layout_->addWidget( joints_widget_ );

  stacked_layout_->setCurrentIndex( 0 );
  //left_layout->addLayout( stacked_layout_ );
  
  // Rviz Right Side -------------------------------------
  QLabel *temp = new QLabel( "RVIZ", this );
  temp->setMinimumWidth( 300 );
  right_layout->addWidget( temp );
  right_layout->setAlignment( temp, Qt::AlignCenter );

  // Split screen -----------------------------------------
  QWidget *left_frame = new QWidget( this );
  left_frame->setLayout( stacked_layout_ );
  
  QWidget *right_frame = new QWidget( this );
  right_frame->setLayout( right_layout );
   
  QSplitter *splitter = new QSplitter( Qt::Horizontal, this );
  splitter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  splitter->addWidget( left_frame );
  splitter->addWidget( right_frame ); 
 
  layout->addWidget( splitter );
 

  setLayout(layout);
}

void PlanningGroupsWidget::changeScreen( int index )
{
  stacked_layout_->setCurrentIndex( index );
}


QWidget* PlanningGroupsWidget::createContentsWidget()
{
  // Main widget
  QWidget *content_widget = new QWidget( this );

  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout( this );

  // Table Area --------------------------------------------  

  // Table Label
  QLabel *table_title = new QLabel( this );
  table_title->setText( "Current Groups" );
  QFont table_title_font( "Arial", 10, QFont::Bold );
  table_title->setFont(table_title_font);
  layout->addWidget( table_title);

  // Table
  groups_table_ = new QTableWidget( this );
  groups_table_->setColumnCount(2);
  groups_table_->setSortingEnabled(true);
  //connect(groups_table_, SIGNAL(cellChanged(int,int)), this, SLOT(toggleCheckBox(int,int)));
  layout->addWidget(groups_table_);

  // Table Headers
  QStringList header_list;
  header_list.append("Planning Group");
  header_list.append("Group Type");
  groups_table_->setHorizontalHeaderLabels(header_list);
  groups_table_->resizeColumnToContents(0);
  groups_table_->resizeColumnToContents(1);
 
  // Bottom Area ----------------------------------------
  QHBoxLayout *controls1_layout = new QHBoxLayout( );

  // Add Joint Colletion Group
  QPushButton *btn_joint = new QPushButton( "Add &Joint Collection Group", this );
  btn_joint->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btn_joint->setMinimumWidth(250);
  connect(btn_joint, SIGNAL(clicked()), this, SLOT(addJointCollectionGroup()));
  controls1_layout->addWidget(btn_joint);

  // Add Link Colletion Group
  QPushButton *btn_link = new QPushButton( "Add &Link Collection Group", this );
  btn_link->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btn_link->setMinimumWidth(250);
  connect(btn_link, SIGNAL(clicked()), this, SLOT(addLinkCollectionGroup()));
  controls1_layout->addWidget(btn_link);

  QHBoxLayout *controls2_layout = new QHBoxLayout( );

  // Add Kinematics Chain Button
  QPushButton *btn_kinematics = new QPushButton( this );
  btn_kinematics->setText("Add &Kinematics Chain Group");
  btn_kinematics->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btn_kinematics->setMinimumWidth(250);
  connect(btn_kinematics, SIGNAL(clicked()), this, SLOT(addKinematicChainGroup()));
  controls2_layout->addWidget(btn_kinematics);

  // Add End Effector Button
  QPushButton *btn_end_effector = new QPushButton( this );
  btn_end_effector->setText("Add &End Effector");
  btn_end_effector->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btn_end_effector->setMinimumWidth(250);
  connect(btn_end_effector, SIGNAL(clicked()), this, SLOT(addEndEffector()));
  controls2_layout->addWidget(btn_end_effector);

  QHBoxLayout *controls3_layout = new QHBoxLayout( );

  // Add Super Group Button
  QPushButton *btn_super_group = new QPushButton( this );
  btn_super_group->setText("Add &Super Group");
  btn_super_group->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btn_super_group->setMinimumWidth(250);
  connect(btn_super_group, SIGNAL(clicked()), this, SLOT(addSuperGroup()));
  controls3_layout->addWidget(btn_super_group);

  
  // Add Controls to layout
  layout->addLayout( controls1_layout );
  layout->addLayout( controls2_layout );
  layout->addLayout( controls3_layout );

  // Set layout
  content_widget->setLayout(layout);

  return content_widget;
}

// ******************************************************************************************
// Displays data in the link_pairs_ data structure into a QtTableWidget
// ******************************************************************************************
void PlanningGroupsWidget::loadGroupsTable()
{
  int row = 0;
  groups_table_->setUpdatesEnabled(false); // prevent table from updating until we are completely done
  groups_table_->setDisabled(true); // make sure we disable it so that the cellChanged event is not called
  groups_table_->clearContents();
  /*
  // Intially set the table to be worst-case scenario of every possible element pair
  groups_table_->setRowCount( link_pairs_.size() ); 

  for ( moveit_setup_assistant::LinkPairMap::const_iterator pair_it = link_pairs_.begin(); 
  pair_it != link_pairs_.end(); 
  ++pair_it)
  {
  // Add link pair row if 1) it is disabled from collision checking or 2) the SHOW ALL LINK PAIRS checkbox is checked
  if( pair_it->second.disable_check || collision_checkbox_->isChecked() ) 
  {
      
  // Create row elements
  QTableWidgetItem* linkA = new QTableWidgetItem( pair_it->first.first.c_str() ); 
  linkA->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

  QTableWidgetItem* linkB = new QTableWidgetItem( pair_it->first.second.c_str() );
  linkB->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

  CheckboxSortWidgetItem* disable_check = new CheckboxSortWidgetItem;
  disable_check->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
  if( pair_it->second.disable_check ) // Checked means no collision checking
  disable_check->setCheckState(Qt::Checked);
  else
  disable_check->setCheckState(Qt::Unchecked);

  QTableWidgetItem* reason = new QTableWidgetItem( longReasonsToString.at( pair_it->second.reason ) );
  reason->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

  // Insert row elements into collision table
  groups_table_->setItem( row, 0, linkA);
  groups_table_->setItem( row, 1, linkB);
  groups_table_->setItem( row, 2, disable_check);
  groups_table_->setItem( row, 3, reason);
            
  // Increment row count
  ++row;
  }
    
  ++progress_counter; // for calculating progress bar

  if( progress_counter % 200 == 0 )
  {
  // Update Progress Bar
  progress_bar_->setValue( progress_counter * 100 / link_pairs_.size() );  
  QApplication::processEvents(); // allow the progress bar to be shown
  }

  }
  */

  // Reduce the table size to only the number of used rows
  groups_table_->setRowCount( row ); 

  groups_table_->setUpdatesEnabled(true); // prevent table from updating until we are completely done
}

void PlanningGroupsWidget::addJointCollectionGroup()
{
  std::cout << "ADD JOINT COLL" << std::endl;
  stacked_layout_->setCurrentIndex( 1 );

}

void PlanningGroupsWidget::addLinkCollectionGroup()
{
  std::cout << "ADD Link COLL" << std::endl;
}

void PlanningGroupsWidget::addKinematicChainGroup()
{
  std::cout << "ADD KIN CHAIN" << std::endl;
}

void PlanningGroupsWidget::addEndEffector()
{
  std::cout << "ADD END EFFEC" << std::endl;
}

void PlanningGroupsWidget::addSuperGroup()
{
  std::cout << "ADD SUPER GROUP" << std::endl;
}

void PlanningGroupsWidget::doneEditing()
{
  std::cout << "BACK" << std::endl;
  stacked_layout_->setCurrentIndex( 0 );
}
