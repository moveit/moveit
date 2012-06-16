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


// ******************************************************************************************
// 
// ******************************************************************************************
PlanningGroupsWidget::PlanningGroupsWidget( QWidget *parent, moveit_configuration_tools::MoveItConfigDataPtr config_data )
  : QWidget( parent ), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout( this );
  QVBoxLayout *left_layout = new QVBoxLayout( );
  QVBoxLayout *right_layout = new QVBoxLayout( );
  left_layout->setContentsMargins( 0, 0, 0, 0);

  // Top Label Area ------------------------------------------------

  HeaderWidget *header = new HeaderWidget( "Create Planning Groups",
                                           "Select planning groups for your robot based on kinematic chains or joint collections.",
                                           this);
  left_layout->addWidget( header );

  // Table Area --------------------------------------------  

  // Table Label
  QLabel *table_title = new QLabel( this );
  table_title->setText( "Current Groups" );
  QFont table_title_font( "Arial", 10, QFont::Bold );
  table_title->setFont(table_title_font);
  left_layout->addWidget( table_title);

  // Table
  groups_table_ = new QTableWidget( this );
  groups_table_->setColumnCount(3);
  //groups_table_->resizeColumnToContents(0);
  /*groups_table_->setColumnWidth(0, 300);
  groups_table_->setColumnWidth(1, 150);
  groups_table_->setColumnWidth(2, 85);*/
  groups_table_->setSortingEnabled(true);
  //connect(groups_table_, SIGNAL(cellChanged(int,int)), this, SLOT(toggleCheckBox(int,int)));
  left_layout->addWidget(groups_table_);

  // Table Headers
  QStringList header_list;
  header_list.append("Planning Group");
  header_list.append("Group Type");
  header_list.append("Category");
  groups_table_->setHorizontalHeaderLabels(header_list);
  groups_table_->resizeColumnToContents(0);
  groups_table_->resizeColumnToContents(1);
  groups_table_->resizeColumnToContents(2);
 
  // Bottom Area ----------------------------------------
  QHBoxLayout *controls_layout = new QHBoxLayout( );

  // Add Kinematics Chain Button
  QPushButton *btn_kinematics = new QPushButton( this );
  btn_kinematics->setText("Add &Kinematics Chain Group");
  btn_kinematics->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  //btn_kinematics->setMinimumWidth(300);
  connect(btn_kinematics, SIGNAL(clicked()), this, SLOT(addKinematicChainGroup()));
  controls_layout->addWidget(btn_kinematics);
  controls_layout->setAlignment(btn_kinematics, Qt::AlignRight);
  
  // Add Joint Colletion Group
  QPushButton *btn_joint = new QPushButton( this );
  btn_joint->setText("Add &Joint Collection Group");
  btn_joint->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  //btn_joint->setMinimumWidth(300);
  connect(btn_joint, SIGNAL(clicked()), this, SLOT(addJointCollectionGroup()));
  controls_layout->addWidget(btn_joint);
  controls_layout->setAlignment(btn_joint, Qt::AlignRight);

  // Delete Button
  QPushButton *btn_delete = new QPushButton( this );
  btn_delete->setText("&Delete Group");
  btn_delete->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  //btn_delete->setMinimumWidth(300);
  //connect(btn_delete, SIGNAL(clicked()), this, SLOT(addDeleteCollectionGroup()));
  controls_layout->addWidget(btn_delete);
  controls_layout->setAlignment(btn_delete, Qt::AlignRight);

  // Add Controls to layout
  left_layout->addLayout( controls_layout );

  // Rviz Right Side -------------------------------------
  QLabel *temp = new QLabel( "RVIZ", this );
  right_layout->addWidget( temp );

  // Split screen -----------------------------------------
  QWidget *left_frame = new QWidget( this );
  left_frame->setLayout(left_layout);

  QWidget *right_frame = new QWidget( this );
  right_frame->setLayout(right_layout);
  
  QSplitter *splitter = new QSplitter( Qt::Horizontal, this );
  splitter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  splitter->addWidget( left_frame );
  splitter->addWidget( right_frame );  
  layout->addWidget( splitter );

  setLayout(layout);
}

// ******************************************************************************************
// Displays data in the link_pairs_ data structure into a QtTableWidget
// ******************************************************************************************
void PlanningGroupsWidget::loadCollisionTable()
{
  int row = 0;
  groups_table_->setUpdatesEnabled(false); // prevent table from updating until we are completely done
  groups_table_->setDisabled(true); // make sure we disable it so that the cellChanged event is not called
  groups_table_->clearContents();
  /*
  // Intially set the table to be worst-case scenario of every possible element pair
  groups_table_->setRowCount( link_pairs_.size() ); 

  for ( moveit_configuration_tools::LinkPairMap::const_iterator pair_it = link_pairs_.begin(); 
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


void PlanningGroupsWidget::addKinematicChainGroup()
{
  std::cout << "ADD KIN CHAIN" << std::endl;
}

void PlanningGroupsWidget::addJointCollectionGroup()
{
  std::cout << "ADD JOINT COLL" << std::endl;
}
