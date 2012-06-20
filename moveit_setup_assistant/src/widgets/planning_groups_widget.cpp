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
#include <QApplication>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QTreeWidgetItem>
#include <QHeaderView>
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
  //  groups_tree_widget_ = new PlanningGroupsTableWidget( this, config_data_ );
  groups_tree_widget_ = createContentsWidget();

  joints_widget_ = new JointCollectionWidget( this, config_data_ );
  connect( joints_widget_, SIGNAL( doneEditing() ), this, SLOT( doneEditing() ) );

  // Combine into stack
  stacked_layout_ = new QStackedLayout( this );
  stacked_layout_->addWidget( groups_tree_widget_ );
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

  // process the gui
  QApplication::processEvents(); 

  // TODO: remove this demo
  //addJointCollectionGroup();
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

  // Tree Box
  groups_tree_ = new QTreeWidget( this );
  //groups_tree_->setColumnCount(2);
  //groups_tree_->setSortingEnabled(true);
  //connec(tgroups_tree_, SIGNAL(cellChanged(int,int)), this, SLOT(toggleCheckBox(int,int)));
  layout->addWidget(groups_tree_);

  /*
  // Table Headers
  QStringList header_list;
  header_list.append("Planning Group");
  header_list.append("Group Type");
  groups_tree_->setHorizontalHeaderLabels(header_list);
  groups_tree_->resizeColumnToContents(0);
  groups_tree_->resizeColumnToContents(1);
  */

  // Bottom Area ----------------------------------------
  /*
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
    btn_end_effector->setText("Add &End Effector");p
    btn_end_effector->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    bt;n_end_effector->setMinimumWidth(250);
    connect(btn_end_effector, SIGNAL(clicked()), this, SLOT(addEndEffector()));
    controls2_layout->addWidget(btn_end_effector);
  */


  QHBoxLayout *controls3_layout = new QHBoxLayout( );

  // Add Super Group Button
  QPushButton *btn_super_group = new QPushButton( "&Add Group", this );
  btn_super_group->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btn_super_group->setMinimumWidth(250);
  connect(btn_super_group, SIGNAL(clicked()), this, SLOT(addSuperGroup()));
  controls3_layout->addWidget(btn_super_group);
  controls3_layout->setAlignment( btn_super_group, Qt::AlignRight );

  
  // Add Controls to layout
  //layout->addLayout( controls1_layout );
  //layout->addLayout( controls2_layout );
  layout->addLayout( controls3_layout );

  // Set layout
  content_widget->setLayout(layout);

  return content_widget;
}

// ******************************************************************************************
// Displays data in the link_pairs_ data structure into a QtTableWidget
// ******************************************************************************************
void PlanningGroupsWidget::loadGroupsTree()
{
  std::cout << "LOADING " << std::endl;

  groups_tree_->setUpdatesEnabled(false); // prevent table from updating until we are completely done
  groups_tree_->setDisabled(true); // make sure we disable it so that the cellChanged event is not called


  // Display all groups by looping through them
  for( std::vector<srdf::Model::Group>::const_iterator group_it = config_data_->srdf_->groups_.begin(); 
       group_it != config_data_->srdf_->groups_.end();  ++group_it )
  {

    //addLinktoTreeRecursive( group_it, NULL );
    QTreeWidgetItem *toAdd = new QTreeWidgetItem( groups_tree_ );
    toAdd->setText( 0, group_it->name_.c_str() );
    groups_tree_->addTopLevelItem( toAdd );

  }

  //groups_tree_->expandToDepth(0);
  //groups_tree_->header()->setResizeMode(0, QHeaderView::ResizeToContents);
  //groups_tree_->header()->setStretchLastSection(false);

  // Reenable
  groups_tree_->setUpdatesEnabled(true); // prevent table from updating until we are completely done
  groups_tree_->setDisabled(false); // make sure we disable it so that the cellChanged event is not called

  // Resize table
  //groups_tree_->resizeColumnToContents(0);
  //groups_tree_->resizeColumnToContents(1);

}

void PlanningGroupsWidget::addJointCollectionGroup()
{
  std::cout << "ADD JOINT COLL" << std::endl;

  // Switch screens
  stacked_layout_->setCurrentIndex( 1 );

  // Load the available joints list
  joints_widget_->loadJoints();
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
  //loadGroupsTree();
}


void PlanningGroupsWidget::paintEvent( QPaintEvent * p )
{
  std::cout << "PAINTING" << std::endl;
  loadGroupsTree();
}
