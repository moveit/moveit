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
#include <QDebug>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <QString>
#include <QTreeWidgetItem>
#include <QHeaderView>
#include "ros/ros.h"
#include "header_widget.h"




// ******************************************************************************************
// Constructor
// ******************************************************************************************
PlanningGroupsWidget::PlanningGroupsWidget( QWidget *parent, moveit_setup_assistant::MoveItConfigDataPtr config_data )
  : SetupScreenWidget( parent ), config_data_(config_data)
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

// ******************************************************************************************
// Switch which screen is being shown
// ******************************************************************************************
void PlanningGroupsWidget::changeScreen( int index )
{
  stacked_layout_->setCurrentIndex( index );
}

// ******************************************************************************************
// Create the main tree view widget
// ******************************************************************************************
QWidget* PlanningGroupsWidget::createContentsWidget()
{
  // Main widget
  QWidget *content_widget = new QWidget( this );

  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout( this );


  // Tree Box ----------------------------------------------------------------------

  groups_tree_ = new QTreeWidget( this );
  groups_tree_->setHeaderLabel( "Current Groups" );
  //connec(tgroups_tree_, SIGNAL(cellChanged(int,int)), this, SLOT(toggleCheckBox(int,int)));
  layout->addWidget(groups_tree_);


  // Bottom Controls -------------------------------------------------------------
  
  QHBoxLayout *controls_layout = new QHBoxLayout( );

  // Expand/Contract controls
  QLabel *expand_controls = new QLabel( this );
  expand_controls->setText("<a href='expand'>Expand All</a> <a href='contract'>Collapse All</a>");
  connect( expand_controls, SIGNAL(linkActivated( const QString )), this, SLOT( alterTree( const QString )));
  controls_layout->addWidget( expand_controls );

  //  Edit Selected Button
  QPushButton *btn_edit = new QPushButton( "&Edit Selected", this );
  btn_edit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btn_edit->setMinimumWidth(200);
  connect(btn_edit, SIGNAL(clicked()), this, SLOT(editSelected()));
  controls_layout->addWidget(btn_edit);
  controls_layout->setAlignment( btn_edit, Qt::AlignRight );

  // Add Super Group Button
  QPushButton *btn_super_group = new QPushButton( "&Add Group", this );
  btn_super_group->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btn_super_group->setMinimumWidth(200);
  connect(btn_super_group, SIGNAL(clicked()), this, SLOT(addGroup()));
  controls_layout->addWidget(btn_super_group);
  controls_layout->setAlignment( btn_super_group, Qt::AlignRight );

  
  // Add Controls to layout
  layout->addLayout( controls_layout );

  // Set layout
  content_widget->setLayout(layout);

  return content_widget;
}

// ******************************************************************************************
// Displays data in the link_pairs_ data structure into a QtTableWidget
// ******************************************************************************************
void PlanningGroupsWidget::loadGroupsTree()
{
  // Disable Tree
  groups_tree_->setUpdatesEnabled(false); // prevent table from updating until we are completely done
  groups_tree_->setDisabled(true); // make sure we disable it so that the cellChanged event is not called
  groups_tree_->clear(); // reset the tree



  // Display all groups by looping through them
  for( std::vector<srdf::Model::Group>::const_iterator group_it = config_data_->srdf_->groups_.begin(); 
       group_it != config_data_->srdf_->groups_.end();  ++group_it )
  {
    addLinkToTreeRecursive( *group_it, NULL );
  }

  
  // Reenable Tree
  groups_tree_->setUpdatesEnabled(true); // prevent table from updating until we are completely done
  groups_tree_->setDisabled(false); // make sure we disable it so that the cellChanged event is not called
}

// ******************************************************************************************
// Recursively Adds Groups, and subgroups to groups...
// ******************************************************************************************
void PlanningGroupsWidget::addLinkToTreeRecursive( const srdf::Model::Group &group_it, QTreeWidgetItem *parent )
{

  // Fonts for tree
  const QFont top_level_font( "Arial", 11, QFont::Bold );
  const QFont type_font( "Arial", 11, QFont::Normal, QFont::StyleItalic );

  QTreeWidgetItem *group;

  // Allow a subgroup to open into a whole new group
  if( parent == NULL )
  {
    group = new QTreeWidgetItem( groups_tree_ );
    group->setText( 0, group_it.name_.c_str() );
    group->setFont( 0, top_level_font );
    qDebug() << "Adding top level item";
    groups_tree_->addTopLevelItem( group );
  }
  else
  {
    group= new QTreeWidgetItem( parent );
    group->setText( 0, group_it.name_.c_str() );
    group->setFont( 0, top_level_font );
    qDebug() << "Adding child";
    parent->addChild( group );
  }

  // Joints --------------------------------------------------------------
  QTreeWidgetItem *joints = new QTreeWidgetItem( group );
  joints->setText( 0, "Joints" );
  joints->setFont( 0, type_font );
  group->addChild( joints );
    
  // Loop through all aval. joints
  for( std::vector<std::string>::const_iterator joint_it = group_it.joints_.begin();
       joint_it != group_it.joints_.end(); ++joint_it )
  {
    QTreeWidgetItem *j = new QTreeWidgetItem( joints );
    j->setText( 0, joint_it->c_str() );
    joints->addChild( j );
  }

  // Links -------------------------------------------------------------
  QTreeWidgetItem *links = new QTreeWidgetItem( group );
  links->setText( 0, "Links" );
  links->setFont( 0, type_font );
  group->addChild( links );
    
  // Loop through all aval. links
  for( std::vector<std::string>::const_iterator joint_it = group_it.links_.begin();
       joint_it != group_it.links_.end(); ++joint_it )
  {
    QTreeWidgetItem *j = new QTreeWidgetItem( links );
    j->setText( 0, joint_it->c_str() );
    links->addChild( j );
  }

  // Chains -------------------------------------------------------------
  QTreeWidgetItem *chains = new QTreeWidgetItem( group );
  chains->setText( 0, "Chain" );
  chains->setFont( 0, type_font );
  group->addChild( chains );
    
  // Loop through all aval. chains
  for( std::vector<std::pair<std::string, std::string> >::const_iterator chain_it = group_it.chains_.begin();
       chain_it != group_it.chains_.end(); ++chain_it )
  {
    QTreeWidgetItem *j = new QTreeWidgetItem( chains );
    j->setText( 0, QString(chain_it->first.c_str() ).append("  ->  ").append( chain_it->second.c_str() ) );
    chains->addChild( j );
  }

  // Subgroups -------------------------------------------------------------
  QTreeWidgetItem *subgroups = new QTreeWidgetItem( group );
  subgroups->setText( 0, "Subgroups" );
  subgroups->setFont( 0, type_font );
  group->addChild( subgroups );
    
  // Loop through all aval. subgroups
  for( std::vector<std::string>::const_iterator subgroup_it = group_it.subgroups_.begin();
       subgroup_it != group_it.subgroups_.end(); ++subgroup_it )
  {
    // Find group with this subgroups' name

    const srdf::Model::Group *searched_group = NULL; // used for holding our search results

    for( std::vector<srdf::Model::Group>::const_iterator group2_it = config_data_->srdf_->groups_.begin();
         group2_it != config_data_->srdf_->groups_.end(); ++group2_it )
    {
      if( group2_it->name_ == *subgroup_it ) // this is the group we are looking for
      {
        searched_group = &(*group2_it); 
        break; // we are done searching
      }
    }

    
    // Check if subgroup was found
    if( searched_group == NULL ) // not found
    {
      QMessageBox::critical( this, "Error Loading SRDF", 
                             QString("Subgroup '").append( subgroup_it->c_str() ).append( "' of group '")
                             .append( group_it.name_.c_str() ).append( "' not found. Your SRDF is invalid" ));
      return; // TODO: something better for error handling?
    }

    // subgroup found!

    // Recurse this function for each new group
    addLinkToTreeRecursive( *searched_group, subgroups );
  }


}

// ******************************************************************************************
// Edit whatever element is selected in the tree view
// ******************************************************************************************
void PlanningGroupsWidget::editSelected()
{
  std::cout << "Edit Selected" << std::endl;
}

// ******************************************************************************************
// Create a new, empty group
// ******************************************************************************************
void PlanningGroupsWidget::addGroup()
{
  std::cout << "ADD GROUP" << std::endl;
}

// ******************************************************************************************
// Call when widget is ready to return to main screen
// ******************************************************************************************
void PlanningGroupsWidget::doneEditing()
{
  std::cout << "BACK" << std::endl;
  stacked_layout_->setCurrentIndex( 0 );
  loadGroupsTree();
}

// ******************************************************************************************
// Called when setup assistant navigation switches to this screen
// ******************************************************************************************
void PlanningGroupsWidget::focusGiven()
{
  // Show the current groups screen
  stacked_layout_->setCurrentIndex( 0 );

  // Load the data to the tree
  loadGroupsTree();
}

// ******************************************************************************************
// Expand/Collapse Tree
// ******************************************************************************************
void PlanningGroupsWidget::alterTree( const QString &link )
{
  if( link.contains("expand") )
    groups_tree_->expandAll();
  else
    groups_tree_->collapseAll();
}
