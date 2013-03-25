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

// ******************************************************************************************
/* DEVELOPER NOTES

   This widget has 6 subscreens, located in somewhat different places
   - Main screen, the tree view of all groups & subgroups - embedded in this file as a function
   - Add/Edit Group screen - located in group_edit_widget.cpp
   - Joint Collection Screen - implements the double_list_widget.cpp widget
   - Link Collection Screen - implements the double_list_widget.cpp widget
   - Kinematic Chain Screen - uses it own custom widget - kinematic_chain_widget.cpp
   - Subgroup Screen - implements the double_list_widget.cpp widget
*/
// ******************************************************************************************

#include "ros/ros.h"
#include "header_widget.h"
#include "planning_groups_widget.h"
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp> // for checking convertion of string to double
// Qt
#include <QApplication>
#include <QDebug>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <QString>
#include <QLineEdit>
#include <QTreeWidgetItem>
#include <QHeaderView>
// Cycle checking
#include <boost/utility.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/visitors.hpp>

namespace moveit_setup_assistant
{

// Name of rviz topic in ROS
static const std::string VIS_TOPIC_NAME = "planning_components_visualization";

// Used for checking for cycles in a subgroup hierarchy
struct cycle_detector : public boost::dfs_visitor<>
{
  cycle_detector(bool& has_cycle)
    : m_has_cycle(has_cycle) { }

  template <class Edge, class Graph>
  void back_edge(Edge, Graph&) { m_has_cycle = true; }
protected:
  bool& m_has_cycle;
};

// ******************************************************************************************
// Constructor
// ******************************************************************************************
PlanningGroupsWidget::PlanningGroupsWidget( QWidget *parent, moveit_setup_assistant::MoveItConfigDataPtr config_data )
  : SetupScreenWidget( parent ), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout();

  // Top Label Area ------------------------------------------------
  HeaderWidget *header = new HeaderWidget( "Planning Groups",
                                           "Create and edit planning groups for your robot based on joint collections, link collections, kinematic chains and subgroups.",
                                           this);
  layout->addWidget( header );

  // Left Side ---------------------------------------------

  // Create left side widgets
  groups_tree_widget_ = createContentsWidget(); // included in this file

  // Joints edit widget
  joints_widget_ = new DoubleListWidget( this, config_data_, "Joint Collection", "Joint" );
  connect( joints_widget_, SIGNAL( cancelEditing() ), this, SLOT( cancelEditing() ) );
  connect( joints_widget_, SIGNAL( doneEditing() ), this, SLOT( saveJointsScreen() ) );
  connect( joints_widget_, SIGNAL( previewSelected( std::vector<std::string> ) ),
           this, SLOT( previewSelectedJoints( std::vector<std::string> ) ) );

  // Links edit widget
  links_widget_ = new DoubleListWidget( this, config_data_, "Link Collection", "Link" );
  connect( links_widget_, SIGNAL( cancelEditing() ), this, SLOT( cancelEditing() ) );
  connect( links_widget_, SIGNAL( doneEditing() ), this, SLOT( saveLinksScreen() ) );
  connect( links_widget_, SIGNAL( previewSelected( std::vector<std::string> ) ),
           this, SLOT( previewSelectedLink( std::vector<std::string> ) ) );

  // Chain Widget
  chain_widget_ = new KinematicChainWidget( this, config_data );
  connect( chain_widget_, SIGNAL( cancelEditing() ), this, SLOT( cancelEditing() ) );
  connect( chain_widget_, SIGNAL( doneEditing() ), this, SLOT( saveChainScreen() ) );
  connect( chain_widget_, SIGNAL( unhighlightAll() ), this, SIGNAL( unhighlightAll() ) );
  connect( chain_widget_, SIGNAL( highlightLink(const std::string&)), this, SIGNAL(highlightLink(const std::string&)) );

  // Subgroups Widget
  subgroups_widget_ = new DoubleListWidget( this, config_data_, "Subgroup", "Subgroup" );
  connect( subgroups_widget_, SIGNAL( cancelEditing() ), this, SLOT( cancelEditing() ) );
  connect( subgroups_widget_, SIGNAL( doneEditing() ), this, SLOT( saveSubgroupsScreen() ) );
  connect( subgroups_widget_, SIGNAL( previewSelected( std::vector<std::string> ) ),
           this, SLOT( previewSelectedSubgroup( std::vector<std::string> ) ) );

  // Group Edit Widget
  group_edit_widget_ = new GroupEditWidget( this, config_data_ );
  connect( group_edit_widget_, SIGNAL( cancelEditing() ), this, SLOT( cancelEditing() ) );
  connect( group_edit_widget_, SIGNAL( deleteGroup() ), this, SLOT( deleteGroup() ) );
  connect( group_edit_widget_, SIGNAL( save() ), this, SLOT( saveGroupScreenEdit() ) );
  connect( group_edit_widget_, SIGNAL( saveJoints() ), this, SLOT( saveGroupScreenJoints() ) );
  connect( group_edit_widget_, SIGNAL( saveLinks() ), this, SLOT( saveGroupScreenLinks() ) );
  connect( group_edit_widget_, SIGNAL( saveChain() ), this, SLOT( saveGroupScreenChain() ) );
  connect( group_edit_widget_, SIGNAL( saveSubgroups() ), this, SLOT( saveGroupScreenSubgroups() ) );


  // Combine into stack
  stacked_layout_ = new QStackedLayout( this );
  stacked_layout_->addWidget( groups_tree_widget_ ); // screen index 0
  stacked_layout_->addWidget( joints_widget_ ); // screen index 1
  stacked_layout_->addWidget( links_widget_ ); // screen index 2
  stacked_layout_->addWidget( chain_widget_ ); // screen index 3
  stacked_layout_->addWidget( subgroups_widget_ ); // screen index 4
  stacked_layout_->addWidget( group_edit_widget_ ); // screen index 5

  showMainScreen();

  // Finish GUI -----------------------------------------------------------

  // Create Widget wrapper for layout
  QWidget *stacked_layout_widget = new QWidget( this );
  stacked_layout_widget->setLayout( stacked_layout_ );

  layout->addWidget( stacked_layout_widget );

  setLayout(layout);

  // process the gui
  QApplication::processEvents();
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
  connect( groups_tree_, SIGNAL( itemDoubleClicked( QTreeWidgetItem*, int) ), this, SLOT(editSelected()));
  connect( groups_tree_, SIGNAL( itemClicked( QTreeWidgetItem*, int) ), this, SLOT(previewSelected()));
  layout->addWidget(groups_tree_);


  // Bottom Controls -------------------------------------------------------------

  QHBoxLayout *controls_layout = new QHBoxLayout( );

  // Expand/Contract controls
  QLabel *expand_controls = new QLabel( this );
  expand_controls->setText("<a href='expand'>Expand All</a> <a href='contract'>Collapse All</a>");
  connect( expand_controls, SIGNAL(linkActivated( const QString )), this, SLOT( alterTree( const QString )));
  controls_layout->addWidget( expand_controls );

  // Spacer
  QWidget *spacer = new QWidget( this );
  spacer->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
  controls_layout->addWidget( spacer );

  // Delete Selected Button
  btn_delete_ = new QPushButton( "&Delete Selected", this );
  btn_delete_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred );
  btn_delete_->setMaximumWidth( 300 );
  connect( btn_delete_, SIGNAL(clicked()), this, SLOT( deleteGroup() ) );
  controls_layout->addWidget( btn_delete_ );
  controls_layout->setAlignment(btn_delete_, Qt::AlignRight);

  //  Edit Selected Button
  btn_edit_ = new QPushButton( "&Edit Selected", this );
  btn_edit_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred );
  btn_edit_->setMaximumWidth(300);
  btn_edit_->hide(); // show once we know if there are existing groups
  connect(btn_edit_, SIGNAL(clicked()), this, SLOT(editSelected()));
  controls_layout->addWidget(btn_edit_);
  controls_layout->setAlignment( btn_edit_, Qt::AlignRight );

  // Add Group Button
  QPushButton *btn_add = new QPushButton( "&Add Group", this );
  btn_add->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred );
  btn_add->setMaximumWidth(300);
  connect(btn_add, SIGNAL(clicked()), this, SLOT(addGroup()));
  controls_layout->addWidget(btn_add);
  controls_layout->setAlignment( btn_add, Qt::AlignRight );


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
  for( std::vector<srdf::Model::Group>::iterator group_it = config_data_->srdf_->groups_.begin();
       group_it != config_data_->srdf_->groups_.end();  ++group_it )
  {
    loadGroupsTreeRecursive( *group_it, NULL );
  }

  // Reenable Tree
  groups_tree_->setUpdatesEnabled(true); // prevent table from updating until we are completely done
  groups_tree_->setDisabled(false); // make sure we disable it so that the cellChanged event is not called

  // Show Edit button if there are things to edit
  if( !config_data_->srdf_->groups_.empty() )
  {
    btn_edit_->show();
    btn_delete_->show();
  }
  else
  {
    btn_edit_->hide();
    btn_delete_->hide();
  }

  alterTree( "expand" );
}

// ******************************************************************************************
// Recursively Adds Groups, and subgroups to groups...
// ******************************************************************************************
void PlanningGroupsWidget::loadGroupsTreeRecursive( srdf::Model::Group &group_it, QTreeWidgetItem *parent )
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
    group->setData( 0, Qt::UserRole, QVariant::fromValue( PlanGroupType( &group_it, GROUP ) ) );
    groups_tree_->addTopLevelItem( group );
  }
  else
  {
    group= new QTreeWidgetItem( parent );
    group->setText( 0, group_it.name_.c_str() );
    group->setFont( 0, top_level_font );
    group->setData( 0, Qt::UserRole, QVariant::fromValue( PlanGroupType( &group_it, GROUP ) ) );
    parent->addChild( group );
  }

  // Joints --------------------------------------------------------------
  QTreeWidgetItem *joints = new QTreeWidgetItem( group );
  joints->setText( 0, "Joints" );
  joints->setFont( 0, type_font );
  joints->setData( 0, Qt::UserRole, QVariant::fromValue( PlanGroupType( &group_it, JOINT ) ) );
  group->addChild( joints );

  // Retrieve pointer to the shared kinematic model
  const robot_model::RobotModelConstPtr &model = config_data_->getRobotModel();

  // Loop through all aval. joints
  for( std::vector<std::string>::const_iterator joint_it = group_it.joints_.begin();
       joint_it != group_it.joints_.end(); ++joint_it )
  {
    QTreeWidgetItem *j = new QTreeWidgetItem( joints );
    j->setData( 0, Qt::UserRole, QVariant::fromValue( PlanGroupType( &group_it, JOINT ) ) );
    std::string joint_name;

    // Get the type of joint this is
    const robot_model::JointModel* jm = model->getJointModel(*joint_it);
    if(jm) // check if joint model was found
    {
      joint_name = *joint_it + " - " + jm->getTypeName();
    }
    else
    {
      joint_name = *joint_it;
    }

    // Add to tree
    j->setText( 0, joint_name.c_str() );
    joints->addChild( j );
  }

  // Links -------------------------------------------------------------
  QTreeWidgetItem *links = new QTreeWidgetItem( group );
  links->setText( 0, "Links" );
  links->setFont( 0, type_font );
  links->setData( 0, Qt::UserRole, QVariant::fromValue( PlanGroupType( &group_it, LINK ) ) );
  group->addChild( links );

  // Loop through all aval. links
  for( std::vector<std::string>::const_iterator joint_it = group_it.links_.begin();
       joint_it != group_it.links_.end(); ++joint_it )
  {
    QTreeWidgetItem *j = new QTreeWidgetItem( links );
    j->setData( 0, Qt::UserRole, QVariant::fromValue( PlanGroupType( &group_it, LINK ) ) );
    j->setText( 0, joint_it->c_str() );
    links->addChild( j );
  }

  // Chains -------------------------------------------------------------
  QTreeWidgetItem *chains = new QTreeWidgetItem( group );
  chains->setText( 0, "Chain" );
  chains->setFont( 0, type_font );
  chains->setData( 0, Qt::UserRole, QVariant::fromValue( PlanGroupType( &group_it, CHAIN ) ) );
  group->addChild( chains );

  // Warn if there is more than 1 chain per group
  static bool warn_once = true;
  if( group_it.chains_.size() > 1 && warn_once )
  {
    warn_once = false;
    QMessageBox::warning( this, "Group with Multiple Kinematic Chains", "Warning: this MoveIt Setup Assistant is only designed to handle one kinematic chain per group. The loaded SRDF has more than one kinematic chain for a group. A possible loss of data may occur.");
  }

  // Loop through all aval. chains
  for( std::vector<std::pair<std::string, std::string> >::const_iterator chain_it = group_it.chains_.begin();
       chain_it != group_it.chains_.end(); ++chain_it )
  {
    QTreeWidgetItem *j = new QTreeWidgetItem( chains );
    j->setData( 0, Qt::UserRole, QVariant::fromValue( PlanGroupType( &group_it, CHAIN ) ) );
    j->setText( 0, QString(chain_it->first.c_str() ).append("  ->  ").append( chain_it->second.c_str() ) );
    chains->addChild( j );
  }

  // Subgroups -------------------------------------------------------------
  QTreeWidgetItem *subgroups = new QTreeWidgetItem( group );
  subgroups->setText( 0, "Subgroups" );
  subgroups->setFont( 0, type_font );
  subgroups->setData( 0, Qt::UserRole, QVariant::fromValue( PlanGroupType( &group_it, SUBGROUP ) ) );
  group->addChild( subgroups );

  // Loop through all aval. subgroups
  for( std::vector<std::string>::iterator subgroup_it = group_it.subgroups_.begin();
       subgroup_it != group_it.subgroups_.end(); ++subgroup_it )
  {
    // Find group with this subgroups' name

    srdf::Model::Group *searched_group = NULL; // used for holding our search results

    for( std::vector<srdf::Model::Group>::iterator group2_it = config_data_->srdf_->groups_.begin();
         group2_it != config_data_->srdf_->groups_.end(); ++group2_it )
    {
      if( group2_it->name_ == *subgroup_it ) // this is the group we are looking for
      {
        searched_group = &(*group2_it);  // convert to pointer from iterator
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
    loadGroupsTreeRecursive( *searched_group, subgroups );
  }


}

// ******************************************************************************************
// Highlight the group of whatever element is selected in the tree view
// ******************************************************************************************
void PlanningGroupsWidget::previewSelected()
{
  QTreeWidgetItem* item = groups_tree_->currentItem();

  // Check that something was actually selected
  if(item == NULL)
    return;

  // Get the user custom properties of the currently selected row
  PlanGroupType plan_group = item->data( 0, Qt::UserRole ).value<PlanGroupType>();

  // Unhighlight all links
  Q_EMIT unhighlightAll();

  // Highlight the group
  Q_EMIT( highlightGroup( plan_group.group_->name_ ) );
}

// ******************************************************************************************
// Edit whatever element is selected in the tree view
// ******************************************************************************************
void PlanningGroupsWidget::editSelected()
{
  QTreeWidgetItem* item = groups_tree_->currentItem();

  // Check that something was actually selected
  if(item == NULL)
    return;

  adding_new_group_ = false;

  // Get the user custom properties of the currently selected row
  PlanGroupType plan_group = item->data( 0, Qt::UserRole ).value<PlanGroupType>();

  if( plan_group.type_ == JOINT )
  {
    // Load the data
    loadJointsScreen( plan_group.group_ );

    // Switch to screen
    changeScreen( 1 ); // 1 is index of joints
  }
  else if( plan_group.type_ == LINK )
  {
    // Load the data
    loadLinksScreen( plan_group.group_ );

    // Switch to screen
    changeScreen( 2 );
  }
  else if( plan_group.type_ == CHAIN )
  {
    // Load the data
    loadChainScreen( plan_group.group_ );

    // Switch to screen
    changeScreen( 3 );
  }
  else if( plan_group.type_ == SUBGROUP )
  {
    // Load the data
    loadSubgroupsScreen( plan_group.group_ );

    // Switch to screen
    changeScreen( 4 );
  }
  else if( plan_group.type_ == GROUP )
  {
    // Load the data
    loadGroupScreen( plan_group.group_ );

    // Switch to screen
    changeScreen( 5 );
  }
  else
  {
    QMessageBox::critical( this, "Error Loading", "An internal error has occured while loading.");
  }
}

// ******************************************************************************************
// Load the popup screen with correct data for joints
// ******************************************************************************************
void PlanningGroupsWidget::loadJointsScreen( srdf::Model::Group *this_group )
{
  // Retrieve pointer to the shared kinematic model
  const robot_model::RobotModelConstPtr &model = config_data_->getRobotModel();

  // Get the names of the all joints
  const std::vector<std::string> &joints = model->getJointModelNames();

  if( joints.size() == 0 )
  {
    QMessageBox::critical( this, "Error Loading", "No joints found for robot model");
    return;
  }

  // Set the available joints (left box)
  joints_widget_->setAvailable( joints );

  // Set the selected joints (right box)
  joints_widget_->setSelected( this_group->joints_ );

  // Set the title
  joints_widget_->title_->setText( QString("Edit '").append( QString::fromUtf8( this_group->name_.c_str() ) )
                                   .append( "' Joint Collection") );

  // Remember what is currently being edited so we can later save changes
  current_edit_group_ = this_group->name_;
  current_edit_element_ = JOINT;
}

// ******************************************************************************************
// Load the popup screen with correct data for links
// ******************************************************************************************
void PlanningGroupsWidget::loadLinksScreen( srdf::Model::Group *this_group )
{
  // Retrieve pointer to the shared kinematic model
  const robot_model::RobotModelConstPtr &model = config_data_->getRobotModel();

  // Get the names of the all links
  const std::vector<std::string> &links = model->getLinkModelNames();

  if( links.size() == 0 )
  {
    QMessageBox::critical( this, "Error Loading", "No links found for robot model");
    return;
  }

  // Set the available links (left box)
  links_widget_->setAvailable( links );

  // Set the selected links (right box)
  links_widget_->setSelected( this_group->links_ );

  // Set the title
  links_widget_->title_->setText( QString("Edit '").append( QString::fromUtf8( this_group->name_.c_str() ) )
                                  .append( "' Link Collection") );

  // Remember what is currently being edited so we can later save changes
  current_edit_group_ = this_group->name_;
  current_edit_element_ = LINK;
}

// ******************************************************************************************
// Load the popup screen with correct data for chains
// ******************************************************************************************
void PlanningGroupsWidget::loadChainScreen( srdf::Model::Group *this_group )
{
  // Tell the kin chain widget to load up the chain from a kinematic model
  chain_widget_->setAvailable();

  // Make sure there isn't more than 1 chain pair
  if( this_group->chains_.size() > 1 )
  {
    QMessageBox::warning( this, "Multiple Kinematic Chains", "Warning: This setup assistant is only designed to handle one kinematic chain per group. The loaded SRDF has more than one kinematic chain for a group. A possible loss of data may occur.");
  }

  // Set the selected tip and base of chain if one exists
  if( this_group->chains_.size() > 0 )
  {
    chain_widget_->setSelected( this_group->chains_[0].first, this_group->chains_[0].second );
  }

  // Set the title
  chain_widget_->title_->setText( QString("Edit '").append( QString::fromUtf8( this_group->name_.c_str() ) )
                                  .append( "' Kinematic Chain") );

  // Remember what is currently being edited so we can later save changes
  current_edit_group_ = this_group->name_;
  current_edit_element_ = CHAIN;
}

// ******************************************************************************************
// Load the popup screen with correct data for subgroups
// ******************************************************************************************
void PlanningGroupsWidget::loadSubgroupsScreen( srdf::Model::Group *this_group )
{
  // Load all groups into the subgroup screen except the current group
  std::vector<std::string> subgroups;

  // Display all groups by looping through them
  for( std::vector<srdf::Model::Group>::iterator group_it = config_data_->srdf_->groups_.begin();
       group_it != config_data_->srdf_->groups_.end();  ++group_it )
  {
    if( group_it->name_ != this_group->name_ ) //  do not include current group
    {
      // add to available subgroups list
      subgroups.push_back( group_it->name_ );
    }
  }

  // Set the available subgroups (left box)
  subgroups_widget_->setAvailable( subgroups );

  // Set the selected subgroups (right box)
  subgroups_widget_->setSelected( this_group->subgroups_ );

  // Set the title
  subgroups_widget_->title_->setText( QString("Edit '").append( QString::fromUtf8( this_group->name_.c_str() ) )
                                      .append( "' Subgroups") );

  // Remember what is currently being edited so we can later save changes
  current_edit_group_ = this_group->name_;
  current_edit_element_ = SUBGROUP;
}

// ******************************************************************************************
// Load the popup screen with correct data for groups
// ******************************************************************************************
void PlanningGroupsWidget::loadGroupScreen( srdf::Model::Group *this_group )
{
  // Load the avail kin solvers. This function only runs once
  group_edit_widget_->loadKinematicPlannersComboBox();

  if( this_group == NULL ) // this is a new screen
  {
    current_edit_group_.clear(); // provide a blank group name
    group_edit_widget_->title_->setText( "Create New Planning Group" );
    group_edit_widget_->btn_delete_->hide();
    group_edit_widget_->new_buttons_widget_->show(); // helps user choose next step
    group_edit_widget_->btn_save_->hide(); // this is only for edit mode
  }
  else // load the group name into the widget
  {
    current_edit_group_ = this_group->name_;
    group_edit_widget_->title_->setText( QString("Edit Planning Group '")
                                         .append( current_edit_group_.c_str() ).append("'") );
    group_edit_widget_->btn_delete_->show();
    group_edit_widget_->new_buttons_widget_->hide(); // not necessary for existing groups
    group_edit_widget_->btn_save_->show(); // this is only for edit mode
  }

  // Set the data in the edit box
  group_edit_widget_->setSelected( current_edit_group_ );


  // Remember what is currently being edited so we can later save changes
  current_edit_element_ = GROUP;
}

// ******************************************************************************************
// Delete a group
// ******************************************************************************************
void PlanningGroupsWidget::deleteGroup()
{
  std::string group = current_edit_group_;
  if (group.empty())
  {
    QTreeWidgetItem* item = groups_tree_->currentItem();
    // Check that something was actually selected
    if(item == NULL)
      return;
    // Get the user custom properties of the currently selected row
    PlanGroupType plan_group = item->data( 0, Qt::UserRole ).value<PlanGroupType>();
    if (plan_group.group_)
      group = plan_group.group_->name_;
  }
  else
    current_edit_group_.clear();
  if (group.empty())
    return;

  // Find the group we are editing based on the goup name string
  srdf::Model::Group *searched_group = config_data_->findGroupByName( group );

  // Confirm user wants to delete group
  if( QMessageBox::question( this, "Confirm Group Deletion",
                             QString("Are you sure you want to delete the planning group '")
                             .append( searched_group->name_.c_str() )
                             .append( "'? This will also delete all references in subgroups, robot poses and end effectors." ),
                             QMessageBox::Ok | QMessageBox::Cancel)
      == QMessageBox::Cancel )
  {
    return;
  }

  // delete all robot poses that use that group's name
  bool haveConfirmedGroupStateDeletion = false;
  bool haveDeletedGroupState = true;
  while( haveDeletedGroupState )
  {
    haveDeletedGroupState = false;
    for( std::vector<srdf::Model::GroupState>::iterator pose_it = config_data_->srdf_->group_states_.begin();
         pose_it != config_data_->srdf_->group_states_.end(); ++pose_it )
    {
      // check if this group state depends on the currently being deleted group
      if( pose_it->group_ == searched_group->name_ )
      {
        if( !haveConfirmedGroupStateDeletion )
        {
          haveConfirmedGroupStateDeletion = true;

          // confirm the user wants to delete group states
          if( QMessageBox::question( this, "Confirm Group State Deletion",
                                     QString("The group that is about to be deleted has robot poses (robot states) that depend on this group. Are you sure you want to delete this group as well as all dependend robot poses?"),
                                     QMessageBox::Ok | QMessageBox::Cancel)
              == QMessageBox::Cancel )
          {
            return;
          }
        }

        // the user has confirmed, now delete this group state
        config_data_->srdf_->group_states_.erase( pose_it );
        haveDeletedGroupState = true;
        break; // you can only delete 1 item in vector before invalidating iterator
      }
    }
  }

  // delete all end effectors that use that group's name
  bool haveConfirmedEndEffectorDeletion = false;
  bool haveDeletedEndEffector = true;
  while( haveDeletedEndEffector )
  {
    haveDeletedEndEffector = false;
    for( std::vector<srdf::Model::EndEffector>::iterator effector_it = config_data_->srdf_->end_effectors_.begin();
         effector_it != config_data_->srdf_->end_effectors_.end(); ++effector_it )
    {
      // check if this group state depends on the currently being deleted group
      if( effector_it->component_group_ == searched_group->name_ )
      {
        if( !haveConfirmedEndEffectorDeletion )
        {
          haveConfirmedEndEffectorDeletion = true;

          // confirm the user wants to delete group states
          if( QMessageBox::question( this, "Confirm End Effector Deletion",
                                     QString("The group that is about to be deleted has end effectors (grippers) that depend on this group. Are you sure you want to delete this group as well as all dependend end effectors?"),
                                     QMessageBox::Ok | QMessageBox::Cancel)
              == QMessageBox::Cancel )
          {
            return;
          }
        }

        // the user has confirmed, now delete this group state
        config_data_->srdf_->end_effectors_.erase( effector_it );
        haveDeletedEndEffector = true;
        break; // you can only delete 1 item in vector before invalidating iterator
      }
    }
  }

  // delete actual group
  for( std::vector<srdf::Model::Group>::iterator group_it = config_data_->srdf_->groups_.begin();
       group_it != config_data_->srdf_->groups_.end(); ++group_it )
  {
    // check if this is the group we want to delete
    if( group_it->name_ == group ) // string match
    {
      config_data_->srdf_->groups_.erase( group_it );
      break;
    }
  }

  // loop again to delete all subgroup references
  for( std::vector<srdf::Model::Group>::iterator group_it = config_data_->srdf_->groups_.begin();
       group_it != config_data_->srdf_->groups_.end(); ++group_it )
  {
    // delete all subgroup references
    bool deleted_subgroup = true;
    while( deleted_subgroup )
    {
      deleted_subgroup = false;

      // check if the subgroups reference our deleted group
      for( std::vector<std::string>::iterator subgroup_it = group_it->subgroups_.begin();
           subgroup_it != group_it->subgroups_.end(); ++subgroup_it )
      {
        // Check if that subgroup references the deletion group. if so, delete it
        if( subgroup_it->compare( group ) == 0 ) // same name
        {
          group_it->subgroups_.erase( subgroup_it ); // delete
          deleted_subgroup = true;
          break;
        }
      }
    }
  }

  // Switch to main screen
  showMainScreen();

  // Reload main screen table
  loadGroupsTree();

  // Update the kinematic model with changes
  config_data_->updateRobotModel();
}

// ******************************************************************************************
// Create a new, empty group
// ******************************************************************************************
void PlanningGroupsWidget::addGroup()
{
  adding_new_group_ = true;

  // Load the data
  loadGroupScreen( NULL ); // NULL indicates this is a new group, not an existing one

  // Switch to screen
  changeScreen( 5 );
}


// ******************************************************************************************
// Call when joints edit sceen is done and needs to be saved
// ******************************************************************************************
void PlanningGroupsWidget::saveJointsScreen()
{
  // Find the group we are editing based on the goup name string
  srdf::Model::Group *searched_group = config_data_->findGroupByName( current_edit_group_ );

  // clear the old data
  searched_group->joints_.clear();

  // copy the data
  for( int i = 0; i < joints_widget_->selected_data_table_->rowCount(); ++i )
  {
    searched_group->joints_.push_back( joints_widget_->selected_data_table_->item( i, 0 )->text().toStdString() );
  }

  // Switch to main screen
  showMainScreen();

  // Reload main screen table
  loadGroupsTree();

  // Update the kinematic model with changes
  config_data_->updateRobotModel();
}

// ******************************************************************************************
// Call when links edit sceen is done and needs to be saved
// ******************************************************************************************
void PlanningGroupsWidget::saveLinksScreen()
{
  // Find the group we are editing based on the goup name string
  srdf::Model::Group *searched_group = config_data_->findGroupByName( current_edit_group_ );

  // Find the group we are editing based on the goup name string
  // clear the old data
  searched_group->links_.clear();

  // copy the data
  for( int i = 0; i < links_widget_->selected_data_table_->rowCount(); ++i )
  {
    searched_group->links_.push_back( links_widget_->selected_data_table_->item( i, 0 )->text().toStdString() );
  }

  // Switch to main screen
  showMainScreen();

  // Reload main screen table
  loadGroupsTree();

  // Update the kinematic model with changes
  config_data_->updateRobotModel();
}

// ******************************************************************************************
// Call when chains edit sceen is done and needs to be saved
// ******************************************************************************************
void PlanningGroupsWidget::saveChainScreen()
{
  // Find the group we are editing based on the goup name string
  srdf::Model::Group *searched_group = config_data_->findGroupByName( current_edit_group_ );

  // Get a reference to the supplied strings
  const std::string &tip = chain_widget_->tip_link_field_->text().toStdString();
  const std::string &base = chain_widget_->base_link_field_->text().toStdString();

  // Check that box the tip and base, or neither, have text
  if( ( !tip.empty() && base.empty() ) ||
      ( tip.empty() && !base.empty() ) )
  {
    QMessageBox::warning( this, "Error Saving", "You must specify a link for both the base and tip, or leave both blank.");
    return;
  }

  // Check that both given links are valid links, unless they are both blank
  if( !tip.empty() && !base.empty() )
  {
    // Check that they are not the same link
    if( tip.compare( base ) == 0 ) // they are same
    {
      QMessageBox::warning( this, "Error Saving", "Tip and base link cannot be the same link.");
      return;
    }

    bool found_tip = false;
    bool found_base = false;
    const std::vector<std::string> &links = config_data_->getRobotModel()->getLinkModelNames();

    for( std::vector<std::string>::const_iterator link_it = links.begin(); link_it != links.end(); ++link_it )
    {
      // Check if string matches either of user specefied links
      if( link_it->compare(tip) == 0) // they are same
        found_tip = true;
      else if( link_it->compare(base) == 0) // they are same
        found_base = true;

      // Check if we are done searching
      if( found_tip && found_base )
        break;
    }

    // Check if we found both links
    if( !found_tip || !found_base )
    {
      QMessageBox::warning( this, "Error Saving", "Tip or base link(s) were not found in kinematic chain.");
      return;
    }

  }

  // clear the old data
  searched_group->chains_.clear();

  // Save the data if there is data to save
  if( !tip.empty() && !base.empty() )
  {
    searched_group->chains_.push_back( std::pair<std::string,std::string>( base, tip ) );
  }

  // Switch to main screen
  showMainScreen();

  // Reload main screen table
  loadGroupsTree();

  // Update the kinematic model with changes
  config_data_->updateRobotModel();
}

// ******************************************************************************************
// Call when subgroups edit sceen is done and needs to be saved
// ******************************************************************************************
void PlanningGroupsWidget::saveSubgroupsScreen()
{
  // Find the group we are editing based on the goup name string
  srdf::Model::Group *searched_group = config_data_->findGroupByName( current_edit_group_ );

  // Check for cycles -------------------------------

  // Create vector index of all nodes
  std::map<std::string,int> group_nodes;

  // Create vector of all nodes for use as id's
  int node_id = 0;
  for( std::vector<srdf::Model::Group>::iterator group_it = config_data_->srdf_->groups_.begin();
       group_it != config_data_->srdf_->groups_.end(); ++group_it )
  {
    // Add string to vector
    group_nodes.insert( std::pair<std::string,int>(group_it->name_, node_id) );
    ++node_id;
  }

  // Create the empty graph
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS> Graph;
  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
  Graph g( group_nodes.size() );


  // Traverse the group list again, this time inserting subgroups into graph
  int from_id = 0; // track the outer node we are on to reduce searches performed
  for( std::vector<srdf::Model::Group>::iterator group_it = config_data_->srdf_->groups_.begin();
       group_it != config_data_->srdf_->groups_.end(); ++group_it )
  {

    // Check if group_it is same as current group
    if( group_it->name_ == searched_group->name_ ) // yes, same group
    {

      // add new subgroup list from widget, not old one. this way we can check for new cycles
      for( int i = 0; i < subgroups_widget_->selected_data_table_->rowCount(); ++i )
      {
        // Get std::string of subgroup
        const std::string to_string = subgroups_widget_->selected_data_table_->item( i, 0 )->text().toStdString();

        // convert subgroup string to associated id
        int to_id = group_nodes[ to_string ];

        // Add edge from from_id to to_id
        add_edge( from_id, to_id, g);
      }
    }
    else //this group is not the group we are editing, so just add subgroups from memory
    {
      // add new subgroup list from widget, not old one. this way we can check for new cycles
      for( unsigned int i = 0; i < group_it->subgroups_.size(); ++i )
      {
        // Get std::string of subgroup
        const std::string to_string = group_it->subgroups_.at(i);

        // convert subgroup string to associated id
        int to_id = group_nodes[ to_string ];

        // Add edge from from_id to to_id
        add_edge( from_id, to_id, g);
      }
    }

    ++from_id;
  }

  // Check for cycles
  bool has_cycle = false;
  cycle_detector vis(has_cycle);
  boost::depth_first_search(g, visitor(vis));

  if( has_cycle )
  {
    // report to user the error
    QMessageBox::warning( this, "Error Saving", "Depth first search reveals a cycle in the subgroups");
    return;
  }

  // clear the old data
  searched_group->subgroups_.clear();

  // copy the data
  for( int i = 0; i < subgroups_widget_->selected_data_table_->rowCount(); ++i )
  {
    searched_group->subgroups_.push_back( subgroups_widget_->selected_data_table_->item( i, 0 )->text().toStdString() );
  }

  // Switch to main screen
  showMainScreen();

  // Reload main screen table
  loadGroupsTree();

  // Update the kinematic model with changes
  config_data_->updateRobotModel();
}

// ******************************************************************************************
// Call when groups edit sceen is done and needs to be saved
// ******************************************************************************************
bool PlanningGroupsWidget::saveGroupScreen()
{
  // Get a reference to the supplied strings
  const std::string &group_name = group_edit_widget_->group_name_field_->text().toStdString();
  const std::string &kinematics_solver = group_edit_widget_->kinematics_solver_field_->currentText().toStdString();
  const std::string &kinematics_resolution = group_edit_widget_->kinematics_resolution_field_->text().toStdString();
  const std::string &kinematics_timeout = group_edit_widget_->kinematics_timeout_field_->text().toStdString();
  const std::string &kinematics_attempts = group_edit_widget_->kinematics_attempts_field_->text().toStdString();

  // Used for editing existing groups
  srdf::Model::Group *searched_group = NULL;

  // Check that a valid group name has been given
  if( group_name.empty() )
  {
    QMessageBox::warning( this, "Error Saving", "A name must be given for the group!" );
    return false;
  }

  // Check if this is an existing group
  if( !current_edit_group_.empty() )
  {
    // Find the group we are editing based on the goup name string
    searched_group = config_data_->findGroupByName( current_edit_group_ );
  }

  // Check that the group name is unique
  for( std::vector<srdf::Model::Group>::const_iterator group_it = config_data_->srdf_->groups_.begin();
       group_it != config_data_->srdf_->groups_.end();  ++group_it )
  {

    if( group_it->name_.compare( group_name ) == 0 ) // the names are the same
    {
      // is this our existing group? check if group pointers are same
      if( &(*group_it) != searched_group )
      {
        QMessageBox::warning( this, "Error Saving", "A group already exists with that name!" );
        return false;
      }
    }
  }

  // Check that the resolution is an double number
  double kinematics_resolution_double;
  try
  {
    kinematics_resolution_double = boost::lexical_cast<double>(kinematics_resolution);
  }
  catch( boost::bad_lexical_cast& )
  {
    QMessageBox::warning( this, "Error Saving", "Unable to convert kinematics resolution to a double number." );
    return false;
  }

  // Check that the timeout is a double number
  double kinematics_timeout_double;
  try
  {
    kinematics_timeout_double = boost::lexical_cast<double>(kinematics_timeout);
  }
  catch( boost::bad_lexical_cast& )
  {
    QMessageBox::warning( this, "Error Saving", "Unable to convert kinematics solver timeout to a double number." );
    return false;
  }

  // Check that the attempts is an int number
  int kinematics_attempts_int;
  try
  {
    kinematics_attempts_int = boost::lexical_cast<int>(kinematics_attempts);
  }
  catch( boost::bad_lexical_cast& )
  {
    QMessageBox::warning( this, "Error Saving", "Unable to convert kinematics solver attempts to an int number." );
    return false;
  }

  // Check that all numbers are >0
  if(kinematics_resolution_double <= 0)
  {
    QMessageBox::warning( this, "Error Saving", "Kinematics solver search resolution must be greater than 0." );
    return false;
  }
  if(kinematics_timeout_double <= 0)
  {
    QMessageBox::warning( this, "Error Saving", "Kinematics solver search timeout must be greater than 0." );
    return false;
  }
  if(kinematics_attempts_int <= 0)
  {
    QMessageBox::warning( this, "Error Saving", "Kinematics solver attempts must be greater than 0." );
    return false;
  }

  adding_new_group_ = false;

  // Save the new group name or create the new group
  if( searched_group == NULL ) // create new
  {
    srdf::Model::Group new_group;
    new_group.name_ = group_name;
    config_data_->srdf_->groups_.push_back( new_group );
    adding_new_group_ = true; // remember this group is new
  }
  else
  {
    // Remember old group name
    const std::string old_group_name = searched_group->name_;

    // Change group name
    searched_group->name_ = group_name;

    // Change all references to this group name in other subgroups
    // Loop through every group
    for( std::vector<srdf::Model::Group>::iterator group_it = config_data_->srdf_->groups_.begin();
         group_it != config_data_->srdf_->groups_.end();  ++group_it )
    {
      // Loop through every subgroup
      for( std::vector<std::string>::iterator subgroup_it = group_it->subgroups_.begin();
           subgroup_it != group_it->subgroups_.end(); ++subgroup_it )
      {
        // Check if that subgroup references old group name. if so, update it
        if( subgroup_it->compare( old_group_name ) == 0 ) // same name
        {
          subgroup_it->assign(group_name); // updated
        }
      }
    }
  }

  // Save the group meta data
  config_data_->group_meta_data_[ group_name ].kinematics_solver_ = kinematics_solver;
  config_data_->group_meta_data_[ group_name ].kinematics_solver_search_resolution_ = kinematics_resolution_double;
  config_data_->group_meta_data_[ group_name ].kinematics_solver_timeout_ = kinematics_timeout_double;
  config_data_->group_meta_data_[ group_name ].kinematics_solver_attempts_ = kinematics_attempts_int;

  // Reload main screen table
  loadGroupsTree();

  // Update the current edit group so that we can proceed to the next screen, if user desires
  current_edit_group_ = group_name;

  return true;
}

// ******************************************************************************************
// Call when a new group is created and ready to progress to next screen
// ******************************************************************************************
void PlanningGroupsWidget::saveGroupScreenEdit()
{
  // Save the group
  if( !saveGroupScreen() )
    return;

  // Switch to main screen
  showMainScreen();
}

// ******************************************************************************************
// Call when a new group is created and ready to progress to next screen
// ******************************************************************************************
void PlanningGroupsWidget::saveGroupScreenJoints()
{
  // Save the group
  if( !saveGroupScreen() )
    return;

  // Find the group we are editing based on the goup name string
  loadJointsScreen( config_data_->findGroupByName( current_edit_group_ ) );

  // Switch to screen
  changeScreen( 1 ); // 1 is index of joints
}

// ******************************************************************************************
// Call when a new group is created and ready to progress to next screen
// ******************************************************************************************
void PlanningGroupsWidget::saveGroupScreenLinks()
{
  // Save the group
  if( !saveGroupScreen() )
    return;

  // Find the group we are editing based on the goup name string
  loadLinksScreen( config_data_->findGroupByName( current_edit_group_ ) );

  // Switch to screen
  changeScreen( 2 ); // 2 is index of links
}

// ******************************************************************************************
// Call when a new group is created and ready to progress to next screen
// ******************************************************************************************
void PlanningGroupsWidget::saveGroupScreenChain()
{
  // Save the group
  if( !saveGroupScreen() )
    return;

  // Find the group we are editing based on the goup name string
  loadChainScreen( config_data_->findGroupByName( current_edit_group_ ) );

  // Switch to screen
  changeScreen( 3 );
}

// ******************************************************************************************
// Call when a new group is created and ready to progress to next screen
// ******************************************************************************************
void PlanningGroupsWidget::saveGroupScreenSubgroups()
{
  // Save the group
  if( !saveGroupScreen() )
    return;

  // Find the group we are editing based on the goup name string
  loadSubgroupsScreen( config_data_->findGroupByName( current_edit_group_ ) );

  // Switch to screen
  changeScreen( 4 );
}

// ******************************************************************************************
// Call when edit screen is canceled
// ******************************************************************************************
void PlanningGroupsWidget::cancelEditing()
{
  if (!current_edit_group_.empty() && adding_new_group_)
  {
    srdf::Model::Group *editing = config_data_->findGroupByName( current_edit_group_ );
    if( editing && editing->joints_.empty() && editing->links_.empty() &&
        editing->chains_.empty() && editing->subgroups_.empty())
    {
      config_data_->group_meta_data_.erase(editing->name_);
      for( std::vector<srdf::Model::Group>::iterator group_it = config_data_->srdf_->groups_.begin();
           group_it != config_data_->srdf_->groups_.end();  ++group_it )
        if (&(*group_it) == editing)
        {
          config_data_->srdf_->groups_.erase(group_it);
          break;
        }
      current_edit_group_.clear();
      // Load the data to the tree
      loadGroupsTree();
    }
  }

  // Switch to main screen
  showMainScreen();
}

// ******************************************************************************************
// Called when setup assistant navigation switches to this screen
// ******************************************************************************************
void PlanningGroupsWidget::focusGiven()
{
  // Show the current groups screen
  showMainScreen();

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

// ******************************************************************************************
// Switch to current groups view
// ******************************************************************************************
void PlanningGroupsWidget::showMainScreen()
{
  stacked_layout_->setCurrentIndex( 0 );

  // Announce that this widget is not in modal mode
  Q_EMIT isModal( false );
}

// ******************************************************************************************
// Switch which screen is being shown
// ******************************************************************************************
void PlanningGroupsWidget::changeScreen( int index )
{
  stacked_layout_->setCurrentIndex( index );

  // Announce this widget's mode
  Q_EMIT isModal( index != 0 );
}

// ******************************************************************************************
// Called from Double List widget to highlight a link
// ******************************************************************************************
void PlanningGroupsWidget::previewSelectedLink( std::vector<std::string> links )
{
  // Unhighlight all links
  Q_EMIT unhighlightAll();

  for(int i = 0; i < links.size(); ++i)
  {
    if( links[i].empty() )
    {
      continue;
    }

    std::cout << "    previewSelectedLink " << links[i] << std::endl;

    // Highlight link
    Q_EMIT highlightLink( links[i] );
  }
}

// ******************************************************************************************
// Called from Double List widget to highlight joints
// ******************************************************************************************
void PlanningGroupsWidget::previewSelectedJoints( std::vector<std::string> joints )
{
  // Unhighlight all links
  Q_EMIT unhighlightAll();

  for(int i = 0; i < joints.size(); ++i)
  {

    const robot_model::JointModel *joint_model = config_data_->getRobotModel()->getJointModel( joints[i] );

    // Check that a joint model was found
    if( !joint_model )
    {
      continue;
    }

    // Get the name of the link
    const std::string link = joint_model->getChildLinkModel()->getName();

    if( link.empty() )
    {
      continue;
    }

    // Highlight link
    Q_EMIT highlightLink( link );
  }
}

// ******************************************************************************************
// Called from Double List widget to highlight a subgroup
// ******************************************************************************************
void PlanningGroupsWidget::previewSelectedSubgroup( std::vector<std::string> groups )
{
  // Unhighlight all links
  Q_EMIT unhighlightAll();

  for(int i = 0; i < groups.size(); ++i)
  {
    // Highlight group
    Q_EMIT highlightGroup( groups[i] );
  }
}



} // namespace


// ******************************************************************************************
// ******************************************************************************************
// CLASS
// ******************************************************************************************
// ******************************************************************************************

PlanGroupType::PlanGroupType( srdf::Model::Group *group, const moveit_setup_assistant::GroupType type )
  : group_( group ), type_( type )
{
}


