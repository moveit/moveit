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

// SA
#include "robot_poses_widget.h"
#include <moveit_msgs/JointLimits.h>
// Qt
#include <QFormLayout>
#include <QMessageBox>
#include <QDoubleValidator>

namespace moveit_setup_assistant
{

// ******************************************************************************************
// Outer User Interface for MoveIt Configuration Assistant
// ******************************************************************************************
RobotPosesWidget::RobotPosesWidget( QWidget *parent, moveit_setup_assistant::MoveItConfigDataPtr config_data )
  : SetupScreenWidget( parent ), config_data_(config_data)
{
  // Set pointer to null so later we can tell if we need to delete it
  joint_list_layout_ = NULL;

  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout( );

  // Top Header Area ------------------------------------------------

  HeaderWidget *header = new HeaderWidget( "Robot Poses",
                                           "Create poses for robot defined as sets of joint values for particular planning groups. This is useful for things like 'folded arms'.",
                                           this);
  layout->addWidget( header );

  // Create contents screens ---------------------------------------
  pose_list_widget_ = createContentsWidget();
  pose_edit_widget_ = createEditWidget();

  // Create stacked layout -----------------------------------------
  stacked_layout_ = new QStackedLayout( this );
  stacked_layout_->addWidget( pose_list_widget_ ); // screen index 0
  stacked_layout_->addWidget( pose_edit_widget_ ); // screen index 1

  // Create Widget wrapper for layout
  QWidget *stacked_layout_widget = new QWidget( this );
  stacked_layout_widget->setLayout( stacked_layout_ );

  layout->addWidget( stacked_layout_widget );


  // Finish Layout --------------------------------------------------
  this->setLayout(layout);


  // Create joint publisher -----------------------------------------
  ros::NodeHandle nh;

  // Create scene publisher for later use
  pub_scene_ = nh.advertise<moveit_msgs::PlanningScene>( MOVEIT_PLANNING_SCENE, 1 );

  // Set the planning scene
  config_data_->getPlanningScene()->setName("MoveIt Planning Scene");  
}

// ******************************************************************************************
// Create the main content widget
// ******************************************************************************************
QWidget* RobotPosesWidget::createContentsWidget()
{
  // Main widget
  QWidget *content_widget = new QWidget( this );

  // Basic widget container
  QVBoxLayout *layout = new QVBoxLayout( this );

  // Table ------------ ------------------------------------------------

  data_table_ = new QTableWidget( this );
  data_table_->setColumnCount(2);
  data_table_->setSortingEnabled(true);
  data_table_->setSelectionBehavior( QAbstractItemView::SelectRows );
  connect( data_table_, SIGNAL( cellDoubleClicked( int, int ) ), this, SLOT( editDoubleClicked( int, int ) ) );
  connect( data_table_, SIGNAL( cellClicked( int, int ) ), this, SLOT( previewClicked( int, int ) ) );
  layout->addWidget( data_table_ );  

  // Set header labels
  QStringList header_list;
  header_list.append("Pose Name");
  header_list.append("Group Name");
  data_table_->setHorizontalHeaderLabels(header_list);
  
  // Bottom Buttons --------------------------------------------------

  QHBoxLayout *controls_layout = new QHBoxLayout();

  // Set Default Button
  QPushButton *btn_default = new QPushButton( "&Show Default Pose", this );
  btn_default->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred );
  btn_default->setMaximumWidth(300);
  connect(btn_default, SIGNAL(clicked()), this, SLOT(showDefaultPose()));
  controls_layout->addWidget(btn_default);
  controls_layout->setAlignment( btn_default, Qt::AlignLeft );

  // Spacer
  QWidget *spacer = new QWidget( this );
  spacer->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
  controls_layout->addWidget( spacer );

  // Edit Selected Button
  btn_edit_ = new QPushButton( "&Edit Selected", this );
  btn_edit_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred );
  btn_edit_->setMaximumWidth(300);
  btn_edit_->hide(); // show once we know if there are existing poses
  connect(btn_edit_, SIGNAL(clicked()), this, SLOT(editSelected()));
  controls_layout->addWidget(btn_edit_);
  controls_layout->setAlignment( btn_edit_, Qt::AlignRight );

  // Add Group Button
  QPushButton *btn_add = new QPushButton( "&Add Pose", this );
  btn_add->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred );
  btn_add->setMaximumWidth(300);
  connect(btn_add, SIGNAL(clicked()), this, SLOT(showNewScreen()));
  controls_layout->addWidget(btn_add);
  controls_layout->setAlignment( btn_add, Qt::AlignRight );
  
  // Add layout
  layout->addLayout( controls_layout );

  
  // Set layout -----------------------------------------------------
  content_widget->setLayout(layout);

  return content_widget;
}

// ******************************************************************************************
// Create the edit widget
// ******************************************************************************************
QWidget* RobotPosesWidget::createEditWidget()
{
  // Main widget
  QWidget *edit_widget = new QWidget( this );
  // Layout
  QVBoxLayout *layout = new QVBoxLayout();

  // 2 columns -------------------------------------------------------
  
  QHBoxLayout *columns_layout = new QHBoxLayout();
  QVBoxLayout *column1 = new QVBoxLayout();
  column2_ = new QVBoxLayout();

  // Column 1 --------------------------------------------------------

  // Simple form -------------------------------------------
  QFormLayout *form_layout = new QFormLayout();
  //form_layout->setContentsMargins( 0, 15, 0, 15 );
  form_layout->setRowWrapPolicy( QFormLayout::WrapAllRows );

  // Name input
  pose_name_field_ = new QLineEdit( this );
  //pose_name_field_->setMaximumWidth( 300 );
  form_layout->addRow( "Pose Name:", pose_name_field_ );

  group_name_field_ = new QComboBox( this );
  group_name_field_->setEditable( false );
  //group_name_field_->setMaximumWidth( 300 );
  connect( group_name_field_, SIGNAL( currentIndexChanged( const QString & ) ), 
           this, SLOT( loadJointSliders( const QString & ) ) );
  form_layout->addRow( "Planning Group:", group_name_field_ );
  
  column1->addLayout( form_layout );
  columns_layout->addLayout( column1 );

  // Column 2 --------------------------------------------------------

  // Box to hold joint sliders
  joint_list_widget_ = new QWidget( this );
  
  // Create scroll area
  scroll_area_ = new QScrollArea( this );
  //scroll_area_->setBackgroundRole(QPalette::Dark);
  scroll_area_->setWidget( joint_list_widget_ );

  column2_->addWidget( scroll_area_ );
  //  column2_->addWidget( joint_list_widget_ );

  columns_layout->addLayout( column2_ );

  // Set columns in main layout
  layout->addLayout( columns_layout );

  // Bottom Buttons --------------------------------------------------

  QHBoxLayout *controls_layout = new QHBoxLayout();
  controls_layout->setContentsMargins( 0, 25, 0, 15 );

  // Delete
  btn_delete_ = new QPushButton( "&Delete Pose", this );
  connect( btn_delete_, SIGNAL(clicked()), this, SLOT( deleteItem() ) );
  controls_layout->addWidget( btn_delete_ );
  controls_layout->setAlignment(btn_delete_, Qt::AlignLeft);

  // Spacer
  QWidget *spacer = new QWidget( this );
  spacer->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
  controls_layout->addWidget( spacer );

  // Save
  QPushButton *btn_save_ = new QPushButton( "&Save", this );
  btn_save_->setMaximumWidth( 200 );
  connect( btn_save_, SIGNAL(clicked()), this, SLOT( doneEditing() ) );
  controls_layout->addWidget( btn_save_ );
  controls_layout->setAlignment(btn_save_, Qt::AlignRight);

  // Cancel
  QPushButton *btn_cancel_ = new QPushButton( "&Cancel", this );
  btn_cancel_->setMaximumWidth( 200 );
  connect( btn_cancel_, SIGNAL(clicked()), this, SLOT( cancelEditing() ) );
  controls_layout->addWidget( btn_cancel_ );
  controls_layout->setAlignment(btn_cancel_, Qt::AlignRight);
  
  // Add layout
  layout->addLayout( controls_layout );

  
  // Set layout -----------------------------------------------------
  edit_widget->setLayout(layout);

  return edit_widget;
}

// ******************************************************************************************
// Show edit screen for creating a new pose
// ******************************************************************************************
void RobotPosesWidget::showNewScreen()
{
  // Remember that this is a new pose
  current_edit_pose_.clear();

  // Hide delete button because this is a new widget
  btn_delete_->hide();

  // Clear previous data
  pose_name_field_->setText("");
  group_name_field_->clearEditText(); // actually this just chooses first option

  // Switch to screen
  stacked_layout_->setCurrentIndex( 1 ); 
}

// ******************************************************************************************
// Edit whatever element is selected
// ******************************************************************************************
void RobotPosesWidget::editDoubleClicked( int row, int column )
{
  // Get the first column of the row that was double clicked
  //edit( data_table_->itemAt( 0, row )->text().toStdString() ); // x,y

  editSelected();
}

// ******************************************************************************************
// Preview whatever element is selected
// ******************************************************************************************
void RobotPosesWidget::previewClicked( int row, int column )
{
  // Get list of all selected items
  QList<QTableWidgetItem*> selected = data_table_->selectedItems();

  // Check that an element was selected
  if( !selected.size() )
    return;

  // Find the selected in datastructure
  srdf::Model::GroupState *pose = findPoseByName( selected[0]->text().toStdString() );

  // Set pose joint values by adding them to the local joint state map
  for( std::map<std::string, std::vector<double> >::const_iterator value_it = pose->joint_values_.begin();
       value_it != pose->joint_values_.end(); ++value_it )
  {
    // Only copy the first joint value // TODO: add capability for multi-DOF joints?
    joint_state_map_[ value_it->first ] = value_it->second[0];

    std::cout << "JOINT " << value_it->first << " -- " << value_it->second[0] << std::endl;
  }

  // Update the joints
  publishJoints();
}

// ******************************************************************************************
// Show the robot in its default joint positions
// ******************************************************************************************
void RobotPosesWidget::showDefaultPose()
{
  namespace pm = planning_models;

  // Get list of all joints for the robot
  std::vector<const pm::KinematicModel::JointModel*> joint_models = config_data_->getKinematicModel()->getJointModels();
  
  // Iterate through the joints
  for( std::vector<const pm::KinematicModel::JointModel*>::const_iterator joint_it = joint_models.begin();
       joint_it < joint_models.end(); ++joint_it )
  {
    
    // Check that this joint only represents 1 variable.
    if( (*joint_it)->getVariableCount() == 1 )
    {
      double init_value;

      // get the first joint value in its vector
      std::vector<double> default_values;
      (*joint_it)->getDefaultValues( default_values );
      init_value = default_values[0];

      // Change joint's value in joint_state_map to the default
      joint_state_map_[ (*joint_it)->getName() ] = init_value;
    }
  }

  // Update the joints
  publishJoints();

}

// ******************************************************************************************
// Edit whatever element is selected
// ******************************************************************************************
void RobotPosesWidget::editSelected()
{
  // Get list of all selected items
  QList<QTableWidgetItem*> selected = data_table_->selectedItems();

  // Check that an element was selected
  if( !selected.size() )
    return;

  // Get selected name and edit it
  edit( selected[0]->text().toStdString() );
}

// ******************************************************************************************
// Edit pose
// ******************************************************************************************
void RobotPosesWidget::edit( const std::string &name )
{
  // Remember what we are editing
  current_edit_pose_ = name;

  // Find the selected in datastruture
  srdf::Model::GroupState *pose = findPoseByName( name );

  // Set pose name
  pose_name_field_->setText( pose->name_.c_str() );

  // Set pose joint values by adding them to the local joint state map
  for( std::map<std::string, std::vector<double> >::const_iterator value_it = pose->joint_values_.begin();
       value_it != pose->joint_values_.end(); ++value_it )
  {
    // Only copy the first joint value // TODO: add capability for multi-DOF joints?
    joint_state_map_[ value_it->first ] = value_it->second[0];
  }
  
  // Update robot model in rviz
  publishJoints();
  
  // Set group:
  int index = group_name_field_->findText( pose->group_.c_str() );
  if( index == -1 )
  {
    QMessageBox::critical( this, "Error Loading", "Unable to find group name in drop down box" );
    return;
  }

  group_name_field_->setCurrentIndex( index );

  // Show delete button because its an existing item
  btn_delete_->show();

  // Switch to screen
  stacked_layout_->setCurrentIndex( 1 ); 
}

// ******************************************************************************************
// Populate the combo dropdown box with avail group names
// ******************************************************************************************
void RobotPosesWidget::loadGroupsComboBox()
{
  // Remove all old groups
  group_name_field_->clear();
  
  // Add all group names to combo box
  for( std::vector<srdf::Model::Group>::iterator group_it = config_data_->srdf_->groups_.begin();
       group_it != config_data_->srdf_->groups_.end(); ++group_it )
  {
    group_name_field_->addItem( group_it->name_.c_str() );
  }  

}

// ******************************************************************************************
// Load the joint sliders based on group selected from combo box
// ******************************************************************************************
void RobotPosesWidget::loadJointSliders( const QString &selected )
{
  // Ignore this event if the combo box is empty. This occurs when clearing the combo box and reloading with the
  // newest groups
  if( !group_name_field_->count() || selected.isEmpty() )
    return;

  // Get group name from input
  const std::string group_name = selected.toStdString();

  // Check that joint model exist
  if( !config_data_->getKinematicModel()->hasJointModelGroup( group_name ) )
  {
    QMessageBox::critical( this, "Error Loading", 
                           QString("Unable to find joint model group for group: ").append( group_name.c_str()).append(" Are you sure this group has associated joints/links?" ));
    return;
  }

  // Delete old sliders from joint_list_layout_ if this has been loaded before
  if( joint_list_layout_ )
  {
    delete joint_list_layout_;
    qDeleteAll( joint_list_widget_->children() );
  }

  // Create layout again
  joint_list_layout_ = new QVBoxLayout();
  joint_list_widget_->setLayout( joint_list_layout_ );
  //  joint_list_widget_->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );
  joint_list_widget_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  joint_list_widget_->setMinimumSize( 50, 50 ); // w, h

  // Load joints ---------------------------------------------------------
  namespace pm = planning_models;

  // Get list of associated joints  
  const pm::KinematicModel::JointModelGroup *joint_model_group = 
    config_data_->getKinematicModel()->getJointModelGroup( group_name );
  joint_models_ = joint_model_group->getJointModels();
  
  // Iterate through the joints
  int num_joints = 0;
  for( std::vector<const pm::KinematicModel::JointModel*>::const_iterator joint_it = joint_models_.begin();
       joint_it < joint_models_.end(); ++joint_it )
  {
    
    // Check that this joint only represents 1 variable.
    if( (*joint_it)->getVariableCount() == 1 )
    {
      double init_value;

      // Decide what this joint's initial value is
      if( joint_state_map_.find( (*joint_it)->getName() ) == joint_state_map_.end() )
      {
        // the joint state map does not yet have an entry for this joint
        
        // get the first joint value in its vector
        std::vector<double> default_values;
        (*joint_it)->getDefaultValues( default_values );
        init_value = default_values[0];

      }
      else // there is already a value in the map
      {
        init_value = joint_state_map_[ (*joint_it)->getName() ];
      }

      // For each joint in group add slider
      SliderWidget *sw = new SliderWidget( this, *joint_it, init_value );
      joint_list_layout_->addWidget( sw );
      
      // Connect value change event
      connect( sw, SIGNAL( jointValueChanged( const std::string &, double ) ), 
               this, SLOT( updateKinematicModel( const std::string &, double ) ) );

      ++num_joints;
    }
  }

  // Copy the width of column 2 and manually calculate height from number of joints
  joint_list_widget_->resize( 280, num_joints * 70 ); //w, h

  // Update the robot model in Rviz with newly selected joint values
  publishJoints();
}

// ******************************************************************************************
// Find a group by pointer using its string name
// ******************************************************************************************
srdf::Model::Group *RobotPosesWidget::findGroupByName( const std::string &name )
{
  // Find the group we are editing based on the goup name string
  srdf::Model::Group *searched_group = NULL; // used for holding our search results

  for( std::vector<srdf::Model::Group>::iterator group_it = config_data_->srdf_->groups_.begin();
       group_it != config_data_->srdf_->groups_.end(); ++group_it )
  {
    if( group_it->name_ == name ) // string match
    {
      searched_group = &(*group_it);  // convert to pointer from iterator
      break; // we are done searching
    }
  }  

  // Check if subgroup was found
  if( searched_group == NULL ) // not found
  {
    QMessageBox::critical( this, "Error Loading", "An internal error has occured while searching for groups");
    exit(0); // TODO: is this the ROS way?
  }
  
  return searched_group;
}

// ******************************************************************************************
// Find the associated data by name
// ******************************************************************************************
srdf::Model::GroupState *RobotPosesWidget::findPoseByName( const std::string &name )
{
  // Find the group state we are editing based on the pose name
  srdf::Model::GroupState *searched_group = NULL; // used for holding our search results

  for( std::vector<srdf::Model::GroupState>::iterator pose_it = config_data_->srdf_->group_states_.begin();
       pose_it != config_data_->srdf_->group_states_.end(); ++pose_it )
  {
    if( pose_it->name_ == name ) // string match
    {
      searched_group = &(*pose_it);  // convert to pointer from iterator
      break; // we are done searching
    }
  }  

  // Check if pose was found
  if( searched_group == NULL ) // not found
  {
    QMessageBox::critical( this, "Error Saving", "An internal error has occured while saving. Quitting.");
    exit(0); // TODO: is this the ROS way?
  }
  
  return searched_group;
}

// ******************************************************************************************
// Delete currently editing item
// ******************************************************************************************
void RobotPosesWidget::deleteItem()
{
  // Confirm user wants to delete group
  if( QMessageBox::question( this, "Confirm Pose Deletion", 
                             QString("Are you sure you want to delete the pose '")
                             .append( current_edit_pose_.c_str() )
                             .append( "'?" ), 
                             QMessageBox::Ok | QMessageBox::Cancel) 
      == QMessageBox::Cancel )
  {
    return;
  }

  // Delete pose from vector
  for( std::vector<srdf::Model::GroupState>::iterator pose_it = config_data_->srdf_->group_states_.begin();
       pose_it != config_data_->srdf_->group_states_.end(); ++pose_it )
  {
    // check if this is the group we want to delete
    if( pose_it->name_ == current_edit_pose_ ) // string match
    {
      config_data_->srdf_->group_states_.erase( pose_it );
      break;
    }
  }

  // Reload main screen table
  loadDataTable();

  // Switch to screen  
  stacked_layout_->setCurrentIndex( 0 ); 
}

// ******************************************************************************************
// Save editing changes
// ******************************************************************************************
void RobotPosesWidget::doneEditing()
{
  // Get a reference to the supplied strings
  const std::string pose_name = pose_name_field_->text().toStdString();

  // Used for editing existing groups
  srdf::Model::GroupState *searched_data = NULL;

  // Check that name field is not empty
  if( pose_name.empty() )
  {
    QMessageBox::warning( this, "Error Saving", "A name must be given for the pose!" );
    return;    
  }

  // Check if this is an existing group
  if( !current_edit_pose_.empty() )
  {
    // Find the group we are editing based on the goup name string
    searched_data = findPoseByName( current_edit_pose_ );
  }
  
  // Check that the pose name is unique
  for( std::vector<srdf::Model::GroupState>::const_iterator data_it = config_data_->srdf_->group_states_.begin(); 
       data_it != config_data_->srdf_->group_states_.end();  ++data_it )
  {
    if( data_it->name_.compare( pose_name ) == 0 ) // the names are the same
    {
      // is this our existing pose? check if pose pointers are same
      if( &(*data_it) != searched_data )
      {
        QMessageBox::warning( this, "Error Saving", "A pose already exists with that name!" );
        return;
      }
    }
  }

  // Check that a group was selected
  if( group_name_field_->currentText().isEmpty() )
  {
    QMessageBox::warning( this, "Error Saving", "A planning group must be chosen!" );
    return;    
  }

  // Save the new pose name or create the new pose ----------------------------
  bool isNew = false;

  if( searched_data == NULL ) // create new
  {
    isNew = true;

    searched_data = new srdf::Model::GroupState();
  }

  // Copy name data ----------------------------------------------------
  searched_data->name_ = pose_name;
  searched_data->group_ = group_name_field_->currentText().toStdString();

  // Copy joint positions ----------------------------------------
    
  // Clear the old values
  searched_data->joint_values_.clear();

  // Iterate through the current group's joints and add to SRDF
  for( std::vector<const planning_models::KinematicModel::JointModel*>::const_iterator joint_it = joint_models_.begin();
       joint_it < joint_models_.end(); ++joint_it )
  {
    // Check that this joint only represents 1 variable.
    if( (*joint_it)->getVariableCount() == 1 )
    {
      // Create vector for new joint values
      std::vector<double> joint_value;
      joint_value.push_back( joint_state_map_[ (*joint_it)->getName() ] );

      // Add joint vector to SRDF
      searched_data->joint_values_[ (*joint_it)->getName() ] = joint_value;
    }
  }

  // Insert new poses into group state vector --------------------------
  if( isNew )
  {
    config_data_->srdf_->group_states_.push_back( *searched_data );
  }

  // Finish up ------------------------------------------------------

  // Reload main screen table
  loadDataTable();

  // Switch to screen
  stacked_layout_->setCurrentIndex( 0 ); 
}

// ******************************************************************************************
// Cancel changes
// ******************************************************************************************
void RobotPosesWidget::cancelEditing()
{
  // Switch to screen
  stacked_layout_->setCurrentIndex( 0 ); 
}

// ******************************************************************************************
// Load the robot poses into the table
// ******************************************************************************************
void RobotPosesWidget::loadDataTable()
{
  // Disable Table
  data_table_->setUpdatesEnabled(false); // prevent table from updating until we are completely done
  data_table_->setDisabled(true); // make sure we disable it so that the cellChanged event is not called
  data_table_->clearContents();

  // Set size of datatable
  data_table_->setRowCount( config_data_->srdf_->group_states_.size() ); 

  // Loop through every pose
  int row = 0;
  for( std::vector<srdf::Model::GroupState>::const_iterator data_it = config_data_->srdf_->group_states_.begin(); 
       data_it != config_data_->srdf_->group_states_.end(); ++data_it )
  {
    // Create row elements
    QTableWidgetItem* data_name = new QTableWidgetItem( data_it->name_.c_str() );
    data_name->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    QTableWidgetItem* group_name = new QTableWidgetItem( data_it->group_.c_str() );
    group_name->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    // Add to table
    data_table_->setItem( row, 0, data_name );
    data_table_->setItem( row, 1, group_name );
    
    // Increment counter
    ++row;
  }

  // Reenable
  data_table_->setUpdatesEnabled(true); // prevent table from updating until we are completely done
  data_table_->setDisabled(false); // make sure we disable it so that the cellChanged event is not called

  // Resize header
  data_table_->resizeColumnToContents(0);
  data_table_->resizeColumnToContents(1);

  // Show edit button if applicable
  if( config_data_->srdf_->group_states_.size() )
    btn_edit_->show();
}

// ******************************************************************************************
// Called when setup assistant navigation switches to this screen
// ******************************************************************************************
void RobotPosesWidget::focusGiven()
{
  // Show the current poses screen
  stacked_layout_->setCurrentIndex( 0 );

  // Load the data to the tree
  loadDataTable();

  // Load the avail groups to the combo box
  loadGroupsComboBox();

  // TODO: remove demo
  //edit( "tuck_right_arm" );
}

// ******************************************************************************************
// Call when one of the sliders has its value changed
// ******************************************************************************************
void RobotPosesWidget::updateKinematicModel( const std::string &name, double value )
{
  // Save the new value
  joint_state_map_[ name ] = value;

  // Update the robot model/rviz
  publishJoints();
}

// ******************************************************************************************
// Publish the joint values in the joint_state_map_ to Rviz
// ******************************************************************************************
void RobotPosesWidget::publishJoints()
{
  // Change the scene
  //scene.getCurrentState().setToDefaultValues();//set to default values of 0 OR half between low and high joint values  
  //config_data_->getPlanningScene()->getCurrentState().setToRandomValues(); 

  // Set the joints based on the map
  config_data_->getPlanningScene()->getCurrentState().setStateValues( joint_state_map_ );

  // Create a planning scene message
  moveit_msgs::PlanningScene psmsg;
  config_data_->getPlanningScene()->getPlanningSceneMsg( psmsg );

  // Publish!
  pub_scene_.publish( psmsg );
}


// ******************************************************************************************
// ******************************************************************************************
// Slider Widget
// ******************************************************************************************
// ******************************************************************************************

// ******************************************************************************************
// Simple widget for adjusting joints of a robot
// ******************************************************************************************
SliderWidget::SliderWidget( QWidget *parent, const planning_models::KinematicModel::JointModel *joint_model, 
                            double init_value )
  : QWidget( parent ), joint_model_( joint_model )
{
  // Create layouts
  QVBoxLayout *layout = new QVBoxLayout();
  QHBoxLayout *row2 = new QHBoxLayout();
  
  // Row 1
  joint_label_ = new QLabel( joint_model_->getName().c_str() , this );
  joint_label_->setContentsMargins( 0, 0, 0, 0 );
  layout->addWidget( joint_label_ );

  // Row 2 -------------------------------------------------------------
  joint_slider_ = new QSlider( Qt::Horizontal, this );
  joint_slider_->setTickPosition(QSlider::TicksBelow);
  joint_slider_->setSingleStep( 100 );
  joint_slider_->setPageStep( 5000 );
  joint_slider_->setTickInterval( 1000 );
  joint_slider_->setContentsMargins( 0, 0, 0, 0 );
  row2->addWidget( joint_slider_ );

  // Joint Value Box ------------------------------------------------
  joint_value_ = new QLineEdit( this );
  joint_value_->setMaximumWidth( 50 );
  joint_value_->setContentsMargins( 0, 0, 0, 0 );
  connect( joint_value_, SIGNAL( editingFinished() ), this, SLOT( changeJointSlider() ) );
  row2->addWidget( joint_value_ );

  // Joint Limits ----------------------------------------------------
  std::vector<moveit_msgs::JointLimits> limits = joint_model_->getLimits();
  if( limits.empty() )
  {
    QMessageBox::critical( this, "Error Loading", "An internal error has occured while loading the joints" );
    return;
  }

  // Only use the first limit, because there is only 1 variable (as checked earlier)
  moveit_msgs::JointLimits joint_limit = limits[0];
  max_position_ = joint_limit.max_position;
  min_position_ = joint_limit.min_position;

  // Set the slider limits
  joint_slider_->setMaximum( max_position_*10000 );
  joint_slider_->setMinimum( min_position_*10000 ); 
    
  // Set the text box limits    
  QDoubleValidator *validator = new QDoubleValidator( this );
  validator->setRange( min_position_, max_position_, 2 );
  joint_value_->setValidator( validator );

  // Connect slider to joint value box
  connect( joint_slider_, SIGNAL(valueChanged(int)), this, SLOT(changeJointValue(int)) );

  // Initial joint values -------------------------------------------
  int value = init_value * 10000; // scale double to integer for slider use
  joint_slider_->setSliderPosition( value ); // set slider value
  changeJointValue( value ); // show in textbox

  // Finish GUI ----------------------------------------
  layout->addLayout( row2 );

  this->setContentsMargins( 0, 0, 0, 0 );
  //this->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
  this->setGeometry(QRect(110, 80, 120, 80));
  this->setLayout( layout );

  // Declare std::string as metatype so we can use it in a signal
  qRegisterMetaType<std::string>("std::string");
}

// ******************************************************************************************
// Deconstructor
// ******************************************************************************************
SliderWidget::~SliderWidget()
{
}

// ******************************************************************************************
// Called when the joint value slider is changed
// ******************************************************************************************
void SliderWidget::changeJointValue( int value )
{
  // Get joint value
  const double double_value = double( value ) / 10000;

  // Set textbox
  joint_value_->setText( QString( "%1" ).arg( double_value, 0, 'f', 2 ) );
  
  // Send event to parent widget
  Q_EMIT jointValueChanged( joint_model_->getName(), double_value );
}

// ******************************************************************************************
// Called when the joint value box is changed
// ******************************************************************************************
void SliderWidget::changeJointSlider()
{
  // Get joint value
  const double value = joint_value_->text().toDouble();

  // We assume it converts to double because of the validator
  joint_slider_->setSliderPosition( value * 10000 );

  // Send event to parent widget
  Q_EMIT jointValueChanged( joint_model_->getName(), value );
}






} // namespace
