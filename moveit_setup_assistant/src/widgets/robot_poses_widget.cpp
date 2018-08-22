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
 *   * Neither the name of Willow Garage nor the names of its
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
#include <QApplication>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Outer User Interface for MoveIt! Configuration Assistant
// ******************************************************************************************
RobotPosesWidget::RobotPosesWidget(QWidget* parent, moveit_setup_assistant::MoveItConfigDataPtr config_data)
  : SetupScreenWidget(parent), config_data_(config_data)
{
  // Set pointer to null so later we can tell if we need to delete it
  joint_list_layout_ = NULL;

  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();

  // Top Header Area ------------------------------------------------

  HeaderWidget* header =
      new HeaderWidget("Robot Poses", "Create poses for the robot. Poses are defined as sets of joint values for "
                                      "particular planning groups. This is useful for things like <i>folded arms</i>.",
                       this);
  layout->addWidget(header);

  // Create contents screens ---------------------------------------
  pose_list_widget_ = createContentsWidget();
  pose_edit_widget_ = createEditWidget();

  // Create stacked layout -----------------------------------------
  stacked_layout_ = new QStackedLayout(this);
  stacked_layout_->addWidget(pose_list_widget_);  // screen index 0
  stacked_layout_->addWidget(pose_edit_widget_);  // screen index 1

  // Create Widget wrapper for layout
  QWidget* stacked_layout_widget = new QWidget(this);
  stacked_layout_widget->setLayout(stacked_layout_);

  layout->addWidget(stacked_layout_widget);

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);

  // Create joint publisher -----------------------------------------
  ros::NodeHandle nh;

  // Create scene publisher for later use
  pub_robot_state_ = nh.advertise<moveit_msgs::DisplayRobotState>(MOVEIT_ROBOT_STATE, 1);

  // Set the planning scene
  config_data_->getPlanningScene()->setName("MoveIt! Planning Scene");

  // Collision Detection initializtion -------------------------------

  // Setup the request
  request.contacts = true;
  request.max_contacts = 1;
  request.max_contacts_per_pair = 1;
  request.verbose = false;
}

// ******************************************************************************************
// Create the main content widget
// ******************************************************************************************
QWidget* RobotPosesWidget::createContentsWidget()
{
  // Main widget
  QWidget* content_widget = new QWidget(this);

  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Table ------------ ------------------------------------------------

  data_table_ = new QTableWidget(this);
  data_table_->setColumnCount(2);
  data_table_->setSortingEnabled(true);
  data_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  connect(data_table_, SIGNAL(cellDoubleClicked(int, int)), this, SLOT(editDoubleClicked(int, int)));
  connect(data_table_, SIGNAL(cellClicked(int, int)), this, SLOT(previewClicked(int, int)));
  layout->addWidget(data_table_);

  // Set header labels
  QStringList header_list;
  header_list.append("Pose Name");
  header_list.append("Group Name");
  data_table_->setHorizontalHeaderLabels(header_list);

  // Bottom Buttons --------------------------------------------------

  QHBoxLayout* controls_layout = new QHBoxLayout();

  // Set Default Button
  QPushButton* btn_default = new QPushButton("&Show Default Pose", this);
  btn_default->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_default->setMaximumWidth(300);
  connect(btn_default, SIGNAL(clicked()), this, SLOT(showDefaultPose()));
  controls_layout->addWidget(btn_default);
  controls_layout->setAlignment(btn_default, Qt::AlignLeft);

  // Set play button
  QPushButton* btn_play = new QPushButton("&MoveIt!", this);
  btn_play->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_play->setMaximumWidth(300);
  connect(btn_play, SIGNAL(clicked()), this, SLOT(playPoses()));
  controls_layout->addWidget(btn_play);
  controls_layout->setAlignment(btn_play, Qt::AlignLeft);

  // Spacer
  QWidget* spacer = new QWidget(this);
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  controls_layout->addWidget(spacer);

  // Edit Selected Button
  btn_edit_ = new QPushButton("&Edit Selected", this);
  btn_edit_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_edit_->setMaximumWidth(300);
  btn_edit_->hide();  // show once we know if there are existing poses
  connect(btn_edit_, SIGNAL(clicked()), this, SLOT(editSelected()));
  controls_layout->addWidget(btn_edit_);
  controls_layout->setAlignment(btn_edit_, Qt::AlignRight);

  // Delete
  btn_delete_ = new QPushButton("&Delete Selected", this);
  connect(btn_delete_, SIGNAL(clicked()), this, SLOT(deleteSelected()));
  controls_layout->addWidget(btn_delete_);
  controls_layout->setAlignment(btn_delete_, Qt::AlignRight);

  // Add Group Button
  QPushButton* btn_add = new QPushButton("&Add Pose", this);
  btn_add->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_add->setMaximumWidth(300);
  connect(btn_add, SIGNAL(clicked()), this, SLOT(showNewScreen()));
  controls_layout->addWidget(btn_add);
  controls_layout->setAlignment(btn_add, Qt::AlignRight);

  // Add layout
  layout->addLayout(controls_layout);

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
  QWidget* edit_widget = new QWidget(this);
  // Layout
  QVBoxLayout* layout = new QVBoxLayout();

  // 2 columns -------------------------------------------------------

  QHBoxLayout* columns_layout = new QHBoxLayout();
  QVBoxLayout* column1 = new QVBoxLayout();
  column2_ = new QVBoxLayout();

  // Column 1 --------------------------------------------------------

  // Simple form -------------------------------------------
  QFormLayout* form_layout = new QFormLayout();
  // form_layout->setContentsMargins( 0, 15, 0, 15 );
  form_layout->setRowWrapPolicy(QFormLayout::WrapAllRows);

  // Name input
  pose_name_field_ = new QLineEdit(this);
  // pose_name_field_->setMaximumWidth( 300 );
  form_layout->addRow("Pose Name:", pose_name_field_);

  // Group name input
  group_name_field_ = new QComboBox(this);
  group_name_field_->setEditable(false);
  // Connect the signal for changes to the drop down box
  connect(group_name_field_, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(loadJointSliders(const QString&)));
  // group_name_field_->setMaximumWidth( 300 );
  form_layout->addRow("Planning Group:", group_name_field_);

  // Indicator that robot is in collision or not
  collision_warning_ = new QLabel("<font color='red'><b>Robot in Collision State</b></font>", this);
  collision_warning_->setTextFormat(Qt::RichText);
  collision_warning_->hide();  // show later
  form_layout->addRow(" ", collision_warning_);

  column1->addLayout(form_layout);
  columns_layout->addLayout(column1);

  // Column 2 --------------------------------------------------------

  // Box to hold joint sliders
  joint_list_widget_ = new QWidget(this);

  // Create scroll area
  scroll_area_ = new QScrollArea(this);
  // scroll_area_->setBackgroundRole(QPalette::Dark);
  scroll_area_->setWidget(joint_list_widget_);

  column2_->addWidget(scroll_area_);

  columns_layout->addLayout(column2_);

  // Set columns in main layout
  layout->addLayout(columns_layout);

  // Bottom Buttons --------------------------------------------------

  QHBoxLayout* controls_layout = new QHBoxLayout();
  controls_layout->setContentsMargins(0, 25, 0, 15);

  // Spacer
  QWidget* spacer = new QWidget(this);
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  controls_layout->addWidget(spacer);

  // Save
  QPushButton* btn_save_ = new QPushButton("&Save", this);
  btn_save_->setMaximumWidth(200);
  connect(btn_save_, SIGNAL(clicked()), this, SLOT(doneEditing()));
  controls_layout->addWidget(btn_save_);
  controls_layout->setAlignment(btn_save_, Qt::AlignRight);

  // Cancel
  QPushButton* btn_cancel_ = new QPushButton("&Cancel", this);
  btn_cancel_->setMaximumWidth(200);
  connect(btn_cancel_, SIGNAL(clicked()), this, SLOT(cancelEditing()));
  controls_layout->addWidget(btn_cancel_);
  controls_layout->setAlignment(btn_cancel_, Qt::AlignRight);

  // Add layout
  layout->addLayout(controls_layout);

  // Set layout -----------------------------------------------------
  edit_widget->setLayout(layout);

  return edit_widget;
}

// ******************************************************************************************
// Show edit screen for creating a new pose
// ******************************************************************************************
void RobotPosesWidget::showNewScreen()
{
  // Switch to screen - do this before clearEditText()
  stacked_layout_->setCurrentIndex(1);

  // Remember that this is a new pose
  current_edit_pose_ = nullptr;

  // Manually send the load joint sliders signal
  if (!group_name_field_->currentText().isEmpty())
    loadJointSliders(group_name_field_->currentText());

  // Clear previous data
  pose_name_field_->setText("");

  // Announce that this widget is in modal mode
  Q_EMIT isModal(true);
}

// ******************************************************************************************
// Edit whatever element is selected
// ******************************************************************************************
void RobotPosesWidget::editDoubleClicked(int row, int column)
{
  // We'll just base the edit on the selection highlight
  editSelected();
}

// ******************************************************************************************
// Preview whatever element is selected
// ******************************************************************************************
void RobotPosesWidget::previewClicked(int row, int column)
{
  const std::string& name = data_table_->item(row, 0)->text().toStdString();
  const std::string& group = data_table_->item(row, 1)->text().toStdString();

  // Find the selected in datastructure
  srdf::Model::GroupState* pose = findPoseByName(name, group);

  showPose(pose);
}

// ******************************************************************************************
// Show the robot in the current pose
// ******************************************************************************************
void RobotPosesWidget::showPose(srdf::Model::GroupState* pose)
{
  // Set pose joint values by adding them to the local joint state map
  for (std::map<std::string, std::vector<double> >::const_iterator value_it = pose->joint_values_.begin();
       value_it != pose->joint_values_.end(); ++value_it)
  {
    // Only copy the first joint value // TODO: add capability for multi-DOF joints?
    joint_state_map_[value_it->first] = value_it->second[0];
  }

  // Update the joints
  publishJoints();

  // Unhighlight all links
  Q_EMIT unhighlightAll();

  // Highlight group
  Q_EMIT highlightGroup(pose->group_);
}

// ******************************************************************************************
// Show the robot in its default joint positions
// ******************************************************************************************
void RobotPosesWidget::showDefaultPose()
{
  // Get list of all joints for the robot
  std::vector<const robot_model::JointModel*> joint_models = config_data_->getRobotModel()->getJointModels();

  // Iterate through the joints
  for (std::vector<const robot_model::JointModel*>::const_iterator joint_it = joint_models.begin();
       joint_it < joint_models.end(); ++joint_it)
  {
    // Check that this joint only represents 1 variable.
    if ((*joint_it)->getVariableCount() == 1)
    {
      double init_value;

      // get the first joint value in its vector
      (*joint_it)->getVariableDefaultPositions(&init_value);

      // Change joint's value in joint_state_map to the default
      joint_state_map_[(*joint_it)->getName()] = init_value;
    }
  }

  // Update the joints
  publishJoints();

  // Unhighlight all links
  Q_EMIT unhighlightAll();
}

// ******************************************************************************************
// Play through the poses
// ******************************************************************************************
void RobotPosesWidget::playPoses()
{
  // Loop through each pose and play them
  for (std::vector<srdf::Model::GroupState>::iterator pose_it = config_data_->srdf_->group_states_.begin();
       pose_it != config_data_->srdf_->group_states_.end(); ++pose_it)
  {
    ROS_INFO_STREAM("Showing pose " << pose_it->name_);
    showPose(&(*pose_it));
    ros::Duration(0.05).sleep();
    QApplication::processEvents();
    ros::Duration(0.45).sleep();
  }
}

// ******************************************************************************************
// Edit whatever element is selected
// ******************************************************************************************
void RobotPosesWidget::editSelected()
{
  const auto& ranges = data_table_->selectedRanges();
  if (!ranges.size())
    return;
  edit(ranges[0].bottomRow());
}

// ******************************************************************************************
// Edit pose
// ******************************************************************************************
void RobotPosesWidget::edit(int row)
{
  const std::string& name = data_table_->item(row, 0)->text().toStdString();
  const std::string& group = data_table_->item(row, 1)->text().toStdString();

  // Find the selected in datastruture
  srdf::Model::GroupState* pose = findPoseByName(name, group);
  current_edit_pose_ = pose;

  // Set pose name
  pose_name_field_->setText(pose->name_.c_str());

  // Set group:
  int index = group_name_field_->findText(pose->group_.c_str());
  if (index == -1)
  {
    QMessageBox::critical(this, "Error Loading", "Unable to find group name in drop down box");
    return;
  }
  group_name_field_->setCurrentIndex(index);

  // Set pose joint values by adding them to the local joint state map
  for (std::map<std::string, std::vector<double> >::const_iterator value_it = pose->joint_values_.begin();
       value_it != pose->joint_values_.end(); ++value_it)
  {
    // Only copy the first joint value // TODO: add capability for multi-DOF joints?
    joint_state_map_[value_it->first] = value_it->second[0];
  }

  // Update robot model in rviz
  publishJoints();

  // Switch to screen - do this before setCurrentIndex
  stacked_layout_->setCurrentIndex(1);

  // Announce that this widget is in modal mode
  Q_EMIT isModal(true);

  // Manually send the load joint sliders signal
  loadJointSliders(QString(pose->group_.c_str()));
}

// ******************************************************************************************
// Populate the combo dropdown box with avail group names
// ******************************************************************************************
void RobotPosesWidget::loadGroupsComboBox()
{
  // Remove all old groups
  group_name_field_->clear();

  // Add all group names to combo box
  for (std::vector<srdf::Model::Group>::iterator group_it = config_data_->srdf_->groups_.begin();
       group_it != config_data_->srdf_->groups_.end(); ++group_it)
  {
    group_name_field_->addItem(group_it->name_.c_str());
  }
}

// ******************************************************************************************
// Load the joint sliders based on group selected from combo box
// ******************************************************************************************
void RobotPosesWidget::loadJointSliders(const QString& selected)
{
  // Ignore this event if the combo box is empty. This occurs when clearing the combo box and reloading with the
  // newest groups. Also ignore if we are not on the edit screen
  if (!group_name_field_->count() || selected.isEmpty() || stacked_layout_->currentIndex() == 0)
    return;

  // Get group name from input
  const std::string group_name = selected.toStdString();

  // Check that joint model exist
  if (!config_data_->getRobotModel()->hasJointModelGroup(group_name))
  {
    QMessageBox::critical(this, "Error Loading", QString("Unable to find joint model group for group: ")
                                                     .append(group_name.c_str())
                                                     .append(" Are you sure this group has associated joints/links?"));
    return;
  }

  // Delete old sliders from joint_list_layout_ if this has been loaded before
  if (joint_list_layout_)
  {
    delete joint_list_layout_;
    qDeleteAll(joint_list_widget_->children());
  }

  // Create layout again
  joint_list_layout_ = new QVBoxLayout();
  joint_list_widget_->setLayout(joint_list_layout_);
  //  joint_list_widget_->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );
  joint_list_widget_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  joint_list_widget_->setMinimumSize(50, 50);  // w, h

  // Get list of associated joints
  const robot_model::JointModelGroup* joint_model_group = config_data_->getRobotModel()->getJointModelGroup(group_name);
  joint_models_ = joint_model_group->getJointModels();

  // Iterate through the joints
  int num_joints = 0;
  for (const robot_model::JointModel* joint_model : joint_models_)
  {
    if (joint_model->getVariableCount() != 1 ||  // only consider 1-variable joints
        joint_model->isPassive() ||              // ignore passive
        joint_model->getMimic())                 // and mimic joints
      continue;

    double init_value;

    // Decide what this joint's initial value is
    if (joint_state_map_.find(joint_model->getName()) == joint_state_map_.end())
    {
      // The joint state map does not yet have an entry for this joint
      // Get the first joint value in its vector
      joint_model->getVariableDefaultPositions(&init_value);
    }
    else  // There is already a value in the map
    {
      init_value = joint_state_map_[joint_model->getName()];
    }

    // For each joint in group add slider
    SliderWidget* sw = new SliderWidget(this, joint_model, init_value);
    joint_list_layout_->addWidget(sw);

    // Connect value change event
    connect(sw, SIGNAL(jointValueChanged(const std::string&, double)), this,
            SLOT(updateRobotModel(const std::string&, double)));

    ++num_joints;
  }

  // Copy the width of column 2 and manually calculate height from number of joints
  joint_list_widget_->resize(300, num_joints * 70);  // w, h

  // Update the robot model in Rviz with newly selected joint values
  publishJoints();

  // Highlight the planning group
  Q_EMIT unhighlightAll();
  Q_EMIT highlightGroup(group_name);
}

// ******************************************************************************************
// Find the associated data by name
// ******************************************************************************************
srdf::Model::GroupState* RobotPosesWidget::findPoseByName(const std::string& name, const std::string& group)
{
  // Find the group state we are editing based on the pose name
  srdf::Model::GroupState* searched_state = NULL;  // used for holding our search results

  for (srdf::Model::GroupState& state : config_data_->srdf_->group_states_)
  {
    if (state.name_ == name && state.group_ == group)  // match
    {
      searched_state = &state;
      break;
    }
  }

  return searched_state;
}

// ******************************************************************************************
// Delete currently editing item
// ******************************************************************************************
void RobotPosesWidget::deleteSelected()
{
  const auto& ranges = data_table_->selectedRanges();
  if (!ranges.size())
    return;
  int row = ranges[0].bottomRow();

  const std::string& name = data_table_->item(row, 0)->text().toStdString();
  const std::string& group = data_table_->item(row, 1)->text().toStdString();

  // Confirm user wants to delete group
  if (QMessageBox::question(this, "Confirm Pose Deletion",
                            QString("Are you sure you want to delete the pose '").append(name.c_str()).append("'?"),
                            QMessageBox::Ok | QMessageBox::Cancel) == QMessageBox::Cancel)
  {
    return;
  }

  // Delete pose from vector
  for (std::vector<srdf::Model::GroupState>::iterator pose_it = config_data_->srdf_->group_states_.begin();
       pose_it != config_data_->srdf_->group_states_.end(); ++pose_it)
  {
    // check if this is the group we want to delete
    if (pose_it->name_ == name && pose_it->group_ == group)  // match
    {
      config_data_->srdf_->group_states_.erase(pose_it);
      break;
    }
  }

  // Reload main screen table
  loadDataTable();
  config_data_->changes |= MoveItConfigData::POSES;
}

// ******************************************************************************************
// Save editing changes
// ******************************************************************************************
void RobotPosesWidget::doneEditing()
{
  // Get a reference to the supplied strings
  const std::string& name = pose_name_field_->text().toStdString();
  const std::string& group = group_name_field_->currentText().toStdString();

  // Used for editing existing groups
  srdf::Model::GroupState* searched_data = NULL;

  // Check that name field is not empty
  if (name.empty())
  {
    QMessageBox::warning(this, "Error Saving", "A name must be given for the pose!");
    pose_name_field_->setFocus();
    return;
  }
  // Check that a group was selected
  if (group.empty())
  {
    QMessageBox::warning(this, "Error Saving", "A planning group must be chosen!");
    group_name_field_->setFocus();
    return;
  }

  // If creating a new pose, check if the (name, group) pair already exists
  if (!current_edit_pose_)
  {
    searched_data = findPoseByName(name, group);
    if (searched_data != current_edit_pose_)
    {
      if (QMessageBox::warning(this, "Warning Saving", "A pose already exists with that name! Overwrite?",
                               QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::No)
        return;
    }
  }
  else
    searched_data = current_edit_pose_;  // overwrite edited pose

  config_data_->changes |= MoveItConfigData::POSES;

  // Save the new pose name or create the new pose ----------------------------
  bool isNew = false;

  if (searched_data == NULL)  // create new
  {
    isNew = true;
    searched_data = new srdf::Model::GroupState();
  }

  // Copy name data ----------------------------------------------------
  searched_data->name_ = name;
  searched_data->group_ = group;

  // Copy joint positions ----------------------------------------

  // Clear the old values
  searched_data->joint_values_.clear();

  // Iterate through the current group's joints and add to SRDF
  for (std::vector<const robot_model::JointModel*>::const_iterator joint_it = joint_models_.begin();
       joint_it < joint_models_.end(); ++joint_it)
  {
    // Check that this joint only represents 1 variable.
    if ((*joint_it)->getVariableCount() == 1)
    {
      // Create vector for new joint values
      std::vector<double> joint_value;
      joint_value.push_back(joint_state_map_[(*joint_it)->getName()]);

      // Add joint vector to SRDF
      searched_data->joint_values_[(*joint_it)->getName()] = joint_value;
    }
  }

  // Insert new poses into group state vector --------------------------
  if (isNew)
  {
    config_data_->srdf_->group_states_.push_back(*searched_data);
  }

  // Finish up ------------------------------------------------------

  // Reload main screen table
  loadDataTable();

  // Switch to screen
  stacked_layout_->setCurrentIndex(0);

  // Announce that this widget is done with modal mode
  Q_EMIT isModal(false);
}

// ******************************************************************************************
// Cancel changes
// ******************************************************************************************
void RobotPosesWidget::cancelEditing()
{
  // Switch to screen
  stacked_layout_->setCurrentIndex(0);

  // Announce that this widget is done with modal mode
  Q_EMIT isModal(false);
}

// ******************************************************************************************
// Load the robot poses into the table
// ******************************************************************************************
void RobotPosesWidget::loadDataTable()
{
  // Disable Table
  data_table_->setUpdatesEnabled(false);  // prevent table from updating until we are completely done
  data_table_->setDisabled(true);         // make sure we disable it so that the cellChanged event is not called
  data_table_->clearContents();

  // Set size of datatable
  data_table_->setRowCount(config_data_->srdf_->group_states_.size());

  // Loop through every pose
  int row = 0;
  for (std::vector<srdf::Model::GroupState>::const_iterator data_it = config_data_->srdf_->group_states_.begin();
       data_it != config_data_->srdf_->group_states_.end(); ++data_it)
  {
    // Create row elements
    QTableWidgetItem* data_name = new QTableWidgetItem(data_it->name_.c_str());
    data_name->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    QTableWidgetItem* group_name = new QTableWidgetItem(data_it->group_.c_str());
    group_name->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    // Add to table
    data_table_->setItem(row, 0, data_name);
    data_table_->setItem(row, 1, group_name);

    // Increment counter
    ++row;
  }

  // Reenable
  data_table_->setUpdatesEnabled(true);  // prevent table from updating until we are completely done
  data_table_->setDisabled(false);       // make sure we disable it so that the cellChanged event is not called

  // Resize header
  data_table_->resizeColumnToContents(0);
  data_table_->resizeColumnToContents(1);

  // Show edit button if applicable
  if (config_data_->srdf_->group_states_.size())
    btn_edit_->show();
}

// ******************************************************************************************
// Called when setup assistant navigation switches to this screen
// ******************************************************************************************
void RobotPosesWidget::focusGiven()
{
  // Show the current poses screen
  stacked_layout_->setCurrentIndex(0);

  // Load the data to the tree
  loadDataTable();

  // Load the avail groups to the combo box
  loadGroupsComboBox();
}

// ******************************************************************************************
// Call when one of the sliders has its value changed
// ******************************************************************************************
void RobotPosesWidget::updateRobotModel(const std::string& name, double value)
{
  // Save the new value
  joint_state_map_[name] = value;

  // Update the robot model/rviz
  publishJoints();
}

// ******************************************************************************************
// Publish the joint values in the joint_state_map_ to Rviz
// ******************************************************************************************
void RobotPosesWidget::publishJoints()
{
  // Change the scene
  // scene.getCurrentState().setToDefaultValues();//set to default values of 0 OR half between low and high joint values
  // config_data_->getPlanningScene()->getCurrentState().setToRandomValues();

  // Set the joints based on the map
  config_data_->getPlanningScene()->getCurrentStateNonConst().setVariablePositions(joint_state_map_);

  // Create a planning scene message
  moveit_msgs::DisplayRobotState msg;
  robot_state::robotStateToRobotStateMsg(config_data_->getPlanningScene()->getCurrentState(), msg.state);

  // Publish!
  pub_robot_state_.publish(msg);

  // Prevent dirty collision body transforms
  config_data_->getPlanningScene()->getCurrentStateNonConst().update();

  // Decide if current state is in collision
  collision_detection::CollisionResult result;
  config_data_->getPlanningScene()->checkSelfCollision(
      request, result, config_data_->getPlanningScene()->getCurrentState(), config_data_->allowed_collision_matrix_);
  // Show result notification
  if (result.contacts.size())
  {
    collision_warning_->show();
  }
  else
  {
    collision_warning_->hide();
  }
}

// ******************************************************************************************
// ******************************************************************************************
// Slider Widget
// ******************************************************************************************
// ******************************************************************************************

// ******************************************************************************************
// Simple widget for adjusting joints of a robot
// ******************************************************************************************
SliderWidget::SliderWidget(QWidget* parent, const robot_model::JointModel* joint_model, double init_value)
  : QWidget(parent), joint_model_(joint_model)
{
  // Create layouts
  QVBoxLayout* layout = new QVBoxLayout();
  QHBoxLayout* row2 = new QHBoxLayout();

  // Row 1
  joint_label_ = new QLabel(joint_model_->getName().c_str(), this);
  joint_label_->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(joint_label_);

  // Row 2 -------------------------------------------------------------
  joint_slider_ = new QSlider(Qt::Horizontal, this);
  joint_slider_->setTickPosition(QSlider::TicksBelow);
  joint_slider_->setSingleStep(10);
  joint_slider_->setPageStep(500);
  joint_slider_->setTickInterval(1000);
  joint_slider_->setContentsMargins(0, 0, 0, 0);
  row2->addWidget(joint_slider_);

  // Joint Value Box ------------------------------------------------
  joint_value_ = new QLineEdit(this);
  joint_value_->setMaximumWidth(62);
  joint_value_->setContentsMargins(0, 0, 0, 0);
  connect(joint_value_, SIGNAL(editingFinished()), this, SLOT(changeJointSlider()));
  row2->addWidget(joint_value_);

  // Joint Limits ----------------------------------------------------
  const std::vector<moveit_msgs::JointLimits>& limits = joint_model_->getVariableBoundsMsg();
  if (limits.empty())
  {
    QMessageBox::critical(this, "Error Loading", "An internal error has occured while loading the joints");
    return;
  }

  // Only use the first limit, because there is only 1 variable (as checked earlier)
  moveit_msgs::JointLimits joint_limit = limits[0];
  max_position_ = joint_limit.max_position;
  min_position_ = joint_limit.min_position;

  // Set the slider limits
  joint_slider_->setMaximum(max_position_ * 10000);
  joint_slider_->setMinimum(min_position_ * 10000);

  // Connect slider to joint value box
  connect(joint_slider_, SIGNAL(valueChanged(int)), this, SLOT(changeJointValue(int)));

  // Initial joint values -------------------------------------------
  int value = init_value * 10000;           // scale double to integer for slider use
  joint_slider_->setSliderPosition(value);  // set slider value
  changeJointValue(value);                  // show in textbox

  // Finish GUI ----------------------------------------
  layout->addLayout(row2);

  this->setContentsMargins(0, 0, 0, 0);
  // this->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
  this->setGeometry(QRect(110, 80, 120, 80));
  this->setLayout(layout);

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
void SliderWidget::changeJointValue(int value)
{
  // Get joint value
  const double double_value = double(value) / 10000;

  // Set textbox
  joint_value_->setText(QString("%1").arg(double_value, 0, 'f', 4));

  // Send event to parent widget
  Q_EMIT jointValueChanged(joint_model_->getName(), double_value);
}

// ******************************************************************************************
// Called when the joint value box is changed
// ******************************************************************************************
void SliderWidget::changeJointSlider()
{
  // Get joint value
  double value = joint_value_->text().toDouble();

  if (min_position_ > value || value > max_position_)
  {
    value = (min_position_ > value) ? min_position_ : max_position_;
    joint_value_->setText(QString("%1").arg(value, 0, 'f', 4));
  }

  // We assume it converts to double because of the validator
  joint_slider_->setSliderPosition(value * 10000);

  // Send event to parent widget
  Q_EMIT jointValueChanged(joint_model_->getName(), value);
}

}  // namespace
