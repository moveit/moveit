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
#include "virtual_joints_widget.h"
// Qt
#include <QFormLayout>
#include <QMessageBox>
#include <QApplication>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Constructor
// ******************************************************************************************
VirtualJointsWidget::VirtualJointsWidget(QWidget* parent, moveit_setup_assistant::MoveItConfigDataPtr config_data)
  : SetupScreenWidget(parent), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();

  // Top Header Area ------------------------------------------------

  HeaderWidget* header =
      new HeaderWidget("Virtual Joints", "Define a virtual joint between a robot link and an external frame of "
                                         "reference (considered fixed with respect to the robot).",
                       this);
  layout->addWidget(header);

  // Create contents screens ---------------------------------------

  vjoint_list_widget_ = createContentsWidget();
  vjoint_edit_widget_ = createEditWidget();

  // Create stacked layout -----------------------------------------
  stacked_layout_ = new QStackedLayout(this);
  stacked_layout_->addWidget(vjoint_list_widget_);  // screen index 0
  stacked_layout_->addWidget(vjoint_edit_widget_);  // screen index 1

  // Create Widget wrapper for layout
  QWidget* stacked_layout_widget = new QWidget(this);
  stacked_layout_widget->setLayout(stacked_layout_);

  layout->addWidget(stacked_layout_widget);

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

// ******************************************************************************************
// Create the main content widget
// ******************************************************************************************
QWidget* VirtualJointsWidget::createContentsWidget()
{
  // Main widget
  QWidget* content_widget = new QWidget(this);

  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Table ------------ ------------------------------------------------

  data_table_ = new QTableWidget(this);
  data_table_->setColumnCount(4);
  data_table_->setSortingEnabled(true);
  data_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  connect(data_table_, SIGNAL(cellDoubleClicked(int, int)), this, SLOT(editDoubleClicked(int, int)));
  connect(data_table_, SIGNAL(cellClicked(int, int)), this, SLOT(previewClicked(int, int)));
  layout->addWidget(data_table_);

  // Set header labels
  QStringList header_list;
  header_list.append("Virtual Joint Name");
  header_list.append("Child Link");
  header_list.append("Parent Frame");
  header_list.append("Type");
  data_table_->setHorizontalHeaderLabels(header_list);

  // Bottom Buttons --------------------------------------------------

  QHBoxLayout* controls_layout = new QHBoxLayout();

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

  // Delete Selected Button
  btn_delete_ = new QPushButton("&Delete Selected", this);
  connect(btn_delete_, SIGNAL(clicked()), this, SLOT(deleteSelected()));
  controls_layout->addWidget(btn_delete_);
  controls_layout->setAlignment(btn_delete_, Qt::AlignRight);

  // Add VJoint Button
  QPushButton* btn_add = new QPushButton("&Add Virtual Joint", this);
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
QWidget* VirtualJointsWidget::createEditWidget()
{
  // Main widget
  QWidget* edit_widget = new QWidget(this);
  // Layout
  QVBoxLayout* layout = new QVBoxLayout();

  // Simple form -------------------------------------------
  QFormLayout* form_layout = new QFormLayout();
  // form_layout->setContentsMargins( 0, 15, 0, 15 );
  form_layout->setRowWrapPolicy(QFormLayout::WrapAllRows);

  // Name input
  vjoint_name_field_ = new QLineEdit(this);
  form_layout->addRow("Virtual Joint Name:", vjoint_name_field_);

  // Child Link input
  child_link_field_ = new QComboBox(this);
  child_link_field_->setEditable(false);
  form_layout->addRow("Child Link:", child_link_field_);

  // Parent frame name input
  parent_name_field_ = new QLineEdit(this);
  form_layout->addRow("Parent Frame Name:", parent_name_field_);

  // Type input
  joint_type_field_ = new QComboBox(this);
  joint_type_field_->setEditable(false);
  loadJointTypesComboBox();  // only do this once
  // connect( joint_type_field_, SIGNAL( currentIndexChanged( const QString & ) ),
  //         this, SLOT( loadJoinSliders( const QString & ) ) );
  form_layout->addRow("Joint Type:", joint_type_field_);

  layout->addLayout(form_layout);

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
// Show edit screen for creating a new vjoint
// ******************************************************************************************
void VirtualJointsWidget::showNewScreen()
{
  // Remember that this is a new vjoint
  current_edit_vjoint_.clear();

  // Clear previous data
  vjoint_name_field_->setText("");
  parent_name_field_->setText("");
  child_link_field_->clearEditText();
  joint_type_field_->clearEditText();  // actually this just chooses first option

  // Switch to screen
  stacked_layout_->setCurrentIndex(1);

  // Announce that this widget is in modal mode
  Q_EMIT isModal(true);
}

// ******************************************************************************************
// Edit whatever element is selected
// ******************************************************************************************
void VirtualJointsWidget::editDoubleClicked(int row, int column)
{
  editSelected();
}

// ******************************************************************************************
// Preview whatever element is selected
// ******************************************************************************************
void VirtualJointsWidget::previewClicked(int row, int column)
{
  // TODO: highlight the virtual joint?

  /*  // Get list of all selected items
  QList<QTableWidgetItem*> selected = data_table_->selectedItems();

  // Check that an element was selected
  if( !selected.size() )
    return;

  // Find the selected in datastructure
  srdf::Model::GroupState *vjoint = findVjointByName( selected[0]->text().toStdString() );

  // Set vjoint joint values by adding them to the local joint state map
  for( std::map<std::string, std::vector<double> >::const_iterator value_it = vjoint->joint_values_.begin();
       value_it != vjoint->joint_values_.end(); ++value_it )
  {
    // Only copy the first joint value
    joint_state_map_[ value_it->first ] = value_it->second[0];
  }

  // Update the joints
  publishJoints();
  */
}

// ******************************************************************************************
// Edit whatever element is selected
// ******************************************************************************************
void VirtualJointsWidget::editSelected()
{
  // Get list of all selected items
  QList<QTableWidgetItem*> selected = data_table_->selectedItems();

  // Check that an element was selected
  if (!selected.size())
    return;

  // Get selected name and edit it
  edit(selected[0]->text().toStdString());
}

// ******************************************************************************************
// Edit vjoint
// ******************************************************************************************
void VirtualJointsWidget::edit(const std::string& name)
{
  // Remember what we are editing
  current_edit_vjoint_ = name;

  // Find the selected in datastruture
  srdf::Model::VirtualJoint* vjoint = findVJointByName(name);

  // Set vjoint name
  vjoint_name_field_->setText(vjoint->name_.c_str());
  parent_name_field_->setText(vjoint->parent_frame_.c_str());

  // Set vjoint child link
  int index = child_link_field_->findText(vjoint->child_link_.c_str());
  if (index == -1)
  {
    QMessageBox::critical(this, "Error Loading", "Unable to find child link in drop down box");
    return;
  }
  child_link_field_->setCurrentIndex(index);

  // Set joint type
  index = joint_type_field_->findText(vjoint->type_.c_str());
  if (index == -1)
  {
    QMessageBox::critical(this, "Error Loading", "Unable to find joint type in drop down box");
    return;
  }
  joint_type_field_->setCurrentIndex(index);

  // Switch to screen
  stacked_layout_->setCurrentIndex(1);

  // Announce that this widget is in modal mode
  Q_EMIT isModal(true);
}

// ******************************************************************************************
// Populate the combo dropdown box with joint types
// ******************************************************************************************
void VirtualJointsWidget::loadJointTypesComboBox()
{
  // Remove all old items
  joint_type_field_->clear();

  // joint types (hard coded)
  joint_type_field_->addItem("fixed");
  joint_type_field_->addItem("floating");
  joint_type_field_->addItem("planar");
}

// ******************************************************************************************
// Populate the combo dropdown box with avail child links
// ******************************************************************************************
void VirtualJointsWidget::loadChildLinksComboBox()
{
  // Remove all old links
  child_link_field_->clear();

  // Get all links in robot model
  std::vector<const robot_model::LinkModel*> link_models = config_data_->getRobotModel()->getLinkModels();

  // Add all links to combo box
  for (std::vector<const robot_model::LinkModel*>::const_iterator link_it = link_models.begin();
       link_it < link_models.end(); ++link_it)
  {
    child_link_field_->addItem((*link_it)->getName().c_str());
  }
}

// ******************************************************************************************
// Find the associated data by name
// ******************************************************************************************
srdf::Model::VirtualJoint* VirtualJointsWidget::findVJointByName(const std::string& name)
{
  // Find the group state we are editing based on the vjoint name
  srdf::Model::VirtualJoint* searched_group = NULL;  // used for holding our search results

  for (std::vector<srdf::Model::VirtualJoint>::iterator vjoint_it = config_data_->srdf_->virtual_joints_.begin();
       vjoint_it != config_data_->srdf_->virtual_joints_.end(); ++vjoint_it)
  {
    if (vjoint_it->name_ == name)  // string match
    {
      searched_group = &(*vjoint_it);  // convert to pointer from iterator
      break;                           // we are done searching
    }
  }

  // Check if vjoint was found
  if (searched_group == NULL)  // not found
  {
    QMessageBox::critical(this, "Error Saving", "An internal error has occured while saving. Quitting.");
    QApplication::quit();
  }

  return searched_group;
}

// ******************************************************************************************
// Delete currently editing item
// ******************************************************************************************
void VirtualJointsWidget::deleteSelected()
{
  // Get list of all selected items
  QList<QTableWidgetItem*> selected = data_table_->selectedItems();

  // Check that an element was selected
  if (!selected.size())
    return;

  // Get selected name and edit it
  current_edit_vjoint_ = selected[0]->text().toStdString();

  // Confirm user wants to delete group
  if (QMessageBox::question(this, "Confirm Virtual Joint Deletion",
                            QString("Are you sure you want to delete the virtual joint '")
                                .append(current_edit_vjoint_.c_str())
                                .append("'?"),
                            QMessageBox::Ok | QMessageBox::Cancel) == QMessageBox::Cancel)
  {
    return;
  }

  // Delete vjoint from vector
  for (std::vector<srdf::Model::VirtualJoint>::iterator vjoint_it = config_data_->srdf_->virtual_joints_.begin();
       vjoint_it != config_data_->srdf_->virtual_joints_.end(); ++vjoint_it)
  {
    // check if this is the group we want to delete
    if (vjoint_it->name_ == current_edit_vjoint_)  // string match
    {
      config_data_->srdf_->virtual_joints_.erase(vjoint_it);
      break;
    }
  }

  // Reload main screen table
  loadDataTable();
  config_data_->changes |= MoveItConfigData::VIRTUAL_JOINTS;
}

// ******************************************************************************************
// Save editing changes
// ******************************************************************************************
void VirtualJointsWidget::doneEditing()
{
  // Get a reference to the supplied strings
  const std::string vjoint_name = vjoint_name_field_->text().toStdString();
  const std::string parent_name = parent_name_field_->text().toStdString();

  // Used for editing existing groups
  srdf::Model::VirtualJoint* searched_data = NULL;

  // Check that name field is not empty
  if (vjoint_name.empty())
  {
    QMessageBox::warning(this, "Error Saving", "A name must be given for the virtual joint!");
    return;
  }

  // Check that parent frame name field is not empty
  if (parent_name.empty())
  {
    QMessageBox::warning(this, "Error Saving", "A name must be given for the parent frame");
    return;
  }

  // Check if this is an existing vjoint
  if (!current_edit_vjoint_.empty())
  {
    // Find the group we are editing based on the goup name string
    searched_data = findVJointByName(current_edit_vjoint_);
  }

  // Check that the vjoint name is unique
  for (std::vector<srdf::Model::VirtualJoint>::const_iterator data_it = config_data_->srdf_->virtual_joints_.begin();
       data_it != config_data_->srdf_->virtual_joints_.end(); ++data_it)
  {
    if (data_it->name_.compare(vjoint_name) == 0)  // the names are the same
    {
      // is this our existing vjoint? check if vjoint pointers are same
      if (&(*data_it) != searched_data)
      {
        QMessageBox::warning(this, "Error Saving", "A virtual joint already exists with that name!");
        return;
      }
    }
  }

  // Check that a joint type was selected
  if (joint_type_field_->currentText().isEmpty())
  {
    QMessageBox::warning(this, "Error Saving", "A joint type must be chosen!");
    return;
  }

  // Check that a child link was selected
  if (child_link_field_->currentText().isEmpty())
  {
    QMessageBox::warning(this, "Error Saving", "A child link must be chosen!");
    return;
  }

  config_data_->changes |= MoveItConfigData::VIRTUAL_JOINTS;

  // Save the new vjoint name or create the new vjoint ----------------------------
  bool isNew = false;

  if (searched_data == NULL)  // create new
  {
    isNew = true;

    searched_data = new srdf::Model::VirtualJoint();
  }

  // Copy name data ----------------------------------------------------
  searched_data->name_ = vjoint_name;
  searched_data->parent_frame_ = parent_name;
  searched_data->child_link_ = child_link_field_->currentText().toStdString();
  searched_data->type_ = joint_type_field_->currentText().toStdString();

  bool emit_frame_notice = false;

  // Insert new vjoints into group state vector --------------------------
  if (isNew)
  {
    if (searched_data->child_link_ == config_data_->getRobotModel()->getRootLinkName())
      emit_frame_notice = true;
    config_data_->srdf_->virtual_joints_.push_back(*searched_data);
    config_data_->updateRobotModel();
  }

  // Finish up ------------------------------------------------------

  // Reload main screen table
  loadDataTable();

  // Switch to screen
  stacked_layout_->setCurrentIndex(0);

  // Announce that this widget is not in modal mode
  Q_EMIT isModal(false);

  // if the root frame changed for the robot, emit the signal
  if (emit_frame_notice)
  {
    Q_EMIT referenceFrameChanged();
  }
}

// ******************************************************************************************
// Cancel changes
// ******************************************************************************************
void VirtualJointsWidget::cancelEditing()
{
  // Switch to screen
  stacked_layout_->setCurrentIndex(0);

  // Announce that this widget is not in modal mode
  Q_EMIT isModal(false);
}

// ******************************************************************************************
// Load the virtual joints into the table
// ******************************************************************************************
void VirtualJointsWidget::loadDataTable()
{
  // Disable Table
  data_table_->setUpdatesEnabled(false);  // prevent table from updating until we are completely done
  data_table_->setDisabled(true);         // make sure we disable it so that the cellChanged event is not called
  data_table_->clearContents();

  // Set size of datatable
  data_table_->setRowCount(config_data_->srdf_->virtual_joints_.size());

  // Loop through every virtual joint
  int row = 0;
  for (std::vector<srdf::Model::VirtualJoint>::const_iterator data_it = config_data_->srdf_->virtual_joints_.begin();
       data_it != config_data_->srdf_->virtual_joints_.end(); ++data_it)
  {
    // Create row elements
    QTableWidgetItem* data_name = new QTableWidgetItem(data_it->name_.c_str());
    data_name->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    QTableWidgetItem* child_link_name = new QTableWidgetItem(data_it->child_link_.c_str());
    child_link_name->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    QTableWidgetItem* parent_frame_name = new QTableWidgetItem(data_it->parent_frame_.c_str());
    parent_frame_name->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    QTableWidgetItem* type_name = new QTableWidgetItem(data_it->type_.c_str());
    type_name->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    // Add to table
    data_table_->setItem(row, 0, data_name);
    data_table_->setItem(row, 1, child_link_name);
    data_table_->setItem(row, 2, parent_frame_name);
    data_table_->setItem(row, 3, type_name);

    // Increment counter
    ++row;
  }

  // Reenable
  data_table_->setUpdatesEnabled(true);  // prevent table from updating until we are completely done
  data_table_->setDisabled(false);       // make sure we disable it so that the cellChanged event is not called

  // Resize header
  data_table_->resizeColumnToContents(0);
  data_table_->resizeColumnToContents(1);
  data_table_->resizeColumnToContents(2);
  data_table_->resizeColumnToContents(3);

  // Show edit button if applicable
  if (config_data_->srdf_->virtual_joints_.size())
    btn_edit_->show();
}

// ******************************************************************************************
// Called when setup assistant navigation switches to this screen
// ******************************************************************************************
void VirtualJointsWidget::focusGiven()
{
  // Show the current vjoints screen
  stacked_layout_->setCurrentIndex(0);

  // Load the data to the tree
  loadDataTable();

  // Load the avail groups to the combo box
  loadChildLinksComboBox();
}

}  // namespace
