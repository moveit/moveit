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

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QFormLayout>
#include <QString>
#include <QGroupBox>
#include "group_edit_widget.h"
#include <pluginlib/class_loader.hpp>  // for loading all avail kinematic planners

namespace moveit_setup_assistant
{
// ******************************************************************************************
//
// ******************************************************************************************
GroupEditWidget::GroupEditWidget(QWidget* parent, moveit_setup_assistant::MoveItConfigDataPtr config_data)
  : QWidget(parent), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();

  QGroupBox* group1 = new QGroupBox("Kinematics");
  QGroupBox* group2 = new QGroupBox("OMPL Planning");

  // Label ------------------------------------------------
  title_ = new QLabel(this);  // specify the title from the parent widget
  QFont group_title_font(QFont().defaultFamily(), 12, QFont::Bold);
  title_->setFont(group_title_font);
  layout->addWidget(title_);

  // Kinematic form -------------------------------------------
  QFormLayout* form_layout = new QFormLayout();
  form_layout->setContentsMargins(0, 12, 0, 12);

  // Name input
  group_name_field_ = new QLineEdit(this);
  group_name_field_->setMaximumWidth(400);
  form_layout->addRow("Group Name:", group_name_field_);

  // Kinematic solver
  kinematics_solver_field_ = new QComboBox(this);
  kinematics_solver_field_->setEditable(false);
  kinematics_solver_field_->setMaximumWidth(400);
  form_layout->addRow("Kinematic Solver:", kinematics_solver_field_);

  // resolution to use with solver
  kinematics_resolution_field_ = new QLineEdit(this);
  kinematics_resolution_field_->setMaximumWidth(400);
  form_layout->addRow("Kin. Search Resolution:", kinematics_resolution_field_);

  // resolution to use with solver
  kinematics_timeout_field_ = new QLineEdit(this);
  kinematics_timeout_field_->setMaximumWidth(400);
  form_layout->addRow("Kin. Search Timeout (sec):", kinematics_timeout_field_);

  // number of IK attempts
  kinematics_attempts_field_ = new QLineEdit(this);
  kinematics_attempts_field_->setMaximumWidth(400);
  form_layout->addRow("Kin. Solver Attempts:", kinematics_attempts_field_);

  group1->setLayout(form_layout);

  // OMPL Planner form --------------------------------------------

  QFormLayout* form_layout2 = new QFormLayout();
  form_layout2->setContentsMargins(0, 12, 0, 12);

  // Kinematic default planner
  default_planner_field_ = new QComboBox(this);
  default_planner_field_->setEditable(false);
  default_planner_field_->setMaximumWidth(400);
  form_layout2->addRow("Group Default Planner:", default_planner_field_);

  group2->setLayout(form_layout2);

  layout->addWidget(group1);
  layout->addWidget(group2);

  layout->setAlignment(Qt::AlignTop);

  // New Group Options  ---------------------------------------------------------
  new_buttons_widget_ = new QWidget();

  QVBoxLayout* new_buttons_layout_container = new QVBoxLayout();
  QHBoxLayout* label_layout = new QHBoxLayout();
  QHBoxLayout* recommended_options = new QHBoxLayout();
  QHBoxLayout* advanced_options = new QHBoxLayout();

  QLabel* save_and_add = new QLabel("Next, Add Components To Group:", this);
  QFont save_and_add_font(QFont().defaultFamily(), 12, QFont::Bold);
  save_and_add->setFont(save_and_add_font);
  label_layout->addWidget(save_and_add);

  // Recommended options
  QLabel* add_subtitle = new QLabel("Recommended: ", this);
  QFont add_subtitle_font(QFont().defaultFamily(), 10, QFont::Bold);
  add_subtitle->setFont(add_subtitle_font);
  recommended_options->addWidget(add_subtitle, 0, Qt::AlignLeft);

  // Save and add joints
  QPushButton* btn_save_joints = new QPushButton("Add Joints", this);
  btn_save_joints->setMaximumWidth(200);
  connect(btn_save_joints, SIGNAL(clicked()), this, SIGNAL(saveJoints()));
  recommended_options->addWidget(btn_save_joints);

  // Advanced options
  QLabel* add_subtitle2 = new QLabel("Advanced Options:", this);
  add_subtitle2->setFont(add_subtitle_font);
  advanced_options->addWidget(add_subtitle2, 0, Qt::AlignLeft);

  // Save and add links
  QPushButton* btn_save_links = new QPushButton("Add Links", this);
  btn_save_links->setMaximumWidth(200);
  connect(btn_save_links, SIGNAL(clicked()), this, SIGNAL(saveLinks()));
  advanced_options->addWidget(btn_save_links);

  // Save and add chain
  QPushButton* btn_save_chain = new QPushButton("Add Kin. Chain", this);
  btn_save_chain->setMaximumWidth(200);
  connect(btn_save_chain, SIGNAL(clicked()), this, SIGNAL(saveChain()));
  advanced_options->addWidget(btn_save_chain);

  // Save and add subgroups
  QPushButton* btn_save_subgroups = new QPushButton("Add Subgroups", this);
  btn_save_subgroups->setMaximumWidth(200);
  connect(btn_save_subgroups, SIGNAL(clicked()), this, SIGNAL(saveSubgroups()));
  advanced_options->addWidget(btn_save_subgroups);

  // Add layouts
  new_buttons_layout_container->addLayout(label_layout);
  new_buttons_layout_container->addLayout(recommended_options);
  new_buttons_layout_container->addLayout(advanced_options);

  // Create widget and add to main layout
  new_buttons_widget_->setLayout(new_buttons_layout_container);
  layout->addWidget(new_buttons_widget_);

  // Verticle Spacer -----------------------------------------------------
  QWidget* vspacer = new QWidget(this);
  vspacer->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  layout->addWidget(vspacer);

  // Bottom Controls ---------------------------------------------------------
  QHBoxLayout* controls_layout = new QHBoxLayout();

  // Delete
  btn_delete_ = new QPushButton("&Delete Group", this);
  btn_delete_->setMaximumWidth(200);
  connect(btn_delete_, SIGNAL(clicked()), this, SIGNAL(deleteGroup()));
  controls_layout->addWidget(btn_delete_);
  controls_layout->setAlignment(btn_delete_, Qt::AlignRight);

  // Horizontal Spacer
  QWidget* spacer = new QWidget(this);
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  controls_layout->addWidget(spacer);

  // Save
  btn_save_ = new QPushButton("&Save", this);
  btn_save_->setMaximumWidth(200);
  connect(btn_save_, SIGNAL(clicked()), this, SIGNAL(save()));
  controls_layout->addWidget(btn_save_);
  controls_layout->setAlignment(btn_save_, Qt::AlignRight);

  // Cancel
  QPushButton* btn_cancel = new QPushButton("&Cancel", this);
  btn_cancel->setMaximumWidth(200);
  connect(btn_cancel, SIGNAL(clicked()), this, SIGNAL(cancelEditing()));
  controls_layout->addWidget(btn_cancel);
  controls_layout->setAlignment(btn_cancel, Qt::AlignRight);

  // Add layout
  layout->addLayout(controls_layout);

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

// ******************************************************************************************
// Set the link field with previous value
// ******************************************************************************************
void GroupEditWidget::setSelected(const std::string& group_name)
{
  group_name_field_->setText(QString(group_name.c_str()));

  // Load properties from moveit_config_data.cpp ----------------------------------------------

  // Load resolution
  double* resolution = &config_data_->group_meta_data_[group_name].kinematics_solver_search_resolution_;
  if (*resolution == 0)
  {
    // Set default value
    *resolution = DEFAULT_KIN_SOLVER_SEARCH_RESOLUTION_;
  }
  kinematics_resolution_field_->setText(QString::number(*resolution));

  // Load timeout
  double* timeout = &config_data_->group_meta_data_[group_name].kinematics_solver_timeout_;
  if (*timeout == 0)
  {
    // Set default value
    *timeout = DEFAULT_KIN_SOLVER_TIMEOUT_;
  }
  kinematics_timeout_field_->setText(QString::number(*timeout));

  // Load attempts
  int* attempts = &config_data_->group_meta_data_[group_name].kinematics_solver_attempts_;
  if (*attempts == 0)
  {
    // Set default value
    *attempts = DEFAULT_KIN_SOLVER_ATTEMPTS_;
  }
  kinematics_attempts_field_->setText(QString::number(*attempts));

  // Set kin solver
  std::string kin_solver = config_data_->group_meta_data_[group_name].kinematics_solver_;

  // If this group doesn't have a solver, reset it to 'None'
  if (kin_solver.empty())
  {
    kin_solver = "None";
  }

  // Set the kin solver combo box
  int index = kinematics_solver_field_->findText(kin_solver.c_str());
  if (index == -1)
  {
    QMessageBox::warning(this, "Missing Kinematic Solvers",
                         QString("Unable to find the kinematic solver '")
                             .append(kin_solver.c_str())
                             .append("'. Trying running rosmake for this package. Until fixed, this setting will be "
                                     "lost the next time the MoveIt! configuration files are generated"));
    return;
  }
  else
  {
    kinematics_solver_field_->setCurrentIndex(index);
  }

  // Set default planner
  std::string default_planner = config_data_->group_meta_data_[group_name].default_planner_;

  // If this group doesn't have a solver, reset it to 'None'
  if (default_planner.empty())
  {
    default_planner = "None";
  }

  index = default_planner_field_->findText(default_planner.c_str());
  if (index == -1)
  {
    QMessageBox::warning(this, "Missing Default Planner",
                         QString("Unable to find the default planner '%1'").arg(default_planner.c_str()));
  }
  else
  {
    default_planner_field_->setCurrentIndex(index);
  }
}

// ******************************************************************************************
// Populate the combo dropdown box with kinematic planners
// ******************************************************************************************
void GroupEditWidget::loadKinematicPlannersComboBox()
{
  // Only load this combo box once
  static bool hasLoaded = false;
  if (hasLoaded)
    return;
  hasLoaded = true;

  // Remove all old items
  kinematics_solver_field_->clear();
  default_planner_field_->clear();

  // Add none option, the default
  kinematics_solver_field_->addItem("None");
  default_planner_field_->addItem("None");

  // load all avail kin planners
  std::unique_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> loader;
  try
  {
    loader.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    QMessageBox::warning(this, "Missing Kinematic Solvers", "Exception while creating class loader for kinematic "
                                                            "solver plugins");
    ROS_ERROR_STREAM(ex.what());
    return;
  }

  // Get classes
  const std::vector<std::string>& classes = loader->getDeclaredClasses();

  // Warn if no plugins are found
  if (classes.empty())
  {
    QMessageBox::warning(this, "Missing Kinematic Solvers", "No MoveIt!-compatible kinematics solvers found. Try "
                                                            "installing moveit_kinematics (sudo apt-get install "
                                                            "ros-${ROS_DISTRO}-moveit-kinematics)");
    return;
  }

  // Loop through all planners and add to combo box
  for (std::vector<std::string>::const_iterator plugin_it = classes.begin(); plugin_it != classes.end(); ++plugin_it)
  {
    kinematics_solver_field_->addItem(plugin_it->c_str());
  }

  std::vector<OMPLPlannerDescription> planners = config_data_->getOMPLPlanners();
  for (std::size_t i = 0; i < planners.size(); ++i)
  {
    std::string planner_name = planners[i].name_;
    default_planner_field_->addItem(planner_name.c_str());
  }
}

}  // namespace
