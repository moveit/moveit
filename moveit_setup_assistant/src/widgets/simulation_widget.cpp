/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Mohamad Ayman.
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
 *   * The name of Mohamad Ayman may not be used to endorse or promote products derived
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

/* Author: Mohamad Ayman */

// SA
#include "simulation_widget.h"
#include "header_widget.h"

// Qt
#include <QColor>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <regex>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Constructor
// ******************************************************************************************
SimulationWidget::SimulationWidget(QWidget* parent, const MoveItConfigDataPtr& config_data)
  : SetupScreenWidget(parent), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();
  layout->setAlignment(Qt::AlignTop);

  // Top Header Area ------------------------------------------------

  HeaderWidget* header = new HeaderWidget("Simulate With Gazebo",
                                          "The following tool will auto-generate the URDF changes needed "
                                          "for Gazebo compatibility with ROSControl and MoveIt. The "
                                          "needed changes are shown in green.",
                                          this);
  layout->addWidget(header);

  // Spacing
  QSpacerItem* blank_space = new QSpacerItem(1, 8);
  layout->addSpacerItem(blank_space);

  QLabel* instructions = new QLabel(this);
  instructions->setText("You can run the following command to quickly find the necessary URDF file to edit:");
  layout->addWidget(instructions);

  QTextEdit* instructions_command = new QTextEdit(this);
  instructions_command->setText(std::string("roscd " + config_data->urdf_pkg_name_).c_str());
  instructions_command->setReadOnly(true);
  instructions_command->setMaximumHeight(30);
  layout->addWidget(instructions_command);

  // Spacing
  blank_space = new QSpacerItem(1, 6);
  layout->addSpacerItem(blank_space);

  // Top Buttons --------------------------------------------------
  QHBoxLayout* controls_layout = new QHBoxLayout();

  // Used to make the new URDF visible
  QPushButton* btn_generate = new QPushButton("&Generate URDF", this);
  btn_generate->setMinimumWidth(180);
  btn_generate->setMinimumHeight(40);
  connect(btn_generate, SIGNAL(clicked()), this, SLOT(generateURDFClick()));
  controls_layout->addWidget(btn_generate);
  controls_layout->setAlignment(btn_generate, Qt::AlignLeft);

  // Used to overwrite the original URDF
  btn_overwrite_ = new QPushButton("&Overwrite original URDF", this);
  btn_overwrite_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_overwrite_->setMinimumWidth(180);
  btn_overwrite_->setMinimumHeight(40);
  btn_overwrite_->setEnabled(false);
  connect(btn_overwrite_, SIGNAL(clicked()), this, SLOT(overwriteURDF()));
  controls_layout->addWidget(btn_overwrite_);
  controls_layout->setAlignment(btn_overwrite_, Qt::AlignLeft);

  // Spacer
  controls_layout->addItem(new QSpacerItem(20, 20, QSizePolicy::Expanding, QSizePolicy::Minimum));

  // Add layout
  layout->addLayout(controls_layout);

  // When there are no changes to be made
  no_changes_label_ = new QLabel(this);
  no_changes_label_->setText("URDF is ready for Gazebo. No changes required.");
  no_changes_label_->setFont(QFont(QFont().defaultFamily(), 18));
  layout->addWidget(no_changes_label_);
  no_changes_label_->setVisible(false);

  // URDF text
  simulation_text_ = new QTextEdit(this);
  simulation_text_->setLineWrapMode(QTextEdit::NoWrap);
  layout->addWidget(simulation_text_);

  // Copy URDF link, hidden initially
  copy_urdf_ = new QLabel(this);
  copy_urdf_->setText("<a href='contract'>Copy to Clipboard</a>");
  connect(copy_urdf_, SIGNAL(linkActivated(const QString)), this, SLOT(copyURDF(const QString)));
  copy_urdf_->setVisible(false);
  layout->addWidget(copy_urdf_);

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

// ******************************************************************************************
// Called when generate URDF button is clicked
// ******************************************************************************************
void SimulationWidget::generateURDFClick()
{
  simulation_text_->setVisible(true);
  std::string text = config_data_->getGazeboCompatibleURDF();
  config_data_->gazebo_urdf_string_ = text;

  // Check if the urdf do need new elements to be added
  displayURDF();

  // Add generated Gazebo URDF to config file if not empty
  bool have_changes = !text.empty();
  config_data_->save_gazebo_urdf_ = have_changes;

  // GUI elements are visible only if there are URDF changes to display/edit
  simulation_text_->setVisible(have_changes);
  btn_overwrite_->setVisible(have_changes);
  copy_urdf_->setVisible(have_changes);
  no_changes_label_->setVisible(!have_changes);

  // Disable overwrite button if URDF originates from xacro
  btn_overwrite_->setDisabled(config_data_->urdf_from_xacro_);
  QString tooltip;
  if (btn_overwrite_->isEnabled())
    tooltip = tr("Overwrite URDF in original location:\n").append(config_data_->urdf_path_.c_str());
  else
    tooltip = tr("Cannot overwrite original, <i>xacro-based</i> URDF");
  btn_overwrite_->setToolTip(tooltip);

  if (have_changes)
    config_data_->changes |= MoveItConfigData::SIMULATION;
  else
    config_data_->changes &= ~MoveItConfigData::SIMULATION;
}

// ******************************************************************************************
// Called when save URDF button is clicked
// ******************************************************************************************
void SimulationWidget::overwriteURDF()
{
  if (!config_data_->outputGazeboURDFFile(config_data_->urdf_path_))
    QMessageBox::warning(this, "Gazebo URDF", tr("Failed to save to ").append(config_data_->urdf_path_.c_str()));
  else  // Display success message
    QMessageBox::information(this, "Overwriting Successfull",
                             "Original robot description URDF was successfully overwritten.");

  // Remove Gazebo URDF file from list of to-be-written config files
  config_data_->save_gazebo_urdf_ = false;
  config_data_->changes &= ~MoveItConfigData::SIMULATION;
}

// ******************************************************************************************
// Displays the gazebo urdf on the GUI
// ******************************************************************************************
void SimulationWidget::displayURDF()
{
  std::size_t urdf_length = config_data_->gazebo_urdf_string_.length();
  simulation_text_->clear();

  if (urdf_length > 0)
  {
    // Split the added elements from the original urdf to view them in different colors
    std::smatch start_match;
    std::smatch end_match;
    std::regex start_reg_ex("<inertial");
    std::regex end_reg_ex("</inertial");

    // Search for inertial elemnts using regex
    std::regex_search(config_data_->gazebo_urdf_string_, start_match, start_reg_ex);
    std::regex_search(config_data_->gazebo_urdf_string_, end_match, end_reg_ex);

    // Used to cache the positions of the opening and closing of the inertial elements
    std::vector<int> inertial_opening_matches;
    std::vector<int> inertial_closing_matches;

    inertial_closing_matches.push_back(0);

    // Cache the positions of the openings of the inertial elements
    for (auto it = std::sregex_iterator(config_data_->gazebo_urdf_string_.begin(),
                                        config_data_->gazebo_urdf_string_.end(), start_reg_ex);
         it != std::sregex_iterator(); ++it)
    {
      inertial_opening_matches.push_back(it->position());
    }

    inertial_opening_matches.push_back(urdf_length);

    // Cache the positions of the closings of the inertial elements
    for (auto it = std::sregex_iterator(config_data_->gazebo_urdf_string_.begin(),
                                        config_data_->gazebo_urdf_string_.end(), end_reg_ex);
         it != std::sregex_iterator(); ++it)
    {
      inertial_closing_matches.push_back(it->position());
    }

    for (std::size_t match_number = 0; match_number < inertial_opening_matches.size() - 1; match_number++)
    {
      // Show the unmodified elements in black
      simulation_text_->setTextColor(QColor("black"));
      simulation_text_->append(
          QString(config_data_->gazebo_urdf_string_
                      .substr(inertial_closing_matches[match_number],
                              inertial_opening_matches[match_number] - inertial_closing_matches[match_number])
                      .c_str()));

      // Show the added elements in green
      simulation_text_->setTextColor(QColor("green"));

      simulation_text_->append(
          QString(config_data_->gazebo_urdf_string_
                      .substr(inertial_opening_matches[match_number],
                              inertial_closing_matches[match_number + 1] - inertial_opening_matches[match_number] + 11)
                      .c_str()));
      inertial_closing_matches[match_number + 1] += 11;
    }

    // Position of the first transmission element in the urdf
    std::size_t first_transmission = config_data_->gazebo_urdf_string_.find("<transmission");

    // Position of the last inertial element in the urdf
    std::size_t last_inertial = inertial_closing_matches[inertial_opening_matches.size() - 1];

    if (first_transmission != std::string::npos)
    {
      simulation_text_->setTextColor(QColor("black"));
      simulation_text_->append(
          QString(config_data_->gazebo_urdf_string_.substr(last_inertial, first_transmission - last_inertial).c_str()));

      // Write from the first transmission element until the closing robot element in green
      simulation_text_->setTextColor(QColor("green"));
      simulation_text_->append(QString(
          config_data_->gazebo_urdf_string_.substr(first_transmission, urdf_length - first_transmission - 10).c_str()));

      // Write the closing robot element in black
      simulation_text_->setTextColor(QColor("black"));
      simulation_text_->append(QString("</robot>"));
    }
  }
  else
  {
    simulation_text_->clear();
  }
}

// ******************************************************************************************
// Called the copy to clipboard button is clicked
// ******************************************************************************************
void SimulationWidget::copyURDF(const QString& /*link*/)
{
  simulation_text_->selectAll();
  simulation_text_->copy();
}

}  // namespace moveit_setup_assistant
