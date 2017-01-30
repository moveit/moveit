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

#ifndef MOVEIT_MOVEIT_SETUP_ASSISTANT_WIDGETS_DEFAULT_COLLISIONS_WIDGET__
#define MOVEIT_MOVEIT_SETUP_ASSISTANT_WIDGETS_DEFAULT_COLLISIONS_WIDGET__

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QTableWidget>
#include <QSlider>
#include <QPushButton>
#include <QGroupBox>
#include <QProgressBar>
#include <QCheckBox>
#include <QSpinBox>

#ifndef Q_MOC_RUN
#include <boost/thread.hpp>
#include <moveit/setup_assistant/tools/compute_default_collisions.h>
#include <moveit/setup_assistant/tools/moveit_config_data.h>
#endif

#include "header_widget.h"
#include "setup_screen_widget.h"  // a base class for screens in the setup assistant

namespace moveit_setup_assistant
{
/**
 * \brief User interface for editing the default collision matrix list in an SRDF
 */
class DefaultCollisionsWidget : public SetupScreenWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /**
   * \brief User interface for editing the default collision matrix list in an SRDF
   * \param urdf_file String srdf file location. It will create a new file or will edit an existing one
   */
  DefaultCollisionsWidget(QWidget* parent, moveit_setup_assistant::MoveItConfigDataPtr config_data);

  /**
   * \brief Output Link Pairs to SRDF Format
   */
  void linkPairsToSRDF();

  /**
   * \brief Load Link Pairs from SRDF Format
   */
  void linkPairsFromSRDF();

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /**
   * \brief
   Qt close event function for reminding user to saveCreates a thread and updates the GUI progress bar
   */
  void generateCollisionTable();

  /**
   * \brief GUI func for showing sampling density amount
   * \param value Sampling density
   */
  void changeDensityLabel(int value);

  /**
   * \brief Displays data in the link_pairs data structure into a QtTableWidget
   */
  void loadCollisionTable();

  /**
   * \brief Changes the table to show or hide collisions that are not disabled (that have collision checking enabled
   */
  void collisionCheckboxToggle();

  /**
   * \brief Called when user changes data in table, really just the checkbox
   */
  void toggleCheckBox(int row, int column);

  /**
  * \brief Called when current row has changed
  */
  void previewSelected(int row);

  /**
   * \brief Called when setup assistant navigation switches to this screen
   */
  void focusGiven();

private:
  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  QLabel* page_title_;
  QTableWidget* collision_table_;
  QVBoxLayout* layout_;
  QLabel* density_value_label_;
  QSlider* density_slider_;
  QPushButton* btn_generate_;
  QGroupBox* controls_box_;
  QProgressBar* progress_bar_;
  QLabel* progress_label_;
  QCheckBox* collision_checkbox_;
  QGroupBox* controls_box_bottom_;
  QLabel* fraction_label_;
  QSpinBox* fraction_spinbox_;
  QTimer* update_timer_;

  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// main storage of link pair data
  moveit_setup_assistant::LinkPairMap link_pairs_;

  /// Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

  /**
   * \brief The thread that is called to allow the GUI to update. Calls an external function to do calcs
   * \param collision_progress A shared pointer between 3 threads to allow progress bar to update. See declaration
   * location for more details and warning.
   */
  void generateCollisionTableThread(unsigned int* collision_progress);

  /**
   * \brief Helper function to disable parts of GUI during computation
   * \param disable A command
   */
  void disableControls(bool disable);
};
}

#endif
