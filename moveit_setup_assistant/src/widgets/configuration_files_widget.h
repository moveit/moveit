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

#ifndef MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_CONFIGURATION_FILES_WIDGET_
#define MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_CONFIGURATION_FILES_WIDGET_

#include <QWidget>
#include <QPushButton>
#include <QString>
#include <QProgressBar>
#include <QLabel>
#include <QListWidget>
#include <QList>
#include <moveit/setup_assistant/tools/moveit_config_data.h>
#include "header_widget.h"
#include "setup_screen_widget.h" // a base class for screens in the setup assistant

namespace moveit_setup_assistant
{

class ConfigurationFilesWidget : public SetupScreenWidget
{
  Q_OBJECT

  public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  ConfigurationFilesWidget( QWidget *parent, moveit_setup_assistant::MoveItConfigDataPtr config_data );


  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  QPushButton *btn_save_;
  LoadPathWidget *stack_path_;
  QProgressBar *progress_bar_;
  QListWidget *action_list_;
  QLabel *action_label_;
  QLabel *success_label_;
  QList<QString> action_desc_; // Holds the descriptions explaining all performed actions

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// Save package using default path
  void savePackage();

  /// Quit the program because we are done
  void exitSetupAssistant();

  /// Display the selected action in the desc box
  void changeActionDesc(int id);

private:


  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;

  /// Track progress
  unsigned int action_num;

  /// Total actions - update this whenever a new call to displayAction() is added
  static const unsigned int action_num_total = 24; // note: this is worse case number of actions

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

  /// Verify with user if certain screens have not been completed
  bool checkDependencies();

  /// A function for showing progress and user feedback about what happened
  void displayAction( const QString title, const QString desc, bool skipped = false );

  /// Get the last folder name in a directory path
  const std::string getPackageName( std::string package_path );

  /// Check that no group is empty (without links/joints/etc)
  bool noGroupsEmpty();

  /** 
   * Copy a template from location <template_path> to location <output_path> and replace package name
   * 
   * @param template_path path to template file
   * @param output_path desired path to copy to
   * @param new_package_name name of the new package being created, to replace key word in template
   * 
   * @return bool if the template was copied correctly
   */
  bool copyTemplate( const std::string& template_path, const std::string& output_path, 
                     const std::string& new_package_name );


};

} //namespace moveit_setup_assistant

#endif
