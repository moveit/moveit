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

#ifndef MOVEIT_ROS_MOVEIT_CONFIGURATION_TOOLS_WIDGETS_START_SCREEN_WIDGET_
#define MOVEIT_ROS_MOVEIT_CONFIGURATION_TOOLS_WIDGETS_START_SCREEN_WIDGET_

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QCheckBox>
#include <QString>
#include <QFont>
#include <QApplication>
#include <QTimer>
#include <QLineEdit>
#include <QSpacerItem>
#include <QFileDialog>
#include <ros/ros.h>
#include <ros/package.h> // for getting file path for loading images
#include "header_widget.h" // title and instructions
#include <fstream>  // for reading in urdf
#include <streambuf>
#include <boost/algorithm/string.hpp> // for trimming whitespace from user input
#include <urdf/model.h> // for testing a valid urdf is loaded
#include <srdf/model.h> // for testing a valid srdf is loaded
#include "moveit_configuration_tools/tools/moveit_config_data.h" // common datastructure class

// Class Prototypes
class SelectModeWidget;
class LoadPathWidget;

/**
 * \brief Start screen user interface for MoveIt Configuration Assistant
 */
class StartScreenWidget : public QWidget
{
  Q_OBJECT

  public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /**
   * \brief Start screen user interface for MoveIt Configuration Assistant
   */
  StartScreenWidget( QWidget* parent, moveit_configuration_tools::MoveItConfigDataPtr config_data );

  ~StartScreenWidget();


  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  SelectModeWidget *select_mode_;
  LoadPathWidget *stack_path_;
  LoadPathWidget *urdf_file_;
  LoadPathWidget *srdf_file_;
  QPushButton *btn_load_;

  /// Contains all the configuration data for the setup assistant
  moveit_configuration_tools::MoveItConfigDataPtr config_data_;

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// User has chosen to show new options
  void showNewOptions();

  /// User has chosen to show edit options
  void showExistingOptions();

  /// Button event for loading user chosen files
  void loadFiles();

Q_SIGNALS:

  /// Event that is fired when the start screen has all its requirements completed and user can move on
  void readyToProgress();

private:


  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// location to save/load SRDF
  //std::string srdf_file_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

};

// ******************************************************************************************
// ******************************************************************************************
// Class for selecting which mode
// ******************************************************************************************
// ******************************************************************************************

class SelectModeWidget : public QFrame
{
  Q_OBJECT

  private:

    private Q_SLOTS:

  public:

  SelectModeWidget( QWidget * parent );

  // Load file button
  QPushButton *btn_new_;
  QPushButton *btn_exist_;

};

// ******************************************************************************************
// ******************************************************************************************
// Class for selecting files
// ******************************************************************************************
// ******************************************************************************************

class LoadPathWidget : public QFrame
{
  Q_OBJECT

  private:
  // Load file button
  QPushButton *button_;
  // Only allow user to select folders
  bool dir_only_;
  // Only allow user to load files (not save)
  bool load_only_;
  // Stores the path qstring
  QLineEdit *path_box_;

private Q_SLOTS:
  /// Load the file dialog
  void btn_file_dialog();

public:

  LoadPathWidget( const std::string &title, const std::string &instructions, 
                  const bool dir_only = false, const bool load_only = false, QWidget * parent=0 );

  /// Returns the file path in QString format
  const QString getQPath();

  /// Returns the file path in std::string format
  const std::string getPath();

  /// Set the URDF path
  void setPath( const QString &path );
};



#endif
