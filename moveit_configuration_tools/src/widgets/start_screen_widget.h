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
  StartScreenWidget();


  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  QFrame *stack_path_;
  QFrame *urdf_file_;
  QFrame *srdf_file_;
  QPushButton *btn_load_;

  private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /**
   * \brief Button click event for loading all fiels
   */
  void loadFiles();

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


/**
 * \brief Re-usable file selection widget with instructions and box
 */
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

private Q_SLOTS:
  /// Load the file dialog
  void btn_file_dialog();

public:
  QLineEdit *path_box_;

  /** 
   * Create sub-widet
   * 
   * @param title Title of frame
   * @param instructions User advice
   * @param dir_only Only allow user to load a folder
   * @param load_only Only allow user to select an existing file
   * 
   * @return 
   */  
  LoadPathWidget( const std::string &title, const std::string &instructions, 
                  const bool dir_only = false, const bool load_only = false );
};

#endif
