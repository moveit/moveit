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

#ifndef MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_DOUBLE_LIST_WIDGET_
#define MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_DOUBLE_LIST_WIDGET_

#include <QWidget>
#include <QLabel>
#include <QTableWidget>
#include "moveit_setup_assistant/tools/moveit_config_data.h" // common datastructure class

class DoubleListWidget : public QWidget
{
  Q_OBJECT

// ******************************************************************************************
// Reusable double list widget for selecting and deselecting a subset from a set
// ******************************************************************************************
public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /// Constructor
  DoubleListWidget( QWidget *parent, moveit_setup_assistant::MoveItConfigDataPtr config_data, 
                    QString long_name, QString short_name );

  /// Loads the availble data list
  void setAvailable( const std::vector<std::string> &items );

  /// Set the right box
  void setSelected( const std::vector<std::string> &items );

  /// Convenience function for reusing set table code
  void setTable( const std::vector<std::string> &items, QTableWidget *table );

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  
  QTableWidget *data_table_;
  QTableWidget *selected_data_table_;
  QLabel *title_; // specify the title from the parent widget

  /// Name of datatype
  QString long_name_;
  QString short_name_;

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// Move selected data right
  void selectDataButtonClicked();

  /// Move selected data left
  void deselectDataButtonClicked();

Q_SIGNALS:

  // ******************************************************************************************
  // Emitted Signals
  // ******************************************************************************************

  /// Event sent when this widget is done making data changes and parent widget can save
  void doneEditing();

  /// Event sent when user presses cancel button
  void cancelEditing();

private:


  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

};

#endif
