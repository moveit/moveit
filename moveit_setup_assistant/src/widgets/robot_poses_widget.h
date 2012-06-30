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

#ifndef MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_ROBOT_POSES_WIDGET_
#define MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_ROBOT_POSES_WIDGET_

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QStackedLayout>
#include <QString>
#include "moveit_setup_assistant/tools/moveit_config_data.h"
#include "header_widget.h"
#include "setup_screen_widget.h" // a base class for screens in the setup assistant

namespace moveit_setup_assistant
{

class RobotPosesWidget : public SetupScreenWidget
{
  Q_OBJECT

  public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  RobotPosesWidget( QWidget *parent, moveit_setup_assistant::MoveItConfigDataPtr config_data );

  /// Recieved when this widget is chosen from the navigation menu
  virtual void focusGiven();

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  QTableWidget *data_table_;
  QPushButton *btn_edit_;
  QPushButton *btn_delete_;
  QPushButton *btn_save_;
  QPushButton *btn_cancel_;
  QStackedLayout *stacked_layout_;
  QLineEdit *pose_name_field_;
  QLineEdit *group_name_field_;
  QWidget *joint_list_widget_;
  QVBoxLayout *joint_list_layout_;
  QWidget *pose_list_widget_;
  QWidget *pose_edit_widget_;
                                                                                              
private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// Show edit screen
  void showEditScreen();

  /// Edit whatever element is selected
  void editSelected();

  /// Delete currently editing ite
  void deleteItem();

  /// Save editing changes
  void doneEditing();

  /// Cancel changes
  void cancelEditing();

  /// Run this whenever the group is changed
  void loadJointSliders();

private:

  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;
  
  /// Orignal name of pose currently being edited. This is used to find the element in the vector
  std::string current_edit_pose_;


  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

  /** 
   * Find the associated data by name
   * 
   * @param name - name of data to find in datastructure
   * @return pointer to data in datastructure
   */
  srdf::Model::Group *findGroupByName( const std::string &name );

  /** 
   * Find the associated data by name
   * 
   * @param name - name of data to find in datastructure
   * @return pointer to data in datastructure
   */
  srdf::Model::GroupState *findPoseByName( const std::string &name );

  /** 
   * Create the main list view of poses for robot
   * 
   * @return the widget
   */
  QWidget* createContentsWidget();

  /** 
   * Create the screen for editing poses
   * 
   * @return the widget
   */
  QWidget* createEditWidget();
  
  /** 
   * Load the robot poses into the table
   * 
   */
  void loadDataTable();


};


// ******************************************************************************************
// ******************************************************************************************
// Slider Widget
// ******************************************************************************************
// ******************************************************************************************
class SliderWidget : public QWidget
{
  Q_OBJECT

  public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  SliderWidget( QWidget *parent );


  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

  QLabel *joint_label_;
  QSlider *joint_slider_;
  QLineEdit *joint_value_;
                                                                                              
private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************


private:

  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// Name of joint this edits
  std::string joint_name_;


  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************


};


} //namespace
#endif

