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

#ifndef MOVEIT_ROS_MOVEIT_CONFIGURATION_TOOLS_WIDGETS_COMPUTE_DEFAULT_COLLISIONS_
#define MOVEIT_ROS_MOVEIT_CONFIGURATION_TOOLS_WIDGETS_COMPUTE_DEFAULT_COLLISIONS_

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSlider>
#include <QProgressDialog>
#include <QProgressBar>
#include <QTableWidget>
#include <QCheckBox>
#include <QString>
#include <QFont>
#include <QApplication>
#include <boost/thread.hpp>
#include "ros/ros.h"
#include <moveit_configuration_tools/tools/compute_default_collisions.h>

class ComputeDefaultCollisionsWidget : public QWidget
{
  Q_OBJECT

public:
  ComputeDefaultCollisionsWidget();

private Q_SLOTS:
  void quit();
  void generateCollisionTable();
  void changeDensityLabel(int value);

private:
  // Qt Components
  QTableWidget *collision_table_;
  QPushButton *quitButton_;
  QVBoxLayout *layout_;
  QLabel *density_value_label_;
  QSlider *density_slider_;
  QPushButton *generate_button_;
  QGroupBox *controls_box_;
  QProgressBar *progress_bar_;
  QLabel *progress_label_;

  // Data 
  moveit_configuration_tools::LinkPairMap link_pairs;

  // Functions
  void generateCollisionTableThread( unsigned int *collision_progress );
                                     
  void loadCollisionTable();

  void disableControls(bool disable);
};

#endif
