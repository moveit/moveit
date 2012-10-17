/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: E. Gil Jones

#ifndef _PRIMITIVE_OBJECT_ADDITION_DIALOG_H_
#define _PRIMITIVE_OBJECT_ADDITION_DIALOG_H_

#include <sstream>

#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QGroupBox>
#include <QSpinBox>
#include <QString>
#include <QLineEdit>
#include <QPushButton>

#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/ColorRGBA.h>

namespace moveit_visualization_ros
{

class PrimitiveObjectAdditionDialog: public QDialog
{
  Q_OBJECT
  
  public:
  
  PrimitiveObjectAdditionDialog(QWidget* parent);

public Q_SLOTS:

  void selectColorButtonPressed();
  void createObjectConfirmedPressed();

Q_SIGNALS:

  void addCollisionObjectRequested(const moveit_msgs::CollisionObject& obj,
                                   const QColor& color);

protected:

  QComboBox* collision_object_type_box_;
  QLineEdit* collision_object_name_;
  QSpinBox* collision_object_scale_x_box_;
  QSpinBox* collision_object_scale_y_box_;
  QSpinBox* collision_object_scale_z_box_;
  QSpinBox* collision_object_pos_x_box_;
  QSpinBox* collision_object_pos_y_box_;
  QSpinBox* collision_object_pos_z_box_;
  QPushButton* make_object_button_;

  QPushButton* color_button_;
  QColor selected_color_;
};

}

#endif
