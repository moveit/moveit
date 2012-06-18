/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <moveit_visualization_ros/primitive_object_addition_dialog.h>
#include <moveit_visualization_ros/qt_helper_functions.h>

#include <QColorDialog>
#include <QObject>
#include <QMetaType>

namespace moveit_visualization_ros 
{

PrimitiveObjectAdditionDialog::PrimitiveObjectAdditionDialog(QWidget* parent) :
  QDialog(parent),
  selected_color_(128,128,128)
{
  qRegisterMetaType<moveit_msgs::CollisionObject>("CollisionObject");

  QVBoxLayout* layout = new QVBoxLayout(this);
  QGroupBox* panel = new QGroupBox(this);
  panel->setTitle("New Primitive Collision Object");

  QVBoxLayout* panelLayout = new QVBoxLayout();

  collision_object_name_ = new QLineEdit(this);
  panelLayout->addWidget(collision_object_name_);

  collision_object_type_box_ = new QComboBox(this);
  collision_object_type_box_->addItem("Box");
  collision_object_type_box_->addItem("Cylinder");
  collision_object_type_box_->addItem("Sphere");
  collision_object_type_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  panelLayout->addWidget(collision_object_type_box_);

  QGroupBox* scaleBox = new QGroupBox(panel);
  scaleBox->setTitle("Scale (x,y,z) cm");
  QHBoxLayout* scaleLayout = new QHBoxLayout();

  collision_object_scale_x_box_ = new QSpinBox(scaleBox);
  collision_object_scale_y_box_ = new QSpinBox(scaleBox);
  collision_object_scale_z_box_ = new QSpinBox(scaleBox);
  collision_object_scale_x_box_->setRange(1, 10000);
  collision_object_scale_y_box_->setRange(1, 10000);
  collision_object_scale_z_box_->setRange(1, 10000);
  collision_object_scale_x_box_->setValue(10);
  collision_object_scale_y_box_->setValue(10);
  collision_object_scale_z_box_->setValue(10);
  collision_object_scale_x_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  collision_object_scale_y_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  collision_object_scale_z_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  scaleLayout->addWidget(collision_object_scale_x_box_);
  scaleLayout->addWidget(collision_object_scale_y_box_);
  scaleLayout->addWidget(collision_object_scale_z_box_);

  scaleBox->setLayout(scaleLayout);
  panelLayout->addWidget(scaleBox);

  QGroupBox* posBox = new QGroupBox(panel);
  posBox->setTitle("Position (x,y,z) cm");
  QHBoxLayout* posLayout = new QHBoxLayout();

  collision_object_pos_x_box_ = new QSpinBox(posBox);
  collision_object_pos_y_box_ = new QSpinBox(posBox);
  collision_object_pos_z_box_ = new QSpinBox(posBox);
  collision_object_pos_x_box_->setRange(-10000, 10000);
  collision_object_pos_y_box_->setRange(-10000, 10000);
  collision_object_pos_z_box_->setRange(-10000, 10000);
  collision_object_pos_x_box_->setValue(50);
  collision_object_pos_y_box_->setValue(0);
  collision_object_pos_z_box_->setValue(50);
  collision_object_pos_x_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  collision_object_pos_y_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  collision_object_pos_z_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  posLayout->addWidget(collision_object_pos_x_box_);
  posLayout->addWidget(collision_object_pos_y_box_);
  posLayout->addWidget(collision_object_pos_z_box_);

  posBox->setLayout(posLayout);
  panelLayout->addWidget(posBox);

  panel->setLayout(panelLayout);
  layout->addWidget(panel);

  color_button_ = new QPushButton(this);
  color_button_->setText(makeQStringFromColor(selected_color_));
  color_button_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  connect(color_button_, SIGNAL(clicked()), this, SLOT(selectColorButtonPressed()));

  make_object_button_ = new QPushButton(this);
  make_object_button_->setText("Create...");
  make_object_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  make_object_button_->setDefault(true);
  connect(make_object_button_, SIGNAL(clicked()), this, SLOT(createObjectConfirmedPressed()));

  layout->addWidget(color_button_);
  layout->addWidget(make_object_button_);
  setLayout(layout);
}

void PrimitiveObjectAdditionDialog::selectColorButtonPressed() {

  selected_color_ = QColorDialog::getColor(selected_color_);
  color_button_->setText(makeQStringFromColor(selected_color_));
};

void PrimitiveObjectAdditionDialog::createObjectConfirmedPressed() {
  moveit_msgs::CollisionObject coll;
  coll.id = collision_object_name_->text().toStdString();
  
  coll.primitives.resize(1);
  std::string object_type = collision_object_type_box_->currentText().toStdString();
  if(object_type == "Box") {
    coll.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    coll.primitives[0].dimensions.x = (float)collision_object_scale_x_box_->value() / 100.0f;
    coll.primitives[0].dimensions.y = (float)collision_object_scale_y_box_->value() / 100.0f;
    coll.primitives[0].dimensions.z = (float)collision_object_scale_z_box_->value() / 100.0f;
  } else if (object_type == "Cylinder") {
    coll.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    coll.primitives[0].dimensions.x = (float)collision_object_scale_x_box_->value() / 100.0f;
    coll.primitives[0].dimensions.z = (float)collision_object_scale_z_box_->value() / 100.0f;
  } else if (object_type == "Sphere") {
    coll.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
    coll.primitives[0].dimensions.x = (float)collision_object_scale_x_box_->value() / 100.0f;
  }

  coll.primitive_poses.resize(1);
  coll.primitive_poses[0].position.x = (float)collision_object_pos_x_box_->value() / 100.0f;
  coll.primitive_poses[0].position.y = (float)collision_object_pos_y_box_->value() / 100.0f;
  coll.primitive_poses[0].position.z = (float)collision_object_pos_z_box_->value() / 100.0f;
  coll.primitive_poses[0].orientation.x = 0;
  coll.primitive_poses[0].orientation.y = 0;
  coll.primitive_poses[0].orientation.z = 0;
  coll.primitive_poses[0].orientation.w = 1;
  
  addCollisionObjectRequested(coll, selected_color_);
}

}
