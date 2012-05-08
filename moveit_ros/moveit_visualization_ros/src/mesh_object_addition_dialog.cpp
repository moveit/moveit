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

#include <moveit_visualization_ros/mesh_object_addition_dialog.h>
#include <moveit_visualization_ros/qt_helper_functions.h>
#include <geometric_shapes/shape_operations.h>

#include <QColorDialog>
#include <QObject>
#include <QMetaType>
#include <QGroupBox>
#include <QFileDialog>
#include <QFontMetrics>

namespace moveit_visualization_ros 
{

MeshObjectAdditionDialog::MeshObjectAdditionDialog(QWidget* parent) :
  QDialog(parent),
  selected_color_(128,128,128)
{
  qRegisterMetaType<moveit_msgs::CollisionObject>("CollisionObject");

  QVBoxLayout* layout = new QVBoxLayout(this);
  QGroupBox* panel = new QGroupBox(this);
  panel->setTitle("New Mesh Collision Object");

  QVBoxLayout* panel_layout = new QVBoxLayout();

  collision_object_name_ = new QLineEdit(this);
  panel_layout->addWidget(collision_object_name_);

  QHBoxLayout* filename_layout = new QHBoxLayout();
  QPushButton* select_file_button = new QPushButton("Select mesh file");
  mesh_file_label_ = new QLabel(this);
  mesh_file_label_->setText("No Mesh File Selected");
  filename_layout->addWidget(select_file_button);
  filename_layout->addWidget(mesh_file_label_);
  panel_layout->addLayout(filename_layout);

  QGroupBox* scale_box = new QGroupBox(panel);
  scale_box->setTitle("Scale (x,y,z) %");
  QHBoxLayout* scale_layout = new QHBoxLayout();

  collision_object_scale_x_box_ = new QSpinBox(scale_box);
  collision_object_scale_y_box_ = new QSpinBox(scale_box);
  collision_object_scale_z_box_ = new QSpinBox(scale_box);
  collision_object_scale_x_box_->setRange(1, 1000);
  collision_object_scale_y_box_->setRange(1, 1000);
  collision_object_scale_z_box_->setRange(1, 1000);
  collision_object_scale_x_box_->setValue(100);
  collision_object_scale_y_box_->setValue(100);
  collision_object_scale_z_box_->setValue(100);
  collision_object_scale_x_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  collision_object_scale_y_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  collision_object_scale_z_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  scale_layout->addWidget(collision_object_scale_x_box_);
  scale_layout->addWidget(collision_object_scale_y_box_);
  scale_layout->addWidget(collision_object_scale_z_box_);

  scale_box->setLayout(scale_layout);
  panel_layout->addWidget(scale_box);

  QGroupBox* pos_box = new QGroupBox(panel);
  pos_box->setTitle("Position (x,y,z) cm");
  QHBoxLayout* pos_layout = new QHBoxLayout();

  collision_object_pos_x_box_ = new QSpinBox(pos_box);
  collision_object_pos_y_box_ = new QSpinBox(pos_box);
  collision_object_pos_z_box_ = new QSpinBox(pos_box);
  collision_object_pos_x_box_->setRange(-10000, 10000);
  collision_object_pos_y_box_->setRange(-10000, 10000);
  collision_object_pos_z_box_->setRange(-10000, 10000);
  collision_object_pos_x_box_->setValue(50);
  collision_object_pos_y_box_->setValue(0);
  collision_object_pos_z_box_->setValue(50);
  collision_object_pos_x_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  collision_object_pos_y_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  collision_object_pos_z_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  pos_layout->addWidget(collision_object_pos_x_box_);
  pos_layout->addWidget(collision_object_pos_y_box_);
  pos_layout->addWidget(collision_object_pos_z_box_);

  pos_box->setLayout(pos_layout);
  panel_layout->addWidget(pos_box);

  panel->setLayout(panel_layout);
  layout->addWidget(panel);

  color_button_ = new QPushButton(this);
  color_button_->setText(makeQStringFromColor(selected_color_));
  color_button_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  connect(color_button_, SIGNAL(clicked()), this, SLOT(selectColorButtonPressed()));

  make_object_button_ = new QPushButton(this);
  make_object_button_->setText("Create...");
  make_object_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  make_object_button_->setDefault(true);
  make_object_button_->setDisabled(true);
  connect(make_object_button_, SIGNAL(clicked()), this, SLOT(createObjectConfirmedPressed()));

  connect(collision_object_name_, SIGNAL(textChanged(const QString&)), this, SLOT(collisionNameEdited(const QString&)));
  connect(select_file_button, SIGNAL(clicked()), this,  SLOT(selectFileButtonPressed()));

  layout->addWidget(color_button_);
  layout->addWidget(make_object_button_);
  setLayout(layout);
}

void MeshObjectAdditionDialog::collisionNameEdited(const QString& coll)
{
  if(!coll.isEmpty() && !mesh_file_full_name_.isEmpty()) {
    make_object_button_->setEnabled(true);
  } else {
    make_object_button_->setDisabled(true);
  }
}

void MeshObjectAdditionDialog::selectFileButtonPressed() {
  QFileDialog dialog(this);
  dialog.setFileMode(QFileDialog::ExistingFile);
  dialog.setNameFilter("Meshes (*.stl *.dae)");
  if(dialog.exec()) {
    QStringList mesh_name_list = dialog.selectedFiles();
    mesh_file_full_name_ = mesh_name_list[0];
    mesh_file_label_->setText(mesh_file_label_->fontMetrics().elidedText(mesh_name_list[0], Qt::ElideLeft, 200));
    if(!collision_object_name_->text().isEmpty()) {
      make_object_button_->setEnabled(true);
    }
  }
}

void MeshObjectAdditionDialog::selectColorButtonPressed() {

  selected_color_ = QColorDialog::getColor(selected_color_);
  color_button_->setText(makeQStringFromColor(selected_color_));
};

void MeshObjectAdditionDialog::createObjectConfirmedPressed() {
  moveit_msgs::CollisionObject coll;
  coll.id = collision_object_name_->text().toStdString();
  
  Eigen::Vector3d scale((double)collision_object_scale_x_box_->value() / 100.0f,
                        (double)collision_object_scale_y_box_->value() / 100.0f,
                        (double)collision_object_scale_z_box_->value() / 100.0f);
  
  shapes::Mesh* mesh = shapes::createMeshFromResource("file://"+mesh_file_full_name_.toStdString(), 
                                                      scale);

  if(!mesh) {
    ROS_WARN_STREAM("Some problem loading mesh");
    return;
  }

  coll.shapes.resize(1);
  if(!constructMsgFromShape(mesh, coll.shapes[0])) {
    ROS_WARN_STREAM("Some problem constructing shape msg");
    return;
  }
  delete mesh;

  coll.poses.resize(1);
  coll.poses[0].position.x = (float)collision_object_pos_x_box_->value() / 100.0f;
  coll.poses[0].position.y = (float)collision_object_pos_y_box_->value() / 100.0f;
  coll.poses[0].position.z = (float)collision_object_pos_z_box_->value() / 100.0f;
  coll.poses[0].orientation.x = 0;
  coll.poses[0].orientation.y = 0;
  coll.poses[0].orientation.z = 0;
  coll.poses[0].orientation.w = 1;
  
  addCollisionObjectRequested(coll, selected_color_);
}

}
