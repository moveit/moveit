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

#include <moveit_manipulation_visualization/household_object_addition_dialog.h>
#include <moveit_visualization_ros/qt_helper_functions.h>
#include <geometric_shapes/shape_operations.h>

#include <QColorDialog>
#include <QObject>
#include <QMetaType>
#include <QGroupBox>
#include <QFileDialog>
#include <QFontMetrics>
#include <QHeaderView>
#include <QVBoxLayout>
#include <QHBoxLayout>

namespace moveit_manipulation_visualization
{

HouseholdObjectAdditionDialog::HouseholdObjectAdditionDialog(QWidget* parent,
                                                             const planning_models::SemanticModelConstPtr& semantic_model) :
  QDialog(parent),
  semantic_model_(semantic_model),
  database_(NULL),
  selected_color_(128,128,128),
  current_id_(-1)
{
  qRegisterMetaType<moveit_msgs::CollisionObject>("CollisionObject");
  qRegisterMetaType<moveit_manipulation_msgs::Grasp>("GraspMsg");

  QVBoxLayout* layout = new QVBoxLayout(this);
  collision_object_name_ = new QLineEdit(this);
  layout->addWidget(collision_object_name_);

  household_objects_table_ = new QTableWidget(this);
  layout->addWidget(household_objects_table_);

  QGroupBox* pos_box = new QGroupBox(this);
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
  layout->addWidget(pos_box);

  
  color_button_ = new QPushButton(this);
  color_button_->setText(moveit_visualization_ros::makeQStringFromColor(selected_color_));
  color_button_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  connect(color_button_, SIGNAL(clicked()), this, SLOT(selectColorButtonPressed()));

  make_object_button_ = new QPushButton(this);
  make_object_button_->setText("Create...");
  make_object_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  make_object_button_->setDefault(true);
  make_object_button_->setDisabled(true);
  connect(make_object_button_, SIGNAL(clicked()), this, SLOT(createObjectConfirmedPressed()));

  connect(collision_object_name_, SIGNAL(textChanged(const QString&)), this, SLOT(collisionNameEdited(const QString&)));
  connect(household_objects_table_, SIGNAL(itemSelectionChanged()), this, SLOT(tableSelectionChanged()));

  layout->addWidget(color_button_);
  layout->addWidget(make_object_button_);
  setLayout(layout);
}

void HouseholdObjectAdditionDialog::showEvent(QShowEvent* event) {
  if(connectToDatabase()) {
    populateDatabaseInformation();
    QDialog::showEvent(event);
    household_objects_table_->horizontalHeader()->resizeSections(QHeaderView::Stretch);
  }
}

bool HouseholdObjectAdditionDialog::connectToDatabase(const std::string& database_host,
                                                      const int& database_port,
                                                      const std::string& database_user,
                                                      const std::string& database_pass,
                                                      const std::string& database_name)
{
  if(database_) return true;
  std::stringstream ss; ss << database_port; 
  std::string database_port_string = ss.str();
  
  database_ = new moveit_household_objects_database::ObjectsDatabase(database_host, database_port_string, database_user, database_pass, database_name);
  if (!database_->isConnected())
  {
    ROS_ERROR("ObjectsDatabaseNode: failed to open model database on host "
              "%s, port %s, user %s with password %s, database %s. Unable to do grasp "
              "planning on database recognized objects. Exiting.",
              database_host.c_str(), database_port_string.c_str(), 
              database_user.c_str(), database_pass.c_str(), database_name.c_str());
    delete database_; database_ = NULL;
    return false;
  } else {
    ROS_INFO_STREAM("Connected to household objects database");
  }
  return true;
}

void HouseholdObjectAdditionDialog::populateDatabaseInformation() {
  std::vector< boost::shared_ptr<moveit_household_objects_database::DatabaseScaledModel> > models;
  if (!database_->getScaledModelsBySet(models, "REDUCED_MODEL_SET"))
  {
    ROS_WARN_STREAM("Error getting models");
  }
  household_objects_table_->clear();
  household_objects_table_->setRowCount(models.size());
  household_objects_table_->setColumnCount(3);

  QStringList labels;
  labels.append("ID");
  labels.append("Name");
  labels.append("Maker");

  for (size_t i=0; i<models.size(); i++)
  {
    std::stringstream ss;
    ss << models[i]->id_.data();
    QTableWidgetItem* id_item = new QTableWidgetItem(QString::fromStdString(ss.str()));
    id_item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    household_objects_table_->setItem(i, 0, id_item);

    QTableWidgetItem* name_item = new QTableWidgetItem(QString::fromStdString(models[i]->model_.data()));
    name_item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    household_objects_table_->setItem(i, 1, name_item);

    QTableWidgetItem* maker_item = new QTableWidgetItem(QString::fromStdString(models[i]->maker_.data()));
    maker_item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    household_objects_table_->setItem(i, 2, maker_item);
  }

  household_objects_table_->setHorizontalHeaderLabels(labels);
  household_objects_table_->setMinimumWidth(500);
  household_objects_table_->horizontalHeader()->resizeSections(QHeaderView::Stretch);
}

bool HouseholdObjectAdditionDialog::loadDatabaseMesh(int model_id) {
  if(!database_->getScaledModelMesh(model_id, loaded_meshes_[model_id])) {
    ROS_WARN_STREAM("Unable to load mesh for " << model_id);
    loaded_meshes_.erase(model_id);
    return false;
  }
  return true;
}

void HouseholdObjectAdditionDialog::generateGraspList(const std::string& obj_name,
                                                      const std::string& arm_name)
{
  std::vector<moveit_manipulation_msgs::Grasp> grasps;
  std::map<std::string, int>::const_iterator it = collision_object_name_to_model_id_map_.find(obj_name);

  if(it == collision_object_name_to_model_id_map_.end()) {
    //emit
    graspListGenerated(false, grasps);
  }
  if(loaded_meshes_.find(it->second) == loaded_meshes_.end()) {
    ROS_WARN_STREAM("Haven't loaded mesh for object " << obj_name << " model id " << it->second);
    //emit
    graspListGenerated(false, grasps);
  }
  if(!loadDatabaseGrasps(it->second, arm_name, grasps)) {
    //emit
    graspListGenerated(false, grasps);
  }
  graspListGenerated(true, grasps);
}                                                     

bool HouseholdObjectAdditionDialog::loadDatabaseGrasps(const int model_id,
                                                       const std::string& arm_name,
                                                       std::vector<moveit_manipulation_msgs::Grasp>& grasps) 
{
  if(semantic_model_->getModelName() != "pr2" &&
     semantic_model_->getModelName() != "pr2_test") {
    ROS_INFO_STREAM("Not generating grasps for non-pr2 robot " << semantic_model_->getModelName());
    return false;
  }
  grasps.clear();
  std::vector< boost::shared_ptr<moveit_household_objects_database::DatabaseGrasp> > db_grasps;
  if(!database_->getClusterRepGrasps(model_id, 
                                     "WILLOW_GRIPPER_2010",
                                     db_grasps)) {
    ROS_WARN_STREAM("Unable to load grasps for " << model_id);
    return false;
  }
  std::string end_effector_group = semantic_model_->getEndEffector(arm_name);
  std::vector<std::string> end_effector_joints = semantic_model_->getGroupJoints(end_effector_group);

  std::vector< boost::shared_ptr<moveit_household_objects_database::DatabaseGrasp> >::iterator it;
  for (it = db_grasps.begin(); it != db_grasps.end(); it++)
  {
    moveit_manipulation_msgs::Grasp grasp;
    grasp.pre_grasp_posture.name = end_effector_joints;
    grasp.grasp_posture.name = end_effector_joints;
    grasp.pre_grasp_posture.position.resize(end_effector_joints.size(),
                                            (*it)->pre_grasp_posture_.get().joint_angles_.at(0));
    grasp.grasp_posture.position.resize(end_effector_joints.size(),
                                        (*it)->final_grasp_posture_.get().joint_angles_.at(0));
    if(grasp.grasp_posture.position.size() != end_effector_joints.size()) {
      ROS_WARN_STREAM("Have " << end_effector_joints.size() << " but only " << grasp.grasp_posture.position.size() << " positions");
    } else {
      for(unsigned int i = 0; i < grasp.grasp_posture.position.size(); i++) {
        ROS_DEBUG_STREAM("Joint " << end_effector_joints[i] << " position " << grasp.grasp_posture.position[i]);
      }
    }	
    grasp.desired_approach_distance = 0.10;
    grasp.min_approach_distance = 0.05;
    grasp.grasp_pose = (*it)->final_grasp_pose_.get().pose_;
    grasps.push_back(grasp);
  }
  return true;
}

void HouseholdObjectAdditionDialog::collisionNameEdited(const QString& coll)
{
  if(!coll.isEmpty() && current_id_ >= 0) {
    make_object_button_->setEnabled(true);
  } else {
    make_object_button_->setDisabled(true);
  }
}

void HouseholdObjectAdditionDialog::tableSelectionChanged() 
{
  QList<QTableWidgetItem*> items = household_objects_table_->selectedItems();
  
  if(items.size() > 0)
  {
    QTableWidgetItem* item = items[0];
    QTableWidgetItem* id_item = household_objects_table_->item(item->row(),0);
    std::stringstream id_stream(id_item->text().toStdString());
    id_stream >> current_id_;
    if(!collision_object_name_->text().isEmpty()) {
      make_object_button_->setEnabled(true);      
    }
  } else {
    current_id_ = -1;
    make_object_button_->setDisabled(true);
  }
}

void HouseholdObjectAdditionDialog::selectColorButtonPressed() {

  selected_color_ = QColorDialog::getColor(selected_color_);
  color_button_->setText(moveit_visualization_ros::makeQStringFromColor(selected_color_));
};

void HouseholdObjectAdditionDialog::createObjectConfirmedPressed() {
  moveit_msgs::CollisionObject coll;
  coll.id = collision_object_name_->text().toStdString();

  if(!loadDatabaseMesh(current_id_)) {
    return;
  }

  collision_object_name_to_model_id_map_[coll.id] = current_id_;
  
  coll.shapes.push_back(loaded_meshes_[current_id_]);

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
