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
#include <planning_models/transforms.h>

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
                                                             const planning_models::KinematicModelConstPtr& kinematic_model) :
  QDialog(parent),
  kinematic_model_(kinematic_model),
  database_(NULL),
  selected_color_(128,128,128),
  current_id_(-1)
{
  ros::NodeHandle nh;
  nh.param("grasp_database_host", grasp_database_host_, std::string("wgs36.willowgarage.com"));
  nh.param("grasp_database_port", grasp_database_port_, 5432);
  nh.param("grasp_database_user", grasp_database_user_, std::string("willow"));
  nh.param("grasp_database_pass", grasp_database_pass_, std::string("willow"));
  nh.param("grasp_database_name", grasp_database_name_, std::string("household_objects"));

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
  if(connectToDatabase(grasp_database_host_,
                       grasp_database_port_,
                       grasp_database_user_,
                       grasp_database_pass_,
                       grasp_database_name_)) {
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

void HouseholdObjectAdditionDialog::getMeshGivenModelId(int model_id) {
  if(loaded_meshes_.find(model_id) != loaded_meshes_.end()) {
    Q_EMIT modelMeshFetched(loaded_meshes_.at(model_id));
  } else if(loadDatabaseMesh(model_id)) {
    Q_EMIT modelMeshFetched(loaded_meshes_.at(model_id));
  }
}

bool HouseholdObjectAdditionDialog::loadDatabaseMesh(int model_id) {
  if(!database_->getScaledModelMesh(model_id, loaded_meshes_[model_id])) {
    ROS_WARN_STREAM("Unable to load mesh for " << model_id);
    loaded_meshes_.erase(model_id);
    return false;
  }
  return true;
}

void HouseholdObjectAdditionDialog::generateGraspList(const std::string& database_name,
                                                      const std::string& obj_name,
                                                      const std::string& arm_name)
{
  std::vector<moveit_manipulation_msgs::Grasp> grasps;
  std::map<std::string, int>::const_iterator it = collision_object_name_to_model_id_map_.find(obj_name);

  if(it == collision_object_name_to_model_id_map_.end()) {
    Q_EMIT graspListGenerated(false, grasps);
  }
  if(loaded_meshes_.find(it->second) == loaded_meshes_.end()) {
    ROS_WARN_STREAM("Haven't loaded mesh for object " << obj_name << " model id " << it->second);
    Q_EMIT graspListGenerated(false, grasps);
    return;
  }
  if(!loadDatabaseGrasps(database_name, it->second, arm_name, grasps)) {
    Q_EMIT graspListGenerated(false, grasps);
    return;
  }
  Q_EMIT graspListGenerated(true, grasps);
}                                                     

bool HouseholdObjectAdditionDialog::loadDatabaseGrasps(const std::string& database_name,
                                                       const int model_id,
                                                       const std::string& arm_name,
                                                       std::vector<moveit_manipulation_msgs::Grasp>& grasps) 
{
  if(database_name.empty()) {
    ROS_INFO_STREAM("Not generating grasps as database name is empty ");
    return false;
  }
  bool is_pr2 = false;
  if(kinematic_model_->getName() == "pr2" ||
     kinematic_model_->getName() == "pr2_test") {
    is_pr2 = true;
  } 
  grasps.clear();
  std::vector< boost::shared_ptr<moveit_household_objects_database::DatabaseGrasp> > db_grasps;
  if(is_pr2 && !database_->getClusterRepGrasps(model_id, 
                                               database_name,
                                               db_grasps)) {
    ROS_WARN_STREAM("Unable to load grasps for " << model_id);
    return false;
  } else if(!database_->getGrasps(model_id, 
                                  database_name,
                                  db_grasps)) {
    ROS_WARN_STREAM("Unable to load grasps for " << model_id << " database " << database_name);
    return false;
  }

  std::string end_effector_group = kinematic_model_->getJointModelGroup(arm_name)->getAttachedEndEffectorGroupName();
  std::vector<std::string> end_effector_joints = kinematic_model_->getJointModelGroup(end_effector_group)->getJointModelNames();

  std::vector< boost::shared_ptr<moveit_household_objects_database::DatabaseGrasp> >::iterator it;
  for (it = db_grasps.begin(); it != db_grasps.end(); it++)
  {
    moveit_manipulation_msgs::Grasp grasp;
    grasp.pre_grasp_posture.name = end_effector_joints;
    grasp.grasp_posture.name = end_effector_joints;
    if(is_pr2) {
      grasp.pre_grasp_posture.position.resize(end_effector_joints.size(),
                                              (*it)->pre_grasp_posture_.get().joint_angles_.at(0));
      grasp.grasp_posture.position.resize(end_effector_joints.size(),
                                          (*it)->final_grasp_posture_.get().joint_angles_.at(0));
    } else {
      
      if((*it)->pre_grasp_posture_.get().joint_angles_.size() != end_effector_joints.size()) {
        ROS_WARN_STREAM("Size mismatch for pre-grasp between end effector joints num " << end_effector_joints.size() 
                        << " and number of joint angles from database " << (*it)->pre_grasp_posture_.get().joint_angles_.size());
      } 
      if((*it)->final_grasp_posture_.get().joint_angles_.size() != end_effector_joints.size()) {
        ROS_WARN_STREAM("Size mismatch for pre-grasp between end effector joints num " << end_effector_joints.size() 
                        << " and number of joint angles from database " << (*it)->final_grasp_posture_.get().joint_angles_.size());
      } 
      grasp.pre_grasp_posture.position.resize(end_effector_joints.size());
      grasp.grasp_posture.position.resize(end_effector_joints.size());
      
      for(unsigned int i = 0; i < (*it)->pre_grasp_posture_.get().joint_angles_.size(); i++) {
        grasp.pre_grasp_posture.position[i] = (*it)->pre_grasp_posture_.get().joint_angles_.at(i);
      }
      for(unsigned int i = 0; i < (*it)->final_grasp_posture_.get().joint_angles_.size(); i++) {
        grasp.grasp_posture.position[i] = (*it)->final_grasp_posture_.get().joint_angles_.at(i);
      }
    }
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
    // Eigen::Affine3d orig;
    // planning_models::poseFromMsg(grasp.grasp_pose, orig);
    // Eigen::Affine3d rot(Eigen::Translation3d(.10,0.0,0.0)*Eigen::AngleAxisd(-90.f * (M_PI/180.f), Eigen::Vector3d::UnitZ()));
    // Eigen::Affine3d np = orig*rot;
    // planning_models::msgFromPose(np, grasp.grasp_pose);
    // geometry_msgs::Pose p;
    // p = grasp.grasp_pose;
    // p.orientation.x = grasp.grasp_pose.orientation.w;
    // p.orientation.y = grasp.grasp_pose.orientation.x;
    // p.orientation.z = grasp.grasp_pose.orientation.y;
    // p.orientation.w = grasp.grasp_pose.orientation.z;
    // grasp.grasp_pose = p;
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
  
  coll.meshes.push_back(loaded_meshes_[current_id_]);

  coll.mesh_poses.resize(1);
  coll.mesh_poses[0].position.x = (float)collision_object_pos_x_box_->value() / 100.0f;
  coll.mesh_poses[0].position.y = (float)collision_object_pos_y_box_->value() / 100.0f;
  coll.mesh_poses[0].position.z = (float)collision_object_pos_z_box_->value() / 100.0f;
  coll.mesh_poses[0].orientation.x = 0;
  coll.mesh_poses[0].orientation.y = 0;
  coll.mesh_poses[0].orientation.z = 0;
  coll.mesh_poses[0].orientation.w = 1;
  
  Q_EMIT addCollisionObjectRequested(coll, selected_color_);
}

void HouseholdObjectAdditionDialog::addHouseholdObjectToScene(std::string name,
                                                              int model_id,
                                                              geometry_msgs::Pose pose)
{
  if(!database_) {
    connectToDatabase();
    populateDatabaseInformation();
  }
  moveit_msgs::CollisionObject coll;
  coll.id = name;

  if(loaded_meshes_.find(model_id) == loaded_meshes_.end()) {
    if(!loadDatabaseMesh(model_id)) {
      return;
    }
  } 
  collision_object_name_to_model_id_map_[coll.id] = model_id;
  coll.meshes.push_back(loaded_meshes_.at(model_id));
  //coll.header.frame_id = 
  coll.mesh_poses.push_back(pose);
  QColor col(128, 128, 128, 255);
  Q_EMIT addCollisionObjectRequested(coll,col);
}

}
