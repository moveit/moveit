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

#ifndef _HOUSEHOLD_OBJECT_ADDITION_DIALOG_H_
#define _HOUSEHOLD_OBJECT_ADDITION_DIALOG_H_

#include <sstream>

#include <QDialog>
#include <QString>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QSpinBox>
#include <QLineEdit>

#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/ColorRGBA.h>

#include <moveit_household_objects_database/objects_database.h>
#include <moveit_manipulation_msgs/Grasp.h>

#include <planning_models/kinematic_model.h>

namespace moveit_manipulation_visualization
{

class HouseholdObjectAdditionDialog: public QDialog
{
  Q_OBJECT
  
  public:
  
  HouseholdObjectAdditionDialog(QWidget* parent,
                                const planning_models::KinematicModelConstPtr& kinematic_model);

  bool connectToDatabase(const std::string& database_host = "wgs36.willowgarage.com",
                         const int& database_port = 5432,
                         const std::string& database_user = "willow",
                         const std::string& database_pass = "willow",
                         const std::string& database_name = "household_objects");

  virtual void showEvent(QShowEvent* event);
                                                                                
public Q_SLOTS:

  void selectColorButtonPressed();
  void createObjectConfirmedPressed();
  void collisionNameEdited(const QString&);
  void tableSelectionChanged();

  void generateGraspList(const std::string& database_name,
                         const std::string&,
                         const std::string&);

  void getMeshGivenModelId(int model_id);

  void addHouseholdObjectToScene(std::string,
                                 int,
                                 geometry_msgs::Pose);

Q_SIGNALS:

  void addCollisionObjectRequested(const moveit_msgs::CollisionObject& obj,
                                   const QColor& color);
  
  void graspListGenerated(bool, 
                          std::vector<moveit_manipulation_msgs::Grasp>);

  void modelMeshFetched(shape_msgs::Mesh& mesh);

protected:

  void populateDatabaseInformation();
  bool loadDatabaseMesh(int model_id);
  bool loadDatabaseGrasps(const std::string& database_name,
                          const int model_id,
                          const std::string& arm_name,
                          std::vector<moveit_manipulation_msgs::Grasp>& grasps);

  planning_models::KinematicModelConstPtr kinematic_model_;
  moveit_household_objects_database::ObjectsDatabase *database_;
  QTableWidget* household_objects_table_;

  std::string grasp_database_host_;
  int grasp_database_port_;
  std::string grasp_database_user_;
  std::string grasp_database_pass_;
  std::string grasp_database_name_;  

  std::map<std::string, int> collision_object_name_to_model_id_map_;
  std::map<int, shape_msgs::Mesh> loaded_meshes_;

  QLineEdit* collision_object_name_;
  QSpinBox* collision_object_pos_x_box_;
  QSpinBox* collision_object_pos_y_box_;
  QSpinBox* collision_object_pos_z_box_;
  QPushButton* make_object_button_;

  QPushButton* color_button_;
  QColor selected_color_;
  int current_id_;
};

}

#endif
