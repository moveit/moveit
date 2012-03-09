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

#ifndef _GRASP_EVALUATION_VISUALIZATION_DIALOG_H_
#define _GRASP_EVALUATION_VISUALIZATION_DIALOG_H_

#include <moveit_manipulation_visualization/grasp_evaluation_visualization.h>
#include <moveit_manipulation_visualization/grasp_generator_visualization.h>

#include <QDialog>
#include <QComboBox>
#include <QSpinBox>
#include <QLabel>

namespace moveit_manipulation_visualization {

class GraspEvaluationVisualizationDialog: public QDialog {

  Q_OBJECT
  
  public:

  GraspEvaluationVisualizationDialog(QWidget* parent, 
                                     const planning_scene::PlanningSceneConstPtr& planning_scene,
                                     boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                                     boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader>& kinematics_plugin_loader,
                                     ros::Publisher& marker_publisher);
  
  virtual ~GraspEvaluationVisualizationDialog() {};
                                            
  void updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene);

public Q_SLOTS:
  
  void populateObjectComboBox(const planning_scene::PlanningSceneConstPtr& planning_scene);

  void generatedGraspBrowserNumberChanged(int i);
  void generateGraspsForObject();

  void evaluateGeneratedGrasps();
  void evaluatedGraspBrowserNumberChanged(int i);

  void planningGroupChanged(const QString&);
  void selectedObjectChanged(const QString &text);

  void playInterpolatedTrajectory();

  void planForGraspExecution();

  void planGenerationFinished(const std::string&,
                              const trajectory_msgs::JointTrajectory&);

  void planGenerationFailed(moveit_msgs::MoveItErrorCodes& err);

  void playFullGraspExecution();

Q_SIGNALS:

  void newPlanningSceneUpdated(const planning_scene::PlanningSceneConstPtr&);

  void requestSetGoalState(const std::string&,
                           const planning_models::KinematicState*);

  void requestPlanGeneration(bool);

protected:

  void disableGeneration();
  void disableEvaluation();

  void playFullGraspExecutionThread();

  planning_scene::PlanningSceneConstPtr planning_scene_;

  std::string current_object_;
  std::string current_arm_;
  std::string generated_grasp_frame_;
  std::vector<moveit_manipulation_msgs::Grasp> current_generated_grasps_;
  trajectory_msgs::JointTrajectory last_planned_trajectory_;
  

  QComboBox* object_name_combo_;
  QSpinBox* generated_grasps_browser_;
  QPushButton* evaluate_grasp_button_;
  QSpinBox* evaluated_grasp_browser_;
  QLabel* evaluation_result_indicator_;
  QPushButton* play_interpolated_trajectory_button_;
  QPushButton* plan_for_grasp_execution_button_;
  QLabel* plan_execution_indicator_;
  QPushButton* play_full_grasp_execution_button_;

  boost::shared_ptr<GraspGeneratorVisualization> grasp_generator_visualization_;
  boost::shared_ptr<GraspEvaluationVisualization> grasp_evaluation_visualization_;

};

}
#endif
