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

#include <grasp_place_evaluation/grasp_evaluator_fast.h>
#include <grasp_place_evaluation/place_evaluator_fast.h>
#include <moveit_manipulation_visualization/grasp_generator_visualization.h>
#include <moveit_manipulation_visualization/grasp_evaluation_visualization.h>
#include <moveit_manipulation_visualization/place_generator_visualization.h>
#include <moveit_manipulation_visualization/place_evaluation_visualization.h>
#include <trajectory_execution_manager/trajectory_execution_manager.h>
#include <interactive_markers/interactive_marker_server.h>
#include <planning_models_loader/kinematic_model_loader.h>

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
                                     boost::shared_ptr<planning_models_loader::KinematicModelLoader>& kinematic_model_loader,
                                     boost::shared_ptr<trajectory_execution_manager::TrajectoryExecutionManager> trajectory_execution_manager,
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
  void selectedSupportChanged(const QString &text);
  void selectedPlaceChanged(const QString &text);

  void playInterpolatedTrajectory();
  void planForGraspExecution();

  void planGenerationFinished(const std::string&,
                              const trajectory_msgs::JointTrajectory&);
  void planGenerationFailed(moveit_msgs::MoveItErrorCodes& err);

  void playFullGraspExecution();

  void generatePlaceLocations();
  void generatedPlaceLocationsBrowserNumberChanged(int);
  void evaluateGeneratedPlaceLocations();
  void evaluatedPlaceLocationsBrowserNumberChanged(int);
  void playPlacingInterpolatedTrajectory();
  void planForPlaceExecution();
  void playFullGraspAndPlaceExecution();

  void gotGeneratedGraspList(bool,
                             std::vector<moveit_manipulation_msgs::Grasp>);                             
  
  void executeFullGraspAndPlace();

Q_SIGNALS:

  void newPlanningSceneUpdated(const planning_scene::PlanningSceneConstPtr&);

  void requestSetGoalState(const std::string&,
                           const planning_models::KinematicState*);

  void requestPlanGeneration(bool);

  void requestDiffScenePlanGeneration(const std::string&,
                                      const planning_scene::PlanningSceneConstPtr&,
                                      const planning_models::KinematicState*);
  void requestGraspListGeneration(const std::string&,
                                  const std::string&,
                                  const std::string&);

protected:

  bool doneWithExecution() {
    ROS_INFO_STREAM("Done");
    return true;
  } 
  void disableGeneration();
  void disableEvaluation();
  void disablePlaceGeneration();
  void disablePlaceEvaluation();

  void playFullGraspExecutionThread(bool show_at_end);
  void playFullGraspAndPlaceExecutionThread();

  void loadEndEffectorParameters();

  void generateTrajectoryFromJointState(const sensor_msgs::JointState& js,
                                        trajectory_msgs::JointTrajectory& traj);


  std::map<std::string, std::string> end_effector_database_id_map_;
  std::map<std::string, geometry_msgs::Vector3> end_effector_approach_direction_map_;

  planning_scene::PlanningSceneConstPtr planning_scene_;
  boost::shared_ptr<moveit_visualization_ros::JointTrajectoryVisualization> joint_trajectory_visualization_;
  boost::shared_ptr<trajectory_execution_manager::TrajectoryExecutionManager> trajectory_execution_manager_;

  std::string current_object_;
  std::string current_arm_;
  std::string generated_grasp_frame_;
  std::string current_support_;
  std::vector<moveit_manipulation_msgs::Grasp> current_generated_grasps_;
  trajectory_msgs::JointTrajectory last_planned_trajectory_;

  std::string current_place_;
  std::vector<geometry_msgs::PoseStamped> current_generated_place_locations_;
  trajectory_msgs::JointTrajectory last_place_planned_trajectory_;
  trajectory_msgs::JointTrajectory last_return_from_place_planned_trajectory_;

  QComboBox* object_name_combo_;
  QComboBox* support_name_combo_;

  QSpinBox* generated_grasps_browser_;

  QPushButton* evaluate_grasp_button_;
  QSpinBox* evaluated_grasp_browser_;
  QLabel* evaluation_result_indicator_;
  QPushButton* play_interpolated_trajectory_button_;
  QPushButton* plan_for_grasp_execution_button_;
  QLabel* plan_execution_indicator_;
  QPushButton* play_full_grasp_execution_button_;

  QComboBox* place_name_combo_;
  QPushButton* generate_place_locations_button_;
  QSpinBox* generated_place_locations_browser_;

  QPushButton* evaluate_place_locations_button_;
  QSpinBox* evaluated_place_locations_browser_;
  QLabel* evaluation_place_locations_result_indicator_;
  QPushButton* play_placing_interpolated_trajectory_button_;
  QPushButton* plan_for_place_execution_button_;
  QLabel* plan_place_execution_indicator_;
  QPushButton* play_grasp_and_place_execution_button_;
  QPushButton* execute_grasp_and_place_button_;

  grasp_place_evaluation::GraspExecutionInfoVector last_grasp_evaluation_info_;
  boost::shared_ptr<grasp_place_evaluation::GraspEvaluatorFast> grasp_evaluator_fast_;

  grasp_place_evaluation::PlaceExecutionInfoVector last_place_evaluation_info_;
  boost::shared_ptr<grasp_place_evaluation::PlaceEvaluatorFast> place_evaluator_fast_;

  boost::shared_ptr<GraspGeneratorVisualization> grasp_generator_visualization_;
  boost::shared_ptr<GraspEvaluationVisualization> grasp_evaluation_visualization_;

  boost::shared_ptr<PlaceGeneratorVisualization> place_generator_visualization_;
  boost::shared_ptr<PlaceEvaluationVisualization> place_evaluation_visualization_;

};

}
#endif
