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

#include <moveit_manipulation_visualization/grasp_evaluation_visualization_dialog.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFrame>
#include <QPushButton>

namespace moveit_manipulation_visualization {

GraspEvaluationVisualizationDialog::
GraspEvaluationVisualizationDialog(QWidget* parent, 
                                   const planning_scene::PlanningSceneConstPtr& planning_scene,
                                   boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                                   boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader>& kinematics_plugin_loader,
                                   ros::Publisher& marker_publisher) :
  QDialog(parent),
  planning_scene_(planning_scene),
  grasp_generator_visualization_(new GraspGeneratorVisualization(marker_publisher)),
  grasp_evaluation_visualization_(new GraspEvaluationVisualization(planning_scene, 
                                                                   interactive_marker_server, 
                                                                   kinematics_plugin_loader, 
                                                                   marker_publisher))
{
  qRegisterMetaType<planning_scene::PlanningSceneConstPtr>("planning_scene::PlanningSceneConstPtr");

  QVBoxLayout* layout = new QVBoxLayout(this);
  setLayout(layout);

  QHBoxLayout* object_layout = new QHBoxLayout();
  QLabel* object_name_line = new QLabel(this);
  object_name_line->setText("Object: ");
  object_name_combo_ = new QComboBox(this);
  object_layout->addWidget(object_name_line);
  object_layout->addWidget(object_name_combo_);
  layout->addLayout(object_layout);

  QFrame* line_1 = new QFrame();
  line_1->setFrameShape(QFrame::HLine);
  line_1->setFrameShadow(QFrame::Sunken);
  layout->addWidget(line_1);

  QHBoxLayout* generator_layout = new QHBoxLayout();
  QPushButton* get_grasps_button = new QPushButton("Generate Grasps");
  generated_grasps_browser_ = new QSpinBox(this);
  generated_grasps_browser_->setDisabled(true);
  generator_layout->addWidget(get_grasps_button);
  generator_layout->addWidget(generated_grasps_browser_);
  layout->addLayout(generator_layout);
  
  QFrame* line_2 = new QFrame();
  line_2->setFrameShape(QFrame::HLine);
  line_2->setFrameShadow(QFrame::Sunken);
  layout->addWidget(line_2);

  evaluate_grasp_button_ = new QPushButton("Evaluate Until Success");
  evaluate_grasp_button_->setDisabled(true);
  layout->addWidget(evaluate_grasp_button_);
  QHBoxLayout* evaluator_layout = new QHBoxLayout();
  evaluated_grasp_browser_ = new QSpinBox(this);
  evaluation_result_indicator_ = new QLabel(this);
  evaluation_result_indicator_->setText("None");
  evaluator_layout->addWidget(evaluated_grasp_browser_);
  evaluator_layout->addWidget(evaluation_result_indicator_);
  layout->addLayout(evaluator_layout);
  play_interpolated_trajectory_button_ = new QPushButton("Play Interpolated Trajectory");
  layout->addWidget(play_interpolated_trajectory_button_);
  QHBoxLayout* grasp_execution_layout = new QHBoxLayout();
  plan_for_grasp_execution_button_ = new QPushButton("Plan for Grasp Execution");
  plan_execution_indicator_ = new QLabel(this);
  plan_execution_indicator_->setText("None");
  grasp_execution_layout->addWidget(plan_for_grasp_execution_button_);
  grasp_execution_layout->addWidget(plan_execution_indicator_);
  layout->addLayout(grasp_execution_layout);
  play_full_grasp_execution_button_ = new QPushButton("Play full grasp execution");
  layout->addWidget(play_full_grasp_execution_button_);

  connect(get_grasps_button, SIGNAL(clicked()), this, SLOT(generateGraspsForObject()));
  connect(generated_grasps_browser_, SIGNAL(valueChanged(int)), this, SLOT(generatedGraspBrowserNumberChanged(int)));
  connect(this, SIGNAL(newPlanningSceneUpdated(const planning_scene::PlanningSceneConstPtr&)), 
          this, SLOT(populateObjectComboBox(const planning_scene::PlanningSceneConstPtr&)));
  connect(object_name_combo_, SIGNAL(currentIndexChanged(const QString&)),
          this, SLOT(selectedObjectChanged(const QString&)));
  connect(evaluate_grasp_button_, SIGNAL(clicked()), this, SLOT(evaluateGeneratedGrasps()));
  connect(evaluated_grasp_browser_, SIGNAL(valueChanged(int)), this, SLOT(evaluatedGraspBrowserNumberChanged(int)));
  connect(play_interpolated_trajectory_button_, SIGNAL(clicked()), this, SLOT(playInterpolatedTrajectory()));
  connect(plan_for_grasp_execution_button_, SIGNAL(clicked()), this, SLOT(planForGraspExecution()));
  connect(play_full_grasp_execution_button_, SIGNAL(clicked()), this, SLOT(playFullGraspExecution()));

  populateObjectComboBox(planning_scene);
}

void GraspEvaluationVisualizationDialog::updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene) {
  planning_scene_ = planning_scene;
  grasp_evaluation_visualization_->updatePlanningScene(planning_scene);
  //signalling so that result is processed in GUI thread
  newPlanningSceneUpdated(planning_scene);
}

void GraspEvaluationVisualizationDialog::planningGroupChanged(const QString& new_group) {
  current_arm_ = new_group.toStdString();
  ROS_INFO_STREAM("Have arm named " << current_arm_);
}

void GraspEvaluationVisualizationDialog::populateObjectComboBox(const planning_scene::PlanningSceneConstPtr& planning_scene) 
{
  const QString current = object_name_combo_->currentText();
  object_name_combo_->clear();
  std::vector<std::string> object_ids = planning_scene->getCollisionWorld()->getObjectIds();
  bool has_current_text = false;
  unsigned int ind = 0;
  for(unsigned int i = 0; i < object_ids.size(); i++) {
    object_name_combo_->addItem(object_ids[i].c_str());
    if(object_ids[i] == current.toStdString()) {
      has_current_text = true;
      ind = i;
    }
  }
  if(has_current_text) {
    object_name_combo_->setCurrentIndex(ind);
  } else {
    disableGeneration();
    disableEvaluation();
  }
}

void GraspEvaluationVisualizationDialog::generateGraspsForObject() {
  generated_grasps_browser_->setRange(0, 0);
  generated_grasps_browser_->setDisabled(true);
  current_object_ = object_name_combo_->currentText().toStdString();
  ROS_INFO_STREAM("Attempting to grasp " << current_object_);
  current_generated_grasps_.clear();
  if(!grasp_generator_visualization_->generateGrasps(planning_scene_,
                                                     current_arm_,
                                                     current_object_,
                                                     current_generated_grasps_,
                                                     generated_grasp_frame_)) {
    ROS_WARN_STREAM("Something wrong with grasp generator");
    return;
  }
  if(current_generated_grasps_.size() == 0) {
    ROS_WARN_STREAM("No grasps generated");
    return;
  }
  generated_grasps_browser_->setEnabled(true);
  generated_grasps_browser_->setRange(0, current_generated_grasps_.size());
  generated_grasps_browser_->setValue(1);
  generatedGraspBrowserNumberChanged(1);
  evaluate_grasp_button_->setEnabled(true);
}

void GraspEvaluationVisualizationDialog::generatedGraspBrowserNumberChanged(int i) {
  if(i == 0) {
    grasp_generator_visualization_->removeAllMarkers();
  } else {
    grasp_generator_visualization_->showGrasp(planning_scene_,
                                              current_arm_,
                                              current_object_,
                                              generated_grasp_frame_,
                                              current_generated_grasps_[i-1]);
  }
}

void GraspEvaluationVisualizationDialog::evaluateGeneratedGrasps() {
  moveit_manipulation_msgs::PickupGoal goal;
  goal.arm_name = current_arm_;
  goal.collision_object_name = current_object_;
  goal.target.collision_name = current_object_;
  goal.target.reference_frame_id = generated_grasp_frame_;
  
  goal.lift.direction.vector.z = 1;
  goal.lift.desired_distance = .1;

  grasp_evaluation_visualization_->evaluateGrasps(current_arm_,
                                                  goal,
                                                  &planning_scene_->getCurrentState(),
                                                  current_generated_grasps_);
  if(grasp_evaluation_visualization_->getEvaluationInfoSize() > 0) {
    evaluated_grasp_browser_->setRange(0, grasp_evaluation_visualization_->getEvaluationInfoSize());
    bool success = false;
    for(unsigned int i = 0; i < grasp_evaluation_visualization_->getEvaluationInfoSize(); i++) {
      grasp_place_evaluation::GraspExecutionInfo ev;
      grasp_evaluation_visualization_->getEvaluatedGrasp(i, ev);
      if(ev.result_.result_code == moveit_manipulation_msgs::GraspResult::SUCCESS) {
        evaluated_grasp_browser_->setEnabled(true);
        evaluated_grasp_browser_->setValue(i+1);
        evaluation_result_indicator_->setEnabled(true);
        evaluatedGraspBrowserNumberChanged(1);
        success = true;
        break;
      }
    }
    if(!success) {
      evaluated_grasp_browser_->setValue(0);
    }
  }
}

void GraspEvaluationVisualizationDialog::evaluatedGraspBrowserNumberChanged(int i) {
  if(i == 0) {
    grasp_evaluation_visualization_->removeAllMarkers();
    evaluation_result_indicator_->setText("None");
    play_interpolated_trajectory_button_->setDisabled(true);
    plan_for_grasp_execution_button_->setDisabled(true);
    plan_execution_indicator_->setText("None");
  } else {
    grasp_place_evaluation::GraspExecutionInfo ev;
    grasp_evaluation_visualization_->getEvaluatedGrasp(i-1, ev);
    evaluation_result_indicator_->setText(grasp_place_evaluation::convertToStringStatus(ev.result_).c_str());
    grasp_evaluation_visualization_->showGraspPose(i-1,
                                                  true,
                                                  true,
                                                  true);
    if(ev.result_.result_code == moveit_manipulation_msgs::GraspResult::SUCCESS) {
      play_interpolated_trajectory_button_->setEnabled(true);
      plan_for_grasp_execution_button_->setEnabled(true);
    }
  }
}

void GraspEvaluationVisualizationDialog::selectedObjectChanged(const QString &text) {
  if(text.toStdString() == current_object_) {
    return;
  }
  disableGeneration();
  disableEvaluation();
}

void GraspEvaluationVisualizationDialog::disableGeneration() {
  generated_grasps_browser_->setDisabled(true);
  generated_grasps_browser_->setRange(0, 0);
  grasp_generator_visualization_->removeAllMarkers();
}

void GraspEvaluationVisualizationDialog::disableEvaluation() {
  evaluate_grasp_button_->setDisabled(true);
  evaluated_grasp_browser_->setDisabled(true);
  evaluated_grasp_browser_->setRange(0,0);
  evaluation_result_indicator_->setText("None");
  evaluation_result_indicator_->setDisabled(true);
  play_interpolated_trajectory_button_->setDisabled(true);
  plan_for_grasp_execution_button_->setDisabled(true);
  plan_execution_indicator_->setText("None");
  plan_execution_indicator_->setDisabled(true);
  play_full_grasp_execution_button_->setDisabled(true);
}

void GraspEvaluationVisualizationDialog::playInterpolatedTrajectory() {
  grasp_evaluation_visualization_->playInterpolatedTrajectories(evaluated_grasp_browser_->value()-1,
                                                                true, 
                                                                true);
}

void GraspEvaluationVisualizationDialog::planForGraspExecution() {
  grasp_place_evaluation::GraspExecutionInfo ev;
  grasp_evaluation_visualization_->getEvaluatedGrasp(evaluated_grasp_browser_->value()-1, ev);
  if(ev.approach_trajectory_.points.size() == 0) {
    ROS_WARN_STREAM("Asked to plan, but no approach trajectory");
    return;
  }
  planning_models::KinematicState state(planning_scene_->getCurrentState());
  state.setStateValues(ev.approach_trajectory_.joint_names,
                       ev.approach_trajectory_.points.front().positions);

  plan_execution_indicator_->setText("Planning");

  //emit
  requestSetGoalState(current_arm_,
                      &state);
  
  //emit
  requestPlanGeneration(false);
}

void GraspEvaluationVisualizationDialog::planGenerationFinished(const std::string& group_name,
                                                                const trajectory_msgs::JointTrajectory& traj) {
  plan_execution_indicator_->setText("Success");
  play_full_grasp_execution_button_->setEnabled(true);
  last_planned_trajectory_ = traj;
}

void GraspEvaluationVisualizationDialog::planGenerationFailed(moveit_msgs::MoveItErrorCodes& err) {
  plan_execution_indicator_->setText("Failed");
}

void GraspEvaluationVisualizationDialog::playFullGraspExecution() {
  boost::thread(boost::bind(&GraspEvaluationVisualizationDialog::playFullGraspExecutionThread, this));
}

void GraspEvaluationVisualizationDialog::playFullGraspExecutionThread() {
  std_msgs::ColorRGBA col;
  col.b = col.r = col.a = 1.0;
  grasp_evaluation_visualization_->getJointTrajectoryVisualization()->setTrajectory(planning_scene_->getCurrentState(),
                                                                                    current_arm_,
                                                                                    last_planned_trajectory_,
                                                                                    col);
  grasp_evaluation_visualization_->getJointTrajectoryVisualization()->playCurrentTrajectory(true);
  grasp_evaluation_visualization_->playInterpolatedTrajectories(evaluated_grasp_browser_->value()-1,
                                                                true,
                                                                true);
}

} //namespace
