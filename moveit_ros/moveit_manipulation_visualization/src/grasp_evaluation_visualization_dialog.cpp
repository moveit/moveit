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
                                   boost::shared_ptr<planning_models_loader::KinematicModelLoader>& kinematic_model_loader,
                                   boost::shared_ptr<trajectory_execution_manager::TrajectoryExecutionManager> trajectory_execution_manager,
                                   ros::Publisher& marker_publisher) :
  QDialog(parent),
  planning_scene_(planning_scene),
  joint_trajectory_visualization_(new moveit_visualization_ros::JointTrajectoryVisualization(planning_scene,
                                                                                             marker_publisher)),
  trajectory_execution_manager_(trajectory_execution_manager),
  grasp_generator_visualization_(new GraspGeneratorVisualization(marker_publisher)),
  place_generator_visualization_(new PlaceGeneratorVisualization(marker_publisher))
{
  std::map<std::string, kinematics::KinematicsBasePtr> solver_map = kinematic_model_loader->generateKinematicsSolversMap();

  grasp_evaluator_fast_.reset(new grasp_place_evaluation::GraspEvaluatorFast(planning_scene_->getKinematicModel(),
                                                                             solver_map));
  grasp_evaluation_visualization_.reset(new GraspEvaluationVisualization(marker_publisher));
  place_evaluator_fast_.reset(new grasp_place_evaluation::PlaceEvaluatorFast(planning_scene_->getKinematicModel(),
                                                                             solver_map));
  place_evaluation_visualization_.reset(new PlaceEvaluationVisualization(marker_publisher));

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
  QHBoxLayout* support_layout = new QHBoxLayout();
  QLabel* support_name_line = new QLabel(this);
  support_name_line->setText("Support surface: ");
  support_name_combo_ = new QComboBox(this);
  support_layout->addWidget(support_name_line);
  support_layout->addWidget(support_name_combo_);
  layout->addLayout(support_layout);

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

  evaluate_grasp_button_ = new QPushButton("Start Grasp Evaluation");
  evaluate_grasp_button_->setDisabled(true);
  layout->addWidget(evaluate_grasp_button_);
  QHBoxLayout* evaluator_layout = new QHBoxLayout();
  evaluated_grasp_browser_ = new QSpinBox(this);
  evaluated_grasp_browser_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  evaluation_result_indicator_ = new QLabel(this);
  evaluation_result_indicator_->setText("None");
  evaluation_result_indicator_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
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

  QFrame* line_3 = new QFrame();
  line_3->setFrameShape(QFrame::HLine);
  line_3->setFrameShadow(QFrame::Sunken);
  layout->addWidget(line_3);

  QHBoxLayout* place_surface_layout = new QHBoxLayout();
  QLabel* place_name_line = new QLabel(this);
  place_name_line->setText("Place surface: ");
  place_name_combo_ = new QComboBox(this);
  place_surface_layout->addWidget(place_name_line);
  place_surface_layout->addWidget(place_name_combo_);
  layout->addLayout(place_surface_layout);

  QHBoxLayout* place_generator_layout = new QHBoxLayout();
  generate_place_locations_button_ = new QPushButton("Generate Place Locations");
  generated_place_locations_browser_ = new QSpinBox(this);
  place_generator_layout->addWidget(generate_place_locations_button_);
  place_generator_layout->addWidget(generated_place_locations_browser_);
  layout->addLayout(place_generator_layout);

  QFrame* line_4 = new QFrame();
  line_4->setFrameShape(QFrame::HLine);
  line_4->setFrameShadow(QFrame::Sunken);
  layout->addWidget(line_4);

  evaluate_place_locations_button_ = new QPushButton("Start Place Location Evaluation");
  evaluate_place_locations_button_->setDisabled(true);
  layout->addWidget(evaluate_place_locations_button_);
  QHBoxLayout* place_evaluator_layout = new QHBoxLayout();
  evaluated_place_locations_browser_ = new QSpinBox(this);
  evaluated_place_locations_browser_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  evaluation_place_locations_result_indicator_ = new QLabel(this);
  evaluation_place_locations_result_indicator_->setText("None");
  evaluation_place_locations_result_indicator_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  place_evaluator_layout->addWidget(evaluated_place_locations_browser_);
  place_evaluator_layout->addWidget(evaluation_place_locations_result_indicator_);
  layout->addLayout(place_evaluator_layout);
  play_placing_interpolated_trajectory_button_ = new QPushButton("Play Placing Interpolated Trajectory");
  layout->addWidget(play_placing_interpolated_trajectory_button_);
  QHBoxLayout* place_location_execution_layout = new QHBoxLayout();
  plan_for_place_execution_button_ = new QPushButton("Plan for Place Location Execution");
  plan_place_execution_indicator_ = new QLabel(this);
  plan_place_execution_indicator_->setText("None");
  place_location_execution_layout->addWidget(plan_for_place_execution_button_);
  place_location_execution_layout->addWidget(plan_place_execution_indicator_);
  layout->addLayout(place_location_execution_layout);
  play_grasp_and_place_execution_button_ = new QPushButton("Play full grasp and place execution");
  layout->addWidget(play_grasp_and_place_execution_button_);

  // if(trajectory_execution_manager) {
  //   execute_grasp_and_place_button_ = new QPushButton("Execute full grasp and place");
  //   layout->addWidget(execute_grasp_and_place_button_);
  //   connect(execute_grasp_and_place_button_,
  //           SIGNAL(clicked()),
  //           this, 
  //           SLOT(executeFullGraspAndPlace()));
  // } else {
  execute_grasp_and_place_button_ = NULL;
  //}

  connect(get_grasps_button, SIGNAL(clicked()), this, SLOT(generateGraspsForObject()));
  connect(generated_grasps_browser_, SIGNAL(valueChanged(int)), this, SLOT(generatedGraspBrowserNumberChanged(int)));
  connect(this, SIGNAL(newPlanningSceneUpdated(const planning_scene::PlanningSceneConstPtr&)), 
          this, SLOT(populateObjectComboBox(const planning_scene::PlanningSceneConstPtr&)));
  connect(object_name_combo_, SIGNAL(currentIndexChanged(const QString&)),
          this, SLOT(selectedObjectChanged(const QString&)));
  connect(support_name_combo_, SIGNAL(currentIndexChanged(const QString&)),
          this, SLOT(selectedSupportChanged(const QString&)));
  connect(place_name_combo_, SIGNAL(currentIndexChanged(const QString&)),
          this, SLOT(selectedPlaceChanged(const QString&)));

  connect(evaluate_grasp_button_, SIGNAL(clicked()), this, SLOT(evaluateGeneratedGrasps()));
  connect(evaluated_grasp_browser_, SIGNAL(valueChanged(int)), this, SLOT(evaluatedGraspBrowserNumberChanged(int)));
  connect(play_interpolated_trajectory_button_, SIGNAL(clicked()), this, SLOT(playInterpolatedTrajectory()));
  connect(plan_for_grasp_execution_button_, SIGNAL(clicked()), this, SLOT(planForGraspExecution()));
  connect(play_full_grasp_execution_button_, SIGNAL(clicked()), this, SLOT(playFullGraspExecution()));

  connect(generate_place_locations_button_, SIGNAL(clicked()), this, SLOT(generatePlaceLocations()));
  connect(generated_place_locations_browser_, SIGNAL(valueChanged(int)), this, SLOT(generatedPlaceLocationsBrowserNumberChanged(int)));

  connect(evaluate_place_locations_button_, SIGNAL(clicked()), this, SLOT(evaluateGeneratedPlaceLocations()));
  connect(evaluated_place_locations_browser_, SIGNAL(valueChanged(int)), this, SLOT(evaluatedPlaceLocationsBrowserNumberChanged(int)));
  connect(play_placing_interpolated_trajectory_button_, SIGNAL(clicked()), this, SLOT(playPlacingInterpolatedTrajectory()));
  connect(plan_for_place_execution_button_, SIGNAL(clicked()), this, SLOT(planForPlaceExecution()));
  connect(play_grasp_and_place_execution_button_, SIGNAL(clicked()), this, SLOT(playFullGraspAndPlaceExecution()));

  populateObjectComboBox(planning_scene);
  loadEndEffectorParameters();
}

void GraspEvaluationVisualizationDialog::loadEndEffectorParameters() 
{
  ros::NodeHandle nh;
  const std::vector<srdf::Model::EndEffector>& end_effectors = planning_scene_->getSrdfModel()->getEndEffectors();
  for(unsigned int i = 0; i < end_effectors.size(); i++) {
    std::string database_id;
    if(!nh.getParam(end_effectors[i].name_+"/database_id", database_id)) {
      ROS_WARN_STREAM("End effector " << end_effectors[i].name_ << " has no database id");
    }
    end_effector_database_id_map_[end_effectors[i].name_] = database_id;
    XmlRpc::XmlRpcValue approach;
    if(!nh.getParam(end_effectors[i].name_+"/approach_direction", approach)) {
      ROS_WARN_STREAM("End effector " << end_effectors[i].name_ << " has no approach direction");
      continue;
    }
    if(approach.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN_STREAM("End effector " << end_effectors[i].name_ << " approach direction not a vector");
      continue;
    }
    std::vector<double> direction;
    if(approach.size() != 3) {
      ROS_WARN_STREAM("End effector " << end_effectors[i].name_ << " approach direction should have three values");
      continue;
    }
    for(int j = 0; j < approach.size(); j++) {
      direction.push_back(static_cast<double>(approach[j]));
    }
    geometry_msgs::Vector3 v;
    v.x = direction[0];
    v.y = direction[1];
    v.z = direction[2];
    ROS_DEBUG_STREAM("End effector " << end_effectors[i].name_ << " approach " << v.x << " " << v.y << " " << v.z);
    end_effector_approach_direction_map_[end_effectors[i].name_] = v;
  }
}

void GraspEvaluationVisualizationDialog::updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene) {
  planning_scene_ = planning_scene;
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
  const QString current_support = support_name_combo_->currentText();
  support_name_combo_->clear();
  const QString current_place = place_name_combo_->currentText();
  place_name_combo_->clear();
  std::vector<std::string> object_ids = planning_scene->getCollisionWorld()->getObjectIds();
  bool has_current_text = false;
  bool has_current_support = false;
  bool has_current_place = false;
  unsigned int ind = 0;
  unsigned int support_ind = 0;
  unsigned int place_ind = 0;
  support_name_combo_->addItem("");
  for(unsigned int i = 0; i < object_ids.size(); i++) {
    object_name_combo_->addItem(object_ids[i].c_str());
    support_name_combo_->addItem(object_ids[i].c_str());
    place_name_combo_->addItem(object_ids[i].c_str());
    if(object_ids[i] == current.toStdString()) {
      has_current_text = true;
      ind = i;
    }
    if(object_ids[i] == current_support.toStdString()) {
      has_current_support = true;
      support_ind = i;
    }
    if(object_ids[i] == current_place.toStdString()) {
      has_current_place = true;
      place_ind = i;
    }
  }
  if(has_current_text) {
    object_name_combo_->setCurrentIndex(ind);
  } else {
    disableGeneration();
  }
  if(has_current_support) {
    support_name_combo_->setCurrentIndex(support_ind+1);
  }
  if(has_current_place) {
    place_name_combo_->setCurrentIndex(place_ind);
  } else {
    disablePlaceEvaluation();
  }
}

void GraspEvaluationVisualizationDialog::generateGraspsForObject() {
  generated_grasps_browser_->setRange(0, 0);
  generated_grasps_browser_->setDisabled(true);
  current_object_ = object_name_combo_->currentText().toStdString();
  ROS_INFO_STREAM("Attempting to grasp " << current_object_);
  current_generated_grasps_.clear();

  std::string end_effector_name = planning_scene_->getKinematicModel()->getJointModelGroup(current_arm_)->getAttachedEndEffectorGroupName();

  Q_EMIT requestGraspListGeneration(end_effector_database_id_map_[end_effector_name],
                                    current_object_,
                                    current_arm_);
}

void GraspEvaluationVisualizationDialog::gotGeneratedGraspList(bool ok,
                                                               std::vector<moveit_manipulation_msgs::Grasp> grasps)
{
  if(!ok || grasps.size() == 0) {
    ROS_INFO_STREAM("Using dummy grasp generator");
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
  } else {
    ROS_INFO_STREAM("Using database grasps");
    generated_grasp_frame_ = current_object_;
    current_generated_grasps_ = grasps;
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
  if(generated_grasp_frame_ == current_object_) {
  }

  goal.lift.direction.vector.z = 1;
  goal.lift.desired_distance = .1;

  if(!current_support_.empty()) {
    //DON'T actually want this
    //goal.allow_gripper_support_collision = true;
    goal.collision_support_surface_name = current_support_;
  }
  unsigned int cur_size = last_grasp_evaluation_info_.size();

  if(cur_size == current_generated_grasps_.size()) {
    ROS_INFO_STREAM("Starting from scratch");
    last_grasp_evaluation_info_.clear();
  }

  std::string end_effector_name = planning_scene_->getKinematicModel()->getJointModelGroup(current_arm_)->getAttachedEndEffectorGroupName();

  grasp_evaluator_fast_->testGrasps(planning_scene_,
                                    &planning_scene_->getCurrentState(),
                                    goal,
                                    end_effector_approach_direction_map_[end_effector_name],
                                    current_generated_grasps_,
                                    last_grasp_evaluation_info_,
                                    true);

  if(last_grasp_evaluation_info_.size() > 0) {
    evaluated_grasp_browser_->setRange(0, last_grasp_evaluation_info_.size());
    evaluated_grasp_browser_->setEnabled(true);
    bool success = false;
    for(unsigned int i = cur_size; i < last_grasp_evaluation_info_.size(); i++) {
      grasp_place_evaluation::GraspExecutionInfo& ev = last_grasp_evaluation_info_[i];
      if(ev.result_.result_code == moveit_manipulation_msgs::GraspResult::SUCCESS) {
        evaluated_grasp_browser_->setValue(i+1);
        evaluation_result_indicator_->setEnabled(true);
        evaluatedGraspBrowserNumberChanged(i+1);
        success = true;
        if(last_grasp_evaluation_info_.size() < current_generated_grasps_.size()) {
          evaluate_grasp_button_->setText("Continue Grasp Evaluation"); 
        }
        break;
      }
    }
    if(!success) {
      evaluated_grasp_browser_->setValue(cur_size);
    }
  }
}

void GraspEvaluationVisualizationDialog::evaluatedGraspBrowserNumberChanged(int i) {
  play_full_grasp_execution_button_->setDisabled(true);
  disablePlaceGeneration();
  if(i == 0) {
    grasp_evaluation_visualization_->removeAllMarkers();
    evaluation_result_indicator_->setText("None");
    play_interpolated_trajectory_button_->setDisabled(true);
    plan_for_grasp_execution_button_->setDisabled(true);
    plan_execution_indicator_->setText("None");
    plan_execution_indicator_->setDisabled(true);
  } else {
    plan_execution_indicator_->setText("None");
    grasp_place_evaluation::GraspExecutionInfo& ev = last_grasp_evaluation_info_[i-1];
    evaluation_result_indicator_->setText(grasp_place_evaluation::convertGraspResultToStringStatus(ev.result_).c_str());
    grasp_evaluation_visualization_->showGraspPose(planning_scene_,
                                                   last_grasp_evaluation_info_,
                                                   i-1,
                                                   true,
                                                   true,
                                                   true);
    if(ev.result_.result_code == moveit_manipulation_msgs::GraspResult::SUCCESS) {
      play_interpolated_trajectory_button_->setEnabled(true);
      plan_for_grasp_execution_button_->setEnabled(true);
      plan_execution_indicator_->setEnabled(true);
    } else {
      play_interpolated_trajectory_button_->setDisabled(true);
      plan_for_grasp_execution_button_->setDisabled(true);
      plan_execution_indicator_->setDisabled(true);
    }
  }
}

void GraspEvaluationVisualizationDialog::selectedObjectChanged(const QString &text) {
  if(text.toStdString() == current_object_) {
    return;
  }
  current_object_ = text.toStdString();
  disableGeneration();
}

void GraspEvaluationVisualizationDialog::selectedSupportChanged(const QString &text) {
  if(text.toStdString() == current_support_) {
    return;
  }
  current_support_ = text.toStdString();
  disableEvaluation();
}

void GraspEvaluationVisualizationDialog::selectedPlaceChanged(const QString &text) {
  if(text.toStdString() == current_place_) {
    return;
  }
  current_place_ = text.toStdString();
  disablePlaceEvaluation();
}

void GraspEvaluationVisualizationDialog::playInterpolatedTrajectory() {
  grasp_evaluation_visualization_->playInterpolatedTrajectories(planning_scene_,
                                                                last_grasp_evaluation_info_,
                                                                joint_trajectory_visualization_,
                                                                evaluated_grasp_browser_->value()-1,
                                                                true, 
                                                                true,
                                                                true,
                                                                true);
}

void GraspEvaluationVisualizationDialog::planForGraspExecution() {
  grasp_place_evaluation::GraspExecutionInfo& ev = last_grasp_evaluation_info_[evaluated_grasp_browser_->value()-1];
  if(ev.approach_trajectory_.points.size() == 0) {
    ROS_WARN_STREAM("Asked to plan, but no approach trajectory");
    return;
  }
  planning_models::KinematicState goal_state(planning_scene_->getCurrentState());
  goal_state.setStateValues(ev.approach_trajectory_.joint_names,
                            ev.approach_trajectory_.points.front().positions);

  plan_execution_indicator_->setText("Planning");

  Q_EMIT requestDiffScenePlanGeneration(current_arm_,
                                        planning_scene_,
                                        &goal_state);
}

void GraspEvaluationVisualizationDialog::planGenerationFinished(const std::string& group_name,
                                                                const trajectory_msgs::JointTrajectory& traj) {
  if(plan_place_execution_indicator_->isEnabled()) {
    if(last_place_planned_trajectory_.points.size() == 0) {
      last_place_planned_trajectory_ = traj;
      ROS_INFO_STREAM("Setting place");
      grasp_place_evaluation::PlaceExecutionInfo& ev = last_place_evaluation_info_[evaluated_place_locations_browser_->value()-1];
      if(ev.retreat_trajectory_.points.size() == 0) {
        ROS_WARN_STREAM("Asked to plan, but no approach trajectory");
        return;
      }
      ev.detached_object_diff_scene_->getCurrentState().setStateValues(ev.retreat_trajectory_.joint_names,
                                                                       ev.retreat_trajectory_.points.back().positions);

      plan_execution_indicator_->setText("Planning back to start");
      
      Q_EMIT requestDiffScenePlanGeneration(current_arm_,
                                            ev.detached_object_diff_scene_,
                                            &planning_scene_->getCurrentState());
      
    } else {
      ROS_INFO_STREAM("Return from place traj has " << traj.points.size() << " points");
      last_return_from_place_planned_trajectory_ = traj;
      plan_place_execution_indicator_->setText("Success");
      play_grasp_and_place_execution_button_->setEnabled(true);
      if(execute_grasp_and_place_button_) {
        execute_grasp_and_place_button_->setEnabled(true);
      }
    }
  } else {
    ROS_INFO_STREAM("Setting grasp");
    plan_execution_indicator_->setText("Success");
    play_full_grasp_execution_button_->setEnabled(true);
    last_planned_trajectory_ = traj;
    place_name_combo_->setEnabled(true);
    generate_place_locations_button_->setEnabled(true);
  }
}

void GraspEvaluationVisualizationDialog::planGenerationFailed(moveit_msgs::MoveItErrorCodes& err) {
  if(plan_place_execution_indicator_->isEnabled()) {
    plan_place_execution_indicator_->setText("Failed");
  } else {
    plan_execution_indicator_->setText("Failed");
  }
}

void GraspEvaluationVisualizationDialog::playFullGraspExecution() {
  boost::thread(boost::bind(&GraspEvaluationVisualizationDialog::playFullGraspExecutionThread, this, true));
}

void GraspEvaluationVisualizationDialog::playFullGraspExecutionThread(bool show_at_end) {

  if(last_planned_trajectory_.points.size() == 0) {
    ROS_WARN_STREAM("Last planned trajectory is size 0");
    return;
  }

  std_msgs::ColorRGBA col;
  col.b = col.r = col.a = 1.0;
  planning_models::KinematicState state(planning_scene_->getCurrentState());
  state.setStateValues(current_generated_grasps_[evaluated_grasp_browser_->value()-1].pre_grasp_posture);
  joint_trajectory_visualization_->setTrajectory(state,
                                                 current_arm_,
                                                 last_planned_trajectory_,
                                                 col);
  grasp_evaluation_visualization_->hideAllMarkers();
  joint_trajectory_visualization_->playCurrentTrajectory(true);
  grasp_evaluation_visualization_->playInterpolatedTrajectories(planning_scene_,
                                                                last_grasp_evaluation_info_,
                                                                joint_trajectory_visualization_,
                                                                evaluated_grasp_browser_->value()-1,
                                                                true,
                                                                true, 
                                                                false,
                                                                show_at_end);
}

void GraspEvaluationVisualizationDialog::generatePlaceLocations() {
  generated_place_locations_browser_->setRange(0,0);
  generated_place_locations_browser_->setDisabled(true);
  current_generated_place_locations_.clear();

  grasp_place_evaluation::GraspExecutionInfo& ev = last_grasp_evaluation_info_[evaluated_grasp_browser_->value()-1];
  if(ev.result_.result_code != moveit_manipulation_msgs::GraspResult::SUCCESS) {
    ROS_WARN_STREAM("Can't call place for unsuccessful grasp");
    return;
  }
  std::string current_object = object_name_combo_->currentText().toStdString();
  std::string place_support_surface = place_name_combo_->currentText().toStdString();
  if(current_object == place_support_surface) {
    ROS_WARN_STREAM("Can't place object " << current_object << " on itself");
    return;
  }
  ROS_INFO_STREAM("Generating place locations");
  if(!place_generator_visualization_->generatePlaces(ev.attached_object_diff_scene_,
                                                     current_object,
                                                     place_support_surface,
                                                     current_generated_place_locations_)) {
    ROS_WARN_STREAM("Something wrong with place generator");
    return;
  }
  if(current_generated_place_locations_.size() == 0) {
    ROS_WARN_STREAM("No place locations generated");
    return;
  }
  generated_place_locations_browser_->setEnabled(true);
  generated_place_locations_browser_->setRange(0, current_generated_place_locations_.size());
  generated_place_locations_browser_->setValue(1);
  generatedPlaceLocationsBrowserNumberChanged(1);
  evaluate_place_locations_button_->setEnabled(true);
}

void GraspEvaluationVisualizationDialog::generatedPlaceLocationsBrowserNumberChanged(int i) {
  if(i == 0) {
    place_generator_visualization_->removeAllMarkers();
  } else {
    grasp_place_evaluation::GraspExecutionInfo& ev = last_grasp_evaluation_info_[evaluated_grasp_browser_->value()-1];
    place_generator_visualization_->showPlace(ev.attached_object_diff_scene_,
                                              current_arm_,
                                              current_generated_grasps_[evaluated_grasp_browser_->value()-1],
                                              current_generated_place_locations_[generated_place_locations_browser_->value()-1]);
  }
}

void GraspEvaluationVisualizationDialog::evaluateGeneratedPlaceLocations() {
  grasp_place_evaluation::GraspExecutionInfo& ev = last_grasp_evaluation_info_[evaluated_grasp_browser_->value()-1];
  
  moveit_manipulation_msgs::PlaceGoal goal;
  goal.arm_name = current_arm_;
  goal.place_locations = current_generated_place_locations_;
  goal.grasp = current_generated_grasps_[evaluated_grasp_browser_->value()-1];
  goal.min_retreat_distance = .1;
  goal.desired_retreat_distance = .1;
  goal.approach.direction.vector.z = 1.0;
  goal.approach.desired_distance = .1;
  goal.allow_gripper_support_collision = true;
  goal.collision_object_name = object_name_combo_->currentText().toStdString();
  goal.collision_support_surface_name = place_name_combo_->currentText().toStdString();
  
  unsigned int cur_size = last_place_evaluation_info_.size();
  
  if(cur_size == current_generated_place_locations_.size()) {
    ROS_INFO_STREAM("Starting from scratch");
    last_place_evaluation_info_.clear();
  }
  ev.attached_object_diff_scene_->getCurrentState().setStateValues(ev.lift_trajectory_.joint_names,
                                                                   ev.lift_trajectory_.points.back().positions);
  std::string end_effector_name = planning_scene_->getKinematicModel()->getJointModelGroup(current_arm_)->getAttachedEndEffectorGroupName();
  place_evaluator_fast_->testPlaceLocations(ev.attached_object_diff_scene_,
                                            &ev.attached_object_diff_scene_->getCurrentState(),
                                            goal,
                                            end_effector_approach_direction_map_[end_effector_name],                                            
                                            current_generated_place_locations_,
                                            last_place_evaluation_info_,
                                            true);
  if(last_place_evaluation_info_.size() > 0) {
    evaluated_place_locations_browser_->setRange(0, last_place_evaluation_info_.size());
    evaluated_place_locations_browser_->setEnabled(true);
    bool success = false;
    for(unsigned int i = cur_size; i < last_place_evaluation_info_.size(); i++) {
      grasp_place_evaluation::PlaceExecutionInfo& ev = last_place_evaluation_info_[i];
      if(ev.result_.result_code == moveit_manipulation_msgs::PlaceLocationResult::SUCCESS) {
        evaluated_place_locations_browser_->setValue(i+1);
        evaluation_place_locations_result_indicator_->setEnabled(true);
        evaluatedPlaceLocationsBrowserNumberChanged(i+1);
        success = true;
        if(last_place_evaluation_info_.size() < current_generated_place_locations_.size()) {
          evaluate_place_locations_button_->setText("Continue Place Evaluation");
        }
        break;
      }
    }
    if(!success) {
      evaluated_place_locations_browser_->setValue(cur_size);
    }
  }
}

void GraspEvaluationVisualizationDialog::evaluatedPlaceLocationsBrowserNumberChanged(int i) {
  place_generator_visualization_->removeAllMarkers();
  play_grasp_and_place_execution_button_->setDisabled(true);
  if(i == 0) {
    place_evaluation_visualization_->removeAllMarkers();
    evaluation_place_locations_result_indicator_->setText("None");
    play_placing_interpolated_trajectory_button_->setDisabled(true);
    plan_for_place_execution_button_->setDisabled(true);
    plan_place_execution_indicator_->setText("None");
    plan_place_execution_indicator_->setDisabled(true);
  } else {
    plan_place_execution_indicator_->setText("None");
    grasp_place_evaluation::GraspExecutionInfo& gev = last_grasp_evaluation_info_[evaluated_grasp_browser_->value()-1];
    grasp_place_evaluation::PlaceExecutionInfo& pev = last_place_evaluation_info_[i-1];
    evaluation_place_locations_result_indicator_->setText(grasp_place_evaluation::convertPlaceResultToStringStatus(pev.result_).c_str());
    place_evaluation_visualization_->showPlacePose(gev.attached_object_diff_scene_,
                                                   last_place_evaluation_info_,
                                                   i-1,
                                                   true,
                                                   true,
                                                   true);
    if(pev.result_.result_code == moveit_manipulation_msgs::PlaceLocationResult::SUCCESS) {
      play_placing_interpolated_trajectory_button_->setEnabled(true);
      plan_for_place_execution_button_->setEnabled(true);
      plan_place_execution_indicator_->setEnabled(true);
    } else {
      play_placing_interpolated_trajectory_button_->setDisabled(true);
      plan_for_place_execution_button_->setDisabled(true);
      plan_place_execution_indicator_->setDisabled(true);
    }
  }
}

void GraspEvaluationVisualizationDialog::playPlacingInterpolatedTrajectory() {
  grasp_place_evaluation::GraspExecutionInfo& gev = last_grasp_evaluation_info_[evaluated_grasp_browser_->value()-1];
  place_evaluation_visualization_->playInterpolatedTrajectories(gev.attached_object_diff_scene_,
                                                                last_place_evaluation_info_,
                                                                joint_trajectory_visualization_,
                                                                evaluated_place_locations_browser_->value()-1,
                                                                true,
                                                                true,
                                                                true, 
                                                                true);
}

void GraspEvaluationVisualizationDialog::planForPlaceExecution() {
  grasp_place_evaluation::GraspExecutionInfo& ev_grasp = last_grasp_evaluation_info_[evaluated_grasp_browser_->value()-1];
  if(ev_grasp.lift_trajectory_.points.size() == 0) {
    ROS_WARN_STREAM("Asked to plan, but no lift trajectory for grasp");
    return;
  }

  grasp_place_evaluation::PlaceExecutionInfo& ev_place = last_place_evaluation_info_[evaluated_place_locations_browser_->value()-1];
  if(ev_place.approach_trajectory_.points.size() == 0) {
    ROS_WARN_STREAM("Asked to plan, but no approach trajectory");
    return;
  }
  ev_grasp.attached_object_diff_scene_->getCurrentState().setStateValues(ev_grasp.lift_trajectory_.joint_names,
                                                                         ev_grasp.lift_trajectory_.points.back().positions);

  planning_models::KinematicState goal_state(ev_grasp.attached_object_diff_scene_->getCurrentState());
  goal_state.setStateValues(ev_place.approach_trajectory_.joint_names,
                            ev_place.approach_trajectory_.points.front().positions);

  last_place_planned_trajectory_.points.clear();

  plan_place_execution_indicator_->setText("Planning to place");

  Q_EMIT requestDiffScenePlanGeneration(current_arm_,
                                        ev_grasp.attached_object_diff_scene_,
                                        &goal_state);

}

void GraspEvaluationVisualizationDialog::playFullGraspAndPlaceExecution() {
  boost::thread(boost::bind(&GraspEvaluationVisualizationDialog::playFullGraspAndPlaceExecutionThread, this));
}

void GraspEvaluationVisualizationDialog::playFullGraspAndPlaceExecutionThread() {
  //this plays until lift
  place_evaluation_visualization_->hideAllMarkers();
  playFullGraspExecutionThread(false);

  ROS_INFO_STREAM("Done with grasp execution thread");

  grasp_place_evaluation::GraspExecutionInfo& ev_grasp = last_grasp_evaluation_info_[evaluated_grasp_browser_->value()-1];

  grasp_place_evaluation::PlaceExecutionInfo& ev_place = last_place_evaluation_info_[evaluated_place_locations_browser_->value()-1];
  
  std_msgs::ColorRGBA col;
  col.b = col.r = col.a = 1.0;
  joint_trajectory_visualization_->setTrajectory(ev_grasp.attached_object_diff_scene_->getCurrentState(),
                                                 current_arm_,
                                                 last_place_planned_trajectory_,
                                                 col);
  joint_trajectory_visualization_->playCurrentTrajectory(true);
  place_evaluation_visualization_->playInterpolatedTrajectories(ev_grasp.attached_object_diff_scene_,
                                                                last_place_evaluation_info_,
                                                                joint_trajectory_visualization_,
                                                                evaluated_place_locations_browser_->value()-1,
                                                                true,
                                                                true, 
                                                                false,
                                                                false);
  joint_trajectory_visualization_->setTrajectory(ev_place.detached_object_diff_scene_->getCurrentState(),
                                                 current_arm_,
                                                 last_return_from_place_planned_trajectory_,
                                                 col);
  joint_trajectory_visualization_->playCurrentTrajectory(true);
  grasp_evaluation_visualization_->showHiddenMarkers();
  place_evaluation_visualization_->showHiddenMarkers();
}

void GraspEvaluationVisualizationDialog::generateTrajectoryFromJointState(const sensor_msgs::JointState& js,
                                                                          trajectory_msgs::JointTrajectory& traj) 
{
  traj.joint_names = js.name;
  traj.points.resize(1);
  traj.points[0].positions = js.position;
  traj.points[0].velocities.resize(js.name.size(), 0.0);
  traj.points[0].time_from_start = ros::Duration(2.5);
}

void GraspEvaluationVisualizationDialog::executeFullGraspAndPlace() {
  // std::vector<trajectory_execution::TrajectoryExecutionRequest> ter_reqs;

  // std::string end_effector_name = planning_scene_->getKinematicModel()->getJointModelGroup(current_arm_)->getAttachedEndEffectorGroupName();  
  // grasp_place_evaluation::GraspExecutionInfo& ev_grasp = last_grasp_evaluation_info_[evaluated_grasp_browser_->value()-1];

  // grasp_place_evaluation::PlaceExecutionInfo& ev_place = last_place_evaluation_info_[evaluated_place_locations_browser_->value()-1];

  // trajectory_execution::TrajectoryExecutionRequest ter;

  // //first we need to open the gripper
  // ter.group_name_ = end_effector_name;
  // generateTrajectoryFromJointState(current_generated_grasps_[evaluated_grasp_browser_->value()-1].pre_grasp_posture,
  //                                  ter.trajectory_);
  // ROS_INFO_STREAM("Open joint is " << ter.trajectory_.points[0].positions[0]);
  // ter.failure_ok_ = true;
  // ter.failure_time_factor_ = 2.0;
  // ter_reqs.push_back(ter);

  // //next we go to pregrasp
  // ter.group_name_ = current_arm_;
  // ter.trajectory_ = last_planned_trajectory_;
  // ter.failure_ok_ = false;
  // ter.failure_time_factor_ = 100.0;
  // ter_reqs.push_back(ter);
  
  // //approach interpolation
  // ter.group_name_ = current_arm_;
  // ter.trajectory_ = ev_grasp.approach_trajectory_;
  // ter.failure_ok_ = false;
  // ter.failure_time_factor_ = 100.0;
  // ter_reqs.push_back(ter);

  // //close gripper
  // ter.group_name_ = end_effector_name;
  // generateTrajectoryFromJointState(current_generated_grasps_[evaluated_grasp_browser_->value()-1].grasp_posture,
  //                                  ter.trajectory_);
  // ROS_INFO_STREAM("Close joint is " << ter.trajectory_.points[0].positions[0]);
  // ter.failure_ok_ = true;
  // ter.failure_time_factor_ = 2.0;
  // ter_reqs.push_back(ter);

  // //lift interpolation
  // ter.group_name_ = current_arm_;
  // ter.trajectory_ = ev_grasp.lift_trajectory_;
  // ter.failure_ok_ = false;
  // ter.failure_time_factor_ = 100.0;
  // ter_reqs.push_back(ter);

  // //plan to pre-place
  // ter.group_name_ = current_arm_;
  // ter.trajectory_ = last_place_planned_trajectory_;
  // ter.failure_ok_ = false;
  // ter.failure_time_factor_ = 100.0;
  // ter_reqs.push_back(ter);

  // //place interpolation
  // ter.group_name_ = current_arm_;
  // ter.trajectory_ = ev_place.approach_trajectory_;
  // ter.failure_ok_ = false;
  // ter.failure_time_factor_ = 100.0;
  // ter_reqs.push_back(ter);

  // //open gripper
  // ter.group_name_ = end_effector_name;
  // generateTrajectoryFromJointState(current_generated_grasps_[evaluated_grasp_browser_->value()-1].pre_grasp_posture,
  //                                  ter.trajectory_);
  // ter.failure_time_factor_ = 2.0;
  // ter.failure_ok_ = true;
  // ter_reqs.push_back(ter);

  // //retreat interpolation
  // ter.group_name_ = current_arm_;
  // ter.trajectory_ = ev_place.retreat_trajectory_;
  // ter.failure_ok_ = false;
  // ter.failure_time_factor_ = 100.0;
  // ter_reqs.push_back(ter);
  
  // //plan to return
  // ter.group_name_ = current_arm_;
  // ter.trajectory_ = last_return_from_place_planned_trajectory_;
  // ter.failure_ok_ = false;
  // ter.failure_time_factor_ = 100.0;
  // ter_reqs.push_back(ter);

  // trajectory_execution_manager_->executeTrajectories(ter_reqs,
  //                                                    boost::bind(&GraspEvaluationVisualizationDialog::doneWithExecution, this));
}

void GraspEvaluationVisualizationDialog::disableGeneration() {
  generated_grasps_browser_->setDisabled(true);
  generated_grasps_browser_->setRange(0, 0);
  grasp_generator_visualization_->removeAllMarkers();
  disableEvaluation();
}

void GraspEvaluationVisualizationDialog::disableEvaluation() {
  evaluate_grasp_button_->setText("Start Grasp Evaluation"); 
  evaluate_grasp_button_->setDisabled(true);
  last_grasp_evaluation_info_.clear();
  evaluated_grasp_browser_->setDisabled(true);
  evaluated_grasp_browser_->setRange(0,0);
  evaluation_result_indicator_->setText("None");
  evaluation_result_indicator_->setDisabled(true);
  play_interpolated_trajectory_button_->setDisabled(true);
  plan_for_grasp_execution_button_->setDisabled(true);
  plan_execution_indicator_->setText("None");
  plan_execution_indicator_->setDisabled(true);
  play_full_grasp_execution_button_->setDisabled(true);
  grasp_evaluation_visualization_->removeAllMarkers();
  disablePlaceGeneration();
}

void GraspEvaluationVisualizationDialog::disablePlaceGeneration() {
  place_name_combo_->setDisabled(true);
  generate_place_locations_button_->setDisabled(true);
  generated_place_locations_browser_->setDisabled(true);
  generated_place_locations_browser_->setRange(0,0);
  place_generator_visualization_->removeAllMarkers();
  disablePlaceEvaluation();
}

void GraspEvaluationVisualizationDialog::disablePlaceEvaluation() {
  evaluate_place_locations_button_->setText("Start Place Location Evaluation"); 
  evaluate_place_locations_button_->setDisabled(true);
  last_place_evaluation_info_.clear();
  evaluated_place_locations_browser_->setDisabled(true);
  evaluated_place_locations_browser_->setRange(0,0);
  evaluation_place_locations_result_indicator_->setText("None");
  evaluation_place_locations_result_indicator_->setDisabled(true);
  play_placing_interpolated_trajectory_button_->setDisabled(true);
  plan_for_place_execution_button_->setDisabled(true);
  plan_place_execution_indicator_->setText("None");
  plan_place_execution_indicator_->setDisabled(true);
  play_grasp_and_place_execution_button_->setDisabled(true);
  if(execute_grasp_and_place_button_) {
    execute_grasp_and_place_button_->setDisabled(true);
  }
  place_evaluation_visualization_->removeAllMarkers();
}

} //namespace
