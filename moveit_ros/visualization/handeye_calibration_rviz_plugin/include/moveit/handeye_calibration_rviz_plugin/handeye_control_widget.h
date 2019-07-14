/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019,  Intel Corporation.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THEsensorPoseUpdate
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Yu Yan */

#ifndef MOVEIT_HANDEYE_CALIBRATION_RVIZ_PLUGIN_HANDEYE_CALIBRATE_WIDGET_
#define MOVEIT_HANDEYE_CALIBRATION_RVIZ_PLUGIN_HANDEYE_CALIBRATE_WIDGET_

// qt
#include <QFile>
#include <QLabel>
#include <QString>
#include <QTreeView>
#include <QComboBox>
#include <QGroupBox>
#include <QTextStream>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QMessageBox>
#include <QProgressBar>
#include <QtConcurrent/QtConcurrent>
#include <QStandardItemModel>

// ros
#include <tf2_eigen/tf2_eigen.h>
#include <pluginlib/class_loader.hpp>
#include <tf2_ros/transform_listener.h>
#include <rviz_visual_tools/tf_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/handeye_calibration_solver/handeye_solver_base.h>
#include <moveit/background_processing/background_processing.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <yaml-cpp/yaml.h>

namespace mhc = moveit_handeye_calibration;

namespace moveit_rviz_plugin
{
class ProgressBarWidget : public QWidget
{
  Q_OBJECT

public:
  ProgressBarWidget(QWidget* parent, int min = 0, int max = 0, int value = 0);

  ~ProgressBarWidget() override = default;

  void setMax(int value);
  void setMin(int value);
  void setValue(int value);
  int getValue();

  QLabel* name_label_;
  QLabel* value_label_;
  QLabel* max_label_;
  QProgressBar* bar_;
};

class ControlTabWidget : public QWidget
{
  Q_OBJECT
public:
  explicit ControlTabWidget(QWidget* parent = Q_NULLPTR);
  ~ControlTabWidget()
  {
    tf_tools_.reset();
    tf_buffer_.reset();
    solver_.reset();
    solver_plugins_loader_.reset();
    move_group_.reset();
    planning_scene_monitor_.reset();
  }

  void loadWidget(const rviz::Config& config);
  void saveWidget(rviz::Config& config);

  void setTFTool(rviz_visual_tools::TFVisualToolsPtr& tf_pub);

  void addPoseSampleToTreeView(const geometry_msgs::TransformStamped& cTo, const geometry_msgs::TransformStamped& bTe,
                               int id);

  bool loadSolverPlugin(std::vector<std::string>& plugins);

  bool createSolverInstance(const std::string& plugin_name);

  void fillSolverTypes(const std::vector<std::string>& plugins);

  std::string parseSolverName(const std::string& solver_name, char delimiter);

  bool takeTranformSamples();

  bool solveCameraRobotPose();

  bool frameNamesEmpty();

  bool checkJointStates();

  void computePlan();

  void computeExecution();

Q_SIGNALS:

  void sensorPoseUpdate(double x, double y, double z, double rx, double ry, double rz);

public Q_SLOTS:

  void UpdateSensorMountType(int index);

  void updateFrameNames(std::map<std::string, std::string> names);

private Q_SLOTS:

  void takeSampleBtnClicked(bool clicked);

  void resetSampleBtnClicked(bool clicked);

  void saveCameraPoseBtnClicked(bool clicked);

  void planningGroupNameChanged(const QString& text);

  void saveJointStateBtnClicked(bool clicked);

  void loadJointStateBtnClicked(bool clicked);

  void autoPlanBtnClicked(bool clicked);

  void autoExecuteBtnClicked(bool clicked);

  void autoSkipBtnClicked(bool clicked);

  void planFinished();

  void executeFinished();

private:
  // **************************************************************
  // Qt components
  // **************************************************************

  QTreeView* sample_tree_view_;
  QStandardItemModel* tree_view_model_;

  QComboBox* calibration_solver_;

  // Load & save pose samples and joint goals
  QPushButton* save_joint_state_btn_;
  QPushButton* load_joint_state_btn_;
  QPushButton* save_camera_pose_btn_;

  // Manual calibration
  QPushButton* take_sample_btn_;
  QPushButton* reset_sample_btn_;

  // Auto calibration
  QComboBox* group_name_;
  QPushButton* auto_plan_btn_;
  QPushButton* auto_execute_btn_;
  QPushButton* auto_skip_btn_;

  // Progress of finished joint states for auto calibration
  ProgressBarWidget* auto_progress_;

  QFutureWatcher<void>* plan_watcher_;
  QFutureWatcher<void>* execution_watcher_;

  // **************************************************************
  // Variables
  // **************************************************************

  mhc::SENSOR_MOUNT_TYPE sensor_mount_type_;
  std::map<std::string, std::string> frame_names_;
  // Transform samples
  std::vector<Eigen::Isometry3d> effector_wrt_world_;
  std::vector<Eigen::Isometry3d> object_wrt_sensor_;
  std::string from_frame_tag_;
  Eigen::Isometry3d camera_robot_pose_;
  std::vector<std::vector<double>> joint_states_;
  std::vector<std::string> joint_names_;
  bool auto_started_;
  bool planning_res_;

  // **************************************************************
  // Ros components
  // **************************************************************

  ros::NodeHandle nh_;
  // ros::CallbackQueue callback_queue_;
  // ros::AsyncSpinner spinner_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rviz_visual_tools::TFVisualToolsPtr tf_tools_;
  std::unique_ptr<pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeSolverBase>> solver_plugins_loader_;
  pluginlib::UniquePtr<moveit_handeye_calibration::HandEyeSolverBase> solver_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::MoveGroupInterface::PlanPtr current_plan_;
};

}  // namespace moveit_rviz_plugin
#endif