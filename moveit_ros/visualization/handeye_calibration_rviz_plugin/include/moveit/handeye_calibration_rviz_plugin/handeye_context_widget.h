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
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
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

#ifndef MOVEIT_HANDEYE_CALIBRATION_RVIZ_PLUGIN_HANDEYE_CONTEXT_WIDGET_
#define MOVEIT_HANDEYE_CALIBRATION_RVIZ_PLUGIN_HANDEYE_CONTEXT_WIDGET_

// qt
#include <QLabel>
#include <QWidget>
#include <QSlider>
#include <QComboBox>
#include <QLineEdit>
#include <QGroupBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QRadioButton>

// ros
#include <shape_msgs/Mesh.h>
#include <rviz/frame_manager.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/CameraInfo.h>
#include <rviz_visual_tools/tf_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <image_geometry/pinhole_camera_model.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/handeye_calibration_solver/handeye_solver_base.h>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

namespace rvt = rviz_visual_tools;
namespace mhc = moveit_handeye_calibration;

namespace moveit_rviz_plugin
{
enum FRAME_SOURCE
{
  ROBOT_FRAME = 0,
  CAMERA_FRAME = 1,
  ENVIRONMENT_FRAME = 2
};

// **************************************************
// Custom QComboBox for frame name
// **************************************************
class TFFrameNameComboBox : public QComboBox
{
  Q_OBJECT
public:
  TFFrameNameComboBox(FRAME_SOURCE source = ROBOT_FRAME, QWidget* parent = 0) : QComboBox(parent), frame_source_(source)
  {
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    frame_manager_.reset(new rviz::FrameManager());
  }

  ~TFFrameNameComboBox()
  {
    robot_model_loader_.reset();
  }

  bool hasFrame(const std::string& frame_name);

protected:
  void mousePressEvent(QMouseEvent* event);

private:
  FRAME_SOURCE frame_source_;
  std::unique_ptr<rviz::FrameManager> frame_manager_;
  robot_model_loader::RobotModelLoaderConstPtr robot_model_loader_;
};

// **************************************************
// Custom slider class
// **************************************************
class SliderWidget : public QWidget
{
  Q_OBJECT

public:
  SliderWidget(QWidget* parent, std::string name, double min, double max);

  ~SliderWidget() override = default;

  double getValue();

  void setValue(double value);

  QLabel* label_;
  QSlider* slider_;
  QLineEdit* edit_;

private Q_SLOTS:

  // Called when the slider is changed
  void changeValue(int value);

  // Called when the edit box is changed
  void changeSlider();

Q_SIGNALS:

  // Indicate value when slider widget changed
  void valueChanged(double value);

private:
  // Max & min position
  double max_position_;
  double min_position_;
};

class ContextTabWidget : public QWidget
{
  Q_OBJECT
public:
  explicit ContextTabWidget(QWidget* parent = Q_NULLPTR);
  ~ContextTabWidget()
  {
    camera_info_.reset();
    visual_tools_.reset();
    tf_tools_.reset();
  }

  void loadWidget(const rviz::Config& config);
  void saveWidget(rviz::Config& config);
  void setTFTool(rviz_visual_tools::TFVisualToolsPtr& tf_pub);

  void updateAllMarkers();

  void updateFOVPose();

  static shape_msgs::Mesh getCameraFOVMesh(const sensor_msgs::CameraInfo& camera_info, double maxdist);

  visualization_msgs::Marker getCameraFOVMarker(const Eigen::Isometry3d& pose, const shape_msgs::Mesh& mesh,
                                                rvt::colors color, double alpha, std::string frame_id);

  visualization_msgs::Marker getCameraFOVMarker(const geometry_msgs::Pose& pose, const shape_msgs::Mesh& mesh,
                                                rvt::colors color, double alpha, std::string frame_id);

  void setCameraPose(double tx, double ty, double tz, double rx, double ry, double rz);

public Q_SLOTS:

  void setCameraInfo(sensor_msgs::CameraInfo camera_info);

  void setOpticalFrame(const std::string& frame_id);

  void updateCameraPose(double tx, double ty, double tz, double rx, double ry, double rz);

private Q_SLOTS:

  // Called when the sensor_mount_type_ changed
  void updateSensorMountType(int index);

  // Called when the TFFrameNameComboBox changed
  void updateFrameName(int index);

  // Called when the slider of initial camera pose guess changed
  void updateCameraMarkerPose(double value);

  // Called when the fov_on_off_ button toggled
  void fovOnOffBtnToggled(bool checked);

Q_SIGNALS:

  void sensorMountTypeChanged(int index);

  void frameNameChanged(std::map<std::string, std::string> names);

private:
  // **************************************************************
  // Qt components
  // **************************************************************

  // Calibration algorithm, sensor mount type area
  QComboBox* sensor_mount_type_;

  // Frame selection area
  std::map<std::string, TFFrameNameComboBox*> frames_;

  // FOV setting area
  QRadioButton* fov_on_off_;
  SliderWidget* fov_alpha_;

  // Initial camera pose
  std::map<std::string, SliderWidget*> guess_pose_;

  // **************************************************************
  // Variables
  // **************************************************************

  sensor_msgs::CameraInfoPtr camera_info_;

  // Transform from camera to robot base or end-effector
  Eigen::Isometry3d camera_pose_;

  std::string optical_frame_;

  // Transform from camera to fov
  Eigen::Isometry3d fov_pose_;

  // **************************************************************
  // Ros components
  // **************************************************************

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  rviz_visual_tools::TFVisualToolsPtr tf_tools_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace moveit_rviz_plugin

#endif