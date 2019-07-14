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

#include <moveit/handeye_calibration_rviz_plugin/handeye_context_widget.h>
#include <math.h>

namespace moveit_rviz_plugin
{
const std::string LOGNAME = "handeye_context_widget";

void TFFrameNameComboBox::mousePressEvent(QMouseEvent* event)
{
  std::vector<std::string> names;
  frame_manager_->update();
  frame_manager_->getTF2BufferPtr()->_getFrameStrings(names);

  clear();
  addItem(QString(""));
  if (robot_model_loader_->getModel())  // Ensure that robot is brought up
  {
    const std::vector<std::string>& robot_links = robot_model_loader_->getModel()->getLinkModelNames();
    for (const std::string& name : names)
    {
      auto it = std::find(robot_links.begin(), robot_links.end(), name);
      size_t index = name.find("camera");

      if (frame_source_ == ROBOT_FRAME)
        if (it != robot_links.end())
          addItem(QString(name.c_str()));

      if (frame_source_ == CAMERA_FRAME)
        if (index != std::string::npos)
          addItem(QString(name.c_str()));

      if (frame_source_ == ENVIRONMENT_FRAME)
        if (it == robot_links.end() && index == std::string::npos)
          addItem(QString(name.c_str()));
    }
  }
  showPopup();
}

bool TFFrameNameComboBox::hasFrame(const std::string& frame_name)
{
  std::vector<std::string> names;
  frame_manager_->update();
  frame_manager_->getTF2BufferPtr()->_getFrameStrings(names);

  auto it = std::find(names.begin(), names.end(), frame_name);
  return it != names.end();
}

SliderWidget::SliderWidget(QWidget* parent, std::string name, double min, double max)
  : QWidget(parent), min_position_(min), max_position_(max)
{
  QHBoxLayout* row = new QHBoxLayout(this);
  row->setContentsMargins(0, 10, 0, 10);

  // QLabel init
  label_ = new QLabel(QString(name.c_str()), this);
  label_->setContentsMargins(0, 0, 0, 0);
  row->addWidget(label_);

  // QSlider init
  slider_ = new QSlider(Qt::Horizontal, this);
  slider_->setSingleStep(100);
  slider_->setPageStep(100);
  slider_->setTickInterval(1000);
  slider_->setContentsMargins(0, 0, 0, 0);
  row->addWidget(slider_);

  slider_->setMaximum(max_position_ * 10000);
  slider_->setMinimum(min_position_ * 10000);

  connect(slider_, SIGNAL(valueChanged(int)), this, SLOT(changeValue(int)));

  // QLineEdit init
  edit_ = new QLineEdit(this);
  edit_->setMinimumWidth(62);
  edit_->setContentsMargins(0, 0, 0, 0);
  connect(edit_, SIGNAL(editingFinished()), this, SLOT(changeSlider()));
  row->addWidget(edit_);

  this->setLayout(row);
}

double SliderWidget::getValue()
{
  return edit_->text().toDouble();
}

void SliderWidget::setValue(double value)
{
  if (min_position_ > value || value > max_position_)
  {
    value = (min_position_ > value) ? min_position_ : max_position_;
  }
  edit_->setText(QString("%1").arg(value, 0, 'f', 4));
  slider_->setSliderPosition(value * 10000);
}

void SliderWidget::changeValue(int value)
{
  const double double_value = double(value) / 10000;

  // Set textbox
  edit_->setText(QString("%1").arg(double_value, 0, 'f', 4));

  // Send event to parent widget
  Q_EMIT valueChanged(double_value);
}

void SliderWidget::changeSlider()
{
  // Get joint value
  double value = edit_->text().toDouble();

  setValue(value);

  // Send event to parent widget
  Q_EMIT valueChanged(value);
}

ContextTabWidget::ContextTabWidget(QWidget* parent) : QWidget(parent), tf_listener_(tf_buffer_)
{
  // Context setting tab ----------------------------------------------------
  QHBoxLayout* layout = new QHBoxLayout();
  this->setLayout(layout);
  QVBoxLayout* layout_left = new QVBoxLayout();
  layout->addLayout(layout_left);
  QVBoxLayout* layout_right = new QVBoxLayout();
  layout->addLayout(layout_right);

  // Sensor mount type area and fov area
  QGroupBox* group_left_top = new QGroupBox("General Setting", this);
  layout_left->addWidget(group_left_top);
  QFormLayout* layout_left_top = new QFormLayout();
  group_left_top->setLayout(layout_left_top);

  sensor_mount_type_ = new QComboBox();
  sensor_mount_type_->addItem("Eye-to-Hand");
  sensor_mount_type_->addItem("Eye-in-hand");
  layout_left_top->addRow("Sensor Mount Type", sensor_mount_type_);
  connect(sensor_mount_type_, SIGNAL(activated(int)), this, SLOT(updateSensorMountType(int)));

  // Frame name selection area
  QGroupBox* frame_group = new QGroupBox("Frames Selection", this);
  layout_left->addWidget(frame_group);
  QFormLayout* frame_layout = new QFormLayout();
  frame_group->setLayout(frame_layout);

  frames_.insert(std::make_pair("sensor", new TFFrameNameComboBox(CAMERA_FRAME)));
  frame_layout->addRow("Sensor Frame:", frames_["sensor"]);

  frames_.insert(std::make_pair("object", new TFFrameNameComboBox(ENVIRONMENT_FRAME)));
  frame_layout->addRow("Object Frame:", frames_["object"]);

  frames_.insert(std::make_pair("eef", new TFFrameNameComboBox(ROBOT_FRAME)));
  frame_layout->addRow("End-Effector Frame:", frames_["eef"]);

  frames_.insert(std::make_pair("base", new TFFrameNameComboBox(ROBOT_FRAME)));
  frame_layout->addRow("Robot Base Frame:", frames_["base"]);

  for (std::pair<const std::string, TFFrameNameComboBox*>& frame : frames_)
    connect(frame.second, SIGNAL(activated(int)), this, SLOT(updateFrameName(int)));

  // FOV area
  QGroupBox* fov_group = new QGroupBox("FOV", this);
  layout_left->addWidget(fov_group);
  QFormLayout* fov_layout = new QFormLayout();
  fov_group->setLayout(fov_layout);

  fov_alpha_ = new SliderWidget(this, "Transparency", 0, 1);
  fov_alpha_->setValue(0.5);
  fov_layout->addRow(fov_alpha_);
  connect(fov_alpha_, SIGNAL(valueChanged(double)), this, SLOT(updateCameraMarkerPose(double)));

  fov_on_off_ = new QRadioButton();
  fov_on_off_->setChecked(true);
  fov_layout->addRow("ON/OFF", fov_on_off_);
  connect(fov_on_off_, SIGNAL(toggled(bool)), this, SLOT(fovOnOffBtnToggled(bool)));

  // Camera Pose initial guess area
  QGroupBox* pose_group = new QGroupBox("Camera Pose Inital Guess", this);
  pose_group->setMinimumWidth(300);
  layout_right->addWidget(pose_group);
  QFormLayout* pose_layout = new QFormLayout();
  pose_group->setLayout(pose_layout);

  guess_pose_.insert(std::make_pair("Tx", new SliderWidget(this, "TranslX", -2.0, 2.0)));
  pose_layout->addRow(guess_pose_["Tx"]);

  guess_pose_.insert(std::make_pair("Ty", new SliderWidget(this, "TranslY", -2.0, 2.0)));
  pose_layout->addRow(guess_pose_["Ty"]);

  guess_pose_.insert(std::make_pair("Tz", new SliderWidget(this, "TranslZ", -2.0, 2.0)));
  pose_layout->addRow(guess_pose_["Tz"]);

  guess_pose_.insert(std::make_pair("Rx", new SliderWidget(this, "RotateX", -M_PI, M_PI)));
  pose_layout->addRow(guess_pose_["Rx"]);

  guess_pose_.insert(std::make_pair("Ry", new SliderWidget(this, "RotateY", -M_PI, M_PI)));
  pose_layout->addRow(guess_pose_["Ry"]);

  guess_pose_.insert(std::make_pair("Rz", new SliderWidget(this, "RotateZ", -M_PI, M_PI)));
  pose_layout->addRow(guess_pose_["Rz"]);

  for (std::pair<const std::string, SliderWidget*>& dim : guess_pose_)
  {
    dim.second->setValue(0);
    connect(dim.second, SIGNAL(valueChanged(double)), this, SLOT(updateCameraMarkerPose(double)));
  }

  // Variable Initialization
  camera_pose_ = Eigen::Isometry3d::Identity();
  fov_pose_ = Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5);
  fov_pose_.translate(Eigen::Vector3d(0.0149, 0.0325, 0.0125));

  camera_info_.reset(new sensor_msgs::CameraInfo());

  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world"));
  visual_tools_->enableFrameLocking(true);
  visual_tools_->setAlpha(1.0);
  visual_tools_->setLifetime(0.0);
  visual_tools_->trigger();
}

void ContextTabWidget::loadWidget(const rviz::Config& config)
{
  int index;
  if (config.mapGetInt("sensor_mount_type", &index))
    sensor_mount_type_->setCurrentIndex(index);

  Q_EMIT sensorMountTypeChanged(index);

  for (std::pair<const std::string, TFFrameNameComboBox*>& frame : frames_)
  {
    QString frame_name;
    if (config.mapGetString(frame.first.c_str(), &frame_name))
    {
      frame.second->clear();
      if (!frame_name.isEmpty() && frame.second->hasFrame(frame_name.toStdString()))
        frame.second->addItem(frame_name);
    }
  }

  float alpha;
  if (config.mapGetFloat("fov_transparent", &alpha))
    fov_alpha_->setValue(alpha);

  bool fov_enabled;
  if (config.mapGetBool("fov_on_off", &fov_enabled))
    fov_on_off_->setChecked(fov_enabled);

  for (std::pair<const std::string, SliderWidget*>& dim : guess_pose_)
  {
    float value;
    if (config.mapGetFloat(dim.first.c_str(), &value))
      dim.second->setValue(value);
  }
  updateAllMarkers();

  std::map<std::string, std::string> names;
  for (std::pair<const std::string, TFFrameNameComboBox*>& frame : frames_)
    names.insert(std::make_pair(frame.first, frame.second->currentText().toStdString()));

  Q_EMIT frameNameChanged(names);
}

void ContextTabWidget::saveWidget(rviz::Config& config)
{
  config.mapSetValue("sensor_mount_type", sensor_mount_type_->currentIndex());

  for (std::pair<const std::string, TFFrameNameComboBox*>& frame : frames_)
    config.mapSetValue(frame.first.c_str(), frame.second->currentText());

  config.mapSetValue("fov_transparent", fov_alpha_->getValue());
  config.mapSetValue("fov_on_off", fov_on_off_->isChecked());

  for (std::pair<const std::string, SliderWidget*>& dim : guess_pose_)
    config.mapSetValue(dim.first.c_str(), dim.second->getValue());
}

void ContextTabWidget::setTFTool(rviz_visual_tools::TFVisualToolsPtr& tf_pub)
{
  tf_tools_ = tf_pub;
}

void ContextTabWidget::updateAllMarkers()
{
  if (visual_tools_ && tf_tools_)
  {
    visual_tools_->deleteAllMarkers();
    tf_tools_->clearAllTransforms();

    QString from_frame("");
    mhc::SENSOR_MOUNT_TYPE setup = static_cast<mhc::SENSOR_MOUNT_TYPE>(sensor_mount_type_->currentIndex());

    switch (setup)
    {
      case mhc::EYE_TO_HAND:
        from_frame = frames_["base"]->currentText();
        break;
      case mhc::EYE_IN_HAND:
        from_frame = frames_["eef"]->currentText();
        break;
      default:
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Error sensor mount type.");
        break;
    }

    if (!from_frame.isEmpty())
    {
      for (std::pair<const std::string, TFFrameNameComboBox*> frame : frames_)
      {
        // Publish selected frame axis
        const std::string& frame_id = frame.second->currentText().toStdString();
        if (!frame_id.empty())
        {
          visual_tools_->setBaseFrame(frame_id);
          visual_tools_->setAlpha(1.0);
          visual_tools_->publishAxisLabeled(Eigen::Isometry3d::Identity(), frame_id);
        }
      }

      // Publish camera and fov marker
      QString to_frame = frames_["sensor"]->currentText();
      if (!to_frame.isEmpty())
      {
        // // Get camera pose guess
        setCameraPose(guess_pose_["Tx"]->getValue(), guess_pose_["Ty"]->getValue(), guess_pose_["Tz"]->getValue(),
                      guess_pose_["Rx"]->getValue(), guess_pose_["Ry"]->getValue(), guess_pose_["Rz"]->getValue());

        // Publish new transform from robot base or end-effector to sensor frame
        tf_tools_->publishTransform(camera_pose_, from_frame.toStdString(), to_frame.toStdString());

        // Publish new FOV marker
        shape_msgs::Mesh mesh = getCameraFOVMesh(*camera_info_, 1.5);
        if (fov_on_off_->isChecked())
        {
          visual_tools_->setBaseFrame(to_frame.toStdString());
          visual_tools_->setAlpha(fov_alpha_->getValue());
          visual_tools_->publishMesh(fov_pose_, mesh, rvt::YELLOW, 1.0, "fov", 1);
        }
      }
    }

    visual_tools_->trigger();
  }
  else
    ROS_ERROR("Visual or TF tool is NULL.");
}

void ContextTabWidget::updateFOVPose()
{
  QString sensor_frame = frames_["sensor"]->currentText();
  geometry_msgs::TransformStamped tf_msg;
  if (!optical_frame_.empty() && !sensor_frame.isEmpty())
  {
    try
    {
      // Get FOV pose W.R.T sensor frame
      tf_msg = tf_buffer_.lookupTransform(sensor_frame.toStdString(), optical_frame_, ros::Time(0));
      fov_pose_ = tf2::transformToEigen(tf_msg);
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "FOV pose from '" << sensor_frame.toStdString() << "' to '" << optical_frame_
                                                        << "' is:"
                                                        << "\nTranslation:\n"
                                                        << fov_pose_.translation() << "\nRotation:\n"
                                                        << fov_pose_.rotation());
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN_STREAM("TF exception: " << e.what());
    }
  }
}

shape_msgs::Mesh ContextTabWidget::getCameraFOVMesh(const sensor_msgs::CameraInfo& camera_info, double max_dist)
{
  shape_msgs::Mesh mesh;
  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(camera_info);
  double delta_x = camera_model.getDeltaX(camera_info.width / 2, max_dist);
  double delta_y = camera_model.getDeltaY(camera_info.height / 2, max_dist);

  std::vector<double> x_cords = { -delta_x, delta_x };
  std::vector<double> y_cords = { -delta_y, delta_y };

  // Get corners
  mesh.vertices.clear();
  // Add the first corner at origin of the optical frame
  mesh.vertices.push_back(geometry_msgs::Point());

  // Add the four corners at bottom
  for (const double& x_it : x_cords)
    for (const double& y_it : y_cords)
    {
      geometry_msgs::Point vertex;
      // Check in case camera info is not valid
      if (std::isfinite(x_it) && std::isfinite(y_it) && std::isfinite(max_dist))
      {
        vertex.x = x_it;
        vertex.y = y_it;
        vertex.z = max_dist;
      }
      mesh.vertices.push_back(vertex);
    }

  // Get surface triangles
  mesh.triangles.resize(4);
  mesh.triangles[0].vertex_indices = { 0, 1, 2 };
  mesh.triangles[1].vertex_indices = { 0, 2, 4 };
  mesh.triangles[2].vertex_indices = { 0, 4, 3 };
  mesh.triangles[3].vertex_indices = { 0, 3, 1 };
  return mesh;
}

visualization_msgs::Marker ContextTabWidget::getCameraFOVMarker(const Eigen::Isometry3d& pose,
                                                                const shape_msgs::Mesh& mesh, rvt::colors color,
                                                                double alpha, std::string frame_id)
{
  return getCameraFOVMarker(rvt::RvizVisualTools::convertPose(pose), mesh, color, alpha, frame_id);
}

visualization_msgs::Marker ContextTabWidget::getCameraFOVMarker(const geometry_msgs::Pose& pose,
                                                                const shape_msgs::Mesh& mesh, rvt::colors color,
                                                                double alpha, std::string frame_id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "camera_fov";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0.0);
  visual_tools_->setAlpha(alpha);
  marker.color = visual_tools_->getColor(color);
  marker.pose = pose;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.points.clear();
  for (const shape_msgs::MeshTriangle& triangle : mesh.triangles)
    for (const uint32_t& index : triangle.vertex_indices)
      marker.points.push_back(mesh.vertices[index]);

  return marker;
}

void ContextTabWidget::setCameraPose(double tx, double ty, double tz, double rx, double ry, double rz)
{
  camera_pose_.setIdentity();
  camera_pose_ = visual_tools_->convertFromXYZRPY(tx, ty, tz, rx, ry, rz, rviz_visual_tools::XYZ);
}

void ContextTabWidget::setCameraInfo(sensor_msgs::CameraInfo camera_info)
{
  camera_info_->header = camera_info.header;
  camera_info_->height = camera_info.height;
  camera_info_->width = camera_info.width;
  camera_info_->distortion_model = camera_info.distortion_model;
  camera_info_->D = camera_info.D;
  camera_info_->K = camera_info.K;
  camera_info_->R = camera_info.R;
  camera_info_->P = camera_info.P;
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Camera info changed: " << *camera_info_);
}

void ContextTabWidget::setOpticalFrame(const std::string& frame_id)
{
  optical_frame_ = frame_id;
  updateFOVPose();
}

void ContextTabWidget::updateCameraPose(double tx, double ty, double tz, double rx, double ry, double rz)
{
  // setCameraPose(tx, ty, tz, rx, ry, rz);
  guess_pose_["Tx"]->setValue(tx);
  guess_pose_["Ty"]->setValue(ty);
  guess_pose_["Tz"]->setValue(tz);
  guess_pose_["Rx"]->setValue(rx);
  guess_pose_["Ry"]->setValue(ry);
  guess_pose_["Rz"]->setValue(rz);
  updateCameraMarkerPose(0);
}

void ContextTabWidget::updateSensorMountType(int index)
{
  for (std::pair<const std::string, SliderWidget*> dim : guess_pose_)
    dim.second->setValue(0);

  updateAllMarkers();

  Q_EMIT sensorMountTypeChanged(index);
}

void ContextTabWidget::updateFrameName(int index)
{
  updateAllMarkers();
  updateFOVPose();

  std::map<std::string, std::string> names;
  for (std::pair<const std::string, TFFrameNameComboBox*>& frame : frames_)
    names.insert(std::make_pair(frame.first, frame.second->currentText().toStdString()));

  Q_EMIT frameNameChanged(names);
}

void ContextTabWidget::updateCameraMarkerPose(double value)
{
  updateAllMarkers();
}

void ContextTabWidget::fovOnOffBtnToggled(bool checked)
{
  updateAllMarkers();
}

}  // namgespace moveit_rviz_plugin