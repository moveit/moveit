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

#include <moveit/handeye_calibration_rviz_plugin/handeye_target_widget.h>

namespace moveit_rviz_plugin
{
const std::string LOGNAME = "handeye_target_widget";

void RosTopicComboBox::addMsgsFilterType(QString msgs_type)
{
  message_types_.insert(msgs_type);
}

bool RosTopicComboBox::hasTopic(const QString& topic_name)
{
  getFilteredTopics();
  return image_topics_.contains(topic_name);
}

bool RosTopicComboBox::getFilteredTopics()
{
  // Get all topic names
  ros::master::V_TopicInfo ros_topic_vec;
  if (ros::master::getTopics(ros_topic_vec))
  {
    image_topics_.clear();
    // Filter out the topic names with specific topic type
    for (const ros::master::TopicInfo& topic_info : ros_topic_vec)
    {
      if (message_types_.contains(QString(topic_info.datatype.c_str())))
      {
        image_topics_.insert(QString(topic_info.name.c_str()));
      }
    }
  }

  clear();
  addItem(QString(""));
  for (const QString& topic : image_topics_)
  {
    addItem(topic);
  }

  return !image_topics_.isEmpty();
}

void RosTopicComboBox::mousePressEvent(QMouseEvent* event)
{
  getFilteredTopics();
  showPopup();
}

TargetTabWidget::TargetTabWidget(QWidget* parent)
  : QWidget(parent), nh_("~"), it_(nh_), target_plugins_loader_(nullptr), target_(nullptr)
{
  // Target setting tab area -----------------------------------------------
  QHBoxLayout* layout = new QHBoxLayout();
  this->setLayout(layout);
  QVBoxLayout* layout_left = new QVBoxLayout();
  layout->addLayout(layout_left);

  // Target creation area
  QGroupBox* group_left_top = new QGroupBox("Target_Intrinsic_Params", this);
  layout_left->addWidget(group_left_top);
  QFormLayout* layout_left_top = new QFormLayout();
  group_left_top->setLayout(layout_left_top);

  target_type_ = new QComboBox();
  connect(target_type_, SIGNAL(activated(const QString&)), this, SLOT(targetTypeComboboxChanged(const QString&)));
  layout_left_top->addRow("Target Type", target_type_);

  dictionary_id_ = new QComboBox();
  layout_left_top->addRow("Dictionary", dictionary_id_);

  target_params_.clear();
  target_params_.insert(std::make_pair("markers_x", new QLineEdit()));
  target_params_.insert(std::make_pair("markers_y", new QLineEdit()));
  target_params_.insert(std::make_pair("marker_size", new QLineEdit()));
  target_params_.insert(std::make_pair("marker_dist", new QLineEdit()));
  target_params_.insert(std::make_pair("marker_border", new QLineEdit()));

  target_params_["markers_x"]->setText(QString("4"));
  target_params_["markers_x"]->setValidator(new QIntValidator(1, 50));
  layout_left_top->addRow("Num_Markers_X", target_params_["markers_x"]);

  target_params_["markers_y"]->setText(QString("5"));
  target_params_["markers_y"]->setValidator(new QIntValidator(1, 50));
  layout_left_top->addRow("Num_Markers_Y", target_params_["markers_y"]);

  target_params_["marker_size"]->setText(QString("200"));
  target_params_["marker_size"]->setValidator(new QIntValidator(100, 1000));
  layout_left_top->addRow("Marker_Size_(pixels)", target_params_["marker_size"]);

  target_params_["marker_dist"]->setText(QString("20"));
  target_params_["marker_dist"]->setValidator(new QIntValidator(10, 200));
  layout_left_top->addRow("Marker_Dist_(pixels)", target_params_["marker_dist"]);

  target_params_["marker_border"]->setText(QString("1"));
  target_params_["marker_border"]->setValidator(new QIntValidator(1, 4));
  layout_left_top->addRow("Marker_Border_(bits)", target_params_["marker_border"]);

  // Target 3D pose recognition area
  QGroupBox* group_left_bottom = new QGroupBox("Target_Pose_Recognition", this);
  layout_left->addWidget(group_left_bottom);
  QFormLayout* layout_left_bottom = new QFormLayout();
  group_left_bottom->setLayout(layout_left_bottom);

  ros_topics_.insert(std::make_pair("image_topic", new RosTopicComboBox(this)));
  ros_topics_["image_topic"]->addMsgsFilterType("sensor_msgs/Image");
  layout_left_bottom->addRow("Image Topic", ros_topics_["image_topic"]);
  connect(ros_topics_["image_topic"], SIGNAL(activated(const QString&)), this,
          SLOT(imageTopicComboboxChanged(const QString&)));

  ros_topics_.insert(std::make_pair("camera_info_topic", new RosTopicComboBox(this)));
  ros_topics_["camera_info_topic"]->addMsgsFilterType("sensor_msgs/CameraInfo");
  layout_left_bottom->addRow("CameraInfo Topic", ros_topics_["camera_info_topic"]);
  connect(ros_topics_["camera_info_topic"], SIGNAL(activated(const QString&)), this,
          SLOT(cameraInfoComboBoxChanged(const QString&)));

  target_real_dims_.insert(std::make_pair("marker_size_real", new QLineEdit()));
  target_real_dims_.insert(std::make_pair("marker_dist_real", new QLineEdit()));

  target_real_dims_["marker_size_real"]->setText("0.0256");
  target_real_dims_["marker_size_real"]->setValidator(new QDoubleValidator(0, 2, 4));
  layout_left_bottom->addRow("Marker Size (m)", target_real_dims_["marker_size_real"]);

  target_real_dims_["marker_dist_real"]->setText("0.0066");
  target_real_dims_["marker_dist_real"]->setValidator(new QDoubleValidator(0, 2, 4));
  layout_left_bottom->addRow("Marker Dist (m)", target_real_dims_["marker_dist_real"]);

  // Target image dislay, create and save area
  QGroupBox* group_right = new QGroupBox("Target_Create_Save", this);
  group_right->setMinimumWidth(330);
  layout->addWidget(group_right);
  QVBoxLayout* layout_right = new QVBoxLayout();
  group_right->setLayout(layout_right);

  target_display_label_ = new QLabel();
  target_display_label_->setAlignment(Qt::AlignHCenter);
  layout_right->addWidget(target_display_label_);

  QPushButton* create_target_btn = new QPushButton("Create Target");
  layout_right->addWidget(create_target_btn);
  connect(create_target_btn, SIGNAL(clicked(bool)), this, SLOT(createTargetImageBtnClicked(bool)));

  QPushButton* save_target_btn = new QPushButton("Save Target");
  layout_right->addWidget(save_target_btn);
  connect(save_target_btn, SIGNAL(clicked(bool)), this, SLOT(saveTargetImageBtnClicked(bool)));

  // Load target availible plugins
  if (loadTargetPlugin() && createTargetInstance(target_type_->currentText().toStdString()))
    fillDictionaryIds();

  // Initialize image publisher
  image_pub_ = it_.advertise("/handeye_calibration/target_detection", 1);

  // Initialize camera info dada
  camera_info_.reset(new sensor_msgs::CameraInfo());

  // Register custom types
  qRegisterMetaType<sensor_msgs::CameraInfo>();
  qRegisterMetaType<std::string>();
}

void TargetTabWidget::loadWidget(const rviz::Config& config)
{
  if (target_type_->count() > 0)
  {
    QString type;
    if (config.mapGetString("target_type", &type) && target_type_->findText(type, Qt::MatchCaseSensitive) != -1)
      target_type_->setCurrentText(type);

    createTargetInstance(target_type_->currentText().toStdString());

    QString dict_name;
    config.mapGetString("dictionary", &dict_name);
    fillDictionaryIds(dict_name.toStdString());
  }

  int param_int;
  for (const std::pair<const std::string, QLineEdit*>& param : target_params_)
    if (config.mapGetInt(param.first.c_str(), &param_int))
      param.second->setText(std::to_string(param_int).c_str());

  for (const std::pair<const std::string, RosTopicComboBox*>& topic : ros_topics_)
  {
    QString topic_name;
    if (config.mapGetString(topic.first.c_str(), &topic_name))
    {
      if (topic.second->hasTopic(topic_name))
      {
        topic.second->setCurrentText(topic_name);
        try
        {
          if (!topic.first.compare("image_topic"))
          {
            image_sub_.shutdown();
            image_sub_ = it_.subscribe(topic_name.toStdString(), 1, &TargetTabWidget::imageCallback, this);
          }

          if (!topic.first.compare("camera_info_topic"))
          {
            camerainfo_sub_.shutdown();
            camerainfo_sub_ = nh_.subscribe(topic_name.toStdString(), 1, &TargetTabWidget::cameraInfoCallback, this);
          }
        }
        catch (const image_transport::TransportLoadException& e)
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Subscribe to " << topic_name.toStdString() << " fail: " << e.what());
        }
      }
    }
  }

  for (const std::pair<const std::string, QLineEdit*>& param : target_real_dims_)
  {
    float param_double;
    if (config.mapGetFloat(param.first.c_str(), &param_double))
      param.second->setText(std::to_string(param_double).c_str());
  }
}

void TargetTabWidget::saveWidget(rviz::Config& config)
{
  config.mapSetValue("target_type", target_type_->currentText());
  config.mapSetValue("dictionary", dictionary_id_->currentText());
  for (const std::pair<const std::string, QLineEdit*>& param : target_params_)
    config.mapSetValue(param.first.c_str(), param.second->text().toInt());

  for (const std::pair<const std::string, RosTopicComboBox*>& topic : ros_topics_)
    config.mapSetValue(topic.first.c_str(), topic.second->currentText());

  for (const std::pair<const std::string, QLineEdit*>& param : target_real_dims_)
    config.mapSetValue(param.first.c_str(), param.second->text().toDouble());
}

bool TargetTabWidget::loadTargetPlugin()
{
  if (!target_plugins_loader_)
  {
    try
    {
      target_plugins_loader_.reset(new pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeTargetBase>(
          "moveit_ros_perception", "moveit_handeye_calibration::HandEyeTargetBase"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      QMessageBox::warning(this, tr("Exception while creating handeye target plugin loader "), tr(ex.what()));
      return false;
    }
  }

  // Get target classes
  const std::vector<std::string>& classes = target_plugins_loader_->getDeclaredClasses();

  target_type_->clear();
  if (classes.empty())
  {
    QMessageBox::warning(this, tr("Missing target plugins"), "No MoveIt handeye calibration target plugin found.");
    return false;
  }

  for (const std::string& it : classes)
    target_type_->addItem(tr(it.c_str()));

  return true;
}

bool TargetTabWidget::createTargetInstance(const std::string& plugin_name)
{
  if (plugin_name.empty())
    return false;

  try
  {
    target_ = target_plugins_loader_->createUniqueInstance(plugin_name);
    target_->initialize(target_params_["markers_x"]->text().toInt(), target_params_["markers_y"]->text().toInt(),
                        target_params_["marker_size"]->text().toInt(), target_params_["marker_dist"]->text().toInt(),
                        target_params_["marker_border"]->text().toInt(), dictionary_id_->currentText().toStdString(),
                        target_real_dims_["marker_size_real"]->text().toDouble(),
                        target_real_dims_["marker_dist_real"]->text().toDouble());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    QMessageBox::warning(this, tr("Exception while loading a handeye target plugin"), tr(ex.what()));
    target_ = nullptr;
    return false;
  }

  return true;
}

void TargetTabWidget::fillDictionaryIds(std::string id)
{
  if (target_)
  {
    // Get dictionary ids
    const std::vector<std::string>& ids = target_->getDictionaryIds();
    dictionary_id_->clear();
    for (const std::string& id : ids)
    {
      dictionary_id_->addItem(tr(id.c_str()));
    }

    // Check if 'id' exists in the list
    if (!id.empty())
    {
      auto it = std::find(ids.begin(), ids.end(), id);
      if (it != ids.end())
        dictionary_id_->setCurrentText(QString(id.c_str()));
    }
  }
}

void TargetTabWidget::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  targetParamsSet();

  // Depth image format `16UC1` cannot be converted to `MONO8`
  if (msg->encoding == "16UC1")
    return;

  std::string frame_id = msg->header.frame_id;
  if (!frame_id.empty())
  {
    if (optical_frame_.compare(frame_id))
    {
      optical_frame_ = frame_id;
      Q_EMIT opticalFrameChanged(optical_frame_);
    }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Image msg has empty frame_id.");
    return;
  }

  if (msg->data.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Image msg has empty data.");
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

    sensor_msgs::ImagePtr pub_msg;
    if (target_ && target_->detectTargetPose(cv_ptr->image))
    {
      pub_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cv_ptr->image).toImageMsg();

      geometry_msgs::TransformStamped tf2_msg = target_->getTransformStamped(optical_frame_);
      tf_pub_.sendTransform(tf2_msg);
    }
    else
    {
      pub_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv_ptr->image).toImageMsg();
    }
    image_pub_.publish(pub_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "cv_bridge exception: " << e.what());
  }
  catch (cv::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "cv exception: " << e.what());
  }
}

void TargetTabWidget::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  if (target_)
  {
    if (msg->height > 0 && msg->width > 0 && !msg->K.empty() && !msg->D.empty())
    {
      if (msg->K != camera_info_->K || msg->P != camera_info_->P)
      {
        ROS_DEBUG("Received camera info.");
        camera_info_->header = msg->header;
        camera_info_->height = msg->height;
        camera_info_->width = msg->width;
        camera_info_->distortion_model = msg->distortion_model;
        camera_info_->D = msg->D;
        camera_info_->K = msg->K;
        camera_info_->R = msg->R;
        camera_info_->P = msg->P;
        target_->setCameraIntrinsicParams(camera_info_);
        Q_EMIT cameraInfoChanged(*camera_info_);
      }
    }
  }
}

void TargetTabWidget::targetTypeComboboxChanged(const QString& text)
{
  if (!text.isEmpty())
  {
    if (createTargetInstance(text.toStdString()))
    {
      fillDictionaryIds();
    }
  }
}

void TargetTabWidget::createTargetImageBtnClicked(bool clicked)
{
  if (target_)
  {
    targetParamsSet();
    target_->createTargetImage(target_image_);
  }
  else
    QMessageBox::warning(this, tr("Fail to create a target image."), "No available target plugin.");

  if (!target_image_.empty())
  {
    // Show target image
    QImage qimage(target_image_.data, target_image_.cols, target_image_.rows, QImage::Format_Grayscale8);
    if (target_image_.cols > target_image_.rows)
      qimage = qimage.scaledToWidth(320, Qt::SmoothTransformation);
    else
      qimage = qimage.scaledToHeight(260, Qt::SmoothTransformation);
    target_display_label_->setPixmap(QPixmap::fromImage(qimage));
  }
}

void TargetTabWidget::targetParamsSet(const QString& text)
{
  if (target_)
  {
    target_->setTargetIntrinsicParams(
        target_params_["markers_x"]->text().toInt(), target_params_["markers_y"]->text().toInt(),
        target_params_["marker_size"]->text().toInt(), target_params_["marker_dist"]->text().toInt(),
        target_params_["marker_border"]->text().toInt(), dictionary_id_->currentText().toStdString());
    target_->setTargetDimension(target_real_dims_["marker_size_real"]->text().toDouble(),
                                target_real_dims_["marker_dist_real"]->text().toDouble());
  }
}

void TargetTabWidget::saveTargetImageBtnClicked(bool clicked)
{
  if (target_image_.empty())
  {
    QMessageBox::warning(this, tr("Unable to save image"), tr("Please create a target at first."));
    return;
  }

  QString fileName =
      QFileDialog::getSaveFileName(this, tr("Save Target Image"), "", tr("Target Image (*.png);;All Files (*)"));

  if (fileName.isEmpty())
    return;

  if (!fileName.endsWith(".png"))
    fileName += ".png";

  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly))
  {
    QMessageBox::warning(this, tr("Unable to open file"), file.errorString());
    return;
  }

  if (!cv::imwrite(cv::String(fileName.toStdString()), target_image_))
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error OpenCV saving image.");
}

void TargetTabWidget::imageTopicComboboxChanged(const QString& topic)
{
  image_sub_.shutdown();
  if (!topic.isNull() and !topic.isEmpty())
  {
    try
    {
      image_sub_ = it_.subscribe(topic.toStdString(), 1, &TargetTabWidget::imageCallback, this);
    }
    catch (image_transport::TransportLoadException& e)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Subscribe to image topic: " << topic.toStdString() << " failed. " << e.what());
    }
  }
}

void TargetTabWidget::cameraInfoComboBoxChanged(const QString& topic)
{
  camerainfo_sub_.shutdown();
  if (!topic.isNull() and !topic.isEmpty())
  {
    try
    {
      camerainfo_sub_ = nh_.subscribe(topic.toStdString(), 1, &TargetTabWidget::cameraInfoCallback, this);
    }
    catch (ros::Exception& e)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Subscribe to camera info topic: " << topic.toStdString() << " failed. "
                                                                         << e.what());
    }
  }
}

}  // namedist moveit_rviz_plugin