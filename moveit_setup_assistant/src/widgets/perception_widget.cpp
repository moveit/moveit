/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Mohamad Ayman.
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
 *   * The name of Mohamad Ayman may not be used to endorse or promote products derived
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

/* Author: Mohamad Ayman */

// SA
#include "perception_widget.h"

// Qt
#include <QVBoxLayout>
#include <QFormLayout>
#include <QApplication>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Constructor
// ******************************************************************************************
PerceptionWidget::PerceptionWidget(QWidget* parent, moveit_setup_assistant::MoveItConfigDataPtr config_data)
  : SetupScreenWidget(parent), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();
  layout->setAlignment(Qt::AlignTop);

  // Top Header Area ------------------------------------------------

  HeaderWidget* header =
      new HeaderWidget("Setup 3D Perception Sensors",
                       "Configure your 3D sensors to work with Moveit! "
                       "Please see <a "
                       "href='http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/perception_pipeline/"
                       "perception_pipeline_tutorial.html'>Perception Documentation</a> "
                       "for more details.",
                       this);
  layout->addWidget(header);

  // Add spacing
  QSpacerItem* blank_space = new QSpacerItem(1, 8);
  layout->addSpacerItem(blank_space);

  // Plugin type combo box
  QLabel* plugin_field_title = new QLabel(this);
  plugin_field_title->setText("Optionally choose a type of 3D sensor plugin to configure:");
  layout->addWidget(plugin_field_title);

  sensor_plugin_field_ = new QComboBox(this);
  sensor_plugin_field_->setEditable(false);
  sensor_plugin_field_->setMaximumWidth(600);
  connect(sensor_plugin_field_, SIGNAL(currentIndexChanged(int)), this, SLOT(sensorPluginChanged(int)));
  layout->addWidget(sensor_plugin_field_);

  // Point Cloud group -------------------------------------------
  point_cloud_group_ = new QGroupBox("Point Cloud");

  QFormLayout* point_cloud_form_layout = new QFormLayout();
  point_cloud_form_layout->setContentsMargins(0, 15, 0, 15);

  // Point Cloud Topic
  point_cloud_topic_field_ = new QLineEdit(this);
  point_cloud_topic_field_->setMaximumWidth(400);
  // point_cloud_topic_field_->setText(QString("/clud topic"));
  point_cloud_form_layout->addRow("Point Cloud Topic:", point_cloud_topic_field_);

  // Max Range
  max_range_field_ = new QLineEdit(this);
  max_range_field_->setMaximumWidth(400);
  point_cloud_form_layout->addRow("Max Range:", max_range_field_);

  // Point Subsample
  point_subsample_field_ = new QLineEdit(this);
  point_subsample_field_->setMaximumWidth(400);
  point_cloud_form_layout->addRow("Point Subsample:", point_subsample_field_);

  // Padding Offset
  padding_offset_field_ = new QLineEdit(this);
  padding_offset_field_->setMaximumWidth(400);
  point_cloud_form_layout->addRow("Padding Offset:", padding_offset_field_);

  // Padding Scale
  padding_scale_field_ = new QLineEdit(this);
  padding_scale_field_->setMaximumWidth(400);
  point_cloud_form_layout->addRow("Padding Scale:", padding_scale_field_);

  // Filtered Cloud Topic
  filtered_cloud_topic_field_ = new QLineEdit(this);
  filtered_cloud_topic_field_->setMaximumWidth(400);
  point_cloud_form_layout->addRow("Filtered Cloud Topic:", filtered_cloud_topic_field_);

  // Max Update Rate
  max_update_rate_field_ = new QLineEdit(this);
  max_update_rate_field_->setMaximumWidth(400);
  point_cloud_form_layout->addRow("Max Update Rate:", max_update_rate_field_);

  // Piont Cloud form layout
  point_cloud_group_->setLayout(point_cloud_form_layout);
  layout->addWidget(point_cloud_group_);

  // Depth map group --------------------------------------------
  depth_map_group_ = new QGroupBox("Depth Map");

  // Depth Map form layout
  QFormLayout* depth_map_form_layout = new QFormLayout();
  depth_map_form_layout->setContentsMargins(0, 15, 0, 15);

  // Image Topic
  image_topic_field_ = new QLineEdit(this);
  image_topic_field_->setMaximumWidth(400);
  depth_map_form_layout->addRow("Image Topic:", image_topic_field_);

  // Queue Size
  queue_size_field_ = new QLineEdit(this);
  queue_size_field_->setMaximumWidth(400);
  depth_map_form_layout->addRow("Queue Size:", queue_size_field_);

  // Near Clipping Plane Distance
  near_clipping_field_ = new QLineEdit(this);
  near_clipping_field_->setMaximumWidth(400);
  depth_map_form_layout->addRow("Near Clipping Plane Distance:", near_clipping_field_);

  // Far Clipping Plane Distance
  far_clipping_field_ = new QLineEdit(this);
  far_clipping_field_->setMaximumWidth(400);
  depth_map_form_layout->addRow("Far Clipping Plane Distance:", far_clipping_field_);

  // Shadow Threshold
  shadow_threshold_field_ = new QLineEdit(this);
  shadow_threshold_field_->setMaximumWidth(400);
  depth_map_form_layout->addRow("Shadow Threshold:", shadow_threshold_field_);

  // Padding Offset
  depth_padding_offset_field_ = new QLineEdit(this);
  depth_padding_offset_field_->setMaximumWidth(400);
  depth_map_form_layout->addRow("Padding Offset:", depth_padding_offset_field_);

  // Padding Scale
  depth_padding_scale_field_ = new QLineEdit(this);
  depth_padding_scale_field_->setMaximumWidth(400);
  depth_map_form_layout->addRow("Padding Scale:", depth_padding_scale_field_);

  // Filtered Cloud Topic
  depth_filtered_cloud_topic_field_ = new QLineEdit(this);
  depth_filtered_cloud_topic_field_->setMaximumWidth(400);
  depth_map_form_layout->addRow("Filtered Cloud Topic:", depth_filtered_cloud_topic_field_);

  // Filtered Cloud Topic
  depth_max_update_rate_field_ = new QLineEdit(this);
  depth_max_update_rate_field_->setMaximumWidth(400);
  depth_map_form_layout->addRow("Max Update Rate:", depth_max_update_rate_field_);

  depth_map_group_->setLayout(depth_map_form_layout);
  layout->addWidget(depth_map_group_);

  layout->setAlignment(Qt::AlignTop);

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

// ******************************************************************************************
// Received when this widget is chosen from the navigation menu
// ******************************************************************************************
void PerceptionWidget::focusGiven()
{
  loadSensorPluginsComboBox();
}

// ******************************************************************************************
// Received when another widget is chosen from the navigation menu
// ******************************************************************************************
bool PerceptionWidget::focusLost()
{
  // Save the sensor plugin configuration to sensors_plugin_config data structure
  if (sensor_plugin_field_->currentIndex() == 1)
  {
    // Point Cloud plugin feilds
    config_data_->addGenericParameterToSensorPluginConfig("sensor_plugin", "occupancy_map_monitor/"
                                                                           "PointCloudOctomapUpdater");
    config_data_->addGenericParameterToSensorPluginConfig("point_cloud_topic",
                                                          point_cloud_topic_field_->text().trimmed().toStdString());
    config_data_->addGenericParameterToSensorPluginConfig("max_range",
                                                          max_range_field_->text().trimmed().toStdString());
    config_data_->addGenericParameterToSensorPluginConfig("point_subsample",
                                                          point_subsample_field_->text().trimmed().toStdString());
    config_data_->addGenericParameterToSensorPluginConfig("padding_offset",
                                                          padding_offset_field_->text().trimmed().toStdString());
    config_data_->addGenericParameterToSensorPluginConfig("padding_scale",
                                                          padding_scale_field_->text().trimmed().toStdString());
    config_data_->addGenericParameterToSensorPluginConfig("max_update_rate",
                                                          max_update_rate_field_->text().trimmed().toStdString());
    config_data_->addGenericParameterToSensorPluginConfig("filtered_cloud_topic",
                                                          filtered_cloud_topic_field_->text().trimmed().toStdString());

    config_data_->changes |= MoveItConfigData::SENSORS_CONFIG;
  }
  else if (sensor_plugin_field_->currentIndex() == 2)
  {
    // Depth Map plugin feilds
    config_data_->addGenericParameterToSensorPluginConfig("sensor_plugin", "occupancy_map_monitor/"
                                                                           "DepthImageOctomapUpdater");
    config_data_->addGenericParameterToSensorPluginConfig("image_topic",
                                                          image_topic_field_->text().trimmed().toStdString());
    config_data_->addGenericParameterToSensorPluginConfig("queue_size",
                                                          queue_size_field_->text().trimmed().toStdString());
    config_data_->addGenericParameterToSensorPluginConfig("near_clipping_plane_distance",
                                                          near_clipping_field_->text().trimmed().toStdString());
    config_data_->addGenericParameterToSensorPluginConfig("far_clipping_plane_distance",
                                                          far_clipping_field_->text().trimmed().toStdString());
    config_data_->addGenericParameterToSensorPluginConfig("shadow_threshold",
                                                          shadow_threshold_field_->text().trimmed().toStdString());
    config_data_->addGenericParameterToSensorPluginConfig("padding_scale",
                                                          depth_padding_scale_field_->text().trimmed().toStdString());
    config_data_->addGenericParameterToSensorPluginConfig("padding_offset",
                                                          depth_padding_offset_field_->text().trimmed().toStdString());
    config_data_->addGenericParameterToSensorPluginConfig(
        "filtered_cloud_topic", depth_filtered_cloud_topic_field_->text().trimmed().toStdString());
    config_data_->addGenericParameterToSensorPluginConfig("max_update_rate",
                                                          depth_max_update_rate_field_->text().trimmed().toStdString());

    config_data_->changes |= MoveItConfigData::SENSORS_CONFIG;
  }
  else
  {
    // Clear the sensors_plugin_config data structure
    config_data_->clearSensorPluginConfig();
    config_data_->changes ^= MoveItConfigData::SENSORS_CONFIG;
  }
  return true;
}

void PerceptionWidget::sensorPluginChanged(int index)
{
  if (index == 1)
  {
    // Point cloud form visible, depth map form invisible
    point_cloud_group_->setVisible(true);
    depth_map_group_->setVisible(false);
  }
  else if (index == 2)
  {
    // Depth map form visible, point cloud form invisible
    point_cloud_group_->setVisible(false);
    depth_map_group_->setVisible(true);
  }
  else
  {
    // All forms invisible
    point_cloud_group_->setVisible(false);
    depth_map_group_->setVisible(false);
  }
}

void PerceptionWidget::loadSensorPluginsComboBox()
{
  // Only load this combo box once
  static bool hasLoaded = false;
  if (hasLoaded)
    return;
  hasLoaded = true;

  // Remove all old items
  sensor_plugin_field_->clear();

  // Add None option, the default
  sensor_plugin_field_->addItem("None");

  // Add the two avilable plugins to combo box
  sensor_plugin_field_->addItem("Point Cloud");
  sensor_plugin_field_->addItem("Depth Map");

  // Load deafult config, or use the one in the config package if exists
  std::vector<std::map<std::string, GenericParameter> > sensors_vec_map = config_data_->getSensorPluginConfig();
  for (std::size_t i = 0; i < sensors_vec_map.size(); ++i)
  {
    if (sensors_vec_map[i]["sensor_plugin"].getValue() == std::string("occupancy_map_monitor/PointCloudOctomapUpdater"))
    {
      sensor_plugin_field_->setCurrentIndex(1);
      point_cloud_topic_field_->setText(QString(sensors_vec_map[i]["point_cloud_topic"].getValue().c_str()));
      max_range_field_->setText(QString(sensors_vec_map[i]["max_range"].getValue().c_str()));
      point_subsample_field_->setText(QString(sensors_vec_map[i]["point_subsample"].getValue().c_str()));
      padding_offset_field_->setText(QString(sensors_vec_map[i]["padding_offset"].getValue().c_str()));
      padding_scale_field_->setText(QString(sensors_vec_map[i]["padding_scale"].getValue().c_str()));
      max_update_rate_field_->setText(QString(sensors_vec_map[i]["max_update_rate"].getValue().c_str()));
      filtered_cloud_topic_field_->setText(QString(sensors_vec_map[i]["filtered_cloud_topic"].getValue().c_str()));
    }
    else if (sensors_vec_map[i]["sensor_plugin"].getValue() ==
             std::string("occupancy_map_monitor/DepthImageOctomapUpdater"))
    {
      sensor_plugin_field_->setCurrentIndex(2);
      image_topic_field_->setText(QString(sensors_vec_map[i]["image_topic"].getValue().c_str()));
      queue_size_field_->setText(QString(sensors_vec_map[i]["queue_size"].getValue().c_str()));
      near_clipping_field_->setText(QString(sensors_vec_map[i]["near_clipping_plane_distance"].getValue().c_str()));
      far_clipping_field_->setText(QString(sensors_vec_map[i]["far_clipping_plane_distance"].getValue().c_str()));
      shadow_threshold_field_->setText(QString(sensors_vec_map[i]["shadow_threshold"].getValue().c_str()));
      depth_padding_scale_field_->setText(QString(sensors_vec_map[i]["padding_scale"].getValue().c_str()));
      depth_padding_offset_field_->setText(QString(sensors_vec_map[i]["padding_offset"].getValue().c_str()));
      depth_filtered_cloud_topic_field_->setText(
          QString(sensors_vec_map[i]["filtered_cloud_topic"].getValue().c_str()));
      depth_max_update_rate_field_->setText(QString(sensors_vec_map[i]["max_update_rate"].getValue().c_str()));
    }
  }

  // If no sensor config exists, default to None
  if (sensors_vec_map.size() == 2)
  {
    sensor_plugin_field_->setCurrentIndex(0);
  }
}

}  // namespace
