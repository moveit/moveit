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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Mohamad Ayman */
#include <gtest/gtest.h>
#include <sstream>
#include <ctype.h>
#include <iostream>  // For writing yaml and launch files
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <string>
#include <boost/filesystem/path.hpp>
#include <moveit_resources/config.h>
#include <boost/filesystem.hpp>  // for creating/deleting folders/files
#include <moveit/setup_assistant/tools/moveit_config_data.h>

// This tests writing/parsing of ros_controller.yaml
class MoveItConfigData : public testing::Test
{
protected:
  void SetUp() override
  {
    boost::filesystem::path res_path(MOVEIT_TEST_RESOURCES_DIR);

    srdf_model.reset(new srdf::Model());
    std::string xml_string;
    std::fstream xml_file((res_path / "panda_description/urdf/panda.urdf").string().c_str(), std::fstream::in);
    if (xml_file.is_open())
    {
      while (xml_file.good())
      {
        std::string line;
        std::getline(xml_file, line);
        xml_string += (line + "\n");
      }
      xml_file.close();
      urdf_model = urdf::parseURDF(xml_string);
    }
    srdf_model->initFile(*urdf_model, (res_path / "panda_moveit_config/config/panda.srdf").string());
    robot_model.reset(new moveit::core::RobotModel(urdf_model, srdf_model));
  };

protected:
  urdf::ModelInterfaceSharedPtr urdf_model;
  srdf::ModelSharedPtr srdf_model;
  moveit::core::RobotModelPtr robot_model;
};

TEST_F(MoveItConfigData, ReadingControllers)
{
  boost::filesystem::path res_path(MOVEIT_TEST_RESOURCES_DIR);

  // Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;

  config_data_.reset(new moveit_setup_assistant::MoveItConfigData());
  config_data_->srdf_->srdf_model_ = srdf_model;
  config_data_->setRobotModel(robot_model);

  // Initially no controllers
  EXPECT_EQ(config_data_->getROSControllers().size(), 0u);

  // Adding default controllers, a controller for each planning group
  config_data_->addDefaultControllers();

  // Number of the planning groups defined in the model srdf
  size_t group_count = config_data_->srdf_->srdf_model_->getGroups().size();

  // Test that addDefaultControllers() did accually add a controller for the new_group
  EXPECT_EQ(config_data_->getROSControllers().size(), group_count);

  // Temporary file used during the test and is deleted when the test is finished
  char test_file[] = "/tmp/msa_unittest_ros_controller.yaml";

  // ros_controller.yaml written correctly
  EXPECT_EQ(config_data_->outputROSControllersYAML(test_file), true);

  // Reset MoveIt! config MoveItConfigData
  config_data_.reset(new moveit_setup_assistant::MoveItConfigData());

  // Initially no controllers
  EXPECT_EQ(config_data_->getROSControllers().size(), 0u);

  // ros_controllers.yaml read correctly
  EXPECT_EQ(config_data_->inputROSControllersYAML(test_file), true);

  // ros_controllers.yaml parsed correctly
  EXPECT_EQ(config_data_->getROSControllers().size(), group_count);

  // Remove ros_controllers.yaml temp file which was used in testing
  boost::filesystem::remove(test_file);
}

// This tests parsing of sensors_rgbd.yaml
TEST_F(MoveItConfigData, ReadingSensorsConfig)
{
  // Contains all the config data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;
  config_data_.reset(new moveit_setup_assistant::MoveItConfigData());

  boost::filesystem::path setup_assistant_path(config_data_->setup_assistant_path_);

  // Before parsing, no config available
  EXPECT_EQ(config_data_->getSensorPluginConfig().size(), 0u);

  // Read the file containing the default config parameters
  config_data_->input3DSensorsYAML((setup_assistant_path / "resources/default_config/sensors_3d.yaml").string());

  // Default config for the two available sensor plugins
  // Make sure both are parsed correctly
  EXPECT_EQ(config_data_->getSensorPluginConfig().size(), 2u);

  EXPECT_EQ(config_data_->getSensorPluginConfig()[0]["sensor_plugin"].getValue(),
            std::string("occupancy_map_monitor/PointCloudOctomapUpdater"));

  EXPECT_EQ(config_data_->getSensorPluginConfig()[1]["sensor_plugin"].getValue(),
            std::string("occupancy_map_monitor/DepthImageOctomapUpdater"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
