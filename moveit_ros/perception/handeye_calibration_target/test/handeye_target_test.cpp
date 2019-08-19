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

#include <fstream>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <opencv2/core/core.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/CameraInfo.h>
#include <pluginlib/class_loader.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/handeye_calibration_target/handeye_target_base.h>

class MoveItHandEyeTargetTester : public ::testing::Test
{
protected:
  void SetUp() override
  {
    try
    {
      target_plugins_loader_.reset(new pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeTargetBase>(
          "moveit_ros_perception", "moveit_handeye_calibration::HandEyeTargetBase"));
      target_ = target_plugins_loader_->createUniqueInstance("HandEyeTarget/Aruco");
      target_->initialize(4, 3, 200, 20, 1, "DICT_4X4_250", 0.0256, 0.0066);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Exception while creating handeye target plugin: " << ex.what());
      return;
    }

    std::string image_path = ros::package::getPath("moveit_ros_perception") +
                             "/handeye_calibration_target/test/test_aruco_marker_detection.png";

    image_ = cv::imread(image_path, cv::IMREAD_COLOR);

    resource_ok_ = false;
    if (!image_.data)
      ROS_ERROR_STREAM("Could not open or find the image file: " << image_path);
    else
      resource_ok_ = true;
  }

  void TearDown() override
  {
    target_.reset();
    target_plugins_loader_.reset();
  }

protected:
  pluginlib::UniquePtr<moveit_handeye_calibration::HandEyeTargetBase> target_;
  std::unique_ptr<pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeTargetBase> > target_plugins_loader_;
  bool resource_ok_;
  cv::Mat image_;
};

TEST_F(MoveItHandEyeTargetTester, InitOK)
{
  ASSERT_TRUE(resource_ok_);
  ASSERT_EQ(image_.cols, 640);
  ASSERT_EQ(image_.rows, 480);
  ASSERT_TRUE(target_);
  ASSERT_EQ(target_->getDictionaryIds().size(), 5);
}

TEST_F(MoveItHandEyeTargetTester, DetectArucoMarkerPose)
{
  // Set camera intrinsic parameters
  sensor_msgs::CameraInfoPtr camera_info(new sensor_msgs::CameraInfo());
  camera_info->height = 480;
  camera_info->width = 640;
  camera_info->header.frame_id = "camera_color_optical_frame";
  camera_info->distortion_model = "plumb_bob";
  camera_info->D = std::vector<double>{ 0.0, 0.0, 0.0, 0.0, 0.0 };
  camera_info->K = boost::array<double, 9>{
    618.6002197265625, 0.0, 321.9837646484375, 0.0, 619.1103515625, 241.1459197998047, 0.0, 0.0, 1.0
  };
  camera_info->R = boost::array<double, 9>{ 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
  camera_info->P = boost::array<double, 12>{
    618.6002197265625, 0.0, 321.9837646484375, 0.0, 0.0, 619.1103515625, 241.1459197998047, 0.0, 0.0, 0.0, 1.0, 0.0
  };
  ASSERT_TRUE(target_->setCameraIntrinsicParams(camera_info));

  // Check target image creation
  cv::Mat target_image;
  ASSERT_TRUE(target_->createTargetImage(target_image));

  // Get target pose
  cv::Mat gray_image;
  cv::cvtColor(image_, gray_image, cv::COLOR_RGB2GRAY);
  ASSERT_TRUE(target_->detectTargetPose(gray_image));

  // Get translation and rotation part
  geometry_msgs::TransformStamped camera_transform;
  ros::Time::init();
  camera_transform = target_->getTransformStamped(camera_info->header.frame_id);
  Eigen::Affine3d ret = tf2::transformToEigen(camera_transform);
  Eigen::Vector3d t(0.014898, 0.012310, 0.58609);
  Eigen::Vector3d r(2.12328, -1.50481, -1.29729);
  ASSERT_TRUE(ret.translation().isApprox(t, 0.01));
  ASSERT_TRUE(ret.rotation().eulerAngles(0, 1, 2).isApprox(r, 0.01));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}