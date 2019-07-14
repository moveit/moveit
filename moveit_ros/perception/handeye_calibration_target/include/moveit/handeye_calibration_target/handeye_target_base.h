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
 *   * Neither the name of Intel nor the names of its
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

#pragma once

#include <mutex>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>

namespace moveit_handeye_calibration
{
/**
 * @class HandEyeTargetBase
 * @brief Provides an interface for handeye calibration target detectors.
 * A target used for handeye calibration is usually a 2D board that consists of an array of markers.
 * The markers can be circles, rectangles or their combinations.
 */
class HandEyeTargetBase
{
public:
  const std::string LOGNAME = "handeye_target_base";
  const std::size_t CAMERA_MATRIX_VECTOR_DIMENSION = 9;  // 3x3 camera intrinsic matrix
  const std::size_t CAMERA_MATRIX_WIDTH = 3;
  const std::size_t CAMERA_MATRIX_HEIGHT = 3;
  const std::size_t CAMERA_DISTORTION_VECTOR_DIMENSION = 5;  // distortion parameters (k1, k2, t1, t2, k3)

  virtual ~HandEyeTargetBase() = default;
  HandEyeTargetBase()
  {
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    distortion_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
  }

  /**
   * @brief Initialization function for the handeye target pose detecting, for use with marker array based target
   * detectors.
   * @param markers_x Number of markers along X axis.
   * @param markers_y Number of markers along Y axis.
   * @param marker_size The width or radius of a marker in a 2D target image (pixels).
   * @param separation The distance between two neighbouring markers in a 2D target image (pixels).
   * @param border_bits The distance from the markers to the target boarder (bits).
   * @param dictionary_id Marker dictionary id.
   * @param marker_measured_size Marker size measured from the printed image.
   * @param marker_measured_separation Marker separation distance measured from the printed image.
   * @return True if initialization was successful, false otherwise.
   */
  virtual bool initialize(int markers_x, int markers_y, int marker_size_, int separation, int border_bits,
                          const std::string& dictionary_id, double marker_measured_size,
                          double marker_measured_separation) = 0;

  /**
   * @brief Set the target intrinsic parameters, for updating the target intrinsic parameters.
   * @param markers_x Number of markers along X axis.
   * @param markers_y Number of markers along Y axis.
   * @param marker_size_ The width or radius of a marker in a 2D target image (pixels).
   * @param separation The distance between two neighbouring markers in a 2D target image (pixels).
   * @param border_bits The margins between the markers and the target image boarder (bits).
   * @param dictionary_id Marker dictionary id.
   * @return True if parameters are correctly set, false otherwise.
   */
  virtual bool setTargetIntrinsicParams(int markers_x, int markers_y, int marker_size_, int separation, int border_bits,
                                        const std::string& dictionary_id) = 0;

  /**
   * @brief Set the parameters of printed target, for updating the real target parameters.
   * @param marker_measured_size Printed marker size.
   * @param marker_measured_separation Printed marker separation distance.
   * @return True if parameters are correctly set, false otherwise.
   */
  virtual bool setTargetDimension(double marker_measured_size, double marker_measured_separation) = 0;

  /**
   * @brief Create an target image, so that the target can be viewed and printed.
   * @param image Use for storing the created image.
   * @return True if no errors happen, false otherwise.
   */
  virtual bool createTargetImage(cv::Mat& image) const = 0;

  /**
   * @brief Given an image containing a target captured from a camera view point, get the target pose with respect to
   * the camera optical frame. Target parameters and camera intrinsic parameters should be correctly set
   * before calling this function.
   * @param image Input image, assume a grayscale image.
   * @return True if no errors happen, false otherwise.
   */
  virtual bool detectTargetPose(cv::Mat& image) = 0;

  /**
   * @brief Get available target dictionary ids that can be used to fill the dropdown list of the handeye calibration
   * GUI component.
   * @return A vector of dictinary names.
   */
  virtual std::vector<std::string> getDictionaryIds() const = 0;

  /**
   * @brief Get `TransformStamped` message from the target detection result, use for TF publish.
   * @param frame_id The name of the frame this transform is with respect to.
   * @return A `TransformStamped` message.
   */
  virtual geometry_msgs::TransformStamped getTransformStamped(const std::string& frame_id) const = 0;

  /**
   * @brief Set camera intrinsic parameters, e.g. camera intrinsic matrix and distortion coefficients.
   * @param msg Input camera info message.
   * @return Ture if the input camera info format is correct, false otherwise.
   */
  virtual bool setCameraIntrinsicParams(const sensor_msgs::CameraInfoPtr& msg)
  {
    if (!msg)
    {
      ROS_ERROR_NAMED(LOGNAME, "CameraInfo msg is NULL.");
      return false;
    }

    if (msg->K.size() != CAMERA_MATRIX_VECTOR_DIMENSION)
    {
      ROS_ERROR_NAMED(LOGNAME, "Invalid camera matrix dimension, current is %ld, required is %zu.", msg->K.size(),
                      CAMERA_MATRIX_VECTOR_DIMENSION);
      return false;
    }

    if (msg->D.size() != CAMERA_DISTORTION_VECTOR_DIMENSION)
    {
      ROS_ERROR_NAMED(LOGNAME, "Invalid distortion parameters dimension, current is %ld, required is %zu.",
                      msg->D.size(), CAMERA_DISTORTION_VECTOR_DIMENSION);
      return false;
    }

    std::lock_guard<std::mutex> base_lock(base_mutex_);

    // Store camera matrix info
    for (size_t i = 0; i < CAMERA_MATRIX_WIDTH; i++)
    {
      for (size_t j = 0; j < CAMERA_MATRIX_HEIGHT; j++)
      {
        camera_matrix_.at<double>(i, j) = msg->K[i * CAMERA_MATRIX_WIDTH + j];
      }
    }

    // Store camera distortion info
    for (size_t i = 0; i < CAMERA_DISTORTION_VECTOR_DIMENSION; i++)
    {
      distortion_coeffs_.at<double>(i, 0) = msg->D[i];
    }

    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Set camera intrinsic parameter to: " << *msg);
    return true;
  }

protected:
  // 3x3 floating-point camera matrix
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  cv::Mat camera_matrix_;

  // Vector of distortion coefficients (k1, k2, t1, t2, k3)
  // Assume `plumb_bob` model
  cv::Mat distortion_coeffs_;

  // flag to indicate if target parameter values are correctly defined
  bool target_params_ready_;

  std::mutex base_mutex_;
};
}  // namespace moveit_handeye_calibration