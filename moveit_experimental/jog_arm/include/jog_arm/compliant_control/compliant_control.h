///////////////////////////////////////////////////////////////////////////////
//      Title     : compliant_control.h
//      Project   : compliant_control
//      Created   : 9/27/2017
//      Author    : Nitish Sharma
//
// BSD 3-Clause License
//
// Copyright (c) 2018, Los Alamos National Security, LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef COMPLIANT_CONTROL_H
#define COMPLIANT_CONTROL_H

/**
 * compliant control class. Allows you to control each dimension with a
 * compliant constant.
 * The key equation for each dimension is compliance_velocity[i] =
 * wrench[i]/stiffness[i]
 */

#include <float.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <string>
#include <vector>

namespace compliantEnum
{
/**
 * dimension enum.
 */
enum dimension
{
  NUM_DIMS = 6 /**< 3 translational, 3 rotational dimensions. */
};

/**
 * exitCondition enum.
 * The exitCondition enum is used to know the condition of the controller in
 * the end.
 */
enum exitCondition
{
  NOT_CONTROLLED = 0,    /**< None of the dimension is set to be controlled. */
  FT_VIOLATION = 1,      /**< Force or torque was read as maximum allowable. */
  CONDITION_MET = 2,     /**< One of the compliant conditions is met. */
  CONDITION_NOT_MET = 3, /**< No violation or condition. */
  POSE_ACHIEVED = 4      /**< The target pose was reached within tolerances. */
};                       /**< The number of return conditions. */
}
namespace compliant_control
{
class CompliantControl;
class LowPassFilter;

class CompliantControl
{
public:
  // Constructor.
  CompliantControl(std::vector<double> stiffness, std::vector<double> deadband, std::vector<double> endConditionWrench,
                   double filterParam, geometry_msgs::WrenchStamped bias, double highestAllowableForce,
                   double highestAllowableTorque);

  // Set the "springiness" of compliance in each direction.
  void setStiffness(std::vector<double> stiffness);

  // Exit when the given force/torque wrench is achieved in any direction
  void setEndCondition(std::vector<double> endConditionWrench);

  // Update member variables with current, filtered forces/torques
  void getFT(geometry_msgs::WrenchStamped ftData);

  // Set the "springiness" of compliance in each direction
  void adjustStiffness(compliantEnum::dimension dim, double stiffness);

  // Update FT values
  void dataCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  // Bias the FT values
  void biasSensor(geometry_msgs::WrenchStamped bias);

  // Set the target FT wrench
  compliantEnum::exitCondition getVelocity(std::vector<double> vIn, geometry_msgs::WrenchStamped ftData,
                                           std::vector<double>& vOut);

  /**
   * Set the topic that force/torque data is read from.
   * @param ftTop      The force/torque data topic.
   */
  void setFTTopic(std::string ftTop);

  /**
   * Set the topic to output velocity commands to.
   * @param velTopic    The velocity jog command topic.
   */
  void setVelTopic(std::string velTop);

  std::vector<double> stiffness_;
  std::vector<double> deadband_;
  std::vector<double> end_condition_wrench_;
  std::vector<double> ft_;
  std::vector<double> bias_;                 // Initial biased force
  double safeForceLimit_, safeTorqueLimit_;  // Quit if these forces/torques are exceeded
  std::vector<compliant_control::LowPassFilter> vectorOfFilters_;

private:
};

class LowPassFilter
{
public:
  LowPassFilter(double filterParam);
  double filter(const double& new_msrmt);

  // Related to the cutoff frequency of the filter.
  // filterParam=1 results in a cutoff at 1/4 of the sampling rate.
  // See bitbucket.org/AndyZe/pid for slightly more sophistication.
  // Larger filterParam --> trust the filtered data more, trust the measurements
  // less.
  double filterParam_ = 4.;

  void reset(double data);

private:
  std::vector<double> prev_msrmts_ = { 0., 0., 0. };
  std::vector<double> prev_filtered_msrmts_ = { 0., 0. };
};
}
#endif