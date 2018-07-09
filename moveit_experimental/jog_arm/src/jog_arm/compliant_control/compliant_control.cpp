///////////////////////////////////////////////////////////////////////////////
//      Title     : compliant_control.cpp
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
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#include "jog_arm/compliant_control.h"

namespace compliant_control {
CompliantControl::CompliantControl(
    const std::vector<double> &stiffness, const std::vector<double> &deadband,
    const std::vector<double> &end_condition_wrench, double filter_param,
    geometry_msgs::WrenchStamped bias, double highest_allowable_force,
    double highest_allowable_torque)
    : stiffness_(stiffness), deadband_(deadband),
      end_condition_wrench_(end_condition_wrench),
      safe_force_limit_(highest_allowable_force),
      safe_torque_limit_(highest_allowable_torque) {
  bias_.resize(compliant_control::NUM_DIMS);
  ft_.resize(compliant_control::NUM_DIMS);

  for (int i = 0; i < compliant_control::NUM_DIMS; i++)
    vectorOfFilters_.push_back(LowPassFilter(filter_param));

  biasSensor(bias);
  ft_ = bias_;
}

// Tare or bias the wrench readings -- i.e. reset its ground truth
void CompliantControl::biasSensor(const geometry_msgs::WrenchStamped &bias) {
  bias_[0] = bias.wrench.force.x;
  bias_[1] = bias.wrench.force.y;
  bias_[2] = bias.wrench.force.z;
  bias_[3] = bias.wrench.torque.x;
  bias_[4] = bias.wrench.torque.y;
  bias_[5] = bias.wrench.torque.z;

  for (int i = 0; i < compliant_control::NUM_DIMS; i++) {
    vectorOfFilters_[i].reset(0.);
  }
}

void CompliantControl::setStiffness(const std::vector<double> &stiffness) {
  if (stiffness.size() != compliant_control::NUM_DIMS) {
    ROS_ERROR_NAMED("compliant_control", "Invalid stiffness vector: ");
  } else {
    for (int i = 0; i < compliant_control::NUM_DIMS; i++) {
      if (fabs(stiffness[i]) <= 1e-3) {
        ROS_ERROR_STREAM_NAMED("compliant_control",
                               "Stiffness must be non-zero.Ignoring "
                               "compliance in direction: "
                                   << i);
        stiffness_[i] = DBL_MAX;
      } else {
        stiffness_[i] = stiffness[i];
      }
    }
  }
}

void CompliantControl::setEndCondition(
    const std::vector<double> &end_condition_wrench) {
  if (end_condition_wrench.size() != compliant_control::NUM_DIMS) {
    ROS_ERROR_NAMED("compliant_control",
                    "Invalid vector end_condition_wrench: ");
  } else {
    for (int i = 0; i < compliant_control::NUM_DIMS; i++) {
      end_condition_wrench_[i] = end_condition_wrench[i];
    }
  }
}

void CompliantControl::getForceTorque(
    geometry_msgs::WrenchStamped force_torque_data) {
  std::vector<double> biasedFT(6, 0.);

  // Apply the deadband
  if (fabs(force_torque_data.wrench.force.x - bias_[0]) < fabs(deadband_[0]))
    biasedFT[0] = 0.;
  else
    biasedFT[0] = force_torque_data.wrench.force.x - bias_[0];
  if (fabs(force_torque_data.wrench.force.y - bias_[1]) < fabs(deadband_[1]))
    biasedFT[1] = 0.;
  else
    biasedFT[1] = force_torque_data.wrench.force.y - bias_[1];
  if (fabs(force_torque_data.wrench.force.z - bias_[2]) < fabs(deadband_[2]))
    biasedFT[2] = 0.;
  else
    biasedFT[2] = force_torque_data.wrench.force.z - bias_[2];

  if (fabs(force_torque_data.wrench.torque.x - bias_[3]) < fabs(deadband_[3]))
    biasedFT[3] = 0.;
  else
    biasedFT[3] = force_torque_data.wrench.torque.x - bias_[3];
  if (fabs(force_torque_data.wrench.torque.y - bias_[4]) < fabs(deadband_[4]))
    biasedFT[4] = 0.;
  else
    biasedFT[4] = force_torque_data.wrench.torque.y - bias_[4];
  if (fabs(force_torque_data.wrench.torque.z - bias_[5]) < fabs(deadband_[5]))
    biasedFT[5] = 0.;
  else
    biasedFT[5] = force_torque_data.wrench.torque.x - bias_[5];

  ft_[0] = vectorOfFilters_[0].filter(biasedFT[0]);
  ft_[1] = vectorOfFilters_[1].filter(biasedFT[1]);
  ft_[2] = vectorOfFilters_[2].filter(biasedFT[2]);
  ft_[3] = vectorOfFilters_[3].filter(biasedFT[3]);
  ft_[4] = vectorOfFilters_[4].filter(biasedFT[4]);
  ft_[5] = vectorOfFilters_[5].filter(biasedFT[5]);
}

compliant_control::exitCondition
CompliantControl::getVelocity(std::vector<double> v_in,
                              geometry_msgs::WrenchStamped force_torque_data,
                              std::vector<double> &vOut) {
  compliant_control::exitCondition exitCondition =
      compliant_control::NOT_CONTROLLED;
  getForceTorque(force_torque_data);

  if (((fabs(ft_[0]) + fabs(ft_[1]) + fabs(ft_[2])) >= safe_force_limit_) ||
      ((fabs(ft_[3]) + fabs(ft_[4]) + fabs(ft_[5])) >= safe_torque_limit_)) {
    ROS_ERROR_NAMED(
        "compliant_control",
        "Total force or torque exceeds safety limits. Stopping motion.");
    vOut = std::vector<double>(6, 0.0);
    return compliant_control::FT_VIOLATION;
  }

  for (int i = 0; i < compliant_control::NUM_DIMS; i++) {
    if (end_condition_wrench_[i] > 0) {
      if (ft_[i] > end_condition_wrench_[i]) {
        ROS_INFO_STREAM_NAMED("compliant_control",
                              "Exit condition met in direction: " << i);
        vOut[i] = 0.0;
        exitCondition = compliant_control::CONDITION_MET;
      } else {
        vOut[i] = v_in[i] + ft_[i] / stiffness_[i];
        if (exitCondition != compliant_control::CONDITION_MET) {
          exitCondition = compliant_control::CONDITION_NOT_MET;
        }
      }
    } else // end_condition_wrench_[i]<=0
    {
      if (ft_[i] < end_condition_wrench_[i]) {
        ROS_INFO_STREAM_NAMED("compliant_control",
                              "Exit condition met in direction: " << i);
        vOut[i] = 0.0;
        exitCondition = compliant_control::CONDITION_MET;
      } else {
        vOut[i] = v_in[i] + ft_[i] / stiffness_[i];
        if (exitCondition != compliant_control::CONDITION_MET) {
          exitCondition = compliant_control::CONDITION_NOT_MET;
        }
      }
    }
  }
  return exitCondition;
}

LowPassFilter::LowPassFilter(const double filter_param)
    : filter_param_(filter_param) {}

double LowPassFilter::filter(const double new_msrmt) {
  // Push in the new measurement
  prev_msrmts_[2] = prev_msrmts_[1];
  prev_msrmts_[1] = prev_msrmts_[0];
  prev_msrmts_[0] = new_msrmt;

  double new_filtered_msrmt =
      (1 / (1 + filter_param_ * filter_param_ + 1.414 * filter_param_)) *
      (prev_msrmts_[2] + 2 * prev_msrmts_[1] + prev_msrmts_[0] -
       (filter_param_ * filter_param_ - 1.414 * filter_param_ + 1) *
           prev_filtered_msrmts_[1] -
       (-2 * filter_param_ * filter_param_ + 2) * prev_filtered_msrmts_[0]);
  ;

  // Store the new filtered measurement
  prev_filtered_msrmts_[1] = prev_filtered_msrmts_[0];
  prev_filtered_msrmts_[0] = new_filtered_msrmt;

  return new_filtered_msrmt;
}

void LowPassFilter::reset(double data) {
  prev_msrmts_ = {data, data, data};
  prev_filtered_msrmts_ = {data, data};
}
} // end namespace compliant_control
