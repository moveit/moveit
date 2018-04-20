
#include "jog_arm/compliant_control/compliant_control.h"

namespace compliant_control {

CompliantControl::CompliantControl(std::vector<double> stiffness,
                                   std::vector<double> deadband,
                                   std::vector<double> endConditionWrench,
                                   double filterParam,
                                   geometry_msgs::WrenchStamped bias,
                                   double highestAllowableForce,
                                   double highestAllowableTorque)
    : stiffness_(stiffness), deadband_(deadband),
      end_condition_wrench_(endConditionWrench),
      safeForceLimit_(highestAllowableForce),
      safeTorqueLimit_(highestAllowableTorque) {
  bias_.resize(compliantEnum::NUM_DIMS);
  ft_.resize(compliantEnum::NUM_DIMS);

  for (int i = 0; i < compliantEnum::NUM_DIMS; i++)
    vectorOfFilters_.push_back(LowPassFilter(filterParam));

  bias_[0] = bias.wrench.force.x;
  bias_[1] = bias.wrench.force.y;
  bias_[2] = bias.wrench.force.z;
  bias_[3] = bias.wrench.torque.x;
  bias_[4] = bias.wrench.torque.y;
  bias_[5] = bias.wrench.torque.z;
  ft_ = bias_;
}

// Tare or bias the wrench readings -- i.e. reset its ground truth
void CompliantControl::biasSensor(geometry_msgs::WrenchStamped bias) {
  bias_[0] = bias.wrench.force.x;
  bias_[1] = bias.wrench.force.y;
  bias_[2] = bias.wrench.force.z;
  bias_[3] = bias.wrench.torque.x;
  bias_[4] = bias.wrench.torque.y;
  bias_[5] = bias.wrench.torque.z;

  for (int i = 0; i < compliantEnum::NUM_DIMS; i++) {
    vectorOfFilters_[i].reset(0.);
  }
}

void CompliantControl::setStiffness(std::vector<double> b) {
  if (b.size() != compliantEnum::NUM_DIMS) {
    ROS_ERROR_NAMED("compliant_control", "Invalid stiffness vector: ");
  } else {
    for (int i = 0; i < compliantEnum::NUM_DIMS; i++) {
      if (fabs(b[i]) <= 1e-3) {
        ROS_ERROR_STREAM_NAMED("compliant_control",
                               "Stiffness must be non-zero.Ignoring "
                               "Compliance in direction: "
                                   << i);
        stiffness_[i] = DBL_MAX;
      } else {
        stiffness_[i] = b[i];
      }
    }
  }
}

void CompliantControl::setEndCondition(std::vector<double> endConditionWrench) {
  if (endConditionWrench.size() != compliantEnum::NUM_DIMS) {
    ROS_ERROR_NAMED("compliant_control", "Invalid vector endConditionWrench: ");
  } else {
    for (int i = 0; i < compliantEnum::NUM_DIMS; i++) {
      end_condition_wrench_[i] = endConditionWrench[i];
    }
  }
}

void CompliantControl::getFT(geometry_msgs::WrenchStamped ftData) {

  std::vector<double> biasedFT(6, 0.);

  // Apply the deadband
  if (fabs(ftData.wrench.force.x - bias_[0]) < fabs(deadband_[0]))
    biasedFT[0] = 0.;
  else
    biasedFT[0] = ftData.wrench.force.x - bias_[0];
  if (fabs(ftData.wrench.force.y - bias_[1]) < fabs(deadband_[1]))
    biasedFT[1] = 0.;
  else
    biasedFT[1] = ftData.wrench.force.y - bias_[1];
  if (fabs(ftData.wrench.force.z - bias_[2]) < fabs(deadband_[2]))
    biasedFT[2] = 0.;
  else
    biasedFT[2] = ftData.wrench.force.z - bias_[2];

  if (fabs(ftData.wrench.torque.x - bias_[3]) < fabs(deadband_[3]))
    biasedFT[3] = 0.;
  else
    biasedFT[3] = ftData.wrench.torque.x - bias_[3];
  if (fabs(ftData.wrench.torque.y - bias_[4]) < fabs(deadband_[4]))
    biasedFT[4] = 0.;
  else
    biasedFT[4] = ftData.wrench.torque.y - bias_[4];
  if (fabs(ftData.wrench.torque.z - bias_[5]) < fabs(deadband_[5]))
    biasedFT[5] = 0.;
  else
    biasedFT[5] = ftData.wrench.torque.x - bias_[5];

  ft_[0] = vectorOfFilters_[0].filter(biasedFT[0]);
  ft_[1] = vectorOfFilters_[1].filter(biasedFT[1]);
  ft_[2] = vectorOfFilters_[2].filter(biasedFT[2]);
  ft_[3] = vectorOfFilters_[3].filter(biasedFT[3]);
  ft_[4] = vectorOfFilters_[4].filter(biasedFT[4]);
  ft_[5] = vectorOfFilters_[5].filter(biasedFT[5]);
}

compliantEnum::exitCondition
CompliantControl::getVelocity(std::vector<double> vIn,
                              geometry_msgs::WrenchStamped ftData,
                              std::vector<double> &vOut) {
  compliantEnum::exitCondition exitCondition = compliantEnum::NOT_CONTROLLED;
  getFT(ftData);

  if (((fabs(ft_[0]) + fabs(ft_[1]) + fabs(ft_[2])) >= safeForceLimit_) ||
      ((fabs(ft_[3]) + fabs(ft_[4]) + fabs(ft_[5])))) {
    ROS_ERROR_NAMED(
        "compliant_control",
        "Total force or torque exceeds safety limits. Stopping motion.");
    vOut = std::vector<double>(6, 0.0);
    return compliantEnum::FT_VIOLATION;
  }

  for (int i = 0; i < compliantEnum::NUM_DIMS; i++) {
    if (end_condition_wrench_[i] > 0) {
      if (ft_[i] > end_condition_wrench_[i]) {
        ROS_INFO_STREAM_NAMED("compliant_control",
                              "Exit condition met in direction: " << i);
        vOut[i] = 0.0;
        exitCondition = compliantEnum::CONDITION_MET;
      } else {
        vOut[i] = vIn[i] + ft_[i] / stiffness_[i];
        if (exitCondition != compliantEnum::CONDITION_MET) {
          exitCondition = compliantEnum::CONDITION_NOT_MET;
        }
      }
    } else // end_condition_wrench_[i]<=0
    {
      if (ft_[i] < end_condition_wrench_[i]) {
        ROS_INFO_STREAM_NAMED("compliant_control",
                              "Exit condition met in direction: " << i);
        vOut[i] = 0.0;
        exitCondition = compliantEnum::CONDITION_MET;
      } else {
        vOut[i] = vIn[i] + ft_[i] / stiffness_[i];
        if (exitCondition != compliantEnum::CONDITION_MET) {
          exitCondition = compliantEnum::CONDITION_NOT_MET;
        }
      }
    }
  }
  return exitCondition;
}

LowPassFilter::LowPassFilter(double filterParam) : filterParam_(filterParam) {}

double LowPassFilter::filter(const double &new_msrmt) {
  // Push in the new measurement
  prev_msrmts_[2] = prev_msrmts_[1];
  prev_msrmts_[1] = prev_msrmts_[0];
  prev_msrmts_[0] = new_msrmt;

  double new_filtered_msrmt =
      (1 / (1 + filterParam_ * filterParam_ + 1.414 * filterParam_)) *
      (prev_msrmts_[2] + 2 * prev_msrmts_[1] + prev_msrmts_[0] -
       (filterParam_ * filterParam_ - 1.414 * filterParam_ + 1) *
           prev_filtered_msrmts_[1] -
       (-2 * filterParam_ * filterParam_ + 2) * prev_filtered_msrmts_[0]);
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