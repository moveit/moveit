#include "moveit_servo/pose_tracking/pid_controller.h"

using namespace std;

namespace moveit_servo
{
PIDController::PIDController(double dt, double max, double min, double k_p, double k_i, double k_d, double windup_limit)
  : dt_(dt)
  , max_(max)
  , min_(min)
  , k_p_(k_p)
  , k_i_(k_i)
  , k_d_(k_d)
  , prev_error_(0)
  , integral_error_(0)
  , windup_limit_(windup_limit)
{
}

double PIDController::getOutputSignal()
{
  if (!isInitialized())
    return 0;

  // Calculate error
  double error = setpoint_ - state_;

  double p_term = k_p_ * error;

  integral_error_ += error * dt_;
  integral_error_ = clamp(integral_error_, -windup_limit_ / k_i_, windup_limit_ / k_i_);
  double i_term = k_i_ * integral_error_;

  double derivative = (error - prev_error_) / dt_;
  double d_term = k_d_ * derivative;

  // Calculate total output
  double output = p_term + i_term + d_term;

  output = clamp(output, min_, max_);

  prev_error_ = error;

  return output;
}

double PIDController::getSetpoint()
{
  return setpoint_;
}

double PIDController::getCurrentStateMeasurement()
{
  return state_;
}

double PIDController::getMin()
{
  return min_;
}

double PIDController::getMax()
{
  return max_;
}

void PIDController::setSetpoint(double new_setpoint)
{
  setpoint_ = new_setpoint;
  has_setpoint_ = true;
}

void PIDController::setCurrentStateMeasurement(double new_state_measurement)
{
  state_ = new_state_measurement;
  has_state_ = true;
}

void PIDController::setProportionalGain(double proportional_gain)
{
  k_p_ = proportional_gain;
}

void PIDController::setIntegralGain(double integral_gain)
{
  k_i_ = integral_gain;
}

void PIDController::setDerivativeGain(double derivative_gain)
{
  k_d_ = derivative_gain;
}

void PIDController::setLoopRate(double loop_rate)
{
  dt_ = loop_rate;
}

void PIDController::setWindupLimit(double windup_limit)
{
  windup_limit_ = windup_limit;
}

double PIDController::clamp(double input, double min, double max)
{
  return (input < min) ? min : (max < input) ? max : input;
}
}  // namespace moveit_servo
