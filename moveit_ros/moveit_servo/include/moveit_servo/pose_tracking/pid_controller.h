#pragma once

namespace moveit_servo
{
class PIDController
{
public:
  /** \brief Constructor
   * \param Kp proportional gain
   * \param Ki Integral gain
   * \param Kd derivative gain
   * \param dt loop interval time
   * \param max maximum value of manipulated variable
   * \param min minimum value of manipulated variable
   */
  PIDController(double dt, double max, double min, double k_p, double k_i, double k_d, double windup_limit);

  /**
    \brief The controller calculates and provides an output signal.
    * Returns zero unless controller was properly initialized.
  */
  double getOutputSignal();

  double getSetpoint();

  double getCurrentStateMeasurement();

  double getMin();

  double getMax();

  void setSetpoint(double new_setpoint);

  void setCurrentStateMeasurement(double new_state_measurement);

  void setProportionalGain(double proportional_gain);
  void setIntegralGain(double integral_gain);
  void setDerivativeGain(double derivative_gain);

  void setLoopRate(double delta_t);

  /** \brief Set largest allowable contribution of integral term (Ki * error_integral) */
  void setWindupLimit(double windup_limit);

  bool isInitialized() const noexcept
  {
    return has_setpoint_ && has_state_;
  };

  /** \brief Put the controller in a safe state where it will not produce output */
  void resetController() noexcept
  {
    has_setpoint_ = false;
    has_state_ = false;
    resetControllerErrors();
  }

  /** \brief Reset the controller's memory of earlier errors. Useful between discrete motions */
  void resetControllerErrors() noexcept
  {
    prev_error_ = 0;
    integral_error_ = 0;
  }

private:
  double clamp(double input, double min, double max);

  double dt_ = 0;
  double max_ = 0;
  double min_ = 0;
  double k_p_ = 0;
  double k_i_ = 0;
  double k_d_ = 0;
  double prev_error_ = 0;
  double integral_error_ = 0;
  double setpoint_ = 0;
  double state_ = 0;
  double windup_limit_ = 0;

  bool has_setpoint_ = false;
  bool has_state_ = false;
};
}  // namespace moveit_servo
