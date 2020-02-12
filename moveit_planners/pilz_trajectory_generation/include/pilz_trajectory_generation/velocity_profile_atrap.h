/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VELOCITY_PROFILE_ATRAP_H
#define VELOCITY_PROFILE_ATRAP_H

#include "kdl/velocityprofile.hpp"
#include <iostream>

namespace pilz {

/**
 * @brief A PTP Trajectory Generator of Asymmetric Trapezoidal Velocity Profile.
 * Differences to VelocityProfile_Trap:
 *   - Maximal acceleration and deceleration can be different, resulting
 *     an asymmetric trapezoid shaped velocity profile.
 *   - Function to generate full synchronized PTP trajectory is provided.
 *   - Function to generate trapezoid shaped velocity profile with start velocity.
 */
class VelocityProfile_ATrap : public KDL::VelocityProfile
{
public:
  /**
   * @brief Constructor
   * @param max_vel: maximal velocity (absolute value, always positive)
   * @param max_acc: maximal acceleration (absolute value, always positive)
   * @param max_dec: maximal deceleration (absolute value, always positive)
   */
  VelocityProfile_ATrap(double max_vel = 0, double max_acc = 0, double max_dec = 0);

  /**
   * @brief compute the fastest profile
   * Algorithm:
   *  - compute the minimal distance which is needed to reach maximal velocity
   *  - if maximal velocity can be reached
   *     - compute the coefficients of the trajectory
   *  - if maximal velocity can not be reached
   *     - compute the new velocity can be reached
   *     - compute the coefficients based on this new velocity
   *
   * @param pos1: start position
   * @param pos2: goal position
   */
  virtual void SetProfile(double pos1, double pos2) override;

  /**
   * @brief Profile scaled by the total duration
   * @param pos1: start position
   * @param pos2: goal position
   * @param duration: trajectory duration (must be longer than fastest case, otherwise will be ignored)
   */
  virtual void SetProfileDuration(double pos1, double pos2, double duration) override;

  /**
   * @brief Profile with given acceleration/constant/deceleration durations.
   * Each duration must obey the maximal velocity/acceleration/deceleration constraints.
   * Otherwise the operation will be ignored.
   * Algorithm:
   * - compute the maximal velocity of given durations
   * - compute the acceleration and deceleration of given duraitons
   * - if limits are fulfilled
   *   - compute the coefficients
   * @param pos1: start position
   * @param pos2: goal position
   * @param acc_duration: time of acceleration phase
   * @param const_duration: time of constant phase
   * @param dec_duration: time of deceleration phase
   * @return ture if the combination of three durations is valid
   */
  bool setProfileAllDurations(double pos1, double pos2, double duration1, double duration2, double duration3);


  /**
   * @brief Profile with start velocity
   * Note: This function is not general and is currently only used for live control (vel1*(pos2-pos1)>0).
   * @param pos1: start position
   * @param pos2: goal position
   * @param vel1: start velocity
   * @return
   */
  bool setProfileStartVelocity(double pos1, double pos2, double vel1);

  /**
   * @brief get the time of first phase
   * @return
   */
  double FirstPhaseDuration() const {return t_a_;}
  /**
   * @brief get the time of second phase
   * @return
   */
  double SecondPhaseDuration() const {return t_b_;}
  /**
   * @brief get the time of third phase
   * @return
   */
  double ThirdPhaseDuration() const  {return t_c_;}

  /**
   * @brief Compares two Asymmetric Trapezoidal Velocity Profiles.
   *
   * @return True if equal, false otherwise.
   */
  bool operator==(const VelocityProfile_ATrap& other) const;

   /**
   * @brief Duration
   * @return total duration of the trajectory
   */
  virtual double Duration() const override;
  /**
   * @brief Get position at given time
   * @param time
   * @return
   */
  virtual double Pos(double time) const override;
  /**
   * @brief Get velocity at given time
   * @param time
   * @return
   */
  virtual double Vel(double time) const override;
  /**
   * @brief Get given acceleration/deceleration at given time
   * @param time
   * @return
   */
  virtual double Acc(double time) const override;
  /**
   * @brief Write basic information
   * @param os
   */
  virtual void Write(std::ostream& os) const override;
  /**
   * @brief returns copy of current VelocityProfile object
   * @return
   */
  virtual KDL::VelocityProfile* Clone() const override;

  friend std::ostream &operator<<(std::ostream& os, const VelocityProfile_ATrap& p); //LCOV_EXCL_LINE

  virtual ~VelocityProfile_ATrap();

private:

  /// helper functions
  void setEmptyProfile();

private:

  /// specification of the motion profile :
  const double max_vel_;
  const double max_acc_;
  const double max_dec_;
  double start_pos_;
  double end_pos_;

  /// for initial velocity
  double start_vel_;

  /// three phases of trapezoid
  double a1_,a2_,a3_; /// coef. from ^0 -> ^2 of first phase
  double b1_,b2_,b3_; /// of second phase
  double c1_,c2_,c3_; /// of third phase

  /// time of three phases
  double t_a_; /// the duration of first phase
  double t_b_; /// the duration of second phase
  double t_c_; /// the duration of third phase
};

std::ostream &operator<<(std::ostream& os, const VelocityProfile_ATrap& p);//LCOV_EXCL_LINE

}

#endif // VELOCITY_PROFILE_ATRAP_H
