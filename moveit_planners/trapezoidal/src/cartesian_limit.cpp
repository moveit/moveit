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

#include "trapezoidal_trajectory_generation/cartesian_limit.h"

trapezoidal::CartesianLimit::CartesianLimit():
  has_max_trans_vel_(false),
  max_trans_vel_(0.0),
  has_max_trans_acc_(false),
  max_trans_acc_(0.0),
  has_max_trans_dec_(false),
  max_trans_dec_(0.0),
  has_max_rot_vel_(false),
  max_rot_vel_(0.0)
{

}

// Translational Velocity Limit

bool trapezoidal::CartesianLimit::hasMaxTranslationalVelocity() const
{
  return has_max_trans_vel_;
}

void trapezoidal::CartesianLimit::setMaxTranslationalVelocity(double max_trans_vel)
{
  has_max_trans_vel_ = true;
  max_trans_vel_ = max_trans_vel;
}

double trapezoidal::CartesianLimit::getMaxTranslationalVelocity() const
{
  return max_trans_vel_;
}

// Translational Acceleration Limit

bool trapezoidal::CartesianLimit::hasMaxTranslationalAcceleration() const
{
  return has_max_trans_acc_;
}

void trapezoidal::CartesianLimit::setMaxTranslationalAcceleration(double max_trans_acc)
{
  has_max_trans_acc_ = true;
  max_trans_acc_ = max_trans_acc;
}

double trapezoidal::CartesianLimit::getMaxTranslationalAcceleration() const
{
  return max_trans_acc_;
}

// Translational Deceleration Limit

bool trapezoidal::CartesianLimit::hasMaxTranslationalDeceleration() const
{
  return has_max_trans_dec_;
}

void trapezoidal::CartesianLimit::setMaxTranslationalDeceleration(double max_trans_dec)
{
  has_max_trans_dec_ = true;
  max_trans_dec_ = max_trans_dec;
}

double trapezoidal::CartesianLimit::getMaxTranslationalDeceleration() const
{
  return max_trans_dec_;
}

// Rotational Velocity Limit

bool trapezoidal::CartesianLimit::hasMaxRotationalVelocity() const
{
  return has_max_rot_vel_;
}

void trapezoidal::CartesianLimit::setMaxRotationalVelocity(double max_rot_vel)
{
  has_max_rot_vel_ = true;
  max_rot_vel_ = max_rot_vel;
}

double trapezoidal::CartesianLimit::getMaxRotationalVelocity() const
{
  return max_rot_vel_;
}

