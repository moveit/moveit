/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

