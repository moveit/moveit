/*****************************************************************************
 *
 * Copyright (c) 2010-2011, the Trustees of Indiana University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Indiana University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE TRUSTEES OF INDIANA UNIVERSITY ''AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE TRUSTEES OF INDIANA UNIVERSITY BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ***************************************************************************/

#ifndef PARABOLIC_RAMP_H
#define PARABOLIC_RAMP_H

#include <math.h>
#include "trajectory_processing/ParabolicPathSmooth/Math.h"

namespace ParabolicRamp {

/** @file ParabolicRamp.h
 * @brief Functions for optimal acceleration-bounded trajectories.
 */

/** @brief Stores optimal trajectores for an acceleration and
 * velocity-bounded 1D system.
 *
 * Initialize the members x0 (start position), x1 (end position),
 * dx0 (start velocity), and dx1 (end velocity) before calling the
 * SolveX functions.
 */
class ParabolicRamp1D
{
 public:
  /// Sets the ramp to a constant function for time t
  void SetConstant(Real x,Real t=0);
  /// Sets the ramp to a linear function from x0 to x1 with time t.
  void SetLinear(Real x0,Real x1,Real t);
  /// Solves for minimum time given acceleration and velocity bounds
  bool SolveMinTime(Real amax,Real vmax);
  /// Solves for minimum time given acceleration and velocity bounds, min time
  bool SolveMinTime2(Real amax,Real vmax,Real tLowerBound);
  /// Solves for minimum acceleration given end time and velocity bounds
  bool SolveMinAccel(Real endTime,Real vmax);
  /// Same, but if fails, returns the minimum time > endTime
  Real SolveMinAccel2(Real endTime,Real vmax);
  /// Solves for the minimum-time braking trajectory starting from x0,dx0
  void SolveBraking(Real amax);
  /// Evaluates the trajectory
  Real Evaluate(Real t) const;
  /// Evaluates the derivative of the trajectory
  Real Derivative(Real t) const;
  /// Evaluates the second derivative of the trajectory
  Real Accel(Real t) const;
  /// Returns the time at which x1 is reached
  Real EndTime() const { return ttotal; }
  /// Scales time to slow down (value > 1) or speed up (< 1) the trajectory
  void Dilate(Real timeScale);
  /// Trims off the front [0,tcut] of the trajectory
  void TrimFront(Real tcut);
  /// Trims off the front [T-tcut,T] of the trajectory
  void TrimBack(Real tcut);
  /// Returns the x bounds on the path
  void Bounds(Real& xmin,Real& xmax) const;
  /// Returns the x bounds for the given time interval
  void Bounds(Real ta,Real tb,Real& xmin,Real& xmax) const;
  /// Returns the v bounds on the path
  void DerivBounds(Real& vmin,Real& vmax) const;
  /// Returns the v bounds for the given time interval
  void DerivBounds(Real ta,Real tb,Real& vmin,Real& vmax) const;
  /// Sanity check
  bool IsValid() const;

  /// Input
  Real x0,dx0;
  Real x1,dx1;

  /// Calculated upon SolveX
  Real tswitch1,tswitch2;  //time to switch between ramp/flat/ramp
  Real ttotal;
  Real a1,v,a2;   // accel of first ramp, velocity of linear section, accel of second ramp
};

/** @brief Solves for optimal trajectores for a velocity-bounded ND system.
 *
 * Methods are essentially the same as for ParabolicRamp1D.
 */
class ParabolicRampND
{
 public:
  void SetConstant(const Vector& x,Real t=0);
  void SetLinear(const Vector& x0,const Vector& x1,Real t);
  bool SolveMinTimeLinear(const Vector& amax,const Vector& vmax);
  bool SolveMinTime(const Vector& amax,const Vector& vmax);
  bool SolveMinAccel(const Vector& vmax,Real time);
  bool SolveMinAccelLinear(const Vector& vmax,Real time);
  void SolveBraking(const Vector& amax);
  void Evaluate(Real t,Vector& x) const;
  void Derivative(Real t,Vector& dx) const;
  void Accel(Real t,Vector& ddx) const;
  void Output(Real dt,std::vector<Vector>& path) const;
  void Dilate(Real timeScale);
  void TrimFront(Real tcut);
  void TrimBack(Real tcut);
  void Bounds(Vector& xmin,Vector& xmax) const;
  void Bounds(Real ta,Real tb,Vector& xmin,Vector& xmax) const;
  void DerivBounds(Vector& vmin,Vector& vmax) const;
  void DerivBounds(Real ta,Real tb,Vector& vmin,Vector& vmax) const;
  bool IsValid() const;

  /// Input
  Vector x0,dx0;
  Vector x1,dx1;

  /// Calculated upon SolveX
  Real endTime;
  std::vector<ParabolicRamp1D> ramps;
};

/// Computes a min-time ramp from (x0,v0) to (x1,v1) under the given
/// acceleration, velocity, and x bounds.  Returns true if successful.
bool SolveMinTimeBounded(Real x0,Real v0,Real x1,Real v1,
			 Real amax,Real vmax,Real xmin,Real xmax,
			 ParabolicRamp1D& ramp);

/// Computes a sequence of up to three ramps connecting (x0,v0) to (x1,v1)
/// in minimum-acceleration fashion with a fixed end time, under the given 
/// velocity and x bounds.  Returns true if successful.
bool SolveMinAccelBounded(Real x0,Real v0,Real x1,Real v1,
			  Real endTime,Real vmax,Real xmin,Real xmax,
			  std::vector<ParabolicRamp1D>& ramps);

/// Vector version of above.
/// Returns the time of the minimum time trajectory, or -1 on failure
Real SolveMinTimeBounded(const Vector& x0,const Vector& v0,const Vector& x1,const Vector& v1,
			 const Vector& amax,const Vector& vmax,const Vector& xmin,const Vector& xmax,
			 std::vector<std::vector<ParabolicRamp1D> >& ramps);

/// Vector version of above.
/// Returns true if successful.
bool SolveMinAccelBounded(const Vector& x0,const Vector& v0,const Vector& x1,const Vector& v1,
			 Real endTime,const Vector& vmax,const Vector& xmin,const Vector& xmax,
			 std::vector<std::vector<ParabolicRamp1D> >& ramps);

/// Combines an array of 1-d ramp sequences into a sequence of N-d ramps
void CombineRamps(const std::vector<std::vector<ParabolicRamp1D> >& ramps,std::vector<ParabolicRampND>& ndramps);

} //namespace ParabolicRamp

#endif
