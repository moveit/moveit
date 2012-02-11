/*****************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2009, the Trustees of Indiana University
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
#ifndef PARABOLIC_RAMP_MATH_H
#define PARABOLIC_RAMP_MATH_H

#include <math.h>
#include <stdlib.h>
#include <vector>

namespace ParabolicRamp {

typedef double Real;
typedef std::vector<double> Vector;

//can replace this with your favorite representation/tests of infinity
const static Real Inf = 1e300;
inline bool IsInf(Real x) { return x==Inf; }
inline bool IsFinite(Real x) { return fabs(x)<Inf; }

inline Real Sqr(Real x) { return x*x; }
inline Real Sqrt(Real x) { return sqrt(x); }
inline Real Abs(Real x) { return fabs(x); }
inline Real Sign(Real x) { return (x>0 ? 1 : (x<0 ? -1 : 0)); }
inline Real Min(Real x,Real y) { return (x<y?x:y); }
inline Real Max(Real x,Real y) { return (x>y?x:y); }
inline bool FuzzyZero(Real x,Real tol) { return Abs(x)<=tol; }
inline bool FuzzyEquals(Real x,Real y,Real tol) { return Abs(x-y)<=tol; }
inline void Swap(Real& x,Real& y) { Real temp=x; x=y; y=temp; }

// Returns a random number in [0,1)
inline Real Rand() { return Real(rand())/Real(RAND_MAX); }

} //namespace ParabolicRamp

#endif
