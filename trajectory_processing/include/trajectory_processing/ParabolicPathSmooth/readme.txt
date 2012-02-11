************************************************
*      Parabolic Path Smoothing libraries      *
************************************************

Contents
I. Package Contents
II. Usage
   II.A. Basic Smoothing
   II.B. Exact Collision Detection
   II.C. Online Smoothing
III. Contact Information
IV. Version History
V. License

**** I. Package Contents ****

ParabolicRamp.h/cpp
  - Header and definition file for time-optimal, acceleration-bounded
    trajectories in 1 and N dimensions.
DynamicPath.h/cpp
  - Header and definition file for smoothing paths using a shortcut
    method.
Timer.h/cpp
  - A timer, used in DynamicPath::OnlineShortcut.  These files and
    the code for OnlineShortcut can be safely removed if online
    shortcutting is not used in your program.
Math.h
  - Definitions of basic math subroutines.
Config.h
  - Constant configuration parameters regarding numerical tolerances
   and error logging.
mc_test.cpp
  - A test program that performs Monte Carlo verification of the input
    space for the ParabolicRamp solver
Makefile
  - The project makefile.
doxygen.conf
  - Configuration file for the automatic documentation tool doxygen.


**** II. Usage ****

Add the files *.h and *.cpp to your project and compile with a C++
compiler.  Optionally, documentation can be generated using the
doxygen tool by running the command 'doxygen doxygen.conf' in this
directory.

The definitions in Config.h and Math.h may be altered to suit
the needs of your project.

** II.A. Basic Smoothing **

For a path given as a sequence of milestones (such as a path output
by a probablistic roadmap planner), the basic smoothing algorithm is
invoked as follows.

  vector<Vector> path;        //the sequence of milestones
  Vector vmax,amax;           //velocity and acceleration bounds, respectively
  MyFeasibilityChecker feas;  //a configuration/segment feasibility checker, subclassed from FeasibilityCheckerBase
  int numIters=1000;          //some number of shortcutting iterations
  Real tol=1e-4;              //checks a piecewise linear path whose distance from the parabolic path is no more than the tolerance tol
  RampFeasibilityChecker checker(&feas,tol);
  //TODO: compute milestones, velocity and acceleration bounds

  DynamicPath traj;
  traj.Init(vmax,amax);
  traj.SetMilestones(path);   //now the trajectory starts and stops at every milestone
  printf("Initial path duration: %g\n",traj.GetTotalTime());
  int res=traj.Shortcut(numIters,checker);
  printf("After shortcutting: %d shortcuts taken, duration %g\n",traj.GetTotalTime());

** II.B. Exact Collision Checking **

To use exact trajectory collision checking, you must implement
DistanceCheckerBase to return a lower bound on the L-infinity distance
between a configuration and the obstacle boundary (in C-space).
Then, pass the feasibility checker and distance checker to the overloaded
RampFeasibilityChecker constructor.  (For this variant, the
FeasibilityCheckerBase::SegmentFeasible method does not need to
be implemented. )

** II.C. Online Smoothing **

To avoid waiting for smoothing it is possible to interleave trajectory
smoothing with execution.  Omitting synchronization code, the pseudocode
for doing so is as follows:

Input:
  traj, a DynamicPath to be executed and smoothed
  padTime, a constant ~= a bound on the time it takes to perform one shortcut
  A timer function Time()

Execution thread:
  x = traj.Evaluate(Time())
  Move toward configuration x

Smoothing thread:
  while (Time() < traj.GetTotalTime()) 
    t1 = Rand(Time()+padTime,traj.GetTotalTime())
    t2 = Rand(Time()+padTime,traj.GetTotalTime())
    DynamicPath temp = traj
    temp.TryShortcut(t1,t2)
    // the shortcut may have taken too long, if so, throw it out
    if (Time() > t1)
      continue
    else
      traj = temp

For slightly better performance, the sampling strategy for picking points
along the trajectory can be tuned.  For example, biasing t1 to more heavily
sample points near the current execution time seems to be effective.  See
DynamicPath::OnlineShortcut for more information.


**** III. Contact Information ****

Author: Kris Hauser
Mail:
   School of Informatics and Computing
   919 E. 10th St #257
   Bloomington, IN 47408
Office: (812) 856-7496
Fax: (812) 855-4829
Email: hauserk@indiana.edu
Web: http://cs.indiana.edu/~hauserk


**** IV. Version History ****

v1.3 - 10/21/2011 - Fixed a bug in the bounded min-accel shortcutting method
                    in which bounds of a subproblem was not enforced.  Added
                    better configuration options for OpenRAVE integration.
v1.2 - 5/25/2011  - Added support for configuration box-bounds. Fixed an
                    unaddressed side case in the N-D minimum-time problem.
v1.1 - 1/31/2010  - Update with new convenience functions, better numerical
                    performance.
v1.0 - 8/26/2009  - Initial release.


**** V. License ****

Copyright (c) 2009-2011, the Trustees of Indiana University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Indiana University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE TRUSTEES OF INDIANA UNIVERSITY ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE TRUSTEES OF INDIANA UNIVERSITY BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

