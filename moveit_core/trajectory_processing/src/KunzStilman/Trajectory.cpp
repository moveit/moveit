/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 10/2011
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "trajectory_processing/KunzStilman/Trajectory.h"

using namespace std;
using namespace Eigen;
using namespace ParabolicBlend;

Trajectory::Trajectory(const list<VectorXd> &_path, const VectorXd &maxVelocity, const VectorXd &maxAcceleration, double minWayPointSeparation) :
	path(_path.begin(), _path.end()),
	velocities(path.size() - 1),
	accelerations(path.size()),
	durations(path.size() - 1),
	blendDurations(path.size()),
	duration(0.0)
{
  bool removeWayPoints = false;
	if(minWayPointSeparation > 0.0) {
		removeWayPoints = true;
	}

	// remove waypoints that are too close together
	while(removeWayPoints) {
		removeWayPoints = false;

		vector<VectorXd>::iterator it = path.begin();
		vector<VectorXd>::iterator next = it;
		if(next != path.end())
			next++;
		while(next != path.end()) {
			if((*it - *next).norm() < minWayPointSeparation) {
				removeWayPoints = true;
				vector<VectorXd>::iterator nextNext = next;
				nextNext++;
				if(it == path.begin()) {
					it = path.erase(next);
				}
				else if(nextNext == path.end()) {
					it = path.erase(it);
				}
				else {
					VectorXd newViaPoint = 0.5 * (*it + *next);
					it = path.erase(it);
					it = path.insert(it, newViaPoint);
					it++;
					it = path.erase(it);
				}
				next = it;
				if(next != path.end())
					next++;
			}
			else {
				it = next;
				next++;
			}
		}

		velocities.resize(path.size() - 1);
		accelerations.resize(path.size());
		durations.resize(path.size() - 1);
		blendDurations.resize(path.size());
	}

	// calculate time between waypoints and initial velocities of linear segments
	for(unsigned int i = 0; i < path.size() - 1; i++) {
		durations[i] = 0.0;
		for(int j = 0; j < path[i].size(); j++) {
			durations[i] = max(durations[i], abs(path[i+1][j] - path[i][j]) / maxVelocity[j]);
		}
		velocities[i] = (path[i+1] - path[i]) / durations[i];
	}

	int numBlendsSlowedDown = numeric_limits<int>::max();
	while(numBlendsSlowedDown > 1) {
		numBlendsSlowedDown = 0;
		vector<double> slowDownFactors(path.size(), 1.0);

		for(unsigned int i = 0; i < path.size(); i++) {
			// calculate blend duration and acceleration
			VectorXd previousVelocity = (i == 0) ? VectorXd::Zero(path[i].size()) : velocities[i-1];
			VectorXd nextVelocity = (i == path.size() - 1) ? VectorXd::Zero(path[i].size()) : velocities[i];
			blendDurations[i] = 0.0;
			for(int j = 0; j < path[i].size(); j++) {
				blendDurations[i] = max(blendDurations[i], abs(nextVelocity[j] - previousVelocity[j]) / maxAcceleration[j]);
				accelerations[i] = (nextVelocity - previousVelocity) / blendDurations[i];
			}

      // calculate slow down factor such that the blend phase replaces at most half of the neighboring linear segments
			const double eps = 0.000001;
			if((i > 0 && blendDurations[i] > durations[i-1] + eps && blendDurations[i-1] + blendDurations[i] > 2.0 * durations[i-1] + eps)
				|| i < path.size() - 1 && blendDurations[i] > durations[i] + eps && blendDurations[i] + blendDurations[i+1] > 2.0 * durations[i] + eps)
			{
				numBlendsSlowedDown++;
				const double maxDuration = min(i == 0 ? numeric_limits<double>::max() : durations[i-1],
					i == path.size() - 1 ? numeric_limits<double>::max() : durations[i]);
				slowDownFactors[i] = sqrt(maxDuration / blendDurations[i]);
			}
		}

    // apply slow down factors to linear segments
		for(unsigned int i = 0; i < path.size() - 1; i++) {
			velocities[i] *= min(slowDownFactors[i], slowDownFactors[i+1]);
			durations[i] /= min(slowDownFactors[i], slowDownFactors[i+1]);
		}
  }
	
  // calculate total time of trajectory
	for(unsigned int i = 0; i < path.size() - 1; i++) {
		duration += durations[i];
	}
	duration += 0.5 * blendDurations.front() + 0.5 * blendDurations.back();
}


VectorXd Trajectory::getPosition(double time) const {
  if(time > duration) {
		return path.back();
	}
	double t = time;
	if(t <= 0.5 * blendDurations[0]) {
		return path[0] + 0.5 * t * t * accelerations[0];
	}
	else {
		t -= 0.5 * blendDurations[0];
	}
  unsigned int i = 0;
	while(i < path.size() - 1 && t > durations[i]) {
		t -= durations[i];
		i++;
	}
  if(i == path.size() - 1) {
		t = 0.5 * blendDurations.back() - t;
		return path.back() + 0.5 * t * t * accelerations.back();
	}

	double switchingTime1 = 0.5 * blendDurations[i];
	double switchingTime2 = durations[i] - 0.5 * blendDurations[i+1];

	if(t < switchingTime1) {
		t = switchingTime1 - t;
		return path[i] + switchingTime1 * velocities[i] - t * velocities[i] + 0.5 * t * t * accelerations[i];
	}
	else if(t > switchingTime2) {
		t -= switchingTime2;
		return path[i] + switchingTime2 * velocities[i] + t * velocities[i] + 0.5 * t * t * accelerations[i+1];
	}
	else {
		return path[i] + t * velocities[i];
	}
}


VectorXd Trajectory::getVelocity(double time) const {
  if(time > duration) {
		return VectorXd::Zero(path.back().size());
	}
	double t = time;
	if(t <= 0.5 * blendDurations[0]) {
		return t * accelerations[0];
	}
	else {
		t -= 0.5 * blendDurations[0];
	}
  unsigned int i = 0;
	while(i < path.size() - 1 && t > durations[i]) {
		t -= durations[i];
		i++;
	}
  if(i == path.size() - 1) {
		t = 0.5 * blendDurations.back() - t;
		return - t * accelerations.back();
	}

	double switchingTime1 = 0.5 * blendDurations[i];
	double switchingTime2 = durations[i] - 0.5 * blendDurations[i+1];

  if(t < switchingTime1) {
		t = switchingTime1 - t;
		return velocities[i] - t * accelerations[i];
	}
	else if(t > switchingTime2) {
		t -= switchingTime2;
		return velocities[i] + t * accelerations[i+1];
	}
	else {
		return velocities[i];
	}
}


double Trajectory::getDuration() const {
  return duration;
}
