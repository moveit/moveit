/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
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

#include <limits>
//#include <fstream>
//#include <iostream>
#include <Eigen/Geometry>
#include <algorithm>
#include <angles/angles.h>
#include <cmath>
#include <console_bridge/console.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <vector>

namespace trajectory_processing
{
class LinearPathSegment : public PathSegment
{
public:
  LinearPathSegment(const Eigen::VectorXd& start, const Eigen::VectorXd& end)
    : start(start), end(end), PathSegment((end - start).norm())
  {
  }

  Eigen::VectorXd getConfig(double s) const
  {
    s /= length;
    s = std::max(0.0, std::min(1.0, s));
    return (1.0 - s) * start + s * end;
  }

  Eigen::VectorXd getTangent(double /* s */) const
  {
    return (end - start) / length;
  }

  Eigen::VectorXd getCurvature(double /* s */) const
  {
    return Eigen::VectorXd::Zero(start.size());
  }

  std::list<double> getSwitchingPoints() const
  {
    return std::list<double>();
  }

  LinearPathSegment* clone() const
  {
    return new LinearPathSegment(*this);
  }

private:
  Eigen::VectorXd start;
  Eigen::VectorXd end;
};

class CircularPathSegment : public PathSegment
{
public:
  CircularPathSegment(const Eigen::VectorXd& start, const Eigen::VectorXd& intersection, const Eigen::VectorXd& end,
                      double maxDeviation)
  {
    if ((intersection - start).norm() < 0.000001 || (end - intersection).norm() < 0.000001)
    {
      length = 0.0;
      radius = 1.0;
      center = intersection;
      x = Eigen::VectorXd::Zero(start.size());
      y = Eigen::VectorXd::Zero(start.size());
      return;
    }

    const Eigen::VectorXd startDirection = (intersection - start).normalized();
    const Eigen::VectorXd endDirection = (end - intersection).normalized();

    if ((startDirection - endDirection).norm() < 0.000001)
    {
      length = 0.0;
      radius = 1.0;
      center = intersection;
      x = Eigen::VectorXd::Zero(start.size());
      y = Eigen::VectorXd::Zero(start.size());
      return;
    }

    const double startDistance = (start - intersection).norm();
    const double endDistance = (end - intersection).norm();

    double distance = std::min((start - intersection).norm(), (end - intersection).norm());
    const double angle = acos(startDirection.dot(endDirection));

    distance = std::min(distance, maxDeviation * sin(0.5 * angle) / (1.0 - cos(0.5 * angle)));  // enforce max deviation

    radius = distance / tan(0.5 * angle);
    length = angle * radius;

    center = intersection + (endDirection - startDirection).normalized() * radius / cos(0.5 * angle);
    x = (intersection - distance * startDirection - center).normalized();
    y = startDirection;
  }

  Eigen::VectorXd getConfig(double s) const
  {
    const double angle = s / radius;
    return center + radius * (x * cos(angle) + y * sin(angle));
  }

  Eigen::VectorXd getTangent(double s) const
  {
    const double angle = s / radius;
    return -x * sin(angle) + y * cos(angle);
  }

  Eigen::VectorXd getCurvature(double s) const
  {
    const double angle = s / radius;
    return -1.0 / radius * (x * cos(angle) + y * sin(angle));
  }

  std::list<double> getSwitchingPoints() const
  {
    std::list<double> switchingPoints;
    const double dim = x.size();
    for (unsigned int i = 0; i < dim; i++)
    {
      double switchingAngle = atan2(y[i], x[i]);
      if (switchingAngle < 0.0)
      {
        switchingAngle += M_PI;
      }
      const double switchingPoint = switchingAngle * radius;
      if (switchingPoint < length)
      {
        switchingPoints.push_back(switchingPoint);
      }
    }
    switchingPoints.sort();
    return switchingPoints;
  }

  CircularPathSegment* clone() const
  {
    return new CircularPathSegment(*this);
  }

private:
  double radius;
  Eigen::VectorXd center;
  Eigen::VectorXd x;
  Eigen::VectorXd y;
};

Path::Path(const std::list<Eigen::VectorXd>& path, double maxDeviation) : length(0.0)
{
  if (path.size() < 2)
    return;
  std::list<Eigen::VectorXd>::const_iterator config1 = path.begin();
  std::list<Eigen::VectorXd>::const_iterator config2 = config1;
  config2++;
  std::list<Eigen::VectorXd>::const_iterator config3;
  Eigen::VectorXd startConfig = *config1;
  while (config2 != path.end())
  {
    config3 = config2;
    config3++;
    if (maxDeviation > 0.0 && config3 != path.end())
    {
      CircularPathSegment* blendSegment =
          new CircularPathSegment(0.5 * (*config1 + *config2), *config2, 0.5 * (*config2 + *config3), maxDeviation);
      Eigen::VectorXd endConfig = blendSegment->getConfig(0.0);
      if ((endConfig - startConfig).norm() > 0.000001)
      {
        pathSegments.push_back(new LinearPathSegment(startConfig, endConfig));
      }
      pathSegments.push_back(blendSegment);

      startConfig = blendSegment->getConfig(blendSegment->getLength());
    }
    else
    {
      pathSegments.push_back(new LinearPathSegment(startConfig, *config2));
      startConfig = *config2;
    }
    config1 = config2;
    config2++;
  }

  // Create list of switching point candidates, calculate total path length and
  // absolute positions of path segments
  for (std::list<PathSegment*>::iterator segment = pathSegments.begin(); segment != pathSegments.end(); segment++)
  {
    (*segment)->position = length;
    std::list<double> localSwitchingPoints = (*segment)->getSwitchingPoints();
    for (std::list<double>::const_iterator point = localSwitchingPoints.begin(); point != localSwitchingPoints.end();
         point++)
    {
      switchingPoints.push_back(std::make_pair(length + *point, false));
    }
    length += (*segment)->getLength();
    while (!switchingPoints.empty() && switchingPoints.back().first >= length)
      switchingPoints.pop_back();
    switchingPoints.push_back(std::make_pair(length, true));
  }
  switchingPoints.pop_back();
}

Path::Path(const Path& path) : length(path.length), switchingPoints(path.switchingPoints)
{
  for (std::list<PathSegment*>::const_iterator it = path.pathSegments.begin(); it != path.pathSegments.end(); it++)
  {
    pathSegments.push_back((*it)->clone());
  }
}

Path::~Path()
{
  for (std::list<PathSegment*>::iterator it = pathSegments.begin(); it != pathSegments.end(); it++)
  {
    delete *it;
  }
}

double Path::getLength() const
{
  return length;
}

PathSegment* Path::getPathSegment(double& s) const
{
  std::list<PathSegment*>::const_iterator it = pathSegments.begin();
  std::list<PathSegment*>::const_iterator next = it;
  next++;
  while (next != pathSegments.end() && s >= (*next)->position)
  {
    it = next;
    next++;
  }
  s -= (*it)->position;
  return *it;
}

Eigen::VectorXd Path::getConfig(double s) const
{
  const PathSegment* pathSegment = getPathSegment(s);
  return pathSegment->getConfig(s);
}

Eigen::VectorXd Path::getTangent(double s) const
{
  const PathSegment* pathSegment = getPathSegment(s);
  return pathSegment->getTangent(s);
}

Eigen::VectorXd Path::getCurvature(double s) const
{
  const PathSegment* pathSegment = getPathSegment(s);
  return pathSegment->getCurvature(s);
}

double Path::getNextSwitchingPoint(double s, bool& discontinuity) const
{
  std::list<std::pair<double, bool> >::const_iterator it = switchingPoints.begin();
  while (it != switchingPoints.end() && it->first <= s)
  {
    it++;
  }
  if (it == switchingPoints.end())
  {
    discontinuity = true;
    return length;
  }
  else
  {
    discontinuity = it->second;
    return it->first;
  }
}

std::list<std::pair<double, bool> > Path::getSwitchingPoints() const
{
  return switchingPoints;
}

const double Trajectory::eps = 0.000001;

static double squared(double d)
{
  return d * d;
}

Trajectory::Trajectory(const Path& path, const Eigen::VectorXd& maxVelocity, const Eigen::VectorXd& maxAcceleration,
                       double timeStep)
  : path(path)
  , maxVelocity(maxVelocity)
  , maxAcceleration(maxAcceleration)
  , n(maxVelocity.size())
  , valid(true)
  , timeStep(timeStep)
  , cachedTime(std::numeric_limits<double>::max())
{
  trajectory.push_back(TrajectoryStep(0.0, 0.0));
  double afterAcceleration = getMinMaxPathAcceleration(0.0, 0.0, true);
  while (valid && !integrateForward(trajectory, afterAcceleration) && valid)
  {
    double beforeAcceleration;
    TrajectoryStep switchingPoint;
    if (getNextSwitchingPoint(trajectory.back().pathPos, switchingPoint, beforeAcceleration, afterAcceleration))
    {
      break;
    }
    integrateBackward(trajectory, switchingPoint.pathPos, switchingPoint.pathVel, beforeAcceleration);
  }

  if (valid)
  {
    double beforeAcceleration = getMinMaxPathAcceleration(path.getLength(), 0.0, false);
    integrateBackward(trajectory, path.getLength(), 0.0, beforeAcceleration);
  }

  if (valid)
  {
    // Calculate timing
    std::list<TrajectoryStep>::iterator previous = trajectory.begin();
    std::list<TrajectoryStep>::iterator it = previous;
    it->time = 0.0;
    it++;
    while (it != trajectory.end())
    {
      it->time = previous->time + (it->pathPos - previous->pathPos) / ((it->pathVel + previous->pathVel) / 2.0);
      previous = it;
      it++;
    }
  }
}

Trajectory::~Trajectory(void)
{
}

/*void Trajectory::outputPhasePlaneTrajectory() const
{
  ofstream file1("maxVelocity.txt");
  const double stepSize = path.getLength() / 100000.0;
  for (double s = 0.0; s < path.getLength(); s += stepSize)
  {
    double maxVelocity = getAccelerationMaxPathVelocity(s);
    if (maxVelocity == std::numeric_limits<double>::infinity())
      maxVelocity = 10.0;
    file1 << s << "  " << maxVelocity << "  " << getVelocityMaxPathVelocity(s)
<< endl;
  }
  file1.close();

  ofstream file2("trajectory.txt");
  for (std::list<TrajectoryStep>::const_iterator it = trajectory.begin(); it !=
trajectory.end(); it++)
  {
    file2 << it->pathPos << "  " << it->pathVel << endl;
  }
  for (std::list<TrajectoryStep>::const_iterator it = endTrajectory.begin(); it
!= endTrajectory.end(); it++)
  {
    file2 << it->pathPos << "  " << it->pathVel << endl;
  }
  file2.close();
}*/

// Returns true if end of path is reached.
bool Trajectory::getNextSwitchingPoint(double pathPos, TrajectoryStep& nextSwitchingPoint, double& beforeAcceleration,
                                       double& afterAcceleration)
{
  TrajectoryStep accelerationSwitchingPoint(pathPos, 0.0);
  double accelerationBeforeAcceleration, accelerationAfterAcceleration;
  bool accelerationReachedEnd;
  do
  {
    accelerationReachedEnd =
        getNextAccelerationSwitchingPoint(accelerationSwitchingPoint.pathPos, accelerationSwitchingPoint,
                                          accelerationBeforeAcceleration, accelerationAfterAcceleration);
    double test = getVelocityMaxPathVelocity(accelerationSwitchingPoint.pathPos);
  } while (!accelerationReachedEnd &&
           accelerationSwitchingPoint.pathVel > getVelocityMaxPathVelocity(accelerationSwitchingPoint.pathPos));

  TrajectoryStep velocitySwitchingPoint(pathPos, 0.0);
  double velocityBeforeAcceleration, velocityAfterAcceleration;
  bool velocityReachedEnd;
  do
  {
    velocityReachedEnd = getNextVelocitySwitchingPoint(velocitySwitchingPoint.pathPos, velocitySwitchingPoint,
                                                       velocityBeforeAcceleration, velocityAfterAcceleration);
  } while (!velocityReachedEnd && velocitySwitchingPoint.pathPos <= accelerationSwitchingPoint.pathPos &&
           (velocitySwitchingPoint.pathVel > getAccelerationMaxPathVelocity(velocitySwitchingPoint.pathPos - eps) ||
            velocitySwitchingPoint.pathVel > getAccelerationMaxPathVelocity(velocitySwitchingPoint.pathPos + eps)));

  if (accelerationReachedEnd && velocityReachedEnd)
  {
    return true;
  }
  else if (!accelerationReachedEnd &&
           (velocityReachedEnd || accelerationSwitchingPoint.pathPos <= velocitySwitchingPoint.pathPos))
  {
    nextSwitchingPoint = accelerationSwitchingPoint;
    beforeAcceleration = accelerationBeforeAcceleration;
    afterAcceleration = accelerationAfterAcceleration;
    return false;
  }
  else
  {
    nextSwitchingPoint = velocitySwitchingPoint;
    beforeAcceleration = velocityBeforeAcceleration;
    afterAcceleration = velocityAfterAcceleration;
    return false;
  }
}

bool Trajectory::getNextAccelerationSwitchingPoint(double pathPos, TrajectoryStep& nextSwitchingPoint,
                                                   double& beforeAcceleration, double& afterAcceleration)
{
  double switchingPathPos = pathPos;
  double switchingPathVel;
  while (true)
  {
    bool discontinuity;
    switchingPathPos = path.getNextSwitchingPoint(switchingPathPos, discontinuity);

    if (switchingPathPos > path.getLength() - eps)
    {
      return true;
    }

    if (discontinuity)
    {
      const double beforePathVel = getAccelerationMaxPathVelocity(switchingPathPos - eps);
      const double afterPathVel = getAccelerationMaxPathVelocity(switchingPathPos + eps);
      switchingPathVel = std::min(beforePathVel, afterPathVel);
      beforeAcceleration = getMinMaxPathAcceleration(switchingPathPos - eps, switchingPathVel, false);
      afterAcceleration = getMinMaxPathAcceleration(switchingPathPos + eps, switchingPathVel, true);

      if ((beforePathVel > afterPathVel ||
           getMinMaxPhaseSlope(switchingPathPos - eps, switchingPathVel, false) >
               getAccelerationMaxPathVelocityDeriv(switchingPathPos - 2.0 * eps)) &&
          (beforePathVel < afterPathVel ||
           getMinMaxPhaseSlope(switchingPathPos + eps, switchingPathVel, true) <
               getAccelerationMaxPathVelocityDeriv(switchingPathPos + 2.0 * eps)))
      {
        break;
      }
    }
    else
    {
      switchingPathVel = getAccelerationMaxPathVelocity(switchingPathPos);
      beforeAcceleration = 0.0;
      afterAcceleration = 0.0;

      if (getAccelerationMaxPathVelocityDeriv(switchingPathPos - eps) < 0.0 &&
          getAccelerationMaxPathVelocityDeriv(switchingPathPos + eps) > 0.0)
      {
        break;
      }
    }
  }

  nextSwitchingPoint = TrajectoryStep(switchingPathPos, switchingPathVel);
  return false;
}

bool Trajectory::getNextVelocitySwitchingPoint(double pathPos, TrajectoryStep& nextSwitchingPoint,
                                               double& beforeAcceleration, double& afterAcceleration)
{
  const double stepSize = 0.001;
  const double accuracy = 0.000001;

  bool start = false;
  pathPos -= stepSize;
  do
  {
    pathPos += stepSize;

    if (getMinMaxPhaseSlope(pathPos, getVelocityMaxPathVelocity(pathPos), false) >=
        getVelocityMaxPathVelocityDeriv(pathPos))
    {
      start = true;
    }
  } while ((!start ||
            getMinMaxPhaseSlope(pathPos, getVelocityMaxPathVelocity(pathPos), false) >
                getVelocityMaxPathVelocityDeriv(pathPos)) &&
           pathPos < path.getLength());

  if (pathPos >= path.getLength())
  {
    return true;  // end of trajectory reached
  }

  double beforePathPos = pathPos - stepSize;
  double afterPathPos = pathPos;
  while (afterPathPos - beforePathPos > accuracy)
  {
    pathPos = (beforePathPos + afterPathPos) / 2.0;
    if (getMinMaxPhaseSlope(pathPos, getVelocityMaxPathVelocity(pathPos), false) >
        getVelocityMaxPathVelocityDeriv(pathPos))
    {
      beforePathPos = pathPos;
    }
    else
    {
      afterPathPos = pathPos;
    }
  }

  beforeAcceleration = getMinMaxPathAcceleration(beforePathPos, getVelocityMaxPathVelocity(beforePathPos), false);
  afterAcceleration = getMinMaxPathAcceleration(afterPathPos, getVelocityMaxPathVelocity(afterPathPos), true);
  nextSwitchingPoint = TrajectoryStep(afterPathPos, getVelocityMaxPathVelocity(afterPathPos));
  return false;
}

// Returns true if end of path is reached
bool Trajectory::integrateForward(std::list<TrajectoryStep>& trajectory, double acceleration)
{
  double pathPos = trajectory.back().pathPos;
  double pathVel = trajectory.back().pathVel;

  std::list<std::pair<double, bool> > switchingPoints = path.getSwitchingPoints();
  std::list<std::pair<double, bool> >::iterator nextDiscontinuity = switchingPoints.begin();

  while (true)
  {
    while ((nextDiscontinuity != switchingPoints.end()) &&
           (nextDiscontinuity->first <= pathPos || !nextDiscontinuity->second))
    {
      nextDiscontinuity++;
    }

    double oldPathPos = pathPos;
    double oldPathVel = pathVel;

    pathVel += timeStep * acceleration;
    pathPos += timeStep * 0.5 * (oldPathVel + pathVel);

    if (nextDiscontinuity != switchingPoints.end() && pathPos > nextDiscontinuity->first)
    {
      pathVel = oldPathVel + (nextDiscontinuity->first - oldPathPos) * (pathVel - oldPathVel) / (pathPos - oldPathPos);
      pathPos = nextDiscontinuity->first;
    }

    if (pathPos > path.getLength())
    {
      trajectory.push_back(TrajectoryStep(pathPos, pathVel));
      return true;
    }
    else if (pathVel < 0.0)
    {
      valid = false;
      logError("error");
      return true;
    }

    if (pathVel > getVelocityMaxPathVelocity(pathPos) &&
        getMinMaxPhaseSlope(oldPathPos, getVelocityMaxPathVelocity(oldPathPos), false) <=
            getVelocityMaxPathVelocityDeriv(oldPathPos))
    {
      pathVel = getVelocityMaxPathVelocity(pathPos);
    }

    trajectory.push_back(TrajectoryStep(pathPos, pathVel));
    acceleration = getMinMaxPathAcceleration(pathPos, pathVel, true);

    if (pathVel > getAccelerationMaxPathVelocity(pathPos) || pathVel > getVelocityMaxPathVelocity(pathPos))
    {
      // Find more accurate intersection with max-velocity curve using bisection
      TrajectoryStep overshoot = trajectory.back();
      trajectory.pop_back();
      double before = trajectory.back().pathPos;
      double beforePathVel = trajectory.back().pathVel;
      double after = overshoot.pathPos;
      double afterPathVel = overshoot.pathVel;
      while (after - before > eps)
      {
        const double midpoint = 0.5 * (before + after);
        double midpointPathVel = 0.5 * (beforePathVel + afterPathVel);

        if (midpointPathVel > getVelocityMaxPathVelocity(midpoint) &&
            getMinMaxPhaseSlope(before, getVelocityMaxPathVelocity(before), false) <=
                getVelocityMaxPathVelocityDeriv(before))
        {
          midpointPathVel = getVelocityMaxPathVelocity(midpoint);
        }

        if (midpointPathVel > getAccelerationMaxPathVelocity(midpoint) ||
            midpointPathVel > getVelocityMaxPathVelocity(midpoint))
        {
          after = midpoint;
          afterPathVel = midpointPathVel;
        }
        else
        {
          before = midpoint;
          beforePathVel = midpointPathVel;
        }
      }
      trajectory.push_back(TrajectoryStep(before, beforePathVel));

      if (getAccelerationMaxPathVelocity(after) < getVelocityMaxPathVelocity(after))
      {
        if (after > nextDiscontinuity->first)
        {
          return false;
        }
        else if (getMinMaxPhaseSlope(trajectory.back().pathPos, trajectory.back().pathVel, true) >
                 getAccelerationMaxPathVelocityDeriv(trajectory.back().pathPos))
        {
          return false;
        }
      }
      else
      {
        if (getMinMaxPhaseSlope(trajectory.back().pathPos, trajectory.back().pathVel, false) >
            getVelocityMaxPathVelocityDeriv(trajectory.back().pathPos))
        {
          return false;
        }
      }
    }
  }
}

void Trajectory::integrateBackward(std::list<TrajectoryStep>& startTrajectory, double pathPos, double pathVel,
                                   double acceleration)
{
  std::list<TrajectoryStep>::iterator start2 = startTrajectory.end();
  start2--;
  std::list<TrajectoryStep>::iterator start1 = start2;
  start1--;
  std::list<TrajectoryStep> trajectory;
  double slope;
  assert(start1->pathPos <= pathPos);

  while (start1 != startTrajectory.begin() || pathPos >= 0.0)
  {
    if (start1->pathPos <= pathPos)
    {
      trajectory.push_front(TrajectoryStep(pathPos, pathVel));
      pathVel -= timeStep * acceleration;
      pathPos -= timeStep * 0.5 * (pathVel + trajectory.front().pathVel);
      acceleration = getMinMaxPathAcceleration(pathPos, pathVel, false);
      slope = (trajectory.front().pathVel - pathVel) / (trajectory.front().pathPos - pathPos);

      if (pathVel < 0.0)
      {
        valid = false;
        logError("Error while integrating backward: Negative path velocity");
        endTrajectory = trajectory;
        return;
      }
    }
    else
    {
      start1--;
      start2--;
    }

    // Check for intersection between current start trajectory and backward
    // trajectory segments
    const double startSlope = (start2->pathVel - start1->pathVel) / (start2->pathPos - start1->pathPos);
    const double intersectionPathPos =
        (start1->pathVel - pathVel + slope * pathPos - startSlope * start1->pathPos) / (slope - startSlope);
    if (std::max(start1->pathPos, pathPos) - eps <= intersectionPathPos &&
        intersectionPathPos <= eps + std::min(start2->pathPos, trajectory.front().pathPos))
    {
      const double intersectionPathVel = start1->pathVel + startSlope * (intersectionPathPos - start1->pathPos);
      startTrajectory.erase(start2, startTrajectory.end());
      startTrajectory.push_back(TrajectoryStep(intersectionPathPos, intersectionPathVel));
      startTrajectory.splice(startTrajectory.end(), trajectory);
      return;
    }
  }

  valid = false;
  logError("Error while integrating backward: Did not hit start trajectory");
  endTrajectory = trajectory;
}

double Trajectory::getMinMaxPathAcceleration(double pathPos, double pathVel, bool max)
{
  Eigen::VectorXd configDeriv = path.getTangent(pathPos);
  Eigen::VectorXd configDeriv2 = path.getCurvature(pathPos);
  double factor = max ? 1.0 : -1.0;
  double maxPathAcceleration = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < n; i++)
  {
    if (configDeriv[i] != 0.0)
    {
      maxPathAcceleration =
          std::min(maxPathAcceleration, maxAcceleration[i] / std::abs(configDeriv[i]) -
                                            factor * configDeriv2[i] * pathVel * pathVel / configDeriv[i]);
    }
  }
  return factor * maxPathAcceleration;
}

double Trajectory::getMinMaxPhaseSlope(double pathPos, double pathVel, bool max)
{
  return getMinMaxPathAcceleration(pathPos, pathVel, max) / pathVel;
}

double Trajectory::getAccelerationMaxPathVelocity(double pathPos) const
{
  double maxPathVelocity = std::numeric_limits<double>::infinity();
  const Eigen::VectorXd configDeriv = path.getTangent(pathPos);
  const Eigen::VectorXd configDeriv2 = path.getCurvature(pathPos);
  for (unsigned int i = 0; i < n; i++)
  {
    if (configDeriv[i] != 0.0)
    {
      for (unsigned int j = i + 1; j < n; j++)
      {
        if (configDeriv[j] != 0.0)
        {
          double A_ij = configDeriv2[i] / configDeriv[i] - configDeriv2[j] / configDeriv[j];
          if (A_ij != 0.0)
          {
            maxPathVelocity = std::min(maxPathVelocity, sqrt((maxAcceleration[i] / std::abs(configDeriv[i]) +
                                                              maxAcceleration[j] / std::abs(configDeriv[j])) /
                                                             std::abs(A_ij)));
          }
        }
      }
    }
    else if (configDeriv2[i] != 0.0)
    {
      maxPathVelocity = std::min(maxPathVelocity, sqrt(maxAcceleration[i] / std::abs(configDeriv2[i])));
    }
  }
  return maxPathVelocity;
}

double Trajectory::getVelocityMaxPathVelocity(double pathPos) const
{
  const Eigen::VectorXd tangent = path.getTangent(pathPos);
  double maxPathVelocity = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < n; i++)
  {
    maxPathVelocity = std::min(maxPathVelocity, maxVelocity[i] / std::abs(tangent[i]));
  }
  return maxPathVelocity;
}

double Trajectory::getAccelerationMaxPathVelocityDeriv(double pathPos)
{
  return (getAccelerationMaxPathVelocity(pathPos + eps) - getAccelerationMaxPathVelocity(pathPos - eps)) / (2.0 * eps);
}

double Trajectory::getVelocityMaxPathVelocityDeriv(double pathPos)
{
  const Eigen::VectorXd tangent = path.getTangent(pathPos);
  double maxPathVelocity = std::numeric_limits<double>::max();
  unsigned int activeConstraint;
  for (unsigned int i = 0; i < n; i++)
  {
    const double thisMaxPathVelocity = maxVelocity[i] / std::abs(tangent[i]);
    if (thisMaxPathVelocity < maxPathVelocity)
    {
      maxPathVelocity = thisMaxPathVelocity;
      activeConstraint = i;
    }
  }
  return -(maxVelocity[activeConstraint] * path.getCurvature(pathPos)[activeConstraint]) /
         (tangent[activeConstraint] * std::abs(tangent[activeConstraint]));
}

bool Trajectory::isValid() const
{
  return valid;
}

double Trajectory::getDuration() const
{
  return trajectory.back().time;
}

std::list<Trajectory::TrajectoryStep>::const_iterator Trajectory::getTrajectorySegment(double time) const
{
  if (time >= trajectory.back().time)
  {
    std::list<TrajectoryStep>::const_iterator last = trajectory.end();
    last--;
    return last;
  }
  else
  {
    if (time < cachedTime)
    {
      cachedTrajectorySegment = trajectory.begin();
    }
    while (time >= cachedTrajectorySegment->time)
    {
      cachedTrajectorySegment++;
    }
    cachedTime = time;
    return cachedTrajectorySegment;
  }
}

Eigen::VectorXd Trajectory::getPosition(double time) const
{
  std::list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;

  double timeStep = it->time - previous->time;
  const double acceleration =
      2.0 * (it->pathPos - previous->pathPos - timeStep * previous->pathVel) / (timeStep * timeStep);

  timeStep = time - previous->time;
  const double pathPos = previous->pathPos + timeStep * previous->pathVel + 0.5 * timeStep * timeStep * acceleration;

  return path.getConfig(pathPos);
}

Eigen::VectorXd Trajectory::getVelocity(double time) const
{
  std::list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;

  double timeStep = it->time - previous->time;
  const double acceleration =
      2.0 * (it->pathPos - previous->pathPos - timeStep * previous->pathVel) / (timeStep * timeStep);

  timeStep = time - previous->time;
  const double pathPos = previous->pathPos + timeStep * previous->pathVel + 0.5 * timeStep * timeStep * acceleration;
  const double pathVel = previous->pathVel + timeStep * acceleration;

  return path.getTangent(pathPos) * pathVel;
}

}  // namespace trajectory_processing
