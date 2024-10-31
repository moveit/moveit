/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

/* Author: Ioan Sucan */

#pragma once

#include <moveit/robot_model/joint_model.h>

namespace moveit
{
namespace core
{
/** \brief A planar joint */
class PlanarJointModel : public JointModel
{
public:
  /** \brief different types of planar joints we support */
  enum MotionModel
  {
    HOLONOMIC,  // default
    DIFF_DRIVE
  };

  PlanarJointModel(const std::string& name);

  void getVariableDefaultPositions(double* values, const Bounds& other_bounds) const override;
  void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values,
                                  const Bounds& other_bounds) const override;
  void getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator& rng, double* values,
                                        const Bounds& other_bounds, const double* seed,
                                        const double distance) const override;
  bool enforcePositionBounds(double* values, const Bounds& other_bounds) const override;
  bool satisfiesPositionBounds(const double* values, const Bounds& other_bounds, double margin) const override;

  void interpolate(const double* from, const double* to, const double t, double* state) const override;
  unsigned int getStateSpaceDimension() const override;
  double getMaximumExtent(const Bounds& other_bounds) const override;
  double distance(const double* values1, const double* values2) const override;

  void computeTransform(const double* joint_values, Eigen::Isometry3d& transf) const override;
  void computeVariablePositions(const Eigen::Isometry3d& transf, double* joint_values) const override;

  double getAngularDistanceWeight() const
  {
    return angular_distance_weight_;
  }

  void setAngularDistanceWeight(double weight)
  {
    angular_distance_weight_ = weight;
  }

  double getMinTranslationalDistance() const
  {
    return min_translational_distance_;
  }

  void setMinTranslationalDistance(double min_translational_distance)
  {
    min_translational_distance_ = min_translational_distance;
  }

  MotionModel getMotionModel() const
  {
    return motion_model_;
  }

  void setMotionModel(MotionModel model)
  {
    motion_model_ = model;
  }

  /// Make the yaw component of a state's value vector be in the range [-Pi, Pi]. enforceBounds() also calls this
  /// function;
  /// Return true if a change is actually made
  bool normalizeRotation(double* values) const;

private:
  double angular_distance_weight_;
  MotionModel motion_model_;
  /// Only used for the differential drive motion model @see computeTurnDriveTurnGeometry
  double min_translational_distance_;
};
/**
 * @brief Compute the geometry to turn toward the target point, drive straight and then turn to target orientation
 * @param[in]  from                       A vector representing the initial position [x0, y0, theta0]
 * @param[in]  to                         A vector representing the target position  [x1, y1, theta1]
 * @param[in]  min_translational_distance If the translational distance between \p from and \p to is less than this
 *                                        value the motion will be pure rotation (meters)
 * @param[out] dx                         x1 - x0 (meters)
 * @param[out] dy                         y1 - y0 (meters)
 * @param[out] initial_turn               The initial turn in radians to face the target
 * @param[out] drive_angle                The orientation in radians that the robot will be driving straight at
 * @param[out] final_turn                 The final turn in radians to the target orientation
 */
void computeTurnDriveTurnGeometry(const double* from, const double* to, const double min_translational_distance,
                                  double& dx, double& dy, double& initial_turn, double& drive_angle,
                                  double& final_turn);
}  // namespace core
}  // namespace moveit
