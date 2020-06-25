/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik, LLC.
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
 *   * Neither the name of PickNik nor the names of its
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

/* Author: Omid Heidari */

#pragma once

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>

namespace trajopt
{
/**
 * @brief Extracts the vector part of quaternion
 */
inline Eigen::Vector3d quaternionRotationVector(const Eigen::Matrix3d& matrix)
{
  Eigen::Quaterniond quaternion;
  quaternion = matrix;
  return Eigen::Vector3d(quaternion.x(), quaternion.y(), quaternion.z());
}

/**
 * @brief Appends b to a of type VectorXd
 */
inline Eigen::VectorXd concatVector(const Eigen::VectorXd& vector_a, const Eigen::VectorXd& vector_b)
{
  Eigen::VectorXd out(vector_a.size() + vector_b.size());
  out.topRows(vector_a.size()) = vector_a;
  out.middleRows(vector_a.size(), vector_b.size()) = vector_b;
  return out;
}

/**
 * @brief Appends b to a of type T
 */
template <typename T>
inline std::vector<T> concatVector(const std::vector<T>& vector_a, const std::vector<T>& vector_b)
{
  std::vector<T> out;
  out.insert(out.end(), vector_a.begin(), vector_a.end());
  out.insert(out.end(), vector_b.begin(), vector_b.end());
  return out;
}

/**
 * @brief Used to calculate the error for StaticCartPoseTermInfo
 * This is converted to a cost or constraint using TrajOptCostFromErrFunc or TrajOptConstraintFromErrFunc
 */
struct CartPoseErrCalculator : public sco::VectorOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Isometry3d target_pose_inv_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  std::string link_;
  Eigen::Isometry3d tcp_;

  CartPoseErrCalculator(const Eigen::Isometry3d& pose, const planning_scene::PlanningSceneConstPtr planning_scene,
                        const std::string& link, Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity())
    : target_pose_inv_(pose.inverse()), planning_scene_(planning_scene), link_(link), tcp_(tcp)
  {
  }

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

// TODO(omid): The following should be added and adjusted from trajopt
// JointPosEqCost
// JointPosIneqCost
// JointPosEqConstraint
// JointPosIneqConstraint

struct JointVelErrCalculator : sco::VectorOfVector
{
  /** @brief Velocity target */
  double target_;
  /** @brief Upper tolerance */
  double upper_tol_;
  /** @brief Lower tolerance */
  double lower_tol_;
  JointVelErrCalculator() : target_(0.0), upper_tol_(0.0), lower_tol_(0.0)
  {
  }
  JointVelErrCalculator(double target, double upper_tol, double lower_tol)
    : target_(target), upper_tol_(upper_tol), lower_tol_(lower_tol)
  {
  }

  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const override;
};

struct JointVelJacobianCalculator : sco::MatrixOfVector
{
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const override;
};

}  // namespace trajopt
