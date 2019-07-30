#pragma once

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

namespace trajopt_interface
{
Vector3d rotVec(const Matrix3d& m)
{
  Quaterniond q;
  q = m;
  return Vector3d(q.x(), q.y(), q.z());
}

VectorXd concat(const VectorXd& a, const VectorXd& b)
{
  VectorXd out(a.size() + b.size());
  out.topRows(a.size()) = a;
  out.middleRows(a.size(), b.size()) = b;
  return out;
}

template <typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b)
{
  vector<T> out;
  vector<int> x(a.size() + b.size());
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
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
                        std::string link, Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity())
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

  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const;
};

struct JointVelJacobianCalculator : sco::MatrixOfVector
{
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const;
};

}  // namespace trajopt_interface
