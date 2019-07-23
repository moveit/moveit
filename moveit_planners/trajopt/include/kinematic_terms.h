#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_sco/sco_fwd.hpp>
#include <trajopt_sco/num_diff.hpp>

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

namespace trajopt_interface
{

Vector3d rotVec(const Matrix3d& m)
{
  Quaterniond q; q = m;
  return Vector3d(q.x(), q.y(), q.z());
}


VectorXd concat(const VectorXd& a, const VectorXd& b)
{
  VectorXd out(a.size()+b.size());
  out.topRows(a.size()) = a;
  out.middleRows(a.size(), b.size()) = b;
  return out;
}

template <typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b)
{
  vector<T> out;
  vector<int> x;
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
  //  tesseract::BasicKinConstPtr manip_;
  // tesseract::BasicEnvConstPtr env_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  std::string link_;
  Eigen::Isometry3d tcp_;

  CartPoseErrCalculator(const Eigen::Isometry3d& pose,
                        const planning_scene::PlanningSceneConstPtr planning_scene,
                        std::string link,
                        Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity())
    : target_pose_inv_(pose.inverse()), planning_scene_(planning_scene), link_(link), tcp_(tcp)
  {
  }

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

  //TODO(omid): The following should be added and adjusted from trajopt
  // JointPosEqCost
  // JointPosIneqCost
  // JointPosEqConstraint
  // JointPosIneqConstraint

}  // namespace trajopt
