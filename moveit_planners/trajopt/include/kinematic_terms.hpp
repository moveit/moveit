#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_sco/sco_fwd.hpp>

namespace trajopt_interface
{

/**
 * @brief Used to calculate the error for StaticCartPoseTermInfo
 * This is converted to a cost or constraint using TrajOptCostFromErrFunc or TrajOptConstraintFromErrFunc
 */
struct CartPoseErrCalculator : public VectorOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Isometry3d target_pose_inv_;
  //  tesseract::BasicKinConstPtr manip_;
  // tesseract::BasicEnvConstPtr env_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  std::string link_;
  Eigen::Isometry3d tcp_;

  CartPoseErrCalculator(const Eigen::Isometry3d& pose,
                        const planning_scene::PlanningSceneConstPtr& planning_scene,
                        std::string link,
                        Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity())
    : target_pose_inv_(pose.inverse()), planning_scene_(planning_scene), link_(link), tcp_(tcp)
  {
  }

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};


}  // namespace trajopt
