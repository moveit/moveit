#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <iostream>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/kinematic_terms.hpp>
#include <trajopt/utils.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/eigen_slicing.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace std;
using namespace sco;
using namespace Eigen;
using namespace util;

namespace
{
#if 0
Vector3d rotVec(const Matrix3d& m) {
  Quaterniond q; q = m;
  return Vector3d(q.x(), q.y(), q.z());
}
#endif

#if 0
VectorXd concat(const VectorXd& a, const VectorXd& b) {
  VectorXd out(a.size()+b.size());
  out.topRows(a.size()) = a;
  out.middleRows(a.size(), b.size()) = b;
  return out;
}

template <typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b) {
  vector<T> out;
  vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  return out;
}
#endif
}  // namespace

namespace trajopt_interface
{

// This initial traj gets passed as the solution for results_.x in optimizer::initialize
// which will get updated by optimize() function through getClosestFeasiblePoint
// then in sqp loop, it gets passed to convexifyCosts(const std::vector<CostPtr>& costs, const DblVec& x, Model* model)
// where convex(x, model) function of each cost(CostFromFunc : Cost) gets called.
// So basically, x is an element that starts with an intial value and gets updated as the solution thtough optimize()
VectorXd CartPoseErrCalculator::operator()(const VectorXd& dof_vals) const
{
  // dof_vals is the solution that gets updated in optimize()
  // target_pose_inv_ is the inverse of the target pose that is given
  Isometry3d iteration_new_pose; //, robot_base;
  robot_model::RobotModelConstPtr robot_model =  planning_scene_->getRobotModel();
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));

  //  robot_base = robot_state->getGlobalLinkTransform(robot_model->getRootLinkName()); //state->transforms.at(manip_->getBaseLinkName());
  //  assert(change_base.isApprox(
  //    env_->getState(manip_->getJointNames(), dof_vals)->transforms.at(manip_->getBaseLinkName())));

  //  manip_->calcFwdKin(new_pose, change_base, dof_vals, link_, *state);

  // Calculate the forward kineamtics with the given joint values for the given link
  robot_state->setJointGroupPositions("panda_arm", dof_vals); //   void setJointGroupPositions(const std::string& joint_group_name, const Eigen::VectorXd& values)
  robot_state_->update();
  std::string last_link_name = robot_model->getLinkModelNames.back();
  iteration_new_pose = robot_state_->getGlobalLinkTransform(last_link_name); // the pose that gets updated in optimize() based on each increment solution

  Isometry3d pose_err = target_pose_inv_ * new_pose;
  Quaterniond q(pose_err.rotation());
  VectorXd err = concat(Vector3d(q.x(), q.y(), q.z()), pose_err.translation());
  return err;
}


}  // namespace trajopt
