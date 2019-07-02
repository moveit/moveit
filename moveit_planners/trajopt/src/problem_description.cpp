#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <tesseract_core/basic_kin.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include "moveit/planning_interface/planning_request.h"
#include "moveit/planning_interface/planning_response.h"
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/planning_scene/planning_scene.h>

#include <trajopt/collision_terms.hpp>
#include <trajopt/common.hpp>
#include <trajopt/kinematic_terms.hpp>
#include <trajopt/plot_callback.hpp>

#include <trajopt/trajectory_costs.hpp>
#include <trajopt_sco/expr_op_overloads.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/eigen_slicing.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/vector_ops.hpp>

#include "problem_description.h"

namespace trajopt_interface
{


TrajOptProblem::TrajOptProblem(){}

TrajOptProblem::TrajOptProblem(const int& n_steps,
                   bool& use_time, const robot_model::RobotModelConstPtr& robot_model)
  : OptProb(sco::ModelType::AUTO_SOLVER)
{
  const std::string PLANNING_GROUP = "panda_arm";
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  moveit::core::JointBoundsVector bounds = joint_model_group->getActiveJointModelsBounds();

  int n_dof = robot_model->getJointModelGroup(PLANNING_GROUP)->getActiveJointModelNames().size();

  std::cout << "dof from TrajOptProb " << n_dof << std::endl;

  std::cout << "bounds length ===>>> " << bounds.size() << std::endl;

  Eigen::MatrixX2d limits(n_dof,2);
  for (int k = 0; k < limits.size() / 2; ++k){

    moveit::core::JointModel::Bounds bb = *bounds[k];
    // Joints are considered to have one degree of freedom, then the first element of
    // type Bounds is the one that we want.
    moveit::core::VariableBounds vb = bb.front();

    limits(k,0) =  vb.min_position_;
    limits(k,1) =  vb.max_position_;
  }

  Eigen::VectorXd lower, upper;
  lower = limits.col(0);
  upper = limits.col(1);

   /** @brief The upper limit of 1/dt values allowed in the optimization*/
  double dt_upper_lim = 1.0;
  /** @brief The lower limit of 1/dt values allowed in the optimization*/
  double dt_lower_lim = 1.0;
  trajopt::DblVec vlower, vupper;
  std::vector<std::string> names;
  for (int i = 0; i < n_steps; ++i)
  {
    for (int j = 0; j < n_dof; ++j)
    {
      names.push_back((boost::format("j_%i_%i") % i % j).str());
    }
    vlower.insert(vlower.end(), lower.data(), lower.data() + lower.size());
    vupper.insert(vupper.end(), upper.data(), upper.data() + upper.size());

    //    pci.basic_info.use_time
    if (use_time == true)
    {
      vlower.insert(vlower.end(), dt_lower_lim);
      vupper.insert(vupper.end(), dt_upper_lim);
      names.push_back((boost::format("dt_%i") % i).str());
    }
  }
  sco::VarVector trajvarvec = createVariables(names, vlower, vupper);
  m_traj_vars = trajopt::VarArray(n_steps, n_dof + (use_time ? 1 : 0), trajvarvec.data());
}

}

 /*
void trajopt_interface::TrajOptProblem::SetInitTraj(const trajopt::TrajArray& x) {
    std::cout << "====>>>>> setInitTraj is called " << std::endl;
    std::cout << "x.size: " << x.size()  << std::endl;
    std::cout << " beforeeeeeeeeeeeeeeeeeee " << std::endl;
    m_init_traj = x;
    std::cout << " afteeeeeeeeeeeeeeeeeeeer " << std::endl;
}
 */
