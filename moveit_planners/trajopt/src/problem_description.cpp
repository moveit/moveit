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

TrajOptProblem::TrajOptProblem(int n_steps, const ProblemInfo& problem_info)
  : OptProb(pci.basic_info.convex_solver), planning_scene_(problem_info.planning_scene), planning_group_(problem_info.planning_group_)
{
    robot_model::RobotModelConstPtr robot_model =  planning_scene_->getRobotModel();
//    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
//    robot_state_->update();
//    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(planning_group_);
   const robot_state::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(planning_group_);

  moveit::core::JointBoundsVector bounds = joint_model_group->getActiveJointModelsBounds();
  int n_dof = joint_model_group->getActiveJointModelNames().size(); //bounds.size();

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

    if (problem_info.basic_info.use_time == true)
    {
      vlower.insert(vlower.end(), problem_info.basic_info.dt_lower_lim);
      vupper.insert(vupper.end(), problem_info.basic_info.dt_upper_lim);
      names.push_back((boost::format("dt_%i") % i).str());
    }
  }

  sco::VarVector trajvarvec = createVariables(names, vlower, vupper);
  m_traj_vars = trajopt::VarArray(n_steps, n_dof + (problem_info.basic_info.use_time ? 1 : 0), trajvarvec.data());
  // m_traj_vars is essentialy a matrix of elements like:
  // j_0_0, j_0_1 ...
  // j_1_0, j_1_1 ...
  // its size is n_steps by n_dof
}


TrajOptProblemPtr ConstructProblem(const ProblemInfo& pci)
{
  const BasicInfo& bi = pci.basic_info;
  int n_steps = problem_info.n_steps;

  bool use_time = false;
  // Check that all costs and constraints support the types that are specified in pci
  for (TermInfoPtr cost : pci.cost_infos)
  {
    if (cost->term_type & TT_CNT)
      ROS_WARN("%s is listed as a type TT_CNT but was added to cost_infos", (cost->name).c_str());
    if (!(cost->getSupportedTypes() & TT_COST))
      PRINT_AND_THROW(boost::format("%s is only a constraint, but you listed it as a cost") % cost->name);
    if (cost->term_type & TT_USE_TIME)
    {
      use_time = true;
      if (!(cost->getSupportedTypes() & TT_USE_TIME))
        PRINT_AND_THROW(boost::format("%s does not support time, but you listed it as a using time") % cost->name);
    }
  }
  for (TermInfoPtr cnt : pci.cnt_infos)
  {
    if (cnt->term_type & TT_COST)
      ROS_WARN("%s is listed as a type TT_COST but was added to cnt_infos", (cnt->name).c_str());
    if (!(cnt->getSupportedTypes() & TT_CNT))
      PRINT_AND_THROW(boost::format("%s is only a cost, but you listed it as a constraint") % cnt->name);
    if (cnt->term_type & TT_USE_TIME)
    {
      use_time = true;
      if (!(cnt->getSupportedTypes() & TT_USE_TIME))
        PRINT_AND_THROW(boost::format("%s does not support time, but you listed it as a using time") % cnt->name);
    }
  }

  // Check that if a cost or constraint uses time, basic_info is set accordingly
  if ((use_time == true) && (pci.basic_info.use_time == false))
    PRINT_AND_THROW("A term is using time and basic_info is not set correctly. Try basic_info.use_time = true");

  // This could be removed in the future once we are sure that all costs are
  if ((use_time == false) && (pci.basic_info.use_time == true))
    PRINT_AND_THROW("No terms use time and basic_info is not set correctly. Try basic_info.use_time = false");


  //  TrajOptProbPtr prob(new TrajOptProb(n_steps, pci));
  // unsigned n_dof = prob->GetKin()->numJoints();

  // Generate initial trajectory and check its size
  std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  trajopt::TrajArray init_traj = generateInitialTrajectory(problem_info.num_steps, joint_values);

  if (pci.basic_info.use_time == true)
  {
    prob->SetHasTime(true);
    if (init_traj.rows() != n_steps || init_traj.cols() != n_dof + 1)
    {
      PRINT_AND_THROW(boost::format("Initial trajectory is not the right size matrix\n"
                                    "Expected %i rows (time steps) x %i columns (%i dof + 1 time column)\n"
                                    "Got %i rows and %i columns") %
                      n_steps % (n_dof + 1) % n_dof % init_traj.rows() % init_traj.cols());
    }
  }
  else
  {
    prob->SetHasTime(false);
    if (init_traj.rows() != n_steps || init_traj.cols() != n_dof)
    {
      PRINT_AND_THROW(boost::format("Initial trajectory is not the right size matrix\n"
                                    "Expected %i rows (time steps) x %i columns\n"
                                    "Got %i rows and %i columns") %
                      n_steps % n_dof % init_traj.rows() % init_traj.cols());
    }
  }
  prob->SetInitTraj(init_traj);

  // If start_fixed, constrain the joint values for the first time step to be their initialized values
  if (bi.start_fixed)
  {
    if (init_traj.rows() < 1)
    {
      PRINT_AND_THROW("Initial trajectory must contain at least the start state.");
    }

    if (init_traj.cols() != (n_dof + (use_time ? 1 : 0)))
    {
      PRINT_AND_THROW("robot dof values don't match initialization. I don't "
                      "know what you want me to use for the dof values");
    }

    for (int j = 0; j < static_cast<int>(n_dof); ++j)
    {
      prob->addLinearConstraint(sco::exprSub(sco::AffExpr(prob->m_traj_vars(0, j)), init_traj(0, j)), sco::EQ);
    }
  }

  // Apply constraint to each fixed dof to its initial value for all timesteps (freeze that column)
  if (!bi.dofs_fixed.empty())
  {
    for (const int& dof_ind : bi.dofs_fixed)
    {
      for (int i = 1; i < prob->GetNumSteps(); ++i)
      {
        prob->addLinearConstraint(
            sco::exprSub(sco::AffExpr(prob->m_traj_vars(i, dof_ind)), sco::AffExpr(init_traj(0, dof_ind))), sco::EQ);
      }
    }
  }

  for (const TermInfoPtr& ci : pci.cost_infos)
  {
    ci->hatch(*prob);
  }

  for (const TermInfoPtr& ci : pci.cnt_infos)
  {
    ci->hatch(*prob);
  }
  return prob;

}


CartPoseTermInfo::CartPoseTermInfo() : TermInfo(TT_COST | TT_CNT)
{
  pos_coeffs = Eigen::Vector3d::Ones();
  rot_coeffs = Eigen::Vector3d::Ones();
  tcp.setIdentity();
}

void CartPoseTermInfo::hatch(TrajOptProblem& prob)
{
  int n_dof = prob.GetNumDOF();

  Eigen::Isometry3d input_pose;
  Eigen::Quaterniond q(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
  input_pose.linear() = q.matrix();
  input_pose.translation() = xyz;

  if (term_type == (TT_COST | TT_USE_TIME))
  {
    ROS_ERROR("Use time version of this term has not been defined.");
  }
  else if (term_type == (TT_CNT | TT_USE_TIME))
  {
    ROS_ERROR("Use time version of this term has not been defined.");
  }
  else if ((term_type & TT_COST) && ~(term_type | ~TT_USE_TIME))
  {
    sco::VectorOfVectorPtr f(new CartPoseErrCalculator(input_pose, prob.GetPlanningScene(), link, tcp));
    prob.addCost(sco::CostPtr(new sco::CostFromErrFunc(
        f, prob.GetVarRow(timestep, 0, n_dof), concat(rot_coeffs, pos_coeffs), sco::ABS, name)));
  }
  else if ((term_type & TT_CNT) && ~(term_type | ~TT_USE_TIME))
  {
    sco::VectorOfVectorPtr f(new CartPoseErrCalculator(input_pose, prob.GetPlanningScene, link, tcp));
    prob.addConstraint(sco::ConstraintPtr(new sco::ConstraintFromErrFunc(
        f, prob.GetVarRow(timestep, 0, n_dof), concat(rot_coeffs, pos_coeffs), sco::EQ, name)));
  }
  else
  {
    ROS_WARN("CartPoseTermInfo does not have a valid term_type defined. No cost/constraint applied");
  }
}


}
