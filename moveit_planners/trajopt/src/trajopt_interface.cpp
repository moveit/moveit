/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik, Inc.
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

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/MotionPlanRequest.h>

#include <trajopt_sco/sco_common.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/solver_interface.hpp>

#include <trajopt/trajectory_costs.hpp>

#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <limits>
#include <vector>
#include <Eigen/Geometry>

#include "trajopt_interface.h"
#include "problem_description.h"

namespace trajopt_interface
{
TrajOptInterface::TrajOptInterface(const ros::NodeHandle& nh) : nh_(nh)
{
  prob_ = TrajOptProblemPtr(new TrajOptProblem);
  setDefaultTrajOPtParams();

  //TODO: callbacks should be defined by the user
  callbacks_.push_back(callBackFunc);
}

bool TrajOptInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                             const planning_interface::MotionPlanRequest& req,
                             moveit_msgs::MotionPlanDetailedResponse& res)
{
  std::cout << " ==================================== solve from trajopt_interface is called" << std::endl;
  setTrajOptParams(params_);

  if (!planning_scene)
  {
    ROS_ERROR_STREAM_NAMED("trajopt_planner", "No planning scene initialized.");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return false;
  }

  if (req.start_state.joint_state.position.empty())
  {
    ROS_ERROR_STREAM_NAMED("trajopt_planner", "Start state position is empty");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  if (req.start_state.joint_state.name.empty())
  {
    ROS_ERROR_STREAM_NAMED("trajopt_planner", "Start_state name is empty");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  if (not planning_scene->getRobotModel()->satisfiesPositionBounds(req.start_state.joint_state.position.data()))
  {
    ROS_ERROR_STREAM_NAMED("trajopt_planner", "Start state violates joint limits");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  ros::WallTime start_time = ros::WallTime::now();
  robot_model::RobotModelConstPtr robot_model = planning_scene->getRobotModel();
  robot_state::RobotStatePtr current_state(new robot_state::RobotState(robot_model));
  *current_state = planning_scene->getCurrentState();
  const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(req.group_name);
  std::vector<std::string> joint_names = joint_model_group->getVariableNames();
  std::vector<double> current_joint_values;
  current_state->copyJointGroupPositions(joint_model_group, current_joint_values);
  int dof = current_joint_values.size();

  std::cout << " ======================================= create ProblemInfo" << std::endl;
  ProblemInfo problem_info(planning_scene, req.group_name);

  setProblemInfoParam(problem_info);

  // Hardcoded - Populate Init Info
  Eigen::VectorXd current_joint_values_eigen(dof);
  Eigen::VectorXd tmp_end_pos(dof);
  for(int joint_index = 0; joint_index < dof; ++joint_index)
  {
    current_joint_values_eigen(joint_index) = current_joint_values[joint_index];

    // tmp_end_pos(joint_index) = req.start_state.joint_state.position[joint_index];

      tmp_end_pos(joint_index) = req.goal_constraints[0].joint_constraints[joint_index].position;
  }
  problem_info.init_info.end_pos = current_joint_values_eigen; //  tmp_end_pos;

  // Populate Init Info for GIVEN_TRAJ. We interpolate between current and the start state to make a trajectory to feed init_info
  problem_info.init_info.start_to_end_trajectory = current_joint_values_eigen.transpose().replicate(problem_info.basic_info.n_steps, 1);
  for (auto i = 0; i < dof; ++i)
    problem_info.init_info.start_to_end_trajectory.col(i) = Eigen::VectorXd::LinSpaced(
      static_cast<size_t>(problem_info.basic_info.n_steps), current_joint_values_eigen(i), tmp_end_pos(i));
 //       static_cast<size_t>(problem_info.basic_info.n_steps), current_joint_values_eigen(i), req.start_state.joint_state.position[static_cast<size_t>(i)]);

  std::cout << " ======================================= create Constraints" << std::endl;
  // Convert req.goal_constriant to cartesian pose cnt
  // planning_interface::MotionPlanRequest is the same as moveit_msgs::MotionPlanRequest

  if (req.goal_constraints.empty())
  {
    ROS_ERROR_STREAM_NAMED("trajopt_planner", "No goal constraints specified!");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  if (!req.goal_constraints[0].position_constraints.empty() && !req.goal_constraints[0].orientation_constraints.empty())
  {
    std::cout << "===>>> Creating Pose Constraints" << std::endl;
    CartPoseTermInfoPtr cart_goal_pos(new CartPoseTermInfo);
    //  std::shared_ptr<CartPoseTermInfo> pose_constraint = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);

    //TODO: Feed pose_constraint with request information and the needed param to the setup.yaml file

    // Add the constraint to problem_info
    problem_info.cnt_infos.push_back(cart_goal_pos);
  }
  else if (req.goal_constraints[0].position_constraints.empty() &&
           !req.goal_constraints[0].orientation_constraints.empty())
  {
    ROS_ERROR_STREAM_NAMED("trajopt_planner", "position constraint is not defined");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }
  else if (!req.goal_constraints[0].orientation_constraints.empty() &&
           req.goal_constraints[0].orientation_constraints.empty())
  {
    ROS_ERROR_STREAM_NAMED("trajopt_planner", "orientation constraint is not defined");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }


  // goal from request
  for(auto goal_cnt : req.goal_constraints)
    {
      JointPoseTermInfoPtr joint_pos_term(new JointPoseTermInfo);
      setJointPoseTermInfoParams(joint_pos_term, goal_cnt.name);

      trajopt::DblVec joint_goal_constraints;
      for (const moveit_msgs::JointConstraint& joint_goal_constraint : goal_cnt.joint_constraints)
        {
          joint_goal_constraints.push_back(joint_goal_constraint.position);
        }
      joint_pos_term->targets = joint_goal_constraints;
      problem_info.cnt_infos.push_back(joint_pos_term);
    }

  // Populate Cost Info, vel cost is hard-coded
  std::shared_ptr<JointVelTermInfo> joint_vel =  std::shared_ptr<JointVelTermInfo>(new trajopt_interface::JointVelTermInfo);

  joint_vel->coeffs = std::vector<double>(7, 5.0);
  joint_vel->targets = std::vector<double>(7, 0.0);
  joint_vel->first_step = 0;
  joint_vel->last_step = problem_info.basic_info.n_steps - 1;
  joint_vel->name = "joint_vel";
  joint_vel->term_type = trajopt_interface::TT_COST;
  problem_info.cost_infos.push_back(joint_vel);

  // add the start pos from request as a constraint
  JointPoseTermInfoPtr joint_start_pos(new JointPoseTermInfo);
  trajopt::DblVec joint_start_constraints;
  for (auto pos : req.start_state.joint_state.position)
    {
      joint_start_constraints.push_back(pos);
    }
  joint_start_pos->targets = joint_start_constraints;
  setJointPoseTermInfoParams(joint_start_pos, "start_pos");
  problem_info.cnt_infos.push_back(joint_start_pos);


  if (!req.goal_constraints[0].visibility_constraints.empty())
  {
    //TODO: Add visibility constraint
  }

  std::cout << " ======================================= DEBUG" << std::endl;
  std::cout << " ==== current state" << std::endl;
  for (int q = 0; q < current_joint_values.size(); ++q)
  {
    std::cout << current_joint_values[q] << std::endl;
  }
  std::cout << " ==== start state" << std::endl;
  for (int q = 0; q < req.start_state.joint_state.position.size(); ++q)
  {
    std::cout << req.start_state.joint_state.position[q] << std::endl;
  }

  ROS_INFO_STREAM_NAMED("trajopt_param.improve_ratio_threshold", params_.improve_ratio_threshold);
  ROS_INFO_STREAM_NAMED("trajopt_param.min_trust_box_size", params_.min_trust_box_size);
  ROS_INFO_STREAM_NAMED("trajopt_param.min_approx_improve", params_.min_approx_improve);
  ROS_INFO_STREAM_NAMED("trajopt_param.params_.min_approx_improve_frac", params_.min_approx_improve_frac);
  ROS_INFO_STREAM_NAMED("trajopt_param.max_iter", params_.max_iter);
  ROS_INFO_STREAM_NAMED("trajopt_param.trust_shrink_ratio", params_.trust_shrink_ratio);
  ROS_INFO_STREAM_NAMED("trajopt_param.trust_expand_ratio", params_.trust_expand_ratio);
  ROS_INFO_STREAM_NAMED("trajopt_param.cnt_tolerance", params_.cnt_tolerance);
  ROS_INFO_STREAM_NAMED("trajopt_param.max_merit_coeff_increases", params_.max_merit_coeff_increases);
  ROS_INFO_STREAM_NAMED("trajopt_param.merit_coeff_increase_ratio", params_.merit_coeff_increase_ratio);
  ROS_INFO_STREAM_NAMED("trajopt_param.max_time", params_.max_time);
  ROS_INFO_STREAM_NAMED("trajopt_param.merit_error_coeff", params_.merit_error_coeff);
  ROS_INFO_STREAM_NAMED("trajopt_param.trust_box_size", params_.trust_box_size);

  ROS_INFO_STREAM_NAMED("problem_info.basic_info.n_steps", problem_info.basic_info.n_steps);
  ROS_INFO_STREAM_NAMED("problem_info.basic_info.dt_upper_lim", problem_info.basic_info.dt_upper_lim);
  ROS_INFO_STREAM_NAMED("problem_info.basic_info.dt_lower_lim", problem_info.basic_info.dt_lower_lim);
  ROS_INFO_STREAM_NAMED("problem_info.basic_info.start_fixed", problem_info.basic_info.start_fixed);
  ROS_INFO_STREAM_NAMED("problem_info.basic_info.use_time", problem_info.basic_info.use_time);
  ROS_INFO_STREAM_NAMED("problem_info.basic_info.convex_solver", problem_info.basic_info.convex_solver);

  std::string problem_info_type;
  switch(problem_info.init_info.type)
  {
  case InitInfo::STATIONARY: problem_info_type = "STATIONARY";
    break;
  case InitInfo::JOINT_INTERPOLATED: problem_info_type = "JOINT_INTERPOLATED";
    break;
  case InitInfo::GIVEN_TRAJ: problem_info_type = "GIVEN_TRAJ";
    break;
  }
  ROS_INFO_STREAM_NAMED("problem_info.basic_info.type", problem_info_type);
  ROS_INFO_STREAM_NAMED("problem_info.basic_info.dt", problem_info.init_info.dt);

  // ROS_INFO_STREAM_NAMED("joint_pose_term_info.goal_pos.name", joint_goal_pos->name);
  // ROS_INFO_STREAM_NAMED("joint_pose_term_info.goal_pos.first_timestep", joint_goal_pos->first_step);
  // ROS_INFO_STREAM_NAMED("joint_pose_term_info.goal_pos.last_timestep", joint_goal_pos->last_step);
  // std::string goal_term_type;
  // switch(joint_goal_pos->term_type)
  // {
  // case TT_COST: goal_term_type = "TT_COST";
  //   break;
  // case TT_CNT: goal_term_type = "TT_CNT";
  //   break;
  // case TT_USE_TIME: goal_term_type = "TT_USE_TIME";
  //   break;
  // }
  // ROS_INFO_STREAM_NAMED("joint_pose_term_info.goal_pos.term_type", goal_term_type);


  //ROS_DEBUG_STREAM_NAMED("trajopt_interface", "n_steps "); why does this one not work ?????????????

  std::cout << " ======================================= Construct the problem" << std::endl;
  prob_ = ConstructProblem(problem_info);

  std::cout << " prob_->getNumCosts(): "  << prob_->getNumCosts() << std::endl;
  std::cout << " prob_->getNumConstraints(): "  << prob_->getNumConstraints() << std::endl;

  // const std::vector<sco::ConstraintPtr> prob_consts  =  prob_->getConstraints();
  // const std::vector<sco::CostPtr>& prob_costs = prob_->getCosts();
  // const std::vector<sco::ConstraintPtr>& prob_ineq_consts = prob_->getIneqConstraints();
  // const std::vector<sco::ConstraintPtr>& prob_eq_consts = prob_->getEqConstraints();
  // std::cout << " prob_consts: "  << prob_consts.size() << std::endl;
  // std::cout << " prob_costs: "  << prob_costs.size()  << std::endl;
  // std::cout << " prob_ineq_consts: "  << prob_ineq_consts.size() << std::endl;
  // std::cout << " prob_eq_consts: "  << prob_eq_consts.size() << std::endl;

  std::cout << " ======================================= TrajOpt Optimization" << std::endl;
  sco::BasicTrustRegionSQP opt(prob_);

  opt.setParameters(params_);
  opt.initialize(trajopt::trajToDblVec(prob_->GetInitTraj()));

  // Add all callbacks
  for (const sco::Optimizer::Callback& callback : callbacks_)
  {
    opt.addCallback(callback);
  }

  // Optimize
  ros::WallTime create_time = ros::WallTime::now();
  opt.optimize();

  std::cout << " ======================================= TrajOpt Solution" << std::endl;
  trajopt::TrajArray opt_solution = trajopt::getTraj(opt.x(), prob_->GetVars());
  std::cout << opt_solution << std::endl;

  // assume that the trajectory is now optimized, fill in the output structure:
  ROS_DEBUG_NAMED("trajopt_planner", "Output trajectory has %d joints", opt_solution.rows());

  res.trajectory.resize(1);
  res.trajectory[0].joint_trajectory.joint_names = joint_names;
  res.trajectory[0].joint_trajectory.header = req.start_state.joint_state.header;

  // fill in the entire trajectory
  res.trajectory[0].joint_trajectory.points.resize(opt_solution.rows());
  for (int i = 0; i < opt_solution.rows(); i++)
  {
    res.trajectory[0].joint_trajectory.points[i].positions.resize(opt_solution.cols());
    for (size_t j = 0; j < opt_solution.cols(); j++)
    {
      res.trajectory[0].joint_trajectory.points[i].positions[j] = opt_solution(i, j);
    }
    // Setting invalid timestamps.
    // Further filtering is required to set valid timestamps accounting for velocity and acceleration constraints.
    res.trajectory[0].joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
  }

  int goal_index = opt_solution.rows() - 1;
  ROS_DEBUG_NAMED("trajopt_planner", "Bottom took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  ROS_DEBUG_NAMED("trajopt_planner", "Serviced planning request in %f wall-seconds, trajectory duration is %f",
                  (ros::WallTime::now() - start_time).toSec(),
                  res.trajectory[0].joint_trajectory.points[goal_index].time_from_start.toSec());

  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.processing_time.push_back((ros::WallTime::now() - start_time).toSec());

  // // report planning failure if path has collisions
  // if (not optimizer->isCollisionFree())
  // {
  //   ROS_ERROR_STREAM_NAMED("trajopt_planner", "Motion plan is invalid.");
  //   res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
  //   return false;
  // }

  // unittesssst
  // check if final state is within goal tolerances
  kinematic_constraints::JointConstraint jc(planning_scene->getRobotModel());
  robot_state::RobotState last_state(*current_state);
  last_state.setJointGroupPositions(req.group_name, res.trajectory[0].joint_trajectory.points.back().positions);

  for (int jn = 0; jn < res.trajectory[0].joint_trajectory.points.back().positions.size(); ++jn)
  {
    std::cout << "===>>> xxxxxxxxxxxxxxxxxxxxxx ====>>> " << jn << ": " << res.trajectory[0].joint_trajectory.points.back().positions[jn]  << std::endl;
  }

  bool constraints_are_ok = true;
  int g = 0;
  for (const moveit_msgs::JointConstraint& constraint : req.goal_constraints[0].joint_constraints)
    {
      printf("index %i: jc.configure(constraint)=> %d, jc.decide(last_state).satisfied=> %d, tolerance %f\n", g,
             jc.configure(constraint), jc.decide(last_state).satisfied, constraint.tolerance_above);
      constraints_are_ok = constraints_are_ok and jc.configure(constraint);
      constraints_are_ok = constraints_are_ok and jc.decide(last_state).satisfied;
      if (not constraints_are_ok)
        {
          ROS_ERROR_STREAM_NAMED("trajopt_planner", "Goal constraints are violated: " << constraint.joint_name);
          res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED;
          return false;
        }
      g = g + 1;
    }
  std::cout << " ==================================== response" << std::endl;
  // moveit::core::robotStateMsgToRobotState(req.start_state, res.start_state);
  res.trajectory_start = req.start_state;

  std::cout << "===>>> group name:" << res.group_name << std::endl;
  std::cout << "===>>> request start state:" << req.start_state.joint_state.name.size() << std::endl;
  std::cout << "===>>> traj start joint name size: " << res.trajectory_start.joint_state.name[6] << std::endl;
  std::cout << "===>>> traj start joint position size: " << res.trajectory_start.joint_state.position.size()
            << std::endl;
  std::cout << "===>>> traj size: " << res.trajectory.size() << std::endl;
  std::cout << "===>>> traj joint names size: " << res.trajectory[0].joint_trajectory.joint_names.size() << std::endl;

  for (int jn = 0; jn < res.trajectory[0].joint_trajectory.joint_names.size(); ++jn)
  {
    std::cout << "===>>> joint_" << jn << ": " << res.trajectory[0].joint_trajectory.joint_names[jn] << std::endl;
  }

  return true;
}

void TrajOptInterface::setDefaultTrajOPtParams()
{
  sco::BasicTrustRegionSQPParameters params;
  params_ = params;
}

void TrajOptInterface::setTrajOptParams(sco::BasicTrustRegionSQPParameters& params)
{
  // std::string name = "trajopt_param";
  // ros::NodeHandle rpnh(nh_, name);
  // std::size_t error = 0;
  // double max_iter = 17;
  // error += !rosparam_shortcuts::get(name, rpnh, "max_iter", max_iter);  // Double param

  nh_.param("trajopt_param/improve_ratio_threshold", params.improve_ratio_threshold, 0.25);
  nh_.param("trajopt_param/min_trust_box_size", params.min_trust_box_size, 1e-4);
  nh_.param("trajopt_param/min_approx_improve", params.min_approx_improve, 1e-4);
  nh_.param("trajopt_param/min_approx_improve_frac", params.min_approx_improve_frac, -static_cast<double>(INFINITY));
  nh_.param("trajopt_param/max_iter", params.max_iter, 100.0);
  nh_.param("trajopt_param/trust_shrink_ratio", params.trust_shrink_ratio, 0.1);

  nh_.param("trajopt_param/trust_expand_ratio", params.trust_expand_ratio, 1.5);
  nh_.param("trajopt_param/cnt_tolerance", params.cnt_tolerance, 1e-4);
  nh_.param("trajopt_param/max_merit_coeff_increases", params.max_merit_coeff_increases, 5.0);
  nh_.param("trajopt_param/merit_coeff_increase_ratio", params.merit_coeff_increase_ratio, 10.0);
  nh_.param("trajopt_param/max_time", params.max_time, static_cast<double>(INFINITY));
  nh_.param("trajopt_param/merit_error_coeff", params.merit_error_coeff, 10.0);
  nh_.param("trajopt_param/trust_box_size", params.trust_box_size, 1e-1);
}

void TrajOptInterface::setProblemInfoParam(ProblemInfo& problem_info)
{
  // std::string name = "problem_info";
  // ros::NodeHandle rpnh(nh_, name);
  // std::size_t error = 0;
  // double n_steps = 17;
  // error += !rosparam_shortcuts::get(name, rpnh, "basic_info/n_steps", n_steps);  // Double param

  nh_.param("problem_info/basic_info/n_steps", problem_info.basic_info.n_steps, 20);
  nh_.param("problem_info/basic_info/dt_upper_lim", problem_info.basic_info.dt_upper_lim, 2.0);
  nh_.param("problem_info/basic_info/dt_lower_lim", problem_info.basic_info.dt_lower_lim, 100.0);
  nh_.param("problem_info/basic_info/start_fixed", problem_info.basic_info.start_fixed, true);
  nh_.param("problem_info/basic_info/use_time", problem_info.basic_info.use_time, false);
  int convex_solver_index;
  nh_.param("problem_info/basic_info/convex_solver", convex_solver_index, 1);
  switch (convex_solver_index)
  {
  case 1: problem_info.basic_info.convex_solver = sco::ModelType::AUTO_SOLVER;
    break;
  case 2: problem_info.basic_info.convex_solver = sco::ModelType::BPMPD;
    break;
  case 3: problem_info.basic_info.convex_solver = sco::ModelType::OSQP;
    break;
  case 4: problem_info.basic_info.convex_solver = sco::ModelType::QPOASES;
    break;
  case 5: problem_info.basic_info.convex_solver = sco::ModelType::GUROBI;
    break;
  }

  nh_.param("problem_info/init_info/dt", problem_info.init_info.dt, 0.5);
  int type_index;
  nh_.param("problem_info/init_info/type", type_index, 1);
  switch (type_index)
  {
  case 1: problem_info.init_info.type = InitInfo::STATIONARY;
    break;
  case 2: problem_info.init_info.type = InitInfo::JOINT_INTERPOLATED;
    break;
  case 3: problem_info.init_info.type = InitInfo::GIVEN_TRAJ;
    break;
  }
}

void TrajOptInterface::setJointPoseTermInfoParams(JointPoseTermInfoPtr& jp, std::string name){

    int term_type_index;
    std::string term_type_address = "joint_pos_term_info/" + name + "/term_type";
    nh_.param(term_type_address, term_type_index, 1);

    switch (term_type_index)
      {
      case 1: jp->term_type = TT_COST;
        break;
      case 2: jp->term_type = TT_CNT;
        break;
      case 3: jp->term_type = TT_USE_TIME;
        break;
      }

    std::string first_step_address = "joint_pos_term_info/" + name + "/first_timestep";
    nh_.getParam(first_step_address, jp->first_step);
    std::string last_step_address = "joint_pos_term_info/" + name + "/last_timestep";
    nh_.getParam(last_step_address, jp->last_step);
        std::string name_address = "joint_pos_term_info/" + name + "/name";
    nh_.getParam(name_address, jp->name);

    std::cout << "~~~~~~~ " << jp->first_step << " " << jp->last_step << " " << jp->name << " " << jp->term_type  << std::endl;
}

void callBackFunc(sco::OptProb* opt_prob, sco::OptResults& opt_res)
{
}

}  // namespace trajopt_interface
