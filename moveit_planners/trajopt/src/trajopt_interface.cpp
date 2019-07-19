/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

#include "moveit/planning_interface/planning_request.h"
#include "moveit/planning_interface/planning_response.h"
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/planning_scene/planning_scene.h>

#include <trajopt_sco/sco_common.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/solver_interface.hpp>

#include <trajopt/plot_callback.hpp>
#include <trajopt/file_write_callback.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/eigen_slicing.hpp>
#include <trajopt_utils/vector_ops.hpp>

#include <trajopt_sco/solver_interface.hpp>
#include <trajopt/trajectory_costs.hpp>


#include <trajopt_interface.h>
#include <limits>


#include <tesseract_planning/trajopt/trajopt_planner.h>
#include <tesseract_planning/basic_planner_types.h>

#include <vector>
#include <eigen3/Eigen/Geometry>



namespace trajopt_interface
{

TrajOptInterface::TrajOptInterface(const ros::NodeHandle& nh) : nh_(nh) // , chompPlanner()
{
  prob_ = TrajOptProblemPtr(new TrajOptProblem());
  // setParamsFromROSPramServer();
  setDefaultParams();
}


bool TrajOptInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit_msgs::MotionPlanRequest& req,
                             const sco::BasicTrustRegionSQPParameters& params, planning_interface::MotionPlanResponse& res)
{
    // spec_.model_type = sco::ModelType::AUTO_SOLVER;

    robot_model::RobotModelConstPtr robot_model =  planning_scene->getRobotModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    robot_state_->setToDefaultValues();
    robot_state_->update();
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(req.group_name);
    int dof = joint_model_group->getActiveJointModelNames().size();
    std::vector<std::string> joint_names = joint_model_group->getVariableNames();

    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);

    // Create ProblemInfo
    ProblemInfo problem_info;

    int steps_per_phase = 10;
    problem_info.num_steps = steps_per_phase * 2;
    problem_info.dt_upper_lim = 2.0;
    problem_info.dt_lower_lim = 100.0;
    problem_info.start_fixed = true;
    problem_info.init_info.type = trajopt::InitInfo::STATIONARY;
    problem_info.init_info.dt = 0.5;

    bool u_time = false;
    trajopt_interface::TrajOptProblem traj_prob(problem_info, joint_model_group);
    *prob_ = traj_prob;
    // Set the initial trajectory
    trajopt::TrajArray traj_array_initial = generateInitialTrajectory(problem_info.num_steps, joint_values); // it can be improved by getting the curren state joint positions
    prob_->SetInitTraj(traj_array_initial);


    // ************************ Constraint ****************************
    // Make a constraint from req.goal_constriant to cartesian pose cnt at final point
    // I shoud see how to convert "cartesian pose cnt at final point" to a ConstriantPtr so I can
    // add that to prob_

    // cartesina pose cnt at final point with CartPoseTermInfo
    /* Eigen::Quaterniond rotation(final_pose.linear());
       std::shared_ptr<trajopt::CartPoseTermInfo> pose_constraint =
       std::shared_ptr<trajopt::CartPoseTermInfo>(new trajopt::CartPoseTermInfo);
       pose_constraint->term_type = trajopt::TT_CNT;
       pose_constraint->link = end_effector;
       pose_constraint->timestep = 2 * steps_per_phase - 1;
       pose_constraint->xyz = final_pose.translation();

       pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
       pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
       pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
       pose_constraint->name = "pose_" + std::to_string(2 * steps_per_phase - 1);
       pci.cnt_infos.push_back(pose_constraint);
    */

    // With constructor of JointPoseEqConstraint
    // DblVec => std::vector<double>
    sco::DblVec coeffs(dof, 1); // dof is the size of the vector and 1 is the value of it
    //coeffs = {1, 1.5, 2, 0.4, 0.9, 0.5, 2.1};
    sco::DblVec targets(dof, 1);
    //    targets = {0.5, 0.9, 1, 1.2, 0.6, 0.8, 0.7};
    int first_step = 0;
    int last_step = 0; // what are these first and last for
    trajopt::VarArray vars = prob_->GetVars(); // columns are dof and rows are waypoints
    trajopt::VarArray joint_vars = vars.block(0, 0, vars.rows(), static_cast<int>(dof));
    sco::ConstraintPtr cptr = sco::ConstraintPtr(new trajopt::JointPosEqConstraint(joint_vars, util::toVectorXd(coeffs), util::toVectorXd(targets), first_step, last_step));

    prob_->addConstraint(cptr);

    // ************************ TrajOpt Optimization ****************************
    sco::BasicTrustRegionSQP opt(prob_);

    opt.setParameters(params_);

    opt.initialize(trajopt::trajToDblVec(prob_->GetInitTraj())); // DblVec: a vector of double elements
    // This initial traj gets passed as the solution for results_.x in optimizer::initialize
    // which will get updated by optimize() function through getClosestFeasiblePoint
    // then in sqp loop, it gets passed to convexifyCosts(const std::vector<CostPtr>& costs, const DblVec& x, Model* model)
    // where convex(x, model) function of each cost(CostFromFunc : Cost) gets called.
    // So basically, x is an element that starts with an intial value and gets updated as the solution thtough optimize()

    // Add all callbacks
    //  for (const sco::Optimizer::Callback& callback : config.callbacks)
    //  {

    callbacks_ = callBackFunc;
    opt.addCallback(callbacks_);
    // }

    // how to put ModelType, params, callback, in a request so the user can define them ????
    // I need more information than what the template request has in MoveIt

    // Optimize
    ros::Time tStart = ros::Time::now();
    opt.optimize();


    // ************************ Solution ****************************
    trajopt::TrajArray opt_solution = trajopt::getTraj(opt.x(), prob_->GetVars());

    std::cout << "************* solution ************** " << std::endl;
    std::cout << opt_solution << std::endl;

    trajectory_msgs::JointTrajectory traj_msgs = convertTrajArrayToJointTrajectory(opt_solution, joint_names);

    robot_trajectory::RobotTrajectoryPtr traj =  robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model, req.group_name));

    traj->setRobotTrajectoryMsg(*robot_state, traj_msgs);

    res.trajectory_ = traj;

    ROS_INFO("planning time: %.3f", (ros::Time::now() - tStart).toSec());
}


// Inspired by the same function in trajopt/trajopt/problem_description.cpp
trajopt::TrajArray TrajOptInterface::generateInitialTrajectory(const int& num_steps, const std::vector<double>& joint_vals)
{

  // Stationary initial trajectory
  int dof = joint_vals.size();
  Eigen::VectorXd start_pos(dof);

    for (int k = 0; k < dof; ++k){
      start_pos[k] = joint_vals[k];
    }

    trajopt::TrajArray init_traj = start_pos.transpose().replicate(num_steps,1);
    return init_traj;
}

void TrajOptInterface::setParamsFromRosParamServer(){

  nh_.param("improve_ratio_threshold", params_.improve_ratio_threshold, 0.25);
  nh_.param("min_trust_box_size", params_.min_trust_box_size, 1e-4);
  nh_.param("min_approx_improve", params_.min_approx_improve, 1e-4);
  nh_.param("min_approx_improve_frac", params_.min_approx_improve_frac, -static_cast<double>(INFINITY));
  nh_.param("max_iter", params_.max_iter, 50.0);
  nh_.param("trust_shrink_ratio", params_.trust_shrink_ratio, 0.1);

  nh_.param("trust_expand_ratio", params_.trust_expand_ratio, 1.5);
  nh_.param("cnt_tolerance", params_.cnt_tolerance, 1e-4);
  nh_.param("max_merit_coeff_increases", params_.max_merit_coeff_increases, 5.0);
  nh_.param("merit_coeff_increase_ratio", params_.merit_coeff_increase_ratio, 10.0);
  nh_.param("max_time", params_.max_time, static_cast<double>(INFINITY));
  nh_.param("merit_error_coeff", params_.merit_error_coeff, 10.0);
  nh_.param("trust_box_size",  params_.trust_box_size, 1e-1);
}

void TrajOptInterface::setDefaultParams(){
    sco::BasicTrustRegionSQPParameters params;
    params_ = params;
}


trajectory_msgs::JointTrajectory TrajOptInterface::convertTrajArrayToJointTrajectory(const trajopt::TrajArray& traj_array, const std::vector<std::string>& j_names){

  trajectory_msgs::JointTrajectory traj_msgs;

  double timesteps_num =  traj_array.rows();
  double dofs_num =  traj_array.cols();

  traj_msgs.points.resize(timesteps_num);


  for (int step = 0; step < timesteps_num ; ++step)
  {
    std::vector<double> joints_values;
    for(int joint_at_step = 0; joint_at_step < dofs_num; ++joint_at_step){
      joints_values.push_back(traj_array(step,joint_at_step));
    }

    //    plotVector("===>>> ", v);
    traj_msgs.joint_names = j_names;
    traj_msgs.points[step].positions = joints_values;
    //    ros::Duration t(step * 0.5);
    //traj_msgs.points[step].time_from_start = t;
  }

  return traj_msgs;
}


void callBackFunc(sco::OptProb* opt_prob, sco::OptResults& opt_res)
{

}


}  // namespace trajopt_interface
