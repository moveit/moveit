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

#include <ros/ros.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/MotionPlanRequest.h>

#include <trajopt_interface.h>
#include <limits>


#include <tesseract_planning/trajopt/trajopt_planner.h>
#include <tesseract_planning/basic_planner_types.h>

#include <vector>
#include <eigen3/Eigen/Geometry>

#include "problem_description.h"

namespace trajopt_interface
{

TrajOptInterface::TrajOptInterface(const ros::NodeHandle& nh) : nh_(nh) // , chompPlanner()
{
  prob_ = TrajOptProblemPtr(new TrajOptProblem());
  // setParamsFromROSPramServer();
  setDefaultParams();
}


bool TrajOptInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req,
                             const sco::BasicTrustRegionSQPParameters& params,  moveit_msgs::MotionPlanDetailedResponse& res)
{
  ros::WallTime start_time = ros::WallTime::now();
    std::cout << " ==================================== 1 ============================================="  << std::endl;

    robot_model::RobotModelConstPtr robot_model =  planning_scene->getRobotModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    robot_state->setToDefaultValues();
    robot_state->update();
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(req.group_name);
    int dof = joint_model_group->getActiveJointModelNames().size();
    std::vector<std::string> joint_names = joint_model_group->getVariableNames();
    std::cout << " ==================================== 2 ============================================="  << std::endl;
    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);

    // Create ProblemInfo
    ProblemInfo problem_info(planning_scene, req.group_name);
    std::cout << " ==================================== 3 ============================================="  << std::endl;
    int steps_per_phase = 10;
    problem_info.basic_info.n_steps = steps_per_phase * 2;
    problem_info.basic_info.dt_upper_lim = 2.0;
    problem_info.basic_info.dt_lower_lim = 100.0;
    problem_info.basic_info.start_fixed = false; // with false, we do not get any error;
    problem_info.basic_info.use_time = false;
    problem_info.init_info.type = trajopt_interface::InitInfo::STATIONARY;
    problem_info.init_info.dt = 0.5;
    std::cout << " ==================================== 4 ============================================="  << std::endl;

    // ************************ Constraint ****************************
    // Make a constraint from req.goal_constriant to cartesian pose cnt at final point
    // I shoud see how to convert "cartesian pose cnt at final point" to a ConstriantPtr so I can
    // add that to prob_

    // planning_interface::MotionPlanRequest is the same as the one in moveit_msgs::MotionPlanRequest

    // cartesina pose cnt at a given point with CartPoseTermInfo
    std::shared_ptr<CartPoseTermInfo> pose_constraint = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
    pose_constraint->term_type = trajopt_interface::TT_CNT;
    pose_constraint->timestep = 2 * steps_per_phase - 1;
    pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "pose_" + std::to_string(2 * steps_per_phase - 1);

    // Given pose
    pose_constraint->link = joint_model_group->getEndEffectorName(); // to get the end-effector's name
    pose_constraint->xyz = Eigen::Vector3d(-0.15, 0.6, 1);;
    pose_constraint->wxyz = Eigen::Vector4d(0.0, 0.0, 1.0, 0.0);

    // From MotionPlanRequest
    // geometry_msgs::Vector3 target_position = req.goal_constraints[0].position_constraints[0].target_point_offset; // is this offset to the taretg point not the target itself?
    // pose_constraint->xyz = Eigen::Vector3d(target_position.x, target_position.x, target_position.x);
    // geometry_msgs::Quaternion target_orientation = req.goal_constraints[0].orientation_constraints[0].orientation;
    // pose_constraint->wxyz = Eigen::Vector4d(target_orientation.w, target_orientation.x, target_orientation.y, target_orientation.z);
    // pose_constraint->link = req.goal_constraints[0].position_constraints[0].link_name;

    // add the constraint to problem_info
    problem_info.cnt_infos.push_back(pose_constraint);

    // construct the problem
    prob_ = ConstructProblem(problem_info);

    /*
    // Make a constraint With constructor of JointPoseEqConstraint
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
    */

    // ************************ TrajOpt Optimization ****************************
    sco::BasicTrustRegionSQP opt(prob_);
    std::cout << " ==================================== 9 ============================================="  << std::endl;
    opt.setParameters(params_);
    std::cout << " ==================================== 10 ============================================="  << std::endl;
    opt.initialize(trajopt::trajToDblVec(prob_->GetInitTraj())); // DblVec: a vector of double elements
    // This initial traj gets passed as the solution for results_.x in optimizer::initialize
    // which will get updated by optimize() function through getClosestFeasiblePoint
    // then in sqp loop, it gets passed to convexifyCosts(const std::vector<CostPtr>& costs, const DblVec& x, Model* model)
    // where convex(x, model) function of each cost(CostFromFunc : Cost) gets called.
    // So basically, x is an element that starts with an intial value and gets updated as the solution thtough optimize()

    // Add all callbacks
    //  for (const sco::Optimizer::Callback& callback : config.callbacks)
    //  {
    std::cout << " ==================================== 11 ============================================="  << std::endl;
    callbacks_ = callBackFunc;
    opt.addCallback(callbacks_);
    // }
    std::cout << " ==================================== 12 ============================================="  << std::endl;
    // how to put ModelType, params, callback, in a request so the user can define them ????
    // I need more information than what the template request has in MoveIt

    // Optimize
    ros::WallTime create_time = ros::WallTime::now();
    ros::Time tStart = ros::Time::now();
    moveit::core::RobotState start_state(planning_scene->getCurrentState());
    moveit::core::robotStateMsgToRobotState(req.start_state, start_state);
    start_state.update();

    opt.optimize();

    std::cout << " ==================================== 20 ============================================="  << std::endl;

    // ************************ Solution ****************************
    trajopt::TrajArray opt_solution = trajopt::getTraj(opt.x(), prob_->GetVars());

    std::cout << "************* solution ************** " << std::endl;
    std::cout << opt_solution << std::endl;

    // trajectory_msgs::JointTrajectory traj_msgs = convertTrajArrayToJointTrajectory(opt_solution, joint_names);

    // robot_trajectory::RobotTrajectoryPtr traj =  robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model, req.group_name));

    // traj->setRobotTrajectoryMsg(*robot_state, traj_msgs);

    // // plot the trajectory
    // // for (int s = 0; s < traj->getWayPointCount(); ++s)
    // // {
    // //   robot_state::RobotStatePtr state = traj->getWayPointPtr(s);
    // //   joint_model_group->
    // // }

    // std::cout << "siezeeeeeeeeeeeeeeeeeeeeeeeeee " << traj->getWayPointCount()   << std::endl;

    // res.trajectory_ = traj;

    // ROS_INFO("planning time: %.3f", (ros::Time::now() - tStart).toSec());






  // assume that the trajectory is now optimized, fill in the output structure:
  ROS_DEBUG_NAMED("chomp_planner", "Output trajectory has %d joints", opt_solution.rows());

  res.trajectory.resize(1);
  //  std::vector<std::string>& active_joint_names = joint_model_group->getActiveJointModelNames();
  res.trajectory[0].joint_trajectory.joint_names = joint_names;
  res.trajectory[0].joint_trajectory.header = req.start_state.joint_state.header;  // @TODO this is probably a hack
  // fill in the entire trajectory
  res.trajectory[0].joint_trajectory.points.resize(opt_solution.rows());
  for (int i = 0; i < opt_solution.rows(); i++)
  {
    res.trajectory[0].joint_trajectory.points[i].positions.resize(opt_solution.cols());
    for (size_t j = 0; j < opt_solution.cols(); j++)
    {
      res.trajectory[0].joint_trajectory.points[i].positions[j] = opt_solution(i,j);
    }
    // Setting invalid timestamps.
    // Further filtering is required to set valid timestamps accounting for velocity and acceleration constraints.
    res.trajectory[0].joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
  }
  int goal_index = opt_solution.rows() - 1;
  ROS_DEBUG_NAMED("chomp_planner", "Bottom took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  ROS_DEBUG_NAMED("chomp_planner", "Serviced planning request in %f wall-seconds, trajectory duration is %f",
                  (ros::WallTime::now() - start_time).toSec(),
                  res.trajectory[0].joint_trajectory.points[goal_index].time_from_start.toSec());
  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.processing_time.push_back((ros::WallTime::now() - start_time).toSec());
  std::cout << " ==================================== 24 ============================================="  << std::endl;
  // // report planning failure if path has collisions
  // if (not optimizer->isCollisionFree())
  // {
  //   ROS_ERROR_STREAM_NAMED("chomp_planner", "Motion plan is invalid.");
  //   res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
  //   return false;
  // }

  // check that final state is within goal tolerances
  kinematic_constraints::JointConstraint jc(planning_scene->getRobotModel());
  robot_state::RobotState last_state(start_state);
  last_state.setVariablePositions(res.trajectory[0].joint_trajectory.joint_names,
                                  res.trajectory[0].joint_trajectory.points.back().positions);
  std::cout << " ==================================== 25 ============================================="  << std::endl;
  // uncomment this when I use req for the constraint not the hardcoded one
  // bool constraints_are_ok = true;
  // for (const moveit_msgs::JointConstraint& constraint : req.goal_constraints[0].joint_constraints)
  // {
  //   constraints_are_ok = constraints_are_ok and jc.configure(constraint);
  //   constraints_are_ok = constraints_are_ok and jc.decide(last_state).satisfied;
  //   if (not constraints_are_ok)
  //   {
  //     ROS_ERROR_STREAM_NAMED("chomp_planner", "Goal constraints are violated: " << constraint.joint_name);
  //     res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED;
  //     return false;
  //   }
  // }
  std::cout << " ==================================== 26 ============================================="  << std::endl;
  return true;
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
