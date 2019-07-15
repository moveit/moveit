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

#include <trajopt_interface.h>
#include <limits>

namespace trajopt_interface
{

TrajOptInterface::TrajOptInterface(){
}
  
TrajOptInterface::TrajOptInterface(const ros::NodeHandle& nh) : nh_(nh) // , chompPlanner()
{
 
  setTrajOptPlannerConfiguration();
}


bool TrajOptInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit_msgs::MotionPlanRequest& req,
	       const TrajOptParameters& params, moveit_msgs::MotionPlanResponse& res) const
  {
    sco::BasicTrustRegionSQP opt(prob_);

    opt.setParameters(trajopt_interface::params_);


    opt.initialize(trajopt::trajToDblVec(prob_->GetInitTraj())); // DblVec: a vector of double elements


    // Add all callbacks
    //  for (const sco::Optimizer::Callback& callback : config.callbacks)
    //  {
    // callback should be created ???

    opt.addCallback(trajopt_interface::callbacks_);
    // }

    // how to put ModelType, params, callback, in a request so the user can define them ????
    // I need more information than what the template request has in MoveIt

    // Optimize
    ros::Time tStart = ros::Time::now();


    std::cout << "beforeeeeeeeeeeeeeeeeeeeeeeeeeeeeee "  << std::endl;

    opt.optimize();

    std::cout << "aftreeeeeeeeeeeeeeeeeeeeeeeeeeeeeer  "  << std::endl;


    ROS_INFO("planning time: %.3f", (ros::Time::now() - tStart).toSec());

}


// inspired by the same functin in trajopt/trajopt/problem_description.cpp
trajopt::TrajArray trajopt_interface::generateInitialTrajectory(const int& num_steps){

    // Stationary initial trajectory
    // we can get the current joint values here
    Eigen::VectorXd start_pos(dof_); // dof

    for (int k = 0; k < dof_; ++k){
      start_pos[k] = 0;
    }

    trajopt::TrajArray init_traj = start_pos.transpose().replicate(num_steps,1); // replicate(n_steps,1)

    return init_traj;
  }

void trajopt_interface::TrajOptPlanningContext::setTrajOptPlannerConfiguration(){

    sco::BasicTrustRegionSQPParameters params; // all members in params are double
    trajopt_interface::spec_.params = params;

    // not sure shoul I sue the one above or the one below ??????
  
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

  
    // spec_.model_type = sco::ModelType::AUTO_SOLVER;

    int num_steps = 10;
    bool u_time = false;
    trajopt_interface::TrajOptProblem traj_prob(num_steps, u_time, robot_model_);
    *prob_ = traj_prob;

    trajopt::TrajArray traj_array_initial = generateInitialTrajectory(num_steps);

    prob_->SetInitTraj(traj_array_initial);

    // cartesina pose cnt at final oint
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

    // DblVec => std::vector<double>
    sco::DblVec coeffs(dof_, 1);
    sco::DblVec targets(dof_, 1);
    int first_step = 0;
    int last_step = 0; // what are these first and last for

    trajopt::VarArray vars = prob_->GetVars(); // columns are dof and rows are waypoints
    std::cout << "varsssssssssssssssssssssss "  << vars.at(0,0) << std::endl;
    trajopt::VarArray joint_vars = vars.block(0, 0, vars.rows(), static_cast<int>(dof_));



    sco::ConstraintPtr cptr = sco::ConstraintPtr(new trajopt::JointPosEqConstraint(joint_vars,
							    util::toVectorXd(coeffs), util::toVectorXd(targets), first_step, last_step));

    prob_->addConstraint(cptr);


    // Make callback
    sco::Optimizer::Callback callback = callBackFunc;
    trajopt_interface::callbacks_ = callback;
}
  
void trajopt_interface::callBackFunc(sco::OptProb* oprob, sco::OptResults& ores){

}

  
  
}  // namespace trajopt_interface
