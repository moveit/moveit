
#include "trajopt_planning_context.h"
#include "moveit/planning_interface/planning_request.h"
#include "moveit/planning_interface/planning_response.h"
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/planning_scene/planning_scene.h>

#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <ros/console.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/sco_common.hpp>
TESSERACT_IGNORE_WARNINGS_POP

#include <tesseract_planning/trajopt/trajopt_planner.h>
#include <tesseract_planning/basic_planner_types.h>

#include <trajopt_sco/solver_interface.hpp>
#include <trajopt/trajectory_costs.hpp>


#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/eigen_slicing.hpp>
#include <trajopt_utils/vector_ops.hpp>


#include <vector>
#include <eigen3/Eigen/Geometry>


//using namespace trajopt;


trajopt_interface::TrajOptPlanningContext::TrajOptPlanningContext(const std::string& context_name, const std::string& group_name,
                                                                  const robot_model::RobotModelConstPtr& model)
  : planning_interface::PlanningContext(context_name, group_name), robot_model_(model)
{
  std::cout << "===>>> TrajOptPlanningContext is constructed" << std::endl;
  std::cout << "======================= group name ==================: " << group_  << std::endl;
  dof_ = robot_model_->getJointModelGroup(group_)->getActiveJointModelNames().size();
  std::cout << "dof from planning context constructor " << dof_ << std::endl;
  robot_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(robot_model_));
  // print the jointvalues here
  robot_state_->setToDefaultValues();
  robot_state_->update();

}



bool trajopt_interface::TrajOptPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res){

}



bool trajopt_interface::TrajOptPlanningContext::solve(planning_interface::MotionPlanResponse& resp)
{
  std::cout << "====> soooooooooooooooooooooooooooooolve "  << std::endl;

  //=== Response ===>>> PlannerResponse.trajectory from tesseract a is row-major matrix from Eigen
  // typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray
  // two of Eigen::Dynamic means that both row and column are dynamic. columns are the number of DOF and rows are number of timesteps
  //  Eigen::Matrix.leftCols(p) containts the first p columns
  // I have to make a function that can convert TrajArray to robot_trajectory::RobotTrajectory from planning_interface::MotionPlanResponse
  //  we can convert  moveit_msgs::RobotTrajectory to  moveit_msgs::Robot_trajectory by a function call getMessage() from planning_interface::MotionPlanResponse

  //=== Request ===>>> ConstructProblem is from trajopt_ros but still needs tesseract env
  // ConstructProblem can get ProblemConstructionInfo but this (ProblemConstructionInfo) itself has memebers that include env.
  // So It does not seem to be promising using ConstructProblem function

  //===>>>  The only thing we need to create opt with is OptProbPtr that goes to the constructor of BasicTrustRegionSQP. I do not need to be looking for env in request.
  // this env has a lot of things which changing them to planning_scene would be a lot of work. What we need is to make sco::BasicTrustRegionSQP whose constructor's input is
  // of type OptProb.
  // So I need to get a planning_interface::MotionPlanRequest and internally
  // convert it to an OptProb so I can pass it to sco::BasicTrustRegionSQP
  // in tesseract, everything is provided by config parameter, the request_ is not used as much ???
  // We can not reallu convert MotionPlanRequest to OptProb. I just set OptProb internally
  // BTW, MotionPlanRequest from planning_interface is the same as that from moveit_msgs

  // get the constraints from req and add them to spec_.prob.addCons

  // Create optimizer
  //   sco::OptProbPtr opt_prob_ptr = sco::OptProbPtr(new sco::OptProb(sco::ModelType::AUTO_SOLVER));
  //  trajopt_interface::TrajOptPlanningContext::setTrajOptPlannerConfiguration();
  

   sco::BasicTrustRegionSQP opt(spec_.prob);

   opt.setParameters(trajopt_interface::spec_.params);


   opt.initialize(trajopt::trajToDblVec(spec_.prob->GetInitTraj())); // DblVec: a vector of double elements


  // Add all callbacks
//  for (const sco::Optimizer::Callback& callback : config.callbacks)
//  {
   // callback should be created ???

   opt.addCallback(trajopt_interface::spec_.callbacks);
 // }

    // how to put ModelType, params, callback, in a request so the user can define them ????
    // I need more information than what the template request has in MoveIt

  // Optimize
  ros::Time tStart = ros::Time::now();


  std::cout << "beforeeeeeeeeeeeeeeeeeeeeeeeeeeeeee "  << std::endl;

  opt.optimize();

  std::cout << "aftreeeeeeeeeeeeeeeeeeeeeeeeeeeeeer  "  << std::endl;


  ROS_INFO("planning time: %.3f", (ros::Time::now() - tStart).toSec());


  // ????????
  // how do I pass arguments that can not be parts of MotionPlannerRequest and are not string so they could be passed
  // to the setPlannerConfigurations(). I could make some functions in TrajOptPlannerManager
  // but we can not call these functins from a poitner to the base class unless they are virtual

/*
  // Check and report collisions
  std::vector<tesseract::ContactResultMap> collisions;
  ContinuousContactManagerBasePtr manager = config.prob->GetEnv()->getContinuousContactManager();
  manager->setActiveCollisionObjects(config.prob->GetKin()->getLinkNames());
  manager->setContactDistanceThreshold(0);
  collisions.clear();
  bool found = tesseract::continuousCollisionCheckTrajectory(
      *manager, *config.prob->GetEnv(), *config.prob->GetKin(), getTraj(opt.x(), config.prob->GetVars()), collisions);

  if (found)
  {
    ROS_INFO("Final trajectory is in collision");
  }
  else
  {
    ROS_INFO("Final trajectory is collision free");
  }
*/

  // Send response
  //  response.trajectory = getTraj(opt.x(), config.prob->GetVars()); // getTraj() returns TrajArray type. This function is only dependent on trajopt_ros, no tesseract dependency
  //  response.status_code = opt.results().status;
  // response.joint_names = config.prob->GetKin()->getJointNames();
  //response.status_description = sco::statusToString(opt.results().status);

  trajopt::TrajArray opt_solution = trajopt::getTraj(opt.x(), spec_.prob->GetVars());

  std::cout << "************* solution ************** " << std::endl;
  std::cout << opt_solution << std::endl;
  // resp.trajectory_ = trajopt_interface::TrajOptPlanningContext::convertTrajArrayToJointTrajectory(opt_solution);

  trajectory_msgs::JointTrajectory traj_msgs = trajopt_interface::TrajOptPlanningContext::convertTrajArrayToJointTrajectory(opt_solution);

  
  robot_trajectory::RobotTrajectoryPtr traj =  robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model_, "panda_arm"));

  traj->setRobotTrajectoryMsg(*robot_state_, traj_msgs);

  std::cout << " wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww 1 "  << std::endl;
  
  resp.trajectory_ = traj;
  
  std::cout << " wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww 2  "  << std::endl;
  
  return true;
}

void trajopt_interface::callBackFunc(sco::OptProb* oprob, sco::OptResults& ores){
  
}

bool trajopt_interface::TrajOptPlanningContext::terminate() { /*return false;*/ }
void trajopt_interface::TrajOptPlanningContext::clear() { /*request_ = PlannerRequest();*/ }

//=== Response ===>>> PlannerResponse.trajectory from tesseract a is row-major matrix from Eigen
// typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray
// two of Eigen::Dynamic means that both row and column are dynamic. columns are the number of DOF and rows are number of timesteps
//  Eigen::Matrix.leftCols(p) containts the first p columns
// I have to make a function that can convert TrajArray to robot_trajectory::RobotTrajectory from planning_interface::MotionPlanResponse
//  we can convert  moveit_msgs::RobotTrajectory to  moveit_msgs::Robot_trajectory by a function call getMessage() from planning_interface::MotionPlanResponse

// a function to convert TrajArray (TrajArray has no dependency on tesseract) from trajopt_ros to trajectory_msgs::JointTrajectory.Points

trajectory_msgs::JointTrajectory trajopt_interface::TrajOptPlanningContext::convertTrajArrayToJointTrajectory(const trajopt::TrajArray& traj_array){
 
  trajectory_msgs::JointTrajectory traj_msgs;

  const robot_state::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(group_);
  const std::vector<std::string> j_names = joint_model_group->getVariableNames();

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

/*
void trajopt_interface::TrajOptPlanningContext::plotVector(const std::string& str, const std::vector<double>& v)
{
  std::cout << str << " ";
  for (int i = 0; i < v.size(); ++i)
  {
    std::cout << v[i] << " ";
  }
  std::cout << std::endl;
}
*/


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

  // spec_.model_type = sco::ModelType::AUTO_SOLVER;

  int num_steps = 10;
  bool u_time = false;
  trajopt_interface::TrajOptProblem traj_prob(num_steps, u_time, robot_model_);
  *spec_.prob = traj_prob;

  sco::BasicTrustRegionSQPParameters params; // all members in params are double
  // It has a constructor that sets all these paramters to default values
  /*
  params.improve_ratio_threshold = 0.25;
  params.min_trust_box_size = 1e-4;
  params.min_approx_improve = 1e-4;
  params.min_approx_improve_frac = -INFINITY;
  params.max_iter = 50;
  params.trust_shrink_ratio = 0.1;
  params.trust_expand_ratio = 1.5;
  params.cnt_tolerance = 1e-4;
  params.max_merit_coeff_increases = 5;
  params.merit_coeff_increase_ratio = 10;
  params.max_time = INFINITY;
  params.merit_error_coeff = 10;
  params.trust_box_size = 1e-1;
  */
  trajopt_interface::spec_.params = params;


  std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>. before calling generateInitialTrajectory " << dof_ << std::endl;
  trajopt::TrajArray traj_array_initial = generateInitialTrajectory(num_steps);
  std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>. after calling generateInitialTrajectory " << dof_ << std::endl;

  spec_.prob->SetInitTraj(traj_array_initial);

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

  trajopt::VarArray vars = spec_.prob->GetVars(); // columns are dof and rows are waypoints
  std::cout << "varsssssssssssssssssssssss "  << vars.at(0,0) << std::endl;
  trajopt::VarArray joint_vars = vars.block(0, 0, vars.rows(), static_cast<int>(dof_));



  sco::ConstraintPtr cptr = sco::ConstraintPtr(new trajopt::JointPosEqConstraint(joint_vars, util::toVectorXd(coeffs), util::toVectorXd(targets), first_step, last_step));

  std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  
  spec_.prob->addConstraint(cptr);


  // Make callback
  sco::Optimizer::Callback callback = callBackFunc;
  trajopt_interface::spec_.callbacks = callback;
}
