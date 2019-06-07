
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

#include <vector>

//using namespace trajopt;


trajopt_interface::TrajOptPlanningContext::TrajOptPlanningContext(const std::string& context_name, const std::string& group_name,
                                                                  const robot_model::RobotModelConstPtr& model)
  : planning_interface::PlanningContext(context_name, group_name), robot_model_(model)
{
  std::cout << "===>>> TrajOptPlanningContext is constructed" << std::endl;

  dof = robot_model_->getJointModelGroup(group_)->getActiveJointModelNames().size();
  robot_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(robot_model_));
  // plot the jointvalues here
  robot_state_->setToDefaultValues();
  robot_state_->update();

}


/*
bool TrajOptPlanningContext::solve(planning_interface::MotionPlanResponse& resp)
{
  std::cout << "====>>> solve() is called" << std::endl;

  // get the start state joint values
  std::vector<double> start_joint_values = request_.start_state.joint_state.position;

  std::vector<moveit_msgs::Constraints> goal_constraints = request_.goal_constraints;

  std::vector<moveit_msgs::JointConstraint> goal_joint_constraint =
      goal_constraints[0].joint_constraints;

  std::vector<double> goal_joint_values;
  for (auto x : goal_joint_constraint)
  {
    goal_joint_values.push_back(x.position);
  }

  resp.trajectory_ = robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model_, group_));

  trajectory_msgs::JointTrajectory rob_joint_traj = interpolateMultDOF(start_joint_values, goal_joint_values, 40);

  resp.trajectory_->setRobotTrajectoryMsg(*robot_state_, rob_joint_traj);

  return true;
};
*/

 /*
// solve function from tesseract
bool trajopt_interface::TrajOptPlanningContext::solve(planning_interface::MotionPlanResponse& response)
{
  Json::Value root;
  Json::Reader reader;
  if (request_.config_format == "json")
  {
    bool parse_success = reader.parse(request_.config.c_str(), root);
    if (!parse_success)
    {
      ROS_FATAL("Failed to pass valid json file in the request");
      // response.error_code = -2;
      // response.status_description = status_code_map_[-2];
      return false;
    }
  }
  else
  {
    ROS_FATAL("Invalid config format: %s. Only json format is currently "
              "support for this planner.",
              request_.config_format.c_str());
    // response.status_code = -1;
    // response.status_description = status_code_map_[-1];
    return false;
  }

  TrajOptProbPtr prob = ConstructProblem(root, request_.env);
  return solve(response, prob);
}
*/

bool trajopt_interface::TrajOptPlanningContext::solve(planning_interface::MotionPlanResponse& resp)
{

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

  // Create optimizer
  //   sco::OptProbPtr opt_prob_ptr = sco::OptProbPtr(new sco::OptProb(sco::ModelType::AUTO_SOLVER));




   sco::BasicTrustRegionSQP opt(spec_.prob);

   opt.setParameters(trajopt_interface::spec_.params);

   // I think this trajectory (as the initial) can be kept empty
   trajopt::TrajArray m_init_traj;
   opt.initialize(trajopt::trajToDblVec(m_init_traj)); // DblVec: a vector of double elements

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
  opt.optimize();
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

  return true;
}
/*
bool trajopt_interface::TrajOptPlanningContext::terminate() { return false; }
void trajopt_interface::TrajOptPlanningContest::clear() { request_ = PlannerRequest(); }
*/

// a function to convert TrajArray (TrajArray has no dependency on tesseract) from trajopt_ros to trajectory_msgs::JointTrajectory.Points

trajectory_msgs::JointTrajectory trajopt_interface::TrajOptPlanningContext::convert_TrajArray_to_JointTrajectory(const trajopt::TrajArray& traj_array){

  trajectory_msgs::JointTrajectory joint_traj;
  return joint_traj;
}










/*
trajectory_msgs::JointTrajectory trajopt_interface::TrajOptPlanningContext::interpolateMultDOF(const std::vector<double>& v1,
                                                                            const std::vector<double>& v2,
                                                                            const int& num)
{
  trajectory_msgs::JointTrajectory traj;

  const robot_state::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(group_);

  const std::vector<std::string> j_names = joint_model_group->getVariableNames();

  // robot_model_->getJointModelGroup(group_);

  std::cout << "===>>> degrees of freedom " << dof << std::endl;

  traj.points.resize(num + 1);
  std::cout << "===>>> traj.point.size: " << traj.points.size() << std::endl;

  std::vector<double> dt_vector;
  for (int j = 0; j < dof; ++j)
  {
    double dt = (v2[j] - v1[j]) / num;
    dt_vector.push_back(dt);
  }

  for (int i = 0; i <= num; ++i)
  {
    std::vector<double> v;
    for (int k = 0; k < dof; ++k)
    {
      double j_value = v1[k] + i * dt_vector[k];
      v.push_back(j_value);

      robot_state_->setJointPositions(j_names[k], &j_value);
      robot_state_->update();
    }
    bool isValid = planning_scene_->isStateValid(*robot_state_, group_, false);
    printf("the robot at state %i is valid ? %s", i, isValid ? "true" : "false \n");

    plotVector("===>>> ", v);
    traj.joint_names = j_names;
    traj.points[i].positions = v;
    ros::Duration t(i * 0.5);
    traj.points[i].time_from_start = t;
  }

  return traj;
}


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

<<<<<<< HEAD

void trajopt_interface::TrajOptPlanningContext::registerDefaultPlanners(){

}



=======
// Create parameters, callback functions and
void trajopt_interface::TrajOptPlanningContext::setTrajOptPlannerConfiguration(){

  spec_.model_type = sco::ModelType::AUTO_SOLVER;

  sco::BasicTrustRegionSQPParameters params; // all members in params are double

  params.improve_ratio_threshold = 0.9;  // minimum ratio true_improve/approx_improve
                                   // to accept step
  params.min_trust_box_size = 0.2;       // if trust region gets any smaller, exit and
                                   // report convergence
  params.min_approx_improve = 0.1;       // if model improves less than this, exit and
                                   // report convergence
  params.min_approx_improve_frac = 0.1;  // if model improves less than this, exit and
                                   // report convergence
  params.max_iter = 1000;                 // The max number of iterations
  params.trust_shrink_ratio = 0.5;       // if improvement is less than
  // improve_ratio_threshold, shrink trust region by
  // this ratio
  params.trust_expand_ratio = 0.5;  // if improvement is less than
                              // improve_ratio_threshold, shrink trust region by
                              // this ratio
  params.cnt_tolerance = 0.2;       // after convergence of penalty subproblem, if
  // constraint violation is less than this, we're done
  params. max_merit_coeff_increases = 10;   // number of times that we jack up penalty
                                      // coefficient
  params.merit_coeff_increase_ratio = 2;  // ratio that we increate coeff each time
  params.max_time = 60;                    // not yet implemented
  params.merit_error_coeff = 3;           // initial penalty coefficient
  params.trust_box_size = 5;              // current size of trust region (component-wise)

  trajopt_interface::spec_.params = params;



  sco::Optimizer::Callback callback;
  trajopt_interface::spec_.callbacks = callback;

}
>>>>>>> got rid of "using" namespace trajopt
