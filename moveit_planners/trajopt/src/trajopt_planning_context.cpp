

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

#include "trajopt_planning_context.h"
#include "trajopt_interface.h"

namespace trajopt_interface{

TrajOptPlanningContext::TrajOptPlanningContext(const std::string& context_name, const std::string& group_name,
                                                                  const robot_model::RobotModelConstPtr& model)
  : planning_interface::PlanningContext(context_name, group_name), robot_model_(model)
{
  std::cout << "===>>> TrajOptPlanningContext is constructed" << std::endl;

  trajopt_interface_ = TrajOptInterfacePtr(new TrajOptInterface());
}

bool TrajOptPlanningContext::solve(planning_interface::MotionPlanResponse& resp)
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
  //  TrajOptPlanningContext::setTrajOptPlannerConfiguration();


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

  bool trajopt_solve = trajopt_interface_->solve(planning_scene_, request_, trajopt_interface_->getParams() , resp);

   if(trajopt_solve){

     /*     resp.trajectory_ = res_msg; // traj

     res.description_.push_back("plan");
     res.processing_time_ = res_msg.processing_time;
     res.error_code_ = res_msg.error_code;*/
    return true;

   }else
   {
     //    res.error_code_ = res_msg.error_code;
    return false;
   }
}


bool TrajOptPlanningContext::terminate() { /*return false;*/ }
void TrajOptPlanningContext::clear() { /*request_ = PlannerRequest();*/ }

bool TrajOptPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res){
}


}  // namespace trajopt_interface
