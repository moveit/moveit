#ifndef TrajOpt_PLANNING_CONTEXT_H
#define TrajOpt_PLANNING_CONTEXT_H

#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_interface/planning_interface.h>

#include <trajopt/plot_callback.hpp>
#include <trajopt/file_write_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

#include <tesseract_planning/basic_planner_types.h>

#include <trajopt_sco/solver_interface.hpp>

#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/sco_common.hpp>

#include "problem_description.h"

//using namespace trajopt;

namespace trajopt_interface
{

MOVEIT_CLASS_FORWARD(TrajOptPlanningContext);

struct TrajOptPlannerConfiguration{

  // TrajOptPlannerConfiguration(trajopt::TrajOptProbPtr prob) : prob(prob) {}

  TrajOptPlannerConfiguration() {prob.reset(new TrajOptProblem());}

  virtual ~TrajOptPlannerConfiguration() {}

  //  std::string model_type = sco::ModelType::AUTO_SOLVER
  //  sco::OptProbPtr prob = sco::OptProbPtr(new sco::OptProb(sco::ModelType::AUTO_SOLVER));

  /** @brief Trajopt problem to be solved (Required) */
  TrajOptProblemPtr prob;

  //    TrajOptProblemPtr prob(new TrajOptProblem());
  //  std::shared_ptr<TrajOptProblem> prob = std::shared_ptr(new TrajOptProblem);

  /** @brief Optimization parameters to be used (Optional) */
  sco::BasicTrustRegionSQPParameters params;

  /** @brief Callback functions called on each iteration of the optimization (Optional) */
  //std::vector<sco::Optimizer::Callback> callbacks;
  sco::Optimizer::Callback callbacks;

};

trajopt::TrajArray generateInitialTrajectory(const int& num_steps);
TrajOptPlannerConfiguration spec_;
int dof_;

void callBackFunc(sco::OptProb* oprob, sco::OptResults& ores);
 
class TrajOptPlanningContext : public planning_interface::PlanningContext
{
public:
  TrajOptPlanningContext(const std::string& name, const std::string& group,
                         const robot_model::RobotModelConstPtr& model);
  ~TrajOptPlanningContext() override
  {
  }

  bool solve(planning_interface::MotionPlanResponse& res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;
  void clear() override;

  void setTrajOptPlannerConfiguration();

protected:

private:

  moveit::core::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;

 
  
  trajectory_msgs::JointTrajectory convertTrajArrayToJointTrajectory(const trajopt::TrajArray& traj_array);

  //  sco::VarArray convertVarVectorToVarArray(const VarVector& vv);

  tesseract::tesseract_planning::PlannerRequest tess_request_;
};
} // namespace trajopt_interface

#endif  // TrajOpt_PLANNING_CONTEXT_H
