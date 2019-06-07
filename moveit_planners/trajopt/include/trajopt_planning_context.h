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

//using namespace trajopt;

namespace trajopt_interface
{

MOVEIT_CLASS_FORWARD(TrajOptPlanningContext);

struct TrajOptPlannerConfiguration{

  TrajOptPlannerConfiguration() {}
  virtual ~TrajOptPlannerConfiguration() {}

  /** @brief Trajopt problem to be solved (Required) */
  //  sco::OptProbPtr prob = sco::OptProbPtr(new sco::OptProb(sco::ModelType::AUTO_SOLVER));

  std::string model_type; // sco::ModelType::AUTO_SOLVER
  sco::OptProbPtr prob = sco::OptProbPtr(new sco::OptProb(model_type));


  /** @brief Optimization parameters to be used (Optional) */
  sco::BasicTrustRegionSQPParameters params;

  /** @brief Callback functions called on each iteration of the optimization (Optional) */
  //std::vector<sco::Optimizer::Callback> callbacks;
  sco::Optimizer::Callback callbacks;

};

TrajOptPlannerConfiguration spec_;

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
  //  trajectory_msgs::JointTrajectory interpolateMultDOF(const std::vector<double>& v1, const std::vector<double>& v2,
  //                                                   const int& num);
  //  std::vector<double> interpolateSingleDOF(const double& d1, const double& d2, const int& num);
   int dof;

  moveit::core::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;

  trajectory_msgs::JointTrajectory convert_TrajArray_to_JointTrajectory(const trajopt::TrajArray& traj_array);

  tesseract::tesseract_planning::PlannerRequest tess_request_;
};
} // namespace trajopt_interface

#endif  // TrajOpt_PLANNING_CONTEXT_H
