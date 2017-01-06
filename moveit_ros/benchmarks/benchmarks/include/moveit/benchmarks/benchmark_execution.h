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

/* Author: Ioan Sucan */

#ifndef MOVEIT_BENCHMARKS_BENCHMARK_EXECUTION_
#define MOVEIT_BENCHMARKS_BENCHMARK_EXECUTION_

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/planning_scene_world_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/trajectory_constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_loader.h>
#include <boost/function.hpp>
#include <iostream>

namespace moveit_benchmarks
{
typedef unsigned int BenchmarkType;
static const BenchmarkType BENCHMARK_PLANNERS = 1;
static const BenchmarkType BENCHMARK_GOAL_EXISTANCE = 2;

typedef std::map<std::string, std::string> RunData;
typedef std::vector<RunData> AlgorithmRunsData;

struct PlanningPluginOptions
{
  std::string name;
  std::vector<std::string> planners;
  std::size_t runs;
};

struct BenchmarkRequest
{
  BenchmarkRequest() : benchmark_type(0)
  {
  }

  // The scene to consider in the benchmark
  moveit_msgs::PlanningScene scene;

  // The problem to benchmarked
  moveit_msgs::MotionPlanRequest motion_plan_request;

  // The planning plugins to use in the benchmark
  std::vector<PlanningPluginOptions> plugins;

  BenchmarkType benchmark_type;

  // The file where to store the results
  std::string filename;

  // Optional: name of goal - only used for later analysis
  std::string goal_name;
};

class BenchmarkExecution
{
public:
  BenchmarkExecution(const planning_scene::PlanningScenePtr& scene, warehouse_ros::DatabaseConnection::Ptr conn);

  bool readOptions(const std::string& filename);
  void printOptions(std::ostream& out);

  void runAllBenchmarks(BenchmarkType type);

  void runBenchmark(BenchmarkRequest& req);
  void runPlanningBenchmark(BenchmarkRequest& req);
  void runGoalExistenceBenchmark(BenchmarkRequest& req);

private:
  struct BenchmarkOptions
  {
    std::string scene;
    std::string output;
    std::string start_regex;
    std::string query_regex;
    std::string goal_regex;
    std::string trajectory_regex;
    std::string group_override;
    std::string planning_frame;
    std::string default_constrained_link;
    std::size_t default_run_count;
    double offsets[6];
    double timeout;

    std::vector<PlanningPluginOptions> plugins;

    moveit_msgs::WorkspaceParameters workspace_parameters;
  };

  /// Allows for parameter sweeping of the planner configuration
  struct ParameterOptions
  {
    std::string key;
    std::string log_key;

    // Parameters for sweeping
    bool is_sweep;
    double start;
    double step_size;
    double end;

    // Parameters for fractional factorial analysis (design of experience)
    double high;
    double low;
  };

  /// Contains the parameter combination for one test
  typedef std::map<std::string, double> ParameterInstance;

  void collectMetrics(std::map<std::string, std::string>& rundata,
                      const planning_interface::MotionPlanDetailedResponse& mp_res, bool solved, double total_time);

  /**
   * @brief Called within the benchmarking solve loop to allow parameters to be swept/tested
   * @param planner - pointer to the current planner we are about to use
   * @param planner_id - name of planner we are about to use
   * @param param_combinations_id_ - keeps track of what parameter combo we are currently iterating on
   * @param parameter_data - used for outputting log information to file (results)
   */
  void modifyPlannerConfiguration(planning_interface::PlannerManager& planner, const std::string& planner_id,
                                  std::size_t param_combinations_id_, RunData& parameter_data);

  /**
   * @brief Populates the param_combinations_ vector with all combinations of desired parameters to be tested
   * @return number of combinations to be tested
   */
  std::size_t generateParamCombinations();

  /**
   * @brief Recursively generates all the combinations of parameters to be tested
   * @param options_id - where in the recursive loop we are, id is with respect to the n*n*n*... number of tests we are
   * to generate
   * @param param_instance - holds the generated parameter combinations, the result
   */
  void recursiveParamCombinations(int options_id, ParameterInstance param_instance);

  /// Output to console the settings
  void printConfigurationSettings(const planning_interface::PlannerConfigurationMap& settings, std::ostream& out);

  BenchmarkOptions options_;
  std::vector<ParameterOptions> param_options_;
  std::vector<ParameterInstance> param_combinations_;

  std::vector<std::string> available_plugins_;
  planning_scene::PlanningScenePtr planning_scene_;

  moveit_warehouse::PlanningSceneStorage pss_;
  moveit_warehouse::PlanningSceneWorldStorage psws_;
  moveit_warehouse::ConstraintsStorage cs_;
  moveit_warehouse::TrajectoryConstraintsStorage tcs_;
  moveit_warehouse::RobotStateStorage rs_;

  boost::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader_;
  std::map<std::string, planning_interface::PlannerManagerPtr> planner_interfaces_;
};
}

#endif
