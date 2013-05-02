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
 *   * Neither the name of the Willow Garage nor the names of its
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

  BenchmarkExecution(const planning_scene::PlanningScenePtr &scene, const std::string &host, std::size_t port);

  bool readOptions(const std::string &filename);
  void printOptions(std::ostream &out);

  void runAllBenchmarks(BenchmarkType type);

  void runBenchmark(BenchmarkRequest &req);
  void runPlanningBenchmark(BenchmarkRequest &req);
  void runGoalExistenceBenchmark(BenchmarkRequest &req);

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
  struct FactorOptions
  {
    std::string key;
    std::string log_key;

    // Parameters for sweeping
    bool isSweep;
    double start;
    double iterator;
    double end;

    // Parameters for fractional factorial analysis (design of experience)
    double high;
    double low;
  };

  /// Contains the parameter combination for one test
  typedef std::map<std::string,double> FactorInstance;

  void collectMetrics(std::map<std::string, std::string> &rundata,
                      const planning_interface::MotionPlanDetailedResponse &mp_res,
                      bool solved, double total_time);

  /**
   * @brief Called within the benchmarking solve loop to allow parameters to be swept/tested
   * @param planner - pointer to the current planner we are about to use
   * @param planner_id - name of planner we are about to use
   * @param factor_combinations_id_ - keeps track of what parameter combo we are currently iterating on
   * @param parameter_data - used for outputting log information to file (results)
   */
  void modifyPlannerConfiguration(planning_interface::Planner* planner,
                                  const std::string& planner_id,
                                  std::size_t factor_combinations_id_,
                                  RunData &parameter_data);

  /**
   * @brief Populates the factor_combinations_ vector with fractional factorial combination
   * @return number of combinations to be tested
   */
  std::size_t generateFactorCombinationsHack();

  /**
   * @brief Populates the factor_combinations_ vector with all combinations of desird factors to be tested
   * @return number of combinations to be tested
   */
  std::size_t generateFactorCombinations();

  /// Recursively generates all the combinations of parameters to be tested
  void recursiveFactorCombinations(int options_id, FactorInstance factor_instance);

  /// Output to console the settings
  void printConfigurationSettings(const std::map<std::string,planning_interface::PlanningConfigurationSettings> &settings);

  BenchmarkOptions options_;
  std::vector<FactorOptions> factor_options_;
  std::vector<FactorInstance> factor_combinations_;

  std::vector<std::string> available_plugins_;
  planning_scene::PlanningScenePtr planning_scene_;

  moveit_warehouse::PlanningSceneStorage pss_;
  moveit_warehouse::PlanningSceneWorldStorage psws_;
  moveit_warehouse::ConstraintsStorage cs_;
  moveit_warehouse::TrajectoryConstraintsStorage tcs_;
  moveit_warehouse::RobotStateStorage rs_;

  boost::shared_ptr<pluginlib::ClassLoader<planning_interface::Planner> > planner_plugin_loader_;
  std::map<std::string, boost::shared_ptr<planning_interface::Planner> > planner_interfaces_;

};


}

#endif
