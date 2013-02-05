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

#ifndef MOVEIT_BENCHMARKS_BENCHMARKS_CONFIG_
#define MOVEIT_BENCHMARKS_BENCHMARKS_CONFIG_

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/planning_scene_world_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/trajectory_constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit_msgs/ComputePlanningPluginsBenchmark.h>
#include <boost/function.hpp>

namespace moveit_benchmarks
{

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

  struct PluginOptions
  {
    std::string name;
    std::vector<std::string> planners;
    std::size_t runs;
  };
  
  std::vector<PluginOptions> plugins;
};

typedef boost::function<bool(moveit_msgs::ComputePlanningPluginsBenchmark::Request&)> BenchmarkCallFn;

class BenchmarkConfig
{
public:

  BenchmarkConfig(const std::string &host, std::size_t port);
  
  bool readOptions(const char *filename);
  
  void runBenchmark(const BenchmarkCallFn &call);

  const BenchmarkOptions& getOptions() const
  {
    return opt_;
  }
  
  void printOptions(std::ostream &out);
  
private:
  
  BenchmarkOptions opt_;
  moveit_warehouse::PlanningSceneStorage pss_; 
  moveit_warehouse::PlanningSceneWorldStorage psws_;
  moveit_warehouse::ConstraintsStorage cs_;
  moveit_warehouse::TrajectoryConstraintsStorage tcs_;
  moveit_warehouse::RobotStateStorage rs_;
};


}

#endif
