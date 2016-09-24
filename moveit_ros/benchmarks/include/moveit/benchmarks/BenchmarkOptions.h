/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ryan Luna */

#ifndef MOVEIT_ROS_BENCHMARK_BENCHMARK_OPTIONS_
#define MOVEIT_ROS_BENCHMARK_BENCHMARK_OPTIONS_

#include <string>
#include <map>
#include <vector>
#include <ros/ros.h>
#include <moveit_msgs/WorkspaceParameters.h>

namespace moveit_ros_benchmarks
{
class BenchmarkOptions
{
public:
  BenchmarkOptions();
  BenchmarkOptions(const std::string& ros_namespace);
  virtual ~BenchmarkOptions();

  void setNamespace(const std::string& ros_namespace);

  const std::string& getHostName() const;
  int getPort() const;
  const std::string& getSceneName() const;

  int getNumRuns() const;
  double getTimeout() const;
  const std::string& getBenchmarkName() const;
  const std::string& getGroupName() const;
  const std::string& getOutputDirectory() const;
  const std::string& getQueryRegex() const;
  const std::string& getStartStateRegex() const;
  const std::string& getGoalConstraintRegex() const;
  const std::string& getPathConstraintRegex() const;
  const std::string& getTrajectoryConstraintRegex() const;
  void getGoalOffsets(std::vector<double>& offsets) const;
  const std::map<std::string, std::vector<std::string>>& getPlannerConfigurations() const;
  void getPlannerPluginList(std::vector<std::string>& plugin_list) const;

  const std::string& getWorkspaceFrameID() const;
  const moveit_msgs::WorkspaceParameters& getWorkspaceParameters() const;

protected:
  void readBenchmarkOptions(const std::string& ros_namespace);

  void readWarehouseOptions(ros::NodeHandle& nh);
  void readBenchmarkParameters(ros::NodeHandle& nh);
  void readPlannerConfigs(ros::NodeHandle& nh);

  void readWorkspaceParameters(ros::NodeHandle& nh);
  void readGoalOffset(ros::NodeHandle& nh);

  /// warehouse parameters
  std::string hostname_;
  int port_;
  std::string scene_name_;

  /// benchmark parameters
  int runs_;
  double timeout_;
  std::string benchmark_name_;
  std::string group_name_;
  std::string output_directory_;
  std::string query_regex_;
  std::string start_state_regex_;
  std::string goal_constraint_regex_;
  std::string path_constraint_regex_;
  std::string trajectory_constraint_regex_;
  double goal_offsets[6];

  /// planner configurations
  std::map<std::string, std::vector<std::string>> planners_;

  moveit_msgs::WorkspaceParameters workspace_;
};
}

#endif
