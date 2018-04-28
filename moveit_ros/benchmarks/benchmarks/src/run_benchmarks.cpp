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

#include <moveit/benchmarks/benchmark_execution.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <boost/program_options.hpp>
#include <ros/ros.h>

static const std::string ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_benchmarks", ros::init_options::AnonymousName);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::program_options::options_description desc;
  desc.add_options()("help", "Show help message")("host", boost::program_options::value<std::string>(), "Host for the "
                                                                                                        "MongoDB.")(
      "port", boost::program_options::value<std::size_t>(), "Port for the MongoDB.")(
      "benchmark-goal-existance", "Benchmark the sampling of the goal region")("benchmark-planners", "Benchmark only "
                                                                                                     "the planners");

  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po = boost::program_options::parse_command_line(argc, argv, desc);
  boost::program_options::store(po, vm);
  boost::program_options::notify(vm);

  if (vm.count("help") || argc == 1)  // show help if no parameters passed
  {
    std::cout << desc << std::endl;
    return 1;
  }

  try
  {
    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
    moveit_benchmarks::BenchmarkType btype = 0;
    moveit_benchmarks::BenchmarkExecution be(psm.getPlanningScene(),
                                             vm.count("host") ? vm["host"].as<std::string>() : "",
                                             vm.count("port") ? vm["port"].as<std::size_t>() : 0);
    if (vm.count("benchmark-planners"))
      btype += moveit_benchmarks::BENCHMARK_PLANNERS;
    if (vm.count("benchmark-goal-existance"))
      btype += moveit_benchmarks::BENCHMARK_GOAL_EXISTANCE;

    unsigned int proc = 0;
    std::vector<std::string> files =
        boost::program_options::collect_unrecognized(po.options, boost::program_options::include_positional);
    for (std::size_t i = 0; i < files.size(); ++i)
    {
      if (be.readOptions(files[i].c_str()))
      {
        std::stringstream ss;
        be.printOptions(ss);
        std::cout << "Calling benchmark with options:" << std::endl << ss.str() << std::endl;
        be.runAllBenchmarks(btype);
        proc++;
      }
    }
    ROS_INFO_STREAM("Processed " << proc << " benchmark configuration files");
  }
  catch (mongo_ros::DbConnectException& ex)
  {
    ROS_ERROR_STREAM("Unable to connect to warehouse. If you just created the database, it could take a while for "
                     "initial setup. Please try to run the benchmark again."
                     << std::endl
                     << ex.what());
  }

  ROS_INFO("Benchmarks complete! Shutting down ROS...");  // because sometimes there are segfaults after this
  ros::shutdown();

  return 0;
}
