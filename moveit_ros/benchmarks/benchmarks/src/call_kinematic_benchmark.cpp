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

/* Author: Ioan Sucan, Mario Prats */

#include <ros/ros.h>
#include <moveit/benchmarks/benchmarks_config.h>
#include <moveit/planning_scene/planning_scene.h>
#include <boost/program_options/variables_map.hpp>

namespace moveit_benchmarks
{
std::vector<std::string> benchmarkGetAvailablePluginNames(void);
const std::string BENCHMARK_SERVICE_NAME = "benchmark_planning_problem";
const std::string BENCHMARK_KINEMATICS_SERVICE_NAME = "benchmark_kinematic_problem";
}


int main(int argc, char **argv)
{
  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("host", boost::program_options::value<std::string>(), "Host for the MongoDB.")
    ("port", boost::program_options::value<std::size_t>(), "Port for the MongoDB.")
    ("include-planners", "Benchmark the planners if IK is successful");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  bool include_planners = false;
  if (vm.count("include-planners"))
  {
    include_planners = true;
  }

  ros::init(argc, argv, "call_moveit_kinematic_benchmark", ros::init_options::AnonymousName);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit_benchmarks::BenchmarkConfig bc(vm.count("host") ? vm["host"].as<std::string>() : "",
                                        vm.count("port") ? vm["port"].as<std::size_t>() : 0);

  std::vector<std::string> plugins = moveit_benchmarks::benchmarkGetAvailablePluginNames();

  ros::NodeHandle nh;
  ros::service::waitForService(moveit_benchmarks::BENCHMARK_KINEMATICS_SERVICE_NAME);
  ros::ServiceClient benchmark_kinematics_service_client = nh.serviceClient<moveit_msgs::ComputePlanningPluginsBenchmark>(moveit_benchmarks::BENCHMARK_KINEMATICS_SERVICE_NAME, true);
  ros::ServiceClient benchmark_service_client;
  if (include_planners)
  {
    ros::service::waitForService(moveit_benchmarks::BENCHMARK_SERVICE_NAME);
    benchmark_service_client = nh.serviceClient<moveit_msgs::ComputePlanningPluginsBenchmark>(moveit_benchmarks::BENCHMARK_SERVICE_NAME, true);
  }

  if (plugins.empty())
    ROS_ERROR("There are no plugins to benchmark.");
  else
  {
    unsigned int proc = 0;
    for (int i = 1 ; i < argc ; ++i)
    {
      if (bc.readOptions(argv[i]))
      {
        std::stringstream ss;
        bc.printOptions(ss);
        ROS_INFO("Calling kinematic benchmark with options:\n%s\n", ss.str().c_str());

        const std::vector<moveit_msgs::ComputePlanningPluginsBenchmark::Request>& req_list = bc.getBenchmarkRequests();
        for (std::size_t r = 0; r < req_list.size(); r++)
        {
          moveit_msgs::ComputePlanningPluginsBenchmark::Request res;
          ROS_INFO("Calling kinematic benchmark...");
          if (benchmark_kinematics_service_client.call(req_list[r], res))
          {
            ROS_INFO("Success!");
            if (include_planners)
            {
              ROS_INFO("Calling planner benchmark...");
              if (benchmark_service_client.call(req_list[r], res))
              {
                ROS_INFO("Success!");
              }
              else
                ROS_ERROR("Failed");
            }
          }
          else
            ROS_ERROR("Failed");
        }
        proc++;
      }
    }
    ROS_INFO("Processed %u benchmark configuration files", proc);
  }

  return 0;
}

