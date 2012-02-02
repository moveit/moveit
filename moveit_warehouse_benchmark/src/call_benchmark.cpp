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

#include <moveit_warehouse/warehouse.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/ComputePlanningBenchmark.h>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/variables_map.hpp>

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string BENCHMARK_SERVICE_NAME="benchmark_service";
static const std::string PLANNING_SCENE_TOPIC="planning_scene";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "call_warehouse_benchmark", ros::init_options::AnonymousName);
  
  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("attempts", boost::program_options::value<unsigned int>(), "Number of times to execute a query.")
    ("runtime", boost::program_options::value<double>(), "Amount of time (seconds) to allow planning for each attempt.");
  
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }
    
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::NodeHandle nh;
  tf::TransformListener tf;
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, &tf);
  moveit_warehouse::PlanningSceneStorage pss;
  
  std::vector<std::string> names;
  pss.getPlanningSceneNames(names);
  if (names.empty())
    ROS_INFO("There are no previously stored scenes");
  else
  {
    ROS_INFO("Planning scenes to be benchmarked:");
    for (std::size_t i = 0 ; i < names.size() ; ++i)
      ROS_INFO(" * %s", names[i].c_str());
    
    ros::service::waitForService(BENCHMARK_SERVICE_NAME);
    ros::ServiceClient benchmark_service_client = nh.serviceClient<moveit_msgs::ComputePlanningBenchmark>(BENCHMARK_SERVICE_NAME);
    ros::Publisher pub_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    sleep(1);
    
    for (std::size_t i = 0 ; i < names.size() ; ++i)
    {
      std::vector<moveit_warehouse::MotionPlanRequestWithMetadata> planning_queries;
      pss.getPlanningQueries(planning_queries, names[i]);
      if (planning_queries.empty())
      {
        ROS_INFO("There are no planning queries for scene '%s'", names[i].c_str());
        continue;
      }
      else
        ROS_INFO("%u queries found for scene '%s'", (unsigned int)planning_queries.size(), names[i].c_str());
      moveit_warehouse::PlanningSceneWithMetadata pswm;
      pss.getPlanningScene(pswm, names[i]);
      ROS_INFO("Publishing scene '%s'", names[i].c_str());
      pub_scene.publish(static_cast<const moveit_msgs::PlanningScene&>(*pswm));
      sleep(1);
      for (std::size_t j = 0 ; j < planning_queries.size() ; ++j)
      {
        moveit_msgs::ComputePlanningBenchmark::Request mplan_req;
        moveit_msgs::ComputePlanningBenchmark::Response mplan_res;
        mplan_req.motion_plan_request = *planning_queries[j];
        if (vm.count("attempts"))
          mplan_req.motion_plan_request.num_planning_attempts = vm["attempts"].as<unsigned int>();
        if (vm.count("runtime"))
          mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(vm["runtime"].as<double>());
        benchmark_service_client.call(mplan_req, mplan_res);
        if (mplan_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
          ROS_WARN("Benchmark service failed on request %u", (unsigned int)j);
        else
          ROS_INFO("Benchmark complete for request %u", (unsigned int)j);
      }
    }    
  }
  
  ros::waitForShutdown();
  
  return 0;
}
