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

#include <moveit/warehouse/planning_scene_storage.h>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/variables_map.hpp>

static const std::string PLANNING_SCENE_TOPIC="planning_scene";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_warehouse_scene", ros::init_options::AnonymousName);
  
  boost::program_options::options_description desc;
  desc.add_options()
      ("help", "Show help message")
      ("scene", boost::program_options::value<std::string>(), "Name of scene to publish.");
  
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help") || !vm.count("scene"))
  {
    std::cout << desc << std::endl;
    return 1;
  }
    
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::NodeHandle nh;  
  ros::Publisher pub_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::Publisher pub_req = nh.advertise<moveit_msgs::MotionPlanRequest>("motion_plan_request", 100);
  ros::Publisher pub_res = nh.advertise<moveit_msgs::RobotTrajectory>("motion_plan_results", 100);
  moveit_warehouse::PlanningSceneStorage pss;
  moveit_warehouse::PlanningSceneWithMetadata pswm;
  pss.getPlanningScene(pswm, vm["scene"].as<std::string>());
  ROS_INFO("Publishing scene '%s'", pswm->name.c_str());
  pub_scene.publish(static_cast<const moveit_msgs::PlanningScene&>(*pswm));
  std::vector<moveit_warehouse::MotionPlanRequestWithMetadata> planning_queries;
  std::vector<std::string> query_names;
  pss.getPlanningQueries(planning_queries, query_names, pswm->name);
  ROS_INFO("There are %d planning queries associated to the scene", (int)planning_queries.size());
  ros::WallDuration(0.5).sleep();
  for (std::size_t i = 0 ; i < planning_queries.size() ; ++i)
  {
    ROS_INFO("Publishing query '%s'", query_names[i].c_str());
    pub_req.publish(static_cast<const moveit_msgs::MotionPlanRequest&>(*planning_queries[i]));
    ros::spinOnce();
    std::vector<moveit_warehouse::RobotTrajectoryWithMetadata> planning_results;
    pss.getPlanningResults(planning_results, query_names[i], pswm->name);
    for (std::size_t j = 0 ; j < planning_results.size() ; ++j)
    {
      pub_res.publish(static_cast<const moveit_msgs::RobotTrajectory&>(*planning_results[j]));
      ros::spinOnce();
    }
  }
  ros::WallDuration(1.0).sleep();

  return 0;
}
